// SPDX-License-Identifier: GPL-2.0
/*
 * rpmsg_nsh_tty - Linux tty front-end for a NuttX (openvela) uart_rpmsg
 * console channel, so a Linux user-space program (microcom/cu/agetty) can
 * open an interactive NuttX nsh shell over rpmsg -- i.e. share the single
 * physical UART2 line between Linux and the AMP core (cpu_l3) with no second
 * serial cable.
 *
 * Why not the upstream drivers/tty/rpmsg_tty.c?
 *   The upstream driver binds the "rpmsg-tty" service and exchanges RAW bytes.
 *   NuttX's drivers/serial/uart_rpmsg.c instead wraps every packet in a small
 *   header (command/response/result/cookie + count) and expects a per-write
 *   response for flow control. This driver speaks that exact framing.
 *
 * Wire protocol (must match NuttX drivers/serial/uart_rpmsg.c):
 *   struct urpmsg_header { u32 command; s32 result; u64 cookie; }  // 16 bytes
 *     command bits[30:0] = command id, bit31 = response flag
 *   struct urpmsg_write  { header; u32 count; u32 resolved; u8 data[]; } // 24+
 *   command ids: TTY_WRITE = 0, TTY_WAKEUP = 1
 *
 *   TX (Linux -> NuttX): send TTY_WRITE, response=0, count=len, data=bytes.
 *   RX (NuttX -> Linux): a TTY_WRITE with response=0 carries nsh output; we
 *     push it to the tty, then echo the header back with response=1 and
 *     result = bytes accepted (NuttX uses this to release its TX buffer).
 *   A message with response=1 is the ack to one of OUR writes.
 *   TTY_WAKEUP means the peer freed buffer space (flow control); we rely on
 *   rpmsg_trysend back-pressure instead, so it is a no-op here.
 *
 * This driver exposes /dev/ttyNSHx and binds the "rpmsg-ttyproxy" service, so
 * it coexists with the upstream CONFIG_RPMSG_TTY driver (/dev/ttyRPMSGx,
 * "rpmsg-tty" service) without conflict.
 *
 * Copyright (c) 2026
 */

#define pr_fmt(fmt)		KBUILD_MODNAME ": " fmt

#include <linux/idr.h>
#include <linux/module.h>
#include <linux/rpmsg.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/workqueue.h>

#define RPMSG_NSH_TTY_NAME	"ttyNSH"
#define MAX_TTY_RPMSG		32

/* uart_rpmsg command ids (low 31 bits of header.command) */
#define URPMSG_TTY_WRITE	0u
#define URPMSG_TTY_WAKEUP	1u
/* header.command bit31 = response flag */
#define URPMSG_RESP_FLAG	0x80000000u

struct urpmsg_header {
	__le32	command;	/* bits[30:0] = cmd id, bit31 = response */
	__le32	result;
	__le64	cookie;
} __packed;

struct urpmsg_write {
	struct urpmsg_header	header;
	__le32			count;
	__le32			resolved;
	u8			data[];
} __packed;

static DEFINE_IDR(tty_idr);
static DEFINE_MUTEX(idr_lock);

static struct tty_driver *rpmsg_nsh_tty_driver;

struct rpmsg_nsh_port {
	struct tty_port		port;
	int			id;
	struct rpmsg_device	*rpdev;
	struct work_struct	ack_work;    /* deferred ack sender */
	atomic_t		ack_result;  /* bytes accepted, echoed in ack */
	__le64			ack_cookie;  /* NuttX cookie echoed in ack */
};

/* Deferred ack sender. rpmsg_trysend() must NOT be called from the rpmsg RX
 * callback context (re-entering the rpmsg TX path from within RX corrupted the
 * kernel slab allocator). Instead the RX callback records the result/cookie
 * and schedules this work, which sends the ack from plain process context.
 */

static void rpmsg_nsh_ack_work(struct work_struct *work)
{
	struct rpmsg_nsh_port *cport =
		container_of(work, struct rpmsg_nsh_port, ack_work);
	struct urpmsg_write ack;
	int copied = atomic_read(&cport->ack_result);

	memset(&ack, 0, sizeof(ack));
	ack.header.command = cpu_to_le32(URPMSG_TTY_WRITE | URPMSG_RESP_FLAG);
	ack.header.result  = cpu_to_le32(copied);
	ack.header.cookie  = cport->ack_cookie;
	ack.count          = cpu_to_le32(copied);

	rpmsg_trysend(cport->rpdev->ept, &ack, sizeof(ack));
}

/* NuttX sent us bytes (nsh output) -> push to tty, then ack with response=1 */
static int rpmsg_nsh_handle_write(struct rpmsg_device *rpdev,
				  struct urpmsg_write *msg, int len)
{
	struct rpmsg_nsh_port *cport = dev_get_drvdata(&rpdev->dev);
	u32 count = le32_to_cpu(msg->count);
	int avail, copied = 0;

	if (!cport)
		return -ENXIO;

	/* Explicit int math: len and sizeof mixed would promote to unsigned. */
	avail = len - (int)sizeof(struct urpmsg_write);
	if (avail < 0)
		return -EINVAL;
	if (count > (u32)avail)
		count = (u32)avail;

	if (count) {
		copied = tty_insert_flip_string(&cport->port, msg->data, count);
		if (copied != count)
			dev_warn_ratelimited(&rpdev->dev,
				"tty buffer full: dropped %u bytes\n",
				count - copied);
		tty_flip_buffer_push(&cport->port);
	}

	/* Defer the ack: do NOT call rpmsg_trysend() from this RX callback
	 * context. Re-entering the rpmsg TX path from within RX corrupted the
	 * kernel slab allocator (Oops in __kmem_cache_alloc_node). Record the
	 * result/cookie and let rpmsg_nsh_ack_work() send it from process context.
	 * NuttX issues one write at a time and does not validate the cookie, so a
	 * single pending-ack slot is enough.
	 */
	atomic_set(&cport->ack_result, copied);
	cport->ack_cookie = msg->header.cookie;
	schedule_work(&cport->ack_work);

	return 0;
}

static int rpmsg_nsh_tty_cb(struct rpmsg_device *rpdev, void *data, int len,
			    void *priv, u32 src)
{
	struct urpmsg_write *msg = data;
	u32 command;

	if (len < (int)sizeof(struct urpmsg_header)) {
		dev_err_ratelimited(&rpdev->dev, "short rpmsg: %d\n", len);
		return -EINVAL;
	}

	command = le32_to_cpu(msg->header.command);

	if (command & URPMSG_RESP_FLAG) {
		/* Ack to one of our TX writes; back-pressure handled by trysend. */
		return 0;
	}

	switch (command & ~URPMSG_RESP_FLAG) {
	case URPMSG_TTY_WRITE:
		if (len < (int)sizeof(struct urpmsg_write)) {
			dev_err_ratelimited(&rpdev->dev, "short write msg: %d\n", len);
			return -EINVAL;
		}
		return rpmsg_nsh_handle_write(rpdev, msg, len);
	case URPMSG_TTY_WAKEUP:
		/* Peer freed space; nothing queued on our side. */
		return 0;
	default:
		dev_dbg_ratelimited(&rpdev->dev, "unknown cmd 0x%x\n", command);
		return 0;
	}
}

static int rpmsg_nsh_tty_install(struct tty_driver *driver, struct tty_struct *tty)
{
	struct rpmsg_nsh_port *cport = idr_find(&tty_idr, tty->index);
	struct tty_port *port;

	if (!cport)
		return -ENODEV;

	tty->driver_data = cport;

	/* Take a port reference for this tty; released in rpmsg_nsh_tty_cleanup()
	 * via tty_port_put(). Without this get the kref goes 1->0 on the first
	 * close, freeing cport while the idr and a future open still reference
	 * it -> use-after-free (crash in tty_port_open acquiring port->lock).
	 */
	port = tty_port_get(&cport->port);
	return tty_port_install(port, driver, tty);
}

static void rpmsg_nsh_tty_cleanup(struct tty_struct *tty)
{
	tty_port_put(tty->port);
}

static int rpmsg_nsh_tty_open(struct tty_struct *tty, struct file *filp)
{
	return tty_port_open(tty->port, tty, filp);
}

static void rpmsg_nsh_tty_close(struct tty_struct *tty, struct file *filp)
{
	tty_port_close(tty->port, tty, filp);
}

/* User typed something (into nsh) -> wrap in TTY_WRITE and send to NuttX. */
static int rpmsg_nsh_tty_write(struct tty_struct *tty, const u8 *buf, int len)
{
	struct rpmsg_nsh_port *cport = tty->driver_data;
	struct rpmsg_device *rpdev = cport->rpdev;
	struct urpmsg_write *msg;
	int mtu, room, msg_size, ret;
	u8 txbuf[512];

	if (len <= 0)
		return 0;

	mtu = rpmsg_get_mtu(rpdev->ept);
	if (mtu < 0)
		return mtu;

	room = mtu - sizeof(*msg);
	if (room <= 0)
		return -EMSGSIZE;
	if (room > (int)(sizeof(txbuf) - sizeof(*msg)))
		room = sizeof(txbuf) - sizeof(*msg);
	if (len > room)
		len = room;

	msg = (struct urpmsg_write *)txbuf;
	memset(msg, 0, sizeof(*msg));
	msg->header.command = cpu_to_le32(URPMSG_TTY_WRITE); /* response = 0 */
	msg->header.result  = cpu_to_le32(-ENXIO);
	msg->count          = cpu_to_le32(len);
	memcpy(msg->data, buf, len);

	msg_size = sizeof(*msg) + len;

	/* trysend so the caller is not blocked when no rpmsg buffer is free. */
	ret = rpmsg_trysend(rpdev->ept, msg, msg_size);
	if (ret) {
		dev_dbg_ratelimited(&rpdev->dev, "trysend failed: %d\n", ret);
		return ret;
	}

	return len;
}

static unsigned int rpmsg_nsh_tty_write_room(struct tty_struct *tty)
{
	struct rpmsg_nsh_port *cport = tty->driver_data;
	int mtu;

	mtu = rpmsg_get_mtu(cport->rpdev->ept);
	if (mtu < (int)sizeof(struct urpmsg_write))
		return 0;

	return mtu - sizeof(struct urpmsg_write);
}

static void rpmsg_nsh_tty_hangup(struct tty_struct *tty)
{
	tty_port_hangup(tty->port);
}

static const struct tty_operations rpmsg_nsh_tty_ops = {
	.install	= rpmsg_nsh_tty_install,
	.open		= rpmsg_nsh_tty_open,
	.close		= rpmsg_nsh_tty_close,
	.write		= rpmsg_nsh_tty_write,
	.write_room	= rpmsg_nsh_tty_write_room,
	.hangup		= rpmsg_nsh_tty_hangup,
	.cleanup	= rpmsg_nsh_tty_cleanup,
};

static void rpmsg_nsh_destruct_port(struct tty_port *port)
{
	struct rpmsg_nsh_port *cport =
		container_of(port, struct rpmsg_nsh_port, port);

	mutex_lock(&idr_lock);
	idr_remove(&tty_idr, cport->id);
	mutex_unlock(&idr_lock);

	kfree(cport);
}

static const struct tty_port_operations rpmsg_nsh_port_ops = {
	.destruct = rpmsg_nsh_destruct_port,
};

static int rpmsg_nsh_tty_probe(struct rpmsg_device *rpdev)
{
	struct rpmsg_nsh_port *cport;
	struct device *dev = &rpdev->dev;
	struct device *tty_dev;
	int ret;

	cport = kzalloc(sizeof(*cport), GFP_KERNEL);
	if (!cport)
		return -ENOMEM;

	mutex_lock(&idr_lock);
	ret = idr_alloc(&tty_idr, cport, 0, MAX_TTY_RPMSG, GFP_KERNEL);
	mutex_unlock(&idr_lock);
	if (ret < 0) {
		kfree(cport);
		return ret;
	}
	cport->id = ret;

	tty_port_init(&cport->port);
	cport->port.ops = &rpmsg_nsh_port_ops;
	INIT_WORK(&cport->ack_work, rpmsg_nsh_ack_work);

	tty_dev = tty_port_register_device(&cport->port, rpmsg_nsh_tty_driver,
					   cport->id, dev);
	if (IS_ERR(tty_dev)) {
		ret = dev_err_probe(dev, PTR_ERR(tty_dev),
				    "failed to register tty port\n");
		mutex_lock(&idr_lock);
		idr_remove(&tty_idr, cport->id);
		mutex_unlock(&idr_lock);
		tty_port_put(&cport->port);
		return ret;
	}

	cport->rpdev = rpdev;
	dev_set_drvdata(dev, cport);

	/* Announce our endpoint address back to the NuttX side so it learns
	 * where to send (mirrors rockchip_rpmsg_test).
	 */
	rpdev->announce = rpdev->src != RPMSG_ADDR_ANY;

	dev_info(dev, "new NuttX nsh channel 0x%x -> 0x%x: %s%d\n",
		 rpdev->src, rpdev->dst, RPMSG_NSH_TTY_NAME, cport->id);

	return 0;
}

static void rpmsg_nsh_tty_remove(struct rpmsg_device *rpdev)
{
	struct rpmsg_nsh_port *cport = dev_get_drvdata(&rpdev->dev);

	/* Flush any pending deferred ack before the port (and cport) go away. */
	cancel_work_sync(&cport->ack_work);

	tty_port_tty_hangup(&cport->port, false);
	tty_unregister_device(rpmsg_nsh_tty_driver, cport->id);
	tty_port_put(&cport->port);
}

static struct rpmsg_device_id rpmsg_nsh_tty_id_table[] = {
	{ .name = "rpmsg-ttyproxy" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(rpmsg, rpmsg_nsh_tty_id_table);

static struct rpmsg_driver rpmsg_nsh_tty_rpmsg_drv = {
	.drv.name	= KBUILD_MODNAME,
	.id_table	= rpmsg_nsh_tty_id_table,
	.probe		= rpmsg_nsh_tty_probe,
	.callback	= rpmsg_nsh_tty_cb,
	.remove		= rpmsg_nsh_tty_remove,
};

static int __init rpmsg_nsh_tty_init(void)
{
	int ret;

	rpmsg_nsh_tty_driver = tty_alloc_driver(MAX_TTY_RPMSG,
			TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV);
	if (IS_ERR(rpmsg_nsh_tty_driver))
		return PTR_ERR(rpmsg_nsh_tty_driver);

	rpmsg_nsh_tty_driver->driver_name = "rpmsg_nsh_tty";
	rpmsg_nsh_tty_driver->name = RPMSG_NSH_TTY_NAME;
	rpmsg_nsh_tty_driver->major = 0;
	rpmsg_nsh_tty_driver->type = TTY_DRIVER_TYPE_CONSOLE;
	rpmsg_nsh_tty_driver->init_termios = tty_std_termios;
	/* raw: nsh does its own echo/line-editing */
	rpmsg_nsh_tty_driver->init_termios.c_lflag &= ~(ECHO | ICANON);
	rpmsg_nsh_tty_driver->init_termios.c_oflag &= ~(OPOST | ONLCR);
	tty_set_operations(rpmsg_nsh_tty_driver, &rpmsg_nsh_tty_ops);

	ret = tty_register_driver(rpmsg_nsh_tty_driver);
	if (ret < 0) {
		pr_err("failed to register tty driver: %d\n", ret);
		goto err_put;
	}

	ret = register_rpmsg_driver(&rpmsg_nsh_tty_rpmsg_drv);
	if (ret < 0) {
		pr_err("failed to register rpmsg driver: %d\n", ret);
		goto err_unregister;
	}

	return 0;

err_unregister:
	tty_unregister_driver(rpmsg_nsh_tty_driver);
err_put:
	tty_driver_kref_put(rpmsg_nsh_tty_driver);
	return ret;
}

static void __exit rpmsg_nsh_tty_exit(void)
{
	unregister_rpmsg_driver(&rpmsg_nsh_tty_rpmsg_drv);
	tty_unregister_driver(rpmsg_nsh_tty_driver);
	tty_driver_kref_put(rpmsg_nsh_tty_driver);
	idr_destroy(&tty_idr);
}

module_init(rpmsg_nsh_tty_init);
module_exit(rpmsg_nsh_tty_exit);

MODULE_DESCRIPTION("Linux tty front-end for NuttX uart_rpmsg (nsh over rpmsg)");
MODULE_LICENSE("GPL v2");
