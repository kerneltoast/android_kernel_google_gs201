// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2011 Samsung Electronics.
 *
 */

#include <stdarg.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/ip.h>
#include <net/ip.h>
#include <linux/tcp.h>
#include <linux/udp.h>
#include <linux/rtc.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <soc/google/exynos-modem-ctrl.h>

#include <soc/google/acpm_ipc_ctrl.h>
#include "modem_prj.h"
#include "modem_utils.h"
#include "cpif_version.h"

#define CMD_SUSPEND	((u16)(0x00CA))
#define CMD_RESUME	((u16)(0x00CB))

#define TX_SEPARATOR	"cpif: >>>>>>>>>> Outgoing packet "
#define RX_SEPARATOR	"cpif: Incoming packet <<<<<<<<<<"
#define LINE_SEPARATOR	\
	"cpif: ------------------------------------------------------------"
#define LINE_BUFF_SIZE	80

enum bit_debug_flags {
	DEBUG_FLAG_FMT,
	DEBUG_FLAG_MISC,
	DEBUG_FLAG_RFS,
	DEBUG_FLAG_PS,
	DEBUG_FLAG_BOOT,
	DEBUG_FLAG_DUMP,
	DEBUG_FLAG_CSVT,
	DEBUG_FLAG_LOG,
	DEBUG_FLAG_BT_DUN, /* for rx/tx of umts_router */
	DEBUG_FLAG_ALL
};

#define DEBUG_FLAG_DEFAULT    (1 << DEBUG_FLAG_FMT | 1 << DEBUG_FLAG_MISC)
#ifdef DEBUG_MODEM_IF_PS_DATA
static unsigned long dflags = (DEBUG_FLAG_DEFAULT | 1 << DEBUG_FLAG_RFS | 1 << DEBUG_FLAG_PS);
#else
static unsigned long dflags = (DEBUG_FLAG_DEFAULT);
#endif
module_param(dflags, ulong, 0664);
MODULE_PARM_DESC(dflags, "modem_v1 debug flags");

static unsigned long wakeup_dflags =
		(DEBUG_FLAG_DEFAULT | 1 << DEBUG_FLAG_RFS | 1 << DEBUG_FLAG_PS);
module_param(wakeup_dflags, ulong, 0664);
MODULE_PARM_DESC(wakeup_dflags, "modem_v1 wakeup debug flags");

static const char *hex = "0123456789abcdef";

static struct raw_notifier_head cp_crash_notifier;

struct mif_buff_mng *g_mif_buff_mng;

static inline void ts642utc(struct timespec64 *ts, struct utc_time *utc)
{
	struct tm tm;

	time64_to_tm((ts->tv_sec - (sys_tz.tz_minuteswest * 60)), 0, &tm);
	utc->year = 1900 + (u32)tm.tm_year;
	utc->mon = 1 + tm.tm_mon;
	utc->day = tm.tm_mday;
	utc->hour = tm.tm_hour;
	utc->min = tm.tm_min;
	utc->sec = tm.tm_sec;
	utc->us = (u32)ns2us(ts->tv_nsec);
}

void get_utc_time(struct utc_time *utc)
{
	struct timespec64 ts;

	ktime_get_ts64(&ts);
	ts642utc(&ts, utc);
}

int mif_dump_log(struct modem_shared *msd, struct io_device *iod)
{
	unsigned long read_len = 0;
	unsigned long flags;

	spin_lock_irqsave(&msd->lock, flags);
	while (read_len < MAX_MIF_BUFF_SIZE) {
		struct sk_buff *skb;

		skb = alloc_skb(MAX_IPC_SKB_SIZE, GFP_ATOMIC);
		if (!skb) {
			mif_err("ERR! alloc_skb fail\n");
			spin_unlock_irqrestore(&msd->lock, flags);
			return -ENOMEM;
		}
		memcpy(skb_put(skb, MAX_IPC_SKB_SIZE),
			msd->storage.addr + read_len, MAX_IPC_SKB_SIZE);
		skb_queue_tail(&iod->sk_rx_q, skb);
		read_len += MAX_IPC_SKB_SIZE;
		wake_up(&iod->wq);
	}
	spin_unlock_irqrestore(&msd->lock, flags);
	return 0;
}

static unsigned long long get_kernel_time(void)
{
	int this_cpu;
	unsigned long flags;
	unsigned long long time;

	preempt_disable();
	raw_local_irq_save(flags);

	this_cpu = smp_processor_id();
	time = cpu_clock(this_cpu);

	preempt_enable();
	raw_local_irq_restore(flags);

	return time;
}

void mif_ipc_log(enum mif_log_id id,
	struct modem_shared *msd, const char *data, size_t len)
{
	struct mif_ipc_block *block;
	unsigned long flags;

	spin_lock_irqsave(&msd->lock, flags);

	block = (struct mif_ipc_block *)
		(msd->storage.addr + (MAX_LOG_SIZE * msd->storage.cnt));
	msd->storage.cnt = ((msd->storage.cnt + 1) < MAX_LOG_CNT) ?
		msd->storage.cnt + 1 : 0;

	spin_unlock_irqrestore(&msd->lock, flags);

	block->id = id;
	block->time = get_kernel_time();
	block->len = (len > MAX_IPC_LOG_SIZE) ? MAX_IPC_LOG_SIZE : len;
	memcpy(block->buff, data, block->len);
}

void _mif_irq_log(enum mif_log_id id, struct modem_shared *msd,
	struct mif_irq_map map, const char *data, size_t len)
{
	struct mif_irq_block *block;
	unsigned long flags;

	spin_lock_irqsave(&msd->lock, flags);

	block = (struct mif_irq_block *)
		(msd->storage.addr + (MAX_LOG_SIZE * msd->storage.cnt));
	msd->storage.cnt = ((msd->storage.cnt + 1) < MAX_LOG_CNT) ?
		msd->storage.cnt + 1 : 0;

	spin_unlock_irqrestore(&msd->lock, flags);

	block->id = id;
	block->time = get_kernel_time();
	memcpy(&(block->map), &map, sizeof(struct mif_irq_map));
	if (data)
		memcpy(block->buff, data,
			(len > MAX_IRQ_LOG_SIZE) ? MAX_IRQ_LOG_SIZE : len);
}

void _mif_com_log(enum mif_log_id id,
	struct modem_shared *msd, const char *format, ...)
{
	struct mif_common_block *block;
	unsigned long flags;
	va_list args;

	spin_lock_irqsave(&msd->lock, flags);

	block = (struct mif_common_block *)
		(msd->storage.addr + (MAX_LOG_SIZE * msd->storage.cnt));
	msd->storage.cnt = ((msd->storage.cnt + 1) < MAX_LOG_CNT) ?
		msd->storage.cnt + 1 : 0;

	spin_unlock_irqrestore(&msd->lock, flags);

	block->id = id;
	block->time = get_kernel_time();

	va_start(args, format);
	vsnprintf(block->buff, MAX_COM_LOG_SIZE, format, args);
	va_end(args);
}

void _mif_time_log(enum mif_log_id id, struct modem_shared *msd,
	struct timespec64 epoch, const char *data, size_t len)
{
	struct mif_time_block *block;
	unsigned long flags;

	spin_lock_irqsave(&msd->lock, flags);

	block = (struct mif_time_block *)
		(msd->storage.addr + (MAX_LOG_SIZE * msd->storage.cnt));
	msd->storage.cnt = ((msd->storage.cnt + 1) < MAX_LOG_CNT) ?
		msd->storage.cnt + 1 : 0;

	spin_unlock_irqrestore(&msd->lock, flags);

	block->id = id;
	block->time = get_kernel_time();
	block->epoch = epoch;

	if (data)
		memcpy(block->buff, data,
			(len > MAX_IRQ_LOG_SIZE) ? MAX_IRQ_LOG_SIZE : len);
}

/* dump2hex
 * dump data to hex as fast as possible.
 * the length of @buff must be greater than "@len * 3"
 * it need 3 bytes per one data byte to print.
 */
static inline void dump2hex(char *buff, size_t buff_size,
			    const char *data, size_t data_len)
{
	char *dest = buff;
	size_t len;
	size_t i;

	if (buff_size < (data_len * 3))
		len = buff_size / 3;
	else
		len = data_len;

	for (i = 0; i < len; i++) {
		*dest++ = hex[(data[i] >> 4) & 0xf];
		*dest++ = hex[data[i] & 0xf];
		*dest++ = ' ';
	}

	/* The last character must be overwritten with NULL */
	if (likely(len > 0))
		dest--;

	*dest = 0;
}

static bool wakeup_log_enable;
inline void set_wakeup_packet_log(bool enable)
{
	wakeup_log_enable = enable;
}

inline unsigned long get_log_flags(void)
{
	return wakeup_log_enable ? wakeup_dflags : dflags;
}

void set_dflags(unsigned long flag)
{
	dflags = flag;
}

static inline bool log_enabled(u8 ch, struct link_device *ld)
{
	unsigned long flags = get_log_flags();

	if (ld->is_fmt_ch && ld->is_fmt_ch(ch))
		return test_bit(DEBUG_FLAG_FMT, &flags);
	else if (ld->is_boot_ch && ld->is_boot_ch(ch))
		return test_bit(DEBUG_FLAG_BOOT, &flags);
	else if (ld->is_dump_ch && ld->is_dump_ch(ch))
		return test_bit(DEBUG_FLAG_DUMP, &flags);
	else if (ld->is_rfs_ch && ld->is_rfs_ch(ch))
		return test_bit(DEBUG_FLAG_RFS, &flags);
	else if (ld->is_csd_ch && ld->is_csd_ch(ch))
		return test_bit(DEBUG_FLAG_CSVT, &flags);
	else if (ld->is_log_ch && ld->is_log_ch(ch))
		return test_bit(DEBUG_FLAG_LOG, &flags);
	else if (ld->is_ps_ch && ld->is_ps_ch(ch))
		return test_bit(DEBUG_FLAG_PS, &flags);
	else if (ld->is_router_ch && ld->is_router_ch(ch))
		return test_bit(DEBUG_FLAG_BT_DUN, &flags);
	else if (ld->is_misc_ch && ld->is_misc_ch(ch))
		return test_bit(DEBUG_FLAG_MISC, &flags);
	else
		return test_bit(DEBUG_FLAG_ALL, &flags);
}

/* print ipc packet */
void mif_pkt(u8 ch, const char *tag, struct sk_buff *skb)
{
	if (!skbpriv(skb)->ld)
		return;

	if (!log_enabled(ch, skbpriv(skb)->ld))
		return;

	if (unlikely(!skb)) {
		mif_err("ERR! NO skb!!!\n");
		return;
	}

	pr_skb(tag, skb, skbpriv(skb)->ld);
}

/* print buffer as hex string */
#define PR_BUFFER_SIZE 128
int pr_buffer(const char *tag, const char *data, size_t data_len,
							size_t max_len)
{
	size_t len = min(data_len, max_len);
	unsigned char str[PR_BUFFER_SIZE * 3]; /* 1 <= sizeof <= max_len*3 */

	if (len > PR_BUFFER_SIZE)
		len = PR_BUFFER_SIZE;

	dump2hex(str, (len ? len * 3 : 1), data, len);

	/* don't change this printk to mif_debug for print this as level7 */
	return pr_info("%s: %s(%ld): %s%s\n", MIF_TAG, tag, (long)data_len,
			str, (len == data_len) ? "" : " ...");
}

/* flow control CM from CP, it use in serial devices */
int link_rx_flowctl_cmd(struct link_device *ld, const char *data, size_t len)
{
	struct modem_shared *msd = ld->msd;
	unsigned short *cmd, *end = (unsigned short *)(data + len);

	mif_debug("flow control cmd: size=%ld\n", (long)len);

	for (cmd = (unsigned short *)data; cmd < end; cmd++) {
		switch (*cmd) {
		case CMD_SUSPEND:
			iodevs_for_each(msd, iodev_netif_stop, 0);
			mif_info("flowctl CMD_SUSPEND(%04X)\n", *cmd);
			break;

		case CMD_RESUME:
			iodevs_for_each(msd, iodev_netif_wake, 0);
			mif_info("flowctl CMD_RESUME(%04X)\n", *cmd);
			break;

		default:
			mif_err("flowctl BAD CMD: %04X\n", *cmd);
			break;
		}
	}

	return 0;
}

struct io_device *get_iod_with_format(struct modem_shared *msd,
			u32 format)
{
	struct rb_node *n = msd->iodevs_tree_fmt.rb_node;

	while (n) {
		struct io_device *iodev;

		iodev = rb_entry(n, struct io_device, node_fmt);
		if (format < iodev->format)
			n = n->rb_left;
		else if (format > iodev->format)
			n = n->rb_right;
		else
			return iodev;
	}

	return NULL;
}

void insert_iod_with_channel(struct modem_shared *msd, unsigned int channel,
			     struct io_device *iod)
{
	unsigned int idx = msd->num_channels;

	msd->ch2iod[channel] = iod;
	msd->ch[idx] = channel;
	msd->num_channels++;
}

struct io_device *insert_iod_with_format(struct modem_shared *msd,
		u32 format, struct io_device *iod)
{
	struct rb_node **p = &msd->iodevs_tree_fmt.rb_node;
	struct rb_node *parent = NULL;

	while (*p) {
		struct io_device *iodev;

		parent = *p;
		iodev = rb_entry(parent, struct io_device, node_fmt);
		if (format < iodev->format)
			p = &(*p)->rb_left;
		else if (format > iodev->format)
			p = &(*p)->rb_right;
		else
			return iodev;
	}

	rb_link_node(&iod->node_fmt, parent, p);
	rb_insert_color(&iod->node_fmt, &msd->iodevs_tree_fmt);
	return NULL;
}

void iodev_netif_wake(struct io_device *iod, void *args)
{
	if (iod->io_typ == IODEV_NET && iod->ndev) {
		netif_wake_queue(iod->ndev);
		mif_info("%s\n", iod->name);
	}
}

void iodev_netif_stop(struct io_device *iod, void *args)
{
	if (iod->io_typ == IODEV_NET && iod->ndev) {
		netif_stop_queue(iod->ndev);
		mif_info("%s\n", iod->name);
	}
}

void netif_tx_flowctl(struct modem_shared *msd, bool tx_stop)
{
	struct io_device *iod;

	spin_lock(&msd->active_list_lock);
	list_for_each_entry(iod, &msd->activated_ndev_list, node_ndev) {
		if (tx_stop) {
			netif_stop_subqueue(iod->ndev, 0);
#ifdef DEBUG_MODEM_IF_FLOW_CTRL
			mif_err("tx_stop:%s, iod->ndev->name:%s\n",
				tx_stop ? "suspend" : "resume",
				iod->ndev->name);
#endif
		} else {
			netif_wake_subqueue(iod->ndev, 0);
#ifdef DEBUG_MODEM_IF_FLOW_CTRL
			mif_err("tx_stop:%s, iod->ndev->name:%s\n",
				tx_stop ? "suspend" : "resume",
				iod->ndev->name);
#endif
		}
	}
	spin_unlock(&msd->active_list_lock);
}

void stop_net_iface(struct link_device *ld, unsigned int channel)
{
	struct io_device *iod;
	unsigned long flags;

	spin_lock_irqsave(&ld->netif_lock, flags);

	if (test_bit(channel, &ld->netif_stop_mask)) {
		mif_err("channel %d was already stopped!\n", channel);
		goto exit;
	}

	iod = link_get_iod_with_channel(ld, channel);
	iodev_netif_stop(iod, 0);
	set_bit(channel, &ld->netif_stop_mask);

exit:
	spin_unlock_irqrestore(&ld->netif_lock, flags);
}

void stop_net_ifaces(struct link_device *ld)
{
	unsigned long flags;

	spin_lock_irqsave(&ld->netif_lock, flags);

	if (!atomic_read(&ld->netif_stopped)) {
		if (ld->msd)
			netif_tx_flowctl(ld->msd, true);

		atomic_set(&ld->netif_stopped, 1);
	}

	spin_unlock_irqrestore(&ld->netif_lock, flags);
}

void resume_net_iface(struct link_device *ld, unsigned int channel)
{
	struct io_device *iod;
	unsigned long flags;

	spin_lock_irqsave(&ld->netif_lock, flags);

	if (!test_bit(channel, &ld->netif_stop_mask)) {
		mif_err("channel %d was already resumed!\n", channel);
		goto exit;
	}

	iod = link_get_iod_with_channel(ld, channel);
	iodev_netif_wake(iod, 0);
	clear_bit(channel, &ld->netif_stop_mask);

exit:
	spin_unlock_irqrestore(&ld->netif_lock, flags);
}

void resume_net_ifaces(struct link_device *ld)
{
	unsigned long flags;

	spin_lock_irqsave(&ld->netif_lock, flags);

	if (atomic_read(&ld->netif_stopped) != 0) {
		if (ld->msd)
			netif_tx_flowctl(ld->msd, false);

		atomic_set(&ld->netif_stopped, 0);
	}

	spin_unlock_irqrestore(&ld->netif_lock, flags);
}

/*
 * @brief		ipv4 string to be32 (big endian 32bits integer)
 * @return		zero when errors occurred
 */
__be32 ipv4str_to_be32(const char *ipv4str, size_t count)
{
	unsigned char ip[4];
	char ipstr[16]; /* == strlen("xxx.xxx.xxx.xxx") + 1 */
	char *next = ipstr;
	int i;

	strlcpy(ipstr, ipv4str, ARRAY_SIZE(ipstr));

	for (i = 0; i < 4; i++) {
		char *p;

		p = strsep(&next, ".");
		if (p && kstrtou8(p, 10, &ip[i]) < 0)
			return 0; /* == 0.0.0.0 */
	}

	return *((__be32 *)ip);
}

void mif_add_timer(struct timer_list *timer, unsigned long expire,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 15, 0))
			void (*function)(struct timer_list *))
#else
			void (*function)(unsigned long), unsigned long data)
#endif
{
	if (timer_pending(timer))
		return;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 15, 0))
	timer_setup(timer, function, 0);
	timer->expires = get_jiffies_64() + expire;
#else
	init_timer(timer);
	timer->expires = jiffies + expire;
	timer->function = function;
	timer->data = data;
#endif

	add_timer(timer);
}

void mif_print_data(const u8 *data, int len)
{
	int words = len >> 4;
	int residue = len - (words << 4);
	int i;
	char *b;
	char last[80];

	/* Make the last line, if ((len % 16) > 0) */
	if (residue > 0) {
		char tb[8];

		sprintf(last, "%04X: ", (words << 4));
		b = (char *)data + (words << 4);

		for (i = 0; i < residue; i++) {
			sprintf(tb, "%02x ", b[i]);
			strcat(last, tb);
			if ((i & 0x3) == 0x3) {
				sprintf(tb, " ");
				strcat(last, tb);
			}
		}
	}

	for (i = 0; i < words; i++) {
		b = (char *)data + (i << 4);
		mif_err("%04X: "
			"%02x %02x %02x %02x  %02x %02x %02x %02x  "
			"%02x %02x %02x %02x  %02x %02x %02x %02x\n",
			(i << 4),
			b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7],
			b[8], b[9], b[10], b[11], b[12], b[13], b[14], b[15]);
	}

	/* Print the last line */
	if (residue > 0)
		mif_err("%s\n", last);
}

void mif_dump2format16(const u8 *data, int len, char *buff, char *tag)
{
	char *d;
	int i;
	int words = len >> 4;
	int residue = len - (words << 4);
	char line[LINE_BUFF_SIZE];

	for (i = 0; i < words; i++) {
		memset(line, 0, LINE_BUFF_SIZE);
		d = (char *)data + (i << 4);

		if (tag)
			sprintf(line, "%s%04X| "
				"%02x %02x %02x %02x  "
				"%02x %02x %02x %02x  "
				"%02x %02x %02x %02x  "
				"%02x %02x %02x %02x\n",
				tag, (i << 4),
				d[0], d[1], d[2], d[3],
				d[4], d[5], d[6], d[7],
				d[8], d[9], d[10], d[11],
				d[12], d[13], d[14], d[15]);
		else
			sprintf(line, "%04X| "
				"%02x %02x %02x %02x  "
				"%02x %02x %02x %02x  "
				"%02x %02x %02x %02x  "
				"%02x %02x %02x %02x\n",
				(i << 4),
				d[0], d[1], d[2], d[3],
				d[4], d[5], d[6], d[7],
				d[8], d[9], d[10], d[11],
				d[12], d[13], d[14], d[15]);

		strcat(buff, line);
	}

	/* Make the last line, if (len % 16) > 0 */
	if (residue > 0) {
		char tb[8];

		memset(line, 0, LINE_BUFF_SIZE);
		memset(tb, 0, sizeof(tb));
		d = (char *)data + (words << 4);

		if (tag)
			sprintf(line, "%s%04X|", tag, (words << 4));
		else
			sprintf(line, "%04X|", (words << 4));

		for (i = 0; i < residue; i++) {
			sprintf(tb, " %02x", d[i]);
			strcat(line, tb);
			if ((i & 0x3) == 0x3) {
				sprintf(tb, " ");
				strcat(line, tb);
			}
		}
		strcat(line, "\n");

		strcat(buff, line);
	}
}

void mif_dump2format4(const u8 *data, int len, char *buff, char *tag)
{
	char *d;
	int i;
	int words = len >> 2;
	int residue = len - (words << 2);
	char line[LINE_BUFF_SIZE];

	for (i = 0; i < words; i++) {
		memset(line, 0, LINE_BUFF_SIZE);
		d = (char *)data + (i << 2);

		if (tag)
			sprintf(line, "%s%04X| %02x %02x %02x %02x\n",
				tag, (i << 2), d[0], d[1], d[2], d[3]);
		else
			sprintf(line, "%04X| %02x %02x %02x %02x\n",
				(i << 2), d[0], d[1], d[2], d[3]);

		strcat(buff, line);
	}

	/* Make the last line, if (len % 4) > 0 */
	if (residue > 0) {
		char tb[8];

		memset(line, 0, LINE_BUFF_SIZE);
		memset(tb, 0, sizeof(tb));
		d = (char *)data + (words << 2);

		if (tag)
			sprintf(line, "%s%04X|", tag, (words << 2));
		else
			sprintf(line, "%04X|", (words << 2));

		for (i = 0; i < residue; i++) {
			sprintf(tb, " %02x", d[i]);
			strcat(line, tb);
		}
		strcat(line, "\n");

		strcat(buff, line);
	}
}

void mif_print_dump(const u8 *data, int len, int width)
{
	char *buff;

	buff = kzalloc(len << 3, GFP_ATOMIC);
	if (!buff) {
		mif_err("ERR! kzalloc fail\n");
		return;
	}

	if (width == 16)
		mif_dump2format16(data, len, buff, LOG_TAG);
	else
		mif_dump2format4(data, len, buff, LOG_TAG);

	pr_info("%s", buff);

	kfree(buff);
}

static void strcat_tcp_header(char *buff, u8 *pkt)
{
	struct tcphdr *tcph = (struct tcphdr *)pkt;
	int eol;
	char line[LINE_BUFF_SIZE] = {0, };
	char flag_str[32] = {0, };

/*
 * -------------------------------------------------------------------------

				TCP Header Format

	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|          Source Port          |       Destination Port        |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|                        Sequence Number                        |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|                    Acknowledgment Number                      |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|  Data |       |C|E|U|A|P|R|S|F|                               |
	| Offset| Rsvd  |W|C|R|C|S|S|Y|I|            Window             |
	|       |       |R|E|G|K|H|T|N|N|                               |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|           Checksum            |         Urgent Pointer        |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|                    Options                    |    Padding    |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|                             data                              |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

-------------------------------------------------------------------------
*/

	snprintf(line, LINE_BUFF_SIZE,
		"%s: TCP:: Src.Port %u, Dst.Port %u\n",
		MIF_TAG, ntohs(tcph->source), ntohs(tcph->dest));
	strcat(buff, line);

	snprintf(line, LINE_BUFF_SIZE,
		"%s: TCP:: SEQ 0x%08X(%u), ACK 0x%08X(%u)\n",
		MIF_TAG, ntohs(tcph->seq), ntohs(tcph->seq),
		ntohs(tcph->ack_seq), ntohs(tcph->ack_seq));
	strcat(buff, line);

	if (tcph->cwr)
		strcat(flag_str, "CWR ");
	if (tcph->ece)
		strcat(flag_str, "ECE");
	if (tcph->urg)
		strcat(flag_str, "URG ");
	if (tcph->ack)
		strcat(flag_str, "ACK ");
	if (tcph->psh)
		strcat(flag_str, "PSH ");
	if (tcph->rst)
		strcat(flag_str, "RST ");
	if (tcph->syn)
		strcat(flag_str, "SYN ");
	if (tcph->fin)
		strcat(flag_str, "FIN ");
	eol = strlen(flag_str) - 1;
	if (eol > 0)
		flag_str[eol] = 0;
	snprintf(line, LINE_BUFF_SIZE, "%s: TCP:: Flags {%s}\n",
		MIF_TAG, flag_str);
	strcat(buff, line);

	snprintf(line, LINE_BUFF_SIZE,
		"%s: TCP:: Window %u, Checksum 0x%04X, Urgent %u\n", MIF_TAG,
		ntohs(tcph->window), ntohs(tcph->check), ntohs(tcph->urg_ptr));
	strcat(buff, line);
}

static void strcat_udp_header(char *buff, u8 *pkt)
{
	struct udphdr *udph = (struct udphdr *)pkt;
	char line[LINE_BUFF_SIZE] = {0, };

/*
 * -------------------------------------------------------------------------

				UDP Header Format

	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|          Source Port          |       Destination Port        |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|            Length             |           Checksum            |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|                             data                              |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

-------------------------------------------------------------------------
*/

	snprintf(line, LINE_BUFF_SIZE,
		"%s: UDP:: Src.Port %u, Dst.Port %u\n",
		MIF_TAG, ntohs(udph->source), ntohs(udph->dest));
	strcat(buff, line);

	snprintf(line, LINE_BUFF_SIZE,
		"%s: UDP:: Length %u, Checksum 0x%04X\n",
		MIF_TAG, ntohs(udph->len), ntohs(udph->check));
	strcat(buff, line);

	if (ntohs(udph->dest) == 53) {
		snprintf(line, LINE_BUFF_SIZE, "%s: UDP:: DNS query!!!\n",
			MIF_TAG);
		strcat(buff, line);
	}

	if (ntohs(udph->source) == 53) {
		snprintf(line, LINE_BUFF_SIZE, "%s: UDP:: DNS response!!!\n",
			MIF_TAG);
		strcat(buff, line);
	}
}

void print_ipv4_packet(const u8 *ip_pkt, enum direction dir)
{
	char *buff;
	struct iphdr *iph = (struct iphdr *)ip_pkt;
	char *pkt = (char *)ip_pkt + (iph->ihl << 2);
	u16 flags = (ntohs(iph->frag_off) & 0xE000);
	u16 frag_off = (ntohs(iph->frag_off) & 0x1FFF);
	int eol;
	char line[LINE_BUFF_SIZE] = {0, };
	char flag_str[16] = {0, };

/*
 * ---------------------------------------------------------------------------
				IPv4 Header Format

	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|Version|  IHL  |Type of Service|          Total Length         |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|         Identification        |C|D|M|     Fragment Offset     |
	|                               |E|F|F|                         |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|  Time to Live |    Protocol   |         Header Checksum       |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|                       Source Address                          |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|                    Destination Address                        |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	|                    Options                    |    Padding    |
	+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

	IHL - Header Length
	Flags - Consist of 3 bits
		The 1st bit is "Congestion" bit.
		The 2nd bit is "Dont Fragment" bit.
		The 3rd bit is "More Fragments" bit.

---------------------------------------------------------------------------
*/

	if (iph->version != 4)
		return;

	buff = kzalloc(4096, GFP_ATOMIC);
	if (!buff)
		return;

	if (dir == TX)
		snprintf(line, LINE_BUFF_SIZE, "%s\n", TX_SEPARATOR);
	else
		snprintf(line, LINE_BUFF_SIZE, "%s\n", RX_SEPARATOR);
	strcat(buff, line);

	snprintf(line, LINE_BUFF_SIZE, "%s\n", LINE_SEPARATOR);
	strcat(buff, line);

	snprintf(line, LINE_BUFF_SIZE,
		"%s: IP4:: Version %u, Header Length %u, TOS %u, Length %u\n",
		MIF_TAG, iph->version, (iph->ihl << 2), iph->tos,
		ntohs(iph->tot_len));
	strcat(buff, line);

	snprintf(line, LINE_BUFF_SIZE, "%s: IP4:: ID %u, Fragment Offset %u\n",
		MIF_TAG, ntohs(iph->id), frag_off);
	strcat(buff, line);

	if (flags & IP_CE)
		strcat(flag_str, "CE ");
	if (flags & IP_DF)
		strcat(flag_str, "DF ");
	if (flags & IP_MF)
		strcat(flag_str, "MF ");
	eol = strlen(flag_str) - 1;
	if (eol > 0)
		flag_str[eol] = 0;
	snprintf(line, LINE_BUFF_SIZE, "%s: IP4:: Flags {%s}\n",
		MIF_TAG, flag_str);
	strcat(buff, line);

	snprintf(line, LINE_BUFF_SIZE,
		"%s: IP4:: TTL %u, Protocol %u, Header Checksum 0x%04X\n",
		MIF_TAG, iph->ttl, iph->protocol, ntohs(iph->check));
	strcat(buff, line);

	snprintf(line, LINE_BUFF_SIZE,
		"%s: IP4:: Src.IP %u.%u.%u.%u, Dst.IP %u.%u.%u.%u\n",
		MIF_TAG, ip_pkt[12], ip_pkt[13], ip_pkt[14], ip_pkt[15],
		ip_pkt[16], ip_pkt[17], ip_pkt[18], ip_pkt[19]);
	strcat(buff, line);

	switch (iph->protocol) {
	case 6: /* TCP */
		strcat_tcp_header(buff, pkt);
		break;

	case 17: /* UDP */
		strcat_udp_header(buff, pkt);
		break;

	default:
		break;
	}

	snprintf(line, LINE_BUFF_SIZE, "%s\n", LINE_SEPARATOR);
	strcat(buff, line);

	pr_err("%s\n", buff);

	kfree(buff);
}

bool is_dns_packet(const u8 *ip_pkt)
{
	struct iphdr *iph = (struct iphdr *)ip_pkt;
	struct udphdr *udph = (struct udphdr *)(ip_pkt + (iph->ihl << 2));

	/* If this packet is not a UDP packet, return here. */
	if (iph->protocol != 17)
		return false;

	if (ntohs(udph->dest) == 53 || ntohs(udph->source) == 53)
		return true;
	else
		return false;
}

bool is_syn_packet(const u8 *ip_pkt)
{
	struct iphdr *iph = (struct iphdr *)ip_pkt;
	struct tcphdr *tcph = (struct tcphdr *)(ip_pkt + (iph->ihl << 2));

	/* If this packet is not a TCP packet, return here. */
	if (iph->protocol != 6)
		return false;

	if (tcph->syn || tcph->fin)
		return true;
	else
		return false;
}

void mif_init_irq(struct modem_irq *irq, unsigned int num, const char *name,
		  unsigned long flags)
{
	spin_lock_init(&irq->lock);
	irq->num = num;
	strncpy(irq->name, name, (MAX_NAME_LEN - 1));
	irq->flags = flags;
	mif_info("name:%s num:%d flags:0x%08lX\n", name, num, flags);
}

int mif_request_irq(struct modem_irq *irq, irq_handler_t isr, void *data)
{
	int ret;

	ret = request_irq(irq->num, isr, irq->flags, irq->name, data);
	if (ret) {
		mif_err("%s: ERR! request_irq fail (%d)\n", irq->name, ret);
		return ret;
	}

	enable_irq_wake(irq->num);
	irq->active = true;
	irq->registered = true;

	mif_info("%s(#%d) handler registered (flags:0x%08lX)\n",
		irq->name, irq->num, irq->flags);

	return 0;
}

void mif_enable_irq(struct modem_irq *irq)
{
	unsigned long flags;

	if (irq->registered == false)
		return;

	spin_lock_irqsave(&irq->lock, flags);

	if (irq->active) {
		mif_err("%s(#%d) is already active <%ps>\n", irq->name, irq->num, CALLER);
		goto exit;
	}

	enable_irq(irq->num);
	/*
	 * The pad assignment of CP2AP_ACTIVE is not in PAD_ALIVE to be registered wake-up source.
	 * (Bug 152900487)
	 * This error can affect the crash dump process.
	 * CP2AP_ACTIVE is assigned to XEINT_17 on planned form factor designs.
	 */
	if (!irq->not_alive)
		enable_irq_wake(irq->num);

	irq->active = true;

	mif_debug("%s(#%d) is enabled <%ps>\n", irq->name, irq->num, CALLER);

exit:
	spin_unlock_irqrestore(&irq->lock, flags);
}

void mif_disable_irq(struct modem_irq *irq)
{
	unsigned long flags;

	if (irq->registered == false)
		return;

	spin_lock_irqsave(&irq->lock, flags);

	if (!irq->active) {
		mif_info("%s(#%d) is not active <%ps>\n", irq->name, irq->num, CALLER);
		goto exit;
	}

	disable_irq_nosync(irq->num);
	/*
	 * The pad assignment of CP2AP_ACTIVE is not in PAD_ALIVE to be registered wake-up source.
	 * (Bug 152900487)
	 * This error can affect the crash dump process.
	 * CP2AP_ACTIVE is assigned to XEINT_17 on planned form factor designs.
	 */
	if (!irq->not_alive)
		disable_irq_wake(irq->num);

	irq->active = false;

	mif_debug("%s(#%d) is disabled <%ps>\n", irq->name, irq->num, CALLER);

exit:
	spin_unlock_irqrestore(&irq->lock, flags);
}

bool mif_gpio_set_value(struct cpif_gpio *gpio, int value, unsigned int delay_ms)
{
	int dup = 0;

	if (!gpio_is_valid(gpio->num)) {
		mif_err("SET GPIO %d is failed\n", gpio->num);
		return false;
	}

	if (gpio_get_value(gpio->num) == value)
		dup = 1;

	/* set gpio even if it is set already */
	gpio_set_value(gpio->num, value);

	mif_debug("SET GPIO %s = %d (wait %dms, dup %d)\n", gpio->label, value, delay_ms, dup);

	if (delay_ms > 0 && !dup) {
		if (in_interrupt() || irqs_disabled())
			mdelay(delay_ms);
		else if (delay_ms < 20)
			usleep_range(delay_ms * 1000, (delay_ms * 1000) + 5000);
		else
			msleep(delay_ms);
	}

	return (!dup);
}

int mif_gpio_get_value(struct cpif_gpio *gpio, bool log_print)
{
	int value;

	if (!gpio_is_valid(gpio->num)) {
		mif_err("GET GPIO %d is failed\n", gpio->num);
		return -EINVAL;
	}

	value = gpio_get_value(gpio->num);

	if (log_print)
		mif_debug("GET GPIO %s = %d\n", gpio->label, value);

	return value;
}

int mif_gpio_toggle_value(struct cpif_gpio *gpio, int delay_ms)
{
	int value;

	value = mif_gpio_get_value(gpio, false);
	mif_gpio_set_value(gpio, !value, delay_ms);
	mif_gpio_set_value(gpio, value, 0);

	return value;
}

int board_gpio_export(struct device *dev,
		unsigned int gpio, bool dir, const char *name)
{
	int ret = 0;

	if (!gpio_is_valid(gpio)) {
		mif_err("invalid gpio pins - %s\n", name);
		return -EINVAL;
	}

	ret = gpio_export(gpio, dir);
	if (ret) {
		mif_err("%s: failed to export gpio (%d)\n", name, ret);
		return ret;
	}

	ret = gpio_export_link(dev, name, gpio);
	if (ret) {
		mif_err("%s: failed to export link_gpio (%d)\n", name, ret);
		return ret;
	}

	mif_info("%s exported\n", name);

	return 0;
}

void make_gpio_floating(unsigned int gpio, bool floating)
{
	if (floating)
		gpio_direction_input(gpio);
	else
		gpio_direction_output(gpio, 0);
}

int __ref register_cp_crash_notifier(struct notifier_block *nb)
{
	return raw_notifier_chain_register(&cp_crash_notifier, nb);
}

void __ref modemctl_notify_event(enum modemctl_event evt)
{
	raw_notifier_call_chain(&cp_crash_notifier, evt, NULL);
}

void mif_stop_logging(void)
{
}

static LIST_HEAD(bm_list);
struct mif_buff_mng *init_mif_buff_mng(unsigned char *buffer_start,
	unsigned int buffer_size, unsigned int cell_size)
{
	struct mif_buff_mng *bm;
	unsigned long flags;

	if (buffer_start == NULL || buffer_size == 0 || cell_size == 0) {
		mif_err("parameter ERR!\n");
		return NULL;
	}

	mif_info("buffer:%pK, size:%u, cell_size:%u\n",
		buffer_start, buffer_size, cell_size);

	bm = kzalloc(sizeof(struct mif_buff_mng), GFP_KERNEL);
	if (bm == NULL)
		return NULL;

	bm->buffer_start = buffer_start;
	bm->buffer_end = buffer_start + buffer_size;
	bm->buffer_size = buffer_size;
	bm->cell_size = cell_size;
	bm->cell_count = buffer_size / cell_size;
	bm->free_cell_count = bm->cell_count;
	bm->used_cell_count = 0;

	bm->current_map_index = 0;
	bm->buffer_map_size = (unsigned int)(bm->cell_count /
			MIF_BITS_FOR_MAP_CELL) + 1;
	bm->buffer_map = kzalloc((MIF_BUFF_MAP_CELL_SIZE * bm->buffer_map_size),
		GFP_KERNEL);
	if (bm->buffer_map == NULL) {
		kfree(bm);
		return NULL;
	}

	mif_info("cell_count:%u, map_size:%u, map_size_byte:%lu  buff_map:%pK\n"
		, bm->cell_count, bm->buffer_map_size,
		(sizeof(unsigned int) * bm->buffer_map_size), bm->buffer_map);

#ifdef MIF_BUFF_DEBUG
	mif_info("MIF_BUFF_MAP_CELL_SIZE:%lu\n", MIF_BUFF_MAP_CELL_SIZE);
	mif_info("MIF_BITS_FOR_BYTE:%u\n", MIF_BITS_FOR_BYTE);
	mif_info("MIF_BITS_FOR_MAP_CELL:%lu\n", MIF_BITS_FOR_MAP_CELL);
#endif

	spin_lock_init(&bm->lock);

	spin_lock_irqsave(&bm->lock, flags);
	list_add(&bm->node, &bm_list);
	spin_unlock_irqrestore(&bm->lock, flags);

	return bm;
}

void exit_mif_buff_mng(struct mif_buff_mng *bm)
{
	unsigned long flags;

	if (bm) {
		spin_lock_irqsave(&bm->lock, flags);
		list_del(&bm->node);
		spin_unlock_irqrestore(&bm->lock, flags);

		kfree(bm->buffer_map);
		kfree(bm);
	}
}

void *alloc_mif_buff(struct mif_buff_mng *bm)
{
	unsigned char *buff_allocated;
	int i, j;
	unsigned int location;
	int find_flag = false;
	unsigned long flags;
	uint64_t test_map;
	int last_bit_set;

	if (bm == NULL || bm->buffer_map == NULL)
		return NULL;

	if (bm->free_cell_count == 0) {
#ifdef MIF_BUFF_DEBUG
		mif_info("ERR allocation fail\n");
#endif
		return NULL;
	}

	spin_lock_irqsave(&bm->lock, flags);

	for (i = bm->current_map_index ; i < bm->buffer_map_size; i++) {
		test_map = (uint64_t)bm->buffer_map[i];
		test_map = ~test_map;
		last_bit_set = fls64(test_map);

		if (last_bit_set == 0)
			continue;
		else
			j = MIF_BITS_FOR_MAP_CELL - last_bit_set;

		location = (i * MIF_BITS_FOR_MAP_CELL) + j;

		if (location >= bm->cell_count)
			break;

		find_flag = true;
		bm->buffer_map[i] |= (uint64_t)(MIF_64BIT_FIRST_BIT >> j);
		bm->current_map_index = i;

#ifdef MIF_BUFF_DEBUG
		mif_info("map: %016llx\n", bm->buffer_map[i]);
		mif_info("i:%d j:%d location:%d\n", i, j, location);
#endif
		break;
	}

	if (find_flag == false) {
		for (i = 0 ; i < bm->current_map_index; i++) {
			test_map = (uint64_t)bm->buffer_map[i];
			test_map = ~test_map;
			last_bit_set = fls64(test_map);

			if (last_bit_set == 0)
				continue;
			else
				j = MIF_BITS_FOR_MAP_CELL - last_bit_set;

			location = (i * MIF_BITS_FOR_MAP_CELL) + j;

			if (location >= bm->cell_count)
				break;

			find_flag = true;
			bm->buffer_map[i] |= (uint64_t)(MIF_64BIT_FIRST_BIT >> j);
			bm->current_map_index = i;

#ifdef MIF_BUFF_DEBUG
			mif_info("map: %016llx\n", bm->buffer_map[i]);
			mif_info("i:%d j:%d location:%d\n", i, j, location);
#endif
			break;
		}
	}

	if (find_flag == false) {
#ifdef MIF_BUFF_DEBUG
		mif_info("ERR allocation fail\n");
#endif
		spin_unlock_irqrestore(&bm->lock, flags);
		return NULL;
	}

	buff_allocated = bm->buffer_start;
	buff_allocated += (location * bm->cell_size);

	bm->free_cell_count--;
	bm->used_cell_count++;

	spin_unlock_irqrestore(&bm->lock, flags);

#ifdef MIF_BUFF_DEBUG
	mif_info("location:%d cell_size:%u\n", location, bm->cell_size);
	mif_info("buffer_allocated:%pK\n", buff_allocated);
	mif_info("used/free: %u/%u\n", get_mif_buff_used_count(bm),
			get_mif_buff_free_count(bm));
#endif

	return (void *)buff_allocated;
}

int free_mif_buff(struct mif_buff_mng *bm, void *buffer)
{
	unsigned char *uc_buffer = (unsigned char *)buffer;
	unsigned int addr_diff;
	int i, j, location;
	unsigned long flags;

	if (bm == NULL)
		return -1;

	if (buffer == NULL)
		return 0;

	addr_diff = (unsigned int)(uc_buffer - bm->buffer_start);

	if (addr_diff > bm->buffer_size) {
		mif_err_limited("ERR Buffer:%pK is not my pool one.\n", uc_buffer);
		return -1;
	}

	location = addr_diff / bm->cell_size;
	i = location / MIF_BITS_FOR_MAP_CELL;
	j = location % MIF_BITS_FOR_MAP_CELL;

#ifdef MIF_BUFF_DEBUG
	mif_info("uc_buff:%pK diff:%u\n", uc_buffer, addr_diff);
	mif_info("location:%d i:%d j:%d\n", location, i, j);
#endif

	if ((bm->buffer_map[i] & (MIF_64BIT_FIRST_BIT >> j)) == 0) {
		mif_err_limited("ERR Buffer:%pK is allready freed\n", uc_buffer);
		return -1;
	}

	spin_lock_irqsave(&bm->lock, flags);

	bm->buffer_map[i] &= ~(MIF_64BIT_FIRST_BIT >> j);
	bm->free_cell_count++;
	bm->used_cell_count--;

	spin_unlock_irqrestore(&bm->lock, flags);

#ifdef MIF_BUFF_DEBUG
	mif_info("used/free: %u/%u\n", get_mif_buff_used_count(bm),
			get_mif_buff_free_count(bm));
#endif
	return 0;
}

bool __skb_free_head_cp_zerocopy(struct sk_buff *skb)
{
	struct mif_buff_mng *bm;

	list_for_each_entry(bm, &bm_list, node) {
		if ((bm->buffer_start <= skb->head) && (skb->head < bm->buffer_end)) {
			free_mif_buff(bm, skb->head);
			return true;
		}
	}

	return false;
}
EXPORT_SYMBOL(__skb_free_head_cp_zerocopy);

void cpif_enable_sw_zerocopy(void)
{
	struct mif_buff_mng *bm;

	mif_info("enabled\n");
	list_for_each_entry(bm, &bm_list, node)
		bm->enable_sw_zerocopy = true;
}
EXPORT_SYMBOL(cpif_enable_sw_zerocopy);

const char *get_cpif_driver_version(void)
{
	return &(cpif_driver_version[0]);
}
