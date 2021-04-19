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
#include <linux/delay.h>
#include <linux/bitops.h>
#include <soc/google/exynos-modem-ctrl.h>

#include <soc/google/acpm_ipc_ctrl.h>
#include "modem_prj.h"
#include "modem_utils.h"
#include "cpif_version.h"

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
			void (*function)(struct timer_list *))
{
	if (timer_pending(timer))
		return;

	timer_setup(timer, function, 0);
	timer->expires = get_jiffies_64() + expire;

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

	if (!gpio->valid) {
		mif_err("GET GPIO %d is not valid\n", gpio->num);
		return false;
	}

	if (gpio->get_gpio_value(gpio->num) == value)
		dup = 1;

	/* set gpio even if it is set already */
	gpio->set_gpio_value(gpio->num, value);

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
EXPORT_SYMBOL(mif_gpio_set_value);

int mif_gpio_get_value(struct cpif_gpio *gpio, bool log_print)
{
	int value;

	if (!gpio->valid) {
		mif_err("GET GPIO %d is not valid\n", gpio->num);
		return -EINVAL;
	}

	value = gpio->get_gpio_value(gpio->num);

	if (log_print)
		mif_debug("GET GPIO %s = %d\n", gpio->label, value);

	return value;
}
EXPORT_SYMBOL(mif_gpio_get_value);

int mif_gpio_toggle_value(struct cpif_gpio *gpio, int delay_ms)
{
	int value;

	value = mif_gpio_get_value(gpio, false);
	mif_gpio_set_value(gpio, !value, delay_ms);
	mif_gpio_set_value(gpio, value, 0);

	return value;
}
EXPORT_SYMBOL(mif_gpio_toggle_value);

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

const char *get_cpif_driver_version(void)
{
	return &(cpif_driver_version[0]);
}
