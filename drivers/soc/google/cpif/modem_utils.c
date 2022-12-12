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

static bool wakeup_log_enable;
inline void set_wakeup_packet_log(bool enable)
{
	wakeup_log_enable = enable;
}

inline unsigned long get_log_flags(void)
{
	return wakeup_log_enable ? wakeup_dflags : dflags;
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

/* netif wake/stop queue of iod having activated ndev */
static void netif_tx_flowctl(struct modem_shared *msd, bool tx_stop)
{
	struct io_device *iod;

	if (!msd) {
		mif_err_limited("modem shared data does not exist\n");
		return;
	}

	spin_lock(&msd->active_list_lock);
	list_for_each_entry(iod, &msd->activated_ndev_list, node_ndev) {
		if (tx_stop)
			netif_stop_subqueue(iod->ndev, 0);
		else
			netif_wake_subqueue(iod->ndev, 0);

#ifdef DEBUG_MODEM_IF_FLOW_CTRL
		mif_err("tx_stop:%s, iod->ndev->name:%s\n",
			tx_stop ? "suspend" : "resume",
			iod->ndev->name);
#endif
	}
	spin_unlock(&msd->active_list_lock);
}

bool stop_net_ifaces(struct link_device *ld, unsigned long set_mask)
{
	bool ret = false;

	if (set_mask > 0)
		cpif_set_bit(ld->tx_flowctrl_mask, set_mask);

	if (!atomic_read(&ld->netif_stopped)) {
		mif_info_limited("tx queue stopped: tx_flowctrl=0x%04lx(set_bit:%lu)\n",
			 ld->tx_flowctrl_mask, set_mask);

		netif_tx_flowctl(ld->msd, true);
		atomic_set(&ld->netif_stopped, 1);
		ret = true;
	}

	return ret;
}

void resume_net_ifaces(struct link_device *ld, unsigned long clear_mask)
{
	cpif_clear_bit(ld->tx_flowctrl_mask, clear_mask);

	if (!ld->tx_flowctrl_mask && atomic_read(&ld->netif_stopped)) {
		mif_info_limited("tx queue resumed: tx_flowctrl=0x%04lx(clear_bit:%lu)\n",
			 ld->tx_flowctrl_mask, clear_mask);

		netif_tx_flowctl(ld->msd, false);
		atomic_set(&ld->netif_stopped, 0);
	}
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

#ifdef DEBUG_MODEM_IF_IP_DATA
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
#endif /* DEBUG_MODEM_IF_IP_DATA */

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

void mif_free_irq(struct modem_irq *irq, void *data)
{
	free_irq(irq->num, data);
	mif_info("%s(#%d) handler unregistered (flags:0x%08lX)\n",
		irq->name, irq->num, irq->flags);
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
		mif_err("SET GPIO %d is not valid\n", gpio->num);
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
EXPORT_SYMBOL(mif_gpio_set_value);

int mif_gpio_get_value(struct cpif_gpio *gpio, bool log_print)
{
	int value;

	if (!gpio->valid) {
		mif_err("GET GPIO %d is not valid\n", gpio->num);
		return -EINVAL;
	}

	value = gpio_get_value(gpio->num);

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

void mif_stop_logging(void)
{
}

const char *get_cpif_driver_version(void)
{
	return &(cpif_driver_version[0]);
}

int copy_from_user_memcpy_toio(void __iomem *dst, const void __user *src, size_t count)
{
	u8 buf[256];

	while (count) {
		size_t c = count;

		if (c > sizeof(buf))
			c = sizeof(buf);
		if (copy_from_user(buf, src, c))
			return -EFAULT;

		memcpy_toio(dst, buf, c);
		count -= c;
		dst += c;
		src += c;
	}

	return 0;
}
