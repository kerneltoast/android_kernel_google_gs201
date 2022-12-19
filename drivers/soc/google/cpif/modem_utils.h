/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2011 Samsung Electronics.
 *
 */

#ifndef __MODEM_UTILS_H__
#define __MODEM_UTILS_H__

#include <linux/rbtree.h>
#include "modem_prj.h"
#include "link_device_memory.h"

#define CP_CPU_BASE_ADDRESS	0x40000000

#define MIF_TAG	"cpif"

#define IS_CONNECTED(iod, ld) ((iod)->link_type == (ld)->link_type)

#define MAX_MIF_BUFF_SIZE 0x80000 /* 512kb */
#define MAX_MIF_SEPA_SIZE 32
#define MIF_SEPARATOR "IPC_LOGGER(VER1.1)"
#define MAX_IPC_SKB_SIZE 4096
#define MAX_LOG_SIZE 64

#define MAX_LOG_CNT (MAX_MIF_BUFF_SIZE / MAX_LOG_SIZE)
#define MIF_ID_SIZE sizeof(enum mif_log_id)

#define MAX_IPC_LOG_SIZE \
	(MAX_LOG_SIZE - sizeof(enum mif_log_id) \
	 - sizeof(unsigned long long) - sizeof(size_t))
#define MAX_IRQ_LOG_SIZE \
	(MAX_LOG_SIZE - sizeof(enum mif_log_id) \
	 - sizeof(unsigned long long) - sizeof(struct mif_irq_map))
#define MAX_COM_LOG_SIZE \
	(MAX_LOG_SIZE - sizeof(enum mif_log_id) \
	 - sizeof(unsigned long long))
#define MAX_TIM_LOG_SIZE \
	(MAX_LOG_SIZE - sizeof(enum mif_log_id) \
	 - sizeof(unsigned long long) - sizeof(struct timespec64))

#define PR_BUFFER_SIZE	128

#define PADDR_LO(paddr)	((paddr) & 0xFFFFFFFF)
#define PADDR_HI(paddr)	(((paddr) >> 32) & 0xF)

enum mif_log_id {
	MIF_IPC_RL2AP = 1,
	MIF_IPC_AP2CP,
	MIF_IPC_CP2AP,
	MIF_IPC_AP2RL,
	MIF_IRQ,
	MIF_COM,
	MIF_TIME
};

struct mif_irq_map {
	u16 magic;
	u16 access;

	u16 fmt_tx_in;
	u16 fmt_tx_out;
	u16 fmt_rx_in;
	u16 fmt_rx_out;

	u16 raw_tx_in;
	u16 raw_tx_out;
	u16 raw_rx_in;
	u16 raw_rx_out;

	u16 cp2ap;
};

struct mif_ipc_block {
	enum mif_log_id id;
	unsigned long long time;
	size_t len;
	char buff[MAX_IPC_LOG_SIZE];
};

struct mif_irq_block {
	enum mif_log_id id;
	unsigned long long time;
	struct mif_irq_map map;
	char buff[MAX_IRQ_LOG_SIZE];
};

struct mif_common_block {
	enum mif_log_id id;
	unsigned long long time;
	char buff[MAX_COM_LOG_SIZE];
};

struct mif_time_block {
	enum mif_log_id id;
	unsigned long long time;
	struct timespec64 epoch;
	char buff[MAX_TIM_LOG_SIZE];
};

enum ipc_layer {
	LINK,
	IODEV,
	APP,
	MAX_SIPC_LAYER
};

static const char * const sipc_layer_string[] = {
	[LINK] = "LNK",
	[IODEV] = "IOD",
	[APP] = "APP",
	[MAX_SIPC_LAYER] = "INVALID"
};

static const inline char *layer_str(enum ipc_layer layer)
{
	if (unlikely(layer >= MAX_SIPC_LAYER))
		return "INVALID";
	else
		return sipc_layer_string[layer];
}

static const char * const dev_format_string[] = {
	[IPC_FMT]	= "FMT",
	[IPC_RAW]	= "RAW",
	[IPC_RFS]	= "RFS",
	[IPC_MULTI_RAW]	= "MULTI_RAW",
	[IPC_BOOT]	= "BOOT",
	[IPC_DUMP]	= "DUMP",
	[IPC_CMD]	= "CMD",
	[IPC_DEBUG]	= "DEBUG",
};

static const inline char *dev_str(u32 dev)
{
	if (unlikely(dev >= MAX_DEV_FORMAT))
		return "INVALID";
	else
		return dev_format_string[dev];
}

static inline enum direction opposite(enum direction dir)
{
	return (dir == TX) ? RX : TX;
}

static const char * const direction_string[] = {
	[TX] = "TX",
	[RX] = "RX"
};

static const inline char *dir_str(enum direction dir)
{
	if (unlikely(dir >= MAX_DIR))
		return "INVALID";
	else
		return direction_string[dir];
}

static const char * const udl_string[] = {
	[UL] = "UL",
	[DL] = "DL"
};

static const inline char *udl_str(enum direction dir)
{
	if (unlikely(dir >= ULDL))
		return "INVALID";
	else
		return udl_string[dir];
}

static const char * const q_direction_string[] = {
	[TX] = "TXQ",
	[RX] = "RXQ"
};

static const inline char *q_dir(enum direction dir)
{
	if (unlikely(dir >= MAX_DIR))
		return "INVALID";
	else
		return q_direction_string[dir];
}

static const char * const ipc_direction_string[] = {
	[TX] = "AP->CP",
	[RX] = "AP<-CP"
};

static const inline char *ipc_dir(enum direction dir)
{
	if (unlikely(dir >= MAX_DIR))
		return "INVALID";
	else
		return ipc_direction_string[dir];
}

static const char * const arrow_direction[] = {
	[TX] = "->",
	[RX] = "<-"
};

static const inline char *arrow(enum direction dir)
{
	if (unlikely(dir >= MAX_DIR))
		return "><";
	else
		return arrow_direction[dir];
}

static const char * const modem_state_string[] = {
	[STATE_OFFLINE]		= "OFFLINE",
	[STATE_CRASH_RESET]	= "CRASH_RESET",
	[STATE_CRASH_EXIT]	= "CRASH_EXIT",
	[STATE_BOOTING]		= "BOOTING",
	[STATE_ONLINE]		= "ONLINE",
	[STATE_NV_REBUILDING]	= "NV_REBUILDING",
	[STATE_LOADER_DONE]	= "LOADER_DONE",
	[STATE_SIM_ATTACH]	= "SIM_ATTACH",
	[STATE_SIM_DETACH]	= "SIM_DETACH",
	[STATE_CRASH_WATCHDOG]	= "WDT_RESET",
	[STATE_RESET]		= "RESET",
};

static const inline char *cp_state_str(enum modem_state state)
{
	return modem_state_string[state];
}

static const inline char *mc_state(struct modem_ctl *mc)
{
	return cp_state_str(mc->phone_state);
}

struct __packed utc_time {
	u32 year:18,
	    mon:4,
	    day:5,
	    hour:5;
	u32 min:6,
	    sec:6,
	    us:20;
};

/* {Hour, Minute, Second, U(micro)-second} format */
#define HMSU_FMT	"[%02d:%02d:%02d.%06d]"

static inline unsigned long ns2us(unsigned long ns)
{
	return (ns > 0) ? (ns / 1000) : 0;
}

static inline unsigned long ns2ms(unsigned long ns)
{
	return (ns > 0) ? (ns / 1000000) : 0;
}

static inline unsigned long us2ms(unsigned long us)
{
	return (us > 0) ? (us / 1000) : 0;
}

static inline unsigned long ms2us(unsigned long ms)
{
	return ms * 1E3L;
}

static inline unsigned long ms2ns(unsigned long ms)
{
	return ms * 1E6L;
}

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

static inline void get_utc_time(struct utc_time *utc)
{
	struct timespec64 ts;

	ktime_get_ts64(&ts);
	ts642utc(&ts, utc);
}

/* dump2hex
 * dump data to hex as fast as possible.
 * the length of @buff must be greater than "@len * 3"
 * it need 3 bytes per one data byte to print.
 */
static inline void dump2hex(char *buff, size_t buff_size,
			    const char *data, size_t data_len)
{
	static const char *hex = "0123456789abcdef";
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

static inline unsigned int calc_offset(void *target, void *base)
{
	return (unsigned long)target - (unsigned long)base;
}

static inline struct link_device *find_linkdev(struct modem_shared *msd,
		u32 link_type)
{
	struct link_device *ld;

	list_for_each_entry(ld, &msd->link_dev_list, list) {
		if (ld->link_type == link_type)
			return ld;
	}
	return NULL;
}

static inline unsigned int count_bits(unsigned int n)
{
	unsigned int i;

	for (i = 0; n != 0; i++)
		n &= (n - 1);
	return i;
}

static inline bool count_flood(int cnt, int mask)
{
	return (cnt > 0 && (cnt & mask) == 0) ? true : false;
}

void mif_pkt(u8 ch, const char *tag, struct sk_buff *skb);

/* print buffer as hex string */
int pr_buffer(const char *tag, const char *data, size_t data_len,
							size_t max_len);

/* print a sk_buff as hex string */
#define PRINT_SKBUFF_PROTOCOL_SIT	24
#define PRINT_SKBUFF_PROTOCOL_SIPC	16
static inline void pr_skb(const char *tag, struct sk_buff *skb, struct link_device *ld)
{
	int length = 0;

	switch (ld->protocol) {
	case PROTOCOL_SIT:
		length = PRINT_SKBUFF_PROTOCOL_SIT;
		break;
	case PROTOCOL_SIPC:
		length = PRINT_SKBUFF_PROTOCOL_SIPC;
		break;
	default:
		mif_err("ERR - unknwon protocol\n");
		break;
	}

	pr_buffer(tag, (char *)((skb)->data), (size_t)((skb)->len), length);
}

/* Stop/wake all normal priority TX queues in network interfaces */
bool stop_net_ifaces(struct link_device *ld, unsigned long set_mask);
void resume_net_ifaces(struct link_device *ld, unsigned long clear_mask);

/* Get an IO device */
struct io_device *get_iod_with_format(struct modem_shared *msd,
					u32 format);

static inline struct io_device *link_get_iod_with_format(
			struct link_device *ld, u32 format)
{
	struct io_device *iod = get_iod_with_format(ld->msd, format);

	return (iod && IS_CONNECTED(iod, ld)) ? iod : NULL;
}

static inline struct io_device *get_iod_with_channel(
			struct modem_shared *msd, unsigned int channel)
{
	return msd->ch2iod[channel];
}

static inline struct io_device *link_get_iod_with_channel(
			struct link_device *ld, unsigned int channel)
{
	struct io_device *iod = get_iod_with_channel(ld->msd, channel);
	struct mem_link_device *mld = ld->mdm_data->mld;

	if (!iod && atomic_read(&mld->init_end_cnt))
		mif_err("No IOD matches channel (%d)\n", channel);

	return (iod && IS_CONNECTED(iod, ld)) ? iod : NULL;
}

/* insert iod to tree functions */
struct io_device *insert_iod_with_format(struct modem_shared *msd,
			u32 format, struct io_device *iod);
void insert_iod_with_channel(struct modem_shared *msd, unsigned int channel,
			     struct io_device *iod);

/* iodev for each */
typedef void (*action_fn)(struct io_device *iod, void *args);
static inline void iodevs_for_each(struct modem_shared *msd, action_fn action, void *args)
{
	int i;

	for (i = 0; i < msd->num_channels; i++) {
		u8 ch = msd->ch[i];
		struct io_device *iod = msd->ch2iod[ch];

		action(iod, args);
	}
}

__be32 ipv4str_to_be32(const char *ipv4str, size_t count);

void mif_add_timer(struct timer_list *timer, unsigned long expire,
				void (*function)(struct timer_list *));

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
#define IPV4_HDR_SIZE	20

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
#define TCP_HDR_SIZE	20

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
#define UDP_HDR_SIZE	8

#ifdef DEBUG_MODEM_IF_IP_DATA
void print_ipv4_packet(const u8 *ip_pkt, enum direction dir);
#endif

void mif_init_irq(struct modem_irq *irq, unsigned int num, const char *name,
		  unsigned long flags);
int mif_request_irq(struct modem_irq *irq, irq_handler_t isr, void *data);
void mif_free_irq(struct modem_irq *irq, void *data);
void mif_enable_irq(struct modem_irq *irq);
void mif_disable_irq(struct modem_irq *irq);
bool mif_gpio_set_value(struct cpif_gpio *gpio, int value, unsigned int delay_ms);
int mif_gpio_get_value(struct cpif_gpio *gpio, bool log_print);
int mif_gpio_toggle_value(struct cpif_gpio *gpio, int delay_ms);

struct file *mif_open_file(const char *path);
void mif_save_file(struct file *fp, const char *buff, size_t size);
void mif_close_file(struct file *fp);

void mif_stop_logging(void);
void set_wakeup_packet_log(bool enable);

/* MIF buffer management */

/* Printout debuggin message for MIF buffer */
//#define MIF_BUFF_DEBUG

/*
 * IP packet : 2048
 * sizeof(struct skb_shared_info): 512
 * 2048 + 512 = 2560 (0xA00)
 */
#define MIF_BUFF_DEFAULT_PACKET_SIZE   (2048)
#define MIF_BUFF_CELL_PADDING_SIZE     (512)
#define MIF_BUFF_DEFAULT_CELL_SIZE     (MIF_BUFF_DEFAULT_PACKET_SIZE+MIF_BUFF_CELL_PADDING_SIZE)

const char *get_cpif_driver_version(void);

static inline struct wakeup_source *cpif_wake_lock_register(struct device *dev, const char *name)
{
	struct wakeup_source *ws = NULL;

	ws = wakeup_source_register(dev, name);
	if (ws == NULL) {
		mif_err("%s: wakelock register fail\n", name);
		return NULL;
	}

	return ws;
}

static inline void cpif_wake_lock_unregister(struct wakeup_source *ws)
{
	if (ws == NULL) {
		mif_err("wakelock unregister fail\n");
		return;
	}

	wakeup_source_unregister(ws);
}

static inline void cpif_wake_lock(struct wakeup_source *ws)
{
	if (ws == NULL) {
		mif_err("wakelock fail\n");
		return;
	}

	__pm_stay_awake(ws);
}

static inline void cpif_wake_lock_timeout(struct wakeup_source *ws, long timeout)
{
	if (ws == NULL) {
		mif_err("wakelock timeout fail\n");
		return;
	}

	__pm_wakeup_event(ws, jiffies_to_msecs(timeout));
}

static inline void cpif_wake_unlock(struct wakeup_source *ws)
{
	if (ws == NULL) {
		mif_err("wake unlock fail\n");
		return;
	}

	__pm_relax(ws);
}

static inline int cpif_wake_lock_active(struct wakeup_source *ws)
{
	if (ws == NULL) {
		mif_err("wake unlock fail\n");
		return 0;
	}

	return ws->active;
}

int copy_from_user_memcpy_toio(void __iomem *dst, const void __user *src, size_t count);

#endif/*__MODEM_UTILS_H__*/
