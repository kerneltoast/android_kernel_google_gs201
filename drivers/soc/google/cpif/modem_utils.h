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
	[STATE_INIT] = "INIT",
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

void get_utc_time(struct utc_time *utc);

static inline unsigned int calc_offset(void *target, void *base)
{
	return (unsigned long)target - (unsigned long)base;
}

int mif_dump_log(struct modem_shared *, struct io_device *);

#define mif_irq_log(msd, map, data, len) \
	_mif_irq_log(MIF_IRQ, msd, map, data, len)
#define mif_com_log(msd, format, ...) \
	_mif_com_log(MIF_COM, msd, pr_fmt(format), ##__VA_ARGS__)
#define mif_time_log(msd, epoch, data, len) \
	_mif_time_log(MIF_TIME, msd, epoch, data, len)

void mif_ipc_log(enum mif_log_id id,
	struct modem_shared *msd, const char *data, size_t len);
void _mif_irq_log(enum mif_log_id id,
	struct modem_shared *msd, struct mif_irq_map irq_map, const char *data, size_t len);
void _mif_com_log(enum mif_log_id id,
	struct modem_shared *msd, const char *data, ...);
void _mif_time_log(enum mif_log_id id,
	struct modem_shared *msd, struct timespec64 epoch, const char *data,
	size_t len);

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

/* print a urb as hex string */
#define pr_urb(tag, urb) \
	pr_buffer(tag, (char *)((urb)->transfer_buffer), \
			(size_t)((urb)->actual_length), (size_t)16)

/* Stop/wake all TX queues in network interfaces */
void stop_net_iface(struct link_device *ld, unsigned int channel);
void resume_net_iface(struct link_device *ld, unsigned int channel);
void stop_net_ifaces(struct link_device *ld);
void resume_net_ifaces(struct link_device *ld);

/* flow control CMD from CP, it use in serial devices */
int link_rx_flowctl_cmd(struct link_device *ld, const char *data, size_t len);

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

	if (!iod && atomic_read(&mld->cp_boot_done))
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

/* netif wake/stop queue of iod */
void iodev_netif_wake(struct io_device *iod, void *args);
void iodev_netif_stop(struct io_device *iod, void *args);

/* netif wake/stop queue of iod having activated ndev */
void netif_tx_flowctl(struct modem_shared *msd, bool tx_stop);

__be32 ipv4str_to_be32(const char *ipv4str, size_t count);

void mif_add_timer(struct timer_list *timer, unsigned long expire,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 15, 0))
				void (*function)(struct timer_list *));
#else
				void (*function)(unsigned long), unsigned long data);
#endif

/* debug helper functions for sipc4, sipc5 */
void mif_print_data(const u8 *data, int len);

void mif_dump2format16(const u8 *data, int len, char *buff, char *tag);
void mif_dump2format4(const u8 *data, int len, char *buff, char *tag);
void mif_print_dump(const u8 *data, int len, int width);

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

void print_ipv4_packet(const u8 *ip_pkt, enum direction dir);
bool is_dns_packet(const u8 *ip_pkt);
bool is_syn_packet(const u8 *ip_pkt);

void mif_init_irq(struct modem_irq *irq, unsigned int num, const char *name,
		  unsigned long flags);
int mif_request_irq(struct modem_irq *irq, irq_handler_t isr, void *data);
void mif_enable_irq(struct modem_irq *irq);
void mif_disable_irq(struct modem_irq *irq);
bool mif_gpio_set_value(struct cpif_gpio *gpio, int value, unsigned int delay_ms);
int mif_gpio_get_value(struct cpif_gpio *gpio, bool log_print);
int mif_gpio_toggle_value(struct cpif_gpio *gpio, int delay_ms);

struct file *mif_open_file(const char *path);
void mif_save_file(struct file *fp, const char *buff, size_t size);
void mif_close_file(struct file *fp);

int board_gpio_export(struct device *dev,
		unsigned int gpio, bool dir, const char *name);

void make_gpio_floating(unsigned int gpio, bool floating);

#if IS_ENABLED(CONFIG_ARGOS)
/* kernel team needs to provide argos header file. !!!
 * As of now, there's nothing to use.
 */
#if IS_ENABLED(CONFIG_SCHED_HMP)
extern struct cpumask hmp_slow_cpu_mask;
extern struct cpumask hmp_fast_cpu_mask;
#endif

int argos_irq_affinity_setup_label(unsigned int irq, const char *label,
		struct cpumask *affinity_cpu_mask,
		struct cpumask *default_cpu_mask);
int argos_task_affinity_setup_label(struct task_struct *p, const char *label,
		struct cpumask *affinity_cpu_mask,
		struct cpumask *default_cpu_mask);
#endif

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
#define MIF_BUFF_DEFAULT_PACKET_SIZE	(2048)
#define MIF_BUFF_CELL_PADDING_SIZE	(512)
#define MIF_BUFF_DEFAULT_CELL_SIZE	(MIF_BUFF_DEFAULT_PACKET_SIZE+MIF_BUFF_CELL_PADDING_SIZE)
#define MIF_BUFF_MAP_CELL_SIZE	(sizeof(uint64_t))
#define MIF_BITS_FOR_BYTE	(8)
#define MIF_BITS_FOR_MAP_CELL	(MIF_BUFF_MAP_CELL_SIZE * MIF_BITS_FOR_BYTE)
#define MIF_64BIT_FIRST_BIT	(0x8000000000000000ULL)

struct mif_buff_mng {
	unsigned char *buffer_start;
	unsigned char *buffer_end;
	unsigned int buffer_size;
	unsigned int cell_size;

	unsigned int cell_count;
	unsigned int used_cell_count;
	unsigned int free_cell_count;

	spinlock_t lock;

	uint64_t *buffer_map;
	unsigned int buffer_map_size;
	int current_map_index;

	struct list_head node;

	bool enable_sw_zerocopy;
};

struct mif_buff_mng *init_mif_buff_mng(unsigned char *buffer_start,
	unsigned int buffer_size, unsigned int cell_size);
void exit_mif_buff_mng(struct mif_buff_mng *bm);
void *alloc_mif_buff(struct mif_buff_mng *bm);
int free_mif_buff(struct mif_buff_mng *bm, void *buffer);

static inline unsigned int get_mif_buff_free_count(struct mif_buff_mng *bm)
{
	if (bm)
		return bm->free_cell_count;
	else
		return 0;
}

static inline unsigned int get_mif_buff_used_count(struct mif_buff_mng *bm)
{
	if (bm)
		return bm->used_cell_count;
	else
		return 0;
}

extern struct mif_buff_mng *g_mif_buff_mng;
void set_dflags(unsigned long flag);

const char *get_cpif_driver_version(void);

extern bool __skb_free_head_cp_zerocopy(struct sk_buff *skb);
extern void cpif_enable_sw_zerocopy(void);

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

#endif/*__MODEM_UTILS_H__*/
