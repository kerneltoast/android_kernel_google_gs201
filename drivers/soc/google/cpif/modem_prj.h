/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2010 Samsung Electronics.
 *
 */

#ifndef __MODEM_PRJ_H__
#define __MODEM_PRJ_H__

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/skbuff.h>
#include <linux/wait.h>
#include <linux/completion.h>
#include <linux/pm_wakeup.h>
#include <linux/spinlock.h>
#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/netdevice.h>
#include <linux/pm_runtime.h>
#include <linux/version.h>
#if IS_ENABLED(CONFIG_EXYNOS_ITMON)
#include <soc/google/exynos-itmon.h>
#endif
#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE)
#include <linux/pci.h>
#if IS_ENABLED(CONFIG_GS_S2MPU)
#include <soc/google/s2mpu.h>
#endif
#endif
#include "modem_debug.h"
#include "modem_v1.h"

#include "include/circ_queue.h"
#include "include/sipc5.h"
#include "include/exynos_ipc.h"

#define DEBUG_MODEM_IF
#ifdef DEBUG_MODEM_IF
/* #define DEBUG_MODEM_IF_LINK_TX */
/* #define DEBUG_MODEM_IF_LINK_RX */

/* #define DEBUG_MODEM_IF_IODEV_TX */
/* #define DEBUG_MODEM_IF_IODEV_RX */

/* #define DEBUG_MODEM_IF_FLOW_CTRL */

/* #define DEBUG_MODEM_IF_PS_DATA */
/* #define DEBUG_MODEM_IF_IP_DATA */
#endif

/*
 * IOCTL commands
 */
#define IOCTL_MAGIC	'o'

#define IOCTL_POWER_ON			_IO(IOCTL_MAGIC, 0x19)
#define IOCTL_POWER_OFF			_IO(IOCTL_MAGIC, 0x20)

enum cp_boot_mode {
	CP_BOOT_MODE_NORMAL,
	CP_BOOT_MODE_DUMP,
	CP_BOOT_RE_INIT,
	CP_BOOT_REQ_CP_RAM_LOGGING = 5,
	CP_BOOT_MODE_MANUAL = 7,
	CP_BOOT_EXT_BAAW = 11,
	MAX_CP_BOOT_MODE
};
struct boot_mode {
	enum cp_boot_mode idx;
};
#define IOCTL_POWER_RESET		_IOW(IOCTL_MAGIC, 0x21, struct boot_mode)
#define IOCTL_START_CP_BOOTLOADER	_IOW(IOCTL_MAGIC, 0x22, struct boot_mode)
#define IOCTL_COMPLETE_NORMAL_BOOTUP	_IO(IOCTL_MAGIC, 0x23)
#define IOCTL_GET_CP_STATUS		_IO(IOCTL_MAGIC, 0x27)
#define IOCTL_START_CP_DUMP		_IO(IOCTL_MAGIC, 0x32)
#define IOCTL_TRIGGER_CP_CRASH		_IO(IOCTL_MAGIC, 0x34)
#define IOCTL_TRIGGER_KERNEL_PANIC	_IO(IOCTL_MAGIC, 0x35)

struct cp_image {
	unsigned long long binary;
	u32 size;
	u32 m_offset;
	u32 b_offset;
	u32 mode;
	u32 len;
} __packed;
#define IOCTL_LOAD_CP_IMAGE		_IOW(IOCTL_MAGIC, 0x40, struct cp_image)

#define IOCTL_GET_SRINFO		_IO(IOCTL_MAGIC, 0x45)
#define IOCTL_SET_SRINFO		_IO(IOCTL_MAGIC, 0x46)
#define IOCTL_GET_CP_BOOTLOG		_IO(IOCTL_MAGIC, 0x47)
#define IOCTL_CLR_CP_BOOTLOG		_IO(IOCTL_MAGIC, 0x48)

/* AP capability index - considers first 32bits only*/
#define AP_CAP_PKTPROC_UL		0x00000001

/* Log dump */
#define IOCTL_MIF_LOG_DUMP		_IO(IOCTL_MAGIC, 0x51)

enum cp_log_dump_index {
	LOG_IDX_SHMEM,
	LOG_IDX_VSS,
	LOG_IDX_ACPM,
	LOG_IDX_CP_BTL,
	LOG_IDX_DATABUF_DL,
	LOG_IDX_DATABUF_UL,
	LOG_IDX_L2B,
	MAX_LOG_DUMP_IDX
};
struct cp_log_dump {
	char name[32];
	enum cp_log_dump_index idx;
	u32 size;
} __packed;
#define IOCTL_GET_LOG_DUMP		_IOWR(IOCTL_MAGIC, 0x52, struct cp_log_dump)

struct modem_sec_req {
	u32 mode;
	u32 param2;
	u32 param3;
	u32 param4;
} __packed;
#define IOCTL_REQ_SECURITY		_IOW(IOCTL_MAGIC, 0x53, struct modem_sec_req)

/* Crash Reason */
#define CP_CRASH_INFO_SIZE	512
#define CP_CRASH_TAG		"CP Crash "

enum crash_type {
	CRASH_REASON_CP_ACT_CRASH = 0,
	CRASH_REASON_RIL_MNR,
	CRASH_REASON_RIL_REQ_FULL,
	CRASH_REASON_RIL_PHONE_DIE,
	CRASH_REASON_RIL_RSV_MAX,
	CRASH_REASON_USER = 5,
	CRASH_REASON_MIF_TX_ERR = 6,
	CRASH_REASON_MIF_RIL_BAD_CH,
	CRASH_REASON_MIF_RX_BAD_DATA,
	CRASH_REASON_RIL_TRIGGER_CP_CRASH,
	CRASH_REASON_MIF_FORCED,
	CRASH_REASON_CP_WDOG_CRASH,
	CRASH_REASON_MIF_RSV_MAX = 12,
	CRASH_REASON_CP_SRST,
	CRASH_REASON_CP_RSV_0,
	CRASH_REASON_CP_RSV_MAX,
	CRASH_REASON_CLD = 16,
	CRASH_REASON_NONE = 0xFFFF,
};

struct crash_reason {
	u32 type;
	char string[CP_CRASH_INFO_SIZE];
} __packed;
#define IOCTL_GET_CP_CRASH_REASON	_IOR(IOCTL_MAGIC, 0x55, struct crash_reason)

#define CPIF_VERSION_SIZE	20
struct cpif_version {
	char string[CPIF_VERSION_SIZE];
} __packed;
#define IOCTL_GET_CPIF_VERSION		_IOR('o', 0x56, struct cpif_version)

#define CPID_LEN		15
#define CPSIG_LEN		64

struct t_handover_block_info {
	u32 version; /* version */
	u32 project_id; /* project id */
	u32 revision; /* revision */
	u32 major_id; /* major_id */
	u32 minor_id; /* minor_id */
	u32 modem_sku; /* modem sku */
	u32 modem_hw; /* modem hw */
	u32 cpinfo0;
	u32 cpinfo1;
	u32 cpinfo2;
	u32 rf_sub;
	u32 rf_config; /* rf config */
	u32 reserved[4];
	char cpid[2][CPID_LEN + 1];
	char cpsig[CPSIG_LEN + 1];
} __packed;
#define IOCTL_HANDOVER_BLOCK_INFO	_IO('o', 0x57)

/*
 * Definitions for IO devices
 */
#define MAX_IOD_RXQ_LEN		2048


#define IPv6			6
#define SOURCE_MAC_ADDR		{0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC}

/* Loopback */
#define DATA_LOOPBACK_CHANNEL	31

/* Debugging features */
#define MIF_LOG_DIR		"/sdcard/log"
#define MIF_MAX_PATH_LEN	256

/* Does modem ctl structure will use state ? or status defined below ?*/
enum modem_state {
	STATE_OFFLINE,
	STATE_CRASH_RESET,	/* silent reset */
	STATE_CRASH_EXIT,	/* cp ramdump */
	STATE_BOOTING,
	STATE_ONLINE,
	STATE_NV_REBUILDING,	/* <= rebuilding start */
	STATE_LOADER_DONE,
	STATE_SIM_ATTACH,
	STATE_SIM_DETACH,
	STATE_CRASH_WATCHDOG,	/* cp watchdog crash */
	STATE_INIT,		/* cp booting has not been tried yet */
};

enum link_state {
	LINK_STATE_OFFLINE = 0,
	LINK_STATE_IPC,
	LINK_STATE_CP_CRASH
};

struct sim_state {
	bool online;	/* SIM is online? */
	bool changed;	/* online is changed? */
};

struct cp_power_stats {
	u64 count;			/* count state was entered */
	u64 duration_usec;		/* total time (usecs) in state */
	u64 last_entry_timestamp_usec;	/* timestamp(usecs since boot) of last time entered */
	u64 last_exit_timestamp_usec;	/* timestamp(usecs since boot) of last time exited */
};

struct sec_info {
	enum cp_boot_mode mode;
	u32 size;
};

#define SIPC_MULTI_FRAME_MORE_BIT	(0x80)
#define SIPC_MULTI_FRAME_ID_MASK	(0x7F)
#define SIPC_MULTI_FRAME_ID_BITS	7
#define NUM_SIPC_MULTI_FRAME_IDS	(2 ^ SIPC_MULTI_FRAME_ID_BITS)
#define MAX_SIPC_MULTI_FRAME_ID		(NUM_SIPC_MULTI_FRAME_IDS - 1)

struct __packed sipc_fmt_hdr {
	u16 len;
	u8  msg_seq;
	u8  ack_seq;
	u8  main_cmd;
	u8  sub_cmd;
	u8  cmd_type;
};

/* Channel 0, 5, 6, 27, 255 are reserved in SIPC5.
 * see SIPC5 spec: 2.2.2 Channel Identification (Ch ID) Field.
 * They do not need to store in `iodevs_tree_fmt'
 */
#define sipc5_is_not_reserved_channel(ch) \
	((ch) != 0 && (ch) != 5 && (ch) != 6 && (ch) != 27 && (ch) != 255)

#if IS_ENABLED(CONFIG_MODEM_IF_LEGACY_QOS) || IS_ENABLED(CONFIG_MODEM_IF_QOS)
#define MAX_NDEV_TX_Q 2
#else
#define MAX_NDEV_TX_Q 1
#endif
#define MAX_NDEV_RX_Q 1
/* mark value for high priority packet, hex QOSH */
#define RAW_HPRIO	0x514F5348

/* for fragmented data from link devices */
struct fragmented_data {
	struct sk_buff *skb_recv;
	struct sipc5_frame_data f_data;
	/* page alloc fail retry*/
	unsigned int realloc_offset;
};
#define fragdata(iod, ld) (&(iod)->fragments[(ld)->link_type])

/** struct skbuff_priv - private data of struct sk_buff
 * this is matched to char cb[48] of struct sk_buff
 */
struct skbuff_private {
	struct io_device *iod;
	struct link_device *ld;

	/* for time-stamping */
	struct timespec64 ts;

	u32 sipc_ch:8,	/* SIPC Channel Number			*/
	    frm_ctrl:8,	/* Multi-framing control		*/
	    reserved:14,
	    lnk_hdr:1,	/* Existence of a link-layer header	*/
	    rx_clat:1;	/* IP converted by Rx CLAT		*/

	struct napi_struct *napi;
} __packed;

static inline struct skbuff_private *skbpriv(struct sk_buff *skb)
{
	BUILD_BUG_ON(sizeof(struct skbuff_private) > sizeof(skb->cb));
	return (struct skbuff_private *)&skb->cb;
}

enum iod_rx_state {
	IOD_RX_ON_STANDBY = 0,
	IOD_RX_HEADER,
	IOD_RX_PAYLOAD,
	IOD_RX_PADDING,
	MAX_IOD_RX_STATE
};

static const char * const rx_state_string[] = {
	[IOD_RX_ON_STANDBY]	= "RX_ON_STANDBY",
	[IOD_RX_HEADER]		= "RX_HEADER",
	[IOD_RX_PAYLOAD]	= "RX_PAYLOAD",
	[IOD_RX_PADDING]	= "RX_PADDING",
};

static const inline char *rx_state(enum iod_rx_state state)
{
	if (unlikely(state >= MAX_IOD_RX_STATE))
		return "INVALID_STATE";
	else
		return rx_state_string[state];
}

struct io_device {
	struct list_head list;

	/* rb_tree node for an io device */
	struct rb_node node_chan;
	struct rb_node node_fmt;

	/* Name of the IO device */
	char *name;

	/* Reference count */
	atomic_t opened;

	/* Wait queue for the IO device */
	wait_queue_head_t wq;

	/* char device and net device structures for the IO device */
	struct cdev cdev;
	struct device *cdevice;
	struct net_device *ndev;
	struct list_head node_ndev;
#if IS_ENABLED(CONFIG_CPIF_TP_MONITOR)
	struct list_head node_all_ndev;
#endif

	/* clat net device */
	struct net_device *clat_ndev;
	/* spinlock to hold clat net device */
	spinlock_t clat_lock;

	/* CH and Format for channel on the link */
	unsigned int ch;
	u32 link_type;
	u32 format;
	u32 io_typ;
	enum modem_network net_typ;

	/* Attributes of an IO device */
	u32 attrs;

	/* The size of maximum Tx packet */
	unsigned int max_tx_size;

	/* SIPC version */
	u32 ipc_version;

	/* Whether or not IPC is over SBD-based link device */
	bool sbd_ipc;

	/* Whether or not link-layer header is required */
	bool link_header;

	/* Rx queue of sk_buff */
	struct sk_buff_head sk_rx_q;

	/* For keeping multi-frame packets temporarily */
	struct sk_buff_head sk_multi_q[NUM_SIPC_MULTI_FRAME_IDS];

	/* RX state used in RX FSM */
	enum iod_rx_state curr_rx_state;
	enum iod_rx_state next_rx_state;

	/*
	 * work for each io device, when delayed work needed
	 * use this for private io device rx action
	 */
	struct delayed_work rx_work;

	/* Information ID for supporting 'Multi FMT'
	 * reference SIPC Spec. 2.2.4
	 */
	u8 info_id;
	spinlock_t info_id_lock;

	struct fragmented_data fragments[LINKDEV_MAX];

	int (*recv_skb_single)(struct io_device *iod, struct link_device *ld,
			       struct sk_buff *skb);

	int (*recv_net_skb)(struct io_device *iod, struct link_device *ld,
			    struct sk_buff *skb);

	/* inform the IO device that the SIM is not inserting or removing */
	void (*sim_state_changed)(struct io_device *iod, bool sim_online);

	struct modem_ctl *mc;
	struct modem_shared *msd;

	struct wakeup_source *ws;
	long waketime;

	/* DO NOT use __current_link directly
	 * you MUST use skbpriv(skb)->ld in mc, link, etc..
	 */
	struct link_device *__current_link;

	struct exynos_seq_num seq_num;
	u8 packet_index;
};
#define to_io_device(_cdev) container_of(_cdev, struct io_device, cdev)

/* get_current_link, set_current_link don't need to use locks.
 * In ARM, set_current_link and get_current_link are compiled to
 * each one instruction (str, ldr) as atomic_set, atomic_read.
 * And, the order of set_current_link and get_current_link is not important.
 */
#define get_current_link(iod) ((iod)->__current_link)
#define set_current_link(iod, ld) ((iod)->__current_link = (ld))

struct link_device {
	struct list_head  list;
	u32 link_type;
	u32 interrupt_types;

	struct modem_ctl *mc;
	struct modem_shared *msd;
	struct device *dev;

	char *name;
	bool sbd_ipc;
	bool aligned;

	/* */
	u32 protocol;
	u8 chid_fmt_0;
	u8 chid_rfs_0;
	u32 magic_boot;
	u32 magic_crash;
	u32 magic_dump;
	u32 magic_ipc;
	bool (*is_start_valid)(u8 *frm);
	bool (*is_padding_exist)(u8 *frm);
	bool (*is_multi_frame)(u8 *frm);
	bool (*has_ext_len)(u8 *frm);
	u8 (*get_ch)(u8 *frm);
	u8 (*get_ctrl)(u8 *frm);
	u32 (*calc_padding_size)(u32 len);
	u32 (*get_hdr_len)(u8 *frm);
	u32 (*get_frame_len)(u8 *frm);
	u32 (*get_total_len)(u8 *frm);
	bool (*is_fmt_ch)(u8 ch);
	bool (*is_ps_ch)(u8 ch);
	bool (*is_rfs_ch)(u8 ch);
	bool (*is_boot_ch)(u8 ch);
	bool (*is_dump_ch)(u8 ch);
	bool (*is_bootdump_ch)(u8 ch);
	bool (*is_ipc_ch)(u8 ch);
	bool (*is_csd_ch)(u8 ch);
	bool (*is_log_ch)(u8 ch);
	bool (*is_router_ch)(u8 ch);
	bool (*is_misc_ch)(u8 ch);
	bool (*is_embms_ch)(u8 ch);
	bool (*is_uts_ch)(u8 ch);
	bool (*is_wfs0_ch)(u8 ch);
	bool (*is_wfs1_ch)(u8 ch);
	bool (*is_oem_ch)(u8 ch);

	/* SIPC version */
	u32 ipc_version;

	/* capability check */
	u32 capability_check;

	/* Modem data */
	struct modem_data *mdm_data;

	/* Stop/resume control for network ifaces */
	spinlock_t netif_lock;

	/* bit mask for stopped channel */
	unsigned long netif_stop_mask;
	unsigned long tx_flowctrl_mask;

	/* flag of stopped state for all channels */
	atomic_t netif_stopped;

	struct workqueue_struct *rx_wq;

	/* MIF buffer management */
	struct mif_buff_mng *mif_buff_mng;

	/* Save reason of forced crash */
	struct crash_reason crash_reason;

	int (*init_comm)(struct link_device *ld, struct io_device *iod);
	void (*terminate_comm)(struct link_device *ld, struct io_device *iod);

	/* called by an io_device when it has a packet to send over link
	 * - the io device is passed so the link device can look at id and
	 *   format fields to determine how to route/format the packet
	 */
	int (*send)(struct link_device *ld, struct io_device *iod,
		    struct sk_buff *skb);

	/* method for CP booting */
	int (*load_cp_image)(struct link_device *ld, struct io_device *iod, unsigned long arg);
	void (*link_prepare_normal_boot)(struct link_device *ld, struct io_device *iod);
	int (*link_start_normal_boot)(struct link_device *ld, struct io_device *iod);

	void (*link_trigger_cp_crash)(struct mem_link_device *mld, u32 crash_reason_owner,
			char *crash_reason_string);
	int (*link_start_dump_boot)(struct link_device *ld, struct io_device *iod);

	/* IOCTL extension */
	int (*ioctl)(struct link_device *ld, struct io_device *iod,
		     unsigned int cmd, unsigned long arg);

	/* Close (stop) TX with physical link (on CP crash, etc.) */
	void (*close_tx)(struct link_device *ld);

	/* Change secure mode, Call SMC API */
	int (*security_req)(struct link_device *ld, struct io_device *iod,
			unsigned long arg);

	/* Get crash reason form modem_if driver */
	int (*get_cp_crash_reason)(struct link_device *ld, struct io_device *iod,
			unsigned long arg);

	/* Reset buffer & dma_addr for zerocopy */
	void (*reset_zerocopy)(struct link_device *ld);

	int (*enable_rx_int)(struct link_device *ld);
	int (*disable_rx_int)(struct link_device *ld);

	void (*start_timers)(struct mem_link_device *mld);
	void (*stop_timers)(struct mem_link_device *mld);

	void (*gro_flush)(struct link_device *ld, struct napi_struct *napi);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
	struct timespec64 (*update_flush_time)(struct timespec64 org_flush_time);
#else
	struct timespec (*update_flush_time)(struct timespec org_flush_time);
#endif

	int (*handover_block_info)(struct link_device *ld, unsigned long arg);

#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE)
	int (*register_pcie)(struct link_device *ld);
#endif

#if IS_ENABLED(CONFIG_SBD_BOOTLOG)
	/* print cp boot/main logs */
	struct timer_list cplog_timer;
#endif
};

extern long gro_flush_time;

#define pm_to_link_device(pm)	container_of(pm, struct link_device, pm)

static inline struct sk_buff *rx_alloc_skb(unsigned int length,
		struct io_device *iod, struct link_device *ld)
{
	struct sk_buff *skb;

	skb = dev_alloc_skb(length);
	if (likely(skb)) {
		skbpriv(skb)->iod = iod;
		skbpriv(skb)->ld = ld;
	}

	return skb;
}

struct modemctl_ops {
	int (*power_on)(struct modem_ctl *mc);
	int (*power_off)(struct modem_ctl *mc);
	int (*power_shutdown)(struct modem_ctl *mc);
	int (*power_reset)(struct modem_ctl *mc);
	int (*power_reset_dump)(struct modem_ctl *mc);

	int (*start_normal_boot)(struct modem_ctl *mc);
	int (*complete_normal_boot)(struct modem_ctl *mc);

	int (*trigger_cp_crash)(struct modem_ctl *mc);
	int (*start_dump_boot)(struct modem_ctl *mc);

	int (*suspend)(struct modem_ctl *mc);
	int (*resume)(struct modem_ctl *mc);
};

/* for IPC Logger */
struct mif_storage {
	char *addr;
	unsigned int cnt;
};

/* modem_shared - shared data for all io/link devices and a modem ctl
 * msd : mc : iod : ld = 1 : 1 : M : N
 */
struct modem_shared {
	/* list of link devices */
	struct list_head link_dev_list;

	/* list of activated ndev */
	struct list_head activated_ndev_list;
	spinlock_t active_list_lock;

	/* Array of pointers to IO devices corresponding to ch[n] */
	struct io_device *ch2iod[IOD_CH_ID_MAX];

	/* Array of active channels */
	u8 ch[IOD_CH_ID_MAX];

	/* The number of active channels in the array @ch[] */
	unsigned int num_channels;

	/* rb_tree root of io devices. */
	struct rb_root iodevs_tree_fmt; /* group by dev_format */

	/* for IPC Logger */
	struct mif_storage storage;
	spinlock_t lock;

	/* loopbacked IP address
	 * default is 0.0.0.0 (disabled)
	 * after you setted this, you can use IP packet loopback using this IP.
	 * exam: echo 1.2.3.4 > /sys/devices/virtual/misc/umts_multipdp/loopback
	 */
	__be32 loopback_ipaddr;

	/* char device */
	dev_t cdev_major;
	struct class *cdev_class;
};

struct modem_ctl {
	struct device *dev;
	char *name;
	struct modem_data *mdm_data;

	struct modem_shared *msd;
	void __iomem *sysram_alive;

	enum modem_state phone_state;
	struct sim_state sim_state;

	/* spin lock for each modem_ctl instance */
	spinlock_t lock;
	spinlock_t tx_timer_lock;

	/* list for notify to opened iod when changed modem state */
	struct list_head modem_state_notify_list;

	/* completion for waiting for CP initialization */
	struct completion init_cmpl;

	/* completion for waiting for CP power-off */
	struct completion off_cmpl;

	/* for broadcasting AP's PM state (active or sleep) */
	unsigned int int_pda_active;
	unsigned int int_cp_wakeup;
	/* for checking aliveness of CP */
	unsigned int irq_phone_active;

	/* for broadcasting AP LCD state */
	unsigned int int_lcd_status;

#if IS_ENABLED(CONFIG_LINK_DEVICE_SHMEM)
	unsigned int mbx_pda_active;
	unsigned int mbx_phone_active;
	unsigned int mbx_ap_wakeup;
	unsigned int mbx_ap_status;
	unsigned int mbx_cp_wakeup;
	unsigned int mbx_cp_status;

	/* for notify uart connection with direction*/
	unsigned int mbx_uart_noti;
	unsigned int int_uart_noti;

	/* for checking aliveness of CP */
	struct modem_irq irq_cp_wdt;
	struct modem_irq irq_cp_fail;

	/* Status Bit Info */
	unsigned int sbi_lte_active_mask;
	unsigned int sbi_lte_active_pos;
	unsigned int sbi_cp_status_mask;
	unsigned int sbi_cp_status_pos;

	unsigned int sbi_pda_active_mask;
	unsigned int sbi_pda_active_pos;
	unsigned int sbi_ap_status_mask;
	unsigned int sbi_ap_status_pos;

	unsigned int sbi_uart_noti_mask;
	unsigned int sbi_uart_noti_pos;

	unsigned int sbi_lcd_status_mask;
	unsigned int sbi_lcd_status_pos;

	unsigned int ap2cp_cfg_addr;
	void __iomem *ap2cp_cfg_ioaddr;
#endif

	unsigned int sbi_crash_type_mask;
	unsigned int sbi_crash_type_pos;

	unsigned int sbi_ds_det_mask;
	unsigned int sbi_ds_det_pos;

#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE)
	struct irq_chip *apwake_irq_chip;
	struct pci_dev *s51xx_pdev;
	struct workqueue_struct *wakeup_wq;
	struct work_struct wakeup_work;
	struct work_struct suspend_work;
	struct workqueue_struct *crash_wq;
	struct work_struct crash_work;

	struct wakeup_source *ws;
	struct mutex pcie_onoff_lock;
	struct mutex pcie_check_lock;
	spinlock_t pcie_tx_lock;
	spinlock_t pcie_pm_lock;
	struct pci_driver pci_driver;

	int int_pcie_link_ack;
	int pcie_ch_num;
	int pcie_cto_retry_cnt;
	int pcie_cto_retry_cnt_all;

	bool reserve_doorbell_int;
	bool pcie_registered;
	bool pcie_powered_on;
	bool pcie_pm_suspended;
	bool pcie_pm_resume_wait;
	int pcie_pm_resume_gpio_val;
	bool device_reboot;

#if IS_ENABLED(CONFIG_SUSPEND_DURING_VOICE_CALL)
	bool pcie_voice_call_on;
	struct work_struct call_on_work;
	struct work_struct call_off_work;
	struct notifier_block call_state_nb;
#endif

#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE_GPIO_WA)
	atomic_t dump_toggle_issued;
#endif

	struct cpif_gpio s5100_gpio_cp_pwr;
	struct cpif_gpio s5100_gpio_cp_reset;
	struct cpif_gpio s5100_gpio_cp_ps_hold;
	struct cpif_gpio s5100_gpio_cp_wakeup;
	struct cpif_gpio s5100_gpio_cp_dump_noti;
	struct cpif_gpio s5100_gpio_ap_status;
	struct cpif_gpio s5100_gpio_ap_wakeup;
	struct cpif_gpio s5100_gpio_phone_active;

	struct modem_irq s5100_irq_ap_wakeup;
	struct modem_irq s5100_irq_phone_active;

	bool s5100_cp_reset_required;
	bool s5100_iommu_map_enabled;
#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE_S2MPU)
	bool s5100_s2mpu_enabled;
#endif

#if IS_ENABLED(CONFIG_GS_S2MPU)
	struct s2mpu_info *s2mpu;
#endif

	struct notifier_block reboot_nb;
	struct notifier_block pm_notifier;
#endif

	struct notifier_block send_panic_nb;

#if IS_ENABLED(CONFIG_EXYNOS_BUSMONITOR)
	struct notifier_block busmon_nfb;
#endif

#if IS_ENABLED(CONFIG_MUIC_NOTIFIER)
	struct notifier_block uart_notifier;
#endif
	bool uart_connect;
	bool uart_dir;

	const struct attribute_group *group;

	struct delayed_work dwork;
	struct work_struct work;

	struct modemctl_ops ops;
	struct io_device *iod;
	struct io_device *bootd;

#if IS_ENABLED(CONFIG_EXYNOS_ITMON)
	struct notifier_block itmon_nb;
#endif

	void (*gpio_revers_bias_clear)(void);
	void (*gpio_revers_bias_restore)(void);
	void (*modem_complete)(struct modem_ctl *mc);

	int receive_first_ipc;

	struct notifier_block lcd_notifier;

	struct cp_power_stats cp_power_stats;
	spinlock_t power_stats_lock;
};

static inline bool cp_offline(struct modem_ctl *mc)
{
	if (!mc)
		return true;
	return (mc->phone_state == STATE_OFFLINE);
}

static inline bool cp_online(struct modem_ctl *mc)
{
	if (!mc)
		return false;
	return (mc->phone_state == STATE_ONLINE);
}

static inline bool cp_booting(struct modem_ctl *mc)
{
	if (!mc)
		return false;
	return (mc->phone_state == STATE_BOOTING);
}

static inline bool cp_crashed(struct modem_ctl *mc)
{
	if (!mc)
		return false;
	return (mc->phone_state == STATE_CRASH_EXIT
		|| mc->phone_state == STATE_CRASH_WATCHDOG);
}

static inline bool rx_possible(struct modem_ctl *mc)
{
	if (likely(cp_online(mc)))
		return true;

	if (cp_booting(mc) || cp_crashed(mc))
		return true;

	return false;
}

u16 exynos_build_fr_config(struct io_device *iod, struct link_device *ld,
				unsigned int count);
void exynos_build_header(struct io_device *iod, struct link_device *ld,
				u8 *buff, u16 cfg, u8 ctl, size_t count);
u8 sipc5_build_config(struct io_device *iod, struct link_device *ld,
				unsigned int count);
void sipc5_build_header(struct io_device *iod, u8 *buff, u8 cfg,
				unsigned int tx_bytes, unsigned int remains);
void iodev_dump_status(struct io_device *iod, void *args);
void vnet_setup(struct net_device *ndev);
const struct file_operations *get_bootdump_io_fops(void);
const struct file_operations *get_ipc_io_fops(void);
int sipc5_init_io_device(struct io_device *iod);
void sipc5_deinit_io_device(struct io_device *iod);

#if IS_ENABLED(CONFIG_RPS) && IS_ENABLED(CONFIG_ARGOS)
extern struct net init_net;
extern int sec_argos_register_notifier(struct notifier_block *n, char *label);
extern int sec_argos_unregister_notifier(struct notifier_block *n, char *label);
int mif_init_argos_notifier(void);
#else
static inline int mif_init_argos_notifier(void) { return 0; }
#endif

#endif
