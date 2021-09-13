/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2010 Samsung Electronics.
 *
 */

#ifndef __MODEM_LINK_DEVICE_MEMORY_H__
#define __MODEM_LINK_DEVICE_MEMORY_H__

#include <linux/cpumask.h>
#include <linux/freezer.h>
#include <linux/kthread.h>
#include <linux/sched/rt.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/ktime.h>
#include <linux/hrtimer.h>
#include <linux/notifier.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/netdevice.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/debugfs.h>
#include <linux/mcu_ipc.h>
#include <asm/cacheflush.h>

#include "modem_prj.h"
#include "modem_utils.h"
#include "include/circ_queue.h"
#include "include/sbd.h"
#include "include/sipc5.h"
#include "include/legacy.h"
#include "link_rx_pktproc.h"
#if IS_ENABLED(CONFIG_CP_PKTPROC_UL)
#include "link_tx_pktproc.h"
#endif
#include "boot_device_spi.h"
#include "cpif_tp_monitor.h"

/*============================================================================*/
enum mem_iface_type {
	MEM_EXT_DPRAM = 0x0001,	/* External DPRAM */
	MEM_AP_IDPRAM = 0x0002,	/* DPRAM in AP    */
	MEM_CP_IDPRAM = 0x0004,	/* DPRAM in CP    */
	MEM_PLD_DPRAM = 0x0008,	/* PLD or FPGA    */
	MEM_SYS_SHMEM = 0x0100,	/* Shared-memory (SHMEM) on a system bus   */
	MEM_C2C_SHMEM = 0x0200,	/* SHMEM with C2C (Chip-to-chip) interface */
	MEM_LLI_SHMEM = 0x0400,	/* SHMEM with MIPI-LLI interface           */
};

#define MEM_DPRAM_TYPE_MASK	0x00FF
#define MEM_SHMEM_TYPE_MASK	0xFF00

/*============================================================================*/
#define MASK_INT_VALID		0x0080
#define MASK_TX_FLOWCTL_SUSPEND	0x0010
#define MASK_TX_FLOWCTL_RESUME	0x0000

#define MASK_CMD_VALID		0x0040
#define MASK_CMD_FIELD		0x003F

#define MASK_REQ_ACK_FMT	0x0020
#define MASK_REQ_ACK_RAW	0x0010
#define MASK_RES_ACK_FMT	0x0008
#define MASK_RES_ACK_RAW	0x0004
#define MASK_SEND_FMT		0x0002
#define MASK_SEND_RAW		0x0001
#define MASK_SEND_DATA		0x0001

#define CMD_INIT_START		0x0001
#define CMD_INIT_END		0x0002
#define CMD_REQ_ACTIVE		0x0003
#define CMD_RES_ACTIVE		0x0004
#define CMD_REQ_TIME_SYNC	0x0005
#define CMD_KERNEL_PANIC	0x0006
#define CMD_CRASH_RESET		0x0007
#define CMD_PHONE_START		0x0008
#define CMD_CRASH_EXIT		0x0009
#define CMD_CP_DEEP_SLEEP	0x000A
#define CMD_NV_REBUILDING	0x000B
#define CMD_EMER_DOWN		0x000C
#define CMD_PIF_INIT_DONE	0x000D
#define CMD_SILENT_NV_REBUILD	0x000E
#define CMD_NORMAL_POWER_OFF	0x000F

#define DATALLOC_PERIOD_MS		2	/* 2 ms */

/*============================================================================*/
#define MAX_SKB_TXQ_DEPTH		1024
#define TX_PERIOD_MS			1	/* 1 ms */
#define MAX_TX_BUSY_COUNT		1024
#define BUSY_COUNT_MASK			0xF

#define RES_ACK_WAIT_TIMEOUT		10	/* 10 ms */

#define TXQ_STOP_MASK			(0x1<<0)
#define TX_SUSPEND_MASK			(0x1<<1)
#define SHM_FLOWCTL_BIT			BIT(2)

/*============================================================================*/
#define FORCE_CRASH_ACK_TIMEOUT		(5 * HZ)

/*============================================================================*/
#define SHMEM_SRINFO_DATA_STR	64

#if !IS_ENABLED(CONFIG_SBD_BOOTLOG)
#define SHMEM_BOOTLOG_BASE		0xC00
#define SHMEM_BOOTLOG_BUFF		0x1FF
#define SHMEM_BOOTLOG_OFFSET		0x4
#else
#define SHMEM_BOOTSBDLOG_SIZE		0x1000 /* 4KB */
#define SHMEM_BOOTSBDLOG_MAIN_BASE	0x400
#endif

/*============================================================================*/
struct __packed mem_snapshot {
	/* Timestamp */
	struct timespec64 ts;

	/* Direction (TX or RX) */
	enum direction dir;

	/* The status of memory interface at the time */
	unsigned int magic;
	unsigned int access;

	unsigned int head[MAX_SIPC_MAP][MAX_DIR];
	unsigned int tail[MAX_SIPC_MAP][MAX_DIR];

	u16 int2ap;
	u16 int2cp;
};

struct mst_buff {
	/* These two members must be first. */
	struct mst_buff *next;
	struct mst_buff *prev;

	struct mem_snapshot snapshot;
};

struct mst_buff_head {
	/* These two members must be first. */
	struct mst_buff *next;
	struct mst_buff	*prev;

	u32 qlen;
	spinlock_t lock;
};

/*============================================================================*/
enum mem_ipc_mode {
	MEM_LEGACY_IPC,
	MEM_SBD_IPC,
};

#define FREQ_MAX_LV (40)

struct freq_table {
	int num_of_table;
	u32 use_dfs_max_freq;
	u32 cal_id_mif;
	u32 freq[FREQ_MAX_LV];
};

struct ctrl_msg {
	u32 type;
	union {
		u32 sr_num;
		u32 __iomem *addr;
	};
};

struct mem_link_device {
	/**
	 * COMMON and MANDATORY to all link devices
	 */
	struct link_device link_dev;

	/**
	 * Attributes
	 */
	unsigned long attrs;		/* Set of link_attr_bit flags	*/

	/**
	 * Flags
	 */
	bool dpram_magic;		/* DPRAM-style magic code	*/
	bool iosm;			/* IOSM message			*/

	/**
	 * {physical address, size, virtual address} for BOOT region
	 */
	phys_addr_t boot_start;
	size_t boot_size;
	struct page **boot_pages;	/* pointer to the page table for vmap */
	u8 __iomem *boot_base;

	/**
	 * {physical address, size, virtual address} for IPC region
	 */
	phys_addr_t start;
	size_t size;
	struct page **pages;		/* pointer to the page table for vmap */
	u8 __iomem *base;		/* virtual address of ipc mem start */
	u8 __iomem *hiprio_base;	/* virtual address of priority queue start */

	/**
	 * vss region for dump
	 */
	u8 __iomem *vss_base;

	/**
	 * acpm region for dump
	 */
	u8 __iomem *acpm_base;
	int acpm_size;

	/**
	 * CP Binary size for CRC checking
	 */
	u32 cp_binary_size;

	/**
	 * (u32 *) syscp_alive[0] = Magic Code, Version
	 * (u32 *) syscp_alive[1] = CP Reserved Size
	 * (u32 *) syscp_alive[2] = Shared Mem Size
	 */
	struct resource *syscp_info;

	/* Boot link device */
	struct legacy_link_device legacy_link_dev;

	/* sbd link device */
	struct sbd_link_device sbd_link_dev;
	struct work_struct iosm_w;

	/**
	 * GPIO#, MBOX#, IRQ# for IPC
	 */
	unsigned int mbx_cp2ap_msg;	/* MBOX# for IPC RX */
	unsigned int irq_cp2ap_msg;	/* IRQ# for IPC RX  */

	unsigned int sbi_cp2ap_wakelock_mask;
	unsigned int sbi_cp2ap_wakelock_pos;

	unsigned int mbx_ap2cp_msg;	/* MBOX# for IPC TX */
	unsigned int int_ap2cp_msg;	/* INTR# for IPC TX */

	unsigned int sbi_cp_status_mask;
	unsigned int sbi_cp_status_pos;

	unsigned int total_freq_table_count;

	struct freq_table mif_table;
	struct freq_table cp_table;
	struct freq_table modem_table;

	unsigned int irq_cp2ap_wakelock;	/* INTR# for wakelock */

	unsigned int sbi_cp_rat_mode_mask;		/* MBOX# for pcie */
	unsigned int sbi_cp_rat_mode_pos;		/* MBOX# for pcie */
	unsigned int irq_cp2ap_rat_mode;		/* INTR# for pcie */

	unsigned int irq_cp2ap_change_ul_path;

	unsigned int mbx_cp2ap_status;	/* MBOX# for TX FLOWCTL */
	unsigned int irq_cp2ap_status;	/* INTR# for TX FLOWCTL */
	unsigned int tx_flowctrl_cmd;

	struct wakeup_source *ws;

	/**
	 * Member variables for TX & RX
	 */
	struct mst_buff_head msb_rxq;
	struct mst_buff_head msb_log;

	struct hrtimer tx_timer;
	struct hrtimer sbd_tx_timer;
	struct hrtimer sbd_print_timer;
#if IS_ENABLED(CONFIG_CP_PKTPROC_UL)
	struct hrtimer pktproc_tx_timer;
#endif
	struct work_struct page_reclaim_work;

	/**
	 * Member variables for CP booting and crash dump
	 */
	struct delayed_work bootdump_rx_dwork;
	struct std_dload_info img_info;	/* Information of each binary image */
	atomic_t cp_boot_done;

	/**
	 * Mandatory methods for the common memory-type interface framework
	 */
	void (*send_ap2cp_irq)(struct mem_link_device *mld, u16 mask);

	/**
	 * Optional methods for some kind of memory-type interface media
	 */
	u16 (*recv_cp2ap_irq)(struct mem_link_device *mld);
	u16 (*read_ap2cp_irq)(struct mem_link_device *mld);
	u16 (*recv_cp2ap_status)(struct mem_link_device *mld);
	void (*finalize_cp_start)(struct mem_link_device *mld);
	void (*unmap_region)(void *rgn);
	void (*debug_info)(void);
	void (*cmd_handler)(struct mem_link_device *mld, u16 cmd);

#ifdef DEBUG_MODEM_IF
	/* for logging MEMORY dump */
	struct work_struct dump_work;
	char dump_path[MIF_MAX_PATH_LEN];
#endif

#ifdef DEBUG_MODEM_IF
	struct dentry *dbgfs_dir;
	struct debugfs_blob_wrapper mem_dump_blob;
	struct dentry *dbgfs_frame;
#endif
	unsigned int tx_period_ms;
	unsigned int force_use_memcpy;
	unsigned int memcpy_packet_count;
	unsigned int zeromemcpy_packet_count;

	atomic_t forced_cp_crash;
	struct timer_list crash_ack_timer;

	spinlock_t state_lock;
	enum link_state state;

	struct net_device dummy_net;
	struct napi_struct mld_napi;
	atomic_t stop_napi_poll;
	unsigned int rx_int_enable;
	unsigned int rx_int_count;
	unsigned int rx_poll_count;
	unsigned long long rx_int_disabled_time;

	/* Doorbell interrupt value to separate interrupt */
	unsigned int intval_ap2cp_msg;
	unsigned int intval_ap2cp_status;
	unsigned int intval_ap2cp_active;

	/* Location for arguments in shared memory */
	u32 __iomem *ap_version;
	u32 __iomem *cp_version;
	u32 __iomem *cmsg_offset;	/* address where cmsg offset is written */
	u32 __iomem *srinfo_offset;
	u32 __iomem *clk_table_offset;
	u32 __iomem *buff_desc_offset;
	u32 __iomem *capability_offset;

	u32 __iomem *ap_capability_0_offset;
	u32 __iomem *cp_capability_0_offset;
	u32 __iomem *ap_capability_1_offset;
	u32 __iomem *cp_capability_1_offset;

	/* Location for control messages in shared memory */
	struct ctrl_msg ap2cp_msg;
	struct ctrl_msg cp2ap_msg;
	struct ctrl_msg ap2cp_united_status;
	struct ctrl_msg cp2ap_united_status;
	struct ctrl_msg ap2cp_kerneltime;	/* for DRAM_V1 and MAILBOX_SR */
	struct ctrl_msg ap2cp_kerneltime_sec;	/* for DRAM_V2 */
	struct ctrl_msg ap2cp_kerneltime_usec;	/* for DRAM_V2 */
	struct ctrl_msg ap2cp_handover_block_info;

	u32 __iomem *doorbell_addr;
	struct pci_dev *s51xx_pdev;

	int msi_irq_base;
	int msi_irq_base_enabled;

	u32 __iomem *srinfo_base;
	u32 srinfo_size;
	u32 __iomem *clk_table;

	u32 ap_capability_0;
	u32 cp_capability_0;
	u32 ap_capability_1;
	u32 cp_capability_1;

	int (*pass_skb_to_net)(struct mem_link_device *mld, struct sk_buff *skb);

	struct pktproc_adaptor pktproc;
#if IS_ENABLED(CONFIG_CP_PKTPROC_UL)
	struct pktproc_adaptor_ul pktproc_ul;
#endif

	struct cpboot_spi *boot_spi;

	struct cpif_tpmon *tpmon;
};

#define to_mem_link_device(ld) \
		container_of(ld, struct mem_link_device, link_dev)
#define ld_to_mem_link_device(ld) \
		container_of(ld, struct mem_link_device, link_dev)
#define sbd_to_mem_link_device(sl) \
		container_of(sl, struct mem_link_device, sbd_link_dev)

#define MEM_IPC_MAGIC		0xAA
#define MEM_CRASH_MAGIC		0xDEADDEAD
#define MEM_BOOT_MAGIC		0x424F4F54
#define MEM_DUMP_MAGIC		0x44554D50

#define MAX_TABLE_COUNT 8

struct clock_table_info {
	char table_name[4];
	u32 table_count;
};

struct clock_table {
	char parser_version[4];
	u32 total_table_count;
	struct clock_table_info table_info[MAX_TABLE_COUNT];
};

/*============================================================================*/
static inline bool mem_type_shmem(enum mem_iface_type type)
{
	return (type & MEM_SHMEM_TYPE_MASK) ? true : false;
}

/*============================================================================*/
static inline bool int_valid(u16 x)
{
	return (x & MASK_INT_VALID) ? true : false;
}

static inline u16 mask2int(u16 mask)
{
	return mask | MASK_INT_VALID;
}

/*
 * @remark		This must be invoked after validation with int_valid().
 */
static inline bool cmd_valid(u16 x)
{
	return (x & MASK_CMD_VALID) ? true : false;

}

static inline bool chk_same_cmd(struct mem_link_device *mld, u16 x)
{
	if (mld->tx_flowctrl_cmd != x) {
		mld->tx_flowctrl_cmd = x;
		return false;
	}

	return true;
}

static inline u16 int2cmd(u16 x)
{
	return x & MASK_CMD_FIELD;
}

static inline u16 cmd2int(u16 cmd)
{
	return mask2int(cmd | MASK_CMD_VALID);
}

/*============================================================================*/
static inline struct circ_queue *cq(struct legacy_ipc_device *dev,
				    enum direction dir)
{
	return (dir == TX) ? &dev->txq : &dev->rxq;
}

static inline unsigned int get_txq_head(struct legacy_ipc_device *dev)
{
	return get_head(&dev->txq);
}

static inline void set_txq_head(struct legacy_ipc_device *dev, unsigned int in)
{
	set_head(&dev->txq, in);
}

static inline unsigned int get_txq_tail(struct legacy_ipc_device *dev)
{
	return get_tail(&dev->txq);
}

static inline void set_txq_tail(struct legacy_ipc_device *dev, unsigned int out)
{
	set_tail(&dev->txq, out);
}

static inline char *get_txq_buff(struct legacy_ipc_device *dev)
{
	return get_buff(&dev->txq);
}

static inline unsigned int get_txq_buff_size(struct legacy_ipc_device *dev)
{
	return get_size(&dev->txq);
}

static inline unsigned int get_rxq_head(struct legacy_ipc_device *dev)
{
	return get_head(&dev->rxq);
}

static inline void set_rxq_head(struct legacy_ipc_device *dev, unsigned int in)
{
	set_head(&dev->rxq, in);
}

static inline unsigned int get_rxq_tail(struct legacy_ipc_device *dev)
{
	return get_tail(&dev->rxq);
}

static inline void set_rxq_tail(struct legacy_ipc_device *dev, unsigned int out)
{
	set_tail(&dev->rxq, out);
}

static inline char *get_rxq_buff(struct legacy_ipc_device *dev)
{
	return get_buff(&dev->rxq);
}

static inline unsigned int get_rxq_buff_size(struct legacy_ipc_device *dev)
{
	return get_size(&dev->rxq);
}

static inline u16 msg_mask(struct legacy_ipc_device *dev)
{
	return dev->msg_mask;
}

static inline u16 req_ack_mask(struct legacy_ipc_device *dev)
{
	return dev->req_ack_mask;
}

static inline u16 res_ack_mask(struct legacy_ipc_device *dev)
{
	return dev->res_ack_mask;
}

static inline bool req_ack_valid(struct legacy_ipc_device *dev, u16 val)
{
	if (!cmd_valid(val) && (val & req_ack_mask(dev)))
		return true;
	else
		return false;
}

static inline bool res_ack_valid(struct legacy_ipc_device *dev, u16 val)
{
	if (!cmd_valid(val) && (val & res_ack_mask(dev)))
		return true;
	else
		return false;
}

static inline bool rxq_empty(struct legacy_ipc_device *dev)
{
	u32 head;
	u32 tail;
	unsigned long flags;

	spin_lock_irqsave(&dev->rxq.lock, flags);

	head = get_rxq_head(dev);
	tail = get_rxq_tail(dev);

	spin_unlock_irqrestore(&dev->rxq.lock, flags);

	return circ_empty(head, tail);
}

static inline bool txq_empty(struct legacy_ipc_device *dev)
{
	u32 head;
	u32 tail;
	unsigned long flags;

	spin_lock_irqsave(&dev->txq.lock, flags);

	head = get_txq_head(dev);
	tail = get_txq_tail(dev);

	spin_unlock_irqrestore(&dev->txq.lock, flags);

	return circ_empty(head, tail);
}

static inline int construct_ctrl_msg(struct ctrl_msg *cmsg, u32 *arr_from_dt,
					u8 __iomem *base)
{
	if (!cmsg)
		return -EINVAL;

	cmsg->type = arr_from_dt[0];
	switch (cmsg->type) {
	case MAILBOX_SR:
		cmsg->sr_num = arr_from_dt[1];
		break;
	case DRAM_V1:
	case DRAM_V2:
		cmsg->addr = (u32 __iomem *)(base + arr_from_dt[1]);
		break;
	default:
		mif_err("ERR! wrong type for ctrl msg\n");
		return -EINVAL;
	}

	return 0;
}

static inline void init_ctrl_msg(struct ctrl_msg *cmsg)
{
	switch (cmsg->type) {
	case MAILBOX_SR:
		/* nothing to do */
		break;
	case DRAM_V1:
	case DRAM_V2:
		*cmsg->addr = 0;
		break;
	case GPIO:
		break;
	default:
		break;
	}

}
static inline u32 get_ctrl_msg(struct ctrl_msg *cmsg)
{
	u32 val = 0;

	switch (cmsg->type) {
	case MAILBOX_SR:
#if IS_ENABLED(CONFIG_MCU_IPC)
		val = cp_mbox_get_sr(cmsg->sr_num);
#endif
		break;
	case DRAM_V1:
	case DRAM_V2:
		val = ioread32(cmsg->addr);
		break;
	case GPIO:
		break;
	default:
		break;
	}

	return val;
}

static inline void set_ctrl_msg(struct ctrl_msg *cmsg, u32 msg)
{
	switch (cmsg->type) {
	case MAILBOX_SR:
#if IS_ENABLED(CONFIG_MCU_IPC)
		cp_mbox_set_sr(cmsg->sr_num, msg);
#endif
		break;
	case DRAM_V1:
	case DRAM_V2:
		iowrite32(msg, cmsg->addr);
		break;
	case GPIO:
		break;
	default:
		break;
	}
}

static inline u32 extract_ctrl_msg(struct ctrl_msg *cmsg, u32 mask, u32 pos)
{
	u32 val = 0;

	switch (cmsg->type) {
	case MAILBOX_SR:
#if IS_ENABLED(CONFIG_MCU_IPC)
		val = cp_mbox_extract_sr(cmsg->sr_num, mask, pos);
#endif
		break;
	case DRAM_V1:
	case DRAM_V2:
		val = (ioread32(cmsg->addr) >> pos) & mask;
		break;
	case GPIO:
		break;
	default:
		break;
	}

	return val;
}

static inline void update_ctrl_msg(struct ctrl_msg *cmsg, u32 msg, u32 mask, u32 pos)
{
	u32 val = 0;

	switch (cmsg->type) {
	case MAILBOX_SR:
#if IS_ENABLED(CONFIG_MCU_IPC)
		cp_mbox_update_sr(cmsg->sr_num, msg, mask, pos);
#endif
		break;
	case DRAM_V1:
	case DRAM_V2:
		val = ioread32(cmsg->addr);
		val &= ~(mask << pos);
		val |= (msg & mask) << pos;
		iowrite32(val, cmsg->addr);
		break;
	case GPIO:
		break;
	default:
		break;
	}
}

/*============================================================================*/
int msb_init(void);

struct mst_buff *msb_alloc(void);
void msb_free(struct mst_buff *msb);

void msb_queue_head_init(struct mst_buff_head *list);
void msb_queue_tail(struct mst_buff_head *list, struct mst_buff *msb);
void msb_queue_head(struct mst_buff_head *list, struct mst_buff *msb);

struct mst_buff *msb_dequeue(struct mst_buff_head *list);

void msb_queue_purge(struct mst_buff_head *list);

struct mst_buff *mem_take_snapshot(struct mem_link_device *mld,
				   enum direction dir);

/*============================================================================*/
static inline void send_ipc_irq(struct mem_link_device *mld, u16 val)
{
	if (likely(mld->send_ap2cp_irq))
		mld->send_ap2cp_irq(mld, val);
}

void mem_irq_handler(struct mem_link_device *mld, struct mst_buff *msb);

/*============================================================================*/
void __iomem *mem_vmap(phys_addr_t pa, size_t size, struct page *pages[]);
void mem_vunmap(void *va);

int mem_register_boot_rgn(struct mem_link_device *mld, phys_addr_t start,
			  size_t size);
void mem_unregister_boot_rgn(struct mem_link_device *mld);
int mem_setup_boot_map(struct mem_link_device *mld);

int mem_register_ipc_rgn(struct mem_link_device *mld, phys_addr_t start,
			 size_t size);
void mem_unregister_ipc_rgn(struct mem_link_device *mld);
void mem_setup_ipc_map(struct mem_link_device *mld);

struct mem_link_device *mem_create_link_device(enum mem_iface_type type,
					       struct modem_data *modem);

/*============================================================================*/
int mem_reset_ipc_link(struct mem_link_device *mld);
void mem_cmd_handler(struct mem_link_device *mld, u16 cmd);

/*============================================================================*/
void sbd_txq_stop(struct sbd_ring_buffer *rb);
void sbd_txq_start(struct sbd_ring_buffer *rb);

int sbd_under_tx_flow_ctrl(struct sbd_ring_buffer *rb);
int sbd_check_tx_flow_ctrl(struct sbd_ring_buffer *rb);

#if IS_ENABLED(CONFIG_CP_PKTPROC_UL)
void pktproc_ul_q_stop(struct pktproc_queue_ul *q);
void pktproc_ul_q_start(struct pktproc_queue_ul *q);
int pktproc_under_ul_flow_ctrl(struct pktproc_queue_ul *q);
int pktproc_check_ul_flow_ctrl(struct pktproc_queue_ul *q);
#endif

void tx_flowctrl_suspend(struct mem_link_device *mld);
void tx_flowctrl_resume(struct mem_link_device *mld);
void txq_stop(struct mem_link_device *mld, struct legacy_ipc_device *dev);
void txq_start(struct mem_link_device *mld, struct legacy_ipc_device *dev);

int under_tx_flow_ctrl(struct mem_link_device *mld, struct legacy_ipc_device *dev);
int check_tx_flow_ctrl(struct mem_link_device *mld, struct legacy_ipc_device *dev);

void send_req_ack(struct mem_link_device *mld, struct legacy_ipc_device *dev);
void recv_res_ack(struct mem_link_device *mld, struct legacy_ipc_device *dev,
		  struct mem_snapshot *mst);

void recv_req_ack(struct mem_link_device *mld, struct legacy_ipc_device *dev,
		  struct mem_snapshot *mst);
void send_res_ack(struct mem_link_device *mld, struct legacy_ipc_device *dev);

/*============================================================================*/
void mem_handle_cp_crash(struct mem_link_device *mld, enum modem_state state);
void mem_forced_cp_crash(struct mem_link_device *mld);

/*============================================================================*/
void print_req_ack(struct mem_link_device *mld, struct mem_snapshot *mst,
		   struct legacy_ipc_device *dev, enum direction dir);
void print_res_ack(struct mem_link_device *mld, struct mem_snapshot *mst,
		   struct legacy_ipc_device *dev, enum direction dir);

void print_mem_snapshot(struct mem_link_device *mld, struct mem_snapshot *mst);
void print_dev_snapshot(struct mem_link_device *mld, struct mem_snapshot *mst,
			struct legacy_ipc_device *dev);

static inline struct sk_buff *mem_alloc_skb(unsigned int len)
{
	gfp_t priority;
	struct sk_buff *skb;

	priority = in_interrupt() ? GFP_ATOMIC : GFP_KERNEL;

	skb = alloc_skb(len + NET_SKB_PAD, priority);
	if (!skb) {
		mif_err("ERR! alloc_skb(len:%d + pad:%d, gfp:0x%x) fail\n",
			len, NET_SKB_PAD, priority);
#if IS_ENABLED(CONFIG_SEC_DEBUG_MIF_OOM)
		show_mem(SHOW_MEM_FILTER_NODES);
#endif
		return NULL;
	}

	skb_reserve(skb, NET_SKB_PAD);
	return skb;
}

/*============================================================================*/
/* direction: CP -> AP */
#define IOSM_C2A_MDM_READY	0x80
#define IOSM_C2A_CONF_CH_RSP	0xA3	/* answer of flow control msg */
#define IOSM_C2A_STOP_TX_CH	0xB0
#define IOSM_C2A_START_TX_CH	0xB1
#define IOSM_C2A_ACK		0xE0
#define IOSM_C2A_NACK		0xE1

/* direction: AP -> CP */
#define IOSM_A2C_AP_READY	0x00
#define IOSM_A2C_CONF_CH_REQ	0x22	/* flow control on/off */
#define IOSM_A2C_OPEN_CH	0x24
#define IOSM_A2C_CLOSE_CH	0x25
#define IOSM_A2C_STOP_TX_CH	0x30
#define IOSM_A2C_START_TX_CH	0x30
#define IOSM_A2C_ACK		0x60
#define IOSM_A2C_NACK		0x61

#define IOSM_TRANS_ID_MAX	255
#define IOSM_MSG_AREA_SIZE	(CTRL_RGN_SIZE / 2)
#define IOSM_MSG_TX_OFFSET	CMD_RGN_OFFSET
#define IOSM_MSG_RX_OFFSET	(CMD_RGN_OFFSET + IOSM_MSG_AREA_SIZE)
#define IOSM_MSG_DESC_OFFSET	(CMD_RGN_OFFSET + CMD_RGN_SIZE)

void tx_iosm_message(struct mem_link_device *mld, u8 id, u32 *args);
void iosm_event_work(struct work_struct *work);
void iosm_event_bh(struct mem_link_device *mld, u16 cmd);

#define NET_HEADROOM (NET_SKB_PAD + NET_IP_ALIGN)

#if IS_ENABLED(CONFIG_USB_ANDROID_SAMSUNG_COMPOSITE)
extern int is_rndis_use(void);
#endif

#endif /* __MODEM_LINK_DEVICE_MEMORY_H__ */
