/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2012 Samsung Electronics.
 *
 */

#ifndef __MODEM_V1_H__
#define __MODEM_V1_H__

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/types.h>
#include <dt-bindings/soc/google/exynos-cpif.h>
#include <linux/shm_ipc.h>

#include "cp_btl.h"

#define MAX_STR_LEN		256
#define MAX_NAME_LEN		64
#define MAX_DUMP_LEN		20

#define SMC_ID		0x82000700
#define SMC_ID_CLK	0x82001011
#define SSS_CLK_ENABLE	0
#define SSS_CLK_DISABLE	1

struct __packed multi_frame_control {
	u8 id:7,
	   more:1;
};

enum direction {
	TX = 0,
	UL = 0,
	AP2CP = 0,
	RX = 1,
	DL = 1,
	CP2AP = 1,
	TXRX = 2,
	ULDL = 2,
	MAX_DIR = 2
};

enum read_write {
	RD = 0,
	WR = 1,
	RDWR = 2
};

/**
 * struct modem_io_t - declaration for io_device
 * @name:	device name
 * @id:		for SIPC4, contains format & channel information
 *		(id & 11100000b)>>5 = format  (eg, 0=FMT, 1=RAW, 2=RFS)
 *		(id & 00011111b)    = channel (valid only if format is RAW)
 *		for SIPC5, contains only 8-bit channel ID
 * @format:	device format
 * @io_type:	type of this io_device
 * @link_type:	link_devices to use this io_device
 *		for example, LINKDEV_SHMEM or LINKDEV_PCIE
 */
struct modem_io_t {
	char name[SZ_64];
	u32 ch;
	u32 ch_count;
	u32 format;
	u32 io_type;
	u32 link_type;
	u32 attrs;
	char *option_region;
	unsigned int ul_num_buffers;
	unsigned int ul_buffer_size;
	unsigned int dl_num_buffers;
	unsigned int dl_buffer_size;
};

#if IS_ENABLED(CONFIG_LINK_DEVICE_SHMEM) || IS_ENABLED(CONFIG_LINK_DEVICE_PCIE)
struct modem_mbox {
	unsigned int int_ap2cp_msg;
	unsigned int int_ap2cp_active;
	unsigned int int_ap2cp_wakeup;
	unsigned int int_ap2cp_status;
#if IS_ENABLED(CONFIG_CP_LCD_NOTIFIER)
	unsigned int int_ap2cp_lcd_status;
#endif
#if IS_ENABLED(CONFIG_CP_LLC)
	unsigned int int_ap2cp_llc_status;
#endif
#if IS_ENABLED(CONFIG_CP_PKTPROC_CLAT)
	unsigned int int_ap2cp_clatinfo_send;
#endif
#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE)
	unsigned int int_ap2cp_pcie_link_ack;
#endif
	unsigned int int_ap2cp_uart_noti;

	unsigned int irq_cp2ap_msg;
	unsigned int irq_cp2ap_status;
	unsigned int irq_cp2ap_active;
#if IS_ENABLED(CONFIG_CP_LLC)
	unsigned int irq_cp2ap_llc_status;
#endif
#if IS_ENABLED(CONFIG_CP_PKTPROC_CLAT)
	unsigned int irq_cp2ap_clatinfo_ack;
#endif
	unsigned int irq_cp2ap_wakelock;
	unsigned int irq_cp2ap_rat_mode;
};
#endif

#define AP_CP_CAP_PARTS		2
#define AP_CP_CAP_PART_LEN	4
#define AP_CP_CAP_BIT_MAX	32

/* AP capability[0] index */
enum ap_capability_0_bits {
	AP_CAP_0_PKTPROC_UL_BIT = 0,
	AP_CAP_0_CH_EXTENSION_BIT,
	AP_CAP_0_PKTPROC_36BIT_ADDR_BIT,
	AP_CAP_0_MAX = AP_CP_CAP_BIT_MAX
};

/* AP capability[1] index */
enum ap_capability_1_bits {
	AP_CAP_1_MAX = AP_CP_CAP_BIT_MAX
};

/* platform data */
struct modem_data {
	char *name;
	u32 cp_num;

	struct modem_mbox *mbx;
	struct mem_link_device *mld;

	/* Modem component */
	u32 modem_type;

	u32 link_type;
	char *link_name;
	unsigned long link_attrs;	/* Set of link_attr_bit flags	*/
	u32 interrupt_types;

	u32 protocol;

	/* SIPC version */
	u32 ipc_version;

	/* Information of IO devices */
	unsigned int num_iodevs;
	struct modem_io_t *iodevs[IOD_CH_ID_MAX];

	/* capability check */
	u32 capability_check;

	/* check if cp2ap_active is in alive */
	u32 cp2ap_active_not_alive;

	/* MIF will be off during VoLTE call and we need to register wrstbi interrupt */
	u32 mif_off_during_volte;

	/* legacy buffer setting */
	u32 legacy_fmt_head_tail_offset;
	u32 legacy_fmt_buffer_offset;
	u32 legacy_fmt_txq_size;
	u32 legacy_fmt_rxq_size;
	u32 legacy_raw_head_tail_offset;
	u32 legacy_raw_buffer_offset;
	u32 legacy_raw_txq_size;
	u32 legacy_raw_rxq_size;
	u32 legacy_raw_rx_buffer_cached;

	/* several 4 byte length info in ipc region */
	u32 offset_ap_version;
	u32 offset_cp_version;
	u32 offset_cmsg_offset;
	u32 offset_srinfo_offset;
	u32 offset_clk_table_offset;
	u32 offset_buff_desc_offset;
	u32 offset_capability_offset;

	/* ctrl messages between cp and ap */
	u32 ap2cp_msg[2];
	u32 cp2ap_msg[2];
	u32 cp2ap_united_status[2];
	u32 ap2cp_united_status[2];
#if IS_ENABLED(CONFIG_CP_LLC)
	u32 ap2cp_llc_status[2];
	u32 cp2ap_llc_status[2];
#endif
#if IS_ENABLED(CONFIG_CP_PKTPROC_CLAT)
	u32 ap2cp_clatinfo_xlat_v4_addr[2];
	u32 ap2cp_clatinfo_xlat_addr_0[2];
	u32 ap2cp_clatinfo_xlat_addr_1[2];
	u32 ap2cp_clatinfo_xlat_addr_2[2];
	u32 ap2cp_clatinfo_xlat_addr_3[2];
	u32 ap2cp_clatinfo_index[2];
#endif
	u32 ap2cp_kerneltime[2];
	u32 ap2cp_kerneltime_sec[2];
	u32 ap2cp_kerneltime_usec[2];
	u32 ap2cp_handover_block_info[2];

	/* Status Bit Info */
	unsigned int sbi_lte_active_mask;
	unsigned int sbi_lte_active_pos;
	unsigned int sbi_cp_status_mask;
	unsigned int sbi_cp_status_pos;
	unsigned int sbi_cp2ap_wakelock_mask;
	unsigned int sbi_cp2ap_wakelock_pos;
	unsigned int sbi_cp2ap_rat_mode_mask;
	unsigned int sbi_cp2ap_rat_mode_pos;

	unsigned int sbi_pda_active_mask;
	unsigned int sbi_pda_active_pos;
	unsigned int sbi_ap_status_mask;
	unsigned int sbi_ap_status_pos;

	unsigned int sbi_ap2cp_kerneltime_sec_mask;
	unsigned int sbi_ap2cp_kerneltime_sec_pos;
	unsigned int sbi_ap2cp_kerneltime_usec_mask;
	unsigned int sbi_ap2cp_kerneltime_usec_pos;

	unsigned int sbi_uart_noti_mask;
	unsigned int sbi_uart_noti_pos;
	unsigned int sbi_crash_type_mask;
	unsigned int sbi_crash_type_pos;
	unsigned int sbi_ds_det_mask;
	unsigned int sbi_ds_det_pos;
#if IS_ENABLED(CONFIG_CP_LCD_NOTIFIER)
	unsigned int sbi_lcd_status_mask;
	unsigned int sbi_lcd_status_pos;
#endif

	/* ulpath offset for 2CP models */
	u32 ulpath_offset;

	/* control message offset */
	u32 cmsg_offset;

	/* srinfo settings */
	u32 srinfo_offset;
	u32 srinfo_size;

	/* clk_table offset */
	u32 clk_table_offset;

	/* new SIT buffer descriptor offset */
	u32 buff_desc_offset;

	/* capability */
	u32 capability_offset;
	u32 ap_capability[AP_CP_CAP_PARTS];

#if IS_ENABLED(CONFIG_MODEM_IF_LEGACY_QOS)
	/* SIT priority queue info */
	u32 legacy_raw_qos_head_tail_offset;
	u32 legacy_raw_qos_buffer_offset;
	u32 legacy_raw_qos_txq_size;
	u32 legacy_raw_qos_rxq_size; /* unused for now */
#endif
	struct cp_btl btl;	/* CP background trace log */

	u32 pktproc_use_36bit_addr; /* Check pktproc use 36bit addr */
};

enum cp_gpio_type {
	CP_GPIO_AP2CP_CP_PWR,
	CP_GPIO_AP2CP_NRESET,
	CP_GPIO_AP2CP_WAKEUP,
	CP_GPIO_AP2CP_DUMP_NOTI,
	CP_GPIO_AP2CP_AP_ACTIVE,
#if !IS_ENABLED(CONFIG_CP_WRESET_WA)
	CP_GPIO_AP2CP_CP_WRST_N,
	CP_GPIO_CP2AP_CP_WRST_N,
	CP_GPIO_AP2CP_PM_WRST_N,
#endif
	CP_GPIO_CP2AP_PS_HOLD,
	CP_GPIO_CP2AP_WAKEUP,
	CP_GPIO_CP2AP_CP_ACTIVE,
	CP_GPIO_MAX
};

enum cp_gpio_irq_type {
	CP_GPIO_IRQ_NONE,
	CP_GPIO_IRQ_CP2AP_WAKEUP,
	CP_GPIO_IRQ_CP2AP_CP_ACTIVE,
	CP_GPIO_IRQ_CP2AP_CP_WRST_N,
	CP_GPIO_IRQ_MAX
};

struct modem_irq {
	spinlock_t lock;
	unsigned int num;
	char name[MAX_NAME_LEN];
	unsigned long flags;
	bool active;
	bool registered;
	u32 not_alive;
};

struct cpif_gpio {
	bool valid;
	int num;
	enum cp_gpio_irq_type irq_type;
	const char *label;
	const char *node_name;
};

#define mif_dt_read_enum(np, prop, dest) \
	do { \
		u32 val; \
		if (of_property_read_u32(np, prop, &val)) { \
			mif_err("%s is not defined\n", prop); \
			return -EINVAL; \
		} \
		dest = (__typeof__(dest))(val); \
	} while (0)

#define mif_dt_read_bool(np, prop, dest) \
	do { \
		u32 val; \
		if (of_property_read_u32(np, prop, &val)) { \
			mif_err("%s is not defined\n", prop); \
			return -EINVAL; \
		} \
		dest = val ? true : false; \
	} while (0)

#define mif_dt_read_string(np, prop, dest) \
	do { \
		if (of_property_read_string(np, prop, \
				(const char **)&dest)) { \
			mif_err("%s is not defined\n", prop); \
			return -EINVAL; \
		} \
	} while (0)

#define mif_dt_read_u32(np, prop, dest) \
	do { \
		u32 val; \
		if (of_property_read_u32(np, prop, &val)) { \
			mif_err("%s is not defined\n", prop); \
			return -EINVAL; \
		} \
		dest = val; \
	} while (0)

#define mif_dt_read_u32_noerr(np, prop, dest) \
	do { \
		u32 val; \
		if (!of_property_read_u32(np, prop, &val)) \
			dest = val; \
	} while (0)

#define mif_dt_read_u64(np, prop, dest) \
	do { \
		u64 val; \
		if (of_property_read_u64(np, prop, &val)) { \
			mif_err("%s is not defined\n", prop); \
			return -EINVAL; \
		} \
		dest = val; \
	} while (0)

#define mif_dt_read_u64_noerr(np, prop, dest) \
	do { \
		u64 val; \
		if (!of_property_read_u64(np, prop, &val)) \
			dest = val; \
	} while (0)

#define mif_dt_count_u32_elems(np, prop, dest) \
	do { \
		int val; \
		val = of_property_count_u32_elems(np, prop); \
		if (val < 0) { \
			mif_err("can not get %s\n", prop); \
			return -EINVAL; \
		} \
		dest = (u32)val; \
	} while (0)

#define mif_dt_count_u32_array(np, prop, dest, size) \
	do { \
		int val; \
		val = of_property_read_u32_array(np, prop, dest, size); \
		if (val < 0) { \
			mif_err("can not get %s %d\n", prop, size); \
			return -EINVAL; \
		} \
	} while (0)


#define cpif_set_bit(data, offset)	((data) |= BIT(offset))
#define cpif_clear_bit(data, offset)	((data) &= ~BIT(offset))
#define cpif_check_bit(data, offset)	((data) & BIT(offset))

#define LOG_TAG	"cpif: "
#define CALLER	(__builtin_return_address(0))

#define mif_err_limited(fmt, ...) \
	printk_ratelimited(KERN_ERR LOG_TAG "%s: " pr_fmt(fmt), __func__, ##__VA_ARGS__)
#define mif_err(fmt, ...) \
	pr_err(LOG_TAG "%s: " pr_fmt(fmt), __func__, ##__VA_ARGS__)

#define mif_info_limited(fmt, ...) \
	printk_ratelimited(KERN_INFO LOG_TAG "%s: " pr_fmt(fmt), __func__, ##__VA_ARGS__)
#define mif_info(fmt, ...) \
	pr_info(LOG_TAG "%s: " pr_fmt(fmt), __func__, ##__VA_ARGS__)

#define mif_debug(fmt, ...) \
	pr_debug(LOG_TAG "%s: " pr_fmt(fmt), __func__, ##__VA_ARGS__)

#define mif_trace(fmt, ...) \
	printk(KERN_DEBUG "cpif: %s: %d: called(%pF): " fmt, \
		__func__, __LINE__, __builtin_return_address(0), ##__VA_ARGS__)

#endif
