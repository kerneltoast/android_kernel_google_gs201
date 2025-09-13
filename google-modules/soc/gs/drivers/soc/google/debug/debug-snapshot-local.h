/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 */

#ifndef DEBUG_SNAPSHOT_LOCAL_H
#define DEBUG_SNAPSHOT_LOCAL_H
#include <linux/device.h>
#include <soc/google/debug-snapshot-log.h>

struct dbg_snapshot_param {
	void *dss_log_misc;
	void *dss_items;
	void *dss_log_items;
	void *dss_log;
	void *hook_func;
};

struct dbg_snapshot_info {
	size_t size;
	phys_addr_t vaddr;
	phys_addr_t paddr;
	unsigned int enabled;
};

struct dbg_snapshot_base {
	size_t size;
	phys_addr_t vaddr;
	phys_addr_t paddr;
	unsigned int enabled;
	unsigned int version;
	struct dbg_snapshot_param *param;
};

struct dbg_snapshot_item {
	char *name;
	struct dbg_snapshot_info entry;
	unsigned int persist;
	unsigned char *head_ptr;
	unsigned char *curr_ptr;
};

struct dbg_snapshot_log_item {
	char *name;
	struct dbg_snapshot_info entry;
};

struct dbg_snapshot_desc {
	struct device *dev;
	raw_spinlock_t ctrl_lock;
	int sjtag_status;
	bool in_reboot;
	bool in_panic;
	bool in_warm;
	int panic_action;
};

struct dbg_snapshot_dpm {
	bool enabled;
	unsigned int version;
	bool enabled_debug;
	unsigned int dump_mode;
	int dump_mode_none;
	bool dump_mode_file;
	bool enabled_debug_kinfo;

	unsigned int pre_log;
	unsigned int p_el1_da;
	unsigned int p_el1_sp_pc;
	unsigned int p_el1_ia;
	unsigned int p_el1_undef;
	unsigned int p_el1_inv;
	unsigned int p_el1_serror;
};

void dbg_snapshot_register_vh_log(void);
extern void dbg_snapshot_init_log(void);
extern void dbg_snapshot_init_dpm(void);
extern void dbg_snapshot_init_utils(void);
extern void dbg_snapshot_start_log(void);
extern int dbg_snapshot_dt_scan_dpm(void);
extern int dbg_snapshot_get_enable(void);
extern void __iomem *dbg_snapshot_get_header_vaddr(void);
extern void dbg_snapshot_scratch_reg(unsigned int val);
extern void dbg_snapshot_print_log_report(void);
extern void dbg_snapshot_set_debug_test_buffer_addr(u64 paddr, unsigned int cpu);
extern unsigned int dbg_snapshot_get_debug_test_buffer_addr(unsigned int cpu);
extern void dbg_snapshot_set_qd_entry(unsigned long address);
extern int dbg_snapshot_get_num_items(void);
extern int dbg_snapshot_log_get_num_items(void);
extern struct dbg_snapshot_item *dbg_snapshot_get_item_by_index(int index);
extern struct dbg_snapshot_log_item *dbg_snapshot_log_get_item_by_index(int index);
extern void dbg_snapshot_set_enable_log_item(const char *name, int en);
extern int dbg_snapshot_get_dpm_none_dump_mode(void);
extern void dbg_snapshot_set_dpm_none_dump_mode(unsigned int mode);
extern void dss_flush_cache_all(void);

extern struct dbg_snapshot_log *dss_log;
extern struct dbg_snapshot_desc dss_desc;
extern struct dbg_snapshot_item dss_items[];
extern struct dbg_snapshot_dpm dss_dpm;
extern struct dbg_snapshot_log_item dss_log_items[];
extern struct dbg_snapshot_log_misc dss_log_misc;
extern struct itmon_logs *dss_itmon;

/* Sign domain */
#define DSS_SIGN_RESET			0x0
#define DSS_SIGN_RESERVED		0x1
#define DSS_SIGN_SCRATCH		0xD
#define DSS_SIGN_ALIVE			0xFACE
#define DSS_SIGN_DEAD			0xDEAD
#define DSS_SIGN_PANIC			0xBABA
#define DSS_SIGN_UNKNOWN_REBOOT		0xCACA
#define DSS_SIGN_EMERGENCY_REBOOT	0xCACB
#define DSS_SIGN_WARM_REBOOT		0xCACC
#define DSS_SIGN_SAFE_FAULT		0xFAFA
#define DSS_SIGN_NORMAL_REBOOT		0xCAFE
#define DSS_SIGN_LOCKUP			0xDEADBEEF
#define DSS_SIGN_MAGIC			(0xDB9 << 16)
#define DSS_BOOT_CNT_MAGIC		0xFACEDB90
#define DSS_SLCDUMP_MAGIC		0x1337CACE

#define ARM_CPU_PART_CORTEX_A78		0xD41
#define ARM_CPU_PART_CORTEX_X1		0xD44
#endif
