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
	size_t vaddr;
	size_t paddr;
	unsigned int enabled;
};

struct dbg_snapshot_base {
	size_t size;
	size_t vaddr;
	size_t paddr;
	unsigned int enabled;
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

struct dbg_snapshot_suspend_diag_item {
	const char *action;
	unsigned long long timeout;
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
extern void flush_cache_all(void);

extern struct dbg_snapshot_log *dss_log;
extern struct dbg_snapshot_desc dss_desc;
extern struct dbg_snapshot_item dss_items[];
extern struct dbg_snapshot_dpm dss_dpm;
extern struct dbg_snapshot_log_item dss_log_items[];
extern struct dbg_snapshot_log_misc dss_log_misc;
extern struct itmon_logs *dss_itmon;

/*  Size domain */
#define DSS_KEEP_HEADER_SZ		(SZ_256 * 3)
#define DSS_HEADER_SZ			SZ_4K	/* 0x0 -- 0x1000 */
#define DSS_MMU_REG_SZ			SZ_4K	/* 0x1000 -- 0x2000 */
#define DSS_CORE_REG_SZ			SZ_4K	/* 0x2000 -- 0x3000 */
#define DSS_APM_CORE_REG_SZ		SZ_256
#define DSS_DBGC_SRAM_LOG_SZ		SZ_8K	/* 0x3100 -- 0x5100 */
#define DSS_DBGC_DRAM_LOG_SZ		SZ_16K	/* 0x7000 -- 0xB000 */

#define DSS_MMU_REG_OFFSET		SZ_512
#define DSS_CORE_REG_OFFSET		SZ_512
#define DSS_MAX_BL_SIZE			(20)
#define DSS_PANIC_LOG_SIZE		SZ_1K

/* Sign domain */
#define DSS_SIGN_RESET			0x0
#define DSS_SIGN_RESERVED		0x1
#define DSS_SIGN_SCRATCH		0xD
#define DSS_SIGN_ALIVE			0xFACE
#define DSS_SIGN_DEAD			0xDEAD
#define DSS_SIGN_PANIC			0xBABA
#define DSS_SIGN_UNKNOWN_REBOOT	0xCACA
#define DSS_SIGN_EMERGENCY_REBOOT	0xCACB
#define DSS_SIGN_WARM_REBOOT		0xCACC
#define DSS_SIGN_SAFE_FAULT		0xFAFA
#define DSS_SIGN_NORMAL_REBOOT		0xCAFE
#define DSS_SIGN_LOCKUP			0xDEADBEEF
#define DSS_SIGN_MAGIC			(0xDB9 << 16)
#define DSS_BOOT_CNT_MAGIC		0xFACEDB90
#define DSS_SLCDUMP_MAGIC		0x1337CACE

/*  Specific Address Information */
#define DSS_OFFSET_SCRATCH		(0x100)
#define DSS_OFFSET_NONE_DPM_DUMP_MODE	(0x108)
#define DSS_OFFSET_DEBUG_TEST_BUFFER(n)	(0x190 + (0x8 * n))
#define DSS_OFFSET_SLCDUMP_MAGIC	(0x270)
#define DSS_OFFSET_SLCDUMP_STATUS	(0x274)
#define DSS_OFFSET_SLCDUMP_BASE_REG	(0x278)
#define DSS_OFFSET_PRE_SLCDUMP_BASE_REG	(0x27C)
#define DSS_OFFSET_PMIC_REG_INT_MAGIC	(0x280)
#define DSS_OFFSET_PMIC_REG_INT_1	(0x284)
#define DSS_OFFSET_PMIC_REG_INT_2	(0x285)
#define DSS_OFFSET_PMIC_REG_INT_3	(0x286)
#define DSS_OFFSET_PMIC_REG_INT_4	(0x287)
#define DSS_OFFSET_PMIC_REG_INT_5	(0x288)
#define DSS_OFFSET_PMIC_REG_INT_6	(0x289)
#define DSS_OFFSET_PMIC_REASON		(0x290)
#define DSS_OFFSET_ABL_DUMP_STAT	(0x2C8)
#define DSS_OFFSET_EMERGENCY_REASON	(0x300)
#define DSS_OFFSET_WDT_CALLER		(0x310)
#define DSS_OFFSET_DUMP_GPR_WAIT	(0x380)
#define DSS_OFFSET_WAKEUP_WAIT		(0x390)
#define DSS_OFFSET_CORE_POWER_STAT	(0x400)
#define DSS_OFFSET_CORE_PMU_VAL		(0x440)
#define DSS_OFFSET_CORE_EHLD_STAT	(0x460)
#define DSS_OFFSET_GPR_POWER_STAT	(0x480)
#define DSS_OFFSET_PANIC_STAT		(0x500)
#define DSS_OFFSET_CORE_LAST_PC		(0x600)
#define DSS_OFFSET_QD_ENTRY		(0x680)
#define DSS_OFFSET_PANIC_STRING		(0xC00)

#define ARM_CPU_PART_CORTEX_A78		0xD41
#define ARM_CPU_PART_CORTEX_X1		0xD44

/* PMU register access */
#define PMU_GSA_INFORM0_OFFS		0x0830
#define PMU_GSA_INFORM0_APC_EARLY_WD	(1 << 5)

#endif
