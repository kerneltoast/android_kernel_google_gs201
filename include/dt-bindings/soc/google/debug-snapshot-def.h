/* SPDX-License-Identifier: GPL-2.0 */

#ifndef DEBUG_SNAPSHOT_DEF_H
#define DEBUG_SNAPSHOT_DEF_H

#define SZ_64K			0x00010000
#define SZ_512K			0x00080000
#define SZ_1M			0x00100000

#define DSS_START_ADDR		0xD8100000
#define DSS_HEADER_SIZE		SZ_64K
#define DSS_LOG_KEVENTS_SIZE	(7 * SZ_1M)
#define DSS_LOG_S2D_SIZE	(21 * SZ_1M)
#define DSS_LOG_ARRAYRESET_SIZE	(10 * SZ_1M)
#define DSS_LOG_ARRAYPANIC_SIZE	(10 * SZ_1M)
#define DSS_LOG_SLCDUMP_SIZE	(8 * SZ_1M + SZ_512K)
#define DSS_LOG_BCM_SIZE	(4 * SZ_1M)
#define DSS_LOG_ITMON_SIZE	SZ_64K

#define DSS_HEADER_OFFSET		0
#define DSS_HEADER_ADDR			(DSS_START_ADDR + DSS_HEADER_OFFSET)
#define DSS_LOG_KEVENTS_ADDR		(DSS_HEADER_ADDR + DSS_HEADER_SIZE)
/* S2D addr is manually calculated in dbgc code, try to avoid relocating */
#define DSS_LOG_S2D_ADDR		(DSS_LOG_KEVENTS_ADDR + DSS_LOG_KEVENTS_SIZE)
#define DSS_LOG_ARRAYRESET_ADDR		(DSS_LOG_S2D_ADDR + DSS_LOG_S2D_SIZE)
#define DSS_LOG_ARRAYPANIC_ADDR		(DSS_LOG_ARRAYRESET_ADDR + DSS_LOG_ARRAYRESET_SIZE)
#define DSS_LOG_SLCDUMP_ADDR		(DSS_LOG_ARRAYPANIC_ADDR + DSS_LOG_ARRAYPANIC_SIZE)
#define DSS_LOG_PRE_SLCDUMP_ADDR	(DSS_LOG_SLCDUMP_ADDR + DSS_LOG_SLCDUMP_SIZE)
#define DSS_LOG_BCM_ADDR		(DSS_LOG_PRE_SLCDUMP_ADDR + DSS_LOG_SLCDUMP_SIZE)
#define DSS_LOG_ITMON_ADDR		(DSS_LOG_BCM_ADDR + DSS_LOG_BCM_SIZE)

/* DSS Header contents */
#define DSS_HDR_INFO_SZ			SZ_4K
#define DSS_HDR_SYSREG_SZ		SZ_4K
#define DSS_HDR_COREREG_SZ		SZ_4K
#define DSS_HDR_APM_COREREG_SZ		SZ_256
#define DSS_HDR_DBGC_MISC_SZ		(SZ_16K - SZ_256)
#define DSS_HDR_DBGC_DRAM_LOG_SZ	(15 * SZ_1K)

#define DSS_HDR_SYSREG_OFFS		DSS_HDR_INFO_SZ
#define DSS_HDR_COREREG_OFFS		(DSS_HDR_SYSREG_OFFS + DSS_HDR_SYSREG_SZ)
#define DSS_HDR_APM_COREREG_OFFS	(DSS_HDR_COREREG_OFFS + DSS_HDR_COREREG_SZ)
#define DSS_HDR_DBGC_MISC_OFFS		(DSS_HDR_APM_COREREG_OFFS + DSS_HDR_APM_COREREG_SZ)
#define DSS_HDR_DBGC_DRAM_LOG_OFFS	(DSS_HDR_DBGC_MISC_OFFS + DSS_HDR_DBGC_MISC_SZ)

/* DSS Header DBGC Misc area contents */
#define DSS_HDR_DBGC_VERSION_SZ		48
#define DSS_HDR_DBGC_SRAM_LOG_SZ	(SZ_4K + SZ_1K)
#define DSS_HDR_DBGC_REGDUMP_SZ		SZ_256
#define DSS_HDR_DBGC_EXCHG_BUFF_SZ	SZ_256
#define DSS_HDR_DBGC_FREE_AREA_SZ	(DSS_HDR_DBGC_MISC_SZ - DSS_HDR_DBGC_VERSION_SZ - \
					DSS_HDR_DBGC_SRAM_LOG_SZ - DSS_HDR_DBGC_REGDUMP_SZ - \
					DSS_HDR_DBGC_EXCHG_BUFF_SZ)

#define DSS_HDR_DBGC_VERSION_OFFS	DSS_HDR_DBGC_MISC_OFFS
#define DSS_HDR_DBGC_SRAM_LOG_OFFS	(DSS_HDR_DBGC_VERSION_OFFS + DSS_HDR_DBGC_VERSION_SZ)
#define DSS_HDR_DBGC_REGDUMP_OFFS	(DSS_HDR_DBGC_SRAM_LOG_OFFS + DSS_HDR_DBGC_SRAM_LOG_SZ)
#define DSS_HDR_DBGC_FREE_AREA_OFFS	(DSS_HDR_DBGC_REGDUMP_OFFS + DSS_HDR_DBGC_REGDUMP_SZ)
#define DSS_HDR_DBGC_EXCHG_BUFF_OFFS	(DSS_HDR_DBGC_FREE_AREA_OFFS + DSS_HDR_DBGC_FREE_AREA_SZ)


/* KEVENT ID */
#define DSS_ITEM_HEADER		"header"
#define DSS_ITEM_KEVENTS	"log_kevents"
#define DSS_ITEM_S2D		"log_s2d"
#define DSS_ITEM_ARRDUMP_RESET	"log_array_reset"
#define DSS_ITEM_ARRDUMP_PANIC	"log_array_panic"
#define DSS_ITEM_SLCDUMP	"log_slcdump"
#define DSS_ITEM_PRE_SLCDUMP	"log_preslcdump"
#define DSS_ITEM_BCM		"log_bcm"
#define DSS_ITEM_ITMON		"log_itmon"

#define DSS_LOG_TASK		"task_log"
#define DSS_LOG_WORK		"work_log"
#define DSS_LOG_CPUIDLE		"cpuidle_log"
#define DSS_LOG_SUSPEND		"suspend_log"
#define DSS_LOG_IRQ		"irq_log"
#define DSS_LOG_HRTIMER		"hrtimer_log"
#define DSS_LOG_CLK		"clk_log"
#define DSS_LOG_PMU		"pmu_log"
#define DSS_LOG_FREQ		"freq_log"
#define DSS_LOG_DM		"dm_log"
#define DSS_LOG_REGULATOR	"regulator_log"
#define DSS_LOG_THERMAL		"thermal_log"
#define DSS_LOG_ACPM		"acpm_log"
#define DSS_LOG_PRINTK		"printk_log"

/* MODE */
#define NONE_DUMP		0
#define FULL_DUMP		1
#define QUICK_DUMP		2

/* ACTION */
#define GO_DEFAULT		"default"
#define GO_DEFAULT_ID		0
#define GO_PANIC		"panic"
#define GO_PANIC_ID		1
#define GO_WATCHDOG		"watchdog"
#define GO_WATCHDOG_ID		2
#define GO_S2D			"s2d"
#define GO_S2D_ID		3
#define GO_ARRAYDUMP		"arraydump"
#define GO_ARRAYDUMP_ID		4
#define GO_SCANDUMP		"scandump"
#define GO_SCANDUMP_ID		5
#define GO_HALT			"halt"
#define GO_HALT_ID		6
#define GO_ACTION_MAX		7

/* EXCEPTION POLICY */
#define DPM_F			"feature"
#define DPM_P			"policy"

#define DPM_P_EL1_DA		"el1_da"
#define DPM_P_EL1_IA		"el1_ia"
#define DPM_P_EL1_UNDEF		"el1_undef"
#define DPM_P_EL1_SP_PC		"el1_sp_pc"
#define DPM_P_EL1_INV		"el1_inv"
#define DPM_P_EL1_SERROR	"el1_serror"

/* Enable by DPM */
#define DPM_ENABLE			1
/* Enable by Privileged Debug */
#define PRIVILEGED_ENABLE		2
#endif
