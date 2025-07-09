// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 */

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/sched/clock.h>
#include <linux/of_fdt.h>
#include <linux/libfdt.h>

#include <asm/stacktrace.h>

#include <soc/google/debug-snapshot.h>
#include "debug-snapshot-local.h"

static const char *dpm_policy[] = {
	[GO_DEFAULT_ID]		= GO_DEFAULT,
	[GO_PANIC_ID]		= GO_PANIC,
	[GO_WATCHDOG_ID]	= GO_WATCHDOG,
	[GO_S2D_ID]		= GO_S2D,
	[GO_ARRAYDUMP_ID]	= GO_ARRAYDUMP,
	[GO_SCANDUMP_ID]	= GO_SCANDUMP,
};

struct dbg_snapshot_dpm dss_dpm;
EXPORT_SYMBOL(dss_dpm);

bool dbg_snapshot_get_dpm_status(void)
{
	return dss_dpm.enabled;
}
EXPORT_SYMBOL(dbg_snapshot_get_dpm_status);

bool dbg_snapshot_get_enabled_debug_kinfo(void)
{
	return dss_dpm.enabled_debug_kinfo;
}
EXPORT_SYMBOL(dbg_snapshot_get_enabled_debug_kinfo);

void dbg_snapshot_do_dpm(struct pt_regs *regs)
{
	unsigned int esr = read_sysreg(esr_el1);
	unsigned int val = 0;
	unsigned int policy = GO_DEFAULT_ID;

	/* check dpm */
	if (!dss_dpm.enabled || !dss_dpm.enabled_debug || dss_dpm.dump_mode_none)
		return;

	switch (ESR_ELx_EC(esr)) {
	case ESR_ELx_EC_DABT_CUR:
		val = esr & 63;
		if ((val >= 4 && val <= 7) ||	/* translation fault */
	   			(val >= 9 && val <= 11) || /* page fault */
	   			(val >= 12 && val <= 15))	/* page fault */
			policy = GO_DEFAULT_ID;
		else
			policy = dss_dpm.p_el1_da;
		break;
	case ESR_ELx_EC_IABT_CUR:
		policy = dss_dpm.p_el1_ia;
		break;
	case ESR_ELx_EC_SYS64:
		policy = dss_dpm.p_el1_undef;
		break;
	case ESR_ELx_EC_SP_ALIGN:
		policy = dss_dpm.p_el1_sp_pc;
		break;
	case ESR_ELx_EC_PC_ALIGN:
		policy = dss_dpm.p_el1_sp_pc;
		break;
	case ESR_ELx_EC_UNKNOWN:
		policy = dss_dpm.p_el1_undef;
		break;
	case ESR_ELx_EC_SOFTSTP_LOW:
	case ESR_ELx_EC_SOFTSTP_CUR:
	case ESR_ELx_EC_BREAKPT_LOW:
	case ESR_ELx_EC_BREAKPT_CUR:
	case ESR_ELx_EC_WATCHPT_LOW:
	case ESR_ELx_EC_WATCHPT_CUR:
	case ESR_ELx_EC_BRK64:
		policy = GO_DEFAULT_ID;
		break;
	default:
		policy = dss_dpm.p_el1_serror;
		break;
	}

	if (policy && policy != GO_DEFAULT_ID) {
		if (dss_dpm.pre_log) {
			pr_emerg("pc : %pS\n", (void *)regs->pc);
			pr_emerg("lr : %pS\n", (void *)regs->regs[30]);
		}
		dbg_snapshot_do_dpm_policy(policy, dpm_policy[policy]);
	}
}
EXPORT_SYMBOL_GPL(dbg_snapshot_do_dpm);

static void dbg_snapshot_dt_scan_dpm_feature(struct device_node *node)
{
	struct device_node *item;
	unsigned int val;

	dss_dpm.enabled_debug = false;
	dss_dpm.dump_mode = NONE_DUMP;

	item = of_find_node_by_name(node, "dump-mode");
	if (!item) {
		pr_info("dpm: No such ramdump node, [dump-mode] disabled\n");
		goto exit_dss;
	}

	if (of_property_read_u32(item, "enabled", &val)) {
		pr_info("dpm: No such enabled of dump-mode, [dump-mode] disabled\n");
	} else {
		if (val == FULL_DUMP || val == QUICK_DUMP) {
			pr_info("dpm: dump-mode is %s Dump\n",
				val == FULL_DUMP ? "Full" : "Quick");
			dss_dpm.enabled_debug = true;
			dss_dpm.dump_mode = val;
		} else {
			goto exit_dss;
		}
	}

	if (of_property_read_u32(item, "file-support", &val)) {
		pr_info("dpm: No such file-support of dump-mode, [file-support] disabled\n");
	} else {
		dss_dpm.dump_mode_file = val;
		pr_info("dpm: file-support of dump-mode is %sabled\n",
			val ? "en" : "dis");
	}

	item = of_find_node_by_name(node, "event");
	if (!item) {
		pr_warn("dpm: No such methods of kernel event\n");
		goto exit_dss;
	}

	item = of_find_node_by_name(node, "debug-kinfo");
	if (!item) {
		pr_info("dpm: No such debug-kinfo node, [debug-kinfo] disabled\n");
		goto exit_dss;
	}

	if (of_property_read_u32(item, "enabled", &val)) {
		pr_info("dpm: No such enabled of debug-kinfo, [debug-kinfo] disabled\n");
	} else {
		dss_dpm.enabled_debug_kinfo = val;
		pr_info("dpm: debug-kinfo is %sabled\n", val ? "en" : "dis");
	}

exit_dss:
	return;
}

static void dbg_snapshot_dt_scan_dpm_policy(struct device_node *node)
{
	struct device_node *item;
	unsigned int val;

	item = of_find_node_by_name(node, "exception");
	if (!item) {
		pr_info("dpm: No such exception node, nothing to [policy]\n");
		return;
	}

	if (of_property_read_u32(item, "pre_log", &val))
		pr_info("dpm: No such [%s]\n", DPM_P_EL1_DA);
	else
		dss_dpm.pre_log = val;

	if (of_property_read_u32(item, DPM_P_EL1_DA, &val)) {
		pr_info("dpm: No such [%s]\n", DPM_P_EL1_DA);
	} else {
		dss_dpm.p_el1_da = val;
		pr_info("dpm: run [%s] at [%s]\n",
				dpm_policy[val], DPM_P_EL1_DA);
	}

	if (of_property_read_u32(item, DPM_P_EL1_IA, &val)) {
		pr_info("dpm: No such [%s]\n", DPM_P_EL1_IA);
	} else {
		dss_dpm.p_el1_ia = val;
		pr_info("dpm: run [%s] at [%s]\n",
				dpm_policy[val], DPM_P_EL1_IA);
	}

	if (of_property_read_u32(item, DPM_P_EL1_UNDEF, &val)) {
		pr_info("dpm: No such [%s]\n", DPM_P_EL1_UNDEF);
	} else {
		dss_dpm.p_el1_undef = val;
		pr_info("dpm: run [%s] at [%s]\n",
				dpm_policy[val], DPM_P_EL1_UNDEF);
	}

	if (of_property_read_u32(item, DPM_P_EL1_SP_PC, &val)) {
		pr_info("dpm: No such [%s]\n", DPM_P_EL1_SP_PC);
	} else {
		dss_dpm.p_el1_sp_pc = val;
		pr_info("dpm: run [%s] at [%s]\n",
				dpm_policy[val], DPM_P_EL1_SP_PC);
	}

	if (of_property_read_u32(item, DPM_P_EL1_INV, &val)) {
		pr_info("dpm: No such [%s]\n", DPM_P_EL1_INV);
	} else {
		dss_dpm.p_el1_inv = val;
		pr_info("dpm: run [%s] at [%s]\n",
				dpm_policy[val], DPM_P_EL1_INV);
	}

	if (of_property_read_u32(item, DPM_P_EL1_SERROR, &val)) {
		pr_info("dpm: No such [%s]\n", DPM_P_EL1_SERROR);
	} else {
		dss_dpm.p_el1_serror = val;
		pr_info("dpm: run [%s] at [%s]\n",
				dpm_policy[val], DPM_P_EL1_SERROR);
	}
}

int dbg_snapshot_dt_scan_dpm(void)
{
	struct device_node *root, *next;
	unsigned int val;

	memset(&dss_dpm, 0, sizeof(struct dbg_snapshot_dpm));

	root = of_find_node_by_name(NULL, "dpm");
	if (!root)
		return -ENODEV;

	dss_dpm.enabled = true;

	/* version */
	if (!of_property_read_u32(root, "version", &val)) {
		dss_dpm.version = val;
		pr_info("dpm: v%01d.%02d\n", val / 100, val % 100);
	} else {
		pr_info("dpm: version is not found\n");
	}

	/* feature setting */
	next = of_find_node_by_name(root, DPM_F);
	if (!next) {
		pr_warn("dpm: No such features of debug policy\n");
	} else {
		pr_warn("dpm: found features of debug policy\n");
		dbg_snapshot_dt_scan_dpm_feature(next);
	}

	/* policy setting */
	next = of_find_node_by_name(root, DPM_P);
	if (!next) {
		pr_warn("dpm: No such policy of debug policy\n");
	} else {
		pr_warn("dpm: found policy of debug policy\n");
		dbg_snapshot_dt_scan_dpm_policy(next);
	}

	return 0;
}

void dbg_snapshot_init_dpm(void)
{
	if (dss_dpm.enabled_debug)
		dbg_snapshot_scratch_reg(DSS_SIGN_SCRATCH);

	if (dbg_snapshot_get_dpm_none_dump_mode() > 0)
		dss_dpm.dump_mode_none = 1;
}
