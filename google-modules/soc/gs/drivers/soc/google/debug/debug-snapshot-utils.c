// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 */

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/notifier.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/bitops.h>
#include <linux/sched/clock.h>
#include <linux/sched/debug.h>
#include <linux/nmi.h>
#include <linux/init_task.h>
#include <linux/reboot.h>
#include <linux/kdebug.h>
#include <linux/panic_notifier.h>
#include <linux/reboot.h>

#include <asm/cputype.h>
#include <asm/smp_plat.h>
#include <asm/sysreg.h>
#include <asm/system_misc.h>
#include "system-regs.h"

#include <soc/google/debug-snapshot.h>
#include <soc/google/exynos-pmu-if.h>
#include "debug-snapshot-local.h"

#include <trace/hooks/debug.h>

static const char * const ecc_sel_str_dsu_l1_l2[] = {
	"DSU", "L1", "L2", NULL,
};

static const char * const ecc_sel_str_core_dsu[] = {
	"CORE", "DSU", NULL,
};

static const char * const ecc_sel_str_dsu_core[] = {
	"DSU", "CORE", NULL,
};

struct dbg_snapshot_mmu_reg {
	long SCTLR_EL1;
	long TTBR0_EL1;
	long TTBR1_EL1;
	long TCR_EL1;
	long ESR_EL1;
	long FAR_EL1;
	long CONTEXTIDR_EL1;
	long TPIDR_EL0;
	long TPIDRRO_EL0;
	long TPIDR_EL1;
	long MAIR_EL1;
	long ELR_EL1;
	long SP_EL0;
};

static struct pt_regs __percpu **dss_core_reg;
static struct dbg_snapshot_mmu_reg __percpu **dss_mmu_reg;
static struct dbg_snapshot_helper_ops dss_soc_ops;

void cache_flush_all(void)
{
	dss_flush_cache_all();
}
EXPORT_SYMBOL_GPL(cache_flush_all);

static void dbg_snapshot_dump_panic(char *str, size_t len)
{
	/*  This function is only one which runs in panic function */
	if (str && len && len < DSS_PANIC_STRING_SZ)
		memcpy(dbg_snapshot_get_header_vaddr() + DSS_OFFSET_PANIC_STRING,
				str, len);
}

static void dbg_snapshot_set_core_power_stat(unsigned int val, unsigned int cpu)
{
	void __iomem *header = dbg_snapshot_get_header_vaddr();

	if (header)
		__raw_writel(val, header + DSS_OFFSET_CORE_POWER_STAT + cpu * 4);
}

static unsigned int dbg_snapshot_get_core_panic_stat(unsigned int cpu)
{
	void __iomem *header = dbg_snapshot_get_header_vaddr();

	return header ? __raw_readl(header + DSS_OFFSET_PANIC_STAT + cpu * 4) : 0;
}

static void dbg_snapshot_set_core_panic_stat(unsigned int val, unsigned int cpu)
{
	void __iomem *header = dbg_snapshot_get_header_vaddr();

	if (header)
		__raw_writel(val, header + DSS_OFFSET_PANIC_STAT + cpu * 4);
}

void dbg_snapshot_set_core_cflush_stat(unsigned int val)
{
	void __iomem *header = dbg_snapshot_get_header_vaddr();

	if (header)
		__raw_writel(val, header + DSS_OFFSET_CFLUSH_STAT);
}
EXPORT_SYMBOL_GPL(dbg_snapshot_set_core_cflush_stat);

void dbg_snapshot_set_core_pmu_val(unsigned int val, unsigned int cpu)
{
	void __iomem *header = dbg_snapshot_get_header_vaddr();

	if (header)
		__raw_writel(val, header + DSS_OFFSET_CORE_PMU_VAL + cpu * 4);
}
EXPORT_SYMBOL_GPL(dbg_snapshot_set_core_pmu_val);

void dbg_snapshot_set_usb_otg(unsigned int val)
{
	void __iomem *header = dbg_snapshot_get_header_vaddr();

	if (header)
		__raw_writel(val, header + DSS_OFFSET_PMIC_REG_INT_5_S);
}
EXPORT_SYMBOL_GPL(dbg_snapshot_set_usb_otg);

unsigned int dbg_snapshot_get_core_pmu_val(unsigned int cpu)
{
	void __iomem *header = dbg_snapshot_get_header_vaddr();

	return header ? __raw_readl(header + DSS_OFFSET_CORE_PMU_VAL + cpu * 4) : 0;
}
EXPORT_SYMBOL_GPL(dbg_snapshot_get_core_pmu_val);

void dbg_snapshot_set_core_ehld_stat(unsigned int val, unsigned int cpu)
{
	void __iomem *header = dbg_snapshot_get_header_vaddr();

	if (header)
		__raw_writel(val, header + DSS_OFFSET_CORE_EHLD_STAT + cpu * 4);
}
EXPORT_SYMBOL_GPL(dbg_snapshot_set_core_ehld_stat);

unsigned int dbg_snapshot_get_core_ehld_stat(unsigned int cpu)
{
	void __iomem *header = dbg_snapshot_get_header_vaddr();

	return header ? __raw_readl(header + DSS_OFFSET_CORE_EHLD_STAT + cpu * 4) : 0;
}
EXPORT_SYMBOL_GPL(dbg_snapshot_get_core_ehld_stat);

static void dbg_snapshot_set_abl_dump_stat(unsigned int val)
{
	void __iomem *header = dbg_snapshot_get_header_vaddr();

	if (header)
		__raw_writel(val, header + DSS_OFFSET_ABL_DUMP_STAT);
}

static void dbg_snapshot_report_reason(unsigned int val)
{
	void __iomem *header = dbg_snapshot_get_header_vaddr();

	if (header)
		__raw_writel(val, header + DSS_OFFSET_EMERGENCY_REASON);
}

static void dbg_snapshot_set_reboot_mode(enum reboot_mode mode)
{
	reboot_mode = mode;
}

static void dbg_snapshot_set_wdt_caller(unsigned long addr)
{
	void __iomem *header = dbg_snapshot_get_header_vaddr();

	if (header)
		__raw_writeq(addr, header + DSS_OFFSET_WDT_CALLER);
}

int dbg_snapshot_start_watchdog(int sec)
{
	if (dss_soc_ops.start_watchdog)
		return dss_soc_ops.start_watchdog(true, 0, sec);

	return -ENODEV;
}
EXPORT_SYMBOL_GPL(dbg_snapshot_start_watchdog);

int dbg_snapshot_emergency_reboot_timeout(const char *str, int tick)
{
	void *addr;
	char *reboot_msg;

	reboot_msg = kmalloc(DSS_PANIC_STRING_SZ, GFP_ATOMIC);
	if (!reboot_msg)
		return -ENOMEM;

	/*
	 * Set default "Emergency Reboot" message
	 */
	scnprintf(reboot_msg, DSS_PANIC_STRING_SZ, "Emergency Reboot");

	if (!dss_soc_ops.expire_watchdog) {
		dev_emerg(dss_desc.dev, "There are no wdt functions\n");
		return -ENODEV;
	}

	if (tick == INT_MAX) {
		tick = 1;
		addr = return_address(1);
	} else {
		addr = return_address(0);
	}

	dbg_snapshot_set_wdt_caller((unsigned long)addr);
	if (str)
		scnprintf(reboot_msg, DSS_PANIC_STRING_SZ, str);

	dev_emerg(dss_desc.dev, "WDT Caller: %pS %s\n", addr, str ? str : "");

	dbg_snapshot_dump_panic(reboot_msg, strlen(reboot_msg));
	dump_stack();

	dss_soc_ops.expire_watchdog(tick, 0);
	kfree(reboot_msg);
	return 0;
}
EXPORT_SYMBOL_GPL(dbg_snapshot_emergency_reboot_timeout);

int dbg_snapshot_emergency_reboot(const char *str)
{
	dbg_snapshot_emergency_reboot_timeout(str, INT_MAX);
	dbg_snapshot_spin_func();
	return 0;
}
EXPORT_SYMBOL_GPL(dbg_snapshot_emergency_reboot);

int dbg_snapshot_kick_watchdog(void)
{
	if (dss_soc_ops.start_watchdog)
		return dss_soc_ops.start_watchdog(false, 0, 0);

	return -ENODEV;
}
EXPORT_SYMBOL_GPL(dbg_snapshot_kick_watchdog);

static void dbg_snapshot_dump_one_task_info(struct task_struct *tsk, bool is_main)
{
	static const char state_array[] = {'R', 'S', 'D', 'T', 't', 'X',
			'Z', 'P', 'x', 'K', 'W', 'I', 'N', '?'};
	int idx;
	unsigned int state;
	unsigned long pc;

	if ((!tsk) || !try_get_task_stack(tsk) || (tsk->flags & TASK_FROZEN) ||
	    !(tsk->__state == TASK_RUNNING ||
	    tsk->__state == TASK_UNINTERRUPTIBLE ||
	    tsk->__state == TASK_KILLABLE))
		return;

	state = tsk->__state | tsk->exit_state;
	pc = KSTK_EIP(tsk);
	idx = fls(state);
	if (idx >= sizeof(state_array))
		idx = sizeof(state_array) - 1;

	/*
	 * kick watchdog to prevent unexpected reset during panic sequence
	 * and it prevents the hang during panic sequence by watchedog
	 */
	touch_softlockup_watchdog();

	pr_info("%8d %16llu %16llu %16llu %c(%u) %3d %16pK %16pK %c %16s\n",
		tsk->pid, tsk->utime, tsk->stime,
		tsk->se.exec_start, state_array[idx], (tsk->__state),
		task_cpu(tsk), (void *)pc, tsk, is_main ? '*' : ' ', tsk->comm);

	sched_show_task(tsk);
}

static inline struct task_struct *get_next_thread(struct task_struct *tsk)
{
	return container_of(tsk->thread_group.next, struct task_struct, thread_group);
}

static void dbg_snapshot_dump_task_info(void)
{
	struct task_struct *frst_tsk, *curr_tsk;
	struct task_struct *frst_thr, *curr_thr;

	pr_info("\n");
	pr_info(" current proc : %d %s\n",
			current->pid, current->comm);
	pr_info("------------------------------------------------------------------------------\n");
	pr_info("%8s %8s %8s %16s %4s %3s %16s %16s  %16s\n",
			"pid", "uTime", "sTime", "exec(ns)", "stat", "cpu",
			"user_pc", "task_struct", "comm");
	pr_info("------------------------------------------------------------------------------\n");

	/* processes */
	frst_tsk = &init_task;
	curr_tsk = frst_tsk;
	while (curr_tsk) {
		dbg_snapshot_dump_one_task_info(curr_tsk,  true);
		/* threads */
		if (curr_tsk->thread_group.next != NULL) {
			frst_thr = get_next_thread(curr_tsk);
			curr_thr = frst_thr;
			if (frst_thr != curr_tsk) {
				while (curr_thr != NULL) {
					dbg_snapshot_dump_one_task_info(curr_thr, false);
					curr_thr = get_next_thread(curr_thr);
					if (curr_thr == curr_tsk)
						break;
				}
			}
		}
		curr_tsk = container_of(curr_tsk->tasks.next,
					struct task_struct, tasks);
		if (curr_tsk == frst_tsk)
			break;
	}
	pr_info("------------------------------------------------------------------------------\n");
}

static void dbg_snapshot_save_system(void *unused)
{
	struct dbg_snapshot_mmu_reg *mmu_reg;

	mmu_reg = *per_cpu_ptr(dss_mmu_reg, raw_smp_processor_id());

	asm volatile ("mrs x1, SCTLR_EL1\n\t"	/* SCTLR_EL1 */
		"mrs x2, TTBR0_EL1\n\t"		/* TTBR0_EL1 */
		"stp x1, x2, [%0]\n\t"
		"mrs x1, TTBR1_EL1\n\t"		/* TTBR1_EL1 */
		"mrs x2, TCR_EL1\n\t"		/* TCR_EL1 */
		"stp x1, x2, [%0, #0x10]\n\t"
		"mrs x1, ESR_EL1\n\t"		/* ESR_EL1 */
		"mrs x2, FAR_EL1\n\t"		/* FAR_EL1 */
		"stp x1, x2, [%0, #0x20]\n\t"
		"mrs x1, CONTEXTIDR_EL1\n\t"	/* CONTEXTIDR_EL1 */
		"mrs x2, TPIDR_EL0\n\t"		/* TPIDR_EL0 */
		"stp x1, x2, [%0, #0x30]\n\t"
		"mrs x1, TPIDRRO_EL0\n\t"	/* TPIDRRO_EL0 */
		"mrs x2, TPIDR_EL1\n\t"		/* TPIDR_EL1 */
		"stp x1, x2, [%0, #0x40]\n\t"
		"mrs x1, MAIR_EL1\n\t"		/* MAIR_EL1 */
		"mrs x2, ELR_EL1\n\t"		/* ELR_EL1 */
		"stp x1, x2, [%0, #0x50]\n\t"
		"mrs x1, SP_EL0\n\t"		/* SP_EL0 */
		"str x1, [%0, 0x60]\n\t"
		:				/* output */
		: "r"(mmu_reg)			/* input */
		: "x1", "x2", "memory"		/* clobbered register */
	);
}

static void clear_external_ecc_err(ERXSTATUS_EL1_t erxstatus_el1)
{
	erxstatus_el1.field.CE = 0x3;
	erxstatus_el1.field.UET = 0x3;
	erxstatus_el1.field.SERR = 0x0;
	erxstatus_el1.field.IERR = 0x0;

	write_ERXSTATUS_EL1(erxstatus_el1.reg);
	write_ERXMISC0_EL1(0);
	write_ERXMISC1_EL1(0);
}

static const char *get_external_ecc_err(ERXSTATUS_EL1_t erxstatus_el1)
{
	const char *ext_err;

	if (erxstatus_el1.field.SERR == 0xC)
		ext_err = "Data value from (non-associative) external memory.";
	else if (erxstatus_el1.field.SERR == 0x12)
		ext_err = "Error response from Completer of access.";
	else
		ext_err = NULL;

	clear_external_ecc_err(erxstatus_el1);

	return ext_err;
}

static const char *get_correct_ecc_err(ERXSTATUS_EL1_t erxstatus_el1)
{
	const char *cr_err;

	switch (erxstatus_el1.field.CE) {
	case BIT(1) | BIT(0):
		cr_err = "At least persistent was corrected";
		break;
	case BIT(1):
		cr_err = "At least one error was corrected";
		break;
	case BIT(0):
		cr_err = "At least one transient error was corrected";
		break;
	default:
		cr_err = NULL;
	}

	return cr_err;
}

static void _dbg_snapshot_ecc_dump(bool call_panic, const char * const ecc_sel_str[])
{
	ERRSELR_EL1_t errselr_el1;
	ERRIDR_EL1_t erridr_el1;
	ERXSTATUS_EL1_t erxstatus_el1;
	const char *msg;
	bool is_capable_identifing_err = false;
	int i;

	asm volatile ("HINT #16");
	erridr_el1.reg = read_ERRIDR_EL1();

	for (i = 0; i < (int)erridr_el1.field.NUM; i++) {
		static char errbuf[SZ_256];
		int n = 0;

		errselr_el1.reg = read_ERRSELR_EL1();
		errselr_el1.field.SEL = i;
		write_ERRSELR_EL1(errselr_el1.reg);

		isb();

		erxstatus_el1.reg = read_ERXSTATUS_EL1();
		if (!erxstatus_el1.field.VALID)
			continue;

		n = scnprintf(errbuf + n, sizeof(errbuf) - n,
			      "%4s:  Error: [NUM:%d][ERXSTATUS_EL1:%#016llx]\n",
			      ecc_sel_str[i] ? ecc_sel_str[i] : "", i, erxstatus_el1.reg);

		if (erxstatus_el1.field.AV)
			n += scnprintf(errbuf + n, sizeof(errbuf) - n,
				"\t [ AV ] Detected(Address VALID): [ERXADDR_EL1:%#llx]\n",
				read_ERXADDR_EL1());
		if (erxstatus_el1.field.OF)
			n += scnprintf(errbuf + n, sizeof(errbuf) - n,
				"\t [ OF ] Detected(Overflow): There was more than one error has occurred\n");
		if (erxstatus_el1.field.ER)
			n += scnprintf(errbuf + n, sizeof(errbuf) - n,
				"\t [ ER ] Detected(Error Report by external abort)\n");
		if (erxstatus_el1.field.UE)
			n += scnprintf(errbuf + n, sizeof(errbuf) - n,
				"\t [ UE ] Detected(Uncorrected Error): Not deferred\n");
		if (erxstatus_el1.field.DE)
			n += scnprintf(errbuf + n, sizeof(errbuf) - n,
				"\t [ DE ] Detected(Deferred Error)\n");
		if (erxstatus_el1.field.MV) {
			n += scnprintf(errbuf + n, sizeof(errbuf) - n,
				"\t [ MV ] Detected(Miscellaneous Registers VALID): [ERXMISC0_EL1:%#llx][ERXMISC1_EL1:%#llx]\n",
				read_ERXMISC0_EL1(), read_ERXMISC1_EL1());
		}
		if (erxstatus_el1.field.CE) {
			msg = get_correct_ecc_err(erxstatus_el1);
			if (msg)
				n += scnprintf(errbuf + n, sizeof(errbuf) - n,
				"\t [ CE ] Detected(Corrected Error): %s, [CE:%#x]\n",
				msg, erxstatus_el1.field.CE);
		}
		if (erxstatus_el1.field.SERR) {
			msg = get_external_ecc_err(erxstatus_el1);
			if (msg) {
				n += scnprintf(errbuf + n, sizeof(errbuf) - n,
				"\t [SERR] Detected(External ECC Error): %s, [SERR:%#x]\n",
				msg, erxstatus_el1.field.SERR);
				goto output_cont;
			} else {
				pr_warn("ecc: [SERR] Warning WO msg [CODE:%#x]\n",
					erxstatus_el1.field.SERR);
			}
		}
		is_capable_identifing_err = true;
output_cont:
		pr_emerg("%s", errbuf);
	}

	if (call_panic && is_capable_identifing_err)
		panic("RAS(ECC) error occurred");
}

void dbg_snapshot_ecc_dump(bool call_panic)
{
	unsigned int cpuid_part = read_cpuid_part_number();

	switch (cpuid_part) {
	case ARM_CPU_PART_CORTEX_A55:
	case ARM_CPU_PART_CORTEX_A76:
	case ARM_CPU_PART_CORTEX_A78:
	case ARM_CPU_PART_CORTEX_X1:
		_dbg_snapshot_ecc_dump(call_panic, ecc_sel_str_core_dsu);
		break;
	case ARM_CPU_PART_CORTEX_A510:
		_dbg_snapshot_ecc_dump(call_panic, ecc_sel_str_dsu_l1_l2);
		break;
	case ARM_CPU_PART_MAKALU:
	case ARM_CPU_PART_MAKALU_ELP:
	case ARM_CPU_PART_HAYDEN:
	case ARM_CPU_PART_HUNTER:
	case ARM_CPU_PART_HUNTER_ELP:
		_dbg_snapshot_ecc_dump(call_panic, ecc_sel_str_dsu_core);
		break;
	default:
		pr_emerg("Unknown cpuid part number - 0x%x\n", cpuid_part);
		break;
	}
}
EXPORT_SYMBOL_GPL(dbg_snapshot_ecc_dump);

static inline void dbg_snapshot_save_core(struct pt_regs *regs)
{
	unsigned int cpu = raw_smp_processor_id();
	struct pt_regs *core_reg = *per_cpu_ptr(dss_core_reg, cpu);

	if (!core_reg) {
		pr_err("Core reg is null\n");
		return;
	}
	if (!regs) {
		asm volatile ("str x0, [%0, #0]\n\t"
				"mov x0, %0\n\t"
				"stp x1, x2, [x0, #0x8]\n\t"
				"stp x3, x4, [x0, #0x18]\n\t"
				"stp x5, x6, [x0, #0x28]\n\t"
				"stp x7, x8, [x0, #0x38]\n\t"
				"stp x9, x10, [x0, #0x48]\n\t"
				"stp x11, x12, [x0, #0x58]\n\t"
				"stp x13, x14, [x0, #0x68]\n\t"
				"stp x15, x16, [x0, #0x78]\n\t"
				"stp x17, x18, [x0, #0x88]\n\t"
				"stp x19, x20, [x0, #0x98]\n\t"
				"stp x21, x22, [x0, #0xa8]\n\t"
				"stp x23, x24, [x0, #0xb8]\n\t"
				"stp x25, x26, [x0, #0xc8]\n\t"
				"stp x27, x28, [x0, #0xd8]\n\t"
				"stp x29, x30, [x0, #0xe8]\n\t"
				:
				: "r"(core_reg)
				: "x0", "memory");
		core_reg->sp = core_reg->regs[29];
		core_reg->pc =
			(unsigned long)(core_reg->regs[30] - sizeof(unsigned int));
		/* We don't know other bits but mode is definitely CurrentEL. */
		core_reg->pstate = read_sysreg(CurrentEL);
	} else {
		memcpy(core_reg, regs, sizeof(struct user_pt_regs));
	}
}

void dbg_snapshot_save_context(struct pt_regs *regs, bool stack_dump)
{
	int cpu;
	unsigned long flags;

	if (!dbg_snapshot_get_enable())
		return;

	cpu = raw_smp_processor_id();
	raw_spin_lock_irqsave(&dss_desc.ctrl_lock, flags);

	/* If it was already saved the context information, it should be skipped */
	if (dbg_snapshot_get_core_panic_stat(cpu) !=  DSS_SIGN_PANIC) {
		dbg_snapshot_set_core_panic_stat(DSS_SIGN_PANIC, cpu);
		dbg_snapshot_save_system(NULL);
		dbg_snapshot_save_core(regs);
		dbg_snapshot_ecc_dump(false);
		dev_emerg(dss_desc.dev, "context saved(CPU:%d)\n", cpu);
	} else
		dev_emerg(dss_desc.dev, "skip context saved(CPU:%d)\n", cpu);

	if (stack_dump)
		dump_stack();
	raw_spin_unlock_irqrestore(&dss_desc.ctrl_lock, flags);

	cache_flush_all();
}
EXPORT_SYMBOL_GPL(dbg_snapshot_save_context);

void dbg_snapshot_do_dpm_policy(unsigned int policy, const char *str)
{
	switch (policy) {
	case GO_DEFAULT_ID:
		pr_emerg("%s: %s\n", __func__, str);
		pr_emerg("no-op\n");
		break;
	case GO_PANIC_ID:
		panic("%s: %s", __func__, str);
		break;
	case GO_WATCHDOG_ID:
	case GO_S2D_ID:
		dbg_snapshot_emergency_reboot(str);
		break;
	case GO_ARRAYDUMP_ID:
		pr_emerg("%s: %s\n", __func__, str);
		pr_emerg("Entering Arraydump Mode!\n");
		if (dss_soc_ops.run_arraydump)
			dss_soc_ops.run_arraydump();
		break;
	case GO_SCANDUMP_ID:
		pr_emerg("%s: %s\n", __func__, str);
		pr_emerg("Entering Scandump Mode!\n");
		if (dss_soc_ops.run_scandump_mode)
			dss_soc_ops.run_scandump_mode();
		break;
	case GO_HALT_ID:
		pr_emerg("%s: %s\n", __func__, str);
		pr_emerg("Entering Halt Mode!\n");
		if (dss_soc_ops.stop_all_cpus)
			dss_soc_ops.stop_all_cpus();
		break;
	}
}
EXPORT_SYMBOL_GPL(dbg_snapshot_do_dpm_policy);

static struct die_args *tombstone;

static int dbg_snapshot_panic_handler(struct notifier_block *nb,
				   unsigned long l, void *buf)
{
	char *kernel_panic_msg;
	void *pc_tombstone;
	void *lr_tombstone;
	unsigned long cpu;

	if (!dbg_snapshot_get_enable())
		return 0;

	dss_desc.in_panic = true;

	kernel_panic_msg = kmalloc(DSS_PANIC_STRING_SZ, GFP_ATOMIC);
	if (!kernel_panic_msg)
		return -ENOMEM;

	if (!tombstone) {
		scnprintf(kernel_panic_msg, DSS_PANIC_STRING_SZ, "KP: %s",
				(char *)buf);
		goto panic_msg_ready;
	}

	/* tamper the panic message for Oops */
#if IS_ENABLED(CONFIG_ARM)
	pc_tombstone = (void *)tombstone->regs->ARM_pc;
	lr_tombstone = (void *)tombstone->regs->ARM_lr;
#elif IS_ENABLED(CONFIG_ARM64)
	pc_tombstone = (void *)tombstone->regs->pc;
	lr_tombstone = (void *)tombstone->regs->regs[30];
#endif

	scnprintf(kernel_panic_msg, DSS_PANIC_STRING_SZ,
			"KP: %s: comm:%s PC:%pSR LR:%pSR",
			(char *)buf, current->comm, pc_tombstone, lr_tombstone);

panic_msg_ready:
	/* Again disable log_kevents */
	dbg_snapshot_set_item_enable("log_kevents", false);
	dbg_snapshot_dump_panic(kernel_panic_msg, strlen(kernel_panic_msg));
	dbg_snapshot_report_reason(DSS_SIGN_PANIC);
	dbg_snapshot_set_reboot_mode(REBOOT_WARM);
	for_each_possible_cpu(cpu) {
		if (cpu_is_offline(cpu))
			dbg_snapshot_set_core_power_stat(DSS_SIGN_DEAD, cpu);
		else
			dbg_snapshot_set_core_power_stat(DSS_SIGN_ALIVE, cpu);
	}

	dbg_snapshot_dump_task_info();
	dbg_snapshot_output();
	dbg_snapshot_log_output();
	dbg_snapshot_print_log_report();
	dbg_snapshot_save_context(NULL, false);

	dbg_snapshot_do_dpm_policy(dss_desc.panic_action, kernel_panic_msg);

	if (num_online_cpus() > 1)
		dbg_snapshot_emergency_reboot(kernel_panic_msg);

	kfree(kernel_panic_msg);
	return 0;
}

static int dbg_snapshot_die_handler(struct notifier_block *nb,
				   unsigned long l, void *data)
{
	static struct die_args args;

	memcpy(&args, data, sizeof(args));
	tombstone = &args;

	if (user_mode(tombstone->regs))
		return NOTIFY_DONE;

	dbg_snapshot_save_context(tombstone->regs, false);
	dbg_snapshot_set_item_enable("log_kevents", false);

	return NOTIFY_DONE;
}

static int dbg_snapshot_reboot_handler(struct notifier_block *nb,
				    unsigned long mode, void *cmd)
{
	dss_desc.in_reboot = true;

	if (mode == SYS_POWER_OFF)
		dbg_snapshot_report_reason(DSS_SIGN_NORMAL_REBOOT);

	return NOTIFY_DONE;
}

static int dbg_snapshot_restart_handler(struct notifier_block *nb,
				    unsigned long mode, void *cmd)
{
	int cpu;

	if (!dbg_snapshot_get_enable())
		return NOTIFY_DONE;

	if (dss_desc.in_panic)
		return NOTIFY_DONE;

	if (dss_desc.in_warm) {
		dev_emerg(dss_desc.dev, "warm reset\n");
		dbg_snapshot_report_reason(DSS_SIGN_WARM_REBOOT);
		dbg_snapshot_set_reboot_mode(REBOOT_WARM);
		dbg_snapshot_dump_task_info();
	} else if (dss_desc.in_reboot) {
		dev_emerg(dss_desc.dev, "normal reboot starting\n");
		dbg_snapshot_report_reason(DSS_SIGN_NORMAL_REBOOT);
	} else {
		dev_emerg(dss_desc.dev, "emergency restart\n");
		dbg_snapshot_report_reason(DSS_SIGN_EMERGENCY_REBOOT);
		dbg_snapshot_set_reboot_mode(REBOOT_WARM);
		dbg_snapshot_dump_task_info();
	}

	dbg_snapshot_scratch_clear();

	/* clear DSS_SIGN_PANIC when normal reboot */
	for_each_possible_cpu(cpu) {
		dbg_snapshot_set_core_panic_stat(DSS_SIGN_RESET, cpu);
	}

	cache_flush_all();

	return NOTIFY_DONE;
}

static struct notifier_block nb_reboot_block = {
	.notifier_call = dbg_snapshot_reboot_handler,
	.priority = INT_MAX,
};

static struct notifier_block nb_restart_block = {
	.notifier_call = dbg_snapshot_restart_handler,
	.priority = INT_MAX,
};

static struct notifier_block nb_panic_block = {
	.notifier_call = dbg_snapshot_panic_handler,
	.priority = INT_MIN,
};

static struct notifier_block nb_die_block = {
	.notifier_call = dbg_snapshot_die_handler,
	.priority = INT_MAX,
};

void dbg_snapshot_register_wdt_ops(int (*start)(bool, int, int),
				   int (*expire)(unsigned int, int),
				   int (*stop)(int))
{
	if (start)
		dss_soc_ops.start_watchdog = start;
	if (expire)
		dss_soc_ops.expire_watchdog = expire;
	if (stop)
		dss_soc_ops.stop_watchdog = stop;

	dev_info(dss_desc.dev, "Add %s%s%s functions from %pS\n",
			start ? "wdt start, " : "",
			expire ? "wdt expire, " : "",
			stop ? "wdt stop" : "",
			return_address(0));
}
EXPORT_SYMBOL_GPL(dbg_snapshot_register_wdt_ops);

void dbg_snapshot_register_debug_ops(int (*halt)(void),
				     int (*arraydump)(void),
				     int (*scandump)(void))
{
	if (halt)
		dss_soc_ops.stop_all_cpus = halt;
	if (arraydump)
		dss_soc_ops.run_arraydump = arraydump;
	if (scandump)
		dss_soc_ops.run_scandump_mode = scandump;

	dev_info(dss_desc.dev, "Add %s%s%s functions from %pS\n",
			halt ? "halt, " : "",
			arraydump ? "arraydump, " : "",
			scandump ? "scandump mode" : "",
			return_address(0));
}
EXPORT_SYMBOL_GPL(dbg_snapshot_register_debug_ops);

static void dbg_snapshot_ipi_stop(void *ignore, struct pt_regs *regs)
{
	if (!dss_desc.in_reboot)
		dbg_snapshot_save_context(regs, false);
}

void dbg_snapshot_init_utils(void)
{
	size_t vaddr;
	uintptr_t i;

	vaddr = dss_items[DSS_ITEM_HEADER_ID].entry.vaddr;

	dss_mmu_reg = alloc_percpu(struct dbg_snapshot_mmu_reg *);
	dss_core_reg = alloc_percpu(struct pt_regs *);
	for_each_possible_cpu(i) {
		*per_cpu_ptr(dss_mmu_reg, i) = (struct dbg_snapshot_mmu_reg *)
					  (vaddr + DSS_HDR_INFO_BLOCK_SZ +
					   i * DSS_SYSREG_PER_CORE_SZ);
		*per_cpu_ptr(dss_core_reg, i) = (struct pt_regs *)
					   (vaddr + DSS_HDR_INFO_BLOCK_SZ + DSS_HDR_SYSREG_SZ +
					    i * DSS_COREREG_PER_CORE_SZ);
	}
	/* write default reboot reason as unknown reboot */
	dbg_snapshot_report_reason(DSS_SIGN_UNKNOWN_REBOOT);

	/* write reset value to skip abl dump as debug boot */
	dbg_snapshot_set_abl_dump_stat(DSS_SIGN_RESET);

	register_die_notifier(&nb_die_block);
	register_restart_handler(&nb_restart_block);
	register_reboot_notifier(&nb_reboot_block);
	atomic_notifier_chain_register(&panic_notifier_list, &nb_panic_block);
	register_trace_android_vh_ipi_stop(dbg_snapshot_ipi_stop, NULL);

	smp_call_function(dbg_snapshot_save_system, NULL, 1);
	dbg_snapshot_save_system(NULL);
}

int dbg_snapshot_stop_all_cpus(void)
{
	if (dss_soc_ops.stop_all_cpus)
		return dss_soc_ops.stop_all_cpus();

	return -ENODEV;
}
EXPORT_SYMBOL_GPL(dbg_snapshot_stop_all_cpus);
