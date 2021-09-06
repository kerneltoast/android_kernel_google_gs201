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
#include <linux/smc.h>
#include <linux/kdebug.h>
#include <linux/arm-smccc.h>

#include <asm/cputype.h>
#include <asm/smp_plat.h>
#include <asm/system_misc.h>
#include "system-regs.h"

#include <soc/google/debug-snapshot.h>
#include <soc/google/exynos-pmu-if.h>
#include "debug-snapshot-local.h"

#include <trace/hooks/debug.h>

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
	flush_cache_all();
}
EXPORT_SYMBOL_GPL(cache_flush_all);

static u64 read_errselr_el1(void)
{
	u64 reg;

	asm volatile ("mrs %0, S3_0_c5_c3_1\n" : "=r" (reg));
	return reg;
}

static void write_errselr_el1(u64 val)
{
	asm volatile ("msr S3_0_c5_c3_1, %0\n"
			"isb\n" :: "r" ((__u64)val));
}

static u64 read_erridr_el1(void)
{
	u64 reg;

	asm volatile ("mrs %0, S3_0_c5_c3_0\n" : "=r" (reg));
	return reg;
}

static u64 read_erxstatus_el1(void)
{
	u64 reg;

	asm volatile ("mrs %0, S3_0_c5_c4_2\n" : "=r" (reg));
	return reg;
}

static void __attribute__((unused)) write_erxstatus_el1(u64 val)
{
	asm volatile ("msr S3_0_c5_c4_2, %0\n"
			"isb\n" :: "r" ((__u64)val));
}

static u64 read_erxmisc0_el1(void)
{
	u64 reg;

	asm volatile ("mrs %0, S3_0_c5_c5_0\n" : "=r" (reg));
	return reg;
}

static u64 read_erxmisc1_el1(void)
{
	u64 reg;

	asm volatile ("mrs %0, S3_0_c5_c5_1\n" : "=r" (reg));
	return reg;
}

static u64 read_erxaddr_el1(void)
{
	u64 reg;

	asm volatile ("mrs %0, S3_0_c5_c4_3\n" : "=r" (reg));
	return reg;
}

static void dbg_snapshot_dump_panic(char *str, size_t len)
{
	/*  This function is only one which runs in panic function */
	if (str && len && len < DSS_PANIC_LOG_SIZE)
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

	return header ?  __raw_readl(header + DSS_OFFSET_PANIC_STAT + cpu * 4) : 0;
}

static void dbg_snapshot_set_core_panic_stat(unsigned int val, unsigned int cpu)
{
	void __iomem *header = dbg_snapshot_get_header_vaddr();

	if (header)
		__raw_writel(val, header + DSS_OFFSET_PANIC_STAT + cpu * 4);
}

static void dbg_snapshot_report_reason(unsigned int val)
{
	void __iomem *header = dbg_snapshot_get_header_vaddr();

	if (header)
		__raw_writel(val, header + DSS_OFFSET_EMERGENCY_REASON);
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
	char reboot_msg[DSS_PANIC_LOG_SIZE] = "Emergency Reboot";

	if (!dss_soc_ops.expire_watchdog) {
		dev_emerg(dss_desc.dev, "There is no wdt functions!\n");
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
		scnprintf(reboot_msg, sizeof(reboot_msg), str);

	dev_emerg(dss_desc.dev, "WDT Caller: %pS %s\n", addr, str ? str : "");

	dbg_snapshot_dump_panic(reboot_msg, strlen(reboot_msg));
	dump_stack();

	dss_soc_ops.expire_watchdog(tick, 0);
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
	char state_array[] = {'R', 'S', 'D', 'T', 't', 'X',
			'Z', 'P', 'x', 'K', 'W', 'I', 'N'};
	unsigned char idx = 0;
	unsigned long state, pc = 0;

	if ((!tsk) || !try_get_task_stack(tsk) || (tsk->flags & PF_FROZEN) ||
			!(tsk->state == TASK_RUNNING ||
				tsk->state == TASK_UNINTERRUPTIBLE ||
				tsk->state == TASK_KILLABLE))
		return;

	state = tsk->state | tsk->exit_state;
	pc = KSTK_EIP(tsk);
	while (state) {
		idx++;
		state >>= 1;
	}

	/*
	 * kick watchdog to prevent unexpected reset during panic sequence
	 * and it prevents the hang during panic sequence by watchedog
	 */
	touch_softlockup_watchdog();

	pr_info("%8d %16llu %16llu %16llu %c(%ld) %3d %16pK %16pK %c %16s\n",
		tsk->pid, tsk->utime, tsk->stime,
		tsk->se.exec_start, state_array[idx], (tsk->state),
		task_cpu(tsk), pc, tsk, is_main ? '*' : ' ', tsk->comm);

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
		"str x1, [%0, 0x60]\n\t" :	/* output */
		: "r"(mmu_reg)			/* input */
		: "x1", "x2", "memory"		/* clobbered register */
	);
}

void dbg_snapshot_ecc_dump(void)
{
	struct armv8_a_errselr_el1 errselr_el1;
	struct armv8_a_erridr_el1 erridr_el1;
	struct armv8_a_erxstatus_el1 erxstatus_el1;
	struct armv8_a_erxmisc0_el1 erxmisc0_el1;
	struct armv8_a_erxmisc1_el1 erxmisc1_el1;
	struct armv8_a_erxaddr_el1 erxaddr_el1;
	int i;

	switch (read_cpuid_part_number()) {
	case ARM_CPU_PART_CORTEX_A55:
	case ARM_CPU_PART_CORTEX_A76:
	case ARM_CPU_PART_CORTEX_A77:
	case ARM_CPU_PART_CORTEX_A78:
	case ARM_CPU_PART_CORTEX_X1:
		asm volatile ("HINT #16");
		erridr_el1.reg = read_erridr_el1();
		dev_emerg(dss_desc.dev, "ECC error check erridr_el1.num = 0x%llx\n",
				erridr_el1.field.num);

		for (i = 0; i < (int)erridr_el1.field.num; i++) {
			errselr_el1.reg = read_errselr_el1();
			errselr_el1.field.sel = i;
			write_errselr_el1(errselr_el1.reg);

			isb();

			erxstatus_el1.reg = read_erxstatus_el1();
			if (!erxstatus_el1.field.valid) {
				dev_emerg(dss_desc.dev,
					"ERRSELR_EL1.SEL = %d, NOT Error, ERXSTATUS_EL1 = 0x%llx\n",
					i, erxstatus_el1.reg);
				continue;
			}

			if (erxstatus_el1.field.av) {
				erxaddr_el1.reg = read_erxaddr_el1();
				dev_emerg(dss_desc.dev,
						"Error Address : 0x%llx\n", erxaddr_el1.reg);
			}
			if (erxstatus_el1.field.of)
				dev_emerg(dss_desc.dev,
					"There was more than one error has occurred. the other error have been discarded.\n");
			if (erxstatus_el1.field.er)
				dev_emerg(dss_desc.dev,	"Error Reported by external abort\n");
			if (erxstatus_el1.field.ue)
				dev_emerg(dss_desc.dev, "Uncorrected Error (Not deferred)\n");
			if (erxstatus_el1.field.de)
				dev_emerg(dss_desc.dev,	"Deferred Error\n");
			if (erxstatus_el1.field.mv) {
				erxmisc0_el1.reg = read_erxmisc0_el1();
				erxmisc1_el1.reg = read_erxmisc1_el1();
				dev_emerg(dss_desc.dev,
					"ERXMISC0_EL1 = 0x%llx ERXMISC1_EL1 = 0x%llx ERXSTATUS_EL1[15:8] = 0x%llx, [7:0] = 0x%llx\n",
					erxmisc0_el1.reg, erxmisc1_el1.reg,
					erxstatus_el1.field.ierr, erxstatus_el1.field.serr);
			}
		}
		break;
	default:
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
				"stp x29, x30, [x0, #0xe8]\n\t" :
				: "r"(core_reg));
		core_reg->sp = core_reg->regs[29];
		core_reg->pc =
			(unsigned long)(core_reg->regs[30] - sizeof(unsigned int));
	} else {
		memcpy(core_reg, regs, sizeof(struct user_pt_regs));
	}

	dev_emerg(dss_desc.dev, "core register saved(CPU:%d)\n", cpu);
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
		dbg_snapshot_ecc_dump();
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
	char kernel_panic_msg[DSS_PANIC_LOG_SIZE] = "Kernel Panic";
	unsigned long cpu;

	if (!dbg_snapshot_get_enable())
		return 0;

	dss_desc.in_panic = true;

	if (tombstone) { /* tamper the panic message for Oops */
		char pc_symn[KSYM_SYMBOL_LEN] = "<unknown>";
		char lr_symn[KSYM_SYMBOL_LEN] = "<unknown>";

#if defined(CONFIG_ARM)
		sprint_symbol(pc_symn, tombstone->regs->ARM_pc);
		sprint_symbol(lr_symn, tombstone->regs->ARM_lr);
#elif defined(CONFIG_ARM64)
		sprint_symbol(pc_symn, tombstone->regs->pc);
		sprint_symbol(lr_symn, tombstone->regs->regs[30]);
#endif
		scnprintf(kernel_panic_msg, sizeof(kernel_panic_msg),
				"KP: %s: comm:%s PC:%s LR:%s", (char *)buf,
				current->comm, pc_symn, lr_symn);
	} else {
		scnprintf(kernel_panic_msg, sizeof(kernel_panic_msg), "KP: %s",
				(char *)buf);
	}

	/* Again disable log_kevents */
	dbg_snapshot_set_item_enable("log_kevents", false);
	dbg_snapshot_dump_panic(kernel_panic_msg, strlen(kernel_panic_msg));
	dbg_snapshot_report_reason(DSS_SIGN_PANIC);
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

	if (dss_desc.in_reboot) {
		dev_emerg(dss_desc.dev, "normal reboot starting\n");
		dbg_snapshot_report_reason(DSS_SIGN_NORMAL_REBOOT);
	} else {
		dev_emerg(dss_desc.dev, "emergency restart\n");
		dbg_snapshot_report_reason(DSS_SIGN_EMERGENCY_REBOOT);
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

void dbg_snapshot_register_wdt_ops(void *start, void *expire, void *stop)
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

void dbg_snapshot_register_debug_ops(void *halt, void *arraydump,
				    void *scandump)
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
		dbg_snapshot_save_context(regs, true);
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
					  (vaddr + DSS_HEADER_SZ +
					   i * DSS_MMU_REG_OFFSET);
		*per_cpu_ptr(dss_core_reg, i) = (struct pt_regs *)
					   (vaddr + DSS_HEADER_SZ + DSS_MMU_REG_SZ +
					    i * DSS_CORE_REG_OFFSET);
	}
	/* write default reboot reason as unknown reboot */
	dbg_snapshot_report_reason(DSS_SIGN_UNKNOWN_REBOOT);

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
