// SPDX-License-Identifier: GPL-2.0

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/percpu.h>
#include <linux/time.h>
#include <linux/sched/debug.h>
#include <linux/spinlock.h>
#include <linux/smc.h>
#include <linux/smp.h>
#include <linux/soc/samsung/exynos-smc.h>

#include <asm/debug-monitors.h>
#include <asm/ptrace.h>
#include <asm/memory.h>
#include <asm/cacheflush.h>
#include <asm/bug.h>
#include <asm/stacktrace.h>

#include <soc/google/debug-snapshot.h>

#define HARDLOCKUP_DEBUG_MAGIC		(0xDEADBEEF)
#define BUG_BRK_IMM_HARDLOCKUP		(0x801)

struct hardlockup_debug_param {
	unsigned long last_pc_addr;
	unsigned long spin_pc_addr;
} hardlockup_debug_param;

static raw_spinlock_t hardlockup_seq_lock;
static raw_spinlock_t hardlockup_log_lock;
static int watchdog_fiq;
static int allcorelockup_detected;
static unsigned long hardlockup_core_mask;

static void hardlockup_debug_bug_func(void)
{
	do {
		asm volatile (__stringify(
				__BUG_ENTRY(0);
				brk	BUG_BRK_IMM_HARDLOCKUP));
		unreachable();
	} while (0);
}

static void hardlockup_debug_disable_fiq(void)
{
	asm volatile (__stringify(msr	daifset, #0x1));
}

static void hardlockup_debug_spin_func(void)
{
	do {
		wfi();
	} while(1);
}

static inline int hardlockup_debug_try_lock_timeout(raw_spinlock_t *lock,
								long timeout)
{
	int ret;

	do {
		ret = do_raw_spin_trylock(lock);
		if (!ret)
			udelay(1);
	} while(!ret && (timeout--) > 0);

	return ret;
}

static int hardlockup_debug_bug_handler(struct pt_regs *regs, unsigned int esr)
{
	int cpu = raw_smp_processor_id();
	unsigned int val;
	int ret;

	hardlockup_debug_disable_fiq();

	ret = hardlockup_debug_try_lock_timeout(&hardlockup_seq_lock,
						500 * USEC_PER_MSEC);
	if (ret && !hardlockup_core_mask) {
		if (watchdog_fiq && !allcorelockup_detected) {
			/* 1st WDT FIQ trigger */
			val = dbg_snapshot_get_hardlockup_magic();
			if (val == HARDLOCKUP_DEBUG_MAGIC ||
				val == (HARDLOCKUP_DEBUG_MAGIC + 1)) {
				int _cpu;

				allcorelockup_detected = 1;
				hardlockup_core_mask = 0;
				for_each_possible_cpu(_cpu)
					hardlockup_core_mask |= (1 << _cpu);
			} else {
				pr_emerg("%s: invalid magic from "
					"el3 fiq handler\n", __func__);
				raw_spin_unlock(&hardlockup_seq_lock);
				return DBG_HOOK_ERROR;
			}
		}
	}
	if (ret)
		raw_spin_unlock(&hardlockup_seq_lock);
	else
		pr_emerg("%s: fail to get seq lock\n", __func__);

	/* We expect this bug executed on only lockup core */
	if (hardlockup_core_mask & BIT(cpu)) {
		unsigned long last_pc;

		/* Replace real pc value even if it is invalid */
		last_pc = dbg_snapshot_get_last_pc(cpu);
		regs->pc = last_pc;

		ret = hardlockup_debug_try_lock_timeout(&hardlockup_log_lock,
							5 * USEC_PER_SEC);
		if (!ret)
			pr_emerg("%s: fail to get log lock\n", __func__);

		pr_emerg("%s - Debugging Information for Hardlockup core(%d) -"
			" locked CPUs mask (0x%lx)\n",
			allcorelockup_detected ? "All Core" : "Core", cpu,
			hardlockup_core_mask);
		dump_backtrace(regs, NULL);
		dbg_snapshot_save_context(regs);

		if (ret)
			raw_spin_unlock(&hardlockup_log_lock);

		/* If cpu is locked, wait for WDT reset without executing
		 * code anymore.
		 */
		hardlockup_debug_spin_func();
	}

	pr_emerg("%s: Unintended fiq handling\n", __func__);
	return DBG_HOOK_ERROR;
}

static struct break_hook hardlockup_debug_break_hook = {
	.fn = hardlockup_debug_bug_handler,
	.imm = BUG_BRK_IMM_HARDLOCKUP,
};

static int hardlockup_debug_panic_handler(struct notifier_block *nb,
					  unsigned long l, void *buf)
{
	int cpu;
	unsigned long locked_up_mask = 0;
#ifdef SMC_CMD_KERNEL_PANIC_NOTICE
	unsigned long last_pc_addr, timeout;
	int ret;
#endif

	if (allcorelockup_detected)
		return NOTIFY_OK;

	/* Assume that at this stage, CPUs that are still online
	 * (other than the panic-ing CPU) are locked up.
	 */
	for_each_possible_cpu (cpu) {
		if (cpu != raw_smp_processor_id() && cpu_online(cpu))
			locked_up_mask |= (1 << cpu);
	}

	pr_emerg("Hardlockup CPU mask: 0x%lx\n", locked_up_mask);

#ifdef SMC_CMD_KERNEL_PANIC_NOTICE
	last_pc_addr = dbg_snapshot_get_last_pc_paddr();
	if (locked_up_mask && last_pc_addr) {
		hardlockup_core_mask = locked_up_mask;

		/* Setup for generating NMI interrupt to unstopped CPUs */
		ret = exynos_smc(SMC_CMD_KERNEL_PANIC_NOTICE,
				locked_up_mask,
				(unsigned long)hardlockup_debug_bug_func,
				last_pc_addr);

		if (ret) {
			pr_emerg("Failed to generate NMI for hardlockup, "
				"not support to dump information of core\n");
			locked_up_mask = 0;
		}
	}

	/*  Wait up to 3 seconds for NMI interrupt */
	timeout = USEC_PER_SEC * 3;
	while (locked_up_mask != 0 && timeout--)
		udelay(1);
#endif
	return NOTIFY_OK;
}

static struct notifier_block hardlockup_debug_panic_nb = {
	.notifier_call = hardlockup_debug_panic_handler,
};

static int __init hardlockup_debugger_init(void)
{
	int ret = -1;
	struct device_node *node;

	node = of_find_node_by_name(NULL, "dss");
	if (!node) {
		pr_info("Failed to find debug device tree node\n");
	} else {
		ret = of_property_read_u32(node, "use_multistage_wdt_irq",
						&watchdog_fiq);
		if (ret)
			pr_info("Multistage watchdog is not supported\n");
	}

	if (!ret) {
		hardlockup_debug_param.last_pc_addr =
					dbg_snapshot_get_last_pc_paddr();
		if (hardlockup_debug_param.last_pc_addr) {
			hardlockup_debug_param.spin_pc_addr =
					(unsigned long)virt_to_phys(
					&hardlockup_debug_spin_func);
			__flush_dcache_area((void *)&hardlockup_debug_param,
					sizeof(struct hardlockup_debug_param));
#ifdef SMC_CMD_LOCKUP_NOTICE
			ret = exynos_smc(SMC_CMD_LOCKUP_NOTICE,
				(unsigned long)hardlockup_debug_bug_func,
				watchdog_fiq,
				(unsigned long)(virt_to_phys)(
				&hardlockup_debug_param));
#else
			ret = -EINVAL;
#endif
		} else {
			ret = -ENOMEM;
		}
	}
	pr_info("%s to register all-core lockup detector - ret: %d\n",
				(ret == 0) ? "success" : "failed", ret);


	raw_spin_lock_init(&hardlockup_seq_lock);
	raw_spin_lock_init(&hardlockup_log_lock);

	atomic_notifier_chain_register(&panic_notifier_list,
				       &hardlockup_debug_panic_nb);

	register_kernel_break_hook(&hardlockup_debug_break_hook);

	pr_info("Initialized hardlockup debug dump successfully.\n");

	return 0;
}

module_init(hardlockup_debugger_init);

MODULE_DESCRIPTION("Module for Debugging Hardlockups via FIQ");
MODULE_LICENSE("GPL v2");
