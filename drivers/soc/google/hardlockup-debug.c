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
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/platform_device.h>
#include <linux/soc/samsung/exynos-smc.h>
#include <linux/slab.h>

#include <linux/suspend.h>
#include <linux/sched/task.h>
#include <trace/hooks/cpuidle.h>

#include <asm/debug-monitors.h>
#include <asm/ptrace.h>
#include <asm/memory.h>
#include <asm/cacheflush.h>
#include <asm/bug.h>
#include <asm/stacktrace.h>

#include <soc/google/debug-snapshot.h>
#if IS_ENABLED(CONFIG_GS_ACPM)
#include <soc/google/acpm_ipc_ctrl.h>
#endif

#define HARDLOCKUP_DEBUG_MAGIC		(0xDEADBEEF)
#define BUG_BRK_IMM_HARDLOCKUP		(0x801)
#define FIQ_PENDING_INST_INDEX		(ARRAY_SIZE(hardlockup_debug_cpu_resume_insts) - 1)

#define FIQINFO_CLUSTER_SHIFT		(0)
#define FIQINFO_CLUSTER_MASK		(0xff)
#define FIQINFO_CPU_ID_SHIFT		(8)
#define FIQINFO_CPU_ID_MASK		(0xff)

#define CLUSTER_0_CORE_NR		(6)

unsigned int hardlockup_debug_cpu_resume_insts[] = {
	0x100000a2, //    adr     x2, 14 <__fiq_pending>
	0xd53800a1, //    mrs     x1, mpidr_el1
	0xb2703c21, //    orr     x1, x1, #0xffff0000
	0xb9000041, //    str     w1, [x2]
	0xd61f0000, //    br      x0
	0x00000000, //    [FIQ_PENDING_INST_INDEX] = __fiq_pending:
};

struct hardlockup_param_type {
	unsigned long last_pc_addr;
	unsigned long spin_pc_addr;
	unsigned int spin_func[FIQ_PENDING_INST_INDEX + 1];
};

static struct hardlockup_param_type *hardlockup_param;
static dma_addr_t hardlockup_param_paddr;

static raw_spinlock_t hardlockup_seq_lock;
static raw_spinlock_t hardlockup_log_lock;
static int watchdog_fiq;
static int allcorelockup_detected;
static unsigned long hardlockup_core_mask;
static unsigned long hardlockup_core_handled_mask;

static struct task_struct *pm_suspend_task;
static DEFINE_SPINLOCK(pm_suspend_task_lock);

static void hardlockup_debug_bug_func(void)
{
	do {
		asm volatile (__stringify(
				__BUG_ENTRY(0);
				brk	BUG_BRK_IMM_HARDLOCKUP));
		unreachable();
	} while (0);
}

static int get_pending_fiq_cpu_id(void)
{
	int fiq_info;
	int cluster_id;
	int cpu_id;

	fiq_info = hardlockup_param->spin_func[FIQ_PENDING_INST_INDEX];
	cluster_id = (fiq_info >> FIQINFO_CLUSTER_SHIFT) & FIQINFO_CLUSTER_MASK;
	cpu_id = (fiq_info >> FIQINFO_CPU_ID_SHIFT) & FIQINFO_CPU_ID_MASK;

	return fiq_info ? (cluster_id * CLUSTER_0_CORE_NR + cpu_id) : -1;
}

static void vh_bug_on_wdt_fiq_pending(void *data, int state, struct cpuidle_device *dev)
{
	if (get_pending_fiq_cpu_id() == raw_smp_processor_id())
		hardlockup_debug_bug_func();
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
		ret = raw_spin_trylock(lock);
		if (!ret)
			udelay(1);
	} while(!ret && (timeout--) > 0);

	return ret;
}

static unsigned long hardlockup_debug_get_locked_cpu_mask(void)
{
	unsigned long mask = 0;
	unsigned int val;
	int cpu;

	for_each_online_cpu(cpu) {
		val = dbg_snapshot_get_hardlockup_magic(cpu);
		if (val == HARDLOCKUP_DEBUG_MAGIC ||
			val == (HARDLOCKUP_DEBUG_MAGIC + 1))
			mask |= (1 << cpu);
	}

	return mask;
}

static int hardlockup_debug_bug_handler(struct pt_regs *regs, unsigned int esr)
{
	int cpu = raw_smp_processor_id();
	unsigned int val;
	unsigned long flags;
	int ret;

	hardlockup_debug_disable_fiq();

	ret = hardlockup_debug_try_lock_timeout(&hardlockup_seq_lock,
						500 * USEC_PER_MSEC);
	if (ret && !hardlockup_core_mask) {
		if (watchdog_fiq && !allcorelockup_detected) {
			/* 1st WDT FIQ trigger */
			val = dbg_snapshot_get_hardlockup_magic(cpu);
			if (val == HARDLOCKUP_DEBUG_MAGIC ||
				val == (HARDLOCKUP_DEBUG_MAGIC + 1)) {
				allcorelockup_detected = 1;
				hardlockup_core_mask =
					hardlockup_debug_get_locked_cpu_mask();
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
		if (get_pending_fiq_cpu_id() != cpu)
			regs->pc = last_pc;

		ret = hardlockup_debug_try_lock_timeout(&hardlockup_log_lock,
							5 * USEC_PER_SEC);
		if (!ret)
			pr_emerg("%s: fail to get log lock\n", __func__);

		pr_emerg("%s - Debugging Information for Hardlockup core(%d) -"
			" locked CPUs mask (0x%lx)\n",
			allcorelockup_detected ? "WDT expired" : "Core", cpu,
			hardlockup_core_mask);

		dump_backtrace(regs, NULL, KERN_DEFAULT);
		dbg_snapshot_save_context(regs, false);

		if (ret)
			raw_spin_unlock(&hardlockup_log_lock);

		hardlockup_core_handled_mask |= (1 << cpu);

		if (hardlockup_core_mask == hardlockup_core_handled_mask) {
#if IS_ENABLED(CONFIG_GS_ACPM)
			exynos_acpm_reboot();
#endif
		}

		spin_lock_irqsave(&pm_suspend_task_lock, flags);
		if (pm_suspend_task) {
			pr_emerg("pm_suspend_task '%s' %d hung (state=%d)",
					pm_suspend_task->comm, pm_suspend_task->pid,
					pm_suspend_task->state);
			sched_show_task(pm_suspend_task);
			panic("PM suspend timeout");
		}
		spin_unlock_irqrestore(&pm_suspend_task_lock, flags);

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

static int hardlockup_debugger_pm_notifier(struct notifier_block *notifier,
				  unsigned long pm_event, void *v)
{
	unsigned long flags;

	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
		spin_lock_irqsave(&pm_suspend_task_lock, flags);
		pm_suspend_task = get_task_struct(current);
		spin_unlock_irqrestore(&pm_suspend_task_lock, flags);
		break;
	case PM_POST_SUSPEND:
		spin_lock_irqsave(&pm_suspend_task_lock, flags);
		put_task_struct(pm_suspend_task);
		pm_suspend_task = NULL;
		spin_unlock_irqrestore(&pm_suspend_task_lock, flags);
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block hardlockup_debugger_pm_nb = {
	.notifier_call = hardlockup_debugger_pm_notifier,
	.priority = 0,
};

static int hardlockup_debugger_probe(struct platform_device *pdev)
{
	int ret;
	struct device_node *node = pdev->dev.of_node;

	if (!node) {
		dev_info(&pdev->dev,
			"Failed to find debug device tree node\n");
		return -ENODEV;
	} else {
		ret = of_property_read_u32(node, "use_multistage_wdt_irq",
							&watchdog_fiq);
		if (ret) {
			dev_info(&pdev->dev,
				"Multistage watchdog is not supported\n");
			return ret;
		}
	}

	dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(36));
	hardlockup_param = dma_alloc_coherent(&pdev->dev,
					sizeof(struct hardlockup_param_type),
					&hardlockup_param_paddr, GFP_KERNEL);
	if (!hardlockup_param) {
		dev_err(&pdev->dev,
			"Fail to allocate memory for hardlockup_param\n");
		return -ENOMEM;
	}

	hardlockup_param->last_pc_addr = dbg_snapshot_get_last_pc_paddr();
	BUILD_BUG_ON(sizeof(hardlockup_debug_cpu_resume_insts) >
			sizeof(hardlockup_param->spin_func));
	memcpy(hardlockup_param->spin_func,
			hardlockup_debug_cpu_resume_insts,
			sizeof(hardlockup_param->spin_func));
	hardlockup_param->spin_pc_addr =
				(unsigned long)hardlockup_param_paddr +
				(unsigned long)offsetof(
				struct hardlockup_param_type, spin_func);

	if (hardlockup_param->last_pc_addr) {
#ifdef SMC_CMD_LOCKUP_NOTICE
		ret = exynos_smc(SMC_CMD_LOCKUP_NOTICE,
			(unsigned long)hardlockup_debug_bug_func,
			watchdog_fiq,
			(unsigned long)hardlockup_param_paddr);
		dev_info(&pdev->dev, "%s to register all-core lockup detector - ret: %d\n"
				, (ret == 0) ? "success" : "failed", ret);
#else
		ret = -EINVAL;
		goto error;
#endif
	} else {
		ret = -ENOMEM;
		goto error;
	}

	raw_spin_lock_init(&hardlockup_seq_lock);
	raw_spin_lock_init(&hardlockup_log_lock);

	atomic_notifier_chain_register(&panic_notifier_list,
				       &hardlockup_debug_panic_nb);

	register_kernel_break_hook(&hardlockup_debug_break_hook);

	register_pm_notifier(&hardlockup_debugger_pm_nb);

	WARN_ON(register_trace_android_vh_cpu_idle_exit(
				vh_bug_on_wdt_fiq_pending, NULL));

	dev_info(&pdev->dev,
			"Initialized hardlockup debug dump successfully.\n");
	return 0;
error:
	dma_free_coherent(&pdev->dev, sizeof(struct hardlockup_param_type),
			(void *)hardlockup_param, hardlockup_param_paddr);
	return ret;
}

static const struct of_device_id hardlockup_debug_dt_match[] = {
	{.compatible = "google,hardlockup-debug",
	 .data = NULL,},
	{},
};
MODULE_DEVICE_TABLE(of, hardlockup_debug_dt_match);

static struct platform_driver hardlockup_debug_driver = {
	.probe = hardlockup_debugger_probe,
	.driver = {
			.name = "hardlockup-debug-driver",
			.of_match_table = hardlockup_debug_dt_match,
		},
};
module_platform_driver(hardlockup_debug_driver);

MODULE_DESCRIPTION("Module for Debugging Hardlockups via FIQ");
MODULE_LICENSE("GPL v2");
