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
#include <linux/trusty/smcall.h>
#include <asm/fiq_glue.h>

/* This handler runs at most once per hanged CPU */
static DEFINE_PER_CPU(bool, handled);
static raw_spinlock_t log_lock;

static int watchdog_fiq;

static int hardlockup_debug_panic_handler(struct notifier_block *nb,
					  unsigned long l, void *buf)
{
	int cpu;
	unsigned long locked_up_mask = 0;

	/* Assume that at this stage, CPUs that are still online
	 * (other than the panic-ing CPU) are locked up.
	 */
	for_each_possible_cpu (cpu) {
		if (cpu != raw_smp_processor_id() && cpu_online(cpu)) {
			locked_up_mask |= (1 << cpu);
			continue;
		}
		per_cpu(handled, cpu) = true;
	}

	pr_emerg("Hardlockup CPU mask: 0x%x\n", locked_up_mask);

	return NOTIFY_OK;
}

static struct notifier_block hardlockup_debug_panic_nb = {
	.notifier_call = hardlockup_debug_panic_handler,
};

static void fiq_handler(struct fiq_glue_handler *h, const struct pt_regs *regs,
			void *svc_sp)
{
	int rc;
	unsigned long timeout = USEC_PER_SEC * 13;
	int cpu = raw_smp_processor_id();
	bool *cpu_handled = &get_cpu_var(handled);
	if (*cpu_handled == false) {
		do {
			rc = do_raw_spin_trylock(&log_lock);
			if (!rc)
				udelay(1);
		} while (!rc && timeout--);

		pr_emerg(
			"\n-------------------------------------------------------------\n"
			"      Debugging Information for Hardlockup core - CPU(%d)"
			"\n-------------------------------------------------------------\n\n",
			cpu);
		show_regs((struct pt_regs *)regs);

		do_raw_spin_unlock(&log_lock);

		*cpu_handled = true;
	}
	put_cpu_var(handled);
}

static struct fiq_glue_handler handler = {
	.fiq = fiq_handler,
};

static int __init hardlockup_debugger_init(void)
{
	int rc;
	struct device_node *node;

	node = of_find_node_by_name(NULL, "dss");
	if (!node) {
		pr_err("Failed to find debug device tree node: %d\n", rc);
		goto err;
	}

	if (of_property_read_u32(node, "use_multistage_wdt_irq",
				 &watchdog_fiq)) {
		pr_err("No support for multistage watchdog\n");
		goto err;
	}

	rc = exynos_smc(SMC_FC_REQUEST_FIQ, watchdog_fiq, true, 0);
	if (rc != 0) {
		pr_err("Failed to enable watchdog FIQ: %d\n", rc);
		goto err;
	}

	rc = fiq_glue_register_handler(&handler);
	if (rc != 0) {
		pr_err("Failed to register FIQ handler: %d\n", rc);
		goto err;
	}

	raw_spin_lock_init(&log_lock);

	atomic_notifier_chain_register(&panic_notifier_list,
				       &hardlockup_debug_panic_nb);

	pr_info("Initialized hardlockup debug dump successfully.\n");
	return 0;

err:
	return -1;
}

module_init(hardlockup_debugger_init);
