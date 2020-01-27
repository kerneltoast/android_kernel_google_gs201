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

/* This handler only runs once to avoid spamming the log */
static DEFINE_PER_CPU(bool, handle_once);
static raw_spinlock_t log_lock;

static int watchdog_fiq;

static void fiq_handler(struct fiq_glue_handler *h, const struct pt_regs *regs,
			void *svc_sp)
{
	int rc;
	unsigned long timeout = USEC_PER_SEC * 13;
	int cpu = raw_smp_processor_id();
	bool *handled = &get_cpu_var(handle_once);
	if (*handled == false) {
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

		*handled = true;
	}
	put_cpu_var(handle_once);
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

	pr_info("Initialized hardlockup debug dump successfully.\n");
	return 0;

err:
	return -1;
}

module_init(hardlockup_debugger_init);
