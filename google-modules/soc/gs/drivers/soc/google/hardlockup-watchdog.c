// SPDX-License-Identifier: GPL-2.0

#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/cpu.h>
#include <linux/cpuhotplug.h>
#include <linux/suspend.h>
#include <linux/sched/clock.h>
#include <linux/preempt.h>
#include <uapi/linux/sched/types.h>

struct hardlockup_watchdog_pcpu {
	unsigned long hardlockup_touch_ts;
	struct hrtimer hrtimer;
	unsigned long hrtimer_interrupts;
	unsigned long hrtimer_interrupts_saved;
	bool hard_watchdog_warn;
};

static struct hardlockup_watchdog_pcpu __percpu *hardlockup_watchdog_pcpu;

struct hardlockup_watchdog_desc {
	struct mutex mutex;
	unsigned long enabled;
	struct cpumask allowed_mask;
	u64 sample_period;
	unsigned long thresh;
	u32 sampling_time;
	u32 opportunity_cnt;
	u32 panic;
};

struct hardlockup_watchdog_dev {
	struct device *dev;
	struct hardlockup_watchdog_desc *desc;
	const struct of_device_id *match;
};

static struct hardlockup_watchdog_desc hardlockup_watchdog;

static unsigned long hardlockup_panic = 1;
module_param(hardlockup_panic, ulong, 0664);

static const struct of_device_id hardlockup_watchdog_dt_match[] = {
	{.compatible = "google,hardlockup-watchdog",
	 .data = NULL,},
	{},
};
MODULE_DEVICE_TABLE(of, hardlockup_watchdog_dt_match);

ATOMIC_NOTIFIER_HEAD(hardlockup_notifier_list);
EXPORT_SYMBOL_GPL(hardlockup_notifier_list);

ATOMIC_NOTIFIER_HEAD(hardlockup_handler_notifier_list);
EXPORT_SYMBOL_GPL(hardlockup_handler_notifier_list);

/*
 * Returns seconds, approximately.  We don't need nanosecond
 * resolution, and we don't need to waste time with a big divide when
 * 2^30ns == 1.074s.
 */
static unsigned long get_timestamp(void)
{
	return local_clock() >> 30;
}

static void set_hardlockup_watchdog_sample_period(void)
{
	hardlockup_watchdog.sample_period =
		hardlockup_watchdog.sampling_time * (u64)NSEC_PER_SEC;
	hardlockup_watchdog.thresh =
			hardlockup_watchdog.sampling_time *
			hardlockup_watchdog.opportunity_cnt;
}

/* Commands for resetting the watchdog */
static void __touch_hardlockup_watchdog(void)
{
	struct hardlockup_watchdog_pcpu *pcpu_val =
				this_cpu_ptr(hardlockup_watchdog_pcpu);

	if (pcpu_val)
		pcpu_val->hardlockup_touch_ts = get_timestamp();
}

static void watchdog_interrupt_count(void)
{
	struct hardlockup_watchdog_pcpu *pcpu_val =
				this_cpu_ptr(hardlockup_watchdog_pcpu);

	if (pcpu_val)
		pcpu_val->hrtimer_interrupts++;
}

static unsigned int watchdog_next_cpu(unsigned int cpu)
{
	unsigned int next_cpu;

	next_cpu = cpumask_next(cpu, &hardlockup_watchdog.allowed_mask);
	if (next_cpu >= nr_cpu_ids)
		next_cpu = cpumask_first(&hardlockup_watchdog.allowed_mask);

	if (next_cpu == cpu)
		return nr_cpu_ids;

	return next_cpu;
}

static int is_hardlockup_other_cpu(unsigned int cpu)
{
	struct hardlockup_watchdog_pcpu *pcpu_val =
				per_cpu_ptr(hardlockup_watchdog_pcpu, cpu);

	unsigned long hrint;

	if (!pcpu_val)
		goto out;

	hrint = pcpu_val->hrtimer_interrupts;

	if (pcpu_val->hrtimer_interrupts_saved == hrint) {
		unsigned long now = get_timestamp();
		unsigned long touch_ts = pcpu_val->hardlockup_touch_ts;

		if (time_after(now, touch_ts) &&
			(now - touch_ts >= hardlockup_watchdog.thresh)) {
			pr_err("hardlockup-watchdog: cpu%x: hrint:%lu thresh:%lu, now:%lu, touch_ts:%lu\n",
				cpu, hrint, hardlockup_watchdog.thresh,
				now, touch_ts);

			return 1;
		}
	}
	pcpu_val->hrtimer_interrupts_saved = hrint;
out:
	return 0;
}

static void watchdog_check_hardlockup_other_cpu(void)
{
	struct hardlockup_watchdog_pcpu *pcpu_val =
				this_cpu_ptr(hardlockup_watchdog_pcpu);
	struct hardlockup_watchdog_pcpu *pcpu_val_next;
	int opportunity_cnt = hardlockup_watchdog.opportunity_cnt;
	unsigned int next_cpu;

	if (!pcpu_val)
		return;

	if ((pcpu_val->hrtimer_interrupts) % opportunity_cnt != 0)
		return;

	/* check for a hardlockup on the next cpu */
	next_cpu = watchdog_next_cpu(smp_processor_id());
	if (next_cpu >= nr_cpu_ids)
		return;

	smp_rmb();

	pcpu_val_next = per_cpu_ptr(hardlockup_watchdog_pcpu, next_cpu);
	if (is_hardlockup_other_cpu(next_cpu)) {
		/* only warn once */
		if (pcpu_val_next->hard_watchdog_warn == true)
			return;

		if (hardlockup_watchdog.panic) {
			atomic_notifier_call_chain(&hardlockup_notifier_list, 0, (void *)&next_cpu);
			panic("Watchdog detected hard LOCKUP on cpu %u", next_cpu);
		} else {
			WARN(1, "Watchdog detected hard LOCKUP on cpu %u", next_cpu);
		}

		pcpu_val_next->hard_watchdog_warn = true;
	} else {
		pcpu_val_next->hard_watchdog_warn = false;
	}
}

/* watchdog kicker functions */
static enum hrtimer_restart hardlockup_watchdog_fn(struct hrtimer *hrtimer)
{
	int cpu = raw_smp_processor_id();

	if (!hardlockup_watchdog.enabled)
		return HRTIMER_NORESTART;

	/* kick the hardlockup detector */
	watchdog_interrupt_count();

	/* test for hardlockups on the next cpu */
	watchdog_check_hardlockup_other_cpu();

	__touch_hardlockup_watchdog();

	atomic_notifier_call_chain(&hardlockup_handler_notifier_list, 0, (void *)&cpu);

	/* .. and repeat */
	hrtimer_forward_now(hrtimer, ns_to_ktime(hardlockup_watchdog.sample_period));

	return HRTIMER_RESTART;
}

static void hardlockup_watchdog_enable(unsigned int cpu)
{
	struct hardlockup_watchdog_pcpu *pcpu_val =
				per_cpu_ptr(hardlockup_watchdog_pcpu, cpu);
	struct hrtimer *hrtimer;

	if (!pcpu_val)
		return;

	cpumask_set_cpu(cpu, &hardlockup_watchdog.allowed_mask);
	hrtimer = &pcpu_val->hrtimer;

	hrtimer_init(hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hrtimer->function = hardlockup_watchdog_fn;
	hrtimer_start(hrtimer, ns_to_ktime(hardlockup_watchdog.sample_period),
		      HRTIMER_MODE_REL_PINNED);

	/* Initialize timestamp */
	__touch_hardlockup_watchdog();

	pr_debug("%s: cpu%x: enabled - interval: %llu sec\n", __func__, cpu,
			hardlockup_watchdog.sample_period / NSEC_PER_SEC);
}

static void hardlockup_watchdog_disable(unsigned int cpu)
{
	struct hardlockup_watchdog_pcpu *pcpu_val =
				per_cpu_ptr(hardlockup_watchdog_pcpu, cpu);
	struct hrtimer *hrtimer;

	if (!pcpu_val)
		return;

	hrtimer = &pcpu_val->hrtimer;

	WARN_ON_ONCE(cpu != smp_processor_id());

	pr_debug("%s: cpu%x: disabled\n", __func__, cpu);

	cpumask_clear_cpu(cpu, &hardlockup_watchdog.allowed_mask);
	hrtimer_cancel(hrtimer);
}

static int hardlockup_stop_fn(void *data)
{
	hardlockup_watchdog_disable(smp_processor_id());
	return 0;
}

static void hardlockup_stop_all(void)
{
	int cpu;

	for_each_cpu(cpu, &hardlockup_watchdog.allowed_mask)
		smp_call_on_cpu(cpu, hardlockup_stop_fn, NULL, false);

	cpumask_clear(&hardlockup_watchdog.allowed_mask);
}

static int hardlockup_start_fn(void *data)
{
	hardlockup_watchdog_enable(smp_processor_id());
	return 0;
}

static void hardlockup_start_all(void)
{
	int cpu;

	for_each_online_cpu(cpu)
		smp_call_on_cpu(cpu, hardlockup_start_fn, NULL, false);
}

static int hardlockup_watchdog_online_cpu(unsigned int cpu)
{
	if (!hardlockup_watchdog.sample_period)
		return 0;

	smp_call_on_cpu(cpu, hardlockup_start_fn, NULL, false);
	return 0;
}

static int hardlockup_watchdog_offline_cpu(unsigned int cpu)
{
	if (!cpumask_test_cpu(cpu, &hardlockup_watchdog.allowed_mask))
		return 0;

	smp_call_on_cpu(cpu, hardlockup_stop_fn, NULL, false);
	return 0;
}

static void hardlockup_watchdog_reconfigure(void)
{
	cpus_read_lock();
	hardlockup_stop_all();
	set_hardlockup_watchdog_sample_period();
	if (hardlockup_watchdog.enabled)
		hardlockup_start_all();

	cpus_read_unlock();
}

static void hardlockup_watchdog_setup(void)
{
	int err;

	if (!(hardlockup_watchdog.enabled))
		return;

	mutex_init(&hardlockup_watchdog.mutex);
	hardlockup_watchdog_pcpu =
			alloc_percpu(struct hardlockup_watchdog_pcpu);

	if (!hardlockup_watchdog_pcpu) {
		pr_err("%s: alloc_percpu is failed\n", __func__);
		return;
	}

	err = cpuhp_setup_state(CPUHP_AP_ONLINE_DYN,
			"hardlockup:online",
			hardlockup_watchdog_online_cpu,
			hardlockup_watchdog_offline_cpu);

	if (err < 0) {
		pr_err("%s: cpuhotplug register is failed - err:%d\n", __func__, err);
		return;
	}

	mutex_lock(&hardlockup_watchdog.mutex);
	hardlockup_watchdog_reconfigure();
	mutex_unlock(&hardlockup_watchdog.mutex);
}

static void hardlockup_watchdog_cleanup(void)
{
	mutex_lock(&hardlockup_watchdog.mutex);
	hardlockup_stop_all();
	mutex_unlock(&hardlockup_watchdog.mutex);
}

static int hardlockup_set_property_by_dt_node(struct device *dev)
{
	struct device_node *np = dev->of_node;
	int ret;
	u32 reg;

	ret = of_property_read_u32(np, "sampling_time", &reg);
	if (ret)
		return ret;
	hardlockup_watchdog.sampling_time = reg;

	ret = of_property_read_u32(np, "opportunity_count", &reg);
	if (ret)
		return ret;
	hardlockup_watchdog.opportunity_cnt = reg;

	ret = of_property_read_u32(np, "panic", &reg);
	if (ret)
		return ret;
	hardlockup_watchdog.panic = reg && hardlockup_panic;

	hardlockup_watchdog.enabled = true;
	dev_info(dev, "sampling_time = %usec, opportunity_cnt = %u, panic = %u\n",
					hardlockup_watchdog.sampling_time,
					hardlockup_watchdog.opportunity_cnt,
					hardlockup_watchdog.panic);
	return ret;
}

static int hardlockup_watchdog_probe(struct platform_device *pdev)
{
	struct hardlockup_watchdog_dev *watchdog;
	int ret;

	watchdog = devm_kzalloc(&pdev->dev,
			sizeof(struct hardlockup_watchdog_dev), GFP_KERNEL);
	if (!watchdog)
		return -ENOMEM;

	watchdog->dev = &pdev->dev;
	watchdog->desc = &hardlockup_watchdog;

	platform_set_drvdata(pdev, watchdog);
	ret = hardlockup_set_property_by_dt_node(&pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "Fail to read properties\n");
		return ret;
	}

	hardlockup_watchdog_setup();

	dev_info(&pdev->dev, "Hardlockup Watchdog driver\n");

	return 0;
}

static int hardlockup_watchdog_remove(struct platform_device *pdev)
{
	platform_set_drvdata(pdev, NULL);
	hardlockup_watchdog_cleanup();
	free_percpu(hardlockup_watchdog_pcpu);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int hardlockup_watchdog_suspend(struct device *dev)
{
	hardlockup_watchdog_offline_cpu(0);
	return 0;
}

static int hardlockup_watchdog_resume(struct device *dev)
{
	hardlockup_watchdog_online_cpu(0);
	return 0;
}

static SIMPLE_DEV_PM_OPS(hardlockup_watchdog_pm_ops,
			hardlockup_watchdog_suspend,
			hardlockup_watchdog_resume);
#define HARDLOCKUP_WATCHDOG_PM	(hardlockup_watchdog_pm_ops)
#endif

static struct platform_driver hardlockup_watchdog_driver = {
	.probe = hardlockup_watchdog_probe,
	.remove = hardlockup_watchdog_remove,
	.driver = {
		   .name = "hardlockup-watchdog",
		   .of_match_table = hardlockup_watchdog_dt_match,
		   .pm = &hardlockup_watchdog_pm_ops,
		   },
};
module_platform_driver(hardlockup_watchdog_driver);

MODULE_DESCRIPTION("Module for Detecting Individual Core Lockup");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Hosung Kim <hosung0.kim@samsung.com>");
MODULE_AUTHOR("Changki Kim <changki.kim@samsung.com>");
MODULE_AUTHOR("Donghyeok Choe <d7271.choe@samsung.com>");
