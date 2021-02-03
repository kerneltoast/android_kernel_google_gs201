/* SPDX-License-Identifier: GPL-2.0-only
 *
 * gs101_tmu.h - Samsung GS101 TMU (Thermal Management Unit)
 *
 *  Copyright (C) 2019 Samsung Electronics
 *  Hyeonseong Gil <hs.gill@samsung.com>
 */

#ifndef _GS101_TMU_H
#define _GS101_TMU_H
#include <linux/kthread.h>
#include <soc/google/exynos_pm_qos.h>
#include <soc/google/exynos-cpuhp.h>

#define MCELSIUS        1000

struct gs101_pi_param {
	s64 err_integral;
	int trip_switch_on;
	int trip_control_temp;

	u32 sustainable_power;
	s32 k_po;
	s32 k_pu;
	s32 k_i;
	s32 i_max;
	s32 integral_cutoff;

	int polling_delay_on;
	int polling_delay_off;

	bool switched_on;
};

/**
 * struct gs101_tmu_data : A structure to hold the private data of the TMU
	driver
 * @id: identifier of the one instance of the TMU controller.
 * @base: base address of the single instance of the TMU controller.
 * @irq: irq number of the TMU controller.
 * @soc: id of the SOC type.
 * @irq_work: pointer to the irq work structure.
 * @lock: lock to implement synchronization.
 * @regulator: pointer to the TMU regulator structure.
 * @reg_conf: pointer to structure to register with core thermal.
 * @ntrip: number of supported trip points.
 * @tmu_initialize: SoC specific TMU initialization method
 * @tmu_control: SoC specific TMU control method
 * @tmu_read: SoC specific TMU temperature read method
 * @tmu_set_emulation: SoC specific TMU emulation setting method
 * @tmu_clear_irqs: SoC specific TMU interrupts clearing method
 */
struct gs101_tmu_data {
	int id;
	/* Throttle hotplug related variables */
	bool hotplug_enable;
	int hotplug_in_threshold;
	int hotplug_out_threshold;
	int limited_frequency;
	int limited_threshold;
	int limited_threshold_release;
	struct exynos_pm_qos_request thermal_limit_request;
	bool limited;
	void __iomem *base;
	int irq;
	struct kthread_worker thermal_worker;
	struct kthread_work irq_work;
	struct kthread_work hotplug_work;
	struct mutex lock;			/* lock to protect gs101 tmu */
	struct thermal_zone_device *tzd;
	unsigned int ntrip;
	bool enabled;
	struct thermal_cooling_device *cool_dev;
	struct list_head node;
	char tmu_name[THERMAL_NAME_LENGTH + 1];
	struct device_node *np;
	bool is_cpu_hotplugged_out;
	int temperature;
	bool use_pi_thermal;
	struct kthread_delayed_work pi_work;
	struct gs101_pi_param *pi_param;
	struct cpumask hotplug_cpus;
	char cpuhp_name[CPUHP_USER_NAME_LEN + 1];
};

#endif /* _GS101_TMU_H */
