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
	bool pause_enable;
	unsigned int tmu_type;
	int pause_threshold;
	int resume_threshold;
	bool hardlimit_enable;
	int hardlimit_threshold;
	int hardlimit_clr_threshold;
	unsigned int hardlimit_cooling_state;
	unsigned long max_cdev;
	bool hotplug_enable;
	int hotplug_in_threshold;
	int hotplug_out_threshold;
	bool cpu_hw_throttling_enable;
	int cpu_hw_throttling_trigger_threshold;
	int cpu_hw_throttling_clr_threshold;
	int ppm_clr_throttle_level;
	int ppm_throttle_level;
	int mpmm_clr_throttle_level;
	int mpmm_throttle_level;
	int limited_frequency;
	int limited_threshold;
	int limited_threshold_release;
	struct exynos_pm_qos_request thermal_limit_request;
	bool limited;
	void __iomem *base;
	int irq;
	struct kthread_worker hardlimit_worker;
	struct kthread_worker thermal_worker;
	struct kthread_worker pause_worker;
	struct kthread_worker cpu_hw_throttle_worker;
	struct kthread_work irq_work;
	struct kthread_work pause_work;
	struct kthread_work hardlimit_work;
	struct kthread_work hotplug_work;
	struct kthread_work cpu_hw_throttle_work;
	struct kthread_delayed_work cpu_hw_throttle_init_work;
	struct mutex lock;			/* lock to protect gs101 tmu */
	struct thermal_zone_device *tzd;
	struct bcl_device *bcl_dev;
	unsigned int ntrip;
	bool enabled;
	struct thermal_cooling_device *cool_dev;
	struct list_head node;
	char tmu_name[THERMAL_NAME_LENGTH + 1];
	struct device_node *np;
	bool is_paused;
	bool is_hardlimited;
	bool is_cpu_hotplugged_out;
	bool is_cpu_hw_throttled;
	int temperature;
	bool use_pi_thermal;
	struct kthread_delayed_work pi_work;
	struct gs101_pi_param *pi_param;
	struct cpumask pause_cpus;
	struct cpumask hotplug_cpus;
	struct cpumask tmu_work_affinity;
	struct cpumask hotplug_work_affinity;
	char cpuhp_name[CPUHP_USER_NAME_LEN + 1];
	void *disable_stats;
	void *hardlimit_stats;
};

enum throttling_stats_type {
	DISABLE_STATS = 0,
	HARDLIMIT_STATS,
};

struct throttling_stats {
	spinlock_t lock;
	int stats_type;
	unsigned int disable_total_count;
	unsigned int disable_state;
	unsigned int hardlimit_total_count;
	unsigned int hardlimit_state;
	ktime_t last_time;
	ktime_t *disable_time_in_state;
	ktime_t *hardlimit_time_in_state;
};

#define TMU_INTPEND_P0				0x00F8
#define TMU_INTPEND_REG(i)			(TMU_INTPEND_P0 + 0x50 * (i))

#define TMU_SENSOR_PROBE_NUM 16

enum tmu_zone_t {
	TMU_TOP = 0,
	TMU_SUB = 1,
	TMU_END = 2,
};

enum tmu_sensor_t {
	TMU_P0_SENSOR = 0,
	TMU_P1_SENSOR = 1,
	TMU_P2_SENSOR = 2,
	TMU_P3_SENSOR = 3,
	TMU_P4_SENSOR = 4,
	TMU_P5_SENSOR = 5,
	TMU_P6_SENSOR = 6,
	TMU_P7_SENSOR = 7,
	TMU_P8_SENSOR = 8,
	TMU_P9_SENSOR = 9,
	TMU_P10_SENSOR = 10,
	TMU_P11_SENSOR = 11,
	TMU_P12_SENSOR = 12,
	TMU_P13_SENSOR = 13,
	TMU_P14_SENSOR = 14,
	TMU_P15_SENSOR = 15,
};

#define TMU_P0_SENSOR_MASK (1 << TMU_P0_SENSOR)
#define TMU_P1_SENSOR_MASK (1 << TMU_P1_SENSOR)
#define TMU_P2_SENSOR_MASK (1 << TMU_P2_SENSOR)
#define TMU_P3_SENSOR_MASK (1 << TMU_P3_SENSOR)
#define TMU_P4_SENSOR_MASK (1 << TMU_P4_SENSOR)
#define TMU_P5_SENSOR_MASK (1 << TMU_P5_SENSOR)
#define TMU_P6_SENSOR_MASK (1 << TMU_P6_SENSOR)
#define TMU_P7_SENSOR_MASK (1 << TMU_P7_SENSOR)
#define TMU_P8_SENSOR_MASK (1 << TMU_P8_SENSOR)
#define TMU_P9_SENSOR_MASK (1 << TMU_P9_SENSOR)
#define TMU_P10_SENSOR_MASK (1 << TMU_P10_SENSOR)
#define TMU_P11_SENSOR_MASK (1 << TMU_P11_SENSOR)
#define TMU_P12_SENSOR_MASK (1 << TMU_P12_SENSOR)
#define TMU_P13_SENSOR_MASK (1 << TMU_P13_SENSOR)
#define TMU_P14_SENSOR_MASK (1 << TMU_P14_SENSOR)
#define TMU_P15_SENSOR_MASK (1 << TMU_P15_SENSOR)

struct sensor_data {
	enum tmu_sensor_t probe_id;
};

struct thermal_zone_data {
	enum tmu_zone_t tmu_zone_id;
	u16 sensors_mask;
	struct sensor_data sensors[TMU_SENSOR_PROBE_NUM];
	u16 sensor_cnt;
};

enum thermal_pause_state {
	THERMAL_RESUME = 0,
	THERMAL_SUSPEND,
};

typedef int (*tpu_pause_cb)(enum thermal_pause_state action, void *data);

void register_tpu_thermal_pause_cb(tpu_pause_cb tpu_cb, void *data);

#endif /* _GS101_TMU_H */
