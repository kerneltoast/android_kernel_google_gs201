/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __BCL_H
#define __BCL_H

#if IS_ENABLED(CONFIG_GOOGLE_BCL)
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/thermal.h>
#include <linux/workqueue.h>


enum PMIC_THERMAL_SENSOR {
	PMIC_SOC,
	PMIC_SENSOR_MAX,
};

enum TRIGGERED_SOURCE {
	IRQ_SMPL_WARN,
	IRQ_OCP_WARN_CPUCL1,
	IRQ_OCP_WARN_CPUCL2,
	IRQ_SOFT_OCP_WARN_CPUCL1,
	IRQ_SOFT_OCP_WARN_CPUCL2,
	IRQ_OCP_WARN_TPU,
	IRQ_SOFT_OCP_WARN_TPU,
	IRQ_OCP_WARN_GPU,
	IRQ_SOFT_OCP_WARN_GPU,
	IRQ_PMIC_120C,
	IRQ_PMIC_140C,
	IRQ_PMIC_OVERHEAT,
	IRQ_TRIGGERED_SOURCE_MAX,
};

enum PMIC_REG { S2MPG10, S2MPG11 };

struct ocpsmpl_stats {
	ktime_t _time;
	int capacity;
	int voltage;
};

enum PMIC_SENSOR {
	VDROOP1,
	VDROOP2,
	BATOILO,
	IFPMIC_SENSOR_MAX,
};

struct bcl_device {
	struct device *device;
	struct device *mitigation_dev;
	void __iomem *base_mem[5];
	void __iomem *sysreg_cpucl0;
	struct power_supply *batt_psy;

	struct notifier_block psy_nb;
	struct delayed_work soc_eval_work;

	void *iodev;

	int trip_high_temp;
	int trip_low_temp;
	int trip_val;
	struct mutex state_trans_lock;
	struct mutex ratio_lock;
	struct thermal_zone_device *soc_tzd;
	struct thermal_zone_of_device_ops soc_ops;
	struct mutex triggered_irq_lock[IRQ_TRIGGERED_SOURCE_MAX];
	struct delayed_work triggered_irq_work[IRQ_TRIGGERED_SOURCE_MAX];
	struct thermal_zone_device *triggered_tz_irq[IRQ_TRIGGERED_SOURCE_MAX];

	unsigned int triggered_lvl[IRQ_TRIGGERED_SOURCE_MAX];
	unsigned int triggered_irq[IRQ_TRIGGERED_SOURCE_MAX];
	int triggered_counter[IRQ_TRIGGERED_SOURCE_MAX];
	int triggered_pin[IRQ_TRIGGERED_SOURCE_MAX];
	atomic_t triggered_cnt[IRQ_TRIGGERED_SOURCE_MAX];
	atomic_t if_triggered_cnt[IFPMIC_SENSOR_MAX];
	struct ocpsmpl_stats triggered_stats[IRQ_TRIGGERED_SOURCE_MAX];
	struct ocpsmpl_stats if_triggered_stats[IFPMIC_SENSOR_MAX];

	struct s2mpg10_dev *s2mpg10;
	struct s2mpg11_dev *s2mpg11;

	struct i2c_client *s2mpg10_i2c;
	struct i2c_client *s2mpg11_i2c;

	unsigned int tpu_con_heavy;
	unsigned int tpu_con_light;
	unsigned int gpu_con_heavy;
	unsigned int gpu_con_light;
	unsigned int tpu_clkdivstep;
	unsigned int gpu_clkdivstep;
	unsigned int gpu_clk_stats;
	unsigned int tpu_clk_stats;
	unsigned int tpu_vdroop_flt;
	unsigned int gpu_vdroop_flt;

	bool batt_psy_initialized;
	bool enabled;

	unsigned int offsrc;
	unsigned int pwronsrc;
};

extern int google_set_mpmm(struct bcl_device *data, unsigned int value);
extern int google_set_ppm(struct bcl_device *data, unsigned int value);
extern unsigned int google_get_mpmm(struct bcl_device *data);
extern unsigned int google_get_ppm(struct bcl_device *data);
extern struct bcl_device *google_retrieve_bcl_handle(void);
extern int google_init_gpu_ratio(struct bcl_device *data);
extern int google_init_tpu_ratio(struct bcl_device *data);
#else
struct bcl_device;

static unsigned int google_get_mpmm(struct bcl_device *data)
{
	return 0;
}
static unsigned int google_get_ppm(struct bcl_device *data)
{
	return 0;
}
static inline int google_set_ppm(struct bcl_device *data, unsigned int value)
{
	return 0;
}
static inline int google_set_mpmm(struct bcl_device *data, unsigned int value)
{
	return 0;
}
static struct google_bcl_dev *google_retrieve_bcl_handle(void)
{
	return NULL;
}
#endif /* IS_ENABLED(CONFIG_GOOGLE_BCL) */

#endif /* __BCL_H */
