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
	PMIC_120C,
	PMIC_140C,
	PMIC_OVERHEAT,
	PMIC_THERMAL_SENSOR_MAX,
};

enum IRQ_SOURCE_S2MPG10 {
	IRQ_SMPL_WARN,
	IRQ_OCP_WARN_CPUCL1,
	IRQ_OCP_WARN_CPUCL2,
	IRQ_SOFT_OCP_WARN_CPUCL1,
	IRQ_SOFT_OCP_WARN_CPUCL2,
	IRQ_OCP_WARN_TPU,
	IRQ_SOFT_OCP_WARN_TPU,
	IRQ_PMIC_120C,
	IRQ_PMIC_140C,
	IRQ_PMIC_OVERHEAT,
	IRQ_SOURCE_S2MPG10_MAX,
};

enum IRQ_SOURCE_S2MPG11 {
	IRQ_OCP_WARN_GPU,
	IRQ_SOFT_OCP_WARN_GPU,
	IRQ_SOURCE_S2MPG11_MAX,
};

enum PMIC_REG { S2MPG10, S2MPG11 };

struct ocpsmpl_stats {
	ktime_t _time;
	int capacity;
	int voltage;
};

struct gs101_bcl_dev {
	struct device *device;
	struct dentry *debug_entry;
	void __iomem *base_mem[5];
	void __iomem *sysreg_cpucl0;
	struct power_supply *batt_psy;

	struct notifier_block psy_nb;
	struct delayed_work soc_eval_work;
	struct delayed_work mfd_init;

	void *iodev;

	int trip_high_temp;
	int trip_low_temp;
	int trip_val;
	struct mutex state_trans_lock;
	struct mutex ratio_lock;
	struct thermal_zone_device *soc_tzd;
	struct thermal_zone_of_device_ops soc_ops;
	struct mutex s2mpg10_irq_lock[IRQ_SOURCE_S2MPG10_MAX];
	struct mutex s2mpg11_irq_lock[IRQ_SOURCE_S2MPG11_MAX];
	struct delayed_work s2mpg10_irq_work[IRQ_SOURCE_S2MPG10_MAX];
	struct delayed_work s2mpg11_irq_work[IRQ_SOURCE_S2MPG11_MAX];
	struct thermal_zone_device *s2mpg10_tz_irq[IRQ_SOURCE_S2MPG10_MAX];
	struct thermal_zone_device *s2mpg11_tz_irq[IRQ_SOURCE_S2MPG11_MAX];

	unsigned int s2mpg10_lvl[IRQ_SOURCE_S2MPG10_MAX];
	unsigned int s2mpg11_lvl[IRQ_SOURCE_S2MPG11_MAX];
	unsigned int s2mpg10_irq[IRQ_SOURCE_S2MPG10_MAX];
	unsigned int s2mpg11_irq[IRQ_SOURCE_S2MPG11_MAX];
	int s2mpg10_counter[IRQ_SOURCE_S2MPG10_MAX];
	int s2mpg11_counter[IRQ_SOURCE_S2MPG11_MAX];
	int s2mpg10_pin[IRQ_SOURCE_S2MPG10_MAX];
	int s2mpg11_pin[IRQ_SOURCE_S2MPG11_MAX];
	atomic_t s2mpg10_cnt[IRQ_SOURCE_S2MPG10_MAX];
	struct ocpsmpl_stats s2mpg10_stats[IRQ_SOURCE_S2MPG10_MAX];
	atomic_t s2mpg11_cnt[IRQ_SOURCE_S2MPG11_MAX];
	struct ocpsmpl_stats s2mpg11_stats[IRQ_SOURCE_S2MPG11_MAX];

	struct s2mpg10_dev *s2mpg10;
	struct s2mpg11_dev *s2mpg11;

	struct i2c_client *s2mpg10_i2c;
	struct i2c_client *s2mpg11_i2c;

	unsigned int s2mpg10_triggered_irq[IRQ_SOURCE_S2MPG10_MAX];
	unsigned int s2mpg11_triggered_irq[IRQ_SOURCE_S2MPG11_MAX];

	unsigned int tpu_con_heavy;
	unsigned int tpu_con_light;
	unsigned int gpu_con_heavy;
	unsigned int gpu_con_light;
	unsigned int tpu_clkdivstep;
	unsigned int gpu_clkdivstep;
};

extern int gs101_set_mpmm(struct gs101_bcl_dev *data, unsigned int value);
extern int gs101_set_ppm(struct gs101_bcl_dev *data, unsigned int value);
extern unsigned int gs101_get_mpmm(struct gs101_bcl_dev *data);
extern unsigned int gs101_get_ppm(struct gs101_bcl_dev *data);
extern struct gs101_bcl_dev *gs101_retrieve_bcl_handle(void);
extern int gs101_init_gpu_ratio(struct gs101_bcl_dev *data);
extern int gs101_init_tpu_ratio(struct gs101_bcl_dev *data);
#else
struct gs101_bcl_dev;

static unsigned int gs101_get_mpmm(struct gs101_bcl_dev *data)
{
	return 0;
}
static unsigned int gs101_get_ppm(struct gs101_bcl_dev *data)
{
	return 0;
}
static inline int gs101_set_ppm(struct gs101_bcl_dev *data, unsigned int value)
{
	return 0;
}
static inline int gs101_set_mpmm(struct gs101_bcl_dev *data, unsigned int value)
{
	return 0;
}
static struct gs101_bcl_dev *gs101_retrieve_bcl_handle(void)
{
	return NULL;
}
static int gs101_init_gpu_ratio(struct gs101_bcl_dev *data)
{
	return 0;
}
static int gs101_init_tpu_ratio(struct gs101_bcl_dev *data)
{
	return 0;
}
#endif /* IS_ENABLED(CONFIG_GOOGLE_BCL) */

#endif /* __BCL_H */
