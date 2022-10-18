/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __BCL_H
#define __BCL_H

#if IS_ENABLED(CONFIG_GOOGLE_BCL)
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/thermal.h>
#include <linux/workqueue.h>

#define VD_BATTERY_VOLTAGE 4200
#define VD_UPPER_LIMIT 3350
#define VD_LOWER_LIMIT 2600
#define VD_STEP 50
#define VD_DELAY 300
#define BO_UPPER_LIMIT 6800
#define BO_LOWER_LIMIT 3800
#define BO_STEP 200
#define THERMAL_HYST_LEVEL 100

enum TRIGGERED_SOURCE {
	SMPL_WARN,
	OCP_WARN_CPUCL1,
	OCP_WARN_CPUCL2,
	SOFT_OCP_WARN_CPUCL1,
	SOFT_OCP_WARN_CPUCL2,
	OCP_WARN_TPU,
	SOFT_OCP_WARN_TPU,
	OCP_WARN_GPU,
	SOFT_OCP_WARN_GPU,
	PMIC_SOC,
	UVLO1,
	UVLO2,
	BATOILO,
	PMIC_120C,
	PMIC_140C,
	PMIC_OVERHEAT,
	TRIGGERED_SOURCE_MAX,
};

enum BCL_THERMAL_SOURCE {
	TS_UVLO2,
	TS_UVLO1,
};

enum PMIC_REG {
#if IS_ENABLED(CONFIG_SOC_GS101)
	S2MPG10,
	S2MPG11
#endif
#if IS_ENABLED(CONFIG_SOC_GS201)
	S2MPG12,
	S2MPG13
#endif
};

struct ocpsmpl_stats {
	ktime_t _time;
	int capacity;
	int voltage;
};

typedef int (*pmic_set_uvlo_lvl_fn)(struct i2c_client *client, uint8_t mode, unsigned int lvl);
typedef int (*pmic_get_uvlo_lvl_fn)(struct i2c_client *client, uint8_t mode, unsigned int *lvl);
typedef int (*pmic_set_batoilo_lvl_fn)(struct i2c_client *client, unsigned int lvl);
typedef int (*pmic_get_batoilo_lvl_fn)(struct i2c_client *client, unsigned int *lvl);
typedef int (*pmic_get_vdroop_ok_fn)(struct i2c_client *client, bool *state);

struct bcl_ifpmic_ops {
	pmic_get_vdroop_ok_fn	cb_get_vdroop_ok;
	pmic_set_uvlo_lvl_fn	cb_uvlo_write;
	pmic_get_uvlo_lvl_fn	cb_uvlo_read;
	pmic_set_batoilo_lvl_fn	cb_batoilo_write;
	pmic_get_batoilo_lvl_fn cb_batoilo_read;
};

struct bcl_device {
	struct device *device;
	struct device *main_dev;
	struct device *sub_dev;
	struct device *mitigation_dev;
	struct odpm_info *main_odpm;
	struct odpm_info *sub_odpm;
	void __iomem *base_mem[5];
	void __iomem *sysreg_cpucl0;
	struct power_supply *batt_psy;
	const struct bcl_ifpmic_ops *pmic_ops;

	struct notifier_block psy_nb;
	struct delayed_work init_work;
	struct delayed_work bcl_intf_work[TRIGGERED_SOURCE_MAX];
	unsigned int bcl_lvl[TRIGGERED_SOURCE_MAX];
	atomic_t bcl_cnt[TRIGGERED_SOURCE_MAX];
	int bcl_prev_lvl[TRIGGERED_SOURCE_MAX];

	int trip_high_temp;
	int trip_low_temp;
	int trip_val;
	struct mutex state_trans_lock;
	struct thermal_zone_of_device_ops bcl_ops[TRIGGERED_SOURCE_MAX];
	struct mutex bcl_irq_lock[TRIGGERED_SOURCE_MAX];
	struct delayed_work bcl_irq_work[TRIGGERED_SOURCE_MAX];
	struct thermal_zone_device *bcl_tz[TRIGGERED_SOURCE_MAX];

	unsigned int bcl_irq[TRIGGERED_SOURCE_MAX];
	int bcl_tz_cnt[TRIGGERED_SOURCE_MAX];
	int bcl_pin[TRIGGERED_SOURCE_MAX];
	struct ocpsmpl_stats bcl_stats[TRIGGERED_SOURCE_MAX];

	struct i2c_client *main_pmic_i2c;
	struct i2c_client *sub_pmic_i2c;
	struct i2c_client *intf_pmic_i2c;

	struct mutex ratio_lock;
	unsigned int tpu_con_heavy;
	unsigned int tpu_con_light;
	unsigned int gpu_con_heavy;
	unsigned int gpu_con_light;
	unsigned int tpu_clkdivstep;
	unsigned int gpu_clkdivstep;
	unsigned int cpu2_clkdivstep;
	unsigned int cpu1_clkdivstep;
	unsigned int cpu0_clkdivstep;
	unsigned int gpu_clk_stats;
	unsigned int tpu_clk_stats;
	unsigned int tpu_vdroop_flt;
	unsigned int gpu_vdroop_flt;
	unsigned int odpm_ratio;

	bool batt_psy_initialized;
	bool enabled;
	bool ready;

	unsigned int offsrc;
	unsigned int pwronsrc;

	unsigned int vdroop1_pin;
	unsigned int vdroop2_pin;

	/* debug */
	struct dentry *debug_entry;
	unsigned int gpu_clk_out;
	unsigned int tpu_clk_out;
};

extern void google_bcl_irq_update_lvl(struct bcl_device *bcl_dev, int index, unsigned int lvl);
extern void google_bcl_irq_changed(struct bcl_device *bcl_dev, int index);
extern int google_set_mpmm(struct bcl_device *data, unsigned int value);
extern int google_set_ppm(struct bcl_device *data, unsigned int value);
extern unsigned int google_get_mpmm(struct bcl_device *data);
extern unsigned int google_get_ppm(struct bcl_device *data);
extern struct bcl_device *google_retrieve_bcl_handle(void);
extern int google_bcl_register_ifpmic(struct bcl_device *bcl_dev,
				      const struct bcl_ifpmic_ops *pmic_ops);
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
static struct bcl_device *google_retrieve_bcl_handle(void)
{
	return NULL;
}
static void google_bcl_irq_changed(struct bcl_device *bcl_dev, int index)
{
}
static int google_bcl_register_ifpmic(struct bcl_device *bcl_dev,
				      const struct bcl_ifpmic_ops *pmic_ops)
{
	return 0;
}
#endif /* IS_ENABLED(CONFIG_GOOGLE_BCL) */

#endif /* __BCL_H */
