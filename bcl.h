/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __BCL_H
#define __BCL_H

#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/pm_qos.h>
#include <linux/thermal.h>
#include <linux/hrtimer.h>
#include <linux/workqueue.h>
#include <soc/google/exynos_pm_qos.h>
#include <dt-bindings/power/s2mpg1x-power.h>
#if IS_ENABLED(CONFIG_SOC_ZUMA)
#include <dt-bindings/soc/google/zumapro-bcl.h>
#include <linux/mfd/samsung/rtc-s2mpg14.h>
#elif IS_ENABLED(CONFIG_SOC_GS101)
#include <linux/mfd/samsung/rtc-s2mpg10.h>
#include <dt-bindings/soc/google/gs101-bcl.h>
#elif IS_ENABLED(CONFIG_SOC_GS201)
#include <linux/mfd/samsung/rtc-s2mpg12.h>
#include <dt-bindings/soc/google/gs201-bcl.h>
#endif
#include <trace/events/power.h>
#include "uapi/brownout_stats.h"

#if IS_ENABLED(CONFIG_SOC_GS101)
#define MAIN_METER_PWR_WARN0	S2MPG10_METER_PWR_WARN0
#define SUB_METER_PWR_WARN0	S2MPG11_METER_PWR_WARN0
#elif IS_ENABLED(CONFIG_SOC_GS201)
#define MAIN_METER_PWR_WARN0	S2MPG12_METER_PWR_WARN0
#define SUB_METER_PWR_WARN0	S2MPG13_METER_PWR_WARN0
#elif IS_ENABLED(CONFIG_SOC_ZUMA)
#define MAIN_METER_PWR_WARN0	S2MPG14_METER_PWR_WARN0
#define SUB_METER_PWR_WARN0	S2MPG15_METER_PWR_WARN0
#endif

#define bcl_cb_get_irq(bcl, v) (((bcl)->ifpmic == MAX77759) ? \
        max77759_get_irq(bcl, v) : max77779_get_irq(bcl, v))
#define bcl_cb_clr_irq(bcl, v) (((bcl)->ifpmic == MAX77759) ? \
        max77759_clr_irq(bcl, v) : max77779_clr_irq(bcl, v))
#define bcl_vimon_read(bcl) (((bcl)->ifpmic == MAX77759) ? \
	max77759_vimon_read(bcl) : max77779_vimon_read(bcl))

#define DELTA_5MS			(5 * NSEC_PER_MSEC)
#define DELTA_10MS			(10 * NSEC_PER_MSEC)
#define VSHUNT_MULTIPLIER		10000
#define MILLI_TO_MICRO			1000
#define IRQ_ENABLE_DELAY_MS		50
#define NOT_USED			9999
#define TIMEOUT_1S			1000
#define TIMEOUT_5S			5000
#define TIMEOUT_5000US			5000
#define TIMEOUT_10000US			10000
#define TIMEOUT_10MS			10
#define TIMEOUT_5MS			5
#define TIMEOUT_1MS			1
#define DATA_LOGGING_TIME_MS		48
#define DATA_LOGGING_NUM		50
#define HEAVY_MITIGATION_MODULES_NUM	3
#define MITIGATION_INPUT_DELIM		","
#define MITIGATION_PRINT_BUF_SIZE  	256
#define MITIGATION_TMP_BUF_SIZE	16
#define MAXMIN_RESET_VAL		0x807F
#define BAT_DTLS_OILO_ASSERTED		0x6
#define PWRWARN_LPF_RFFE_MMWAVE_DATA_0         0xCF
#define PWRWARN_LPF_RFFE_MMWAVE_DATA_1         0xD0
#define PWRWARN_THRESH_RFFE_MMWAVE             0x3C
#define PWRWARN_LPF_RFFE_MMWAVE_MSB_MASK       0x0F
#define PWRWARN_LPF_RFFE_MMWAVE_RSHIFT         4
#define DEFAULT_SYS_UVLO1_LVL 0xC /* 3.2V */
#define DEFAULT_SYS_UVLO2_LVL 0x2 /* 2.7V */
#define DEFAULT_VDROOP_INT_MASK 0xDF /* Only BATOILO is passed */
#define DEFAULT_INTB_MASK 0x0 /* All IRQs are passed */
#define DEFAULT_SMPL 0xCB /* 3.2V, 200mV HYS, 38us debounce */
#define DEFAULT_VIMON_PWR_LOOP_CNT 0
#define DEFAULT_VIMON_PWR_LOOP_THRESH 20000
#define MAX77779_VIMON_NV_PRE_LSB 78122
#define MAX77779_VIMON_NA_PRE_LSB 781250
#define BAT_KTIMER_LIMIT_MS 34
#define LAST_CURR_RD_CNT_MAX 10

#if IS_ENABLED(CONFIG_SOC_GS101)
#define MAIN_OFFSRC1 S2MPG10_PM_OFFSRC
#define MAIN_OFFSRC2 S2MPG10_PM_OFFSRC
#define SUB_OFFSRC1 S2MPG11_PM_OFFSRC
#define SUB_OFFSRC2 S2MPG11_PM_OFFSRC
#define MAIN_PWRONSRC S2MPG10_PM_PWRONSRC
#elif IS_ENABLED(CONFIG_SOC_GS201)
#define MAIN_OFFSRC1 S2MPG12_PM_OFFSRC1
#define MAIN_OFFSRC2 S2MPG12_PM_OFFSRC2
#define SUB_OFFSRC1 S2MPG13_PM_OFFSRC
#define SUB_OFFSRC2 S2MPG13_PM_OFFSRC
#define RTC_SCRATCH1 S2MPG12_RTC_SCRATCH1
#define MAIN_PWRONSRC S2MPG12_PM_PWRONSRC
#elif IS_ENABLED(CONFIG_SOC_ZUMA)
#define MAIN_OFFSRC1 S2MPG14_PM_OFFSRC1
#define MAIN_OFFSRC2 S2MPG14_PM_OFFSRC2
#define SUB_OFFSRC1 S2MPG15_PM_OFFSRC1
#define SUB_OFFSRC2 S2MPG15_PM_OFFSRC2
#define RTC_SCRATCH1 S2MPG14_RTC_SCRATCH1
#define MAIN_PWRONSRC S2MPG14_PM_PWRONSRC
#endif

/* SMPL zone name changed after gs201 */
#if IS_ENABLED(CONFIG_SOC_GS101) || IS_ENABLED(CONFIG_SOC_GS201)
#define SMPL_ZONE_NAME "smpl_gm"
#else
#define SMPL_ZONE_NAME "smpl_warn"
#endif

#define PSY_NAME "google,power-supply"
#define PSY_OTG_NAME "google,plc"

#if !IS_ENABLED(CONFIG_SOC_ZUMAPRO)
#define BATOILO_DET_SHIFT 2
#endif

/* This driver determines if HW was throttled due to SMPL/OCP */

enum CPU_CLUSTER {
	LITTLE_CLUSTER,
	MID_CLUSTER,
	BIG_CLUSTER,
	CPU_CLUSTER_MAX,
};

enum SUBSYSTEM_SOURCE {
	SUBSYSTEM_CPU0,
	SUBSYSTEM_CPU1,
	SUBSYSTEM_CPU2,
	SUBSYSTEM_TPU,
	SUBSYSTEM_GPU,
	SUBSYSTEM_AUR,
	SUBSYSTEM_SOURCE_MAX,
};

enum CONCURRENT_PWRWARN_IRQ {
	NONE_BCL_BIN,
	MMWAVE_BCL_BIN,
	RFFE_BCL_BIN,
	MAX_CONCURRENT_PWRWARN_IRQ,
};

enum BCL_BATT_IRQ {
	UVLO1_IRQ_BIN,
	UVLO2_IRQ_BIN,
	BATOILO_IRQ_BIN,
#if IS_ENABLED(CONFIG_SOC_ZUMAPRO)
	BATOILO2_IRQ_BIN,
#endif
	MAX_BCL_BATT_IRQ,
};

enum MITIGATION_MODE {
	START,
	LIGHT,
	MEDIUM,
	HEAVY,
	DISABLED,
	MAX_MITIGATION_MODE,
};

enum IRQ_DURATION_BIN {
	LT_5MS,
	BT_5MS_10MS,
	GT_10MS,
};

enum IRQ_TYPE {
	CORE_MAIN_PMIC,
	CORE_SUB_PMIC,
	IF_PMIC,
};

enum IFPMIC {
	MAX77759,
	MAX77779
};

struct irq_duration_stats {
	atomic_t lt_5ms_count;
	atomic_t bt_5ms_10ms_count;
	atomic_t gt_10ms_count;
	ktime_t start_time;
};

extern const unsigned int subsystem_pmu[];
extern const unsigned int clk_stats_offset[];

struct ocpsmpl_stats {
	ktime_t _time;
	int capacity;
	int voltage;
};

enum RATIO_SOURCE {
	CPU0_CON,
	CPU1_HEAVY,
	CPU2_HEAVY,
	TPU_HEAVY,
	GPU_HEAVY,
	CPU1_LIGHT,
	CPU2_LIGHT,
	TPU_LIGHT,
	GPU_LIGHT
};

enum MPMM_SOURCE {
	LITTLE,
	MID,
	BIG,
	MPMMEN
};

struct qos_throttle_limit {
	struct freq_qos_request cpu0_max_qos_req;
	struct freq_qos_request cpu1_max_qos_req;
	struct freq_qos_request cpu2_max_qos_req;
	struct exynos_pm_qos_request gpu_qos_max;
	struct exynos_pm_qos_request tpu_qos_max;
	int cpu0_limit;
	int cpu1_limit;
	int cpu2_limit;
	int gpu_limit;
	int tpu_limit;
	int cpu0_freq;
	int cpu1_freq;
	int cpu2_freq;
	int gpu_freq;
	int tpu_freq;
};

struct zone_triggered_stats {
	atomic_t triggered_cnt[MAX_MITIGATION_MODE];
	ktime_t triggered_time[MAX_MITIGATION_MODE];
};

struct bcl_zone {
	struct device *device;
	struct completion deassert;
	struct work_struct irq_triggered_work;
	struct delayed_work warn_work;
	struct delayed_work enable_irq_work;
	struct thermal_zone_device *tz;
	struct thermal_zone_device_ops tz_ops;
	struct qos_throttle_limit *bcl_qos;
	struct ocpsmpl_stats bcl_stats;
	struct zone_triggered_stats last_triggered;
	atomic_t bcl_cnt;
	int bcl_prev_lvl;
	int bcl_cur_lvl;
	int bcl_lvl;
	u16 bcl_pin;
	int bcl_irq;
	int irq_type;
	int polarity;
	void *parent;
	int idx;
	bool disabled;
	bool irq_reg;
	bool conf_qos;
	const char *devname;
	u32 current_state;
	bool throttle;
};

struct bcl_core_conf {
	unsigned int con_heavy;
	unsigned int con_light;
	unsigned int clkdivstep;
	unsigned int vdroop_flt;
	unsigned int clk_stats;
	unsigned int clk_out;
	void __iomem *base_mem;
};

enum CPU_BUFF_IDX {
	CPU_BUFF_IDX_MID,
	CPU_BUFF_IDX_BIG
};

enum CPU_BUFF_VALS {
	CPU_BUFF_CON_HEAVY,
	CPU_BUFF_CON_LIGHT,
	CPU_BUFF_CLKDIVSTEP,
	CPU_BUFF_VDROOP_FLT,
	CPU_BUFF_CLK_STATS,
	CPU_BUFF_VALS_MAX
};

struct bcl_cpu_buff_conf {
	unsigned int buff[CPU_BUFF_VALS_MAX];
	unsigned int addr[CPU_BUFF_VALS_MAX];
	uint8_t wr_update_rqd;
	uint8_t rd_update_rqd;
};

struct bcl_batt_irq_conf {
	int batoilo_lower_limit;
	int batoilo_upper_limit;
	u8 batoilo_trig_lvl;
	u8 batoilo_wlc_trig_lvl;
	u8 batoilo_usb_trig_lvl;
	u8 batoilo_otg_trig_lvl;
	u8 batoilo_bat_open_to;
	u8 batoilo_bat_otg_open_to;
	u8 batoilo_rel;
	u8 batoilo_det;
	u8 batoilo_int_rel;
	u8 batoilo_int_det;
	u8 uvlo_rel;
	u8 uvlo_det;
};

#if IS_ENABLED(CONFIG_SOC_ZUMAPRO)
struct bcl_evt_count {
	unsigned int uvlo1;
	unsigned int uvlo2;
	unsigned int batoilo1;
	unsigned int batoilo2;
	u8 enable;
	u8 rate;
};
#endif

struct bcl_mitigation_conf {
	u32 module_id;
	u32 threshold;
};

struct bcl_vimon_intf {
	uint16_t data[VIMON_BUF_SIZE];
	size_t count;
};

struct bcl_device {
	struct device *device;
	struct device *main_dev;
	struct device *sub_dev;
	struct device *mitigation_dev;
	struct odpm_info *main_odpm;
	struct odpm_info *sub_odpm;
	void __iomem *sysreg_cpucl0;
	struct power_supply *batt_psy;
	struct power_supply *otg_psy;

	struct hrtimer hr_timer;

	struct notifier_block psy_nb;
	struct bcl_zone *zone[TRIGGERED_SOURCE_MAX];
	struct delayed_work soc_work;
	struct workqueue_struct *qos_update_wq;
	struct thermal_zone_device *soc_tz;
	struct thermal_zone_device_ops soc_tz_ops;
	bool throttle;

	int trip_high_temp;
	int trip_low_temp;
	int trip_val;
	struct mutex sysreg_lock;

	struct i2c_client *main_pmic_i2c;
	struct i2c_client *main_rtc_i2c;
	struct i2c_client *sub_pmic_i2c;
	struct i2c_client *main_meter_i2c;
	struct i2c_client *sub_meter_i2c;
	struct device *intf_pmic_dev;
	struct device *irq_pmic_dev;
	struct device *fg_pmic_dev;
	struct device *vimon_dev;
	struct mutex qos_update_lock;
	struct mutex cpu_ratio_lock;
	struct bcl_core_conf core_conf[SUBSYSTEM_SOURCE_MAX];
	struct bcl_cpu_buff_conf cpu_buff_conf[CPU_CLUSTER_MAX];
	struct notifier_block cpu_nb;
	struct delayed_work rd_last_curr_work;

	bool batt_psy_initialized;
	bool enabled;

	unsigned int main_offsrc1;
	unsigned int main_offsrc2;
	unsigned int sub_offsrc1;
	unsigned int sub_offsrc2;
	unsigned int pwronsrc;
	unsigned int irq_delay;
	unsigned int last_current;
	unsigned int last_curr_rd_retry_cnt;

	unsigned int vdroop1_pin;
	unsigned int vdroop2_pin;
	unsigned int modem_gpio1_pin;
	unsigned int modem_gpio2_pin;
	unsigned int rffe_channel;

	/* debug */
	struct dentry *debug_entry;
	unsigned int gpu_clk_out;
	unsigned int tpu_clk_out;
	unsigned int aur_clk_out;
	u8 add_perph;
	u64 add_addr;
	u64 add_data;
	void __iomem *base_add_mem[SUBSYSTEM_SOURCE_MAX];

	int main_irq_base, sub_irq_base;
	u8 main_setting[METER_CHANNEL_MAX];
	u8 sub_setting[METER_CHANNEL_MAX];
	u64 main_limit[METER_CHANNEL_MAX];
	u64 sub_limit[METER_CHANNEL_MAX];
	int main_pwr_warn_irq[METER_CHANNEL_MAX];
	int sub_pwr_warn_irq[METER_CHANNEL_MAX];
	bool main_pwr_warn_triggered[METER_CHANNEL_MAX];
	bool sub_pwr_warn_triggered[METER_CHANNEL_MAX];
	struct delayed_work main_pwr_irq_work;
	struct delayed_work sub_pwr_irq_work;
	struct delayed_work setup_main_odpm_work;
	struct delayed_work setup_sub_odpm_work;
	struct irq_duration_stats ifpmic_irq_bins[MAX_BCL_BATT_IRQ][MAX_CONCURRENT_PWRWARN_IRQ];
	struct irq_duration_stats pwrwarn_main_irq_bins[METER_CHANNEL_MAX];
	struct irq_duration_stats pwrwarn_sub_irq_bins[METER_CHANNEL_MAX];
	const char *main_rail_names[METER_CHANNEL_MAX];
	const char *sub_rail_names[METER_CHANNEL_MAX];

	int cpu0_cluster;
	int cpu1_cluster;
	int cpu2_cluster;

	bool cpu0_cluster_on;
	bool cpu1_cluster_on;
	bool cpu2_cluster_on;

	struct bcl_batt_irq_conf batt_irq_conf1;
	struct bcl_batt_irq_conf batt_irq_conf2;
	int pmic_irq;

	enum IFPMIC ifpmic;

	bool enabled_br_stats;
	bool data_logging_initialized;
	unsigned int triggered_idx;
	ssize_t br_stats_size;
	struct brownout_stats *br_stats;
	/* module id */
	struct bcl_mitigation_conf main_mitigation_conf[METER_CHANNEL_MAX];
	struct bcl_mitigation_conf sub_mitigation_conf[METER_CHANNEL_MAX];
	u32 *non_monitored_module_ids;
	u32 non_monitored_mitigation_module_ids;
	atomic_t mitigation_module_ids;
#if IS_ENABLED(CONFIG_REGULATOR_S2MPG14) || IS_ENABLED(CONFIG_REGULATOR_S2MPG12)
	struct gvotable_election *toggle_wlc;
	struct gvotable_election *toggle_usb;
	struct gvotable_election *toggle_otg;
#endif

#if IS_ENABLED(CONFIG_SOC_ZUMAPRO)
	struct bcl_evt_count evt_cnt;
	struct bcl_evt_count evt_cnt_latest;
#endif

	struct bcl_vimon_intf vimon_intf;
	bool config_modem;
	bool rffe_mitigation_enable;

	u8 vdroop_int_mask;
	u8 intb_int_mask;
	u8 uvlo2_lvl;
	u8 uvlo1_lvl;
	u8 smpl_ctrl;
	bool uvlo2_vdrp2_en;
	bool uvlo2_vdrp1_en;
	bool uvlo1_vdrp1_en;
	bool uvlo1_vdrp2_en;
	bool oilo1_vdrp1_en;
	bool oilo1_vdrp2_en;
	bool oilo2_vdrp1_en;
	bool oilo2_vdrp2_en;
	bool vimon_pwr_loop_en;
	bool vimon_pwr_loop_cnt;
	int vimon_pwr_loop_thresh;

	struct delayed_work qos_work;

	bool usb_otg_conf;

	bool bat_ktimer_en;
	unsigned int bat_ktimer;
	struct wakeup_source *ws;
};

extern void google_bcl_irq_update_lvl(struct bcl_device *bcl_dev, int index, unsigned int lvl);
extern int google_set_db(struct bcl_device *data, unsigned int value, enum MPMM_SOURCE index);
extern unsigned int google_get_db(struct bcl_device *data, enum MPMM_SOURCE index);
extern struct bcl_device *google_retrieve_bcl_handle(void);
extern int google_init_gpu_ratio(struct bcl_device *data);
extern int google_init_tpu_ratio(struct bcl_device *data);
extern int google_init_aur_ratio(struct bcl_device *data);
bool bcl_is_subsystem_on(struct bcl_device *bcl_dev, unsigned int addr);
int cpu_sfr_read(struct bcl_device *bcl_dev, int idx, void __iomem *addr, unsigned int *reg);
int cpu_sfr_write(struct bcl_device *bcl_dev, int idx, void __iomem *addr, unsigned int value);
int cpu_buff_write(struct bcl_device *bcl_dev, int cluster, unsigned int type, unsigned int val);
int cpu_buff_read(struct bcl_device *bcl_dev, int cluster, unsigned int type, unsigned int *reg);
bool bcl_disable_power(struct bcl_device *bcl_dev, int cluster);
bool bcl_enable_power(struct bcl_device *bcl_dev, int cluster);
bool bcl_is_cluster_on(struct bcl_device *bcl_dev, int cluster);
int pmic_write(int pmic, struct bcl_device *bcl_dev, u8 reg, u8 value);
int pmic_read(int pmic, struct bcl_device *bcl_dev, u8 reg, u8 *value);
int meter_write(int pmic, struct bcl_device *bcl_dev, u8 reg, u8 value);
int meter_read(int pmic, struct bcl_device *bcl_dev, u8 reg, u8 *value);
u64 settings_to_current(struct bcl_device *bcl_dev, int pmic, int idx, u32 setting);
void google_bcl_qos_update(struct bcl_zone *zone, bool throttle);
int google_bcl_setup_qos(struct bcl_device *bcl_dev);
void google_bcl_remove_qos(struct bcl_device *bcl_dev);
void google_init_debugfs(struct bcl_device *bcl_dev);
int uvlo_reg_read(struct device *dev, enum IFPMIC ifpmic, int triggered, unsigned int *val);
int batoilo_reg_read(struct device *dev, enum IFPMIC ifpmic, int oilo, unsigned int *val);
int max77759_get_irq(struct bcl_device *bcl_dev, u8 *irq_val);
int max77759_clr_irq(struct bcl_device *bcl_dev, int idx);
int max77779_get_irq(struct bcl_device *bcl_dev, u8 *irq_val);
int max77779_clr_irq(struct bcl_device *bcl_dev, int idx);
int google_bcl_init_notifier(struct bcl_device *bcl_dev);
int google_bcl_init_data_logging(struct bcl_device *bcl_dev);
void google_bcl_start_data_logging(struct bcl_device *bcl_dev, int idx);
void google_bcl_remove_data_logging(struct bcl_device *bcl_dev);
void google_bcl_upstream_state(struct bcl_zone *zone, enum MITIGATION_MODE state);
int google_pwr_loop_trigger_mitigation(struct bcl_device *bcl_dev);
int max77759_vimon_read(struct bcl_device *bcl_dev);
int max77779_vimon_read(struct bcl_device *bcl_dev);
int max77759_req_vimon_conv(struct bcl_device *bcl_dev, int idx);
int max77779_vimon_register_callback(struct bcl_device *bcl_dev);

#if IS_ENABLED(CONFIG_SOC_ZUMAPRO)
int max77779_adjust_bat_open_to(struct bcl_device *bcl_dev, bool enable);
int max77779_adjust_batoilo_lvl(struct bcl_device *bcl_dev, u8 lower_enable, u8 set_batoilo1_lvl,
                                u8 set_batoilo2_lvl);

#else
int max77759_adjust_batoilo_lvl(struct bcl_device *bcl_dev, u8 lower_enable, u8 set_batoilo1_lvl);
#endif

#if IS_ENABLED(CONFIG_REGULATOR_S2MPG14) || IS_ENABLED(CONFIG_REGULATOR_S2MPG12)
int google_bcl_setup_votable(struct bcl_device *bcl_dev);
void google_bcl_remove_votable(struct bcl_device *bcl_dev);

#endif

#endif /* __BCL_H */
