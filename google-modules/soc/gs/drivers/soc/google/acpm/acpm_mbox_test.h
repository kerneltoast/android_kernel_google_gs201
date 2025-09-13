/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright 2020 Google LLC
 *
 */
#ifndef __ACPM_MBOX_TEST_H__
#define __ACPM_MBOX_TEST_H__
#include <soc/google/pt.h>
#if defined(CONFIG_SOC_GS101)
#include <dt-bindings/clock/gs101.h>
#include <dt-bindings/soc/google/gs101-devfreq.h>
#elif defined(CONFIG_SOC_GS201)
#include <dt-bindings/clock/gs201.h>
#include <dt-bindings/soc/google/gs201-devfreq.h>
#elif defined(CONFIG_SOC_ZUMA)
#include <dt-bindings/clock/zuma.h>
#include <dt-bindings/soc/google/zuma-devfreq.h>
#endif

enum tz_id {
	TZ_BIG = 0,
	TZ_MID,
	TZ_LIT,
	TZ_GPU,
	TZ_ISP,
	TZ_TPU,
	TZ_END,
};

enum random_output {
	CPU_ID = 0,
	DELAY_MS,
	TERMAL_ZONE_ID,
	DVFS_DOMAIN_ID,
	GRANVILLE_M_REG,
	GRANVILLE_S_REG,
	SLC_REQ_TYPE,
	END,
};

enum acpm_mbox_test_commands {
	ACPM_MBOX_TEST_STOP,
	ACPM_MBOX_TEST_START,
	ACPM_MBOX_CTRLIST,
	ACPM_MBOX_TEST_CMD_MAX,
};

enum acpm_dvfs_test_commands {
	ACPM_DVFS_TEST_MIF,
	ACPM_DVFS_TEST_INT,
	ACPM_DVFS_TEST_CPUCL0,
	ACPM_DVFS_TEST_CPUCL1,
	ACPM_DVFS_TEST_CPUCL2,
#if defined(CONFIG_SOC_ZUMA)
	ACPM_DVFS_TEST_DSU,
	ACPM_DVFS_TEST_BCI,
#endif
	ACPM_DVFS_TEST_G3D,
	ACPM_DVFS_TEST_G3DL2,
	ACPM_DVFS_TEST_TPU,
	ACPM_DVFS_TEST_INTCAM,
	ACPM_DVFS_TEST_TNR,
	ACPM_DVFS_TEST_CAM,
	ACPM_DVFS_TEST_RESULT,
	ACPM_DVFS_CMD_MAX,
};

enum domains {
	DVFS_MIF = 0,
	DVFS_INT,
	DVFS_CPUCL0,
	DVFS_CPUCL1,
	DVFS_CPUCL2,
#if defined(CONFIG_SOC_ZUMA)
	DVFS_DSU,
	DVFS_BCI,
#endif
	DVFS_G3D,		/*not support */
	DVFS_G3DL2,		/*not support */
	DVFS_TPU,		/*not support */
	DVFS_INTCAM,
	DVFS_TNR,
	DVFS_CAM,
	NUM_OF_DVFS_DOMAINS,
};

enum cpu_policy_id {
	CPUCL0_POLICY = 0,
	CPUCL1_POLICY = 4,
	CPUCL2_POLICY = 6,
};

enum cpu_cluster_id {
	CPU_CL0 = 0,
	CPU_CL1 = 1,
	CPU_CL2 = 2,
};

enum slc_req_type {
	VERSION = 0,
	STATE,
	PT_INFO,
	NUM_OF_SLC_REQ_TYPE,
};

enum pt_info_t {
	PT_SIZE = 0,
	PBHA = 1,
	VPTID = 2,
	PRIMARY_WAYS = 3,
	SECONDARY_WAYS = 4,
};

enum pmic_id {
	BUCK1M_MIF = 0,
	BUCK2M_CPUCL2,
	BUCK3M_CPUCL1,
	BUCK4M_CPUCL0,
	BUCK5M_INT,
	BUCK6M_CPUCL2_M,
	BUCK7M_INT_M,
	BUCK10M_TPU,
	LDO12M_CPUCL0_M,
	LDO13M_TPU_M,
	LDO15M_SLC_M,
	NUM_OF_PMIC_MASTER,
	BUCK1S_CAM = NUM_OF_PMIC_MASTER,
	BUCK2S_G3D,
	BUCK3S_LLDO1,
	BUCK4S_VDD2H,
	BUCK5S_VDDQ,
	BUCK8S_G3DL2,
	BUCK9S_AOC,
	LDO2S_AOC_RET,
	NUM_OF_PMIC_ID,
};

#if defined(CONFIG_SOC_GS101)
/* RTC(0x2) Registers */
enum GS101_S2MPG10_RTC_REG {
	RTC_REG_CTRL = 0x0,
	RTC_REG_UPDATE = 0x1,
	RTC_REG_SMPL = 0x2,
	RTC_REG_WTSR = 0x3,
	RTC_REG_CAPSEL = 0x4,
	RTC_REG_MSEC = 0x5,
	RTC_REG_SEC = 0x6,
	RTC_REG_MIN = 0x7,
	RTC_REG_HOUR = 0x8,
	RTC_REG_WEEK = 0x9,
	RTC_REG_DAY = 0xA,
	RTC_REG_MON = 0xB,
	RTC_REG_YEAR = 0xC,
	RTC_REG_A0SEC = 0xD,
	RTC_REG_A0MIN = 0xE,
	RTC_REG_A0HOUR = 0xF,
	RTC_REG_A0WEEK = 0x10,
	RTC_REG_A0DAY = 0x11,
	RTC_REG_A0MON = 0x12,
	RTC_REG_A0YEAR = 0x13,
	RTC_REG_A1SEC = 0x14,
	RTC_REG_A1MIN = 0x15,
	RTC_REG_A1HOUR = 0x16,
	RTC_REG_A1WEEK = 0x17,
	RTC_REG_A1DAY = 0x18,
	RTC_REG_A1MON = 0x19,
	RTC_REG_A1YEAR = 0x1A,
	RTC_REG_OSCCTRL = 0x1B,
};
#elif defined(CONFIG_SOC_GS201)
/* RTC(0x2) Registers */
enum GS201_S2MPG12_RTC_REG {
	RTC_REG_CTRL = 0x0,
	RTC_REG_UPDATE = 0x1,
	RTC_REG_SMPL = 0x2,
	RTC_REG_WTSR = 0x3,
	RTC_REG_CAPSEL = 0x4,
	RTC_REG_MSEC = 0x5,
	RTC_REG_SEC = 0x6,
	RTC_REG_MIN = 0x7,
	RTC_REG_HOUR = 0x8,
	RTC_REG_WEEK = 0x9,
	RTC_REG_DAY = 0xA,
	RTC_REG_MON = 0xB,
	RTC_REG_YEAR = 0xC,
	RTC_REG_A0SEC = 0xD,
	RTC_REG_A0MIN = 0xE,
	RTC_REG_A0HOUR = 0xF,
	RTC_REG_A0WEEK = 0x10,
	RTC_REG_A0DAY = 0x11,
	RTC_REG_A0MON = 0x12,
	RTC_REG_A0YEAR = 0x13,
	RTC_REG_A1SEC = 0x14,
	RTC_REG_A1MIN = 0x15,
	RTC_REG_A1HOUR = 0x16,
	RTC_REG_A1WEEK = 0x17,
	RTC_REG_A1DAY = 0x18,
	RTC_REG_A1MON = 0x19,
	RTC_REG_A1YEAR = 0x1A,
	RTC_REG_OSCCTRL = 0x1B,
};
#elif defined(CONFIG_SOC_ZUMA)
enum ZUMA_S2MPG14_RTC_REG {
	RTC_REG_CTRL = 0x0,
	RTC_REG_UPDATE = 0x1,
	RTC_REG_SMPL = 0x2,
	RTC_REG_WTSR = 0x3,
	RTC_REG_CAPSEL = 0x4,
	RTC_REG_MSEC = 0x5,
	RTC_REG_SEC = 0x6,
	RTC_REG_MIN = 0x7,
	RTC_REG_HOUR = 0x8,
	RTC_REG_WEEK = 0x9,
	RTC_REG_DAY = 0xA,
	RTC_REG_MON = 0xB,
	RTC_REG_YEAR = 0xC,
	RTC_REG_A0SEC = 0xD,
	RTC_REG_A0MIN = 0xE,
	RTC_REG_A0HOUR = 0xF,
	RTC_REG_A0WEEK = 0x10,
	RTC_REG_A0DAY = 0x11,
	RTC_REG_A0MON = 0x12,
	RTC_REG_A0YEAR = 0x13,
	RTC_REG_A1SEC = 0x14,
	RTC_REG_A1MIN = 0x15,
	RTC_REG_A1HOUR = 0x16,
	RTC_REG_A1WEEK = 0x17,
	RTC_REG_A1DAY = 0x18,
	RTC_REG_A1MON = 0x19,
	RTC_REG_A1YEAR = 0x1A,
	RTC_REG_OSC_CTRL = 0x1B,
};
#endif

#define NUM_OF_WQ               16

/* IPC Mailbox Channel */
#define IPC_AP_TMU              9
#define IPC_AP_SLC              10

/* IPC Request Types */
#define TMU_IPC_READ_TEMP       0x02
#define TMU_IPC_AP_SUSPEND      0x04
#define TMU_IPC_AP_RESUME       0x10
#define TMU_IPC_TMU_CONTROL     0x13

#define DVFS_TEST_CYCLE         20

#define STRESS_TRIGGER_DELAY    300

struct acpm_tmu_validity {
	struct delayed_work rd_tmp_concur_wk[NUM_OF_WQ];
	struct delayed_work rd_tmp_random_wk[NUM_OF_WQ];
	struct delayed_work rd_tmp_stress_trigger_wk;
	struct delayed_work suspend_work;
	struct delayed_work resume_work;
	struct workqueue_struct *rd_tmp_concur_wq[NUM_OF_WQ];
	struct workqueue_struct *rd_tmp_random_wq[NUM_OF_WQ];
	struct workqueue_struct *rd_tmp_stress_trigger_wq;
	struct workqueue_struct *suspend_wq;
	struct workqueue_struct *resume_wq;
};

struct acpm_dvfs_validity {
	struct delayed_work rate_change_wk[NUM_OF_WQ];
	struct delayed_work mbox_stress_trigger_wk;
	struct workqueue_struct *rate_change_wq[NUM_OF_WQ];
	struct workqueue_struct *mbox_stress_trigger_wq;
	unsigned int upper_bound;
	unsigned int lower_bound;
	bool is_given_range;
	bool is_steep_jump;
};

#define PMIC_RANDOM_ADDR_RANGE  0x1FF

/* Set 365 days as a year */
#define SECS_PER_YEAR   977616000
/* Set 31 days as a month */
#define SECS_PER_MONTH  2678400
#define SECS_PER_DAY    86400
#define SECS_PER_HR     3600
#define SECS_PER_MIN    60

struct acpm_mfd_validity {
#if defined(CONFIG_SOC_GS101)
	struct s2mpg10_dev *s2mpg_main;
	struct s2mpg11_dev *s2mpg_sub;
#elif defined(CONFIG_SOC_GS201)
	struct s2mpg12_dev *s2mpg_main;
	struct s2mpg13_dev *s2mpg_sub;
#elif defined(CONFIG_SOC_ZUMA)
	struct s2mpg14_dev *s2mpg_main;
	struct s2mpg15_dev *s2mpg_sub;
#endif
	struct i2c_client *main_pmic;
	struct i2c_client *sub_pmic;
	struct i2c_client *rtc;
	struct delayed_work main_pm_mfd_read_wk[NUM_OF_WQ];
	struct delayed_work sub_pm_mfd_read_wk[NUM_OF_WQ];
	struct delayed_work mbox_stress_trigger_wk;
	struct workqueue_struct *main_pm_mfd_read_wq[NUM_OF_WQ];
	struct workqueue_struct *sub_pm_mfd_read_wq[NUM_OF_WQ];
	struct workqueue_struct *mbox_stress_trigger_wq;
	u8 update_reg;
	u8 main_channel;
	u8 sub_channel;
	/* mutex for RTC */
	struct mutex rtc_lock;
	/* mutex for Main/Sub PMIC */
	struct mutex main_pm_lock;
	struct mutex sub_pm_lock;
	int ctrlist_err_result;
	int init_done;
	int *regulator_lst_main;
	int *regulator_lst_sub;
	int num_of_main_regulator_regs;
	int num_of_sub_regulator_regs;
};

#define PT_VERSION		0xd005
#define SLC_STATE		0xd007
#define SLC_PT_INFO		0xd009

#define PT_PTID_MAX 64

struct acpm_slc_validity {
	struct delayed_work slc_request_wk[NUM_OF_WQ];
	struct delayed_work mbox_stress_trigger_wk;
	struct workqueue_struct *slc_request_wq[NUM_OF_WQ];
	struct workqueue_struct *mbox_stress_trigger_wq;
	const char *client_name[PT_PTID_MAX];
	u32 ptid[PT_PTID_MAX];
	u32 client_cnt;
};

struct acpm_mbox_test {
	bool wq_init_done;
	struct device *device;
	struct acpm_tmu_validity *tmu;
	struct acpm_dvfs_validity *dvfs;
	struct acpm_mfd_validity *mfd;
	struct acpm_slc_validity *slc;
};

struct acpm_dvfs_test_stats {
	unsigned int latency;	/* nano sec */
	unsigned int set_rate;
	unsigned int get_rate;
};

struct dvfs_frequency_table {
	unsigned int freq;	/* Hz */
};

struct stats_scale {
	unsigned int limit;		/* in us */
	unsigned int count;
};

#define MICRO_SEC               1000
#define TIME_SCALES             12
#define LATENCY_FAIL_CRITERIA   10  /*the percent of slow DVFS latency*/
#define SLOW_LATENCY_IDX        11  /*bucket idx for 2ms latency case*/

struct stats_scale buckets[TIME_SCALES] = { { 0, 0 }, { 1, 0 }, { 10, 0 },
{ 30, 0 }, { 50, 0 }, { 100, 0 }, { 200, 0 }, { 300, 0 },
{ 400, 0 }, { 500, 0 }, { 1000, 0 }, { 2000, 0 }
};				/* in us */

struct acpm_dvfs_domains {
	char *name;
	struct stats_scale *scales;
	unsigned int cal_id;
	unsigned int devfreq_id;
	unsigned int cpu_policy_id;
};

static struct acpm_dvfs_domains dvfs_dm_list[] = {
	[DVFS_MIF] = {
		      .name = "MIF",
		      .cal_id = ACPM_DVFS_MIF,
		      .devfreq_id = DEVFREQ_MIF,
		      .cpu_policy_id = -1,
		       },
	[DVFS_INT] = {
		      .name = "INT",
		      .cal_id = ACPM_DVFS_INT,
		      .devfreq_id = DEVFREQ_INT,
		      .cpu_policy_id = -1,
		       },
	[DVFS_CPUCL0] = {
			 .name = "CPUCL0",
			 .cal_id = ACPM_DVFS_CPUCL0,
			 .devfreq_id = -1,
			 .cpu_policy_id = CPUCL0_POLICY,
			  },
	[DVFS_CPUCL1] = {
			 .name = "CPUCL1",
			 .cal_id = ACPM_DVFS_CPUCL1,
			 .devfreq_id = -1,
			 .cpu_policy_id = CPUCL1_POLICY,
			  },
	[DVFS_CPUCL2] = {
			 .name = "CPUCL2",
			 .cal_id = ACPM_DVFS_CPUCL2,
			 .devfreq_id = -1,
			 .cpu_policy_id = CPUCL2_POLICY,
			  },
#if defined(CONFIG_SOC_ZUMA)
	[DVFS_DSU] = {
			 .name = "DSU",
			 .cal_id = ACPM_DVFS_DSU,
			 .devfreq_id = DEVFREQ_DSU,
			 .cpu_policy_id = -1,
			  },
	[DVFS_BCI] = {
			 .name = "BCI",
			 .cal_id = ACPM_DVFS_BCI,
			 .devfreq_id = DEVFREQ_BCI,
			 .cpu_policy_id = -1,
			  },
#endif
	[DVFS_G3D] = {
		      .name = "G3D",
		      .cal_id = ACPM_DVFS_G3D,
		      .devfreq_id = -1,
		      .cpu_policy_id = -1,
		       },
	[DVFS_G3DL2] = {
			.name = "G3DL2",
			.cal_id = ACPM_DVFS_G3DL2,
			.devfreq_id = -1,
			.cpu_policy_id = -1,
			 },
	[DVFS_TPU] = {
		      .name = "TPU",
		      .cal_id = ACPM_DVFS_TPU,
		      .devfreq_id = -1,
		      .cpu_policy_id = -1,
		       },
	[DVFS_INTCAM] = {
			 .name = "INTCAM",
			 .cal_id = ACPM_DVFS_INTCAM,
			 .devfreq_id = DEVFREQ_INTCAM,
			 .cpu_policy_id = -1,
			  },
	[DVFS_TNR] = {
		      .name = "TNR",
		      .cal_id = ACPM_DVFS_TNR,
		      .devfreq_id = DEVFREQ_TNR,
		      .cpu_policy_id = -1,
		       },
	[DVFS_CAM] = {
		      .name = "CAM",
		      .cal_id = ACPM_DVFS_CAM,
		      .devfreq_id = DEVFREQ_CAM,
		      .cpu_policy_id = -1,
		       },
};

struct acpm_dvfs_dm {
	char *name;
	unsigned int max_freq;
	unsigned int min_freq;
	unsigned int size;
	unsigned int total_cycle_cnt;
	unsigned int devfreq_id;
	unsigned int cpu_policy_id;
	unsigned int prev_jump_idx;
	struct dvfs_frequency_table *table;
	struct acpm_dvfs_test_stats *stats;
	struct stats_scale *scales;
};

struct acpm_dvfs_test {
	unsigned int max_freq;
	unsigned int min_freq;
	unsigned int size;
	int init_done;

	struct acpm_dvfs_dm *dm[NUM_OF_DVFS_DOMAINS];
};

struct tmu_ipc_request {
	u16 ctx;		/* LSB */
	u16 fw_use;		/* MSB */
	u8 type;
	u8 rsvd;
	u8 tzid;
	u8 rsvd2;
	u8 req_rsvd0;
	u8 req_rsvd1;
	u8 req_rsvd2;
	u8 req_rsvd3;
	u8 req_rsvd4;
	u8 req_rsvd5;
	u8 req_rsvd6;
	u8 req_rsvd7;
};

struct tmu_ipc_response {
	u16 ctx;		/* LSB */
	u16 fw_use;		/* MSB */
	u8 type;
	s8 ret;
	u8 tzid;
	u8 temp;
	u8 stat;
	u8 rsvd;
	u8 rsvd2;
	u8 rsvd3;
	u32 reserved;
};

union tmu_ipc_message {
	u32 data[4];
	struct tmu_ipc_request req;
	struct tmu_ipc_response resp;
};

extern u32 gs_chipid_get_type(void);
extern u32 gs_chipid_get_revision(void);
static int acpm_dvfs_set_cpufreq(unsigned int dm_id, unsigned int rate,
				 int cycle);
static int acpm_dvfs_set_devfreq(unsigned int dm_id, unsigned int rate,
				 int cycle);
static int init_domain_freq_table(struct acpm_dvfs_test *dvfs, int dm_id);
static unsigned int get_random_rate(unsigned int dm_id);
static int dvfs_freq_table_init(void);
static int acpm_pmic_ctrlist_stress(void);

#endif
