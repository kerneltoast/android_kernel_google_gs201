/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright 2020 Google LLC
 *
 */
#ifndef __ACPM_MBOX_TEST_H__
#define __ACPM_MBOX_TEST_H__
#include <soc/google/pt.h>

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
	ACPM_DVFS_TEST_RESULT,
	ACPM_DVFS_CMD_MAX,
};

enum domains {
	DVFS_MIF = 0,
	DVFS_INT,
	DVFS_CPUCL0,
	DVFS_CPUCL1,
	DVFS_CPUCL2,
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
};

#define PMIC_RANDOM_ADDR_RANGE  0x1FF

/* Set 365 days as a year */
#define SECS_PER_YEAR   977616000
/* Set 31 days as a month */
#define SECS_PER_MONTH  2678400
#define SECS_PER_DAY    86400
#define SECS_PER_HR     3600
#define SECS_PER_MIN    60

#define BUCK1M_OUT1     (0x119)
#define BUCK2M_OUT1     (0x11C)
#define BUCK3M_OUT1     (0x11F)
#define BUCK4M_OUT1     (0x122)
#define BUCK5M_OUT1     (0x125)
#define BUCK6M_OUT1     (0x128)
#define BUCK7M_OUT1     (0x12B)
#define BUCK10M_OUT1    (0x134)
#define LDO12M_CTRL1    (0x14C)
#define LDO13M_CTRL1    (0x14E)
#define LDO15M_CTRL1    (0x151)

#define BUCK1S_OUT1     (0x113)
#define BUCK2S_OUT1     (0x116)
#define BUCK3S_OUT1     (0x119)
#define BUCK4S_OUT      (0x11C)
#define BUCK5S_OUT      (0x11E)
#define BUCK8S_OUT1     (0x126)
#define BUCK9S_OUT1     (0x129)
#define LDO2S_CTRL1     (0x143)

static u16 def_lck_regs_m[NUM_OF_PMIC_MASTER] = {
	BUCK1M_OUT1, BUCK2M_OUT1, BUCK3M_OUT1, BUCK4M_OUT1,
	BUCK5M_OUT1, BUCK6M_OUT1, BUCK7M_OUT1, BUCK10M_OUT1,
	LDO12M_CTRL1, LDO13M_CTRL1, LDO15M_CTRL1
};

static u16 def_lck_regs_s[NUM_OF_PMIC_ID - NUM_OF_PMIC_MASTER] = {
	BUCK1S_OUT1, BUCK2S_OUT1, BUCK3S_OUT1, BUCK4S_OUT,
	BUCK5S_OUT, BUCK8S_OUT1, BUCK9S_OUT1, LDO2S_CTRL1
};

struct acpm_mfd_validity {
	struct i2c_client *s2mpg10_pmic;
	struct i2c_client *s2mpg11_pmic;
	struct i2c_client *rtc;
	struct delayed_work s2mpg10_mfd_read_wk[NUM_OF_WQ];
	struct delayed_work s2mpg11_mfd_read_wk[NUM_OF_WQ];
	struct delayed_work mbox_stress_trigger_wk;
	struct workqueue_struct *s2mpg10_mfd_read_wq[NUM_OF_WQ];
	struct workqueue_struct *s2mpg11_mfd_read_wq[NUM_OF_WQ];
	struct workqueue_struct *mbox_stress_trigger_wq;
	u8 update_reg;
	/* mutex for RTC */
	struct mutex lock;
	int ctrlist_err_result;
	int init_done;
};

#define PT_VERSION		0xd005
#define SLC_STATE		0xd007
#define SLC_PT_INFO		0xd009

#define PT_PTID_MAX 64

struct pt_handle {		/* one per client */
	struct mutex mt;	/* serialize write access to the handle */
	struct list_head list;
	pt_resize_callback_t resize_callback;
	int id_cnt;
	struct pt_pts *pts;	/* client partitions */
	struct device_node *node;	/* client node */
	struct ctl_table *sysctl_table;
	struct ctl_table_header *sysctl_header;
	void *data;		/* client private data */
};

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
	int limit;		/* in us */
	int count;
};

#define MICRO_SEC               1000
#define TIME_SCALES             10

struct stats_scale buckets[TIME_SCALES] = { { 0, 0 }, { 1, 0 }, { 10, 0 },
{ 20, 0 }, { 40, 0 }, { 60, 0 }, { 80, 0 },
{ 100, 0 }, { 1000, 0 }, { 10000, 0 }
};				/* in us */

struct acpm_dvfs_domains {
	char *name;
	struct stats_scale *scales;
};

static struct acpm_dvfs_domains gs101_dvfs_domains[] = {
	[DVFS_MIF] = {
		      .name = "MIF",
		       },
	[DVFS_INT] = {
		      .name = "INT",
		       },
	[DVFS_CPUCL0] = {
			 .name = "CPUCL0",
			  },
	[DVFS_CPUCL1] = {
			 .name = "CPUCL1",
			  },
	[DVFS_CPUCL2] = {
			 .name = "CPUCL2",
			  },
};

struct acpm_dvfs_dm {
	char *name;
	unsigned int max_freq;
	unsigned int min_freq;
	unsigned int size;
	unsigned int total_cycle_cnt;
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
static int init_domain_freq_table(struct acpm_dvfs_test *dvfs, int cal_id,
				  int dm_id);
static unsigned int get_random_rate(unsigned int dm_id);
static int dvfs_freq_table_init(void);
static int acpm_pmic_ctrlist_stress(void);

#endif
