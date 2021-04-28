/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright 2020 Google LLC
 *
 */
#ifndef __ACPM_MBOX_TEST_H__
#define __ACPM_MBOX_TEST_H__

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
	END,
};

enum acpm_mbox_test_commands {
	ACPM_MBOX_TEST_STOP,
	ACPM_MBOX_TEST_START,
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

#define NUM_OF_WQ               16

/* IPC Mailbox Channel */
#define IPC_AP_TMU              9

/* IPC Request Types */
#define TMU_IPC_READ_TEMP       0x02
#define TMU_IPC_AP_SUSPEND      0x04
#define TMU_IPC_AP_RESUME       0x10
#define TMU_IPC_TMU_CONTROL     0x13

#define DVFS_TEST_CYCLE         20

struct acpm_tmu_validity {
	bool wq_init_done;
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

struct acpm_mbox_test {
	struct device *device;
	struct acpm_tmu_validity *tmu;
};

struct acpm_dvfs_test_stats {
	unsigned int latency; /* nano sec */
	unsigned int set_rate;
	unsigned int get_rate;
};

struct dvfs_frequency_table {
	unsigned int freq; /* Hz */
};

struct stats_scale {
	int limit;  /* in us */
	int count;
};

#define MICRO_SEC               1000
#define TIME_SCALES             10

struct stats_scale buckets[TIME_SCALES] = {{0, 0}, {1, 0}, {10, 0},
					{20, 0}, {40, 0}, {60, 0}, {80, 0},
					{100, 0}, {1000, 0}, {10000, 0}}; /* in us*/

struct acpm_dvfs_domains {
	char *name;
	struct stats_scale *scales;
};

static struct acpm_dvfs_domains gs101_dvfs_domains[] = {
	[DVFS_MIF]	= {
		.name	= "MIF",
	},
	[DVFS_INT]	= {
		.name	= "INT",
	},
	[DVFS_CPUCL0]	= {
		.name	= "CPUCL0",
	},
	[DVFS_CPUCL1]	= {
		.name	= "CPUCL1",
	},
	[DVFS_CPUCL2]	= {
		.name	= "CPUCL2",
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

	struct acpm_dvfs_dm *dm[NUM_OF_DVFS_DOMAINS];
};

struct tmu_ipc_request {
	u16 ctx;	/* LSB */
	u16 fw_use;	/* MSB */
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
	u16 ctx;	/* LSB */
	u16 fw_use;	/* MSB */
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

#endif
