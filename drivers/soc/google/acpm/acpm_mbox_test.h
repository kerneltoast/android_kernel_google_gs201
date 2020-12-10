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
	ACPM_MBOX_TMU_TEST_STOP,
	ACPM_MBOX_TMU_TEST_START,
	ACPM_MBOX_TEST_CMD_MAX,
};

#define NUM_OF_WQ       16

/* IPC Mailbox Channel */
#define IPC_AP_TMU      9

/* IPC Request Types */
#define TMU_IPC_READ_TEMP	0x02
#define	TMU_IPC_AP_SUSPEND	0x04
#define	TMU_IPC_AP_RESUME	0x10
#define TMU_IPC_TMU_CONTROL	0x13

struct acpm_tmu_mbox_test {
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
