/* SPDX-License-Identifier: GPL-2.0-only
 *
 * exynos_acpm_tmu.h - ACPM TMU plugin interface
 *
 * Copyright (C) 2017 Samsung Electronics
 */

#ifndef __EXYNOS_ACPM_TMU_H__
#define __EXYNOS_ACPM_TMU_H__

/* Return values */
#define RET_SUCCESS		0
#define RET_FAIL		-1
#define RET_OK			RET_SUCCESS
#define RET_NOK			RET_FAIL

/* Return values - error types (minus) */
#define ERR_REQ_TYPE		2
#define ERR_TZ_ID		3
#define ERR_TEMP		4
#define ERR_APM_IRQ		5
#define ERR_APM_DIVIDER		6

/* Return values - capabilities */
#define CAP_APM_IRQ		0x1
#define CAP_APM_DIVIDER		0x2

/* IPC Request Types */
#define TMU_IPC_INIT		0x01
#define TMU_IPC_READ_TEMP	0x02
#define	TMU_IPC_AP_SUSPEND	0x04
#define	TMU_IPC_CP_CALL		0x08
#define	TMU_IPC_AP_RESUME	0x10

#define TMU_IPC_THRESHOLD	0x11
#define TMU_IPC_INTEN		0x12
#define TMU_IPC_TMU_CONTROL	0x13
#define TMU_IPC_IRQ_CLEAR	0x14
#define TMU_IPC_EMUL_TEMP	0x15
#define TMU_IPC_HYSTERESIS	0x16
#define TMU_IPC_REG_READ	0xFE
#define TMU_IPC_REG_WRITE	0xFF

/* TMU register offset shift for IPC messages */
#define TMU_REG_OFFSET_SHIFT_8	(8)

/* TMU register offset MASK for IPC messages */
#define TMU_REG_OFFSET_MASK_1	(0xFF)
#define TMU_REG_OFFSET_MASK_2	(TMU_REG_OFFSET_MASK_1 << TMU_REG_OFFSET_SHIFT_8)

/* TMU register value shift for IPC messages */
#define TMU_REG_VAL_SHIFT_8	(8)
#define TMU_REG_VAL_SHIFT_16	(16)
#define TMU_REG_VAL_SHIFT_24	(24)

/* TMU register value mask for IPC messages */
#define TMU_REG_VAL_MASK_1	(0xFF)
#define TMU_REG_VAL_MASK_2	(TMU_REG_VAL_MASK_1 << TMU_REG_VAL_SHIFT_8)
#define TMU_REG_VAL_MASK_3	(TMU_REG_VAL_MASK_1 << TMU_REG_VAL_SHIFT_16)
#define TMU_REG_VAL_MASK_4	(TMU_REG_VAL_MASK_1 << TMU_REG_VAL_SHIFT_24)

/*
 * 16-byte TMU IPC message format (REQ)
 *  (MSB)    3          2          1          0
 * ---------------------------------------------
 * |        fw_use       |         ctx         |
 * ---------------------------------------------
 * |          | tzid     |          | type     |
 * ---------------------------------------------
 * |          |          |          |          |
 * ---------------------------------------------
 * |          |          |          |          |
 * ---------------------------------------------
 */
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

/*
 * 16-byte TMU IPC message format (RESP)
 *  (MSB)    3          2          1          0
 * ---------------------------------------------
 * |        fw_use       |         ctx         |
 * ---------------------------------------------
 * | temp     |  tz_id   | ret      | type     |
 * ---------------------------------------------
 * |          |          |          | stat     |
 * ---------------------------------------------
 * |          |          |          |          |
 * ---------------------------------------------
 */
struct tmu_ipc_response {
	u16 ctx;	/* LSB */
	u16 fw_use;	/* MSB */
	u8 type;
	s8 ret;
	u8 tzid;
	u8 temp;
	u8 stat;
	u8 rsvd0;
	u8 rsvd1;
	u8 rsvd2;
	u8 rsvd3;
	u8 rsvd4;
	u8 rsvd5;
	u8 rsvd6;
};

union tmu_ipc_message {
	u32 data[4];
	struct tmu_ipc_request req;
	struct tmu_ipc_response resp;
};

struct acpm_tmu_cap {
	bool acpm_irq;
	bool acpm_divider;
};

int exynos_acpm_tmu_set_init(struct acpm_tmu_cap *cap);
int exynos_acpm_tmu_set_read_temp(int tz, int *temp, int *stat);
int exynos_acpm_tmu_set_suspend(int flag);
int exynos_acpm_tmu_set_cp_call(void);
int exynos_acpm_tmu_set_resume(void);
int exynos_acpm_tmu_ipc_dump(int no, unsigned int dump[]);
bool exynos_acpm_tmu_is_test_mode(void);
void exynos_acpm_tmu_set_test_mode(bool mode);
void exynos_acpm_tmu_enable_log(bool mode);
bool exynos_acpm_tmu_is_log_enabled(void);

void exynos_acpm_tmu_set_threshold(int tz, unsigned char temp[]);
void exynos_acpm_tmu_set_hysteresis(int tz, unsigned char hyst[]);
void exynos_acpm_tmu_set_interrupt_enable(int tz, unsigned char inten);
void exynos_acpm_tmu_tz_control(int tz, bool enable);
void exynos_acpm_tmu_clear_tz_irq(int tz);
void exynos_acpm_tmu_set_emul_temp(int tz, unsigned char temp);
void exynos_acpm_tmu_reg_read(u8 tmu_id, u16 offset, u32 *val);
void exynos_acpm_tmu_reg_write(u8 tmu_id, u16 offset, u32 val);

int exynos_acpm_tmu_init(void);

#endif /* __EXYNOS_ACPM_TMU_H__ */
