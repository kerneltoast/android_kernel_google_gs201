/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2023 Google LLC
 */

#ifndef MAX77779_I2CM_H_
#define MAX77779_I2CM_H_

#include "max77779_regs.h"

#define DONEI_SET(v) _max77779_i2cm_interrupt_donei_set(0, v)
#define DONEI_GET(v) _max77779_i2cm_interrupt_donei_get(v)
#define ERRI_SET(v) _max77779_i2cm_interrupt_erri_set(0, v)
#define ERRI_GET(v) _max77779_i2cm_interrupt_erri_get(v)

#define DONEIM_SET(v) _max77779_i2cm_intmask_doneim_set(0, v)
#define DONEIM_GET(v) _max77779_i2cm_intmask_doneim_get(v)
#define ERRIM_SET(v) _max77779_i2cm_intmask_errim_set(0, v)
#define ERRIM_GET(v) _max77779_i2cm_intmask_errim_get(v)

#define ERROR_SET(v) _max77779_i2cm_status_error_set(0, v)
#define ERROR_GET(v) _max77779_i2cm_status_error_get(v)

#define I2CEN_SET(v) _max77779_i2cm_control_i2cen_set(0, v)
#define I2CEN_GET(v) _max77779_i2cm_control_i2cen_get(v)
#define CLOCK_SPEED_SET(v) _max77779_i2cm_control_clock_speed_set(0, v)
#define CLOCK_SPEED_GET(v) _max77779_i2cm_control_clock_speed_get(v)

#define SID_SET(v) _max77779_i2cm_sladd_slave_id_set(0, v)
#define SID_GET(v) _max77779_i2cm_sladd_slave_id_get(v)

#define TXCNT_SET(v) _max77779_i2cm_txdata_cnt_txcnt_set(0, v)
#define TXCNT_GET(v) _max77779_i2cm_txdata_cnt_txcnt_get(v)

#define I2CMWRITE_SET(v) _max77779_i2cm_cmd_i2cmwrite_set(0, v)
#define I2CMWRITE_GET(v) _max77779_i2cm_cmd_i2cmwrite_get(v)

#define I2CMREAD_SET(v) _max77779_i2cm_cmd_i2cmread_set(0, v)
#define I2CMREAD_GET(v) _max77779_i2cm_cmd_i2cmread_get(v)

#define I2CM_ERR_ARBITRATION_LOSS(status_err)	(!!((status_err) & BIT(0)))
#define I2CM_ERR_TIMEOUT(status_err)		(!!((status_err) & BIT(1)))
#define I2CM_ERR_ADDRESS_NACK(status_err)	(!!((status_err) & BIT(2)))
#define I2CM_ERR_DATA_NACK(status_err)		(!!((status_err) & BIT(3)))
#define I2CM_ERR_RX_FIFO_NA(status_err)		(!!((status_err) & BIT(4)))
#define I2CM_ERR_START_OUT_SEQ(status_err)	(!!((status_err) & BIT(5)))
#define I2CM_ERR_STOP_OUT_SEQ(status_err)	(!!((status_err) & BIT(6)))

#define I2CM_MAX_REGISTER			MAX77779_I2CM_RX_BUFFER_31

#define MAX77779_TIMEOUT_DEFAULT		0xff
#define MAX77779_MAX_TIMEOUT			0xff
#define MAX77779_COMPLETION_TIMEOUT_MS_DEFAULT	20
#define MAX77779_MAX_SPEED			0x03
#define MAX77779_SPEED_DEFAULT			0x00

#define MAX77779_I2CM_MAX_WRITE \
		(MAX77779_I2CM_TX_BUFFER_33 - MAX77779_I2CM_TX_BUFFER_0 + 1)
#define MAX77779_I2CM_MAX_READ \
		(MAX77779_I2CM_RX_BUFFER_31 - MAX77779_I2CM_RX_BUFFER_0 + 1)

struct max77779_i2cm_info {
	struct i2c_adapter	adap;  /* bus */
	struct i2c_client	*client;
	int			irq;
	struct device		*dev;
	struct regmap		*regmap;
	struct completion	xfer_done;
	unsigned int		timeout;
	unsigned int		completion_timeout_ms;
	unsigned int		speed;
	u8			reg_vals[I2CM_MAX_REGISTER + 1];
};

int max77779_i2cm_init(struct max77779_i2cm_info *info);
void max77779_i2cm_remove(struct max77779_i2cm_info *info);

#endif
