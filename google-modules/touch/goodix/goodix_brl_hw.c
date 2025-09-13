/*
 * Goodix Touchscreen Driver
 * Copyright (C) 2020 - 2021 Goodix, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */
#include "goodix_ts_core.h"

/* berlin_A SPI mode setting */
#define GOODIX_SPI_MODE_REG 0xC900
#define GOODIX_SPI_NORMAL_MODE_0 0x01

/* berlin_A D12 setting */
#define GOODIX_REG_CLK_STA0 0xD807
#define GOODIX_CLK_STA0_ENABLE 0xFF
#define GOODIX_REG_CLK_STA1 0xD806
#define GOODIX_CLK_STA1_ENABLE 0x77
#define GOODIX_REG_TRIM_D12 0xD006
#define GOODIX_TRIM_D12_LEVEL 0x3C
#define GOODIX_REG_RESET 0xD808
#define GOODIX_RESET_EN 0xFA
#define HOLD_CPU_REG_W 0x0002
#define HOLD_CPU_REG_R 0x2000

#define DEV_CONFIRM_VAL 0xAA
#define BOOTOPTION_ADDR 0x10000
#define FW_VERSION_INFO_ADDR_BRA 0x1000C
#define FW_VERSION_INFO_ADDR 0x10014

#define GOODIX_IC_INFO_MAX_LEN 1024
#define GOODIX_IC_INFO_ADDR_BRA 0x10068
#define GOODIX_IC_INFO_ADDR 0x10070

enum brl_request_code {
	BRL_REQUEST_CODE_CONFIG = 0x01,
	BRL_REQUEST_CODE_REF_ERR = 0x02,
	BRL_REQUEST_CODE_RESET = 0x03,
	BRL_REQUEST_CODE_CLOCK = 0x04,
	BRL_REQUEST_CODE_UPDATE = 0x05,
	BRL_REQUEST_PEN_FREQ_HOP = 0x10,
};

static int brl_select_spi_mode(struct goodix_ts_core *cd)
{
	int ret;
	int i;
	u8 w_value = GOODIX_SPI_NORMAL_MODE_0;
	u8 r_value;

	if (cd->bus->bus_type == GOODIX_BUS_TYPE_I2C ||
		cd->bus->ic_type != IC_TYPE_BERLIN_A)
		return 0;

	for (i = 0; i < GOODIX_RETRY_5; i++) {
		cd->hw_ops->write(cd, GOODIX_SPI_MODE_REG, &w_value, 1);
		ret = cd->hw_ops->read(cd, GOODIX_SPI_MODE_REG, &r_value, 1);
		if (!ret && r_value == w_value)
			return 0;
	}
	ts_err("failed switch SPI mode, ret:%d r_value:%02x", ret, r_value);
	return -EINVAL;
}

static int brl_dev_confirm(struct goodix_ts_core *cd)
{
	struct goodix_ts_hw_ops *hw_ops = cd->hw_ops;
	int ret = 0;
	int retry = GOODIX_RETRY_3;
	u8 tx_buf[8] = { 0 };
	u8 rx_buf[8] = { 0 };

	if (cd->bus->ic_type == IC_TYPE_BERLIN_A &&
		cd->bus->bus_type == GOODIX_BUS_TYPE_SPI)
		return brl_select_spi_mode(cd);

	memset(tx_buf, DEV_CONFIRM_VAL, sizeof(tx_buf));
	while (retry--) {
		ret = hw_ops->write(
			cd, BOOTOPTION_ADDR, tx_buf, sizeof(tx_buf));
		if (ret < 0)
			return ret;
		ret = hw_ops->read(cd, BOOTOPTION_ADDR, rx_buf, sizeof(rx_buf));
		if (ret < 0)
			return ret;
		ts_info("device confirm val: %*ph.", (int)sizeof(rx_buf), rx_buf); /* [GOOG] */
		if (!memcmp(tx_buf, rx_buf, sizeof(tx_buf)))
			break;
		usleep_range(5000, 5100);
	}

	if (retry < 0) {
		ts_err("device confirm failed, rx_buf:%*ph", 8, rx_buf);
		return -EINVAL;
	}

	ts_info("device connected");
	return ret;
}

static int brl_reset_after(struct goodix_ts_core *cd)
{
	u8 reg_val[2] = { 0 };
	u8 temp_buf[12] = { 0 };
	int ret;
	int retry;

	if (cd->bus->ic_type != IC_TYPE_BERLIN_A)
		return 0;

	ts_info("IN");

	/* hold cpu */
	retry = GOODIX_RETRY_10;
	while (retry--) {
		reg_val[0] = 0x01;
		reg_val[1] = 0x00;
		ret = cd->hw_ops->write(cd, HOLD_CPU_REG_W, reg_val, 2);
		ret |= cd->hw_ops->read(cd, HOLD_CPU_REG_R, &temp_buf[0], 4);
		ret |= cd->hw_ops->read(cd, HOLD_CPU_REG_R, &temp_buf[4], 4);
		ret |= cd->hw_ops->read(cd, HOLD_CPU_REG_R, &temp_buf[8], 4);
		if (!ret && !memcmp(&temp_buf[0], &temp_buf[4], 4) &&
			!memcmp(&temp_buf[4], &temp_buf[8], 4) &&
			!memcmp(&temp_buf[0], &temp_buf[8], 4)) {
			break;
		}
	}
	if (retry < 0) {
		ts_err("failed to hold cpu, status:%*ph", 12, temp_buf);
		return -EINVAL;
	}

	/* enable sta0 clk */
	retry = GOODIX_RETRY_5;
	while (retry--) {
		reg_val[0] = GOODIX_CLK_STA0_ENABLE;
		ret = cd->hw_ops->write(cd, GOODIX_REG_CLK_STA0, reg_val, 1);
		ret |= cd->hw_ops->read(cd, GOODIX_REG_CLK_STA0, temp_buf, 1);
		if (!ret && temp_buf[0] == GOODIX_CLK_STA0_ENABLE)
			break;
	}
	if (retry < 0) {
		ts_err("failed to enable group0 clock, ret:%d status:%02x", ret,
			temp_buf[0]);
		return -EINVAL;
	}

	/* enable sta1 clk */
	retry = GOODIX_RETRY_5;
	while (retry--) {
		reg_val[0] = GOODIX_CLK_STA1_ENABLE;
		ret = cd->hw_ops->write(cd, GOODIX_REG_CLK_STA1, reg_val, 1);
		ret |= cd->hw_ops->read(cd, GOODIX_REG_CLK_STA1, temp_buf, 1);
		if (!ret && temp_buf[0] == GOODIX_CLK_STA1_ENABLE)
			break;
	}
	if (retry < 0) {
		ts_err("failed to enable group1 clock, ret:%d status:%02x", ret,
			temp_buf[0]);
		return -EINVAL;
	}

	/* set D12 level */
	retry = GOODIX_RETRY_5;
	while (retry--) {
		reg_val[0] = GOODIX_TRIM_D12_LEVEL;
		ret = cd->hw_ops->write(cd, GOODIX_REG_TRIM_D12, reg_val, 1);
		ret |= cd->hw_ops->read(cd, GOODIX_REG_TRIM_D12, temp_buf, 1);
		if (!ret && temp_buf[0] == GOODIX_TRIM_D12_LEVEL)
			break;
	}
	if (retry < 0) {
		ts_err("failed to set D12, ret:%d status:%02x", ret,
			temp_buf[0]);
		return -EINVAL;
	}

	usleep_range(5000, 5100);
	/* soft reset */
	reg_val[0] = GOODIX_RESET_EN;
	ret = cd->hw_ops->write(cd, GOODIX_REG_RESET, reg_val, 1);
	if (ret < 0)
		return ret;

	/* select spi mode */
	ret = brl_select_spi_mode(cd);
	if (ret < 0)
		return ret;

	ts_info("OUT");

	return 0;
}

static int brl_power_on(struct goodix_ts_core *cd, bool on)
{
	int ret = 0;
	int iovdd_gpio = cd->board_data.iovdd_gpio;
	int avdd_gpio = cd->board_data.avdd_gpio;
	int reset_gpio = cd->board_data.reset_gpio;

	if (on) {
		if (iovdd_gpio > 0) {
			gpio_direction_output(iovdd_gpio, 1);
		} else if (cd->iovdd) {
			ret = regulator_enable(cd->iovdd);
			if (ret < 0) {
				ts_err("Failed to enable iovdd:%d", ret);
				goto power_off;
			}
		}
		usleep_range(3000, 3100);
		if (avdd_gpio > 0) {
			gpio_direction_output(avdd_gpio, 1);
		} else if (cd->avdd) {
			ret = regulator_enable(cd->avdd);
			if (ret < 0) {
				ts_err("Failed to enable avdd:%d", ret);
				goto power_off;
			}
		}
		usleep_range(15000, 15100);
		gpio_direction_output(reset_gpio, 1);
		usleep_range(4000, 4100);
		ret = brl_dev_confirm(cd);
/* [GOOG]
 * Skip the check brl_dev_confirm() as FW recovery mechanism
 * to force FW update(if possible).
		if (ret < 0)
			goto power_off;
*/
		ret = brl_reset_after(cd);
		if (ret < 0)
			goto power_off;

		msleep(goodix_get_normal_reset_delay(cd));
		return 0;
	}

power_off:
	gpio_direction_output(reset_gpio, 0);
	if (iovdd_gpio > 0)
		gpio_direction_output(iovdd_gpio, 0);
	else if (cd->iovdd)
		regulator_disable(cd->iovdd);
	if (avdd_gpio > 0)
		gpio_direction_output(avdd_gpio, 0);
	else if (cd->avdd)
		regulator_disable(cd->avdd);
	return ret;
}

static int brl_suspend(struct goodix_ts_core *cd)
{
	u32 cmd_reg = cd->ic_info.misc.cmd_addr;
	u8 sleep_cmd[] = { 0x00, 0x00, 0x05, 0xC4, 0x01, 0xCA, 0x00 };

	return cd->hw_ops->write(cd, cmd_reg, sleep_cmd, sizeof(sleep_cmd));
}

static int brl_resume(struct goodix_ts_core *cd)
{
	u32 cmd_reg = cd->ic_info.misc.cmd_addr;
	u8 cmd_buf[] = { 0x00, 0x00, 0x04, 0xA7, 0xAB, 0x00 };

	return cd->hw_ops->write(cd, cmd_reg, cmd_buf, sizeof(cmd_buf));
}

#define GOODIX_GESTURE_CMD_BA 0x12
#define GOODIX_GESTURE_CMD 0xA6
static int brl_gesture(struct goodix_ts_core *cd, int gesture_type)
{
	struct goodix_ts_cmd cmd;
	u8 val;

	if ((cd->gesture_type & GESTURE_SINGLE_TAP) &&
		(cd->gesture_type & GESTURE_FOD_PRESS)) {
		val = 0x0F;
	} else if (cd->gesture_type & GESTURE_SINGLE_TAP) {
		val = 0x2F;
	} else if (cd->gesture_type & GESTURE_FOD_PRESS) {
		val = 0x1F;
	} else {
		val = 0xFF;
	}

	if (cd->bus->ic_type == IC_TYPE_BERLIN_A)
		cmd.cmd = GOODIX_GESTURE_CMD_BA;
	else
		cmd.cmd = GOODIX_GESTURE_CMD;
	cmd.len = 6;
	cmd.data[0] = 0xff;
	cmd.data[1] = val;
	if (cd->hw_ops->send_cmd(cd, &cmd))
		ts_err("failed send gesture cmd");

	return 0;
}

static int brl_reset(struct goodix_ts_core *cd, int delay)
{
	ts_info("chip_reset");

	/*
	 * ESD check will fail on firmware reset. When ESD check is failed,
	 * it will reset firmware again. Skip ESD check to avoid double reset.
	 */
	cd->ts_esd.skip_once = true;

	gpio_direction_output(cd->board_data.reset_gpio, 0);
	usleep_range(2000, 2100);
	gpio_direction_output(cd->board_data.reset_gpio, 1);
	if (delay < 20)
		usleep_range(delay * 1000, delay * 1000 + 100);
	else
		msleep(delay);

	return brl_select_spi_mode(cd);
}

static int brl_irq_enable(struct goodix_ts_core *cd, bool enable)
{
	if (enable && !atomic_cmpxchg(&cd->irq_enabled, 0, 1)) {
		enable_irq(cd->irq);
		ts_debug("Irq enabled");
		return 0;
	}

	if (!enable && atomic_cmpxchg(&cd->irq_enabled, 1, 0)) {
		disable_irq(cd->irq);
		ts_debug("Irq disabled");
		return 0;
	}
	// ts_info("warnning: irq deepth inbalance!");
	return 0;
}

static int brl_disable_irq_nosync(struct goodix_ts_core *cd)
{
	if (atomic_cmpxchg(&cd->irq_enabled, 1, 0)) {
		disable_irq_nosync(cd->irq);
		ts_debug("Irq disabled");
	}
	return 0;
}

static int brl_read(struct goodix_ts_core *cd, unsigned int addr,
	unsigned char *data, unsigned int len)
{
	struct goodix_bus_interface *bus = cd->bus;

	return bus->read(bus->dev, addr, data, len);
}

static int brl_read_fast(struct goodix_ts_core *cd, unsigned int addr,
	struct goodix_rx_package *package, unsigned int len)
{
	struct goodix_bus_interface *bus = cd->bus;

	return bus->read_fast(bus->dev, addr, package, len);
}

static int brl_write(struct goodix_ts_core *cd, unsigned int addr,
	unsigned char *data, unsigned int len)
{
	struct goodix_bus_interface *bus = cd->bus;

	return bus->write(bus->dev, addr, data, len);
}

/* command ack info */
#define CMD_ACK_IDLE 0x01
#define CMD_ACK_BUSY 0x02
#define CMD_ACK_BUFFER_OVERFLOW 0x03
#define CMD_ACK_CHECKSUM_ERROR 0x04
#define CMD_ACK_OK 0x80

#define GOODIX_CMD_RETRY 6
static int brl_send_cmd(struct goodix_ts_core *cd, struct goodix_ts_cmd *cmd)
{
	int ret, retry, i;
	struct goodix_ts_cmd cmd_ack;
	struct goodix_ic_info_misc *misc = &cd->ic_info.misc;
	struct goodix_ts_hw_ops *hw_ops = cd->hw_ops;

	mutex_lock(&cd->cmd_lock);
	cmd->state = 0;
	cmd->ack = 0;
	goodix_append_checksum(
		&(cmd->buf[2]), cmd->len - 2, CHECKSUM_MODE_U8_LE);
	ts_debug("cmd data %*ph", cmd->len, &(cmd->buf[2]));

	retry = 0;
	while (retry++ < GOODIX_CMD_RETRY) {
		ret = hw_ops->write(cd, misc->cmd_addr, cmd->buf, sizeof(*cmd));
		if (ret < 0) {
			ts_err("failed write command");
			goto exit;
		}
		for (i = 0; i < GOODIX_CMD_RETRY; i++) {
			/* check command result */
			ret = hw_ops->read(cd, misc->cmd_addr, cmd_ack.buf,
				sizeof(cmd_ack));
			if (ret < 0) {
				ts_err("failed read command ack, %d", ret);
				goto exit;
			}
			ts_debug("cmd ack data %*ph", (int)sizeof(cmd_ack),
				cmd_ack.buf);
			if (cmd_ack.ack == CMD_ACK_OK) {
				ret = 0;
				goto exit;
			}
			if (cmd_ack.ack == CMD_ACK_BUSY ||
				cmd_ack.ack == 0x00) {
				usleep_range(1000, 1100);
				continue;
			}
			if (cmd_ack.ack == CMD_ACK_BUFFER_OVERFLOW)
				usleep_range(10000, 11000);
			usleep_range(1000, 1100);
			break;
		}
	}
	ret = -EINVAL;
	ts_err("failed get valid cmd ack");
exit:
	mutex_unlock(&cd->cmd_lock);
	return ret;
}

#pragma pack(1)
struct goodix_config_head {
	union {
		struct {
			u8 panel_name[8];
			u8 fw_pid[8];
			u8 fw_vid[4];
			u8 project_name[8];
			u8 file_ver[2];
			u32 cfg_id;
			u8 cfg_ver;
			u8 cfg_time[8];
			u8 reserved[15];
			u8 flag;
			u16 cfg_len;
			u8 cfg_num;
			u16 checksum;
		};
		u8 buf[64];
	};
};
#pragma pack()

#define CONFIG_CND_LEN 4
#define CONFIG_CMD_START 0x04
#define CONFIG_CMD_WRITE 0x05
#define CONFIG_CMD_EXIT 0x06
#define CONFIG_CMD_READ_START 0x07
#define CONFIG_CMD_READ_EXIT 0x08

#define CONFIG_CMD_STATUS_PASS 0x80
#define CONFIG_CMD_WAIT_RETRY 20

static int wait_cmd_status(
	struct goodix_ts_core *cd, u8 target_status, int retry)
{
	struct goodix_ts_cmd cmd_ack;
	struct goodix_ic_info_misc *misc = &cd->ic_info.misc;
	struct goodix_ts_hw_ops *hw_ops = cd->hw_ops;
	int i, ret;

	for (i = 0; i < retry; i++) {
		ret = hw_ops->read(
			cd, misc->cmd_addr, cmd_ack.buf, sizeof(cmd_ack));
		if (!ret && cmd_ack.state == target_status) {
			ts_debug("status check pass");
			return 0;
		}
		ts_debug("cmd buf %*ph", (int)sizeof(cmd_ack), cmd_ack.buf);
		msleep(20);
	}

	ts_err("cmd status not ready, retry %d, ack 0x%x, status 0x%x, ret %d",
		i, cmd_ack.ack, cmd_ack.state, ret);
	return -EINVAL;
}

static int send_cfg_cmd(
	struct goodix_ts_core *cd, struct goodix_ts_cmd *cfg_cmd)
{
	int ret;

	ret = cd->hw_ops->send_cmd(cd, cfg_cmd);
	if (ret) {
		ts_err("failed write cfg prepare cmd %d", ret);
		return ret;
	}
	ret = wait_cmd_status(
		cd, CONFIG_CMD_STATUS_PASS, CONFIG_CMD_WAIT_RETRY);
	if (ret) {
		ts_err("failed wait for fw ready for config, %d", ret);
		return ret;
	}
	return 0;
}

static int brl_send_config(struct goodix_ts_core *cd, u8 *cfg, int len)
{
	int ret;
	u8 *tmp_buf;
	struct goodix_ts_cmd cfg_cmd;
	struct goodix_ic_info_misc *misc = &cd->ic_info.misc;
	struct goodix_ts_hw_ops *hw_ops = cd->hw_ops;

	if (len > misc->fw_buffer_max_len) {
		ts_err("config len exceed limit %d > %d", len,
			misc->fw_buffer_max_len);
		return -EINVAL;
	}

	tmp_buf = kzalloc(len, GFP_KERNEL);
	if (!tmp_buf)
		return -ENOMEM;

	cfg_cmd.len = CONFIG_CND_LEN;
	cfg_cmd.cmd = CONFIG_CMD_START;
	ret = send_cfg_cmd(cd, &cfg_cmd);
	if (ret) {
		ts_err("failed write cfg prepare cmd %d", ret);
		goto exit;
	}

	ts_debug("try send config to 0x%x, len %d", misc->fw_buffer_addr, len);
	ret = hw_ops->write(cd, misc->fw_buffer_addr, cfg, len);
	if (ret) {
		ts_err("failed write config data, %d", ret);
		goto exit;
	}
	ret = hw_ops->read(cd, misc->fw_buffer_addr, tmp_buf, len);
	if (ret) {
		ts_err("failed read back config data");
		goto exit;
	}

	if (memcmp(cfg, tmp_buf, len)) {
		ts_err("config data read back compare file");
		ret = -EINVAL;
		goto exit;
	}
	/* notify fw for receiving config */
	memset(cfg_cmd.buf, 0, sizeof(cfg_cmd));
	cfg_cmd.len = CONFIG_CND_LEN;
	cfg_cmd.cmd = CONFIG_CMD_WRITE;
	ret = send_cfg_cmd(cd, &cfg_cmd);
	if (ret)
		ts_err("failed send config data ready cmd %d", ret);

exit:
	memset(cfg_cmd.buf, 0, sizeof(cfg_cmd));
	cfg_cmd.len = CONFIG_CND_LEN;
	cfg_cmd.cmd = CONFIG_CMD_EXIT;
	if (send_cfg_cmd(cd, &cfg_cmd)) {
		ts_err("failed send config write end command");
		ret = -EINVAL;
	}

	if (!ret) {
		ts_info("success send config");
		msleep(100);
	}

	kfree(tmp_buf);
	return ret;
}

/*
 * return: return config length on success, other wise return < 0
 **/
static int brl_read_config(struct goodix_ts_core *cd, u8 *cfg, int size)
{
	int ret;
	struct goodix_ts_cmd cfg_cmd;
	struct goodix_ic_info_misc *misc = &cd->ic_info.misc;
	struct goodix_ts_hw_ops *hw_ops = cd->hw_ops;
	struct goodix_config_head cfg_head;

	if (!cfg)
		return -EINVAL;

	cfg_cmd.len = CONFIG_CND_LEN;
	cfg_cmd.cmd = CONFIG_CMD_READ_START;
	ret = send_cfg_cmd(cd, &cfg_cmd);
	if (ret) {
		ts_err("failed send config read prepare command");
		return ret;
	}

	ret = hw_ops->read(
		cd, misc->fw_buffer_addr, cfg_head.buf, sizeof(cfg_head));
	if (ret) {
		ts_err("failed read config head %d", ret);
		goto exit;
	}

	if (checksum_cmp(cfg_head.buf, sizeof(cfg_head), CHECKSUM_MODE_U8_LE)) {
		ts_err("config head checksum error");
		ret = -EINVAL;
		goto exit;
	}

	cfg_head.cfg_len = le16_to_cpu(cfg_head.cfg_len);
	if (cfg_head.cfg_len > misc->fw_buffer_max_len ||
		cfg_head.cfg_len > size) {
		ts_err("cfg len exceed buffer size %d > %d", cfg_head.cfg_len,
			misc->fw_buffer_max_len);
		ret = -EINVAL;
		goto exit;
	}

	memcpy(cfg, cfg_head.buf, sizeof(cfg_head));
	ret = hw_ops->read(cd, misc->fw_buffer_addr + sizeof(cfg_head),
		cfg + sizeof(cfg_head), cfg_head.cfg_len);
	if (ret) {
		ts_err("failed read cfg pack, %d", ret);
		goto exit;
	}

	ts_info("config len %d", cfg_head.cfg_len);
	if (checksum_cmp(cfg + sizeof(cfg_head), cfg_head.cfg_len,
		    CHECKSUM_MODE_U16_LE)) {
		ts_err("config body checksum error");
		ret = -EINVAL;
		goto exit;
	}
	ts_info("success read config data: len %zu",
		cfg_head.cfg_len + sizeof(cfg_head));
exit:
	memset(cfg_cmd.buf, 0, sizeof(cfg_cmd));
	cfg_cmd.len = CONFIG_CND_LEN;
	cfg_cmd.cmd = CONFIG_CMD_READ_EXIT;
	if (send_cfg_cmd(cd, &cfg_cmd)) {
		ts_err("failed send config read finish command");
		ret = -EINVAL;
	}
	if (ret)
		return -EINVAL;
	return cfg_head.cfg_len + sizeof(cfg_head);
}

/*
 *	return: 0 for no error.
 *	GOODIX_EBUS when encounter a bus error
 *	GOODIX_ECHECKSUM version checksum error
 *	GOODIX_EVERSION  patch ID compare failed,
 *	in this case the sensorID is valid.
 */
static int brl_read_version(
	struct goodix_ts_core *cd, struct goodix_fw_version *version)
{
	int ret, i;
	u32 fw_addr;
	struct goodix_ts_hw_ops *hw_ops = cd->hw_ops;
	u8 buf[sizeof(struct goodix_fw_version)] = { 0 };
	u8 temp_pid[8] = { 0 };

	if (cd->bus->ic_type == IC_TYPE_BERLIN_A)
		fw_addr = FW_VERSION_INFO_ADDR_BRA;
	else
		fw_addr = FW_VERSION_INFO_ADDR;

	for (i = 0; i < GOODIX_RETRY_3; i++) {
		ret = hw_ops->read(cd, fw_addr, buf, sizeof(buf));
		if (ret) {
			ts_info("read fw version: %d, retry %d", ret, i);
			ret = -GOODIX_EBUS;
			usleep_range(5000, 5100);
			continue;
		}

		if (!checksum_cmp(buf, sizeof(buf), CHECKSUM_MODE_U8_LE))
			break;

		ts_info("invalid fw version: checksum error!");
		ts_info("fw version:%*ph", (int)sizeof(buf), buf);
		ret = -GOODIX_ECHECKSUM;
		usleep_range(10000, 11000);
	}
	if (ret) {
		ts_err("failed get valied fw version");
		return ret;
	}
	memcpy(version, buf, sizeof(*version));
	memcpy(temp_pid, version->rom_pid, sizeof(version->rom_pid));
	ts_info("rom_pid:%s", temp_pid);
	ts_info("rom_vid:%*ph", (int)sizeof(version->rom_vid),
		version->rom_vid);
	ts_info("pid:%s", version->patch_pid);
	ts_info("vid:%*ph", (int)sizeof(version->patch_vid),
		version->patch_vid);
	ts_info("sensor_id:%d", version->sensor_id);

	return 0;
}

#define LE16_TO_CPU(x) (x = le16_to_cpu(x))
#define LE32_TO_CPU(x) (x = le32_to_cpu(x))
static int convert_ic_info(struct goodix_ic_info *info, const u8 *data)
{
	int i;
	struct goodix_ic_info_version *version = &info->version;
	struct goodix_ic_info_feature *feature = &info->feature;
	struct goodix_ic_info_param *parm = &info->parm;
	struct goodix_ic_info_misc *misc = &info->misc;
	struct goodix_ic_info_other *other = &info->other;

	info->length = le16_to_cpup((__le16 *)data);

	data += 2;
	memcpy(version, data, sizeof(*version));
	version->config_id = le32_to_cpu(version->config_id);

	data += sizeof(struct goodix_ic_info_version);
	memcpy(feature, data, sizeof(*feature));
	feature->freqhop_feature = le16_to_cpu(feature->freqhop_feature);
	feature->calibration_feature =
		le16_to_cpu(feature->calibration_feature);
	feature->gesture_feature = le16_to_cpu(feature->gesture_feature);
	feature->side_touch_feature = le16_to_cpu(feature->side_touch_feature);
	feature->stylus_feature = le16_to_cpu(feature->stylus_feature);

	data += sizeof(struct goodix_ic_info_feature);
	parm->drv_num = *(data++);
	parm->sen_num = *(data++);
	parm->button_num = *(data++);
	parm->force_num = *(data++);
	parm->active_scan_rate_num = *(data++);
	if (parm->active_scan_rate_num > MAX_SCAN_RATE_NUM) {
		ts_err("invalid scan rate num %d > %d",
			parm->active_scan_rate_num, MAX_SCAN_RATE_NUM);
		return -EINVAL;
	}
	for (i = 0; i < parm->active_scan_rate_num; i++)
		parm->active_scan_rate[i] =
			le16_to_cpup((__le16 *)(data + i * 2));

	data += parm->active_scan_rate_num * 2;
	parm->mutual_freq_num = *(data++);
	if (parm->mutual_freq_num > MAX_SCAN_FREQ_NUM) {
		ts_err("invalid mntual freq num %d > %d", parm->mutual_freq_num,
			MAX_SCAN_FREQ_NUM);
		return -EINVAL;
	}
	for (i = 0; i < parm->mutual_freq_num; i++)
		parm->mutual_freq[i] = le16_to_cpup((__le16 *)(data + i * 2));

	data += parm->mutual_freq_num * 2;
	parm->self_tx_freq_num = *(data++);
	if (parm->self_tx_freq_num > MAX_SCAN_FREQ_NUM) {
		ts_err("invalid tx freq num %d > %d", parm->self_tx_freq_num,
			MAX_SCAN_FREQ_NUM);
		return -EINVAL;
	}
	for (i = 0; i < parm->self_tx_freq_num; i++)
		parm->self_tx_freq[i] = le16_to_cpup((__le16 *)(data + i * 2));

	data += parm->self_tx_freq_num * 2;
	parm->self_rx_freq_num = *(data++);
	if (parm->self_rx_freq_num > MAX_SCAN_FREQ_NUM) {
		ts_err("invalid rx freq num %d > %d", parm->self_rx_freq_num,
			MAX_SCAN_FREQ_NUM);
		return -EINVAL;
	}
	for (i = 0; i < parm->self_rx_freq_num; i++)
		parm->self_rx_freq[i] = le16_to_cpup((__le16 *)(data + i * 2));

	data += parm->self_rx_freq_num * 2;
	parm->stylus_freq_num = *(data++);
	if (parm->stylus_freq_num > MAX_FREQ_NUM_STYLUS) {
		ts_err("invalid stylus freq num %d > %d", parm->stylus_freq_num,
			MAX_FREQ_NUM_STYLUS);
		return -EINVAL;
	}
	for (i = 0; i < parm->stylus_freq_num; i++)
		parm->stylus_freq[i] = le16_to_cpup((__le16 *)(data + i * 2));

	data += parm->stylus_freq_num * 2;
	memcpy(misc, data, sizeof(*misc));
	misc->cmd_addr = le32_to_cpu(misc->cmd_addr);
	misc->cmd_max_len = le16_to_cpu(misc->cmd_max_len);
	misc->cmd_reply_addr = le32_to_cpu(misc->cmd_reply_addr);
	misc->cmd_reply_len = le16_to_cpu(misc->cmd_reply_len);
	misc->fw_state_addr = le32_to_cpu(misc->fw_state_addr);
	misc->fw_state_len = le16_to_cpu(misc->fw_state_len);
	misc->fw_buffer_addr = le32_to_cpu(misc->fw_buffer_addr);
	misc->fw_buffer_max_len = le16_to_cpu(misc->fw_buffer_max_len);
	misc->frame_data_addr = le32_to_cpu(misc->frame_data_addr);
	misc->frame_data_head_len = le16_to_cpu(misc->frame_data_head_len);

	misc->fw_attr_len = le16_to_cpu(misc->fw_attr_len);
	misc->fw_log_len = le16_to_cpu(misc->fw_log_len);
	misc->stylus_struct_len = le16_to_cpu(misc->stylus_struct_len);
	misc->mutual_struct_len = le16_to_cpu(misc->mutual_struct_len);
	misc->self_struct_len = le16_to_cpu(misc->self_struct_len);
	misc->noise_struct_len = le16_to_cpu(misc->noise_struct_len);
	misc->touch_data_addr = le32_to_cpu(misc->touch_data_addr);
	misc->touch_data_head_len = le16_to_cpu(misc->touch_data_head_len);
	misc->point_struct_len = le16_to_cpu(misc->point_struct_len);
	LE32_TO_CPU(misc->mutual_rawdata_addr);
	LE32_TO_CPU(misc->mutual_diffdata_addr);
	LE32_TO_CPU(misc->mutual_refdata_addr);
	LE32_TO_CPU(misc->self_rawdata_addr);
	LE32_TO_CPU(misc->self_diffdata_addr);
	LE32_TO_CPU(misc->self_refdata_addr);
	LE32_TO_CPU(misc->iq_rawdata_addr);
	LE32_TO_CPU(misc->iq_refdata_addr);
	LE32_TO_CPU(misc->im_rawdata_addr);
	LE16_TO_CPU(misc->im_readata_len);
	LE32_TO_CPU(misc->noise_rawdata_addr);
	LE16_TO_CPU(misc->noise_rawdata_len);
	LE32_TO_CPU(misc->stylus_rawdata_addr);
	LE16_TO_CPU(misc->stylus_rawdata_len);
	LE32_TO_CPU(misc->noise_data_addr);
	LE32_TO_CPU(misc->esd_addr);
	LE32_TO_CPU(misc->auto_scan_cmd_addr);
	LE32_TO_CPU(misc->auto_scan_info_addr);

	data += sizeof(*misc);
	memcpy((u8 *)other, data, sizeof(*other));

	return 0;
}

void print_ic_info(struct goodix_ic_info *ic_info)
{
	struct goodix_ic_info_version *version = &ic_info->version;
	struct goodix_ic_info_feature *feature = &ic_info->feature;
	struct goodix_ic_info_param *parm = &ic_info->parm;
	struct goodix_ic_info_misc *misc = &ic_info->misc;
	struct goodix_ic_info_other *other = &ic_info->other;

	ts_info("ic_info_length:                %d", ic_info->length);
	ts_info("info_customer_id:              0x%01X",
		version->info_customer_id);
	ts_info("info_version_id:               0x%01X",
		version->info_version_id);
	ts_info("ic_die_id:                     0x%01X", version->ic_die_id);
	ts_info("ic_version_id:                 0x%01X",
		version->ic_version_id);
	ts_info("config_id:                     0x%4X", version->config_id);
	ts_info("config_version:                0x%01X",
		version->config_version);
	ts_info("frame_data_customer_id:        0x%01X",
		version->frame_data_customer_id);
	ts_info("frame_data_version_id:         0x%01X",
		version->frame_data_version_id);
	ts_info("touch_data_customer_id:        0x%01X",
		version->touch_data_customer_id);
	ts_info("touch_data_version_id:         0x%01X",
		version->touch_data_version_id);

	ts_info("freqhop_feature:               0x%04X",
		feature->freqhop_feature);
	ts_info("calibration_feature:           0x%04X",
		feature->calibration_feature);
	ts_info("gesture_feature:               0x%04X",
		feature->gesture_feature);
	ts_info("side_touch_feature:            0x%04X",
		feature->side_touch_feature);
	ts_info("stylus_feature:                0x%04X",
		feature->stylus_feature);

	ts_info("Drv*Sen,Button,Force num:      %d x %d, %d, %d", parm->drv_num,
		parm->sen_num, parm->button_num, parm->force_num);

	ts_info("Cmd:                           0x%04X, %d", misc->cmd_addr,
		misc->cmd_max_len);
	ts_info("Cmd-Reply:                     0x%04X, %d",
		misc->cmd_reply_addr, misc->cmd_reply_len);
	ts_info("FW-State:                      0x%04X, %d",
		misc->fw_state_addr, misc->fw_state_len);
	ts_info("FW-Buffer:                     0x%04X, %d",
		misc->fw_buffer_addr, misc->fw_buffer_max_len);
	ts_info("Touch-Data:                    0x%04X, %d",
		misc->touch_data_addr, misc->touch_data_head_len);
	ts_info("frame_data_addr:               0x%04X", misc->frame_data_addr);
	ts_info("point_struct_len:              %d", misc->point_struct_len);
	ts_info("mutual_rawdata_addr:           0x%04X",
		misc->mutual_rawdata_addr);
	ts_info("mutual_diffdata_addr:          0x%04X",
		misc->mutual_diffdata_addr);
	ts_info("self_rawdata_addr:             0x%04X",
		misc->self_rawdata_addr);
	ts_info("self_diffdata_addr:            0x%04X",
		misc->self_diffdata_addr);
	ts_info("stylus_rawdata_addr:           0x%04X, %d",
		misc->stylus_rawdata_addr, misc->stylus_rawdata_len);
	ts_info("esd_addr:                      0x%04X", misc->esd_addr);
	ts_info("frame_data_addr:               0x%04X", misc->frame_data_addr);
	ts_info("screen_max_x:                  %u", other->screen_max_x);
	ts_info("screen_max_y:                  %u", other->screen_max_y);
}

static int brl_get_ic_info(
	struct goodix_ts_core *cd, struct goodix_ic_info *ic_info)
{
	int ret, i;
	u16 length = 0;
	u32 ic_addr;
	u8 afe_data[GOODIX_IC_INFO_MAX_LEN] = { 0 };
	struct goodix_ts_hw_ops *hw_ops = cd->hw_ops;

	if (cd->bus->ic_type == IC_TYPE_BERLIN_A)
		ic_addr = GOODIX_IC_INFO_ADDR_BRA;
	else
		ic_addr = GOODIX_IC_INFO_ADDR;

	for (i = 0; i < GOODIX_RETRY_3; i++) {
		ret = hw_ops->read(cd, ic_addr, (u8 *)&length, sizeof(length));
		if (ret) {
			ts_info("failed get ic info length, %d", ret);
			usleep_range(5000, 5100);
			continue;
		}
		length = le16_to_cpu(length);
		if (length >= GOODIX_IC_INFO_MAX_LEN) {
			ts_info("invalid ic info length %d, retry %d", length,
				i);
			continue;
		}

		ret = hw_ops->read(cd, ic_addr, afe_data, length);
		if (ret) {
			ts_info("failed get ic info data, %d", ret);
			usleep_range(5000, 5100);
			continue;
		}
		/* judge whether the data is valid */
		if (is_risk_data((const uint8_t *)afe_data, length)) {
			ts_info("fw info data invalid");
			usleep_range(5000, 5100);
			continue;
		}
		if (checksum_cmp((const uint8_t *)afe_data, length,
			    CHECKSUM_MODE_U8_LE)) {
			ts_info("fw info checksum error!");
			usleep_range(5000, 5100);
			continue;
		}
		break;
	}
	if (i == GOODIX_RETRY_3) {
		ts_err("failed get ic info");
		return -EINVAL;
	}

	ret = convert_ic_info(ic_info, afe_data);
	if (ret) {
		ts_err("convert ic info encounter error");
		return ret;
	}

	/* check some key info */
	if (!ic_info->misc.cmd_addr || !ic_info->misc.fw_buffer_addr ||
		!ic_info->misc.touch_data_addr) {
		ts_err("cmd_addr fw_buf_addr and touch_data_addr is null");
		return -EINVAL;
	}

	return 0;
}

#define GOODIX_ESD_TICK_WRITE_DATA 0xAA
static int brl_esd_check(struct goodix_ts_core *cd)
{
	int ret;
	u32 esd_addr;
	u8 esd_value;

	if (!cd->ic_info.misc.esd_addr)
		return 0;

	esd_addr = cd->ic_info.misc.esd_addr;
	ret = cd->hw_ops->read(cd, esd_addr, &esd_value, 1);
	if (ret) {
		ts_err("failed get esd value, %d", ret);
		return ret;
	}

	if (esd_value != 0xFF) {
		ts_err("esd check failed, 0x%x", esd_value);
		return -EINVAL;
	}
	esd_value = GOODIX_ESD_TICK_WRITE_DATA;
	ret = cd->hw_ops->write(cd, esd_addr, &esd_value, 1);
	if (ret) {
		ts_err("failed refrash esd value");
		return ret;
	}
	return 0;
}

#define IRQ_EVENT_HEAD_LEN 8
#define BYTES_PER_POINT 14
#define BYTES_STYLUS_LEN 16
#define COOR_DATA_CHECKSUM_SIZE 2

#define GOODIX_TOUCH_EVENT 0x80
#define GOODIX_REQUEST_EVENT 0x40
#define GOODIX_GESTURE_EVENT 0x20
#define GOODIX_STATUS_EVENT 0x02
#define GOODIX_FP_EVENT 0x08
#define POINT_TYPE_STYLUS_HOVER 0x01
#define POINT_TYPE_STYLUS 0x03
static void goodix_parse_finger(struct goodix_ts_core *cd,
		struct goodix_touch_data *touch_data,
		u8 *buf, int id)
{
	int point_struct_len = cd->ic_info.misc.point_struct_len;
	struct goodix_ts_coords *coord = &touch_data->coords[id]; /* [GOOG] */

	coord->status = TS_TOUCH;
	coord->x = le16_to_cpup((__le16 *)(buf + 2));
	coord->y = le16_to_cpup((__le16 *)(buf + 4));
	coord->w = le16_to_cpup((__le16 *)(buf + 6));
	if (point_struct_len > 8) {
		coord->p = buf[8];
		coord->major = le16_to_cpup((__le16 *)(buf + 9));
		coord->minor = le16_to_cpup((__le16 *)(buf + 11));
		coord->angle = (signed char)buf[13];
	}
	touch_data->touch_num += 1;
}

static unsigned int goodix_pen_btn_code[] = { BTN_STYLUS, BTN_STYLUS2 };
static void goodix_parse_pen(
	struct goodix_pen_data *pen_data, u8 *event_head, u8 *buf)
{
	u8 cur_key_map = 0;
	int16_t x_angle, y_angle;
	int i;

	if ((buf[0] & 0x0F) == POINT_TYPE_STYLUS_HOVER)
		pen_data->is_hover = true;

	pen_data->coords.status = TS_TOUCH;
	pen_data->coords.x = le16_to_cpup((__le16 *)(buf + 2));
	pen_data->coords.y = le16_to_cpup((__le16 *)(buf + 4));
	pen_data->coords.p = le16_to_cpup((__le16 *)(buf + 6));
	x_angle = le16_to_cpup((__le16 *)(buf + 8));
	y_angle = le16_to_cpup((__le16 *)(buf + 10));
	pen_data->coords.tilt_x = x_angle / 100;
	pen_data->coords.tilt_y = y_angle / 100;

	/* ble pen data */
	pen_data->custom_flag = event_head[3] & 0x01;
	if (pen_data->custom_flag) {
		pen_data->tx1_freq_index = buf[19];
		pen_data->tx2_freq_index = buf[20];
	}

	cur_key_map = (buf[3] & 0x0F) >> 1;
	for (i = 0; i < GOODIX_MAX_PEN_KEY; i++) {
		pen_data->keys[i].code = goodix_pen_btn_code[i];
		if (!(cur_key_map & (1 << i)))
			continue;
		pen_data->keys[i].status = TS_TOUCH;
	}
}

static int goodix_update_heatmap(struct goodix_ts_core *cd, u8 *event_data)
{
	struct goodix_ts_hw_ops *hw_ops = cd->hw_ops;
	struct goodix_ic_info_misc *misc = &cd->ic_info.misc;
	int tx = cd->ic_info.parm.drv_num;
	int rx = cd->ic_info.parm.sen_num;
	int mutual_struct_len = misc->mutual_struct_len;
	struct goodix_mutual_data *mutual_data;
	struct goodix_self_sensing_data *self_sensing_data;
	int ret = 0;

	/*
	 * Memory map of event data
	 * ===========================================
	 *        touch_data_addr -> touch_data
	 *                           ...
	 *        frame_data_addr -> frame_data_head
	 *                           fw_attr
	 *                           fw_log
	 *       mutual_data_addr -> mutual_data
	 * self_sensing_data_addr -> self_sensing_data
	 * ===========================================
	 */
	uint8_t *mutual_head = event_data + misc->frame_data_addr -
			       misc->touch_data_addr +
			       misc->frame_data_head_len + misc->fw_attr_len +
			       misc->fw_log_len;

	mutual_data = (struct goodix_mutual_data *)mutual_head;
	self_sensing_data =
		(struct goodix_self_sensing_data *)(mutual_head + mutual_struct_len);

	/*
	 * If touch IC can't support frame data mode, the driver need to send
	 * additional commands to read mutual-sensing and self-sensing data.
	 */
	if (cd->bus->sub_ic_type == IC_TYPE_SUB_GT7986) {
		ret = hw_ops->read(cd, misc->mutual_diffdata_addr,
			(u8 *)mutual_data, tx * rx * 2);
		if (ret) {
			ts_debug("failed to read mutual data");
			goto exit;
		}

		ret = hw_ops->read(cd, misc->self_diffdata_addr,
			(u8 *)self_sensing_data, (tx + rx) * 2);
		if (ret) {
			ts_debug("failed to read self data");
			goto exit;
		}
	}

	goodix_rotate_abcd2cbad(tx, rx, mutual_data->data, cd->mutual_data);
	memcpy(cd->self_sensing_data, self_sensing_data->data, (tx + rx) * 2);

exit:
	return ret;
}

static int goodix_touch_handler(struct goodix_ts_core *cd,
	struct goodix_ts_event *ts_event,
	struct goodix_ts_touch_event_data *event_data)
{
	struct goodix_ic_info_misc *misc = &cd->ic_info.misc;
	struct goodix_touch_data *touch_data = &ts_event->touch_data;
	struct goodix_pen_data *pen_data = &ts_event->pen_data;
	u8 *data = event_data->data;
	u8 touch_num;
	int checksum_len = 0;
	int ret = 0;
	u8 point_type;
	int tid;
	int i;

	touch_data->touch_num = 0; /* [GOOG] */
	touch_num = event_data->touches;
	if (touch_num > 0) {
		/*
		 * [GOOG]
		 * Touch Packet: 8 bytes header + data(stylus or finger).
		 */
		for (i = 0; i < touch_num; i++) {
			point_type = event_data->type;
			tid = (data[0] >> 4) & 0x0F;
			if (point_type == POINT_TYPE_STYLUS ||
					point_type == POINT_TYPE_STYLUS_HOVER) {
				goodix_parse_pen(pen_data, (u8 *)event_data, data);
				checksum_len += BYTES_STYLUS_LEN;
				data += BYTES_STYLUS_LEN;
			} else {
				/* [GOOG] */
				if (tid >= GOODIX_MAX_TOUCH)
					ts_info("invalid touch#%d id %d", i, tid);
				else
					goodix_parse_finger(cd, touch_data, data, tid);
				checksum_len += misc->point_struct_len;
				data += misc->point_struct_len;
			}
		}
		ret = checksum_cmp(event_data->data,
					checksum_len + 2,
					CHECKSUM_MODE_U8_LE);
		if (ret) {
			ts_debug("touch data checksum error");
			return -EINVAL;
		}
	}

	goodix_update_heatmap(cd, (u8 *)event_data); /* [GOOG] */

	ts_event->fp_flag = event_data->fp_flag;
	ts_event->event_type |= EVENT_TOUCH;
	if (cd->board_data.pen_enable)
		ts_event->event_type |= EVENT_PEN;

	return 0;
}

static int brl_event_handler(struct goodix_ts_core *cd,
			 struct goodix_ts_event *ts_event)
{
	struct goodix_ts_hw_ops *hw_ops = cd->hw_ops;
	struct goodix_ic_info_misc *misc = &cd->ic_info.misc;
/* [GOOG] */
	struct goodix_ts_event_data *event_data;
	int ret;

	/* clean event buffer */
	memset(ts_event, 0, sizeof(*ts_event));

	if (!cd->touch_frame_package || !cd->touch_frame_size)
		return -ENOMEM;

	ret = hw_ops->read_fast(cd, misc->touch_data_addr,
			cd->touch_frame_package, cd->touch_frame_size);
	if (ret) {
		ts_debug("failed get event head data");
		return ret;
	}
	event_data = (struct goodix_ts_event_data *)cd->touch_frame_package->data;

	if (event_data->type == 0x00) {
		ts_debug("invalid touch head");
		return -EINVAL;
	}

	if (checksum_cmp((u8 *)event_data, IRQ_EVENT_HEAD_LEN,
		CHECKSUM_MODE_U8_LE)) {
		ts_debug("touch head checksum err[%*ph]", IRQ_EVENT_HEAD_LEN,
			event_data);
		return -EINVAL;
	}
/*~[GOOG]*/

	ts_event->event_type = EVENT_INVALID;
	ts_event->clear_count1 = event_data->clear_count1;
	ts_event->clear_count2 = event_data->clear_count2;
	/* read status event */
	if (event_data->status_changed) {
		ts_event->event_type |= EVENT_STATUS;
	}

/*
 * [GOOG]
 * Use goodix_update_heatmap() to do the heatmap process and
 * move after_event_handler() into goodix_ts_core.c.
 *
    // read done
	hw_ops->after_event_handler(cd);

	if (cd->heatmap_buffer && cd->bus->sub_ic_type != IC_TYPE_SUB_GT7986) {
		offset = 248 + misc->frame_data_head_len +
		misc->fw_attr_len + misc->fw_log_len + 8;
		memcpy(cd->heatmap_diff, &pre_buf[offset], tx * rx * 2);
		goodix_rotate_abcd2cbad(tx, rx, cd->heatmap_diff);
		offset = 248 + misc->frame_data_head_len +
		misc->fw_attr_len + misc->fw_log_len +
		misc->mutual_struct_len + 10;
		memcpy(cd->heatmap_selfdiff, &pre_buf[offset], (tx + rx) * 2);
	}
 *~[GOOG]
 */

	if (event_data->type & (GOODIX_TOUCH_EVENT >> 4))
		goodix_touch_handler(cd, ts_event,
			(struct goodix_ts_touch_event_data *)event_data);

	if (event_data->type & (GOODIX_REQUEST_EVENT >> 4)) {
		struct goodix_ts_request_event_data *request =
			(struct goodix_ts_request_event_data *)event_data;
		ts_event->event_type |= EVENT_REQUEST;
		if (request->request_type == BRL_REQUEST_CODE_CONFIG)
			ts_event->request_code = REQUEST_TYPE_CONFIG;
		else if (request->request_type == BRL_REQUEST_CODE_RESET)
			ts_event->request_code = REQUEST_TYPE_RESET;
		else if (request->request_type == BRL_REQUEST_CODE_UPDATE)
			ts_event->request_code = REQUEST_TYPE_UPDATE;
		else
			ts_debug("unsupported request code 0x%x",
				request->request_type);
	}

	if (event_data->type & (GOODIX_GESTURE_EVENT >> 4)) {
		struct goodix_ts_gesture_event_data *gesture =
			(struct goodix_ts_gesture_event_data *)event_data;
		ts_event->event_type |= EVENT_GESTURE;
		ts_event->temp_gesture_data.event_type = gesture->event_type;
		ts_event->temp_gesture_data.touches = gesture->touches;
		memcpy(ts_event->temp_gesture_data.data, gesture->data,
			GOODIX_GESTURE_DATA_LEN);
	}

	return 0;
}

static int brl_after_event_handler(struct goodix_ts_core *cd)
{
	struct goodix_ts_hw_ops *hw_ops = cd->hw_ops;
	struct goodix_ic_info_misc *misc = &cd->ic_info.misc;
	u8 sync_clean[1] = { 0 };

	if (cd->tools_ctrl_sync)
		return 0;

	return hw_ops->write(cd, misc->touch_data_addr, sync_clean, sizeof(sync_clean));
}

static int brld_get_framedata(
	struct goodix_ts_core *cd, struct ts_rawdata_info *info)
{
	int ret;
	unsigned char val;
	int retry = 20;
	struct frame_head *frame_head;
	u8 *frame_buf;
	unsigned char *cur_ptr;
	unsigned int flag_addr = cd->ic_info.misc.frame_data_addr;

	frame_buf = kzalloc(GOODIX_MAX_FRAMEDATA_LEN, GFP_KERNEL);
	if (frame_buf == NULL)
		return -ENOMEM;

	/* clean touch event flag */
	val = 0;
	ret = brl_write(cd, flag_addr, &val, 1);
	if (ret < 0) {
		ts_err("clean touch event failed, exit!");
		goto exit;
	}

	while (retry--) {
		usleep_range(2000, 2100);
		ret = brl_read(cd, flag_addr, &val, 1);
		if (!ret && (val & GOODIX_TOUCH_EVENT))
			break;
	}
	if (retry < 0) {
		ts_err("framedata is not ready val:0x%02x, exit!", val);
		ret = -EINVAL;
		goto exit;
	}

	ret = brl_read(cd, flag_addr, frame_buf, GOODIX_MAX_FRAMEDATA_LEN);
	if (ret < 0) {
		ts_err("read frame data failed");
		goto exit;
	}

	if (checksum_cmp(frame_buf, cd->ic_info.misc.frame_data_head_len,
		    CHECKSUM_MODE_U8_LE)) {
		ts_err("frame head checksum error");
		ret = -EINVAL;
		goto exit;
	}

	frame_head = (struct frame_head *)frame_buf;
	if (checksum_cmp(frame_buf, frame_head->cur_frame_len,
		    CHECKSUM_MODE_U16_LE)) {
		ts_err("frame body checksum error");
		ret = -EINVAL;
		goto exit;
	}
	cur_ptr = frame_buf;
	cur_ptr += cd->ic_info.misc.frame_data_head_len;
	cur_ptr += cd->ic_info.misc.fw_attr_len;
	cur_ptr += cd->ic_info.misc.fw_log_len;
	memcpy((u8 *)(info->buff + info->used_size), cur_ptr + 8,
		cd->ic_info.misc.mutual_struct_len - 8);

exit:
	kfree(frame_buf);
	return ret;
}

static int brld_get_cap_data(
	struct goodix_ts_core *cd, struct ts_rawdata_info *info)
{
	struct goodix_ts_cmd temp_cmd;
	int tx = cd->ic_info.parm.drv_num;
	int rx = cd->ic_info.parm.sen_num;
	int size = tx * rx;
	int ret;

	/* disable irq & close esd */
	brl_irq_enable(cd, false);
	goodix_ts_esd_off(cd);

	info->buff[0] = rx;
	info->buff[1] = tx;
	info->used_size = 2;

	temp_cmd.cmd = 0x90;
	temp_cmd.data[0] = 0x81;
	temp_cmd.len = 5;
	ret = brl_send_cmd(cd, &temp_cmd);
	if (ret < 0) {
		ts_err("report rawdata failed, exit!");
		goto exit;
	}

	ret = brld_get_framedata(cd, info);
	if (ret < 0) {
		ts_err("brld get rawdata failed");
		goto exit;
	}
	goodix_rotate_abcd2cbad(tx, rx, &info->buff[info->used_size], NULL);
	info->used_size += size;

	temp_cmd.cmd = 0x90;
	temp_cmd.data[0] = 0x82;
	temp_cmd.len = 5;
	ret = brl_send_cmd(cd, &temp_cmd);
	if (ret < 0) {
		ts_err("report diffdata failed, exit!");
		goto exit;
	}

	ret = brld_get_framedata(cd, info);
	if (ret < 0) {
		ts_err("brld get diffdata failed");
		goto exit;
	}
	goodix_rotate_abcd2cbad(tx, rx, &info->buff[info->used_size], NULL);
	info->used_size += size;

exit:
	temp_cmd.cmd = 0x90;
	temp_cmd.data[0] = 0;
	temp_cmd.len = 5;
	brl_send_cmd(cd, &temp_cmd);
	/* enable irq & esd */
	brl_irq_enable(cd, true);
	goodix_ts_esd_on(cd);
	return ret;
}

#define GOODIX_CMD_RAWDATA 2
#define GOODIX_CMD_COORD 0
static int brl_get_capacitance_data(
	struct goodix_ts_core *cd, struct ts_rawdata_info *info)
{
	int ret;
	int retry = 20;
	struct goodix_ts_cmd temp_cmd;
	u32 flag_addr = cd->ic_info.misc.touch_data_addr;
	u32 raw_addr = cd->ic_info.misc.mutual_rawdata_addr;
	u32 diff_addr = cd->ic_info.misc.mutual_diffdata_addr;
	int tx = cd->ic_info.parm.drv_num;
	int rx = cd->ic_info.parm.sen_num;
	int size = tx * rx;
	u8 val;

	if (!info) {
		ts_err("input null ptr");
		return -EIO;
	}

	if (cd->bus->ic_type == IC_TYPE_BERLIN_D ||
		cd->bus->ic_type == IC_TYPE_NOTTINGHAM)
		return brld_get_cap_data(cd, info);

	/* disable irq & close esd */
	brl_irq_enable(cd, false);
	goodix_ts_esd_off(cd);

	/* switch rawdata mode */
	temp_cmd.cmd = GOODIX_CMD_RAWDATA;
	temp_cmd.len = 4;
	ret = brl_send_cmd(cd, &temp_cmd);
	if (ret < 0) {
		ts_err("switch rawdata mode failed, exit!");
		goto exit;
	}

	/* clean touch event flag */
	val = 0;
	ret = brl_write(cd, flag_addr, &val, 1);
	if (ret < 0) {
		ts_err("clean touch event failed, exit!");
		goto exit;
	}

	while (retry--) {
		usleep_range(5000, 5100);
		ret = brl_read(cd, flag_addr, &val, 1);
		if (!ret && (val & GOODIX_TOUCH_EVENT))
			break;
	}
	if (retry < 0) {
		ts_err("rawdata is not ready val:0x%02x, exit!", val);
		goto exit;
	}

	/* obtain rawdata & diff_rawdata */
	info->buff[0] = rx;
	info->buff[1] = tx;
	info->used_size = 2;

	ret = brl_read(cd, raw_addr, (u8 *)&info->buff[info->used_size],
		size * sizeof(s16));
	if (ret < 0) {
		ts_err("obtian raw_data failed, exit!");
		goto exit;
	}
	goodix_rotate_abcd2cbad(tx, rx, &info->buff[info->used_size], NULL);
	info->used_size += size;

	ret = brl_read(cd, diff_addr, (u8 *)&info->buff[info->used_size],
		size * sizeof(s16));
	if (ret < 0) {
		ts_err("obtian diff_data failed, exit!");
		goto exit;
	}
	goodix_rotate_abcd2cbad(tx, rx, &info->buff[info->used_size], NULL);
	info->used_size += size;

exit:
	/* switch coor mode */
	temp_cmd.cmd = GOODIX_CMD_COORD;
	temp_cmd.len = 4;
	brl_send_cmd(cd, &temp_cmd);
	/* clean touch event flag */
	val = 0;
	brl_write(cd, flag_addr, &val, 1);
	/* enable irq & esd */
	brl_irq_enable(cd, true);
	goodix_ts_esd_on(cd);
	return ret;
}

#define GOODIX_GET_SCAN_MODE_ADDR 0x10219
static int brl_get_scan_mode(struct goodix_ts_core *cd, enum raw_scan_mode* mode)
{
	int ret = 0;

	ret = cd->hw_ops->read(cd, GOODIX_GET_SCAN_MODE_ADDR, mode, 1);
	if (ret != 0) {
		ts_err("failed to get scan mode, ret: %d", ret);
		return ret;
	}

	return 0;
}

#define GOODIX_CMD_SET_SCAN_MODE 0x9F
static int brl_set_scan_mode(struct goodix_ts_core *cd, enum raw_scan_mode mode)
{
	struct goodix_ts_cmd cmd;
	static const uint8_t raw_scan_mode_cmd_codes[] ={
		[RAW_SCAN_MODE_AUTO] = 0x00,
		[RAW_SCAN_MODE_NORMAL_ACTIVE] = 0x02,
		[RAW_SCAN_MODE_NORMAL_IDLE] = 0x03,
		[RAW_SCAN_MODE_LOW_POWER_ACTIVE] = 0x00,
		[RAW_SCAN_MODE_LOW_POWER_IDLE] = 0x00,
		[RAW_SCAN_MODE_SLEEP] = 0x00,
	};


	cmd.cmd = GOODIX_CMD_SET_SCAN_MODE;
	cmd.len = 5;
	cmd.data[0] = raw_scan_mode_cmd_codes[mode];
	if (cd->hw_ops->send_cmd(cd, &cmd))
		ts_err("failed set scan mode cmd");

	return 0;
}

#define GOODIX_CMD_SET_CONTINUOUSLY_REPORT_ENABLED 0xC6
static int brl_set_continuously_report_enabled(struct goodix_ts_core *cd, bool enabled)
{
	struct goodix_ts_cmd cmd;

	cmd.cmd = GOODIX_CMD_SET_CONTINUOUSLY_REPORT_ENABLED;
	cmd.len = 5;
	cmd.data[0] = enabled ? 0 : 1;
	if (cd->hw_ops->send_cmd(cd, &cmd))
		ts_err("failed set continuous mode cmd");

	return 0;
}

#define GOODIX_CMD_SET_FRAMEDATA_ENABLED 0x90
#define GOODIX_CMD_SET_HEATMAP_ENABLED 0xC9
static int brl_set_heatmap_enabled(struct goodix_ts_core *cd, bool enabled)
{
	struct goodix_ts_cmd cmd;
	int ret = 0;

	cmd.cmd = GOODIX_CMD_SET_HEATMAP_ENABLED;
	cmd.len = 5;
	cmd.data[0] = enabled ? 1 : 0;
	ret = cd->hw_ops->send_cmd(cd, &cmd);
	if (ret != 0) {
		ts_err("failed to set heatmap %s, err: %d",
			enabled ? "enabled" : "disabled", ret);
		return ret;
	}
	return ret;
}

#define GOODIX_FEATURE_STATUS_ADDR 0x1021A
#define GOODIX_CMD_SET_CUSTOM_MODE 0xC7
#define CUSTOM_MODE_PALM 0
#define CUSTOM_MODE_GRIP 3
#define CUSTOM_MODE_MASK_PALM 0x02
#define CUSTOM_MODE_MASK_GRIP 0x04
#define CUSTOM_MODE_MASK_SCREEN_PROTECTOR 0x40
#define CUSTOM_MODE_MASK_COORD_FILTER 0x80
static int brl_set_palm_enabled(struct goodix_ts_core *cd, bool enabled)
{
	struct goodix_ts_cmd cmd = { 0 };

	cmd.cmd = GOODIX_CMD_SET_CUSTOM_MODE;
	cmd.len = 6;
	cmd.data[0] = CUSTOM_MODE_PALM;
	cmd.data[1] = enabled ? 1 : 0;
	if (cd->hw_ops->send_cmd(cd, &cmd))
		ts_err("failed to %s palm mode",
			enabled ? "enable" : "disable");

	return 0;
}

static int brl_get_palm_enabled(struct goodix_ts_core *cd, bool *enabled)
{
	int ret = 0;
	u8 val;

	ret = cd->hw_ops->read(cd, GOODIX_FEATURE_STATUS_ADDR, &val, 1);
	if (ret != 0) {
		ts_err("failed to get palm enabled, ret: %d", ret);
		*enabled = false;
		return ret;
	}
	*enabled = (val & CUSTOM_MODE_MASK_PALM) != 0;
	return ret;
}

static int brl_set_grip_enabled(struct goodix_ts_core *cd, bool enabled)
{
	struct goodix_ts_cmd cmd = { 0 };

	cmd.cmd = GOODIX_CMD_SET_CUSTOM_MODE;
	cmd.len = 6;
	cmd.data[0] = CUSTOM_MODE_GRIP;
	cmd.data[1] = enabled ? 1 : 0;
	if (cd->hw_ops->send_cmd(cd, &cmd))
		ts_err("failed to %s grip mode",
			enabled ? "enable" : "disable");

	return 0;
}

static int brl_get_grip_enabled(struct goodix_ts_core *cd, bool *enabled)
{
	int ret = 0;
	u8 val;

	ret = cd->hw_ops->read(cd, GOODIX_FEATURE_STATUS_ADDR, &val, 1);
	if (ret != 0) {
		ts_err("failed to get grip enabled, ret: %d", ret);
		*enabled = false;
		return ret;
	}
	*enabled = (val & CUSTOM_MODE_MASK_GRIP) != 0;
	return ret;
}

#define GOODIX_CMD_SET_SCREEN_PROTECTOR_ENABLED 0x72
static int brl_set_screen_protector_mode_enabled(
	struct goodix_ts_core *cd, bool enabled)
{
	struct goodix_ts_cmd cmd = { 0 };

	cmd.cmd = GOODIX_CMD_SET_SCREEN_PROTECTOR_ENABLED;
	cmd.len = 5;
	cmd.data[0] = enabled ? 1 : 0;
	if (cd->hw_ops->send_cmd(cd, &cmd))
		ts_err("failed to %s screen protector mode",
			enabled ? "enable" : "disable");

	return 0;
}

static int brl_get_screen_protector_mode_enabled(
	struct goodix_ts_core *cd, bool *enabled)
{
	int ret = 0;
	u8 val;

	ret = cd->hw_ops->read(cd, GOODIX_FEATURE_STATUS_ADDR, &val, 1);
	if (ret != 0) {
		ts_err("failed to get screen protector mode enabled, ret: %d",
			ret);
		*enabled = false;
		return ret;
	}
	*enabled = (val & CUSTOM_MODE_MASK_SCREEN_PROTECTOR) != 0;
	return ret;
}

static int brl_get_mutual_data(struct goodix_ts_core *cd, enum frame_data_type type)
{
	int ret = 0;
	u8 val;
	int tx = cd->ic_info.parm.drv_num;
	int rx = cd->ic_info.parm.sen_num;
	u32 mutual_addr;
	u32 flag_addr = cd->ic_info.misc.frame_data_addr;
	int retry = 20;
	struct goodix_ts_cmd cmd = { 0 };

	mutual_addr = cd->hw_ops->get_ms_data_addr(cd, type);
	cmd.cmd = GOODIX_CMD_SET_FRAMEDATA_ENABLED;
	cmd.len = 5;
	cmd.data[0] = type;
	ret = cd->hw_ops->send_cmd(cd, &cmd);
	if (ret < 0) {
		ts_err("report rawdata failed, exit!");
		goto exit;
	}

	/* clean touch event flag */
	val = 0;
	ret = cd->hw_ops->write(cd, flag_addr, &val, 1);
	if (ret < 0) {
		ts_err("clean touch event failed, exit!");
		goto exit;
	}

	while (retry--) {
		usleep_range(2000, 2100);
		ret = cd->hw_ops->read(cd, flag_addr, &val, 1);
		if (!ret && (val & 0x80))
			break;
	}
	if (retry < 0) {
		ts_err("framedata is not ready val:0x%02x, exit!", val);
		ret = -EINVAL;
		goto exit;
	}

	ret = cd->hw_ops->read(cd, mutual_addr, (u8*)cd->mutual_data, tx * rx * 2);
	if (ret < 0) {
		ts_err("read frame data failed");
		goto exit;
	}

	goodix_rotate_abcd2cbad(tx, rx, (s16 *)cd->mutual_data,
		(s16 *)cd->mutual_data_manual);

exit:
	cmd.cmd = GOODIX_CMD_SET_FRAMEDATA_ENABLED;
	cmd.data[0] = 0;
	cmd.len = 5;
	if (ret == 0) {
		ret = cd->hw_ops->send_cmd(cd, &cmd);
	} else {
		cd->hw_ops->send_cmd(cd, &cmd);
	}
	return ret;
}

static int brl_get_self_sensing_data(struct goodix_ts_core *cd, enum frame_data_type type)
{
	int ret = 0;
	u8 val;
	int tx = cd->ic_info.parm.drv_num;
	int rx = cd->ic_info.parm.sen_num;
	u32 self_addr;
	u32 flag_addr = cd->ic_info.misc.frame_data_addr;
	int retry = 20;
	struct goodix_ts_cmd cmd = { 0 };

	self_addr = cd->hw_ops->get_ss_data_addr(cd, type);
	cmd.cmd = GOODIX_CMD_SET_FRAMEDATA_ENABLED;
	cmd.len = 5;
	cmd.data[0] = type;
	ret = cd->hw_ops->send_cmd(cd, &cmd);
	if (ret < 0) {
		ts_err("report rawdata failed, exit!");
		goto exit;
	}

	/* clean touch event flag */
	val = 0;
	ret = cd->hw_ops->write(cd, flag_addr, &val, 1);
	if (ret < 0) {
		ts_err("clean touch event failed, exit!");
		goto exit;
	}

	while (retry--) {
		usleep_range(2000, 2100);
		ret = cd->hw_ops->read(cd, flag_addr, &val, 1);
		if (!ret && (val & 0x80))
			break;
	}
	if (retry < 0) {
		ts_err("framedata is not ready val:0x%02x, exit!", val);
		ret = -EINVAL;
		goto exit;
	}

	ret = cd->hw_ops->read(cd, self_addr, (u8*)cd->self_sensing_data_manual,
		(tx + rx) * 2);
	if (ret < 0) {
		ts_err("read frame data failed");
		goto exit;
	}

exit:
	cmd.cmd = GOODIX_CMD_SET_FRAMEDATA_ENABLED;
	cmd.data[0] = 0;
	cmd.len = 5;
	if (ret == 0) {
		ret = cd->hw_ops->send_cmd(cd, &cmd);
	} else {
		cd->hw_ops->send_cmd(cd, &cmd);
	}
	return ret;
}

#define GOODIX_CMD_SET_COORD_FILTER 0xCA
static int brl_set_coord_filter_enabled(
	struct goodix_ts_core *cd, bool enabled)
{
	struct goodix_ts_cmd cmd = { 0 };
	int ret = 0;

	cmd.cmd = GOODIX_CMD_SET_COORD_FILTER;
	cmd.len = 5;
	cmd.data[0] = enabled ? 0 : 1;

	ret = cd->hw_ops->send_cmd(cd, &cmd);
	if (ret != 0)
		ts_err("failed to %s coordinate filter",
			enabled ? "enable" : "disable");

	return ret;
}

static int brl_get_coord_filter_enabled(
	struct goodix_ts_core *cd, bool *enabled)
{
	int ret = 0;
	u8 val;

	ret = cd->hw_ops->read(cd, GOODIX_FEATURE_STATUS_ADDR, &val, 1);
	if (ret != 0) {
		ts_err("failed to get coordinate filter enabled, ret: %d", ret);
		*enabled = false;
		return ret;
	}
	*enabled = (val & CUSTOM_MODE_MASK_COORD_FILTER) == 0;
	return ret;
}

#define GOODIX_CMD_SET_REPORT_RATE 0x9D
#define GOODIX_REPORT_RATE_240HZ 0
#define GOODIX_REPORT_RATE_120HZ 2
static int brl_set_report_rate(
	struct goodix_ts_core *cd, u32 rate)
{
	struct goodix_ts_cmd cmd = { 0 };
	int ret = 0;

	ts_info("set report rate %d", rate);
	if ((rate != 120) && (rate != 240)) {
		ts_info("Report Rate: %dHz is not support", rate);
		return -EOPNOTSUPP;
	}

	cmd.cmd = GOODIX_CMD_SET_REPORT_RATE;
	cmd.len = 5;
	cmd.data[0] = rate == 240 ?
		GOODIX_REPORT_RATE_240HZ : GOODIX_REPORT_RATE_120HZ;

	ret = cd->hw_ops->send_cmd(cd, &cmd);
	if (ret != 0)
		ts_err("failed to set report rate");
	return ret;
}

typedef struct __attribute__((packed)) {
	uint32_t checksum;
	uint32_t address;
	uint32_t length;
} flash_head_info_t;

#define FLASH_CMD_R_START 0x09
#define FLASH_CMD_W_START 0x0A
#define FLASH_CMD_RW_FINISH 0x0B
#define FLASH_CMD_STATE_READY 0x04
#define FLASH_CMD_STATE_CHECKERR 0x05
#define FLASH_CMD_STATE_DENY 0x06
#define FLASH_CMD_STATE_OKAY 0x07
static int goodix_flash_cmd(
	struct goodix_ts_core *cd, uint8_t cmd, uint8_t status, int retry_count)
{
	u8 cmd_buf[] = { 0x00, 0x00, 0x04, 0x00, 0x00, 0x00 };
	int ret;
	int i;
	u8 r_sta;

	cmd_buf[3] = cmd;
	goodix_append_checksum(&cmd_buf[2], 2, CHECKSUM_MODE_U8_LE);
	ret = cd->hw_ops->write(
		cd, cd->ic_info.misc.cmd_addr, cmd_buf, sizeof(cmd_buf));
	if (ret < 0)
		return ret;

	if (retry_count == 0)
		return 0;

	for (i = 0; i < retry_count; i++) {
		mdelay(2);
		ret = cd->hw_ops->read(
			cd, cd->ic_info.misc.cmd_addr, &r_sta, 1);
		if (ret == 0 && r_sta == status)
			return 0;
	}

	ts_err("r_sta[0x%x] != status[0x%x]", r_sta, status);
	return -EINVAL;
}

static int brl_flash_read(struct goodix_ts_core *cd, u32 addr, u8 *buf, int len)
{
	int i;
	int ret;
	u8 *tmp_buf;
	u32 buffer_addr = cd->ic_info.misc.fw_buffer_addr;
	uint32_t checksum = 0;
	uint32_t read_len;
	flash_head_info_t head_info;
	u8 *p = (u8 *)&head_info.address;

	if ((len % 2) != 0)
		read_len = len + 1; // if odd number, must +1
	else
		read_len = len;

	tmp_buf = kzalloc(read_len + sizeof(flash_head_info_t), GFP_KERNEL);
	if (!tmp_buf)
		return -ENOMEM;

	head_info.address = cpu_to_le32(addr);
	head_info.length = cpu_to_le32(read_len);
	for (i = 0; i < 8; i += 2)
		checksum += p[i] | (p[i + 1] << 8);
	head_info.checksum = checksum;

	ret = goodix_flash_cmd(
		cd, FLASH_CMD_R_START, FLASH_CMD_STATE_READY, 15);
	if (ret < 0) {
		ts_err("failed enter flash read state");
		goto read_end;
	}

	ret = cd->hw_ops->write(
		cd, buffer_addr, (u8 *)&head_info, sizeof(head_info));
	if (ret < 0) {
		ts_err("failed write flash head info");
		goto read_end;
	}

	ret = goodix_flash_cmd(
		cd, FLASH_CMD_RW_FINISH, FLASH_CMD_STATE_OKAY, 50);
	if (ret) {
		ts_err("failed read flash ready state");
		goto read_end;
	}

	ret = cd->hw_ops->read(
		cd, buffer_addr, tmp_buf, read_len + sizeof(flash_head_info_t));
	if (ret < 0) {
		ts_err("failed read data len %lu",
			read_len + sizeof(flash_head_info_t));
		goto read_end;
	}

	checksum = 0;
	for (i = 0; i < read_len + sizeof(flash_head_info_t) - 4; i += 2)
		checksum += tmp_buf[4 + i] | (tmp_buf[5 + i] << 8);

	if (checksum != le32_to_cpup((__le32 *)tmp_buf)) {
		ts_err("read back data checksum error");
		ret = -EINVAL;
		goto read_end;
	}

	memcpy(buf, tmp_buf + sizeof(flash_head_info_t), len);
	ret = 0;
read_end:
	goodix_flash_cmd(cd, 0x0C, 0, 0);
	return ret;
}

static u32 brl_get_ms_data_addr(struct goodix_ts_core *cd, enum frame_data_type type)
{
	u32 addr = cd->ic_info.misc.frame_data_addr +
			cd->ic_info.misc.frame_data_head_len +
			cd->ic_info.misc.fw_attr_len +
			cd->ic_info.misc.fw_log_len + 8;

	return addr;
}

static u32 brl_get_ss_data_addr(struct goodix_ts_core *cd, enum frame_data_type type)
{
	u32 addr = cd->ic_info.misc.frame_data_addr +
			cd->ic_info.misc.frame_data_head_len +
			cd->ic_info.misc.fw_attr_len + cd->ic_info.misc.fw_log_len +
			cd->ic_info.misc.mutual_struct_len + 10;

	return addr;
}

#define GOODIX_CMD_SET_PANEL_SPEED_MODE 0xF2
static int brl_set_panel_speed_mode(
	struct goodix_ts_core *cd, bool is_high_speed)
{
	struct goodix_ts_cmd cmd = { 0 };
	int ret = 0;

	cmd.cmd = GOODIX_CMD_SET_PANEL_SPEED_MODE;
	cmd.len = 5;
	cmd.data[0] = is_high_speed ? 0 : 1;

	ret = cd->hw_ops->send_cmd(cd, &cmd);
	if (ret != 0)
		ts_err("failed to set panel speed mode: %s",
			is_high_speed ? "HS" : "NS");

	return ret;
}

static struct goodix_ts_hw_ops brl_hw_ops = {
	.power_on = brl_power_on,
	.resume = brl_resume,
	.suspend = brl_suspend,
	.gesture = brl_gesture,
	.reset = brl_reset,
	.irq_enable = brl_irq_enable,
	.disable_irq_nosync = brl_disable_irq_nosync,
	.read = brl_read,
	.read_fast = brl_read_fast,
	.write = brl_write,
	.send_cmd = brl_send_cmd,
	.send_config = brl_send_config,
	.read_config = brl_read_config,
	.read_version = brl_read_version,
	.get_ic_info = brl_get_ic_info,
	.esd_check = brl_esd_check,
	.event_handler = brl_event_handler,
	.after_event_handler = brl_after_event_handler,
	.get_capacitance_data = brl_get_capacitance_data,
	.read_flash = brl_flash_read,
/* [GOOG] */
	.ping = brl_dev_confirm,
	.get_scan_mode = brl_get_scan_mode,
	.set_scan_mode = brl_set_scan_mode,
	.set_continuously_report_enabled = brl_set_continuously_report_enabled,
	.set_heatmap_enabled = brl_set_heatmap_enabled,
	.set_palm_enabled = brl_set_palm_enabled,
	.get_palm_enabled = brl_get_palm_enabled,
	.set_grip_enabled = brl_set_grip_enabled,
	.get_grip_enabled = brl_get_grip_enabled,
	.set_screen_protector_mode_enabled =
		brl_set_screen_protector_mode_enabled,
	.get_screen_protector_mode_enabled =
		brl_get_screen_protector_mode_enabled,
	.get_mutual_data = brl_get_mutual_data,
	.get_self_sensing_data = brl_get_self_sensing_data,
	.set_coord_filter_enabled = brl_set_coord_filter_enabled,
	.get_coord_filter_enabled = brl_get_coord_filter_enabled,
	.set_report_rate = brl_set_report_rate,
	.get_ms_data_addr = brl_get_ms_data_addr,
	.get_ss_data_addr = brl_get_ss_data_addr,
	.set_panel_speed_mode = brl_set_panel_speed_mode,
/*~[GOOG] */
};

struct goodix_ts_hw_ops *goodix_get_hw_ops(void)
{
	return &brl_hw_ops;
}
