// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2020 Google LLC
 *
 */

#include <linux/kernel.h>
#include <linux/of_address.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <soc/google/acpm_mfd.h>
#include <soc/google/acpm_ipc_ctrl.h>
#include <linux/sched/clock.h>

int exynos_acpm_read_reg(struct device_node *acpm_mfd_node, u8 channel,
			 u16 type, u8 reg, u8 *dest)
{
	unsigned int channel_num, size;
	struct ipc_config config;
	unsigned int command[4] = {0,};
	int ret = 0;

	if (!acpm_ipc_request_channel(acpm_mfd_node, NULL, &channel_num, &size)) {
		config.cmd = command;
		config.cmd[0] = set_protocol(type, TYPE) | set_protocol(reg, REG) |
				set_protocol(channel, CHANNEL);
		config.cmd[1] = set_protocol(FUNC_READ, FUNC);
		config.cmd[3] = (u32)(sched_clock() / 1000000); /*record ktime ms*/
		config.response = true;

		ACPM_MFD_PRINT("%s - addr: 0x%03x\n", __func__,
			       set_protocol(type, TYPE) | set_protocol(reg, REG));

		ret = acpm_ipc_send_data(channel_num, &config);
		if (ret) {
			pr_err("%s - acpm_ipc_send_data fail.\n", __func__);
			return ret;
		}

		*dest = read_protocol(config.cmd[1], DEST);
		ret = read_protocol(config.cmd[1], RETURN);
		if (ret) {
			pr_err("%s - APM's speedy_rx fail.\n", __func__);
			return ret;
		}

		ACPM_MFD_PRINT("%s - data = 0x%02x ret = 0x%02x\n",
			       __func__, *dest, ret);

		acpm_ipc_release_channel(acpm_mfd_node, channel_num);
	} else {
		pr_err("%s ipc request_channel fail, id:%u, size:%u\n",
		       __func__, channel_num, size);
		ret = -EBUSY;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(exynos_acpm_read_reg);

int exynos_acpm_bulk_read(struct device_node *acpm_mfd_node, u8 channel,
			  u16 type, u8 reg, int count, u8 *buf)
{
	unsigned int channel_num, size;
	int ret = 0;

	if (acpm_ipc_request_channel(acpm_mfd_node, NULL, &channel_num, &size)) {
		pr_err("%s ipc request_channel fail, id:%u, size:%u\n",
				__func__, channel_num, size);
		return -EBUSY;
	}

	while (count > 0) {
		struct ipc_config config;
		unsigned int command[4] = {0,};
		int i, bytes_to_read;

		bytes_to_read = (count > BULK_TRANSFER_LIMIT) ? BULK_TRANSFER_LIMIT : count;

		config.cmd = command;
		config.cmd[0] = set_protocol(type, TYPE) | set_protocol(reg, REG) |
				set_protocol(channel, CHANNEL);
		config.cmd[1] = set_protocol(FUNC_BULK_READ, FUNC) |
			set_protocol(bytes_to_read, CNT);
		config.response = true;

		ACPM_MFD_PRINT("%s - addr: 0x%03x\n", __func__,
					set_protocol(type, TYPE) | set_protocol(reg, REG));

		ret = acpm_ipc_send_data(channel_num, &config);
		if (ret) {
			pr_err("%s - acpm_ipc_send_data fail.\n", __func__);
			goto end;
		}

		ret = read_protocol(config.cmd[1], RETURN);
		if (ret) {
			pr_err("%s - APM's speedy_rx fail.\n", __func__);
			goto end;
		}

		for (i = 0; i < bytes_to_read; i++) {
			if (i < 4)
				buf[i] = read_bulk_protocol(config.cmd[2], BULK_VAL, i);
			else
				buf[i] = read_bulk_protocol(config.cmd[3], BULK_VAL, i - 4);
		}

		count -= bytes_to_read;
		reg += bytes_to_read;
		buf += bytes_to_read;
	}

end:
	acpm_ipc_release_channel(acpm_mfd_node, channel_num);
	return ret;
}
EXPORT_SYMBOL_GPL(exynos_acpm_bulk_read);

int exynos_acpm_write_reg(struct device_node *acpm_mfd_node, u8 channel,
			  u16 type, u8 reg, u8 value)
{
	unsigned int channel_num, size;
	struct ipc_config config;
	unsigned int command[4] = {0,};
	int ret = 0;

	if (!acpm_ipc_request_channel(acpm_mfd_node, NULL, &channel_num, &size)) {
		config.cmd = command;
		config.cmd[0] = set_protocol(type, TYPE) | set_protocol(reg, REG) |
				set_protocol(channel, CHANNEL);
		config.cmd[1] = set_protocol(FUNC_WRITE, FUNC) |
				set_protocol(value, WRITE_VAL);
		config.cmd[3] = (u32)(sched_clock() / 1000000); /*record ktime ms*/
		config.response = true;

		ACPM_MFD_PRINT("%s - addr: 0x%03x val: 0x%02x\n",
			       __func__, set_protocol(type, TYPE) |
			       set_protocol(reg, REG), value);

		ret = acpm_ipc_send_data(channel_num, &config);
		if (ret) {
			pr_err("%s - acpm_ipc_send_data fail\n",
			       __func__);
			return ret;
		}

		ret = read_protocol(config.cmd[1], RETURN);
		if (ret) {
			pr_err("%s - APM's speedy_tx fail.\n",
			       __func__);
			return ret;
		}

		ACPM_MFD_PRINT("%s - read(ret) val: 0x%02x\n", __func__, ret);

		acpm_ipc_release_channel(acpm_mfd_node, channel_num);
	} else {
		pr_err("%s ipc request_channel fail, id:%u, size:%u\n",
		       __func__, channel_num, size);
		ret = -EBUSY;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(exynos_acpm_write_reg);

int exynos_acpm_bulk_write(struct device_node *acpm_mfd_node, u8 channel,
			   u16 type, u8 reg, int count, u8 *buf)
{
	unsigned int channel_num, size;
	int ret = 0;

	if (acpm_ipc_request_channel(acpm_mfd_node, NULL, &channel_num, &size)) {
		pr_err("%s ipc request_channel fail, id:%u, size:%u\n",
				__func__, channel_num, size);
		return -EBUSY;
	}

	while (count > 0) {
		struct ipc_config config;
		unsigned int command[4] = {0,};
		int i, bytes_to_write;

		bytes_to_write = (count > BULK_TRANSFER_LIMIT) ? BULK_TRANSFER_LIMIT : count;

		config.cmd = command;
		config.cmd[0] = set_protocol(type, TYPE) | set_protocol(reg, REG) |
				set_protocol(channel, CHANNEL);
		config.cmd[1] = set_protocol(FUNC_BULK_WRITE, FUNC) |
			set_protocol(bytes_to_write, CNT);
		config.response = true;

		ACPM_MFD_PRINT("%s - addr: 0x%03x\n cnt: 0x%02x\n", __func__,
					set_protocol(type, TYPE) | set_protocol(reg, REG),
					bytes_to_write);

		for (i = 0; i < bytes_to_write; i++) {
			if (i < 4)
				config.cmd[2] |= set_bulk_protocol(buf[i], BULK_VAL, i);
			else
				config.cmd[3] |= set_bulk_protocol(buf[i], BULK_VAL, i - 4);
		}

		ret = acpm_ipc_send_data(channel_num, &config);
		if (ret) {
			pr_err("%s - acpm_ipc_send_data fail.\n", __func__);
			goto end;
		}

		ret = read_protocol(config.cmd[1], RETURN);
		if (ret) {
			pr_err("%s - APM's speedy_tx fail.\n", __func__);
			goto end;
		}

		count -= bytes_to_write;
		reg += bytes_to_write;
		buf += bytes_to_write;
	}

end:
	acpm_ipc_release_channel(acpm_mfd_node, channel_num);
	return ret;
}
EXPORT_SYMBOL_GPL(exynos_acpm_bulk_write);

int exynos_acpm_update_reg(struct device_node *acpm_mfd_node, u8 channel,
			   u16 type, u8 reg, u8 value, u8 mask)
{
	unsigned int channel_num, size;
	struct ipc_config config;
	unsigned int command[4] = {0,};
	int ret = 0;

	if (!acpm_ipc_request_channel(acpm_mfd_node, NULL, &channel_num, &size)) {
		config.cmd = command;
		config.cmd[0] = set_protocol(type, TYPE) | set_protocol(reg, REG) |
				set_protocol(channel, CHANNEL);
		config.cmd[1] = set_protocol(FUNC_UPDATE, FUNC)
				| set_protocol(value, UPDATE_VAL)
				| set_protocol(mask, UPDATE_MASK);
		config.cmd[3] = (u32)(sched_clock() / 1000000); /*record ktime ms*/
		config.response = true;

		ACPM_MFD_PRINT("%s - addr: 0x%03x val: 0x%02x mask: 0x%02x\n", __func__,
			       set_protocol(type, TYPE) | set_protocol(reg, REG),
			       value, mask);

		ret = acpm_ipc_send_data(channel_num, &config);
		if (ret) {
			pr_err("%s - acpm_ipc_send_data fail.\n", __func__);
			return ret;
		}

		ret = read_protocol(config.cmd[1], RETURN);
		if (ret) {
			if (ret == 1)
				pr_err("%s - APM's speedy_rx fail.\n", __func__);
			else if (ret == 2)
				pr_err("%s - APM's speedy_tx fail.\n", __func__);
			return ret;
		}

		acpm_ipc_release_channel(acpm_mfd_node, channel_num);
	} else {
		pr_err("%s ipc request_channel fail, id:%u, size:%u\n",
		       __func__, channel_num, size);
		ret = -EBUSY;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(exynos_acpm_update_reg);
