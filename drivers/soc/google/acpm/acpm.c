// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2020 Google LLC
 *
 */

#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/sched/clock.h>
#include <linux/module.h>

#include "acpm.h"
#include "acpm_ipc.h"
#include "fw_header/framework.h"

static struct acpm_info *exynos_acpm;

static int debug_log_level_get(void *data, unsigned long long *val)
{
	*val = acpm_fw_get_log_level();
	return 0;
}

static int debug_log_level_set(void *data, unsigned long long val)
{
	acpm_fw_set_log_level(val);

	return 0;
}

static int debug_retry_log_ctrl_get(void *data, unsigned long long *val)
{
	*val = acpm_fw_get_retry_log_ctrl();
	return 0;
}

static int debug_retry_log_ctrl_set(void *data, unsigned long long val)
{
	acpm_fw_set_retry_log_ctrl(val);

	return 0;
}

static int debug_ipc_loopback_test_get(void *data, unsigned long long *val)
{
	struct acpm_info *acpm = (struct acpm_info *) data;
	struct ipc_config config;
	int ret = 0;
	unsigned int cmd[4] = {0, };
	unsigned long long ipc_time_start;
	unsigned long long ipc_time_end;
	unsigned int channel_num, size;

	ret = acpm_ipc_request_channel(acpm->dev->of_node, NULL, &channel_num,
				       &size);
	if (ret) {
		pr_err("%s ipc request_channel fail (%d)\n", __func__, ret);
		return ret;
	}

	config.cmd = cmd;
	config.cmd[0] = (1 << ACPM_IPC_PROTOCOL_TEST);

	config.response = true;

	ipc_time_start = sched_clock();
	ret = acpm_ipc_send_data(channel_num, &config);
	ipc_time_end = sched_clock();

	if (!ret)
		*val = ipc_time_end - ipc_time_start;
	else
		*val = 0;

	acpm_ipc_release_channel(acpm->dev->of_node, channel_num);

	return 0;
}

static int acpm_set_print_setting(struct acpm_info *acpm,
				  enum acpm_print_settings setting,
				  u64 val)
{
	struct ipc_config config;
	int ret = 0;
	unsigned int cmd[4] = {0, };
	unsigned int channel_num, size;

	ret = acpm_ipc_request_channel(acpm->dev->of_node, NULL, &channel_num,
				       &size);
	if (ret) {
		pr_err("%s ipc request_channel fail (%d)\n", __func__, ret);
		return ret;
	}

	config.cmd = cmd;
	config.cmd[0] = 0x1 << ACPM_IPC_PROTOCOL_SETTINGS;
	config.cmd[0] |= (setting & 0x3) << (ACPM_IPC_PROTOCOL_SETTINGS + 1);
	config.cmd[1] = val;
	config.cmd[2] = val >> 32;

	config.response = true;

	ret = acpm_ipc_send_data(channel_num, &config);

	acpm_ipc_release_channel(acpm->dev->of_node, channel_num);

	return ret;
}

static int acpm_get_print_setting(struct acpm_info *acpm,
				  enum acpm_print_settings setting,
				  u64 *val)
{
	struct ipc_config config;
	int ret = 0;
	unsigned int cmd[4] = {0, };
	unsigned int channel_num, size;

	ret = acpm_ipc_request_channel(acpm->dev->of_node, NULL, &channel_num,
				       &size);
	if (ret) {
		pr_err("%s ipc request_channel fail (%d)\n", __func__, ret);
		return ret;
	}

	config.cmd = cmd;
	config.cmd[0] = 0x1 << ACPM_IPC_PROTOCOL_SETTINGS;
	config.cmd[0] |= (setting & 0x3) << (ACPM_IPC_PROTOCOL_SETTINGS + 1);

	config.response = true;
	ret = acpm_ipc_send_data(channel_num, &config);

	if (!ret)
		*val = (((u64) config.cmd[2]) << 32) | config.cmd[1];

	acpm_ipc_release_channel(acpm->dev->of_node, channel_num);

	return ret;
}

static int acpm_framework_debug_cmd_setting(struct acpm_info *acpm,
				  u64 subcmd)
{
	struct ipc_config config;
	int ret = 0;
	unsigned int cmd[4] = {0, };
	unsigned int channel_num, size;

	if (subcmd >= ACPM_FRAMEWORK_COMMAND_DEBUG_MAX) {
		pr_err("%s, sub-cmd:%llu, out of range!\n", __func__, subcmd);
		return 0;
	}

	ret = acpm_ipc_request_channel(acpm->dev->of_node, NULL, &channel_num,
				       &size);
	if (ret) {
		pr_err("%s ipc request_channel fail (%d)\n", __func__, ret);
		return ret;
	}

	config.cmd = cmd;
	config.cmd[0] = subcmd;
	config.cmd[0] |= ACPM_FRAMEWORK_COMMAND_DEBUG <<
				(ACPM_IPC_PROTOCOL_SETTINGS + 1);

	config.response = true;

	pr_info("%s, command:0x%X, sub-cmd:0x%llX\n", __func__, config.cmd[0], subcmd);

	ret = acpm_ipc_send_data(channel_num, &config);

	acpm_ipc_release_channel(acpm->dev->of_node, channel_num);

	return ret;
}

static int debug_uart_gprio_level_get(void *data, u64 *val)
{
	struct acpm_info *acpm = (struct acpm_info *) data;

	return acpm_get_print_setting(acpm, ACPM_GET_UART_GPRIO_LEVEL, val);
}

static int debug_uart_gprio_level_set(void *data, u64 val)
{
	struct acpm_info *acpm = (struct acpm_info *) data;

	return acpm_set_print_setting(acpm, ACPM_SET_UART_GPRIO_LEVEL, val);
}

static int debug_logb_gprio_level_get(void *data, u64 *val)
{
	struct acpm_info *acpm = (struct acpm_info *) data;

	return acpm_get_print_setting(acpm, ACPM_GET_LOGB_GPRIO_LEVEL, val);
}

static int debug_logb_gprio_level_set(void *data, u64 val)
{
	struct acpm_info *acpm = (struct acpm_info *) data;

	return acpm_set_print_setting(acpm, ACPM_SET_LOGB_GPRIO_LEVEL, val);
}

static int debug_acpm_framework_cmd_set(void *data, u64 val)
{
	struct acpm_info *acpm = (struct acpm_info *) data;

	return acpm_framework_debug_cmd_setting(acpm, val);
}

DEFINE_SIMPLE_ATTRIBUTE(debug_log_level_fops,
		debug_log_level_get, debug_log_level_set, "0%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(debug_retry_log_ctrl_fops,
		debug_retry_log_ctrl_get, debug_retry_log_ctrl_set, "0%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(debug_ipc_loopback_test_fops,
		debug_ipc_loopback_test_get, NULL, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(debug_uart_gprio_level_fops,
		debug_uart_gprio_level_get,
		debug_uart_gprio_level_set, "0x%016llx\n");
DEFINE_SIMPLE_ATTRIBUTE(debug_logb_gprio_level_fops,
		debug_logb_gprio_level_get,
		debug_logb_gprio_level_set, "0x%016llx\n");
DEFINE_SIMPLE_ATTRIBUTE(debug_acpm_framework_cmd_fops,
		NULL, debug_acpm_framework_cmd_set, "0x%016llx\n");

static void acpm_debugfs_init(struct acpm_info *acpm)
{
	struct dentry *den;

	den = debugfs_lookup("acpm_framework", NULL);
	if (!den)
		den = debugfs_create_dir("acpm_framework", NULL);

	debugfs_create_file("ipc_loopback_test", 0644, den, acpm,
			    &debug_ipc_loopback_test_fops);
	debugfs_create_file("log_level", 0644, den, acpm,
			    &debug_log_level_fops);
	debugfs_create_file("retry_log_ctrl", 0644, den, acpm,
			    &debug_retry_log_ctrl_fops);
	debugfs_create_file("uart_gprio_level", 0644, den, acpm,
			    &debug_uart_gprio_level_fops);
	debugfs_create_file("logb_gprio_level", 0644, den, acpm,
			    &debug_logb_gprio_level_fops);
	debugfs_create_file("acpm_debug_cmd", 0644, den, acpm,
			    &debug_acpm_framework_cmd_fops);
}

void *memcpy_align_4(void *dest, const void *src, unsigned int n)
{
	unsigned int *dp = dest;
	const unsigned int *sp = src;
	int i;

	if ((n % 4))
		WARN_ON(1);

	n = n >> 2;

	for (i = 0; i < n; i++)
		*dp++ = *sp++;

	return dest;
}

static void acpm_enter_wfi(struct acpm_info *acpm)
{
	struct ipc_config config;
	int ret = 0;
	unsigned int channel_num, size;
	unsigned int cmd[4] = {0, };

	if (acpm->enter_wfi)
		return;

	ret = acpm_ipc_request_channel(acpm->dev->of_node, NULL, &channel_num,
				       &size);
	if (ret) {
		pr_err("%s ipc request_channel fail (%d)\n", __func__, ret);
		return;
	}
	config.cmd = cmd;
	config.response = true;
	config.cmd[0] = 1 << ACPM_IPC_PROTOCOL_STOP;

	acpm_stop_log_and_dumpram();
	ret = acpm_ipc_send_data(channel_num, &config);

	if (ret) {
		pr_err("[ACPM] acpm enter wfi fail!!\n");
	} else {
		pr_err("[ACPM] wfi done\n");
		acpm->enter_wfi++;
	}
	acpm_ipc_release_channel(acpm->dev->of_node, channel_num);
}

void exynos_acpm_reboot(void)
{
	acpm_ipc_set_waiting_mode(BUSY_WAIT);

	acpm_enter_wfi(exynos_acpm);
}
EXPORT_SYMBOL_GPL(exynos_acpm_reboot);

static void acpm_shutdown(struct platform_device *pdev)
{
	pr_info("%s...\n", __func__);

	acpm_framework_debug_cmd_setting(exynos_acpm, ACPM_FRAMEWORK_COMMAND_DEBUG_NOTIFY_SHUTDOWN);

	acpm_ipc_set_waiting_mode(BUSY_WAIT);

	acpm_stop_log_and_dumpram();
}

static int acpm_probe(struct platform_device *pdev)
{
	struct acpm_info *acpm;
	struct device_node *node = pdev->dev.of_node;
	int ret = 0;

	dev_info(&pdev->dev, "acpm probe\n");

	if (!node) {
		dev_err(&pdev->dev, "driver cannot support non-dt devices\n");
		return -ENODEV;
	}

	acpm = devm_kzalloc(&pdev->dev, sizeof(struct acpm_info), GFP_KERNEL);
	if (IS_ERR(acpm))
		return PTR_ERR(acpm);
	if (!acpm)
		return -ENOMEM;

	acpm->dev = &pdev->dev;

	exynos_acpm = acpm;

	acpm_debugfs_init(acpm);

	dev_info(&pdev->dev, "acpm probe done.\n");
	return ret;
}

static int acpm_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id acpm_ipc_match[] = {
	{ .compatible = "google,gs-acpm-ipc" },
	{},
};
MODULE_DEVICE_TABLE(of, acpm_ipc_match);

static struct platform_driver samsung_acpm_ipc_driver = {
	.probe	= acpm_ipc_probe,
	.remove	= acpm_ipc_remove,
	.driver	= {
		.name = "gs-acpm-ipc",
		.owner	= THIS_MODULE,
		.of_match_table	= acpm_ipc_match,
	},
};

static const struct of_device_id acpm_match[] = {
	{ .compatible = "google,gs-acpm" },
	{},
};
MODULE_DEVICE_TABLE(of, acpm_match);

static struct platform_driver samsung_acpm_driver = {
	.probe	= acpm_probe,
	.remove	= acpm_remove,
	.shutdown = acpm_shutdown,
	.driver	= {
		.name = "gs-acpm",
		.owner	= THIS_MODULE,
		.of_match_table	= acpm_match,
	},
};

static int exynos_acpm_init(void)
{
	platform_driver_register(&samsung_acpm_ipc_driver);
	platform_driver_register(&samsung_acpm_driver);
	return 0;
}
postcore_initcall_sync(exynos_acpm_init);

static void exynos_acpm_exit(void)
{
	platform_driver_unregister(&samsung_acpm_ipc_driver);
	platform_driver_unregister(&samsung_acpm_driver);
}
module_exit(exynos_acpm_exit);

MODULE_LICENSE("GPL");
