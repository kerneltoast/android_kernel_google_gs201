// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2019 Google LLC
 *    Author: Namhyung Kim <namhyung@google.com>
 */

#define pr_fmt(fmt) "dbgcore: " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>

#include <soc/google/debug-snapshot.h>
#include <soc/google/exynos-adv-tracer.h>

#define DBGCORE_LOG_OFFSET      0x3000
#define DBGCORE_VERSION_LENGTH  48
#define DBGCORE_LOG_LENGTH      1024
#define DBGCORE_LOG_STR_SZ      8

enum dbgcore_plugin_id {
	DBGCORE_PID_FRAMEWORK,
	DBGCORE_PID_ARRAYDUMP,
	DBGCORE_PID_SCAN2DRAM,
	DBGCORE_PID_BCMDBG,
	DBGCORE_PID_CONDBG,
	DBGCORE_PID_TESTPLUG,
	DBGCORE_PID_MAX,
};

static const char *dbgcore_plugin_name[] = {
	"FRM",
	"ARR",
	"S2D",
	"BCM",
	"ECD",
	"TST",
	"???",
};

struct dbgcore_log {
	u8	plugin_id;
	u8	reserved0;
	u16	index;
	char	str[DBGCORE_LOG_STR_SZ];
	u32	val;
};

static struct dentry *dbgcore_dentry;


static ssize_t dbgcore_version_read(struct file *file, char __user *ubuf,
				    size_t count, loff_t *ppos)
{
	uintptr_t dump_base = dbg_snapshot_get_item_vaddr("header");

	if (dump_base == 0)
		return -ENODEV;

	dump_base += DBGCORE_LOG_OFFSET;
	return simple_read_from_buffer(ubuf, count, ppos, (void *)dump_base,
				       DBGCORE_VERSION_LENGTH);
}

static ssize_t dbgcore_logdump_read(struct file *file, char __user *ubuf,
				    size_t count, loff_t *ppos)
{
	struct dbgcore_log *log;
	uintptr_t dump_base = dbg_snapshot_get_item_vaddr("header");
	ssize_t ret = 0;
	char buf[48];

	if (dump_base == 0)
		return -ENODEV;

	if ((*ppos < 0) || (*ppos % sizeof(*log)) != 0) {
		pr_err("unaligned access is not supported\n");
		return -EINVAL;
	}

	dump_base += DBGCORE_LOG_OFFSET + DBGCORE_VERSION_LENGTH;
	while (*ppos + sizeof(*log) <= DBGCORE_LOG_LENGTH) {
		size_t len;
		const char *pname = dbgcore_plugin_name[DBGCORE_PID_MAX];

		if (count <= sizeof(buf))
			break;

		log = (void *)(dump_base + *ppos);
		if (log->reserved0 != 0)
			break;
		if (log->plugin_id < DBGCORE_PID_MAX)
			pname = dbgcore_plugin_name[log->plugin_id];

		len = scnprintf(buf, sizeof(buf), "%#6x: %s: %8.8s: %#10x\n",
			       log->index, pname, log->str, log->val);
		if (copy_to_user(ubuf, buf, len))
			return -EFAULT;

		ret += len;
		ubuf += len;
		count -= len;

		*ppos += sizeof(*log);
	}

	return ret;
}

static ssize_t dbgcore_logdump_write(struct file *file, const char __user *buf,
				     size_t count, loff_t *ppos)
{
	struct adv_tracer_ipc_cmd cmd = {
		.cmd_raw = {
			.cmd = EAT_IPC_CMD_COPY_DEBUG_LOG,
			.size = 1,
		}
	};

	if (*ppos != 0)
		return -EINVAL;

	if (adv_tracer_ipc_send_data_polling(EAT_FRM_CHANNEL, &cmd) < 0) {
		pr_err("sending COPY_DEBUG_LOG cmd failed\n");
		return -EIO;
	}

	*ppos = count;
	return count;
}

static const struct file_operations dbgcore_version_fops = {
	.open	= simple_open,
	.read	= dbgcore_version_read,
	.llseek	= default_llseek,
};

static const struct file_operations dbgcore_logdump_fops = {
	.open	= simple_open,
	.read	= dbgcore_logdump_read,
	.write	= dbgcore_logdump_write,
	.llseek	= default_llseek,
};

static __init int dbgcore_dump_init(void)
{
	struct dentry *version;
	struct dentry *logdump;

	dbgcore_dentry = debugfs_create_dir("dbgcore", NULL);
	if (dbgcore_dentry == NULL) {
		pr_err("init failed\n");
		return -1;
	}

	version = debugfs_create_file("version", 0444, dbgcore_dentry, NULL,
				      &dbgcore_version_fops);
	if (version == NULL)
		goto out;

	logdump = debugfs_create_file("logdump", 0644, dbgcore_dentry, NULL,
				      &dbgcore_logdump_fops);
	if (logdump == NULL)
		goto out;

	pr_info("init ok\n");

	return 0;

out:
	pr_err("debugfs init failed\n");
	debugfs_remove_recursive(dbgcore_dentry);
	dbgcore_dentry = NULL;

	return -1;
}
module_init(dbgcore_dump_init);

static __exit void dbgcore_dump_exit(void)
{
	debugfs_remove_recursive(dbgcore_dentry);
	dbgcore_dentry = NULL;
}
module_exit(dbgcore_dump_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Debug Core Framework Driver");
