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
#include <linux/delay.h>
#include <linux/io.h>

#include <soc/google/debug-snapshot.h>
#include <soc/google/exynos-adv-tracer.h>

#include <asm/cacheflush.h>

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
	char buf[DSS_HDR_DBGC_VERSION_SZ + 2];
	int len;

	if (dump_base == 0)
		return -ENODEV;

	dump_base += DSS_HDR_DBGC_VERSION_OFFS;

	len = scnprintf(buf, sizeof(buf), "%.*s\n", DSS_HDR_DBGC_VERSION_SZ, (char *)dump_base);

	return simple_read_from_buffer(ubuf, count, ppos, buf, len);
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

	dump_base += DSS_HDR_DBGC_SRAM_LOG_OFFS;
	while (*ppos + sizeof(*log) <= DSS_HDR_DBGC_SRAM_LOG_SZ) {
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

static ssize_t dbgcore_nmi_info_read(struct file *file, char __user *ubuf,
				     size_t count, loff_t *ppos)
{
	struct adv_tracer_ipc_cmd cmd = {
		.cmd_raw = {
			.cmd = EAT_IPC_CMD_GET_NMI_INFO,
			.size = 1,
		}
	};
	char buf[64];
	int len;

	if (*ppos != 0)
		return 0;

	if (adv_tracer_ipc_send_data_polling(EAT_FRM_CHANNEL, &cmd) < 0) {
		pr_err("sending GET_NMI_INFO cmd failed\n");
		return -EIO;
	}

	len = scnprintf(buf, sizeof(buf), "prev NMI:\t%x\ncurr NMI:\t%x\n",
		       cmd.buffer[2], cmd.buffer[1]);

	return simple_read_from_buffer(ubuf, count, ppos, buf, len);
}

static ssize_t dbgcore_irq_info_read(struct file *file, char __user *ubuf,
				     size_t count, loff_t *ppos)
{
	struct adv_tracer_ipc_cmd cmd = {
		.cmd_raw = {
			.cmd = EAT_IPC_CMD_GET_IRQ_INFO,
			.size = 2,
		},
	};
	char buf[64];
	int len;

	if (*ppos != 0)
		return 0;

	if (adv_tracer_ipc_send_data_polling(EAT_FRM_CHANNEL, &cmd) < 0) {
		pr_err("sending GET_IRQ_INFO cmd failed\n");
		return -EIO;
	}

	len = scnprintf(buf, sizeof(buf), "IRQ count:\t%u\nhandled IRQ:\t%#x\n",
		       cmd.buffer[2], cmd.buffer[3]);

	return simple_read_from_buffer(ubuf, count, ppos, buf, len);
}

static ssize_t dbgcore_irq_info_write(struct file *file, const char __user *buf,
				      size_t count, loff_t *ppos)
{
	struct adv_tracer_ipc_cmd cmd = {
		.cmd_raw = {
			.cmd = EAT_IPC_CMD_GET_IRQ_INFO,
			.size = 2,
		},
	};
	cmd.buffer[1] = 1;

	if (*ppos != 0)
		return -EINVAL;

	if (adv_tracer_ipc_send_data_polling(EAT_FRM_CHANNEL, &cmd) < 0) {
		pr_err("sending GET_IRQ_INFO cmd failed\n");
		return -EIO;
	}

	*ppos = count;
	return count;
}

static ssize_t dbgcore_apm_ping_read(struct file *file, char __user *ubuf,
				     size_t count, loff_t *ppos)
{
	struct adv_tracer_ipc_cmd cmd = {
		.cmd_raw = {
			.cmd = EAT_IPC_CMD_APM_PING_TEST,
			.size = 1,
		},
	};
	char buf[64];	/* Sufficient for printout w/ longest status message */
	int len;
	const char *status_msg;

	if (*ppos != 0)
		return 0;

	if (adv_tracer_ipc_send_data_polling(EAT_FRM_CHANNEL, &cmd) < 0) {
		pr_err("sending APM_PING_TEST cmd failed\n");
		return -EIO;
	}

	switch (cmd.buffer[1]) {
	case 0:
		status_msg = "Pass";
		break;
	case ENXIO:
		status_msg = "Mailbox not supported";
		break;
	case EINVAL:
		status_msg = "Bad response from APM";
		break;
	case EBUSY:
		status_msg = "No response from APM";
		break;
	default:
		status_msg = "<unknown status>";
		break;
	}

	len = scnprintf(buf, sizeof(buf), "APM-SWD mailbox Ping result: %s\n",
		       status_msg);

	return simple_read_from_buffer(ubuf, count, ppos, buf, len);
}

static ssize_t dbgcore_slc_dump_read(struct file *file, char __user *ubuf,
				     size_t count, loff_t *ppos)
{
	char buf[80];
	int len;

	if (*ppos != 0)
		return 0;

	len = scnprintf(buf, sizeof(buf),
			"Debug boot SLC dump base: 0x%08X\n"
			" Pre-reset SLC dump base: 0x%08X\n",
			dbg_snapshot_get_slcdump_base(),
			dbg_snapshot_get_pre_slcdump_base());

	return simple_read_from_buffer(ubuf, count, ppos, buf, len);
}

extern void pull_down_other_cpus(void);

static ssize_t dbgcore_slc_dump_write(struct file *file, const char __user *ubuf,
				     size_t count, loff_t *ppos)
{
	struct adv_tracer_ipc_cmd cmd = {
		.cmd_raw = {
			.cmd = EAT_IPC_CMD_SLC_DUMP,
			.size = 1,
		},
	};

	if (*ppos != 0)
		return 0;

	if (!dbg_snapshot_get_pre_slcdump_base()) {
		pr_err("Pre-reset SLC dump not enabled\n");
		return -EIO;
	}
	pr_info("Pre-reset SLC dump initiated\n");

	flush_cache_all();
	pull_down_other_cpus();
	adv_tracer_ipc_send_data_async(EAT_FRM_CHANNEL, &cmd);

	pr_info("Pre-reset SLC dump triggered - reboot in a few sec\n");
	local_irq_disable();
	mdelay(30000);	/* If ACPM fails to reboot, the WDT expiry will */
	local_irq_enable();

	/* Should not reach here */

	*ppos = count;
	return count;
}

static int attr_uart_mux_get(void *data, u64 *val)
{
	struct adv_tracer_ipc_cmd cmd = {
		.cmd_raw = {
			.cmd = EAT_IPC_CMD_CONFIG_READ_UART_MUX,
			.size = 2,
		},
	};
	if (adv_tracer_ipc_send_data_polling(EAT_FRM_CHANNEL, &cmd) < 0) {
		pr_err("sending READ_UART_MUX cmd failed\n");
		return -EIO;
	}

	*val = cmd.buffer[1];

	return 0;
}

static int attr_uart_mux_write(void *data, u64 val)
{
	struct adv_tracer_ipc_cmd cmd = {
		.cmd_raw = {
			.cmd = EAT_IPC_CMD_CONFIG_WRITE_UART_MUX,
			.size = 2,
		},
	};
	cmd.buffer[1] = val;

	if (adv_tracer_ipc_send_data_polling(EAT_FRM_CHANNEL, &cmd) < 0) {
		pr_err("sending WRITE_UART_MUX cmd failed\n");
		return -EIO;
	}

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(dbgcore_uart_mux_fops, attr_uart_mux_get,
			attr_uart_mux_write, "%llu\n");

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

static const struct file_operations dbgcore_nmi_info_fops = {
	.open	= simple_open,
	.read	= dbgcore_nmi_info_read,
	.llseek	= default_llseek,
};

static const struct file_operations dbgcore_irq_info_fops = {
	.open	= simple_open,
	.read	= dbgcore_irq_info_read,
	.write	= dbgcore_irq_info_write,
	.llseek	= default_llseek,
};

static const struct file_operations dbgcore_apm_ping_fops = {
	.open	= simple_open,
	.read	= dbgcore_apm_ping_read,
	.llseek	= default_llseek,
};

static const struct file_operations dbgcore_slc_dump_fops = {
	.open	= simple_open,
	.read	= dbgcore_slc_dump_read,
	.write	= dbgcore_slc_dump_write,
	.llseek	= default_llseek,
};

static __init int dbgcore_dump_init(void)
{
	struct dentry *version;
	struct dentry *logdump;
	struct dentry *nmi_info;
	struct dentry *irq_info;
	struct dentry *apm_ping;
	struct dentry *slc_dump;

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

	nmi_info = debugfs_create_file("nmi_info", 0444, dbgcore_dentry, NULL,
				       &dbgcore_nmi_info_fops);
	if (nmi_info == NULL)
		goto out;

	irq_info = debugfs_create_file("irq_info", 0644, dbgcore_dentry, NULL,
				       &dbgcore_irq_info_fops);
	if (irq_info == NULL)
		goto out;

	apm_ping = debugfs_create_file("apm_ping", 0644, dbgcore_dentry, NULL,
				       &dbgcore_apm_ping_fops);
	if (apm_ping == NULL)
		goto out;

	slc_dump = debugfs_create_file("slc_dump", 0644, dbgcore_dentry, NULL,
				       &dbgcore_slc_dump_fops);
	if (slc_dump == NULL)
		goto out;

	irq_info = debugfs_create_file("uart_mux", 0644, dbgcore_dentry, NULL,
				       &dbgcore_uart_mux_fops);
	if (irq_info == NULL)
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
