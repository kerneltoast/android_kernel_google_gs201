// SPDX-License-Identifier: GPL-2.0-only
/* Copyright 2020 Google LLC. All Rights Reserved.
 *
 * Interface to the AoC control service
 */

#define pr_fmt(fmt) "aoc_control: " fmt

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/types.h>

#include "aoc.h"
#include "aoc-interface.h"

#define AOC_CONTROL_NAME "aoc_stats"
#define AOC_SERVICE_NAME "control"
#define STAT_READ_TIMEOUT 1000

static int aoc_control_prepare(struct device *dev);
static void aoc_control_complete(struct device *dev);

struct aoc_stat {
	char name[8];
	u8 type;
};

struct stats_prvdata {
	/* Exclusive access to the channel for attribute transactions */
	struct mutex lock;
	struct work_struct discovery_work;
	struct aoc_service_dev *service;
	struct aoc_stat *discovered_stats;
	int total_stats;
	long service_timeout;
	u8 memory_vote_core_id;
};

const static struct dev_pm_ops aoc_control_pm_ops = {
	.prepare = aoc_control_prepare,
	.complete = aoc_control_complete,
};

static bool notify_on_state_transition;

/* Driver methods */

/*
 * Convenience method to send a write/read combination to the AoC.
 * Returns # of bytes returned, or negative codes for errors.
 */
static ssize_t read_attribute(struct stats_prvdata *prvdata, void *in_cmd,
			      size_t in_size, void *out_cmd, size_t out_size)
{
	struct aoc_service_dev *service = prvdata->service;
	ssize_t ret;

	ret = mutex_lock_interruptible(&prvdata->lock);
	if (ret != 0)
		return ret;

	if (aoc_service_flush_read_data(service))
		pr_err("Previous response left in channel\n");

	ret = aoc_service_write_timeout(service, in_cmd, in_size, prvdata->service_timeout);
	if (ret < 0)
		goto out;

	ret = aoc_service_read_timeout(service, out_cmd, out_size, prvdata->service_timeout);

out:
	mutex_unlock(&prvdata->lock);
	return ret;
}

static const char * const service_names[] = {
	AOC_SERVICE_NAME,
	NULL,
};

static ssize_t read_core_build_info(int core, struct device *dev, char *buf)
{
	struct stats_prvdata *prvdata = dev_get_drvdata(dev);
	struct CMD_SYS_VERSION_GET version_get = { 0 };
	int ret;

	AocCmdHdrSet(&version_get.parent.parent, CMD_SYS_VERSION_GET_ID,
		     sizeof(version_get));

	version_get.parent.core = core;

	ret = read_attribute(prvdata, &version_get, sizeof(version_get),
			     &version_get, sizeof(version_get));

	if (ret < 0)
		return ret;

	if (strnlen(version_get.version, sizeof(version_get.version)) ==
	    sizeof(version_get.version))
		dev_err(dev, "invalid version string returned\n");

	if (strnlen(version_get.link_time, sizeof(version_get.link_time)) ==
	    sizeof(version_get.link_time))
		dev_err(dev, "invalid link time string returned\n");

	ret = scnprintf(buf, PAGE_SIZE, "Build Hash: %.64s\nLink Time: %.64s\n",
			version_get.version, version_get.link_time);

	return ret;
}

static ssize_t a32_build_info_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return read_core_build_info(1, dev, buf);
}

static DEVICE_ATTR_RO(a32_build_info);

static ssize_t f1_build_info_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return read_core_build_info(2, dev, buf);
}

static DEVICE_ATTR_RO(f1_build_info);

static ssize_t hf_build_info_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return read_core_build_info(3, dev, buf);
}

static DEVICE_ATTR_RO(hf_build_info);

static ssize_t memory_exception_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct stats_prvdata *prvdata = dev_get_drvdata(dev);
	struct CMD_GET_MEMORY_EXCEPTION_DATA_COUNT get_count_cmd;
	struct CMD_GET_MEMORY_EXCEPTION_DATA get_data_cmd;
	u32 total_count;
	u32 i;
	u32 bytes_written;
	int ret;

	AocCmdHdrSet(&get_count_cmd.parent, CMD_GET_MEMORY_EXCEPTION_DATA_COUNT_ID,
		     sizeof(get_count_cmd));

	ret = read_attribute(prvdata, &get_count_cmd, sizeof(get_count_cmd),
			     &get_count_cmd, sizeof(get_count_cmd));

	if (ret < 0)
		return ret;

	total_count = get_count_cmd.num_memory_exception;
	if (total_count > 16) {
		total_count = 16;
	}
	bytes_written = 0;
	for (i = 0; i < total_count; i++) {
		AocCmdHdrSet(&get_data_cmd.parent, CMD_GET_MEMORY_EXCEPTION_DATA_ID,
			     sizeof(get_data_cmd));
		get_data_cmd.log_index = i;
		get_data_cmd.valid = false;
		ret = read_attribute(prvdata, &get_data_cmd, sizeof(get_data_cmd),
				     &get_data_cmd, sizeof(get_data_cmd));
		if (ret < 0)
			return ret;

		if (get_data_cmd.valid) {
			bytes_written += scnprintf(buf + bytes_written, PAGE_SIZE - bytes_written,
					    "index = %u, id = %u, return_address = 0x%x, fault_address = 0x%x, task_name = %s\n",
					    i, get_data_cmd.id, get_data_cmd.return_address,
					    get_data_cmd.fault_address, get_data_cmd.task_name);
		}
	}

	return bytes_written;
}

static DEVICE_ATTR_RO(memory_exception);

static ssize_t memory_votes_stats(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct stats_prvdata *prvdata = dev_get_drvdata(dev);
	struct CMD_GET_MEMORY_VOTES_DATA_COUNT get_count_cmd;
	struct CMD_GET_MEMORY_VOTES_DATA get_data_cmd;
	u32 total_count;
	u32 i;
	u32 bytes_written;
	int ret;
	const char* core_names[] = {"A32", "FF1", "HF0", "HF1"};

	if (prvdata->memory_vote_core_id < 1 ||
		prvdata->memory_vote_core_id > 4)
		return -EINVAL;

	AocCmdHdrSet(&get_count_cmd.parent, CMD_GET_MEMORY_VOTES_DATA_COUNT_ID,
		sizeof(get_count_cmd));

	ret = read_attribute(prvdata, &get_count_cmd, sizeof(get_count_cmd),
		&get_count_cmd, sizeof(get_count_cmd));

	if (ret < 0)
		return ret;

	total_count = get_count_cmd.num_of_clients;

	bytes_written = 0;

	bytes_written += scnprintf(buf + bytes_written, PAGE_SIZE - bytes_written,
				"\nCore %s:\n", core_names[prvdata->memory_vote_core_id - 1]);
	for (i = 0; i < total_count; i++) {
		AocCmdHdrSet(&get_data_cmd.parent, CMD_GET_MEMORY_VOTES_DATA_ID, sizeof(get_data_cmd));
		get_data_cmd.app_id = i;
		get_data_cmd.valid = false;
		get_data_cmd.core_id = prvdata->memory_vote_core_id;

		ret = read_attribute(prvdata, &get_data_cmd, sizeof(get_data_cmd),
			&get_data_cmd, sizeof(get_data_cmd));

		if (ret < 0)
			return ret;

		if (get_data_cmd.valid) {
			bytes_written += scnprintf(buf + bytes_written, PAGE_SIZE - bytes_written,
				"App %2hhu, votes Curr/Tot/ON %5u / %5u / %5u Last %10llu us, Dur %10llu us\n",
				get_data_cmd.app_id,
				get_data_cmd.votes,
				get_data_cmd.total_votes,
				get_data_cmd.on_votes,
				get_data_cmd.last_on_time_ns / 1000ULL,
				get_data_cmd.total_time_us);
		}
	}

	return bytes_written;
}

#define DECLARE_MEMORY_VOTES(memory_votes_name, id)                            \
	static ssize_t memory_votes_name##_show(                               \
		struct device *dev, struct device_attribute *attr, char *buf)  \
	{                                                                      \
		struct stats_prvdata *prvdata = dev_get_drvdata(dev);          \
		prvdata->memory_vote_core_id = id;                             \
		return memory_votes_stats(dev, attr, buf);                     \
	}                                                                      \
	static DEVICE_ATTR_RO(memory_votes_name)

DECLARE_MEMORY_VOTES(memory_votes_a32, 1);
DECLARE_MEMORY_VOTES(memory_votes_ff1, 2);

/* Driver methods */

/*
 * Convenience method to send a write to the AoC.
 * Returns negative codes for errors.
 */
static ssize_t write_attribute(struct stats_prvdata *prvdata, void *in_cmd,
			      size_t in_size)
{
	struct aoc_service_dev *service = prvdata->service;
	ssize_t ret;

	ret = mutex_lock_interruptible(&prvdata->lock);
	if (ret != 0)
		return ret;

	if (aoc_service_flush_read_data(service))
		pr_err("Previous response left in channel\n");

	ret = aoc_service_write_timeout(service, in_cmd, in_size, prvdata->service_timeout);

	mutex_unlock(&prvdata->lock);
	return ret;
}

static ssize_t udfps_set_clock_source_store(struct device *dev,
				     struct device_attribute *attr, const char *buf, size_t count)
{
	struct stats_prvdata *prvdata = dev_get_drvdata(dev);
	struct CMD_UDFPS_SET_CLOCK_SOURCE clock_src = { 0 };
	uint8_t type;
	int ret;

	AocCmdHdrSet(&clock_src.parent, CMD_UDFPS_SET_CLOCK_SOURCE_ID,
		     sizeof(clock_src));
	ret = kstrtou8(buf, 10, &type);
	if (ret < 0)
		return ret;
	if (type < SOURCE_TOT)
		clock_src.clock_source = type;
        else
		dev_err(dev, "Invalid input parameter = %d\n", type);

	ret = write_attribute(prvdata, &clock_src, sizeof(clock_src));

	if (ret < 0) {
		dev_err(dev, "udfps freq start ret = %d\n", ret);
        }

	return ret;
}

static DEVICE_ATTR_WO(udfps_set_clock_source);

static ssize_t udfps_get_clock_frequency(uint8_t clk_src, struct device *dev, char *buf)
{
	struct stats_prvdata *prvdata = dev_get_drvdata(dev);
	struct CMD_UDFPS_GET_CLOCK_FREQUENCY get_freq_cmd = { 0 };
	int ret;

	AocCmdHdrSet(&get_freq_cmd.parent, CMD_UDFPS_GET_CLOCK_FREQUENCY_ID,
		     sizeof(get_freq_cmd));

	get_freq_cmd.clock_source = clk_src;
	ret = read_attribute(prvdata, &get_freq_cmd, sizeof(get_freq_cmd),
			     &get_freq_cmd, sizeof(get_freq_cmd));

	if (ret < 0)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "%u\n", get_freq_cmd.clock_freq_in_u32);
}

static ssize_t udfps_get_osc_freq_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	return udfps_get_clock_frequency(SOURCE_OSC, dev, buf);
}

static DEVICE_ATTR_RO(udfps_get_osc_freq);

static ssize_t udfps_get_disp_freq_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	return udfps_get_clock_frequency(SOURCE_DISP, dev, buf);
}

static DEVICE_ATTR_RO(udfps_get_disp_freq);

static ssize_t read_timed_stat(struct device *dev, char *buf, int index)
{
	struct stats_prvdata *prvdata = dev_get_drvdata(dev);
	struct CMD_SYS_STATS_TIMED_GET get_stat = { 0 };
	int ret;

	AocCmdHdrSet(&get_stat.parent.parent, CMD_SYS_STATS_TIMED_GET_ID,
		     sizeof(get_stat));
	get_stat.index = index;

	ret = read_attribute(prvdata, &get_stat, sizeof(get_stat), &get_stat,
			     sizeof(get_stat));

	/* Partial response */
	if (ret >= 0 && ret < sizeof(get_stat))
		ret = -EIO;

	if (ret < 0)
		goto out;

	ret = scnprintf(buf, PAGE_SIZE,
		"Counter: %llu\nCumulative time: %llu\nTime last entered: %llu\nTime last exited: %llu\n",
		le64_to_cpu(get_stat.timed.counter),
		le64_to_cpu(get_stat.timed.cumulative_time),
		le64_to_cpu(get_stat.timed.timestamp_enter_last),
		le64_to_cpu(get_stat.timed.timestamp_exit_last));

out:
	return ret;
}

static ssize_t read_data_stat(struct device *dev, char *buf, int index)
{
	struct stats_prvdata *prvdata = dev_get_drvdata(dev);
	struct CMD_SYS_STATS_DATA_GET get_stat = { 0 };
	int ret;

	AocCmdHdrSet(&get_stat.parent.parent, CMD_SYS_STATS_DATA_GET_ID,
		     sizeof(get_stat));
	get_stat.index = index;

	ret = read_attribute(prvdata, &get_stat, sizeof(get_stat), &get_stat,
			     sizeof(get_stat));

	/* Partial response */
	if (ret >= 0 && ret < sizeof(get_stat))
		ret = -EIO;

	if (ret < 0)
		goto out;

	ret = scnprintf(buf, PAGE_SIZE,
			"Counter: %llu\nFailed: %llu\nTx: %llu\nRx: %llu\n",
			le64_to_cpu(get_stat.transfer.counter),
			le64_to_cpu(get_stat.transfer.counter_failed),
			le64_to_cpu(get_stat.transfer.transfer_tx),
			le64_to_cpu(get_stat.transfer.transfer_rx));

out:
	return ret;
}

static ssize_t read_stat_by_name(struct device *dev, char *buf,
				 const char *name)
{
	struct stats_prvdata *prvdata = dev_get_drvdata(dev);
	int i;

	if (!prvdata)
		return 0;

	for (i = 0; i < prvdata->total_stats; i++) {
		if (strcmp(name, prvdata->discovered_stats[i].name))
			continue;

		switch (prvdata->discovered_stats[i].type) {
		case 0:
			dev_dbg(dev, "Reading timed stat %d (%s)\n", i, name);
			return read_timed_stat(dev, buf, i);
		case 1:
			dev_dbg(dev, "Reading transfer stat %d (%s)\n", i,
				name);
			return read_data_stat(dev, buf, i);
		default:
			dev_err(dev, "Unknown stats type %d\n",
				prvdata->discovered_stats[i].type);
			return 0;
		}
	}

	dev_err(dev, "failed to find stat with name %s\n", name);
	return 0;
}

static int aoc_control_prepare(struct device *dev)
{
	if (notify_on_state_transition) {
				struct stats_prvdata *prvdata = dev_get_drvdata(dev);
		struct CMD_AP_STATE_TRANSITION cmd;
		int ret;

		AocCmdNoAckHdrSet(&cmd.parent, CMD_AP_STATE_TRANSITION_ID, sizeof(cmd));
		cmd.transition = 0;

		ret = write_attribute(prvdata, &cmd, sizeof(cmd));
		if (ret < 0)
			dev_err(dev, "notifying AoC of entering deep sleep ret = %d\n", ret);

		return ret;
	}

	return 0;
}

static void aoc_control_complete(struct device *dev)
{
	if (notify_on_state_transition) {
		struct stats_prvdata *prvdata = dev_get_drvdata(dev);
		struct CMD_AP_STATE_TRANSITION cmd;
		int ret;

		AocCmdNoAckHdrSet(&cmd.parent, CMD_AP_STATE_TRANSITION_ID, sizeof(cmd));
		cmd.transition = 1;

		ret = write_attribute(prvdata, &cmd, sizeof(cmd));

		if (ret < 0)
			dev_err(dev, "notifying AoC of exiting deep sleep ret = %d\n", ret);
	}
}

#define DECLARE_STAT(stat_name, sysfs_name)                                    \
	static ssize_t sysfs_name##_show(                                      \
		struct device *dev, struct device_attribute *attr, char *buf)  \
	{                                                                      \
		return read_stat_by_name(dev, buf, stat_name);                 \
	}                                                                      \
	static DEVICE_ATTR_RO(sysfs_name)

DECLARE_STAT("A3-WFI", a32_wfi);
DECLARE_STAT("A3-RET", a32_retention);
DECLARE_STAT("A3-DWN", a32_off);
DECLARE_STAT("F1-WFI", ff1_wfi);
DECLARE_STAT("F1-RET", ff1_retention);
DECLARE_STAT("F1-DWN", ff1_off);
DECLARE_STAT("H0-WFI", hf0_wfi);
DECLARE_STAT("H0-RET", hf0_retention);
DECLARE_STAT("H0-DWN", hf0_off);
DECLARE_STAT("H1-WFI", hf1_wfi);
DECLARE_STAT("H1-RET", hf1_retention);
DECLARE_STAT("H1-DWN", hf1_off);
DECLARE_STAT("AC-MON", monitor_mode);
DECLARE_STAT("I2C0", i2c0);
DECLARE_STAT("I2C1", i2c1);
DECLARE_STAT("I2C2", i2c2);
DECLARE_STAT("I2C3", i2c3);
DECLARE_STAT("I2C4", i2c4);
DECLARE_STAT("SPI0", spi0);
DECLARE_STAT("SPI1", spi1);
DECLARE_STAT("I3C0", i3c0);
DECLARE_STAT("PDM", pdm);
DECLARE_STAT("PDM-SW", pdm_16khz);
DECLARE_STAT("TDM", tdm);
DECLARE_STAT("MIF_UP", mif_up);
DECLARE_STAT("SLC_UP", slc_up);
DECLARE_STAT("SLEEP", sleep);
DECLARE_STAT("SICD", sicd);
DECLARE_STAT("V_NOM", voltage_nominal);
DECLARE_STAT("V_UD", voltage_underdrive);
DECLARE_STAT("V_SUD", voltage_super_underdrive);
DECLARE_STAT("V_UUD", voltage_ultra_underdrive);
DECLARE_STAT("RING_W", ring_buffer_wakeup);
DECLARE_STAT("HOST_W", host_ipc_wakeup);
DECLARE_STAT("USF_W", usf_wakeup);
DECLARE_STAT("AUD_W", audio_wakeup);
DECLARE_STAT("LOG_W", logging_wakeup);
DECLARE_STAT("WORD_W", hotword_wakeup);

static struct attribute *aoc_stats_attrs[] = {
	&dev_attr_a32_build_info.attr,
	&dev_attr_f1_build_info.attr,
	&dev_attr_hf_build_info.attr,
	&dev_attr_a32_wfi.attr,
	&dev_attr_a32_retention.attr,
	&dev_attr_a32_off.attr,
	&dev_attr_ff1_wfi.attr,
	&dev_attr_ff1_retention.attr,
	&dev_attr_ff1_off.attr,
	&dev_attr_hf0_wfi.attr,
	&dev_attr_hf0_retention.attr,
	&dev_attr_hf0_off.attr,
	&dev_attr_hf1_wfi.attr,
	&dev_attr_hf1_retention.attr,
	&dev_attr_hf1_off.attr,
	&dev_attr_monitor_mode.attr,
	&dev_attr_i2c0.attr,
	&dev_attr_i2c1.attr,
	&dev_attr_i2c2.attr,
	&dev_attr_i2c3.attr,
	&dev_attr_i2c4.attr,
	&dev_attr_spi0.attr,
	&dev_attr_spi1.attr,
	&dev_attr_i3c0.attr,
	&dev_attr_pdm.attr,
	&dev_attr_pdm_16khz.attr,
	&dev_attr_tdm.attr,
	&dev_attr_mif_up.attr,
	&dev_attr_slc_up.attr,
	&dev_attr_sleep.attr,
	&dev_attr_sicd.attr,
	&dev_attr_voltage_nominal.attr,
	&dev_attr_voltage_underdrive.attr,
	&dev_attr_voltage_super_underdrive.attr,
	&dev_attr_voltage_ultra_underdrive.attr,
	&dev_attr_ring_buffer_wakeup.attr,
	&dev_attr_host_ipc_wakeup.attr,
	&dev_attr_usf_wakeup.attr,
	&dev_attr_audio_wakeup.attr,
	&dev_attr_logging_wakeup.attr,
	&dev_attr_hotword_wakeup.attr,
	&dev_attr_memory_exception.attr,
	&dev_attr_memory_votes_a32.attr,
	&dev_attr_memory_votes_ff1.attr,
	NULL
};

ATTRIBUTE_GROUPS(aoc_stats);

static struct attribute *aoc_control_attrs[] = {
	&dev_attr_udfps_set_clock_source.attr,
	&dev_attr_udfps_get_osc_freq.attr,
	&dev_attr_udfps_get_disp_freq.attr,
	NULL
};

ATTRIBUTE_GROUPS(aoc_control);

static int aoc_control_map_handler(u32 handle, phys_addr_t p, size_t size,
				    bool mapped, void *ctx)
{
	struct device *dev = ctx;
	struct stats_prvdata *prvdata = dev_get_drvdata(dev);
	struct CMD_MEM_MAP_REGION mem_map_region = { 0 };
	int ret;

	AocCmdHdrSet(&mem_map_region.parent.parent, CMD_MEM_MAP_REGION_ID,
		     sizeof(mem_map_region));

	mem_map_region.handle = handle;
	mem_map_region.base = (u32)p;
	mem_map_region.size = size;
	mem_map_region.mapped = mapped;

	ret = read_attribute(prvdata, &mem_map_region, sizeof(mem_map_region),
			     &mem_map_region, sizeof(mem_map_region));

	if (ret < 0)
		pr_notice("CMD_MEM_MAP_REGION_ID returned %d\n", ret);

	return ret;
}

static void discovery_workitem(struct work_struct *work)
{
	struct stats_prvdata *prvdata =
		container_of(work, struct stats_prvdata, discovery_work);
	struct CMD_SYS_STATS_TOT cmd_total;
	struct device *dev = &prvdata->service->dev;
	struct aoc_stat *stats;
	int i, ret;

	dev_dbg(dev, "started the discovery thread\n");

	AocCmdHdrSet(&cmd_total.parent.parent, CMD_SYS_STATS_TOT_ID,
		     sizeof(cmd_total));

	ret = read_attribute(prvdata, &cmd_total, sizeof(cmd_total), &cmd_total,
			     sizeof(cmd_total));

	if (ret < sizeof(cmd_total)) {
		dev_err(dev, "failed to read the number of statistics: %d\n", ret);
		goto out;
	}

	if (cmd_total.tot > 256 || cmd_total.tot < 0) {
		dev_err(dev, "invalid number of statistics %d\n",
			cmd_total.tot);
		goto out;
	}

	stats = devm_kcalloc(dev, cmd_total.tot, sizeof(struct aoc_stat),
			     GFP_KERNEL | __GFP_ZERO);
	if (!stats)
		goto out;

	prvdata->total_stats = cmd_total.tot;
	dev_dbg(dev, "Returned %d, %d stats", ret, prvdata->total_stats);

	for (i = 0; i < prvdata->total_stats; i++) {
		struct CMD_SYS_STATS_INFO_GET cmd_info;

		memset(&cmd_info, 0, sizeof(cmd_info));
		AocCmdHdrSet(&cmd_info.parent.parent, CMD_SYS_STATS_INFO_GET_ID,
			     sizeof(cmd_info));
		cmd_info.index = i;

		ret = read_attribute(prvdata, &cmd_info, sizeof(cmd_info),
				     &cmd_info, sizeof(cmd_info));
		if (ret == sizeof(cmd_info)) {
			stats[i].type = cmd_info.info.type;
			memcpy(stats[i].name, cmd_info.info.name,
			       STATS_ENTRY_LEN);
			dev_dbg(dev, "stat %d name %s type %d\n", i,
				stats[i].name, stats[i].type);
		} else {
			dev_err(dev, "failed to read stat %d info: %d\n", i,
				ret);
		}
	}

	prvdata->discovered_stats = stats;
	ret = device_add_groups(&prvdata->service->dev, aoc_stats_groups);

	aoc_set_map_handler(prvdata->service, aoc_control_map_handler, dev);
out:
	return;
}

static int aoc_control_probe(struct aoc_service_dev *sd)
{
	struct device *dev = &sd->dev;
	struct stats_prvdata *prvdata;
	int ret;

	pr_debug("probe service with name %s\n", dev_name(dev));

	prvdata = devm_kzalloc(dev, sizeof(*prvdata), GFP_KERNEL);
	prvdata->service = sd;
	prvdata->service_timeout = msecs_to_jiffies(STAT_READ_TIMEOUT);
	mutex_init(&prvdata->lock);
	prvdata->memory_vote_core_id = 1;

	notify_on_state_transition = of_property_read_bool(dev->parent->of_node,
							"notify-on-state-transition");

	INIT_WORK(&prvdata->discovery_work, discovery_workitem);
	dev_set_drvdata(dev, prvdata);

	ret = device_add_groups(dev, aoc_control_groups);
	if (ret)
		dev_err(dev, "Failed to add device groups\n");

	schedule_work(&prvdata->discovery_work);

	return 0;
}

static int aoc_control_remove(struct aoc_service_dev *sd)
{
	struct device *dev = &sd->dev;
	struct stats_prvdata *prvdata = dev_get_drvdata(dev);

	pr_debug("remove service with name %s\n", dev_name(dev));

	cancel_work_sync(&prvdata->discovery_work);

	device_remove_groups(dev, aoc_stats_groups);
	device_remove_groups(dev, aoc_control_groups);

	aoc_remove_map_handler(prvdata->service);

	return 0;
}

static struct aoc_driver aoc_control_driver = {
	.drv = {
			.name = AOC_CONTROL_NAME,
			.pm = &aoc_control_pm_ops,
		},
	.service_names = service_names,
	.probe = aoc_control_probe,
	.remove = aoc_control_remove,
};

module_aoc_driver(aoc_control_driver);

MODULE_LICENSE("GPL v2");
