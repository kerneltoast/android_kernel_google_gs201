// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License Version 2 as published
 * by the Free Software Foundation.
 *
 * BTS Bus Traffic Shaper device driver
 *
 */

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/list.h>
#include <linux/debugfs.h>
#include <linux/syscore_ops.h>
#include <linux/suspend.h>
#include <soc/google/cal-if.h>
#include <dt-bindings/soc/google/gs-bts.h>
#include <trace/events/power.h>
#if IS_ENABLED(CONFIG_EXYNOS_PM_QOS)
#include <soc/google/exynos_pm_qos.h>
#else
#include <linux/pm_qos.h>
#endif


#include "bts.h"

#define BTS_PDEV_NAME "exynos-bts"
#define ID_DEFAULT 0

#define BTSDBG_LOG(x...)                                                       \
	do {                                                                   \
		if (btsdbg_log)                                                \
			dev_notice(x);                                         \
	} while (0)

static bool btsdbg_log;
static unsigned int vc_num_total;

#if IS_ENABLED(CONFIG_EXYNOS_PM_QOS)
static struct exynos_pm_qos_request exynos_mif_qos;
static struct exynos_pm_qos_request exynos_int_qos;
#else
static struct pm_qos_request exynos_mif_qos;
static struct pm_qos_request exynos_int_qos;
#endif

static struct bts_device *btsdev;

static unsigned int bus1_to_int_freq(unsigned int freq)
{
	unsigned int i;

	if (!btsdev->bus1_int_tbl) {
		dev_err(btsdev->dev, "bus1_int_tbl unavailable\n");
		return freq;
	}

	/* The bus1_int_tbl stores frequencies from high to low */
	i = btsdev->map_row_cnt;
	while (--i > 0) {
		if (freq <= btsdev->bus1_int_tbl[i].bus1_freq)
			break;
	}

	return btsdev->bus1_int_tbl[i].int_freq;
}

#if IS_ENABLED(CONFIG_SOC_ZUMA)
static inline unsigned int bus1_freq_to_mhz(unsigned int freq)
{
	unsigned int i;

	if (!btsdev->bus1_int_tbl) {
		dev_err(btsdev->dev, "bus1_int_tbl unavailable\n");
		return freq;
	}

	i = btsdev->map_row_cnt;
	while (--i > 0) {
		if (freq <= btsdev->bus1_int_tbl[i].bus1_freq)
			break;
	}

	return btsdev->bus1_int_tbl[i].bus1_freq / 1000;
}

static unsigned int bts_cal_urgent_lat_threshold(unsigned int bus1_mhz, unsigned int latency_ns)
{
	return latency_ns * bus1_mhz / URGENT_TIMEOUT_EPOCH / 1000;
}

static void bts_update_urgent_lat_req(unsigned int bus1_freq)
{
	struct bts_info *info = btsdev->bts_list;
	struct bts_urgent_lat *urgent_lat = btsdev->urgent_lat_list;
	unsigned int i, scen, threshold, bts_index;
	struct bts_stat stat;
	char trace_name[32];

	if (!bus1_freq || !btsdev->num_urgent_lat)
		return;

	bus1_freq = bus1_freq_to_mhz(bus1_freq);

	spin_lock(&btsdev->lock);

	scen = btsdev->top_scen;
	for (i = 0; i < btsdev->num_urgent_lat; i++) {
		if (urgent_lat[i].lat_read) {
			bts_index = urgent_lat[i].bts_index;
			if (!info[bts_index].pd_on || !info[bts_index].ops->get_urgent ||
			    !info[bts_index].ops->set_urgent)
				continue;
			threshold = bts_cal_urgent_lat_threshold(bus1_freq, urgent_lat[i].lat_read);
			if (!info[bts_index].stat[scen].stat_on)
				scen = ID_DEFAULT;
			if (threshold > info[bts_index].stat[scen].qurgent_th_r || !threshold)
				threshold = info[bts_index].stat[scen].qurgent_th_r;
			if (threshold == urgent_lat[i].th_read)
				continue;

			urgent_lat[i].th_read = threshold;
			BTSDBG_LOG(btsdev->dev,
				   "Update %s latency: bus1 %uMhz, lat %u ns, th 0x%x\n",
				   urgent_lat[i].name, bus1_freq, urgent_lat[i].lat_read,
				   threshold);

			info[bts_index].ops->get_urgent(info[bts_index].va_base, &stat);
			if (stat.qurgent_th_r != threshold) {
				stat.qurgent_th_r = threshold;
				info[bts_index].ops->set_urgent(info[bts_index].va_base, &stat);
				if (trace_clock_set_rate_enabled()) {
					scnprintf(trace_name, sizeof(trace_name), "BTS_%s_ur_th_rd",
						  urgent_lat[i].name);
					trace_clock_set_rate(trace_name, threshold,
							     raw_smp_processor_id());
				}
			}
		}
	}

	spin_unlock(&btsdev->lock);
}
#endif

static void bts_calc_bw(void)
{
	unsigned int i;
	unsigned int total_read = 0;
	unsigned int total_write = 0;
	unsigned int rt_bw = 0;
	unsigned int mif_freq, int_freq, bus1_freq = 0;
#if IS_ENABLED(CONFIG_SOC_ZUMA)
	struct bts_bw *bw;
	unsigned int nocl_total_rd_bw;
	unsigned int nocl_total_wr_bw;
	unsigned int nocl_peak_freq;
	char buf[80];
	ssize_t ret = 0;
#endif

	mutex_lock(&btsdev->mutex_lock);

	btsdev->peak_bw = 0;
	btsdev->total_bw = 0;

	spin_lock(&btsdev->lock);
	for (i = 0; i < btsdev->num_bts; i++) {
#if !IS_ENABLED(CONFIG_SOC_ZUMA)
		if (btsdev->peak_bw < btsdev->bts_bw[i].peak)
			btsdev->peak_bw = btsdev->bts_bw[i].peak;
#endif
		/* Calculate total RT BW based on RT clients */
		if (btsdev->bts_bw[i].is_rt)
			rt_bw += btsdev->bts_bw[i].rt;

		total_read += btsdev->bts_bw[i].read;
		total_write += btsdev->bts_bw[i].write;
	}
	spin_unlock(&btsdev->lock);
	btsdev->total_bw = total_read + total_write;
	if (btsdev->peak_bw < (total_read / NUM_CHANNEL))
		btsdev->peak_bw = (total_read / NUM_CHANNEL);
	if (btsdev->peak_bw < (total_write / NUM_CHANNEL))
		btsdev->peak_bw = (total_write / NUM_CHANNEL);

	mif_freq = (btsdev->total_bw / BUS_WIDTH) * 100 / MIF_UTIL;
	/* Additional MIF constriant to guarantee RT BW < 40% */
	mif_freq = max(mif_freq, (rt_bw / BUS_WIDTH) * 100 / RT_UTIL);

#if IS_ENABLED(CONFIG_SOC_ZUMA)
	for (i = 0; i < btsdev->num_nocl ; i++) {
		nocl_total_rd_bw = 0;
		nocl_total_wr_bw = 0;
		btsdev->nocl_infos[i].peak_freq = 0;
		list_for_each_entry(bw, &btsdev->nocl_infos[i].list, node) {
			nocl_total_rd_bw += bw->read;
			nocl_total_wr_bw += bw->write;
			nocl_peak_freq = (bw->peak / bw->bus_width) * 100 / INT_UTIL;
			if (btsdev->nocl_infos[i].peak_freq < nocl_peak_freq)
				btsdev->nocl_infos[i].peak_freq = nocl_peak_freq;
		}
		if (!strcmp(btsdev->nocl_infos[i].nocl_name, "nocl2aa") ||
		    !strcmp(btsdev->nocl_infos[i].nocl_name, "nocl2ab")) {
			/* In Zuma, the equivalent of bus1 is NOCL2AA & NOCL2AB
			 * so first calculate the required frequency for these
			 * two nocls to get the bus1_freq.
			 */
			nocl_peak_freq = (nocl_total_rd_bw / INT_BUS_WIDTH) /
				NOCL2A_NUM_CHANNEL * 100 / INT_UTIL;
			btsdev->nocl_infos[i].peak_freq = max(nocl_peak_freq,
				btsdev->nocl_infos[i].peak_freq);
			nocl_peak_freq = (nocl_total_wr_bw / INT_BUS_WIDTH) /
				NOCL2A_NUM_CHANNEL * 100 / INT_UTIL;
			btsdev->nocl_infos[i].peak_freq = max(nocl_peak_freq,
				btsdev->nocl_infos[i].peak_freq);
			bus1_freq = max(bus1_freq, btsdev->nocl_infos[i].peak_freq);
		} else {
			/* This block is to calculate the required frequency
			 * of NOCL1A.
			 */
			nocl_peak_freq = (total_read / INT_BUS_WIDTH) /
				NUM_CHANNEL * 100 / INT_UTIL;
			btsdev->nocl_infos[i].peak_freq = max(nocl_peak_freq,
				btsdev->nocl_infos[i].peak_freq);
			nocl_peak_freq = (total_write / INT_BUS_WIDTH) /
				NUM_CHANNEL * 100 / INT_UTIL;
			btsdev->nocl_infos[i].peak_freq = max(nocl_peak_freq,
				btsdev->nocl_infos[i].peak_freq);
			int_freq = btsdev->nocl_infos[i].peak_freq;
		}
		ret += scnprintf(buf + ret, sizeof(buf) - ret, "%s:%.8u ",
				 btsdev->nocl_infos[i].nocl_name,
				 btsdev->nocl_infos[i].peak_freq);
	}
	BTSDBG_LOG(btsdev->dev, "Freq: %s\n", buf);

	/* Calculate the final INT frequency based on NOCL2AA, NOCL2AB & NOCL1A */
	int_freq = max(int_freq, bus1_to_int_freq(bus1_freq));
	bts_update_urgent_lat_req(bus1_freq);

	BTSDBG_LOG(btsdev->dev,
		   "BW: T:%.8u R:%.8u W:%.8u P:%.8u RT:%.8u MIF:%.8u NOCL2A:%.8u INT:%.8u\n",
		   btsdev->total_bw, total_read, total_write, btsdev->peak_bw, rt_bw,
		   mif_freq, bus1_freq, int_freq);
#else
	bus1_freq = (btsdev->peak_bw / BUS_WIDTH) * 100 / INT_UTIL;
	int_freq = bus1_to_int_freq(bus1_freq);

	BTSDBG_LOG(btsdev->dev,
		   "BW: T:%.8u R:%.8u W:%.8u P:%.8u RT:%.8u MIF:%.8u BUS1:%.8u INT:%.8u\n",
		   btsdev->total_bw, total_read, total_write, btsdev->peak_bw, rt_bw,
		   mif_freq, bus1_freq, int_freq);
#endif
	trace_clock_set_rate("BTS_mif_freq", mif_freq, raw_smp_processor_id());
	trace_clock_set_rate("BTS_int_freq", int_freq, raw_smp_processor_id());

#if IS_ENABLED(CONFIG_EXYNOS_PM_QOS)
	exynos_pm_qos_update_request(&exynos_mif_qos, mif_freq);
	exynos_pm_qos_update_request(&exynos_int_qos, int_freq);
#else
	pm_qos_update_request(&exynos_mif_qos, mif_freq);
	pm_qos_update_request(&exynos_int_qos, int_freq);
#endif
	mutex_unlock(&btsdev->mutex_lock);
}

static void bts_update_stats(unsigned int index)
{
	struct bts_bw *bw = btsdev->bts_bw;
	int i;
	int total_prev_idx, peak_prev_idx;
	int total_bin_idx, peak_bin_idx;
	unsigned int total_bw, peak_bw;
	u64 curr, duration;

	total_bw = bw[index].read + bw[index].write;
	peak_bw = bw[index].peak;

	for (i = 0; i < BTS_HIST_BIN - 1; i++) {
		if (total_bw < bw_trip[i])
			break;
	}
	total_bin_idx = total_bw ? i : -1;

	for (i = 0; i < BTS_HIST_BIN - 1; i++) {
		if (peak_bw < bw_trip[i])
			break;
	}
	peak_bin_idx = peak_bw ? i : -1;

	curr = ktime_get_ns();
	if (!bw[index].stats.start_time) {
		if (total_bin_idx < 0 && peak_bin_idx < 0)
			return;
		bw[index].stats.start_time = curr;
		bw[index].stats.total.hist_idx = total_bin_idx;
		bw[index].stats.peak.hist_idx = peak_bin_idx;
		return;
	}

	total_prev_idx = bw[index].stats.total.hist_idx;
	peak_prev_idx = bw[index].stats.peak.hist_idx;
	bw[index].stats.total.hist_idx = total_bin_idx;
	bw[index].stats.peak.hist_idx = peak_bin_idx;
	duration = curr - bw[index].stats.start_time;
	bw[index].stats.start_time = curr;
	if (total_prev_idx < 0 && peak_prev_idx < 0)
		return;

	if (total_prev_idx >= 0) {
		bw[index].stats.total.count[total_prev_idx]++;
		bw[index].stats.total.total_time[total_prev_idx] += duration;
	}
	if (peak_prev_idx >= 0) {
		bw[index].stats.peak.count[peak_prev_idx]++;
		bw[index].stats.peak.total_time[peak_prev_idx] += duration;
	}
	return;
}

static void bts_set(unsigned int scen, unsigned int index)
{
	struct bts_info *info = btsdev->bts_list;
#if IS_ENABLED(CONFIG_SOC_ZUMA)
	struct bts_urgent_lat *urgent_lat = btsdev->urgent_lat_list;
	struct bts_stat bts_stat;
	unsigned int i, cur_urgent_th_r;
	bool update_urgent = false;
#endif
	int ret;

	if (!info[index].ops->set_bts || !info[index].pd_on)
		return;

	/* Check scenario set exists */
	if (!info[index].stat[scen].stat_on)
		scen = ID_DEFAULT;

#if IS_ENABLED(CONFIG_SOC_ZUMA)
	for (i = 0; i < btsdev->num_urgent_lat; i++) {
		if (urgent_lat[i].bts_index == index) {
			cur_urgent_th_r = info[index].stat[scen].qurgent_th_r;
			if (urgent_lat[i].th_read > 0 && urgent_lat[i].th_read < cur_urgent_th_r) {
				BTSDBG_LOG(btsdev->dev, "Replace %s urgent: th_read 0x%x -> 0x%x\n",
					   urgent_lat[i].name, cur_urgent_th_r,
					   urgent_lat[i].th_read);
				memcpy(&bts_stat, &info[index].stat[scen],
				       sizeof(info[index].stat[scen]));
				bts_stat.qurgent_th_r = urgent_lat[i].th_read;
				update_urgent = true;
			}
			break;
		}
	}

	if (update_urgent)
		ret = info[index].ops->set_bts(info[index].va_base, &bts_stat);
	else
		ret = info[index].ops->set_bts(info[index].va_base, &info[index].stat[scen]);
#else
	ret = info[index].ops->set_bts(info[index].va_base, &info[index].stat[scen]);
#endif

	if (ret)
		dev_err(btsdev->dev,
			"%s failed! (scenario=%u) (index=%u)\n",
			__func__, scen, index);

}

void bts_pd_sync(unsigned int cal_id, int on)
{
	struct bts_info *info = btsdev->bts_list;
	unsigned int i;

	spin_lock(&btsdev->lock);

	for (i = 0; i < btsdev->num_bts; i++) {
		if (!info[i].pd_id)
			continue;
		if (info[i].pd_id == cal_id) {
			info[i].pd_on = on ? true : false;
			if (on)
				bts_set(btsdev->top_scen, i);
		}
	}

	spin_unlock(&btsdev->lock);
}
EXPORT_SYMBOL_GPL(bts_pd_sync);

int bts_get_bwindex(const char *name)
{
	struct bts_bw *bw = btsdev->bts_bw;
	unsigned int index;
	int ret, i;

	spin_lock(&btsdev->lock);

	for (index = 0; bw[index].name && (index < btsdev->num_bts);
	     index++) {
		if (!strcmp(bw[index].name, name)) {
			ret = index;
			goto out;
		}
	}

	if (index == btsdev->num_bts) {
		ret = -EINVAL;
		goto out;
	}

	bw[index].name = devm_kstrdup(btsdev->dev, name, GFP_ATOMIC);
	if (!bw[index].name) {
		dev_err(btsdev->dev, "failed to allocate bandwidth name\n");
		ret = -ENOMEM;
		goto out;
	}
	ret = index;
	bw[index].is_rt = false;
	for (i = 0; i < btsdev->num_rts; i++) {
		if (!strcmp(bw[index].name, btsdev->rt_names[i]))
			bw[index].is_rt = true;
	}
#if IS_ENABLED(CONFIG_SOC_ZUMA)
	for (i = 0; i < btsdev->num_nocl; i++) {
		int j;
		for (j = 0; j < btsdev->nocl_infos[i].num_ip; j++) {
			if (!strcmp(bw[index].name,
			    btsdev->nocl_infos[i].nocl_ips[j].ip_name)) {
				INIT_LIST_HEAD(&bw[index].node);
				list_add(&bw[index].node, &btsdev->nocl_infos[i].list);
				bw[index].bus_width =
					btsdev->nocl_infos[i].nocl_ips[j].ip_bus_width;
			}
		}
	}
#endif
out:
	spin_unlock(&btsdev->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(bts_get_bwindex);

unsigned int bts_get_scenindex(const char *name)
{
	unsigned int index;
	struct bts_scen *scen = btsdev->scen_list;

	spin_lock(&btsdev->lock);

	for (index = 1; index < btsdev->num_scen; index++)
		if (strcmp(name, scen[index].name) == 0)
			break;

	if (index == btsdev->num_scen) {
		spin_unlock(&btsdev->lock);
		return 0;
	}

	spin_unlock(&btsdev->lock);

	return index;
}
EXPORT_SYMBOL_GPL(bts_get_scenindex);

#if IS_ENABLED(CONFIG_SOC_ZUMA)
int bts_get_urgent_lat_bts_index(const char *name)
{
	unsigned int i, bts_index;
	struct bts_urgent_lat *urgent_lat = btsdev->urgent_lat_list;

	spin_lock(&btsdev->lock);
	for (i = 0; i < btsdev->num_urgent_lat; i++) {
		if (strcmp(name, urgent_lat[i].name) == 0)
			break;
	}

	if (i < btsdev->num_urgent_lat) {
		bts_index = urgent_lat[i].bts_index;
	} else {
		bts_index = -EINVAL;
		dev_err(btsdev->dev, "%s: invalid `%s`", __func__, name);
	}
	spin_unlock(&btsdev->lock);

	return bts_index;
}
EXPORT_SYMBOL_GPL(bts_get_urgent_lat_bts_index);

void bts_set_urgent_lat_read(int bts_index, unsigned int lat_read)
{
	unsigned int i;
	struct bts_urgent_lat *urgent_lat = btsdev->urgent_lat_list;
	char trace_name[32];

	if (bts_index < 0 || bts_index >= btsdev->num_bts) {
		dev_err(btsdev->dev, "%s: Invalid bts index %d!\n", __func__, bts_index);
		return;
	}

	spin_lock(&btsdev->lock);
	for (i = 0; i < btsdev->num_urgent_lat; i++) {
		if (bts_index == urgent_lat[i].bts_index) {
			urgent_lat[i].lat_read = lat_read;
			break;
		}
	}

	if (i == btsdev->num_urgent_lat)
		dev_warn(btsdev->dev, "%s: bts index %d does not upport set urgent lat\n", __func__,
			 bts_index);
	else
		BTSDBG_LOG(btsdev->dev, "%s: lat read %u\n", urgent_lat[i].name, lat_read);

	spin_unlock(&btsdev->lock);

	if (trace_clock_set_rate_enabled() && i != btsdev->num_urgent_lat) {
		scnprintf(trace_name, sizeof(trace_name), "BTS_%s_ur_lat_rd", urgent_lat[i].name);
		trace_clock_set_rate(trace_name, lat_read, raw_smp_processor_id());
	}
}
EXPORT_SYMBOL_GPL(bts_set_urgent_lat_read);
#endif

int bts_update_bw(unsigned int index, struct bts_bw bw)
{
	struct bts_bw *bts_bw = btsdev->bts_bw;
	unsigned int total_bw;
	char trace_name[32];

	if (index >= btsdev->num_bts) {
		dev_err(btsdev->dev,
			"Invalid index! Should be smaller than %u(index=%u)\n",
			btsdev->num_bts, index);
		goto err;
	}

	total_bw = bw.read + bw.write;
	/* Valid votes for total & peak BW should be both either non-zero or zero */
	if (!total_bw != !bw.peak) {
		dev_err(btsdev->dev, "%s has invalid votes R: %.8u W: %.8u P: %.8u\n",
				bts_bw[index].name, bw.read, bw.write, bw.peak);
		goto err;
	}

	spin_lock(&btsdev->lock);
	bts_bw[index].peak = bw.peak;
	bts_bw[index].read = bw.read;
	bts_bw[index].write = bw.write;
	if (bts_bw[index].is_rt)
		bts_bw[index].rt = bw.rt;
	spin_unlock(&btsdev->lock);

	if(trace_clock_set_rate_enabled()) {
		scnprintf(trace_name, sizeof(trace_name), "BTS_%s_rd_bw", bts_bw[index].name);
		trace_clock_set_rate(trace_name, bts_bw[index].read, raw_smp_processor_id());
		scnprintf(trace_name, sizeof(trace_name), "BTS_%s_wr_bw", bts_bw[index].name);
		trace_clock_set_rate(trace_name, bts_bw[index].write, raw_smp_processor_id());
		scnprintf(trace_name, sizeof(trace_name), "BTS_%s_rt_bw", bts_bw[index].name);
		trace_clock_set_rate(trace_name, bts_bw[index].rt, raw_smp_processor_id());
		scnprintf(trace_name, sizeof(trace_name), "BTS_%s_peak_bw", bts_bw[index].name);
		trace_clock_set_rate(trace_name, bts_bw[index].peak, raw_smp_processor_id());
	}

	BTSDBG_LOG(btsdev->dev,
		   "%s R: %.8u W: %.8u P: %.8u RT: %.8u\n",
		   bts_bw[index].name, bw.read, bw.write, bw.peak, bts_bw[index].rt);

	bts_calc_bw();
	bts_update_stats(index);

	return 0;

err:
	return -EINVAL;
}
EXPORT_SYMBOL_GPL(bts_update_bw);

int bts_add_scenario(unsigned int index)
{
	struct bts_scen *scen = btsdev->scen_list;
	unsigned int i;

	spin_lock(&btsdev->lock);

	if (index >= btsdev->num_scen) {
		dev_err(btsdev->dev, "Invalid scenario index!\n");
		spin_unlock(&btsdev->lock);
		return -EINVAL;
	}

	scen[index].usage_count++;

	if (scen[index].usage_count == 1) {
		list_add(&scen[index].node, &btsdev->scen_node);
		scen[index].status = true;

		if (index >= btsdev->top_scen) {
			btsdev->top_scen = index;
			for (i = 0; i < btsdev->num_bts; i++)
				bts_set(btsdev->top_scen, i);
			trace_clock_set_rate("BTS_scenario", btsdev->top_scen,
					     raw_smp_processor_id());
		}
	}

	spin_unlock(&btsdev->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(bts_add_scenario);

int bts_del_scenario(unsigned int index)
{
	struct bts_scen *scen = btsdev->scen_list;
	unsigned int i;

	spin_lock(&btsdev->lock);
	if (index >= btsdev->num_scen) {
		dev_err(btsdev->dev, "Invalid scenario index!\n");
		spin_unlock(&btsdev->lock);
		return -EINVAL;
	}

	if (index == ID_DEFAULT && scen[index].usage_count == 1) {
		dev_notice(btsdev->dev,
			   "Default scenario cannot be deleted!\n");
		spin_unlock(&btsdev->lock);
		return 0;
	}

	scen[index].usage_count--;

	if (scen[index].usage_count < 0) {
		dev_warn(btsdev->dev, "Usage count is below 0!\n");
		scen[index].usage_count = 0;
		spin_unlock(&btsdev->lock);
		return 0;
	}

	if (scen[index].usage_count == 0) {
		list_del(&scen[index].node);
		scen[index].status = false;

		if (index == btsdev->top_scen) {
			btsdev->top_scen = ID_DEFAULT;
			list_for_each_entry(scen, &btsdev->scen_node, node) {
				if (scen->index >= btsdev->top_scen)
					btsdev->top_scen = scen->index;
			}
			for (i = 0; i < btsdev->num_bts; i++)
				bts_set(btsdev->top_scen, i);
			trace_clock_set_rate("BTS_scenario", btsdev->top_scen,
					     raw_smp_processor_id());
		}
	}

	spin_unlock(&btsdev->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(bts_del_scenario);

/* DebugFS for BTS */
static int exynos_bts_hwstatus_open_show(struct seq_file *buf, void *d)
{
	struct bts_info info;
	struct bts_stat stat;
	int ret, i;

	/* Check BTS setting */
	for (i = 0; i < btsdev->num_bts; i++) {
		info = btsdev->bts_list[i];

		if (!info.va_base)
			continue;
		if (!info.pd_on)
			continue;
		if (info.type == SCI_BTS)
			stat = *info.stat;

		if (!info.ops->get_bts)
			continue;

		spin_lock(&btsdev->lock);
		ret = info.ops->get_bts(info.va_base, &stat);
		spin_unlock(&btsdev->lock);

		if (ret)
			continue;

		seq_printf(buf, "%s:\tARQOS 0x%X, AWQOS 0x%X, RMO 0x%.4X, WMO 0x%.4X, QUR(%u) TH_R 0x%.2X, TH_W 0x%.2X, EX_QUR(%u), ",
			info.name, stat.arqos, stat.awqos,
			stat.rmo, stat.wmo,
			(stat.qurgent_on ? 1 : 0),
			stat.qurgent_th_r, stat.qurgent_th_w,
			(stat.ex_qurgent_on ? 1 : 0));
		if (stat.blocking_on)
			seq_printf(buf,	"BLK(1) FR 0x%.4X, FW 0x%.4X, BR 0x%.4X, BW 0x%.4X, MAX0_R 0x%.4X, MAX0_W 0x%.4X, MAX1_R 0x%.4X, MAX1_W 0x%.4X\n",
				stat.qfull_limit_r,
				stat.qfull_limit_w,
				stat.qbusy_limit_r,
				stat.qbusy_limit_w,
				stat.qmax0_limit_r,
				stat.qmax0_limit_w,
				stat.qmax1_limit_r,
				stat.qmax1_limit_w);
		else
			seq_puts(buf, "BLK(0)\n");
	}

	return 0;
}

static int exynos_bts_hwstatus_open(struct inode *inode, struct file *file)
{
	return single_open(file, exynos_bts_hwstatus_open_show,
			   inode->i_private);
}

static int exynos_bts_bw_open_show(struct seq_file *buf, void *d)
{
	int i;

	mutex_lock(&btsdev->mutex_lock);
	for (i = 0; (btsdev->bts_bw[i].name != NULL) &&
		(i < btsdev->num_bts); i++) {
		seq_printf(
			buf,
			"%s:\tRead: %.8u Write: %.8u Peak %.8u RT %.8u\n",
			btsdev->bts_bw[i].name, btsdev->bts_bw[i].read,
			btsdev->bts_bw[i].write, btsdev->bts_bw[i].peak,
			btsdev->bts_bw[i].rt);
	}
	mutex_unlock(&btsdev->mutex_lock);
	return 0;
}

static int exynos_bts_bw_open(struct inode *inode, struct file *file)
{
	return single_open(file, exynos_bts_bw_open_show,
			   inode->i_private);
}

static int exynos_bts_bw_hist_open_show(struct seq_file *buf, void *d)
{
	int i, j;

	mutex_lock(&btsdev->mutex_lock);
	seq_printf(buf, "Total BW, Count:\nkB/s:\t");
	for (i = 0; i < BTS_HIST_BIN - 1; i++) {
		seq_printf(
			buf,
			"%u\t",
			bw_trip[i]);
	}
	seq_printf(buf, ">%u\n", bw_trip[i - 1]);
	for (i = 0; (btsdev->bts_bw[i].name != NULL) &&
		(i < btsdev->num_bts); i++) {
		seq_printf(
			buf,
			"%s:\t",
			btsdev->bts_bw[i].name);
		for (j = 0; j < BTS_HIST_BIN; j++) {
			seq_printf(
				buf,
				"%u\t",
				btsdev->bts_bw[i].stats.total.count[j]);
		}
		seq_printf(buf, "\n");
	}
	seq_printf(buf, "\nTotal BW, Total time in ms:\nkB/s:\t");
	for (i = 0; i < BTS_HIST_BIN - 1; i++) {
		seq_printf(
			buf,
			"%u\t",
			bw_trip[i]);
	}
	seq_printf(buf, ">%u\n", bw_trip[i - 1]);

	for (i = 0; (btsdev->bts_bw[i].name != NULL) &&
		(i < btsdev->num_bts); i++) {
		seq_printf(
			buf,
			"%s:\t",
			btsdev->bts_bw[i].name);
		for (j = 0; j < BTS_HIST_BIN; j++) {
			seq_printf(
				buf,
				"%llu\t",
				btsdev->bts_bw[i].stats.total.total_time[j] /
				NSEC_PER_MSEC);
		}
		seq_printf(buf, "\n");
	}
	seq_printf(buf, "\nPeak BW, Count:\nkB/s:\t");
	for (i = 0; i < BTS_HIST_BIN - 1; i++) {
		seq_printf(
			buf,
			"%u\t",
			bw_trip[i]);
	}
	seq_printf(buf, ">%u\n", bw_trip[i - 1]);
	for (i = 0; (btsdev->bts_bw[i].name != NULL) &&
		(i < btsdev->num_bts); i++) {
		seq_printf(
			buf,
			"%s:\t",
			btsdev->bts_bw[i].name);
		for (j = 0; j < BTS_HIST_BIN; j++) {
			seq_printf(
				buf,
				"%u\t",
				btsdev->bts_bw[i].stats.peak.count[j]);
		}
		seq_printf(buf, "\n");
	}
	seq_printf(buf, "\nPeak BW, Total time in ms:\nkB/s:\t");
	for (i = 0; i < BTS_HIST_BIN - 1; i++) {
		seq_printf(
			buf,
			"%u\t",
			bw_trip[i]);
	}
	seq_printf(buf, ">%u\n", bw_trip[i - 1]);

	for (i = 0; (btsdev->bts_bw[i].name != NULL) &&
		(i < btsdev->num_bts); i++) {
		seq_printf(
			buf,
			"%s:\t",
			btsdev->bts_bw[i].name);
		for (j = 0; j < BTS_HIST_BIN; j++) {
			seq_printf(
				buf,
				"%llu\t",
				btsdev->bts_bw[i].stats.peak.total_time[j] /
				NSEC_PER_MSEC);
		}
		seq_printf(buf, "\n");
	}
	mutex_unlock(&btsdev->mutex_lock);
	return 0;
}

static int exynos_bts_bw_hist_open(struct inode *inode, struct file *file)
{
	return single_open(file, exynos_bts_bw_hist_open_show,
			   inode->i_private);
}

static ssize_t bts_stats_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i, j;
	ssize_t ret = 0;

	mutex_lock(&btsdev->mutex_lock);
	ret += scnprintf(buf + ret, PAGE_SIZE - ret,
			"Total BW, Time in ms:\nkB/s:\t");
	for (i = 0; i < BTS_HIST_BIN - 1; i++) {
		ret += scnprintf(
			buf + ret,
			PAGE_SIZE - ret,
			"%u\t",
			bw_trip[i]);
	}
	ret += scnprintf(buf + ret, PAGE_SIZE - ret, ">%u\n", bw_trip[i - 1]);

	for (i = 0; (btsdev->bts_bw[i].name != NULL) &&
		(i < btsdev->num_bts); i++) {
		ret += scnprintf(
			buf + ret,
			PAGE_SIZE - ret,
			"%s:\t",
			btsdev->bts_bw[i].name);
		for (j = 0; j < BTS_HIST_BIN; j++) {
			ret += scnprintf(
				buf + ret,
				PAGE_SIZE - ret,
				"%llu\t",
				btsdev->bts_bw[i].stats.total.total_time[j] /
				NSEC_PER_MSEC);
		}
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "\n");
	}
	ret += scnprintf(buf + ret, PAGE_SIZE - ret,
			"\nPeak BW, Time in ms:\nkB/s:\t");
	for (i = 0; i < BTS_HIST_BIN - 1; i++) {
		ret += scnprintf(
			buf + ret,
			PAGE_SIZE - ret,
			"%u\t",
			bw_trip[i]);
	}
	ret += scnprintf(buf + ret, PAGE_SIZE - ret, ">%u\n", bw_trip[i - 1]);

	for (i = 0; (btsdev->bts_bw[i].name != NULL) &&
		(i < btsdev->num_bts); i++) {
		ret += scnprintf(
			buf + ret,
			PAGE_SIZE - ret,
			"%s:\t",
			btsdev->bts_bw[i].name);
		for (j = 0; j < BTS_HIST_BIN; j++) {
			ret += scnprintf(
				buf + ret,
				PAGE_SIZE - ret,
				"%llu\t",
				btsdev->bts_bw[i].stats.peak.total_time[j] /
				NSEC_PER_MSEC);
		}
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "\n");
	}
	mutex_unlock(&btsdev->mutex_lock);
	return ret;
}

static int exynos_bts_scenario_open_show(struct seq_file *buf, void *d)
{
	struct bts_scen *scen;
	unsigned int i;

	seq_printf(buf, "Current Top scenario: [%u]%s\n  ", btsdev->top_scen,
		   btsdev->scen_list[btsdev->top_scen].name);
	list_for_each_entry(scen, &btsdev->scen_node, node) {
		seq_printf(buf, "%u - ", scen->index);
	}
	seq_puts(buf, "\n");

	for (i = 0; i < btsdev->num_scen; i++)
		seq_printf(buf, "bts scen[%u] %s(%d) - status: %s\n",
			   btsdev->scen_list[i].index,
			   btsdev->scen_list[i].name,
			   btsdev->scen_list[i].usage_count,
			   (btsdev->scen_list[i].status ? "on" : "off"));

	return 0;
}

static int exynos_bts_scenario_open(struct inode *inode, struct file *file)
{
	return single_open(file, exynos_bts_scenario_open_show,
			   inode->i_private);
}

static ssize_t exynos_bts_scenario_write(struct file *file,
					 const char __user *user_buf,
					 size_t count, loff_t *ppos)
{
	char buf[32];
	ssize_t buf_size;
	int ret, scen, on;

	buf_size = simple_write_to_buffer(buf, sizeof(buf) - 1, ppos, user_buf,
					  count);
	if (buf_size < 0)
		return buf_size;

	buf[buf_size] = '\0';

	ret = sscanf(buf, "%d %d\n", &scen, &on);
	if (ret != 2) {
		pr_err("%s: sscanf failed. Invalid input variables. count=(%d)\n",
		       __func__, ret);
		return -EINVAL;
	}

	if (scen >= btsdev->num_scen) {
		pr_err("%s: Index should be in range of (0 ~ %d). input=(%d)\n",
		       __func__, btsdev->num_scen - 1, scen);
		return -EINVAL;
	}

	if (on == 1)
		bts_add_scenario(scen);
	else if (on == 0)
		bts_del_scenario(scen);

	return buf_size;
}

static int exynos_bts_qos_open_show(struct seq_file *buf, void *d)
{
	struct bts_info *info = btsdev->bts_list;
	struct bts_stat stat;
	int ret, i;

	for (i = 0; i < btsdev->num_bts; i++) {
		if (!info[i].ops->get_qos)
			continue;

		spin_lock(&btsdev->lock);

		if (info[i].pd_on) {
			if (info[i].type == SCI_BTS)
				stat = *info[i].stat;

			ret = info[i].ops->get_qos(info[i].va_base, &stat);
			if (!stat.bypass)
				seq_printf(buf,
					   "[%d] %s:   \tAR 0x%.1X AW 0x%.1X\n",
					   i, info[i].name, stat.arqos,
					   stat.awqos);
		} else {
			seq_printf(buf, "[%d] %s:   \tLocal power off!\n", i,
				   info[i].name);
		}

		spin_unlock(&btsdev->lock);
	}

	return 0;
}

static int exynos_bts_qos_open(struct inode *inode, struct file *file)
{
	return single_open(file, exynos_bts_qos_open_show, inode->i_private);
}

static ssize_t exynos_bts_qos_write(struct file *file,
				    const char __user *user_buf, size_t count,
				    loff_t *ppos)
{
	char buf[64];
	ssize_t buf_size;

	struct bts_info *info = btsdev->bts_list;
	struct bts_stat stat;
	int ret, index, bypass, ar, aw;

	buf_size = simple_write_to_buffer(buf, sizeof(buf) - 1, ppos, user_buf,
					  count);
	if (buf_size < 0)
		return buf_size;

	buf[buf_size] = '\0';

	ret = sscanf(buf, "%d %d %d %d\n", &index, &bypass, &ar, &aw);
	if (ret != 4) {
		pr_err("%s: sscanf failed. We need 4 inputs. <IP BYPASS(0/1) ARQOS AWQOS> count=(%d)\n",
			__func__, ret);
		return -EINVAL;
	}

	if (index >= btsdev->num_bts) {
		pr_err("%s: IP index should be in range of (0~%d). input=(%d)\n",
		       __func__, btsdev->num_bts - 1, index);
		return -EINVAL;
	}

	if (info[index].type == SCI_BTS)
		stat = *info[index].stat;

	if (bypass == 0)
		stat.bypass = false;
	else if (bypass == 1)
		stat.bypass = true;

	stat.arqos = ar;
	stat.awqos = aw;

	spin_lock(&btsdev->lock);

	if (info[index].ops->set_qos) {
		if (info[index].pd_on) {
			if (info[index].ops->set_qos(info[index].va_base,
						     &stat))
				pr_warn("%s: set_qos failed. input=(%d) err=(%d)\n",
					__func__, index, ret);
		}
	}

	spin_unlock(&btsdev->lock);

	return buf_size;
}

static int exynos_bts_mo_open_show(struct seq_file *buf, void *d)
{
	struct bts_info *info = btsdev->bts_list;
	struct bts_stat stat;
	int ret, i;

	for (i = 0; i < btsdev->num_bts; i++) {
		if (!info[i].ops->get_mo)
			continue;

		spin_lock(&btsdev->lock);

		if (info[i].pd_on) {
			ret = info[i].ops->get_mo(info[i].va_base, &stat);
			seq_printf(buf, "[%d] %s:   \tRMO 0x%.4X WMO 0x%.4X\n",
				   i, info[i].name, stat.rmo, stat.wmo);
		} else {
			seq_printf(buf, "[%d] %s:   \tLocal power off!\n", i,
				   info[i].name);
		}

		spin_unlock(&btsdev->lock);
	}

	return 0;
}

static int exynos_bts_mo_open(struct inode *inode, struct file *file)
{
	return single_open(file, exynos_bts_mo_open_show, inode->i_private);
}

static ssize_t exynos_bts_mo_write(struct file *file,
				   const char __user *user_buf, size_t count,
				   loff_t *ppos)
{
	char buf[64];
	ssize_t buf_size;

	struct bts_info *info = btsdev->bts_list;
	struct bts_stat stat;
	int ret, index, rmo, wmo;

	buf_size = simple_write_to_buffer(buf, sizeof(buf) - 1, ppos, user_buf,
					  count);
	if (buf_size < 0)
		return buf_size;

	buf[buf_size] = '\0';

	ret = sscanf(buf, "%d %d %d\n", &index, &rmo, &wmo);
	if (ret != 3) {
		pr_err("%s: sscanf failed. We need 3 inputs. <IP RMO WMO> count=(%d)\n",
		       __func__, ret);
		return -EINVAL;
	}

	if (index >= btsdev->num_bts) {
		pr_err("%s: IP index should be in range of (0 ~ %d). input=(%d)\n",
		       __func__, btsdev->num_bts - 1, index);
		return -EINVAL;
	}

	stat.rmo = rmo;
	stat.wmo = wmo;

	spin_lock(&btsdev->lock);

	if (info[index].ops->set_mo) {
		if (info[index].pd_on) {
			if (info[index].ops->set_mo(info[index].va_base, &stat))
				pr_warn("%s: set_mo failed. input=(%d) err=(%d)\n",
					__func__, index, ret);
		}
	}

	spin_unlock(&btsdev->lock);

	return buf_size;
}

static int exynos_bts_urgent_open_show(struct seq_file *buf, void *d)
{
	struct bts_info *info = btsdev->bts_list;
	struct bts_stat stat;
	int ret, i;

	for (i = 0; i < btsdev->num_bts; i++) {
		if (!info[i].ops->get_urgent)
			continue;

		spin_lock(&btsdev->lock);

		if (info[i].pd_on) {
			ret = info[i].ops->get_urgent(info[i].va_base, &stat);
			seq_printf(buf,
				   "[%d] %s:   \tQUR(%u) TH_R 0x%.2X TH_W 0x%.2X EX_QUR(%u)\n",
				   i, info[i].name, (stat.qurgent_on ? 1 : 0),
				   stat.qurgent_th_r, stat.qurgent_th_w,
				   (stat.ex_qurgent_on ? 1 : 0));
		} else {
			seq_printf(buf, "[%d] %s:   \tLocal power off!\n", i,
				   info[i].name);
		}

		spin_unlock(&btsdev->lock);
	}

	return 0;
}

static int exynos_bts_urgent_open(struct inode *inode, struct file *file)
{
	return single_open(file, exynos_bts_urgent_open_show, inode->i_private);
}

static ssize_t exynos_bts_urgent_write(struct file *file,
				       const char __user *user_buf,
				       size_t count, loff_t *ppos)
{
	char buf[64];
	ssize_t buf_size;

	struct bts_info *info = btsdev->bts_list;
	struct bts_stat stat;
	int ret, index, on, th_r, th_w, ex_on;

	buf_size = simple_write_to_buffer(buf, sizeof(buf) - 1, ppos, user_buf,
					  count);
	if (buf_size < 0)
		return buf_size;

	buf[buf_size] = '\0';

	ret = sscanf(buf, "%d %d %d %d %d\n", &index, &on, &th_r, &th_w, &ex_on);
	if (ret != 5) {
		pr_err("%s: sscanf failed. We need 5 inputs. <IP ON/OFF TH_R TH_W EX_ON> count=(%d)\n",
		       __func__, ret);
		return -EINVAL;
	}

	if (index >= btsdev->num_bts) {
		pr_err("%s: IP index should be in range of (0 ~ %d). input=(%d)\n",
		       __func__, btsdev->num_bts - 1, index);
		return -EINVAL;
	}

	stat.qurgent_on = on;
	stat.qurgent_th_r = th_r;
	stat.qurgent_th_w = th_w;
	stat.ex_qurgent_on = ex_on;

	spin_lock(&btsdev->lock);

	if (info[index].ops->set_urgent) {
		if (info[index].pd_on) {
			if (info[index].ops->set_urgent(info[index].va_base,
							&stat))
				pr_warn("%s: set_urgent failed. input=(%d) err=(%d)\n",
					__func__, index, ret);
		}
	}

	spin_unlock(&btsdev->lock);

	return buf_size;
}

static int exynos_bts_blocking_open_show(struct seq_file *buf, void *d)
{
	struct bts_info *info = btsdev->bts_list;
	struct bts_stat stat;
	int ret, i;

	for (i = 0; i < btsdev->num_bts; i++) {
		if (!info[i].ops->get_blocking)
			continue;

		spin_lock(&btsdev->lock);

		if (info[i].pd_on) {
			ret = info[i].ops->get_blocking(info[i].va_base, &stat);
			if (stat.blocking_on)
				seq_printf(buf,
					   "[%d] %s:   \tBLK(1) FR 0x%.4X, FW 0x%.4X, BR 0x%.4X, BW 0x%.4X, MAX0_R 0x%.4X, MAX0_W 0x%.4X, MAX1_R 0x%.4X, MAX1_W 0x%.4X\n",
					   i, info[i].name, stat.qfull_limit_r,
					   stat.qfull_limit_w, stat.qbusy_limit_r,
					   stat.qbusy_limit_w, stat.qmax0_limit_r,
					   stat.qmax0_limit_w, stat.qmax1_limit_r,
					   stat.qmax1_limit_w);
			else
				seq_printf(buf, "[%d] %s:   \tBLK(0)\n", i,
					   info[i].name);
		} else {
			seq_printf(buf, "[%d] %s:   \tLocal power off!\n", i,
				   info[i].name);
		}

		spin_unlock(&btsdev->lock);
	}

	return 0;
}

static int exynos_bts_blocking_open(struct inode *inode, struct file *file)
{
	return single_open(file, exynos_bts_blocking_open_show,
			   inode->i_private);
}

static ssize_t exynos_bts_blocking_write(struct file *file,
					 const char __user *user_buf,
					 size_t count, loff_t *ppos)
{
	char buf[64];
	ssize_t buf_size;

	struct bts_info *info = btsdev->bts_list;
	struct bts_stat stat;
	int ret, index, on, full_r, full_w, busy_r, busy_w, max0_r, max0_w,
		max1_r, max1_w;

	buf_size = simple_write_to_buffer(buf, sizeof(buf) - 1, ppos, user_buf,
					  count);
	if (buf_size < 0)
		return buf_size;

	buf[buf_size] = '\0';

	ret = sscanf(buf, "%d %d %d %d %d %d %d %d %d %d\n", &index, &on,
		     &full_r, &full_w, &busy_r, &busy_w, &max0_r, &max0_w,
		     &max1_r, &max1_w);

	if (ret != 10) {
		pr_err("%s: sscanf failed. We need 10 inputs. <IP ON/OFF FULL_R, FULL_W, BUSY_R, BUSY_W, MAX0_R, MAX0_W, MAX1_R, MAX1_W count=(%d)\n",
			__func__, ret);
		return -EINVAL;
	}

	if (index >= btsdev->num_bts) {
		pr_err("%s: IP index should be in range of (0 ~ %d). input=(%d)\n",
		       __func__, btsdev->num_bts - 1, index);
		return -EINVAL;
	}

	stat.blocking_on = on;
	stat.qfull_limit_r = full_r;
	stat.qfull_limit_w = full_w;
	stat.qbusy_limit_r = busy_r;
	stat.qbusy_limit_w = busy_w;
	stat.qmax0_limit_r = max0_r;
	stat.qmax0_limit_w = max0_w;
	stat.qmax1_limit_r = max1_r;
	stat.qmax1_limit_w = max1_w;

	spin_lock(&btsdev->lock);

	if (info[index].ops->set_blocking) {
		if (info[index].pd_on) {
			if (info[index].ops->set_blocking(info[index].va_base,
							  &stat))
				pr_warn("%s: set_blocking failed. input=(%d) err=(%d)\n",
					__func__, index, ret);
		}
	}

	spin_unlock(&btsdev->lock);

	return buf_size;
}

static int exynos_bts_log_open_show(struct seq_file *buf, void *d)
{
	seq_printf(buf, "BTSDBG_LOG STATUS: %s\n", (btsdbg_log ? "on" : "off"));

	return 0;
}

static int exynos_bts_log_open(struct inode *inode, struct file *file)
{
	return single_open(file, exynos_bts_log_open_show, inode->i_private);
}

static ssize_t exynos_bts_log_write(struct file *file,
				    const char __user *user_buf, size_t count,
				    loff_t *ppos)
{
	char buf[16];
	ssize_t buf_size;

	int ret, status;

	buf_size = simple_write_to_buffer(buf, sizeof(buf) - 1, ppos, user_buf,
					  count);
	if (buf_size < 0)
		return buf_size;

	buf[buf_size] = '\0';

	ret = sscanf(buf, "%d\n", &status);
	if (ret != 1) {
		pr_err("%s: sscanf failed. We need 1 input. <ON> count=(%d)\n",
		       __func__, ret);
		return -EINVAL;
	}

	if (status == 1)
		btsdbg_log = true;
	else if (status == 0)
		btsdbg_log = false;

	return buf_size;
}

static int exynos_bts_vc_open_show(struct seq_file *buf, void *d)
{
	struct bts_info info;
	int ret, i;
	unsigned int value, vc_num = 0;

	for (i = 0; i < btsdev->num_bts; i++) {
		info = btsdev->bts_list[i];
		if (!info.va_base || !info.ops->get_vc)
			continue;

		spin_lock(&btsdev->lock);
		ret = info.ops->get_vc(info.va_base, &value);
		spin_unlock(&btsdev->lock);

		if (ret) {
			pr_warn("%s: get_vc failed. err=(%d)\n",
				__func__, ret);
			continue;
		}

		seq_printf(buf, "[%d] %s:   \t0x%.8X\n",
			   vc_num, info.name, value);
		vc_num++;
	}

	return 0;
}

static int exynos_bts_vc_open(struct inode *inode, struct file *file)
{
	return single_open(file, exynos_bts_vc_open_show, inode->i_private);
}

static ssize_t exynos_bts_vc_write(struct file *file,
				   const char __user *user_buf, size_t count,
				   loff_t *ppos)
{
	struct bts_info info;
	char buf[16];
	ssize_t buf_size;

	int i, ret;
	unsigned int value, vc_num, cnt;

	buf_size = simple_write_to_buffer(buf, sizeof(buf) - 1, ppos, user_buf,
					  count);

	if (buf_size < 0)
		return buf_size;

	buf[buf_size] = '\0';

	ret = sscanf(buf, "%8X %d\n", &value, &vc_num);
	if (ret != 2) {
		pr_err("%s: sscanf failed. We need 2 input. <VALUE VC_NUM> count=(%d)\n",
		       __func__, ret);
		return -EINVAL;
	}

	if (vc_num >= vc_num_total) {
		pr_err("%s: vc_num=%d out of range\n", __func__, vc_num);
		return -EINVAL;
	}

	for (i = 0, cnt = 0; i < btsdev->num_bts && cnt <= vc_num; i++) {
		info = btsdev->bts_list[i];
		if (!info.va_base)
			continue;
		if (!info.ops->set_vc)
			continue;
		cnt++;
	}

	info.ops->set_vc(info.va_base, value);

	return buf_size;
}

static const struct file_operations debug_bts_hwstatus_fops = {
	.open = exynos_bts_hwstatus_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations debug_bts_bw_fops = {
	.open = exynos_bts_bw_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations debug_bts_bw_hist_fops = {
	.open = exynos_bts_bw_hist_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations debug_bts_scenario_fops = {
	.open = exynos_bts_scenario_open,
	.read = seq_read,
	.write = exynos_bts_scenario_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations debug_bts_qos_fops = {
	.open = exynos_bts_qos_open,
	.read = seq_read,
	.write = exynos_bts_qos_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations debug_bts_mo_fops = {
	.open = exynos_bts_mo_open,
	.read = seq_read,
	.write = exynos_bts_mo_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations debug_bts_urgent_fops = {
	.open = exynos_bts_urgent_open,
	.read = seq_read,
	.write = exynos_bts_urgent_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations debug_bts_blocking_fops = {
	.open = exynos_bts_blocking_open,
	.read = seq_read,
	.write = exynos_bts_blocking_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations debug_bts_log_fops = {
	.open = exynos_bts_log_open,
	.read = seq_read,
	.write = exynos_bts_log_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations debug_bts_vc_fops = {
	.open = exynos_bts_vc_open,
	.read = seq_read,
	.write = exynos_bts_vc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

int exynos_bts_debugfs_init(void)
{
	struct dentry *den;

	den = debugfs_create_dir("bts", NULL);
	if (IS_ERR(den))
		return -ENOMEM;

	debugfs_create_file("status", 0444, den, NULL,
			    &debug_bts_hwstatus_fops);
	debugfs_create_file("scenario", 0444, den, NULL,
			    &debug_bts_scenario_fops);
	debugfs_create_file("qos", 0440, den, NULL, &debug_bts_qos_fops);
	debugfs_create_file("mo", 0440, den, NULL, &debug_bts_mo_fops);
	debugfs_create_file("urgent", 0440, den, NULL, &debug_bts_urgent_fops);
	debugfs_create_file("blocking", 0440, den, NULL,
			    &debug_bts_blocking_fops);
	debugfs_create_file("log", 0440, den, NULL, &debug_bts_log_fops);
	debugfs_create_file("bw", 0440, den, NULL, &debug_bts_bw_fops);
	debugfs_create_file("bw_hist", 0440, den, NULL, &debug_bts_bw_hist_fops);
	debugfs_create_file("vc", 0444, den, NULL, &debug_bts_vc_fops);

	return 0;
}

static int exynos_bts_syscore_suspend(void)
{
	return 0;
}

static void exynos_bts_syscore_resume(void)
{
	struct bts_info *info = btsdev->bts_list;
	unsigned int i;

	spin_lock(&btsdev->lock);
	for (i = 0; i < btsdev->num_bts; i++) {
		if (info[i].ops->init_bts)
			info[i].ops->init_bts(info[i].va_base);
		bts_set(btsdev->top_scen, i);
	}
	spin_unlock(&btsdev->lock);
}

/* Syscore operation */
static struct syscore_ops exynos_bts_syscore_ops = {
	.suspend = exynos_bts_syscore_suspend,
	.resume = exynos_bts_syscore_resume,
};

/* BTS Initialize */
static int bts_initialize(struct bts_device *data)
{
	struct bts_info *info = btsdev->bts_list;
	unsigned int i;
	int ret;

	/* Default scenario should be enabled */
	spin_lock(&btsdev->lock);
	for (i = 0; i < btsdev->num_bts; i++) {
		if (info[i].ops->init_bts)
			info[i].ops->init_bts(info[i].va_base);
	}
	spin_unlock(&btsdev->lock);

	btsdev->top_scen = ID_DEFAULT;
	ret = bts_add_scenario(ID_DEFAULT);
	if (ret) {
		dev_err(data->dev, "failed to add scenario!\n");
		return ret;
	}

	return ret;
}

static int bts_parse_setting(struct device_node *np, struct bts_stat *stat)
{
	int tmp;

	of_property_read_u32(np, "stat_on", &tmp);
	stat->stat_on = tmp ? true : false;

	/* Initialize */
	if (!stat->stat_on)
		return 0;

	of_property_read_u32(np, "bypass", &tmp);
	stat->bypass = tmp ? true : false;

	if (of_property_read_u32(np, "arqos", &stat->arqos))
		stat->arqos = DEFAULT_QOS;
	if (of_property_read_u32(np, "awqos", &stat->awqos))
		stat->awqos = DEFAULT_QOS;
	if (of_property_read_u32(np, "rmo", &stat->rmo))
		stat->rmo = MAX_MO;
	if (of_property_read_u32(np, "wmo", &stat->wmo))
		stat->wmo = MAX_MO;

	of_property_read_u32(np, "qurgent_on", &tmp);
	stat->qurgent_on = tmp ? true : false;
	if (of_property_read_u32(np, "ex_qurgent_on", &tmp))
		stat->ex_qurgent_on = false;
	else
		stat->ex_qurgent_on = tmp ? true : false;

	if (of_property_read_u32(np, "qurgent_th_r",
				 &stat->qurgent_th_r))
		stat->qurgent_th_r = MAX_QUTH;
	if (of_property_read_u32(np, "qurgent_th_w",
				 &stat->qurgent_th_w))
		stat->qurgent_th_w = MAX_QUTH;

	of_property_read_u32(np, "blocking_on", &tmp);
	stat->blocking_on = tmp ? true : false;

	if (of_property_read_u32(np, "qfull_limit_r",
				 &stat->qfull_limit_r))
		stat->qfull_limit_r = MAX_MO;
	if (of_property_read_u32(np, "qfull_limit_w",
				 &stat->qfull_limit_w))
		stat->qfull_limit_w = MAX_MO;
	if (of_property_read_u32(np, "qbusy_limit_r",
				 &stat->qbusy_limit_r))
		stat->qbusy_limit_r = MAX_MO;
	if (of_property_read_u32(np, "qbusy_limit_w",
				 &stat->qbusy_limit_w))
		stat->qbusy_limit_w = MAX_MO;
	if (of_property_read_u32(np, "qmax0_limit_r",
				 &stat->qmax0_limit_r))
		stat->qmax0_limit_r = MAX_MO;
	if (of_property_read_u32(np, "qmax0_limit_w",
				 &stat->qmax0_limit_w))
		stat->qmax0_limit_w = MAX_MO;
	if (of_property_read_u32(np, "qmax1_limit_r",
				 &stat->qmax1_limit_r))
		stat->qmax1_limit_r = MAX_MO;
	if (of_property_read_u32(np, "qmax1_limit_w",
				 &stat->qmax1_limit_w))
		stat->qmax1_limit_w = MAX_MO;


	return 0;
}

#if IS_ENABLED(CONFIG_SOC_ZUMA)
static int bts_parse_nocl_data_index(struct device_node *np, struct bts_device *data,
				     int index, const char *nocl_name)
{
	struct device_node *child_np, *nocl_np = NULL;
	int i = 0;

	if (!np)
		return -ENODEV;

	nocl_np = of_get_child_by_name(np, nocl_name);
	if (!nocl_np)
		return -ENODEV;

	data->nocl_infos[index].num_ip = of_get_child_count(nocl_np);
	if (!data->nocl_infos[index].num_ip) {
		BTSDBG_LOG(data->dev,
			   "No %s ip found\n", nocl_name);
		return -EINVAL;
	}
	data->nocl_infos[index].nocl_ips = devm_kcalloc(data->dev,
						data->nocl_infos[index].num_ip,
						sizeof(struct nocl_ip_info), GFP_KERNEL);
	if (!data->nocl_infos[index].nocl_ips) {
		BTSDBG_LOG(data->dev,
			   "Unable to allocate memory for %s ip\n", nocl_name);
		return -ENOMEM;
	}
	for_each_child_of_node(nocl_np, child_np) {
		if (!child_np->name)
			return -EINVAL;
		data->nocl_infos[index].nocl_ips[i].ip_name = child_np->name;
		if (of_property_read_u32(child_np, "bus-width",
					 &data->nocl_infos[index].nocl_ips[i].ip_bus_width))
			return -EINVAL;
		i++;
	}

	return 0;
}
#endif

#define NUM_COLS 2
#define OF_DATA_NUM_MAX 16
static int bts_parse_data(struct device_node *np, struct bts_device *data)
{
	struct bts_scen *scen;
	struct bts_info *info;
	struct device_node *child_np = NULL;
	struct device_node *snp = NULL;
	struct resource res;
	int i, j, map_cnt, count;
	int of_data_int_array[OF_DATA_NUM_MAX];
	int ret = 0;
#if IS_ENABLED(CONFIG_SOC_ZUMA)
	struct bts_urgent_lat *urgent_lat;
	bool has_nocl2aa = false;
	bool has_nocl2ab = false;
#endif

	if (!of_have_populated_dt()) {
		dev_err(data->dev, "Invalid device tree node!\n");
		ret = -EINVAL;
		goto err;
	}

	map_cnt = of_property_count_elems_of_size(np, "bus1_int_map",
						    sizeof(u32));
	if (map_cnt <= 0 || map_cnt % NUM_COLS) {
		ret = -ENODEV;
		goto err;
	}
	if (of_property_read_u32_array(np, "bus1_int_map",
				       (u32 *)of_data_int_array,
				       (size_t)map_cnt)) {
		ret = -ENODEV;
		goto err;
	}
	data->map_row_cnt = map_cnt / NUM_COLS;
	data->bus1_int_tbl =
		devm_kcalloc(data->dev, data->map_row_cnt, sizeof(struct bus1_int_map),
		             GFP_KERNEL);
	if (!data->bus1_int_tbl) {
		dev_err(data->dev,
			"Failed to allocate memory for bus1_int_tbl\n");
		ret = -ENOMEM;
		goto err;
	}
	for (i = 0; i < data->map_row_cnt; i++) {
		data->bus1_int_tbl[i].bus1_freq = of_data_int_array[i * NUM_COLS];
		data->bus1_int_tbl[i].int_freq = of_data_int_array[i * NUM_COLS + 1];
	}

	count = of_property_count_strings(np, "list-scen");
	if (count <= 0) {
		BTSDBG_LOG(data->dev,
			   "There should be at least one scenario\n");
		ret = -EINVAL;
		goto err;
	}

	data->num_scen = count;
	scen = devm_kcalloc(data->dev, data->num_scen,
			    sizeof(struct bts_scen), GFP_KERNEL);
	if (!scen) {
		ret = -ENOMEM;
		goto err;
	}

	for (i = 0; i < data->num_scen; i++) {
		scen[i].index = i;
		scen[i].status = false;
		scen[i].usage_count = 0;
		ret = of_property_read_string_index(np, "list-scen", i,
						    &scen[i].name);
		if (ret < 0) {
			dev_err(data->dev,
				"Unable to get name of bts scenarios\n");
			goto err;
		}
	}

	count = of_property_count_strings(np, "rt-names");
	if (count <= 0) {
		BTSDBG_LOG(data->dev,
			   "No rt names found\n");
		ret = -EINVAL;
		goto err;
	}

	data->num_rts = count;
	data->rt_names = devm_kcalloc(data->dev, data->num_rts,
				      sizeof(char *), GFP_KERNEL);
	if (!data->rt_names) {
		ret = -ENOMEM;
		goto err;
	}
	for (i = 0; i < data->num_rts; i++) {
		const char *rt_name;
		ret = of_property_read_string_index(np, "rt-names", i,
						    &rt_name);
		if (ret) {
			dev_err(data->dev,
				"Unable to get name of rt IPs\n");
			goto err;
		}
		data->rt_names[i] = rt_name;
	}

	data->num_bts = (unsigned int)of_get_child_count(np);
	if (!data->num_bts) {
		BTSDBG_LOG(data->dev,
			   "There should be at least one bts\n");
		ret = -EINVAL;
		goto err;
	}

#if IS_ENABLED(CONFIG_SOC_ZUMA)
	count = of_property_count_strings(np, "nocl-names");
	if (count <= 0 || count >= data->num_bts) {
		BTSDBG_LOG(data->dev,
			   "No nocl names found or the count is wrong\n");
		ret = -EINVAL;
		goto err;
	}
	data->num_nocl = count;
	data->nocl_infos = devm_kcalloc(data->dev, data->num_nocl,
					sizeof(struct nocl_info), GFP_KERNEL);
	if (!data->nocl_infos) {
		ret = -ENOMEM;
		goto err;
	}
	for (i = 0; i < data->num_nocl; i++) {
		ret = of_property_read_string_index(np, "nocl-names", i,
						    &data->nocl_infos[i].nocl_name);
		if (ret) {
			dev_err(data->dev,
				"Unable to get name of nocls\n");
			goto err;
		}
		if (!strcmp(data->nocl_infos[i].nocl_name, "nocl2aa"))
			has_nocl2aa = true;
		else if (!strcmp(data->nocl_infos[i].nocl_name, "nocl2ab"))
			has_nocl2ab = true;
	}
	if (!has_nocl2aa || !has_nocl2ab) {
		dev_err(data->dev,
			"Missing nocl2aa and/or nocl2ab info\n");
		goto err;
	}
	for (i = 0; i < data->num_nocl; i++) {
		ret = bts_parse_nocl_data_index(np, data, i, data->nocl_infos[i].nocl_name);
		if (ret) {
			dev_err(data->dev,
				"Failed to parse nocl data\n");
			goto err;
		}
		INIT_LIST_HEAD(&data->nocl_infos[i].list);
	}
	data->num_bts -= data->num_nocl;
#endif
	info = devm_kcalloc(data->dev, data->num_bts,
			    sizeof(struct bts_info), GFP_KERNEL);
	if (!info) {
		ret = -ENOMEM;
		goto err;
	}

	i = 0;

	for_each_child_of_node(np, child_np) {
#if IS_ENABLED(CONFIG_SOC_ZUMA)
		if (of_property_read_bool(child_np, "nocl_node"))
			continue;
#endif
		/* Parsing scenario data */
		info[i].stat = devm_kcalloc(data->dev, data->num_scen,
					    sizeof(struct bts_stat),
					    GFP_KERNEL);
		if (!info[i].stat) {
			ret = -ENOMEM;
			goto err;
		}

		/* IOREMAP physical address */
		ret = of_address_to_resource(child_np, 0, &res);
		if (ret) {
			dev_err(data->dev,
				"failed to get address_to_resource\n");
			if (ret == -EINVAL)
				ret = 0;
		} else {
			info[i].va_base =
				devm_ioremap_resource(data->dev, &res);
			if (IS_ERR(info[i].va_base)) {
				dev_err(data->dev,
					"failed to ioremap register\n");
				ret = -ENOMEM;
			}
			ret = of_address_to_resource(child_np, 1, &res);
			if (!ret) {
				info[i].stat->qos_va_base =
					devm_ioremap_resource(data->dev,
							      &res);
				if (IS_ERR(info[i].stat->qos_va_base)) {
					dev_err(data->dev,
						"failed to ioremap register\n");
					ret = -ENOMEM;
				}
			}
		}

		info[i].status = of_device_is_available(child_np);

		/* Parsing bts-type */
		if (of_property_read_u32(child_np, "bts-type",
					 &info[i].type)) {
			dev_warn(data->dev, "failed to get bts-type\n");
			ret = -EEXIST;
			goto err;
		}

		if (info[i].type == INTERNAL_BTS) {
			ret = of_property_read_string(child_np, "vc-string", &info[i].name);
			if (ret) {
				dev_err(data->dev, "failed to get vc-string\n");
				goto err;
			}
			vc_num_total++;
		} else {
			info[i].name = child_np->name;
		}

		/* Register operation function */
		ret = register_btsops(&info[i]);
		if (ret)
			goto err;

		/* Parsing local power domain information */
		if (of_property_read_u32(child_np, "cal-pdid",
					 &info[i].pd_id)) {
			info[i].pd_id = 0;
			info[i].pd_on = true;
		} else {
			info[i].pd_on = cal_pd_status(info[i].pd_id) ?
						true :
						false;
		}

		if (!of_get_child_count(child_np)) {
			info[i].stat = NULL;
			i++;
			continue;
		}

		for (j = 0; j < data->num_scen; j++)
			info[i].stat[j].stat_on = 0;

		for_each_child_of_node(child_np, snp) {
			for (j = 0; j < data->num_scen; j++) {
				if (strcmp(snp->name, scen[j].name))
					continue;
				bts_parse_setting(snp,
						  &info[i].stat[j]);
			}
		}

		i++;
	}

	data->bts_bw = devm_kcalloc(data->dev, data->num_bts,
				    sizeof(struct bts_bw), GFP_KERNEL);
	if (!data->bts_bw) {
		ret = -ENOMEM;
		goto err;
	}

#if IS_ENABLED(CONFIG_SOC_ZUMA)
	count = of_property_count_strings(np, "list-urgent-lat");

	if (count > 0) {
		unsigned int index;

		urgent_lat =
			devm_kcalloc(data->dev, count, sizeof(struct bts_urgent_lat), GFP_KERNEL);
		if (!urgent_lat) {
			ret = -ENOMEM;
			goto err;
		}

		for (i = 0; i < count; i++) {
			ret = of_property_read_string_index(np, "list-urgent-lat", i,
							    &urgent_lat[i].name);
			if (ret < 0) {
				dev_err(data->dev, "Unable to get name of urgent lat list\n");
				goto err;
			}
			for (index = 0; index < data->num_bts; index++) {
				if (strcmp(urgent_lat[i].name, info[index].name) == 0)
					break;
			}
			urgent_lat[i].bts_index = index;
		}
		data->num_urgent_lat = count;
		data->urgent_lat_list = urgent_lat;
	}
#endif

	data->bts_list = info;
	data->scen_list = scen;

err:
	return ret;
}

static DEVICE_ATTR_RO(bts_stats);

static struct attribute *bts_dev_attrs[] = {
	&dev_attr_bts_stats.attr,
	NULL
};

ATTRIBUTE_GROUPS(bts_dev);

static int bts_probe(struct platform_device *pdev)
{
	int ret;

	btsdev = devm_kmalloc(&pdev->dev, sizeof(struct bts_device), GFP_KERNEL);
	if (!btsdev)
		return -ENOMEM;

	btsdev->dev = &pdev->dev;

	if (strcmp(pdev->name, "exynos-bts")) {
		dev_err(btsdev->dev,
			"failed to get bts data from device tree\n");
		return -ENOENT;
	}

	ret = bts_parse_data(btsdev->dev->of_node, btsdev);
	if (ret) {
		dev_err(btsdev->dev, "failed to parse data (err=%d)\n",
			ret);
		devm_kfree(btsdev->dev, btsdev);
		return ret;
	}
	spin_lock_init(&btsdev->lock);
	mutex_init(&btsdev->mutex_lock);
	INIT_LIST_HEAD(&btsdev->scen_node);

	ret = bts_initialize(btsdev);
	if (ret) {
		dev_err(btsdev->dev, "failed to initialize (err=%d)\n",
			ret);
		devm_kfree(btsdev->dev, btsdev);
		return ret;
	}

	platform_set_drvdata(pdev, btsdev);

#if IS_ENABLED(CONFIG_EXYNOS_PM_QOS)
	exynos_pm_qos_add_request(&exynos_mif_qos, PM_QOS_BUS_THROUGHPUT, 0);
	exynos_pm_qos_add_request(&exynos_int_qos, PM_QOS_DEVICE_THROUGHPUT, 0);
#else
	pm_qos_add_request(&exynos_mif_qos, PM_QOS_BUS_THROUGHPUT, 0);
	pm_qos_add_request(&exynos_int_qos, PM_QOS_DEVICE_THROUGHPUT, 0);
#endif
	ret = exynos_bts_debugfs_init();
	if (ret)
		dev_err(btsdev->dev, "exynos_bts_debugfs_init failed\n");

	register_syscore_ops(&exynos_bts_syscore_ops);

	pr_info("%s successfully done.\n", __func__);

	return ret;
}

static int bts_remove(struct platform_device *pdev)
{
	devm_kfree(&pdev->dev, btsdev);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

/* Device tree compatible information */
static const struct of_device_id exynos_bts_match[] = {
	{
		.compatible = "samsung,exynos-bts",
	},
	{ },
};
MODULE_DEVICE_TABLE(of, exynos_bts_match);

static struct platform_driver bts_pdrv = {
	.probe = bts_probe,
	.remove = bts_remove,
	.driver = {
		   .name = BTS_PDEV_NAME,
		   .dev_groups = bts_dev_groups,
		   .owner = THIS_MODULE,
		   .of_match_table = exynos_bts_match,
		    },
};

module_platform_driver(bts_pdrv);

MODULE_DESCRIPTION("Samsung BTS driver");
MODULE_LICENSE("GPL");
