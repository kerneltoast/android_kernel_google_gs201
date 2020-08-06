// SPDX-License-Identifier: GPL-2.0-only
/*
 * BigOcean power management
 *
 * Copyright 2020 Google LLC.
 *
 * Author: Vinay Kalia <vinaykalia@google.com>
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/clk.h>
#include <linux/pm_opp.h>
#include <soc/samsung/bts.h>

#include "bigo_pm.h"
#include "bigo_io.h"

#define DEFAULT_RD_BW_FACTOR 5
#define DEFAULT_WR_BW_FACTOR 3
#define PEAK_BW_FACTOR_NUM 3
#define PEAK_BW_FACTOR_DEN 2

static inline u32 bigo_get_total_load(struct bigo_core *core)
{
	struct bigo_inst *inst;
	u32 load = 0;
	u32 curr_load = 0;

	if (list_empty(&core->instances))
		return 0;

	list_for_each_entry(inst, &core->instances, list) {
		curr_load = inst->width * inst->height * inst->fps / 1024;
		if (curr_load < core->pm.max_load - load) {
			load += curr_load;
		} else {
			load = core->pm.max_load;
			break;
		}
	}
	/* 1 <= load <= core->pm.max_load */
	load = max(1U, load);
	load = min(load, core->pm.max_load);
	return load;
}

static inline u32 bigo_get_target_freq(struct bigo_core *core, u32 load)
{
	struct bigo_opp *opp;
	u32 freq = core->pm.min_freq;

	list_for_each_entry(opp, &core->pm.opps, list) {
		if (opp->load_pps >= load) {
			freq = opp->freq_khz;
			break;
		}
	}
	freq = max(freq, core->pm.min_freq);
	return freq;
}

static inline void bigo_set_freq(struct bigo_core *core, u32 freq)
{
	if (!pm_qos_request_active(&core->pm.qos_bigo))
		pm_qos_add_request(&core->pm.qos_bigo, PM_QOS_BO_THROUGHPUT, freq);
	else
		pm_qos_update_request(&core->pm.qos_bigo, freq);
}

static void bigo_scale_freq(struct bigo_core *core)
{
	u32 load = bigo_get_total_load(core);
	u32 freq = bigo_get_target_freq(core, load);

	bigo_set_freq(core, freq);
}

static void bigo_remove_freq(struct bigo_core *core)
{
	u32 freq = bigo_get_target_freq(core, 0);

	bigo_set_freq(core, freq);
}

static void bigo_get_default_bw(struct bigo_inst *inst, struct bts_bw *bw)
{
	u32 load = bigo_get_total_load(inst->core);

	bw->read = load * DEFAULT_RD_BW_FACTOR;
	bw->write = load * DEFAULT_WR_BW_FACTOR;
	bw->peak = (max(bw->read, bw->write) * PEAK_BW_FACTOR_NUM) / PEAK_BW_FACTOR_DEN;
}

static void bigo_get_initial_bw(struct bigo_inst *inst, struct bts_bw *bw)
{
	struct bigo_bw peak = { 0 };
	int i;
	u32 time_ms = 0;
	u32 cum_cycles = 0;
	u32 avg_cycles = 0;
	u32 load = bigo_get_total_load(inst->core);
	u32 freq = bigo_get_target_freq(inst->core, load);

	for (i = 0; i < PEAK_CNT; i++) {
		peak.rd_bw = max(inst->pk_bw[i].rd_bw, peak.rd_bw);
		peak.wr_bw = max(inst->pk_bw[i].wr_bw, peak.wr_bw);
		cum_cycles += inst->hw_cycles[i];
	}
	avg_cycles = cum_cycles / PEAK_CNT;
	time_ms = max(1U, avg_cycles / freq);
	bw->read = peak.rd_bw * BUS_WIDTH / time_ms;
	bw->write = peak.wr_bw * BUS_WIDTH / time_ms;
	bw->peak = (max(bw->read, bw->write) * PEAK_BW_FACTOR_NUM) / PEAK_BW_FACTOR_DEN;
}

static void bigo_get_stable_bw(struct bigo_inst *inst, struct bts_bw *bw)
{
	struct bigo_bw avg = { 0 };
	struct bigo_bw peak = { 0 };
	struct bigo_bw cum = { 0 };
	int i;
	u32 time_ms = 0;
	u32 cum_cycles = 0;
	u32 avg_cycles = 0;
	u32 peak_bw;
	u32 load = bigo_get_total_load(inst->core);
	u32 freq = bigo_get_target_freq(inst->core, load);

	for (i = 0; i < AVG_CNT; i++) {
		cum.rd_bw += inst->avg_bw[i].rd_bw;
		cum.wr_bw += inst->avg_bw[i].wr_bw;
		cum_cycles += inst->hw_cycles[i];
	}
	for (i = 0; i < PEAK_CNT; i++) {
		peak.rd_bw = max(inst->pk_bw[i].rd_bw, peak.rd_bw);
		peak.wr_bw = max(inst->pk_bw[i].wr_bw, peak.wr_bw);
	}
	avg.rd_bw = cum.rd_bw / AVG_CNT;
	avg.wr_bw = cum.wr_bw / AVG_CNT;
	avg_cycles = cum_cycles / AVG_CNT;
	time_ms = max(1U, avg_cycles / freq);
	bw->read = (avg.rd_bw * BUS_WIDTH) / time_ms;
	bw->write = (avg.wr_bw * BUS_WIDTH) / time_ms;
	peak_bw = max(peak.rd_bw, peak.wr_bw);
	peak_bw = max(peak_bw, avg.rd_bw);
	peak_bw = max(peak_bw, avg.wr_bw);
	bw->peak = ((peak_bw / time_ms) * BUS_WIDTH * PEAK_BW_FACTOR_NUM) / PEAK_BW_FACTOR_DEN;
}

static int bigo_scale_bw(struct bigo_inst *inst)
{
	struct bts_bw bw;

	if (!inst) {
		pr_warn("%s: inst is NULL\n", __func__);
		return -EINVAL;
	}

	if (inst->job_cnt < PEAK_CNT)
		bigo_get_default_bw(inst, &bw);
	else if (inst->job_cnt > AVG_CNT)
		bigo_get_initial_bw(inst, &bw);
	else
		bigo_get_stable_bw(inst, &bw);

	return bts_update_bw(inst->core->pm.bwindex, bw);
}

static int bigo_remove_bw(struct bigo_inst *inst)
{
	struct bts_bw bw = { 0 };

	if (!inst) {
		pr_warn("%s: inst is NULL\n", __func__);
		return -EINVAL;
	}
	return bts_update_bw(inst->core->pm.bwindex, bw);
}

/*
 * bigo_pm_init(): Initializes power management for bigocean.
 * @core: the bigocean core
 */
int bigo_pm_init(struct bigo_core *core)
{
	return 0;
}

#if IS_ENABLED(CONFIG_PM)
int bigo_runtime_suspend(struct device *dev)
{
	int rc;
	struct bigo_core *core = dev_get_drvdata(dev);
	struct bigo_inst *inst = core->curr_inst;

	bigo_remove_freq(core);
	rc = bigo_remove_bw(inst);
	if (rc)
		pr_err("%s: failed to remove bw: %d\n", __func__, rc);
	return rc;
}

int bigo_runtime_resume(struct device *dev)
{
	int rc;
	struct bigo_core *core = dev_get_drvdata(dev);
	struct bigo_inst *inst = core->curr_inst;

	rc  = bigo_scale_bw(inst);
	if (rc) {
		pr_err("%s: failed to scale bandwidth: %d\n", __func__, rc);
		goto err;
	}

	bigo_scale_freq(core);

err:
	return rc;
}
#endif
