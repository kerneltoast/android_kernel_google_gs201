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
#include <linux/module.h>
#include <linux/pm_opp.h>
#include <soc/google/bts.h>

#include "bigo_pm.h"
#include "bigo_io.h"

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

	list_for_each_entry(opp, &core->pm.opps, list) {
		if (opp->load_pps >= load)
			break;
	}
	return opp->freq_khz;
}

static inline struct bigo_bw *bigo_get_target_bw(struct bigo_core *core, u32 load)
{
	struct bigo_bw *bw;

	list_for_each_entry(bw, &core->pm.bw, list) {
		if (bw->load_pps >= load)
			break;
	}
	return bw;
}

static inline void bigo_set_freq(struct bigo_core *core, u32 freq)
{
	if (core->debugfs.set_freq)
		freq = core->debugfs.set_freq;

	if (!exynos_pm_qos_request_active(&core->pm.qos_bigo))
		exynos_pm_qos_add_request(&core->pm.qos_bigo, PM_QOS_BO_THROUGHPUT, freq);
	else
		exynos_pm_qos_update_request(&core->pm.qos_bigo, freq);
}

static void bigo_scale_freq(struct bigo_core *core)
{
	u32 load = bigo_get_total_load(core);
	u32 freq = bigo_get_target_freq(core, load);

	bigo_set_freq(core, freq);
}

static void bigo_get_bw(struct bigo_core *core, struct bts_bw *bw)
{
	u32 load = bigo_get_total_load(core);
	struct bigo_bw *bandwidth = bigo_get_target_bw(core, load);

	bw->read = bandwidth->rd_bw;
	bw->write = bandwidth->wr_bw;
	bw->peak = bandwidth->pk_bw;
	pr_debug("BW: load: %u, rd: %u, wr: %u, pk: %u", load, bw->read, bw->write, bw->peak);
}

static int bigo_scale_bw(struct bigo_core *core)
{
	struct bts_bw bw;

	bigo_get_bw(core, &bw);
	return bts_update_bw(core->pm.bwindex, bw);
}

void bigo_update_qos(struct bigo_core *core)
{
	int rc;

	mutex_lock(&core->lock);
	rc = bigo_scale_bw(core);

	if (rc)
		pr_warn("%s: failed to scale bandwidth: %d\n", __func__, rc);

	bigo_scale_freq(core);
	mutex_unlock(&core->lock);
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
	return 0;
}

int bigo_runtime_resume(struct device *dev)
{
	return 0;
}
#endif

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Vinay Kalia <vinaykalia@google.com>");
