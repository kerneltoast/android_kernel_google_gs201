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

static inline u32 bigo_get_total_load(struct bigo_core *core, bool use_bpp)
{
	struct bigo_inst *inst;
	u32 load = 0;
	u32 curr_load = 0;

	if (list_empty(&core->instances))
		return 0;

	list_for_each_entry(inst, &core->instances, list) {
		if (inst->idle)
			continue;
		curr_load = (u64)inst->width * inst->height * inst->fps / 1024;
		if (use_bpp) {
			curr_load *= inst->bpp;
		}
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

#if IS_ENABLED(CONFIG_SOC_ZUMA)

#define BIGW_A0_CSR_PROG_FREQ 166000

static inline void bigo_set_freq(struct bigo_core *core, u32 freq)
{
	if (core->debugfs.set_freq)
		freq = core->debugfs.set_freq;

	/* HW bug workaround: see b/215390692 */
	if (core->ip_ver < 1 && freq > BIGW_A0_CSR_PROG_FREQ)
		freq = BIGW_A0_CSR_PROG_FREQ;

	if (!exynos_pm_qos_request_active(&core->pm.qos_bigo))
		exynos_pm_qos_add_request(&core->pm.qos_bigo, PM_QOS_BW_THROUGHPUT, freq);
	else
		exynos_pm_qos_update_request(&core->pm.qos_bigo, freq);
}

static void bigo_scale_freq(struct bigo_core *core)
{
	u32 load = bigo_get_total_load(core, true);
	u32 freq = bigo_get_target_freq(core, load);

	bigo_set_freq(core, freq);
}

static void bigo_get_bw(struct bigo_core *core, struct bts_bw *bw)
{
	u32 load = bigo_get_total_load(core, false);

	if (load) {
		struct bigo_bw *bandwidth = bigo_get_target_bw(core, load);
		struct bigo_inst *inst;
		bool afbc = true;

		/* Use AFBC BW table when all insts are AFBC */
		list_for_each_entry(inst, &core->instances, list) {
			if (!inst->afbc) {
				afbc = false;
				break;
			}
		}
		if (afbc) {
			bw->read = bandwidth->rd_bw_afbc;
			bw->write = bandwidth->wr_bw_afbc;
			bw->peak = bandwidth->pk_bw_afbc;
		} else {
			if (inst->is_decoder_usage) {
				bw->read = bandwidth->rd_bw;
				bw->write = bandwidth->wr_bw;
				bw->peak = bandwidth->pk_bw;
			} else {
				bw->read = bandwidth->rd_bw_enc;
				bw->write = bandwidth->wr_bw_enc;
				bw->peak = bandwidth->pk_bw_enc;
			}
		}
	} else {
		memset(bw, 0, sizeof(*bw));
	}
	pr_debug("BW: load: %u, rd: %u, wr: %u, pk: %u", load, bw->read, bw->write, bw->peak);
}

#else

#define LARGE_LOAD_MIF_FLOOR 1539000

static inline void update_mif_floor(struct bigo_core *core)
{
	struct bigo_inst *inst;
	u32 load = 0;
	u32 curr_load = 0;

	if (!list_empty(&core->instances)) {
		list_for_each_entry(inst, &core->instances, list) {
			if (inst->idle)
				continue;
			curr_load = inst->width * inst->height * inst->fps * inst->bpp / 1024;
			load += curr_load;
		}
	}

	if (load > core->pm.max_load) {
		if (!exynos_pm_qos_request_active(&core->pm.qos_req_mif))
			exynos_pm_qos_add_request(&core->pm.qos_req_mif, PM_QOS_BUS_THROUGHPUT, LARGE_LOAD_MIF_FLOOR);
		else
			exynos_pm_qos_update_request(&core->pm.qos_req_mif, LARGE_LOAD_MIF_FLOOR);
	} else if (exynos_pm_qos_request_active(&core->pm.qos_req_mif)) {
			exynos_pm_qos_remove_request(&core->pm.qos_req_mif);
	}
}

static inline void bigo_set_freq(struct bigo_core *core, u32 freq)
{
	if (core->debugfs.set_freq)
		freq = core->debugfs.set_freq;

	if (!exynos_pm_qos_request_active(&core->pm.qos_bigo))
		exynos_pm_qos_add_request(&core->pm.qos_bigo, PM_QOS_BW_THROUGHPUT, freq);
	else
		exynos_pm_qos_update_request(&core->pm.qos_bigo, freq);
}

static void bigo_scale_freq(struct bigo_core *core)
{
	u32 load = bigo_get_total_load(core, false);
	u32 freq = bigo_get_target_freq(core, load);

	bigo_set_freq(core, freq);
}

static void bigo_get_bw(struct bigo_core *core, struct bts_bw *bw)
{
	u32 load = bigo_get_total_load(core, false);
	if (load) {
		struct bigo_bw *bandwidth = bigo_get_target_bw(core, load);
		bw->read = bandwidth->rd_bw;
		bw->write = bandwidth->wr_bw;
		bw->peak = bandwidth->pk_bw;
	} else {
		memset(bw, 0, sizeof(*bw));
	}
	pr_debug("BW: load: %u, rd: %u, wr: %u, pk: %u", load, bw->read, bw->write, bw->peak);
}
#endif

static int bigo_scale_bw(struct bigo_core *core)
{
	struct bts_bw bw;

	bigo_get_bw(core, &bw);
	return bts_update_bw(core->pm.bwindex, bw);
}

void bigo_mark_qos_dirty(struct bigo_core *core)
{
	mutex_lock(&core->lock);
	core->qos_dirty = true;
	mutex_unlock(&core->lock);
}

void bigo_update_qos(struct bigo_core *core)
{
	int rc;

	mutex_lock(&core->lock);
	if (core->qos_dirty) {
		rc = bigo_scale_bw(core);
		if (rc)
			pr_warn("%s: failed to scale bandwidth: %d\n", __func__, rc);
#if !IS_ENABLED(CONFIG_SOC_ZUMA)
		update_mif_floor(core);
#endif
		bigo_scale_freq(core);
		core->qos_dirty = false;
	}
	mutex_unlock(&core->lock);
}

void bigo_clocks_off(struct bigo_core *core)
{
	struct bts_bw bw;

	memset(&bw, 0, sizeof(struct bts_bw));

	mutex_lock(&core->lock);
	bts_update_bw(core->pm.bwindex, bw);
	bigo_set_freq(core, bigo_get_target_freq(core, 0));
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
