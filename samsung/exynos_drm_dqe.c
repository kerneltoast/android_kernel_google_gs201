// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 Samsung Electronics Co.Ltd
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#define pr_fmt(fmt)  "%s: " fmt, __func__

#include <linux/of_address.h>
#include <linux/device.h>
#include <drm/drm_drv.h>
#include <drm/drm_modeset_lock.h>
#include <drm/drm_atomic_helper.h>

#include <dqe_cal.h>
#include <decon_cal.h>
#include <regs-dqe.h>

#include "exynos_drm_decon.h"

static inline u8 get_actual_dstep(u8 dstep, int vrefresh)
{
	return dstep * vrefresh / 60;
}

static void
exynos_atc_update(struct exynos_dqe *dqe, struct exynos_dqe_state *state)
{
	const struct exynos_drm_crtc_state *exynos_crtc_state =
		container_of(state, struct exynos_drm_crtc_state, dqe);
	const struct drm_crtc_state *crtc_state = &exynos_crtc_state->base;
	struct decon_device *decon = dqe->decon;
	struct drm_printer p = drm_info_printer(decon->dev);
	u32 id = decon->id;

	if (drm_atomic_crtc_needs_modeset(crtc_state) || dqe->dstep_changed ||
			exynos_crtc_state->seamless_mode_changed) {
		int vrefresh = drm_mode_vrefresh(&crtc_state->mode);

		dqe->force_atc_config.actual_dstep =
			get_actual_dstep(dqe->force_atc_config.dstep, vrefresh);
		dqe->dstep_changed = false;
	}

	pr_debug("en(%d) dirty(%d) vrefresh(%d) dstep(%d/%d)\n",
			dqe->force_atc_config.en,
			dqe->force_atc_config.dirty,
			drm_mode_vrefresh(&crtc_state->mode),
			dqe->force_atc_config.dstep,
			dqe->force_atc_config.actual_dstep);

	if (dqe->force_atc_config.dirty) {
		if (dqe->force_atc_config.en) {
			dqe_reg_set_atc(id, &dqe->force_atc_config);
		} else {
			dqe_reg_set_atc(id, NULL);
		}
		dqe->force_atc_config.dirty = false;
	}

	if (dqe->verbose_atc)
		dqe_reg_print_atc(id, &p);
}

static struct exynos_drm_pending_histogram_event *create_histogram_event(
		struct drm_device *dev, struct drm_file *file)
{
	struct exynos_drm_pending_histogram_event *e = NULL;
	int ret;

	e = kzalloc(sizeof(*e), GFP_KERNEL);
	if (!e)
		return ERR_PTR(-ENOMEM);

	e->event.base.type = EXYNOS_DRM_HISTOGRAM_EVENT;
	e->event.base.length = sizeof(e->event);

	ret = drm_event_reserve_init(dev, file, &e->base, &e->event.base);
	if (ret) {
		kfree(e);
		return ERR_PTR(ret);
	}

	return e;
}

int histogram_request_ioctl(struct drm_device *dev, void *data,
				struct drm_file *file)
{
	struct drm_mode_object *obj;
	struct exynos_drm_crtc *exynos_crtc;
	struct decon_device *decon;
	struct exynos_dqe *dqe;
	uint32_t *crtc_id = data;
	struct exynos_drm_pending_histogram_event *e;
	unsigned long flags;

	obj = drm_mode_object_find(dev, file, *crtc_id, DRM_MODE_OBJECT_CRTC);
	if (!obj) {
		pr_err("failed to find crtc object\n");
		return -ENOENT;
	}

	exynos_crtc = to_exynos_crtc(obj_to_crtc(obj));
	drm_mode_object_put(obj);

	decon = exynos_crtc->ctx;
	dqe = decon->dqe;
	if (!dqe) {
		pr_err("failed to get dqe from decon%u\n", decon->id);
		return -ENODEV;
	}


	e = create_histogram_event(dev, file);
	if (IS_ERR(e)) {
		pr_err("failed to create a histogram event\n");
		return PTR_ERR(e);
	}

	/*
	 * TODO: Now only one observer is allowed at a time at the moment.
	 * This will be allowed for multiple observer in the future.
	 */
	spin_lock_irqsave(&dqe->state.histogram_slock, flags);
	if (dqe->state.event) {
		pr_warn("decon%u histogram already registered\n", decon->id);
		drm_event_cancel_free(dev, &e->base);
		spin_unlock_irqrestore(&dqe->state.histogram_slock, flags);
		return -EBUSY;
	}
	dqe->state.event = e;
	spin_unlock_irqrestore(&dqe->state.histogram_slock, flags);

	pr_debug("created histogram event(0x%pK) of decon%u\n",
			dqe->state.event, decon->id);

	return 0;
}

int histogram_cancel_ioctl(struct drm_device *dev, void *data,
				struct drm_file *file)
{
	struct drm_mode_object *obj;
	struct exynos_drm_crtc *exynos_crtc;
	struct decon_device *decon;
	struct exynos_dqe *dqe;
	uint32_t *crtc_id = data;
	unsigned long flags;

	obj = drm_mode_object_find(dev, file, *crtc_id, DRM_MODE_OBJECT_CRTC);
	if (!obj) {
		pr_err("failed to find crtc object\n");
		return -ENOENT;
	}

	exynos_crtc = to_exynos_crtc(obj_to_crtc(obj));
	drm_mode_object_put(obj);

	decon = exynos_crtc->ctx;
	dqe = decon->dqe;
	if (!dqe) {
		pr_err("failed to get dqe from decon%u\n", decon->id);
		return -ENODEV;
	}

	spin_lock_irqsave(&dqe->state.histogram_slock, flags);
	if (dqe->state.event) {
		pr_debug("remained event(0x%pK)\n", dqe->state.event);
		drm_event_cancel_free(dev, &dqe->state.event->base);
		dqe->state.event = NULL;
	}
	spin_unlock_irqrestore(&dqe->state.histogram_slock, flags);

	pr_debug("terminated histogram event of decon%u\n", decon->id);

	return 0;
}

void handle_histogram_event(struct exynos_dqe *dqe)
{
	/* This function runs in interrupt context */
	struct exynos_drm_pending_histogram_event *e;
	struct drm_device *dev = dqe->decon->drm_dev;
	uint32_t id, crtc_id;

	spin_lock(&dqe->state.histogram_slock);
	crtc_id = dqe->decon->crtc->base.base.id;
	id = dqe->decon->id;
	e = dqe->state.event;
	if (e) {
		pr_debug("Histogram event(0x%pK) will be handled\n", dqe->state.event);
		dqe_reg_get_histogram_bins(id, &e->event.bins);
		e->event.crtc_id = crtc_id;
		drm_send_event(dev, &e->base);
		pr_debug("histogram event of decon%u signalled\n", dqe->decon->id);
		dqe->state.event = NULL;
	}
	spin_unlock(&dqe->state.histogram_slock);
}

static void
exynos_degamma_update(struct exynos_dqe *dqe, struct exynos_dqe_state *state)
{
	struct degamma_debug_override *degamma = &dqe->degamma;
	struct exynos_debug_info *info = &degamma->info;
	struct decon_device *decon = dqe->decon;
	struct drm_printer p = drm_info_printer(decon->dev);
	u32 id = decon->id;

	pr_debug("en(%d) dirty(%d)\n", info->force_en, info->dirty);

	if (info->force_en)
		state->degamma_lut = degamma->force_lut;

	if (dqe->state.degamma_lut != state->degamma_lut || info->dirty) {
		dqe_reg_set_degamma_lut(id, state->degamma_lut);
		dqe->state.degamma_lut = state->degamma_lut;
		info->dirty = false;
	}

	if (info->verbose)
		dqe_reg_print_degamma_lut(id, &p);
}

static void
exynos_cgc_update(struct exynos_dqe *dqe, struct exynos_dqe_state *state)
{
	struct cgc_debug_override *cgc = &dqe->cgc;
	struct exynos_debug_info *info = &cgc->info;
	struct decon_device *decon = dqe->decon;
	struct drm_printer p = drm_info_printer(decon->dev);
	u32 id = decon->id;
	bool updated = false;

	pr_debug("en(%d) dirty(%d)\n", info->force_en, info->dirty);
	if (decon->cgc_dma && !info->force_en)
		return;

	if (info->force_en)
		state->cgc_lut = &cgc->force_lut;

	if (dqe->state.cgc_lut != state->cgc_lut || info->dirty) {
		dqe_reg_set_cgc_lut(id, state->cgc_lut);
		dqe->state.cgc_lut = state->cgc_lut;
		cgc->first_write = true;
		info->dirty = false;
		updated = true;
	} else if (cgc->first_write) {
		dqe_reg_set_cgc_lut(id, dqe->state.cgc_lut);
		cgc->first_write = false;
		updated = true;
	}

	if (info->verbose)
		dqe_reg_print_cgc_lut(id, cgc->verbose_cnt, &p);

	if (updated)
		decon_reg_update_req_cgc(id);
}

static void
exynos_regamma_update(struct exynos_dqe *dqe, struct exynos_dqe_state *state)
{
	struct regamma_debug_override *regamma = &dqe->regamma;
	struct exynos_debug_info *info = &regamma->info;
	struct decon_device *decon = dqe->decon;
	struct drm_printer p = drm_info_printer(decon->dev);
	u32 id = decon->id;

	pr_debug("en(%d) dirty(%d)\n", info->force_en, info->dirty);

	if (info->force_en)
		state->regamma_lut = regamma->force_lut;

	if (dqe->state.regamma_lut != state->regamma_lut || info->dirty) {
		dqe_reg_set_regamma_lut(id, state->regamma_lut);
		dqe->state.regamma_lut = state->regamma_lut;
		info->dirty = false;
	}

	if (info->verbose)
		dqe_reg_print_regamma_lut(id, &p);
}

static void exynos_gamma_matrix_update(struct exynos_dqe *dqe,
					struct exynos_dqe_state *state)
{
	struct matrix_debug_override *gamma = &dqe->gamma;
	struct exynos_debug_info *info = &gamma->info;
	struct decon_device *decon = dqe->decon;
	struct drm_printer p = drm_info_printer(decon->dev);
	u32 id = decon->id;

	pr_debug("en(%d) dirty(%d)\n", info->force_en, info->dirty);

	if (info->force_en)
		state->gamma_matrix = &gamma->force_matrix;

	if (dqe->state.gamma_matrix != state->gamma_matrix || info->dirty) {
		dqe_reg_set_gamma_matrix(id, state->gamma_matrix);
		dqe->state.gamma_matrix = state->gamma_matrix;
		info->dirty = false;
	}

	if (info->verbose)
		dqe_reg_print_gamma_matrix(id, &p);
}

static void exynos_linear_matrix_update(struct exynos_dqe *dqe,
					struct exynos_dqe_state *state)
{
	struct matrix_debug_override *linear = &dqe->linear;
	struct exynos_debug_info *info = &linear->info;
	struct decon_device *decon = dqe->decon;
	struct drm_printer p = drm_info_printer(decon->dev);
	u32 id = decon->id;

	pr_debug("en(%d) dirty(%d)\n", info->force_en, info->dirty);

	if (info->force_en)
		state->linear_matrix = &linear->force_matrix;

	if (dqe->state.linear_matrix != state->linear_matrix || info->dirty) {
		dqe_reg_set_linear_matrix(id, state->linear_matrix);
		dqe->state.linear_matrix = state->linear_matrix;
		info->dirty = false;
	}

	if (info->verbose)
		dqe_reg_print_linear_matrix(id, &p);
}

static void
exynos_dither_update(struct exynos_dqe *dqe, struct exynos_dqe_state *state)
{
	struct decon_device *decon = dqe->decon;
	struct drm_printer p = drm_info_printer(decon->dev);
	u32 id = decon->id;

	if (dqe->cgc_dither_override.force_en) {
		dqe_reg_set_cgc_dither(id, &dqe->cgc_dither_override.val);
		dqe->state.cgc_dither_config = &dqe->cgc_dither_override.val;
	} else if (dqe->state.cgc_dither_config != state->cgc_dither_config) {
		dqe_reg_set_cgc_dither(id, state->cgc_dither_config);
		dqe->state.cgc_dither_config = state->cgc_dither_config;
	}

	if (dqe->cgc_dither_override.verbose)
		dqe_reg_print_dither(id, CGC_DITHER, &p);

	if (dqe->disp_dither_override.force_en) {
		dqe_reg_set_disp_dither(id, &dqe->disp_dither_override.val);
		dqe->state.disp_dither_config = &dqe->disp_dither_override.val;
	} else if (!state->disp_dither_config) {
		struct dither_config dither_config;

		memset(&dither_config, 0, sizeof(dither_config));
		if (decon->config.in_bpc == 10 && decon->config.out_bpc == 8)
			dither_config.en = DITHER_EN(1);
		else
			dither_config.en = DITHER_EN(0);

		dqe_reg_set_disp_dither(id, &dither_config);
		dqe->state.disp_dither_config = NULL;
	} else if (dqe->state.disp_dither_config != state->disp_dither_config) {
		if (decon->config.in_bpc == decon->config.out_bpc &&
			state->disp_dither_config->en == DITHER_EN(1)) {
			pr_warn("try to enable disp_dither while in_bpc "
				"== out_bpc, correct it to \"disable\"\n");
			state->disp_dither_config->en = DITHER_EN(0);
		}
		dqe_reg_set_disp_dither(id, state->disp_dither_config);
		dqe->state.disp_dither_config = state->disp_dither_config;
	}

	if (dqe->disp_dither_override.verbose)
		dqe_reg_print_dither(id, DISP_DITHER, &p);
}

static void
exynos_histogram_update(struct exynos_dqe *dqe, struct exynos_dqe_state *state)
{
	enum histogram_state hist_state;
	struct decon_device *decon = dqe->decon;
	struct drm_printer p = drm_info_printer(decon->dev);
	u32 id = decon->id;

	if (dqe->state.roi != state->roi) {
		dqe_reg_set_histogram_roi(id, state->roi);
		dqe->state.roi = state->roi;
	}

	if (dqe->state.weights != state->weights) {
		dqe_reg_set_histogram_weights(id, state->weights);
		dqe->state.weights = state->weights;
	}

	if (dqe->state.histogram_threshold != state->histogram_threshold) {
		dqe_reg_set_histogram_threshold(id, state->histogram_threshold);
		dqe->state.histogram_threshold = state->histogram_threshold;
	}

	if (dqe->state.histogram_pos != state->histogram_pos) {
		dqe_reg_set_histogram_pos(id, state->histogram_pos);
		dqe->state.histogram_pos = state->histogram_pos;
	}

	if (dqe->state.event && state->roi)
		hist_state = HISTOGRAM_ROI;
	else if (dqe->state.event && !state->roi)
		hist_state = HISTOGRAM_FULL;
	else
		hist_state = HISTOGRAM_OFF;

	dqe_reg_set_histogram(id, hist_state);

	if (dqe->verbose_hist)
		dqe_reg_print_hist(id, &p);
}

static void exynos_rcd_update(struct exynos_dqe *dqe, struct exynos_dqe_state *state)
{
	const struct decon_device *decon = dqe->decon;
	const u32 id = decon->id;

	if (state->rcd_enabled != dqe->state.rcd_enabled) {
		dqe_reg_set_rcd_en(id, state->rcd_enabled);
		decon_reg_set_rcd_enable(id, state->rcd_enabled);
		dqe->state.rcd_enabled = state->rcd_enabled;
	}
}

#define CGC_DMA_REQ_TIMEOUT_US 300
static void exynos_set_cgc_dma(struct decon_device *decon, struct exynos_dqe_state *state)
{
	struct exynos_drm_gem *exynos_cgc_gem;
	u32 id = decon->id;
	u32 cgc_dma_id = decon->cgc_dma->id;

	if (!state->cgc_gem) {
		dqe_reg_set_cgc_en(id, 0);
		cgc_reg_set_config(cgc_dma_id, 0, 0);
	} else {
		dqe_reg_set_cgc_en(id, 1);
		exynos_cgc_gem = to_exynos_gem(state->cgc_gem);
		cgc_reg_set_config(cgc_dma_id, 1, exynos_cgc_gem->dma_addr);
		dqe_reg_set_cgc_coef_dma_req(id);
		cgc_reg_set_cgc_start(cgc_dma_id);
		dqe_reg_wait_cgc_dma_done(id, CGC_DMA_REQ_TIMEOUT_US);
	}
}

static void exynos_cgc_dma_update(struct exynos_dqe *dqe,
					struct exynos_dqe_state *state)
{
	struct decon_device *decon = dqe->decon;
	struct cgc_debug_override *cgc = &dqe->cgc;
	struct exynos_debug_info *info = &cgc->info;
	struct drm_printer p = drm_info_printer(decon->dev);
	u32 id = decon->id;
	bool updated = false;

	if (!decon->cgc_dma || info->force_en)
		return;

	if (dqe->state.cgc_gem != state->cgc_gem) {
		exynos_set_cgc_dma(decon, state);
		cgc->first_write = true;
		updated = true;
	} else if (cgc->first_write) {
		exynos_set_cgc_dma(decon, state);
		cgc->first_write = false;
		updated = true;
	}

	if (info->verbose)
		dqe_reg_print_cgc_lut(id, cgc->verbose_cnt, &p);

	if (updated)
		decon_reg_update_req_cgc(id);
}

static void __exynos_dqe_update(struct exynos_dqe *dqe,
		struct exynos_dqe_state *state, u32 width, u32 height)
{
	const struct decon_device *decon = dqe->decon;
	u32 id = decon->id;

	pr_debug("enabled(%d) +\n", state->enabled);

	dqe->state.enabled = state->enabled && !dqe->force_disabled;

	decon_reg_set_dqe_enable(id, dqe->state.enabled);
	if (!dqe->state.enabled)
		return;

	if (!dqe->initialized) {
		dqe_reg_init(id, width, height);
		dqe->initialized = true;
	}

	exynos_atc_update(dqe, state);

	exynos_gamma_matrix_update(dqe, state);
	exynos_degamma_update(dqe, state);
	exynos_linear_matrix_update(dqe, state);
	exynos_cgc_update(dqe, state);
	exynos_regamma_update(dqe, state);
	exynos_dither_update(dqe, state);
	exynos_histogram_update(dqe, state);
	exynos_rcd_update(dqe, state);
	exynos_cgc_dma_update(dqe, state);

	decon_reg_update_req_dqe(id);

	pr_debug("-\n");
}

static const struct exynos_dqe_funcs dqe_funcs = {
	.update = __exynos_dqe_update,
};

void exynos_dqe_update(struct exynos_dqe *dqe, struct exynos_dqe_state *state,
		u32 width, u32 height)
{
	dqe->funcs->update(dqe, state, width, height);
}

void exynos_dqe_reset(struct exynos_dqe *dqe)
{
	dqe->initialized = false;
	dqe->state.gamma_matrix = NULL;
	dqe->state.degamma_lut = NULL;
	dqe->state.linear_matrix = NULL;
	dqe->state.cgc_lut = NULL;
	dqe->state.regamma_lut = NULL;
	dqe->state.disp_dither_config = NULL;
	dqe->state.cgc_dither_config = NULL;
	dqe->cgc.first_write = false;
	dqe->force_atc_config.dirty = true;
	dqe->state.histogram_threshold = 0;
	dqe->state.histogram_pos = POST_DQE;
	dqe->state.roi = NULL;
	dqe->state.weights = NULL;
	dqe->state.rcd_enabled = false;
	dqe->state.cgc_gem = NULL;
}

void exynos_dqe_save_lpd_data(struct exynos_dqe *dqe)
{
	if (!dqe)
		return;

	if (dqe->force_atc_config.en)
		dqe_reg_save_lpd_atc(dqe->decon->id, dqe->lpd_atc_regs);
}

void exynos_dqe_restore_lpd_data(struct exynos_dqe *dqe)
{
	if (!dqe)
		return;

	if (dqe->force_atc_config.en)
		dqe_reg_restore_lpd_atc(dqe->decon->id, dqe->lpd_atc_regs);
}

static void set_default_atc_config(struct exynos_atc *atc)
{
	atc->dirty = true;
	atc->lt = 0x80;
	atc->ns = 0x80;
	atc->st = 0x80;
	atc->dither = false;
	atc->pl_w1 = 0xA;
	atc->pl_w2 = 0xE;
	atc->ctmode = 0x2;
	atc->pp_en = true;
	atc->upgrade_on = 0;
	atc->tdr_max = 0x384;
	atc->tdr_min = 0x100;
	atc->ambient_light = 0x8C;
	atc->back_light = 0xFF;
	atc->dstep = 0x4;
	atc->actual_dstep = 0x4;
	atc->scale_mode = 0x1;
	atc->threshold_1 = 0x1;
	atc->threshold_2 = 0x1;
	atc->threshold_3 = 0x1;
	atc->gain_limit = 0x1FF;
	atc->lt_calc_ab_shift = 0x1;
}

static ssize_t
atc_u8_store(struct exynos_dqe *dqe, u8 *val, const char *buf, size_t count)
{
	int ret;

	ret = kstrtou8(buf, 0, val);
	if (ret)
		return ret;

	dqe->force_atc_config.dirty = true;

	return count;
}

static ssize_t
atc_u16_store(struct exynos_dqe *dqe, u16 *val, const char *buf, size_t count)
{
	int ret;

	ret = kstrtou16(buf, 0, val);
	if (ret)
		return ret;

	dqe->force_atc_config.dirty = true;

	return count;
}

static ssize_t
atc_bool_store(struct exynos_dqe *dqe, bool *val, const char *buf, size_t count)
{
	if (kstrtobool(buf, val))
		return -EINVAL;

	dqe->force_atc_config.dirty = true;

	return count;
}

#define DQE_ATC_ATTR_RW(_name, _save, _fmt)	\
static ssize_t _name##_store(struct device *dev,	\
		struct device_attribute *attr, const char *buf, size_t count) \
{	\
	struct exynos_dqe *dqe = dev_get_drvdata(dev);	\
	return _save(dqe, &dqe->force_atc_config._name, buf, count);	\
}	\
static ssize_t _name##_show(struct device *dev,	\
		struct device_attribute *attr, char *buf)	\
{	\
	struct exynos_dqe *dqe = dev_get_drvdata(dev);	\
	return snprintf(buf, PAGE_SIZE, _fmt "\n",	\
			dqe->force_atc_config._name);	\
}	\
static DEVICE_ATTR_RW(_name)

#define DQE_ATC_ATTR_U8_RW(_name) DQE_ATC_ATTR_RW(_name, atc_u8_store, "%u")
#define DQE_ATC_ATTR_U16_RW(_name) DQE_ATC_ATTR_RW(_name, atc_u16_store, "%u")
#define DQE_ATC_ATTR_BOOL_RW(_name) DQE_ATC_ATTR_RW(_name, atc_bool_store, "%d")

DQE_ATC_ATTR_BOOL_RW(en);
DQE_ATC_ATTR_U8_RW(lt);
DQE_ATC_ATTR_U8_RW(ns);
DQE_ATC_ATTR_U8_RW(st);
DQE_ATC_ATTR_BOOL_RW(dither);
DQE_ATC_ATTR_U8_RW(pl_w1);
DQE_ATC_ATTR_U8_RW(pl_w2);
DQE_ATC_ATTR_U8_RW(ctmode);
DQE_ATC_ATTR_BOOL_RW(pp_en);
DQE_ATC_ATTR_U8_RW(upgrade_on);
DQE_ATC_ATTR_U16_RW(tdr_max);
DQE_ATC_ATTR_U16_RW(tdr_min);
DQE_ATC_ATTR_U8_RW(ambient_light);
DQE_ATC_ATTR_U8_RW(back_light);
DQE_ATC_ATTR_U8_RW(scale_mode);
DQE_ATC_ATTR_U8_RW(threshold_1);
DQE_ATC_ATTR_U8_RW(threshold_2);
DQE_ATC_ATTR_U8_RW(threshold_3);
DQE_ATC_ATTR_U16_RW(gain_limit);
DQE_ATC_ATTR_U8_RW(lt_calc_ab_shift);

static ssize_t force_update_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct exynos_dqe *dqe = dev_get_drvdata(dev);
	struct decon_device *decon = dqe->decon;
	struct drm_crtc *crtc = &decon->crtc->base;
	struct drm_device *drm_dev = decon->drm_dev;
	struct drm_atomic_state *state;
	struct drm_crtc_state *crtc_state;
	struct drm_modeset_acquire_ctx ctx;
	int ret = 0;

	dqe->force_atc_config.dirty = true;

	state = drm_atomic_state_alloc(drm_dev);
	if (!state)
		return -ENOMEM;
	drm_modeset_acquire_init(&ctx, 0);
	state->acquire_ctx = &ctx;
retry:

	crtc_state = drm_atomic_get_crtc_state(state, crtc);
	if (IS_ERR(crtc_state)) {
		ret = PTR_ERR(crtc_state);
		goto out;
	}
	ret = drm_atomic_commit(state);
out:
	if (ret == -EDEADLK) {
		drm_atomic_state_clear(state);
		ret = drm_modeset_backoff(&ctx);
		if (!ret)
			goto retry;
	}
	drm_atomic_state_put(state);
	drm_modeset_drop_locks(&ctx);
	drm_modeset_acquire_fini(&ctx);

	return ret ? : count;
}
static DEVICE_ATTR_WO(force_update);

static ssize_t dstep_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct exynos_dqe *dqe = dev_get_drvdata(dev);
	const struct exynos_atc *atc = &dqe->force_atc_config;

	return snprintf(buf, PAGE_SIZE, "dstep(%u), actual dstep(%u), vrefresh(%d)\n",
			atc->dstep, atc->actual_dstep, dqe->decon->bts.fps);
}

static ssize_t dstep_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct exynos_dqe *dqe = dev_get_drvdata(dev);
	struct exynos_atc *atc = &dqe->force_atc_config;
	int err;

	err = atc_u8_store(dqe, &atc->dstep, buf, count);
	if (err < 0)
		return err;

	dqe->dstep_changed = true;

	return count;
}
static DEVICE_ATTR_RW(dstep);

static struct attribute *atc_attrs[] = {
	&dev_attr_force_update.attr,
	&dev_attr_en.attr,
	&dev_attr_lt.attr,
	&dev_attr_ns.attr,
	&dev_attr_st.attr,
	&dev_attr_dither.attr,
	&dev_attr_pl_w1.attr,
	&dev_attr_pl_w2.attr,
	&dev_attr_ctmode.attr,
	&dev_attr_pp_en.attr,
	&dev_attr_upgrade_on.attr,
	&dev_attr_tdr_max.attr,
	&dev_attr_tdr_min.attr,
	&dev_attr_ambient_light.attr,
	&dev_attr_back_light.attr,
	&dev_attr_dstep.attr,
	&dev_attr_scale_mode.attr,
	&dev_attr_threshold_1.attr,
	&dev_attr_threshold_2.attr,
	&dev_attr_threshold_3.attr,
	&dev_attr_gain_limit.attr,
	&dev_attr_lt_calc_ab_shift.attr,
	NULL,
};
ATTRIBUTE_GROUPS(atc);

extern u32 gs_chipid_get_type(void);
static enum dqe_version exynos_get_dqe_version(void)
{
	enum dqe_version dqe_ver = DQE_V1;

	/* TODO : when gs_chipid_get_product_id function is created, it will be changed. */
#if defined(CONFIG_SOC_GS101)
	dqe_ver = gs_chipid_get_type() ? DQE_V2 : DQE_V1;
#elif defined(CONFIG_SOC_GS201)
	dqe_ver = DQE_V3;
#else
	#error "Unknown DQE version."
#endif

	return dqe_ver;
}

#define MAX_DQE_NAME_SIZE 10
struct exynos_dqe *exynos_dqe_register(struct decon_device *decon)
{
	struct resource res;
	struct device *dev = decon->dev;
	struct device_node *np = dev->of_node;
	struct exynos_dqe *dqe;
	enum dqe_version dqe_version;
	int i;
	char dqe_name[MAX_DQE_NAME_SIZE] = "dqe";

	i = of_property_match_string(np, "reg-names", "dqe");
	if (i < 0) {
		pr_info("display quality enhancer is not supported\n");
		return NULL;
	}
	if (of_address_to_resource(np, i, &res)) {
		pr_err("failed to get dqe resource\n");
		return NULL;
	}

	dqe = devm_kzalloc(dev, sizeof(struct exynos_dqe), GFP_KERNEL);
	if (!dqe)
		return NULL;

	dqe->regs = of_iomap(np, i);

	if (IS_ERR(dqe->regs)) {
		pr_err("failed to remap dqe registers\n");
		return NULL;
	}

	dqe_version = exynos_get_dqe_version();
	dqe_regs_desc_init(dqe->regs, res.start, "dqe", dqe_version, decon->id);
	dqe->funcs = &dqe_funcs;
	dqe->initialized = false;
	dqe->decon = decon;
	spin_lock_init(&dqe->state.histogram_slock);

	scnprintf(dqe_name, MAX_DQE_NAME_SIZE, "dqe%u", decon->id);
	dqe->dqe_class = class_create(THIS_MODULE, dqe_name);
	if (IS_ERR(dqe->dqe_class)) {
		pr_err("failed to create dqe class\n");
		return NULL;
	}

	dqe->dqe_class->dev_groups = atc_groups;
	dqe->dev = device_create(dqe->dqe_class, dev, 0, dqe, "atc");
	if (IS_ERR(dqe->dev)) {
		pr_err("failed to create to atc sysfs device\n");
		return NULL;
	}

	set_default_atc_config(&dqe->force_atc_config);

	pr_info("display quality enhancer is supported(DQE_V%d)\n",
			dqe_version + 1);

	return dqe;
}
