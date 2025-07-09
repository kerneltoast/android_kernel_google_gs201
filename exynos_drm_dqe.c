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
#include <trace/dpu_trace.h>

#include "exynos_drm_decon.h"

static inline u8 get_actual_dstep(u8 dstep, int vrefresh)
{
	return dstep * vrefresh / 60;
}

void exynos_atc_update(struct exynos_dqe *dqe, struct exynos_dqe_state *state)
{
	const struct exynos_drm_crtc_state *exynos_crtc_state =
		container_of(state, struct exynos_drm_crtc_state, dqe);
	const struct drm_crtc_state *crtc_state = &exynos_crtc_state->base;
	struct decon_device *decon = dqe->decon;
	struct drm_printer p = drm_info_printer(decon->dev);
	u32 id = decon->id;

	DPU_ATRACE_BEGIN(__func__);

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
	if (decon->thread)
		DPU_ATRACE_INT_PID("atc_en", dqe->force_atc_config.en, decon->thread->pid);

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

	DPU_ATRACE_END(__func__);
}
EXPORT_SYMBOL_GPL(exynos_atc_update);

/*
 * emmits event (caller should protect)
 */
static void histogram_emmit_event_locked(struct exynos_dqe *dqe,
					 struct histogram_event_node *e_node)
{
	struct drm_device *dev = dqe->decon->drm_dev;

	DPU_ATRACE_BEGIN(__func__);
	list_del(&e_node->node);
	drm_send_event(dev, e_node->base);
	kfree(e_node);
	DPU_ATRACE_END(__func__);
}

static void histogram_chan_collect_bins_locked(struct exynos_dqe *dqe,
					       enum exynos_histogram_id hist_id,
					       struct histogram_bins *bins)
{
	DPU_ATRACE_BEGIN(__func__);
	/* collect data from bins */
	DPU_EVENT_LOG(DPU_EVT_HIST_COLLECT_BINS, dqe->decon->id, &hist_id);
	dqe_reg_get_histogram_bins(dqe->dev, dqe->decon->id, hist_id, bins);
	DPU_ATRACE_END(__func__);
}

static const char *str_run_state(enum histogram_run_state state)
{
	switch (state) {
	case HSTATE_DISABLED:
		return "disabled";
	case HSTATE_HIBERNATION:
		return "hibernation";
	case HSTATE_PENDING_FRAMEDONE:
		return "pending_framedone";
	case HSTATE_IDLE:
		return "idle";
	default:
		return "";
	}
}

static void histogram_chan_set_run_state_locked(struct exynos_dqe *dqe,
						enum exynos_histogram_id hist_id,
						enum histogram_run_state state)
{
	pr_debug("histogram: run_state: %s -> %s\n",
		 str_run_state(dqe->state.hist_chan[hist_id].run_state), str_run_state(state));
	dqe->state.hist_chan[hist_id].run_state = state;
}

static struct histogram_event_node *create_histogram_event_node(struct drm_pending_event *pending_e)
{
	struct histogram_event_node *e_node = NULL;

	e_node = kzalloc(sizeof(*e_node), GFP_KERNEL);
	if (!e_node)
		return ERR_PTR(-ENOMEM);

	e_node->base = pending_e;
	INIT_LIST_HEAD(&e_node->node);
	return e_node;
}

static void release_histogram_event_node(struct drm_device *dev,
					 struct histogram_event_node *e_node)
{
	list_del(&e_node->node);
	if (e_node->base)
		drm_event_cancel_free(dev, e_node->base);
	kfree(e_node);
}

static struct exynos_drm_pending_histogram_event *create_histogram_event(struct drm_device *dev,
									 struct drm_file *file,
									 uint32_t crtc_id,
									 uint32_t hist_id)
{
	struct exynos_drm_pending_histogram_event *e = NULL;
	int ret;

	e = kzalloc(sizeof(*e), GFP_KERNEL);
	if (!e)
		return ERR_PTR(-ENOMEM);

	e->event.base.type = EXYNOS_DRM_HISTOGRAM_CHANNEL_EVENT;
	e->event.base.length = sizeof(e->event);
	e->event.crtc_id = crtc_id;
	e->event.hist_id = hist_id;

	ret = drm_event_reserve_init(dev, file, &e->base, &e->event.base);
	if (ret) {
		pr_err("drm_event_reserve_init failed, ret(%d)\n", ret);
		kfree(e);
		return ERR_PTR(ret);
	}

	return e;
}

static struct exynos_drm_pending_context_histogram_event *
create_context_histogram_event(struct drm_device *dev, struct drm_file *file, uint32_t crtc_id,
			       uint32_t user_handle)
{
	struct exynos_drm_pending_context_histogram_event *e = NULL;
	int ret;

	e = kzalloc(sizeof(*e), GFP_KERNEL);
	if (!e)
		return ERR_PTR(-ENOMEM);

	e->event.base.type = EXYNOS_DRM_CONTEXT_HISTOGRAM_EVENT;
	e->event.base.length = sizeof(e->event);
	e->event.crtc_id = crtc_id;
	e->event.user_handle = user_handle;

	ret = drm_event_reserve_init(dev, file, &e->base, &e->event.base);
	if (ret) {
		pr_err("drm_event_reserve_init failed, ret(%d)\n", ret);
		kfree(e);
		return ERR_PTR(ret);
	}

	return e;
}

/* histogram_find_event_node_locked is called with histogram_slock held */
static struct histogram_event_node *
histogram_find_event_node_locked(struct list_head *hist_pending_events_list,
				 enum exynos_histogram_id hist_id, uint32_t user_handle)
{
	struct histogram_event_node *e_node;
	struct exynos_drm_pending_histogram_event *chan_e;
	struct exynos_drm_pending_context_histogram_event *context_e;

	list_for_each_entry (e_node, hist_pending_events_list, node) {
		if (e_node->base->event->type == EXYNOS_DRM_HISTOGRAM_CHANNEL_EVENT) {
			chan_e = container_of(e_node->base,
					      struct exynos_drm_pending_histogram_event, base);
			if (chan_e->event.hist_id == hist_id && hist_id < HISTOGRAM_MAX)
				return e_node;
		} else if (e_node->base->event->type == EXYNOS_DRM_CONTEXT_HISTOGRAM_EVENT) {
			context_e = container_of(e_node->base,
						 struct exynos_drm_pending_context_histogram_event,
						 base);
			if (context_e->event.user_handle == user_handle && user_handle != 0)
				return e_node;
		}
	}

	return NULL;
}

int histogram_request_ioctl(struct drm_device *dev, void *data, struct drm_file *file)
{
	pr_err("%s: ignored\n", __func__);
	return 0;
}

int histogram_cancel_ioctl(struct drm_device *dev, void *data, struct drm_file *file)
{
	pr_err("%s: ignored\n", __func__);
	return 0;
}

static int histogram_channel_ioctl_process_arg(struct drm_device *dev, void *data,
					       struct drm_file *file, uint32_t *crtc_id,
					       enum exynos_histogram_id *hist_id,
					       struct decon_device **decon, struct exynos_dqe **dqe)
{
	struct drm_mode_object *obj;
	struct exynos_drm_crtc *exynos_crtc;
	struct exynos_drm_histogram_channel_request *request = data;

	if (!data) {
		pr_err("invalid histogram request, data is NULL\n");
		return -EINVAL;
	}

	*crtc_id = request->crtc_id;
	*hist_id = request->hist_id;
	if (*hist_id >= HISTOGRAM_MAX) {
		pr_err("invalid histogram channel id(%d)\n", *hist_id);
		return -EINVAL;
	}

	obj = drm_mode_object_find(dev, file, *crtc_id, DRM_MODE_OBJECT_CRTC);
	if (!obj) {
		pr_err("failed to find crtc object\n");
		return -ENOENT;
	}

	exynos_crtc = to_exynos_crtc(obj_to_crtc(obj));
	drm_mode_object_put(obj);

	*decon = exynos_crtc->ctx;
	*dqe = (*decon)->dqe;
	if (!*dqe) {
		pr_err("failed to get dqe from decon%u\n", (*decon)->id);
		return -ENODEV;
	}

	return 0;
}

int histogram_channel_request_ioctl(struct drm_device *dev, void *data, struct drm_file *file)
{
	struct decon_device *decon;
	struct exynos_dqe *dqe;
	unsigned long flags;
	struct histogram_event_node *e_node;
	struct exynos_drm_pending_histogram_event *e;
	enum exynos_histogram_id hist_id;
	struct histogram_chan_state *hist_chan;
	uint32_t crtc_id;
	int ret;

	/* validate the histogram ioctl argument */
	ret = histogram_channel_ioctl_process_arg(dev, data, file, &crtc_id, &hist_id, &decon,
						  &dqe);
	if (ret) {
		pr_err("histogram_channel_ioctl_process_arg failed, ret(%d)\n", ret);
		return ret;
	}

	e = create_histogram_event(dev, file, crtc_id, hist_id);
	if (IS_ERR(e)) {
		pr_err("failed to create a histogram event\n");
		return PTR_ERR(e);
	}

	e_node = create_histogram_event_node(&e->base);
	if (!e_node) {
		pr_err("failed to allocate histogram_event_node\n");
		drm_event_cancel_free(dev, &e->base);
		return -ENOMEM;
	}

	/*
	 * TODO: Now only one observer is allowed at a time at the moment.
	 * This will be allowed for multiple observer in the future.
	 */
	spin_lock_irqsave(&dqe->state.histogram_slock, flags);
	if (histogram_find_event_node_locked(&dqe->state.hist_pending_events_list, hist_id, 0)) {
		pr_warn("decon%u histogram%u already registered\n", decon->id, hist_id);
		release_histogram_event_node(dev, e_node);
		spin_unlock_irqrestore(&dqe->state.histogram_slock, flags);
		return -EBUSY;
	}

	list_add_tail(&e_node->node, &dqe->state.hist_pending_events_list);
	hist_chan = &dqe->state.hist_chan[hist_id];

	/* check cached state */
	if (hist_chan->run_state == HSTATE_HIBERNATION) {
		if (dqe->verbose_hist)
			pr_info("histogram: use cached data\n");
		memcpy(&e->event.bins, &hist_chan->bins, sizeof(e->event.bins));
		histogram_emmit_event_locked(dqe, e_node);
	} else if (hist_chan->run_state == HSTATE_IDLE) {
		if (dqe->verbose_hist)
			pr_info("histogram: idle, query now\n");
#if IS_ENABLED(CONFIG_SOC_ZUMA)
		/* need to collect into cached bins: smc requires physical memory */
		histogram_chan_collect_bins_locked(dqe, hist_id, &hist_chan->bins);
		memcpy(&e->event.bins, &hist_chan->bins, sizeof(e->event.bins));
#else
		histogram_chan_collect_bins_locked(dqe, hist_id, &e->event.bins);
#endif
		histogram_emmit_event_locked(dqe, e_node);
	}

	spin_unlock_irqrestore(&dqe->state.histogram_slock, flags);

	pr_debug("histogram: created event(0x%pK) of decon%u, chan %u\n", e, decon->id, hist_id);

	return 0;
}

int histogram_channel_cancel_ioctl(struct drm_device *dev, void *data, struct drm_file *file)
{
	struct decon_device *decon;
	struct exynos_dqe *dqe;
	unsigned long flags;
	enum exynos_histogram_id hist_id;
	struct histogram_event_node *e_node;
	uint32_t crtc_id;
	int ret;

	/* validate the histogram ioctl argument */
	ret = histogram_channel_ioctl_process_arg(dev, data, file, &crtc_id, &hist_id, &decon,
						  &dqe);
	if (ret) {
		pr_err("histogram_channel_ioctl_process_arg failed, ret(%d)\n", ret);
		return ret;
	}

	spin_lock_irqsave(&dqe->state.histogram_slock, flags);
	e_node = histogram_find_event_node_locked(&dqe->state.hist_pending_events_list, hist_id, 0);
	if (e_node) {
		pr_debug("remained event(0x%pK)\n", e_node->base);
		release_histogram_event_node(dev, e_node);
	}
	spin_unlock_irqrestore(&dqe->state.histogram_slock, flags);

	pr_debug("histogram: terminated histogram event of decon%u, chan %u\n", decon->id, hist_id);

	return 0;
}

static int histogram_event_ioctl_process_arg(struct drm_device *dev, void *data,
					     struct drm_file *file, uint32_t *crtc_id,
					     uint32_t *user_handle, struct decon_device **decon,
					     struct exynos_dqe **dqe)
{
	struct drm_mode_object *obj;
	struct exynos_drm_crtc *exynos_crtc;
	struct exynos_drm_context_histogram_arg *request = data;

	if (!data) {
		pr_err("invalid histogram request, data is NULL\n");
		return -EINVAL;
	}

	*crtc_id = request->crtc_id;
	*user_handle = request->user_handle;

	obj = drm_mode_object_find(dev, file, *crtc_id, DRM_MODE_OBJECT_CRTC);
	if (!obj) {
		pr_err("failed to find crtc object\n");
		return -ENOENT;
	}

	exynos_crtc = to_exynos_crtc(obj_to_crtc(obj));
	drm_mode_object_put(obj);

	*decon = exynos_crtc->ctx;
	*dqe = (*decon)->dqe;
	if (!*dqe) {
		pr_err("failed to get dqe from decon%u\n", (*decon)->id);
		return -ENODEV;
	}

	return 0;
}

int histogram_event_request_ioctl(struct drm_device *dev, void *data, struct drm_file *file)
{
	struct decon_device *decon;
	struct exynos_dqe *dqe;
	unsigned long flags;
	struct histogram_event_node *e_node;
	struct exynos_drm_pending_context_histogram_event *e;
	enum exynos_histogram_id hist_id;
	struct histogram_chan_state *hist_chan;
	uint32_t crtc_id, user_handle;
	int ret;

	/* validate the histogram ioctl argument */
	ret = histogram_event_ioctl_process_arg(dev, data, file, &crtc_id, &user_handle, &decon,
						&dqe);
	if (ret) {
		pr_err("histogram_event_ioctl_process_arg failed, ret(%d)\n", ret);
		return ret;
	}
	if (!user_handle) {
		pr_err("decon%u: user_handle should not be 0\n", decon->id);
		return -EINVAL;
	}

	e = create_context_histogram_event(dev, file, crtc_id, user_handle);
	if (IS_ERR(e)) {
		pr_err("create_context_histogram_event failed, ret(%ld)\n", PTR_ERR(e));
		return PTR_ERR(e);
	}

	e_node = create_histogram_event_node(&e->base);
	if (!e_node) {
		pr_err("failed to allocate histogram_event_node\n");
		drm_event_cancel_free(dev, &e->base);
		return -ENOMEM;
	}

	/*
	 * TODO: Now only one observer is allowed at a time at the moment.
	 * This will be allowed for multiple observer in the future.
	 */
	spin_lock_irqsave(&dqe->state.histogram_slock, flags);
	if (histogram_find_event_node_locked(&dqe->state.hist_pending_events_list, HISTOGRAM_MAX,
					     user_handle)) {
		pr_warn("decon%u histogram event (handle#%u) already registered\n", decon->id,
			user_handle);
		release_histogram_event_node(dev, e_node);
		spin_unlock_irqrestore(&dqe->state.histogram_slock, flags);
		return -EBUSY;
	}

	list_add_tail(&e_node->node, &dqe->state.hist_pending_events_list);
	pr_debug("request event(0x%pK)\n", e);

	/* check if any histogram channel is running this user_handle */
	hist_chan = NULL;
	for (hist_id = 0; hist_id < HISTOGRAM_MAX; hist_id++) {
		if (dqe->state.hist_chan[hist_id].user_handle == user_handle) {
			hist_chan = &dqe->state.hist_chan[hist_id];
			break;
		}
	}
	if (!hist_chan) {
		spin_unlock_irqrestore(&dqe->state.histogram_slock, flags);
		return 0;
	}

	/* check cached state */
	if (hist_chan->run_state == HSTATE_HIBERNATION) {
		if (dqe->verbose_hist)
			pr_info("histogram: use cached data\n");
		memcpy(&e->event.bins, &hist_chan->bins, sizeof(e->event.bins));
		histogram_emmit_event_locked(dqe, e_node);
	} else if (hist_chan->run_state == HSTATE_IDLE) {
		if (dqe->verbose_hist)
			pr_info("histogram: idle, query now\n");
#if IS_ENABLED(CONFIG_SOC_ZUMA)
		/* need to collect into cached bins: smc requires physical memory */
		histogram_chan_collect_bins_locked(dqe, hist_id, &hist_chan->bins);
		memcpy(&e->event.bins, &hist_chan->bins, sizeof(e->event.bins));
#else
		histogram_chan_collect_bins_locked(dqe, hist_id, &e->event.bins);
#endif
		histogram_emmit_event_locked(dqe, e_node);
	}

	spin_unlock_irqrestore(&dqe->state.histogram_slock, flags);

	return 0;
}

int histogram_event_cancel_ioctl(struct drm_device *dev, void *data, struct drm_file *file)
{
	struct decon_device *decon;
	struct exynos_dqe *dqe;
	unsigned long flags;
	struct histogram_event_node *e_node;
	uint32_t crtc_id, user_handle;
	int ret;

	/* validate the histogram ioctl argument */
	ret = histogram_event_ioctl_process_arg(dev, data, file, &crtc_id, &user_handle, &decon,
						&dqe);
	if (ret) {
		pr_err("histogram_event_ioctl_process_arg failed, ret(%d)\n", ret);
		return ret;
	}

	spin_lock_irqsave(&dqe->state.histogram_slock, flags);

	/* user_handle 0 is special request to clear all events */
	if (unlikely(!user_handle)) {
		pr_info("decon%u: clear all events\n", decon->id);
		while (!list_empty(&dqe->state.hist_pending_events_list)) {
			e_node = list_first_entry(&dqe->state.hist_pending_events_list,
						  struct histogram_event_node, node);
			release_histogram_event_node(dev, e_node);
		}

		spin_unlock_irqrestore(&dqe->state.histogram_slock, flags);
		return 0;
	}

	e_node = histogram_find_event_node_locked(&dqe->state.hist_pending_events_list,
						  HISTOGRAM_MAX, user_handle);
	if (e_node) {
		pr_debug("terminate remained event(0x%pK)\n", e_node->base);
		release_histogram_event_node(dev, e_node);
	} else {
		pr_debug("no event request for handle#%u\n", user_handle);
	}

	spin_unlock_irqrestore(&dqe->state.histogram_slock, flags);

	return 0;
}

/*
 * configure histogram channel
 */
int histogram_chan_configure(struct exynos_dqe *dqe, const enum exynos_histogram_id hist_id,
			     struct histogram_channel_config *config)
{
	struct decon_device *decon = dqe->decon;
	u32 id = decon->id;

	if (hist_id >= HISTOGRAM_MAX)
		return -EINVAL;

	dqe_reg_set_histogram_threshold(id, hist_id, config->threshold);
	dqe_reg_set_histogram_pos(id, hist_id, config->pos);
	dqe_reg_set_histogram_roi(id, hist_id, &config->roi);
	dqe_reg_set_histogram_weights(id, hist_id, &config->weights);
#if IS_ENABLED(CONFIG_SOC_ZUMA)
	if (config->flags & HISTOGRAM_FLAGS_BLOCKED_ROI)
		dqe_reg_set_histogram_block_roi(id, hist_id, &config->blocked_roi);
#endif
	return 0;
}

int histogram_chan_set_state(struct exynos_dqe *dqe, const enum exynos_histogram_id hist_id,
			     const enum histogram_state hist_state,
			     histogram_chan_callback hist_cb)
{
	struct decon_device *decon = dqe->decon;
	u32 id = decon->id;
	struct histogram_chan_state *hist_chan;

	if (!dqe || hist_id >= HISTOGRAM_MAX)
		return -EINVAL;

	hist_chan = &dqe->state.hist_chan[hist_id];

	pr_debug("decon_id=%u, hist_id=%d hist_state=%d, curr_state=%d\n",
		 id, hist_id, hist_state, hist_chan->state);

	hist_chan->cb = hist_cb;
	hist_chan->state = hist_state;
	dqe_reg_set_histogram(id, hist_id, hist_state);

	return 0;
}

static void histogram_chan_handle_event_locked(struct exynos_dqe *dqe, uint32_t hist_id,
					       bool force_collect)
{
	struct histogram_chan_state *hist_chan = &dqe->state.hist_chan[hist_id];
	histogram_chan_callback hist_cb = hist_chan->cb;
	struct histogram_event_node *e_node;
	struct histogram_bins *bins = NULL;

	e_node = histogram_find_event_node_locked(&dqe->state.hist_pending_events_list, hist_id,
						  hist_chan->user_handle);
	if (!e_node && !hist_cb && !force_collect)
		return;

	histogram_chan_collect_bins_locked(dqe, hist_id, &hist_chan->bins);

	/* handle DRM request */
	if (e_node) {
		pr_debug("decon%u histogram%u: handle event(0x%pK), rstate(%s)\n", dqe->decon->id,
			 hist_id, e_node->base->event, str_run_state(hist_chan->run_state));

		if (e_node->base->event->type == EXYNOS_DRM_HISTOGRAM_CHANNEL_EVENT) {
			bins = &container_of(e_node->base,
					     struct exynos_drm_pending_histogram_event, base)
					->event.bins;
		} else if (e_node->base->event->type == EXYNOS_DRM_CONTEXT_HISTOGRAM_EVENT) {
			bins = &container_of(e_node->base,
					     struct exynos_drm_pending_context_histogram_event,
					     base)
					->event.bins;
		} else {
			return;
		}

		memcpy(bins, &hist_chan->bins, sizeof(*bins));
		histogram_emmit_event_locked(dqe, e_node);
	}

	/* handle LHBM request. TODO: review if LHBM can be moved to DRM fw. */
	if (hist_cb)
		(hist_cb)(dqe->decon->id, hist_id, &hist_chan->bins);
}

/* This function runs in interrupt context */
void handle_histogram_event(struct exynos_dqe *dqe)
{
	uint32_t hist_id;

	spin_lock(&dqe->state.histogram_slock);

	/*
	 * histogram engine data is available after first frame done.
	 * collect data from all active channels.
	 */
	for (hist_id = 0; hist_id < HISTOGRAM_MAX; hist_id++) {
		struct histogram_chan_state *hist_chan = &dqe->state.hist_chan[hist_id];

		/* skip if histogram channel is disabled */
		if (hist_chan->run_state == HSTATE_DISABLED)
			continue;

		histogram_chan_handle_event_locked(dqe, hist_id, false);

		if ((atomic_read(&dqe->decon->frames_pending) == 0) &&
		    (dqe->decon->config.mode.op_mode != DECON_VIDEO_MODE))
			histogram_chan_set_run_state_locked(dqe, hist_id, HSTATE_IDLE);
		else
			histogram_chan_set_run_state_locked(dqe, hist_id, HSTATE_PENDING_FRAMEDONE);
	}

	spin_unlock(&dqe->state.histogram_slock);
}

void histogram_flip_done(struct exynos_dqe *dqe, const struct drm_crtc_state *new_crtc_state)
{
	unsigned long flags;
	enum exynos_histogram_id hist_id;
	const struct exynos_drm_crtc_state *new_exynos_crtc_state =
		to_exynos_crtc_state(new_crtc_state);

	DPU_ATRACE_BEGIN(__func__);
	spin_lock_irqsave(&dqe->state.histogram_slock, flags);

	for (hist_id = 0; hist_id < HISTOGRAM_MAX; hist_id++) {
		struct histogram_chan_state *hist_chan = &dqe->state.hist_chan[hist_id];
		struct drm_property_blob *blob = new_exynos_crtc_state->histogram[hist_id];

		/*
		 * For run_state is HSTATE_HIBERNATION and state is HISTOGRAM_OFF, we should keep it
		 * as HSTATE_HIBERNATION.
		 * 1. We already cache the histogram bins in memory (hist_chan->bins) and
		 *    hist_chan->state is set to OFF before entering hibernation.
		 * 2. For the first commit to exit the hibernation, the skip_update is true to avoid
		 *    most DPU updates including exynos_histogram_update. So the histogram bins
		 *    cache should still serve this case until we have next non-skip frame update
		 *    that will restore the histogram config.
		 */
		if (hist_chan->state != HISTOGRAM_OFF)
			histogram_chan_set_run_state_locked(dqe, hist_id, HSTATE_PENDING_FRAMEDONE);
		else if (hist_chan->run_state != HSTATE_HIBERNATION)
			histogram_chan_set_run_state_locked(dqe, hist_id, HSTATE_DISABLED);

		/*
		 * Update the user_handle (config blob id) when the histogram config is really
		 * applied to the DPU HW (shadow update completes and framestart occurs).
		 */
		hist_chan->user_handle = blob ? blob->base.id : 0;
	}

	spin_unlock_irqrestore(&dqe->state.histogram_slock, flags);
	DPU_ATRACE_END(__func__);
}

static void exynos_degamma_update(struct exynos_dqe *dqe, struct exynos_dqe_state *state)
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

static bool
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
		return false;

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

	return updated;
}

static void
exynos_regamma_update(struct exynos_dqe *dqe, struct exynos_dqe_state *state)
{
	struct regamma_debug_override *regamma = &dqe->regamma;
	struct exynos_debug_info *info = &regamma->info;
	struct decon_device *decon = dqe->decon;
	struct drm_printer p = drm_info_printer(decon->dev);
	u32 id = decon->id;
	u32 regamma_id = 0;

	pr_debug("en(%d) dirty(%d)\n", info->force_en, info->dirty);

	if (info->force_en)
		state->regamma_lut = regamma->force_lut;

	if (dqe->state.regamma_lut != state->regamma_lut || info->dirty) {
		dqe_reg_set_regamma_lut(id, regamma_id, state->regamma_lut);
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

#ifdef CONFIG_SOC_ZUMA
static void exynos_lhbm_histogram_callback(u32 dqe_id, enum exynos_histogram_id hist_id,
					   struct histogram_bins *hist_bins)
{
	u32 i = 0, sum = 0;
	u32 weighted_sum = 0;
	struct decon_device *decon = get_decon_drvdata(dqe_id);
	struct exynos_dqe *dqe = decon->dqe;

	if (hist_id != HISTOGRAM_CHAN_LHBM)
		return;

	/* data is u16 not u8 */
	for (i = 0; i < HISTOGRAM_BIN_COUNT; i++) {
		sum += hist_bins->data[i];
		weighted_sum += hist_bins->data[i] * i;
	}
	if (sum == 0)
		return;
	dqe->lhbm_gray_level = weighted_sum / sum;
	if (dqe->gray_level_callback_data.update_gray_level_callback &&
	    dqe->gray_level_callback_data.conn)
		dqe->gray_level_callback_data.update_gray_level_callback(
			dqe->gray_level_callback_data.conn, dqe->lhbm_gray_level);
}

static void exynos_lhbm_histogram_update(struct decon_device *decon)
{
	if (!decon || !decon->dqe || decon->dqe->lhbm_hist_config.roi.hsize == 0)
		return;

	histogram_chan_configure(decon->dqe, HISTOGRAM_CHAN_LHBM, &decon->dqe->lhbm_hist_config);
	histogram_chan_set_state(decon->dqe, HISTOGRAM_CHAN_LHBM, HISTOGRAM_ROI,
				 exynos_lhbm_histogram_callback);
	histogram_chan_set_run_state_locked(decon->dqe, HISTOGRAM_CHAN_LHBM,
					    HSTATE_PENDING_FRAMEDONE);
}
#endif

static void exynos_histogram_channel_update(struct exynos_dqe *dqe, struct exynos_dqe_state *state,
					    const enum exynos_histogram_id hist_id)
{
	enum histogram_state hist_state;
	unsigned long flags;
	struct histogram_chan_state *hist_chan = &dqe->state.hist_chan[hist_id];
	bool config_changed = false;

	if (hist_id >= HISTOGRAM_MAX)
		return;

	spin_lock_irqsave(&dqe->state.histogram_slock, flags);

	/*
	 * DRM framework histogram channel configuration
	 */
	if (hist_chan->config != state->hist_chan[hist_id].config) {
		u32 weights, roi;
		struct histogram_channel_config *config = state->hist_chan[hist_id].config;

		config_changed = true;
		hist_chan->config = config;
		if (!config) {
			histogram_chan_set_state(dqe, hist_id, HISTOGRAM_OFF, NULL);
			goto update_run_state;
		}

		histogram_chan_configure(dqe, hist_id, config);

		/* refer to values to identify histogram operation mode */
		weights = config->weights.weight_b + config->weights.weight_g +
			  config->weights.weight_r;
		roi = config->roi.hsize + config->roi.vsize;
		if (weights && roi) {
			hist_state = HISTOGRAM_ROI;
#if IS_ENABLED(CONFIG_SOC_ZUMA)
			if (config->flags & HISTOGRAM_FLAGS_BLOCKED_ROI)
				hist_state = HISTOGRAM_BLOCKED_ROI;
#endif
		} else if (weights) {
			hist_state = HISTOGRAM_FULL;
#if IS_ENABLED(CONFIG_SOC_ZUMA)
			if (config->flags & HISTOGRAM_FLAGS_BLOCKED_ROI)
				hist_state = HISTOGRAM_BLOCKED_FULL;
#endif
		} else {
			hist_state = HISTOGRAM_OFF;
		}
		histogram_chan_set_state(dqe, hist_id, hist_state, NULL);
	}

update_run_state:

	/*
	 * Since the framestart will happen very soon after decon_atomic_flush, we should prevent
	 * any risk to capture the in-between frames histogram bins. Set run_state to
	 * HSTATE_PENDING_FRAMEDONE for almost every case except no config changed and already
	 * channel disabled. histogram_flip_done will update the run_state more accurately.
	 */
	if (config_changed || hist_chan->state != HISTOGRAM_OFF)
		histogram_chan_set_run_state_locked(dqe, hist_id, HSTATE_PENDING_FRAMEDONE);

	spin_unlock_irqrestore(&dqe->state.histogram_slock, flags);
}

static void exynos_histogram_update(struct exynos_dqe *dqe, struct exynos_dqe_state *state)
{
	struct decon_device *decon = dqe->decon;
	struct drm_printer p = drm_info_printer(decon->dev);
	int i;

	for (i = 0; i < HISTOGRAM_MAX; i++)
		exynos_histogram_channel_update(dqe, state, i);

#if IS_ENABLED(CONFIG_SOC_ZUMA)
	exynos_lhbm_histogram_update(decon);
#endif

	if (dqe->verbose_hist)
		dqe_reg_print_hist(decon->id, &p);
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

static bool exynos_cgc_dma_update(struct exynos_dqe *dqe, struct exynos_dqe_state *state)
{
	struct decon_device *decon = dqe->decon;
	struct cgc_debug_override *cgc = &dqe->cgc;
	struct exynos_debug_info *info = &cgc->info;
	struct drm_printer p = drm_info_printer(decon->dev);
	u32 id = decon->id;
	bool updated = false;

	if (!decon->cgc_dma || info->force_en)
		return false;

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

	return updated;
}

static void __exynos_dqe_update(struct exynos_dqe *dqe,
				struct exynos_dqe_state *state, u32 width, u32 height)
{
	struct decon_device *decon = dqe->decon;
	u32 id = decon->id;
	bool cgc_updated;

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
	cgc_updated = exynos_cgc_update(dqe, state);
	exynos_regamma_update(dqe, state);
	exynos_dither_update(dqe, state);
	exynos_histogram_update(dqe, state);
	exynos_rcd_update(dqe, state);
	cgc_updated |= exynos_cgc_dma_update(dqe, state);

	decon->dqe_need_update = true;
	decon->cgc_need_update |= cgc_updated;

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

/*
 * operations prior to enter hibernation
 */
void exynos_dqe_hibernation_enter(struct exynos_dqe *dqe)
{
	unsigned long flags;
	enum exynos_histogram_id hist_id;
	struct histogram_chan_state *hist_chan;
	bool decon_idle;

	if (!dqe->state.enabled)
		return;

	DPU_ATRACE_BEGIN(__func__);
	spin_lock_irqsave(&dqe->state.histogram_slock, flags);
	decon_idle = decon_reg_is_idle(dqe->decon->id);

	for (hist_id = 0; hist_id < HISTOGRAM_MAX; hist_id++) {
		hist_chan = &dqe->state.hist_chan[hist_id];

		if (hist_chan->run_state == HSTATE_IDLE) {
			histogram_chan_collect_bins_locked(dqe, hist_id, &hist_chan->bins);
			histogram_chan_set_run_state_locked(dqe, hist_id, HSTATE_HIBERNATION);
		} else if (hist_chan->run_state == HSTATE_PENDING_FRAMEDONE) {
			if (!decon_idle) {
				/* mark as disabled to avoid start_pending_framedone
				 * related issues
				 */
				pr_warn("decon%u histogram%u: pending framedone during hibernation\n",
					dqe->decon->id, hist_id);
				histogram_chan_set_run_state_locked(dqe, hist_id, HSTATE_DISABLED);
			} else {
				pr_debug("decon%u histogram%u: decon is already idle\n",
					 dqe->decon->id, hist_id);
				histogram_chan_handle_event_locked(dqe, hist_id, true);
				histogram_chan_set_run_state_locked(dqe, hist_id,
								    HSTATE_HIBERNATION);
			}
		}
	}
	spin_unlock_irqrestore(&dqe->state.histogram_slock, flags);
	DPU_ATRACE_END(__func__);
}

void exynos_dqe_reset(struct exynos_dqe *dqe)
{
	int i;
	unsigned long flags;

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
	dqe->state.rcd_enabled = false;
	dqe->state.cgc_gem = NULL;
	dqe->lhbm_hist_configured = false;

	/* reflect histogram state  */
	spin_lock_irqsave(&dqe->state.histogram_slock, flags);
	for (i = 0; i < HISTOGRAM_MAX; i++) {
		struct histogram_chan_state *hist_chan = &dqe->state.hist_chan[i];

		hist_chan->config = NULL;
		hist_chan->state = HISTOGRAM_OFF;
		if (hist_chan->run_state != HSTATE_HIBERNATION) {
			histogram_chan_set_run_state_locked(dqe, i, HSTATE_DISABLED);
			hist_chan->user_handle = 0;
		}
	}
	spin_unlock_irqrestore(&dqe->state.histogram_slock, flags);
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
	atc->dim_ratio = 0xFF;
#ifdef CONFIG_SOC_ZUMA
	atc->la_w_on = true;
	atc->la_w = 0x4;
	atc->lt_calc_mode = 0x0;
	atc->gt_lamda_dstep = 0x4;
	atc->gt_lamda = 0x100;
	atc->gt_he_enable = false;
	atc->he_clip_min_0 = 0x40302010;
	atc->he_clip_min_1 = 0x80706050;
	atc->he_clip_min_2 = 0xc0b0a090;
	atc->he_clip_min_3 = 0xf0e0d0;
	atc->he_clip_max_0 = 0xa99b8970;
	atc->he_clip_max_1 = 0xd0c8bfb5;
	atc->he_clip_max_2 = 0xebe5dfd8;
	atc->he_clip_max_3 = 0xfbf6f1;
#endif
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

#if IS_ENABLED(CONFIG_SOC_ZUMA)
static ssize_t
atc_u32_store(struct exynos_dqe *dqe, u32 *val, const char *buf, size_t count)
{
	int ret;

	ret = kstrtou32(buf, 0, val);
	if (ret)
		return ret;

	dqe->force_atc_config.dirty = true;

	return count;
}
#endif

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
#define DQE_ATC_ATTR_U32_RW(_name) DQE_ATC_ATTR_RW(_name, atc_u32_store, "%u")

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
DQE_ATC_ATTR_U16_RW(dim_ratio);
#ifdef CONFIG_SOC_ZUMA
DQE_ATC_ATTR_BOOL_RW(la_w_on);
DQE_ATC_ATTR_U8_RW(la_w);
DQE_ATC_ATTR_BOOL_RW(lt_calc_mode);
DQE_ATC_ATTR_U8_RW(gt_lamda_dstep);
DQE_ATC_ATTR_U16_RW(gt_lamda);
DQE_ATC_ATTR_BOOL_RW(gt_he_enable);
DQE_ATC_ATTR_U32_RW(he_clip_min_0);
DQE_ATC_ATTR_U32_RW(he_clip_min_1);
DQE_ATC_ATTR_U32_RW(he_clip_min_2);
DQE_ATC_ATTR_U32_RW(he_clip_min_3);
DQE_ATC_ATTR_U32_RW(he_clip_max_0);
DQE_ATC_ATTR_U32_RW(he_clip_max_1);
DQE_ATC_ATTR_U32_RW(he_clip_max_2);
DQE_ATC_ATTR_U32_RW(he_clip_max_3);
#endif

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
	&dev_attr_dim_ratio.attr,
#ifdef CONFIG_SOC_ZUMA
	&dev_attr_la_w_on.attr,
	&dev_attr_la_w.attr,
	&dev_attr_lt_calc_mode.attr,
	&dev_attr_gt_lamda_dstep.attr,
	&dev_attr_gt_lamda.attr,
	&dev_attr_gt_he_enable.attr,
	&dev_attr_he_clip_min_0.attr,
	&dev_attr_he_clip_min_1.attr,
	&dev_attr_he_clip_min_2.attr,
	&dev_attr_he_clip_min_3.attr,
	&dev_attr_he_clip_max_0.attr,
	&dev_attr_he_clip_max_1.attr,
	&dev_attr_he_clip_max_2.attr,
	&dev_attr_he_clip_max_3.attr,
#endif
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
#elif defined(CONFIG_SOC_ZUMA)
	dqe_ver = DQE_V4;
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

	i = of_property_match_string(np, "reg-names", "dqe-cgc");
	if (i < 0)
		pr_debug("dqe-cgc is not supported\n");

	if (i >= 0 && of_address_to_resource(np, i, &res)) {
		pr_err("failed to get dqe cgc resource\n");
		return NULL;
	}

	dqe->cgc_regs = of_iomap(np, i);

	dqe_cgc_regs_desc_init(dqe->cgc_regs, res.start, "dqe-cgc", dqe_version, decon->id);

	dqe->funcs = &dqe_funcs;
	dqe->initialized = false;
	dqe->decon = decon;
	spin_lock_init(&dqe->state.histogram_slock);
	INIT_LIST_HEAD(&dqe->state.hist_pending_events_list);

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

	dma_coerce_mask_and_coherent(dqe->dev, DMA_BIT_MASK(64));

	return dqe;
}
