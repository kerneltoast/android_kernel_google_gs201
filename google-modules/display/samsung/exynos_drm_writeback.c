// SPDX-License-Identifier: GPL-2.0-only
/* exynos_drm_writeback.c
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 * Authors:
 *	Wonyeong Choi <won0.choi@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/component.h>
#include <linux/irq.h>
#include <linux/videodev2_exynos_media.h>
#include <linux/dma-buf.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#include <drm/exynos_drm.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_edid.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_fourcc_gs101.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_probe_helper.h>

#include <regs-dpp.h>

#include "exynos_drm_crtc.h"
#include "exynos_drm_decon.h"
#include "exynos_drm_dsim.h"
#include "exynos_drm_fb.h"
#include "exynos_drm_format.h"
#include "exynos_drm_writeback.h"

static const struct drm_display_mode exynos_drm_writeback_modes[] = {
	{ DRM_MODE("2400x1080", DRM_MODE_TYPE_DRIVER, 165984, 2400, 2432,
		   2444, 2470, 0, 1080, 1092, 1096, 1120, 0,
		   DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_NVSYNC) },
};

void wb_dump(struct drm_printer *p, struct writeback_device *wb)
{
	if (wb->state != WB_STATE_ON) {
		pr_info("writeback state is off\n");
		return;
	}

	__dpp_dump(p, wb->id, wb->regs.dpp_base_regs, wb->regs.dma_base_regs,
		   NULL, NULL, NULL, wb->attr);
}

static const uint32_t writeback_formats[] = {
	/* TODO : add DRM_FORMAT_RGBA1010102 */
	DRM_FORMAT_RGBA8888,
	DRM_FORMAT_NV12,
};

static const struct of_device_id wb_of_match[] = {
	{
		.compatible = "samsung,exynos-writeback",
		/* TODO : check odma, wbmux, decon_win restriction */
		.data = &dpp_drv_data,
	},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, wb_of_match);

static int exynos_drm_add_writeback_modes(struct drm_connector *connector)
{
	int i, count, num_modes = 0;
	struct drm_display_mode *mode;
	struct drm_device *dev = connector->dev;

	count = ARRAY_SIZE(exynos_drm_writeback_modes);

	for (i = 0; i < count; i++) {
		const struct drm_display_mode *ptr = &exynos_drm_writeback_modes[i];

		mode = drm_mode_duplicate(dev, ptr);
		if (mode) {
			drm_mode_probed_add(connector, mode);
			num_modes++;
		}
	}

	return num_modes;
}

static int writeback_get_modes(struct drm_connector *connector)
{
	struct drm_device *dev = connector->dev;
	int num_modes = 0;

	num_modes = drm_add_modes_noedid(connector, dev->mode_config.max_width,
				    dev->mode_config.max_height);
	num_modes += exynos_drm_add_writeback_modes(connector);

	return num_modes;
}

static void wb_convert_connector_state_to_config(struct dpp_params_info *config,
				const struct exynos_drm_writeback_state *state)
{
	struct drm_framebuffer *fb = state->base.writeback_job->fb;
	const struct drm_crtc_state *crtc_state = state->base.crtc->state;
	unsigned long long comp_blk_size;

	pr_debug("%s +\n", __func__);

	config->src.x = 0;
	config->src.y = 0;
	config->src.w = crtc_state->mode.hdisplay;
	config->src.h = crtc_state->mode.vdisplay;
	config->src.f_w = fb->width;
	config->src.f_h = fb->height;


	if (has_all_bits(DRM_FORMAT_MOD_SAMSUNG_SBWC(0), fb->modifier)) {
		config->comp_type = COMP_TYPE_SBWC;
		comp_blk_size = SBWC_BLOCK_SIZE_GET(fb->modifier);
		switch (comp_blk_size) {
		case SBWC_FORMAT_MOD_BLOCK_SIZE_32x2:
			config->blk_size = SBWC_BLK_32x2;
			break;
		case SBWC_FORMAT_MOD_BLOCK_SIZE_32x3:
			config->blk_size = SBWC_BLK_32x3;
			break;
		case SBWC_FORMAT_MOD_BLOCK_SIZE_32x4:
			config->blk_size = SBWC_BLK_32x4;
			break;
		case SBWC_FORMAT_MOD_BLOCK_SIZE_32x5:
			config->blk_size = SBWC_BLK_32x5;
			break;
		case SBWC_FORMAT_MOD_BLOCK_SIZE_32x6:
			config->blk_size = SBWC_BLK_32x6;
			break;
		default:
			pr_err("SBWC Block Size(%llu) is not supported\n", comp_blk_size);
			config->blk_size = SBWC_BLK_32x2;
			break;
		}
	} else {
		config->comp_type = COMP_TYPE_NONE;
	}

	config->format = fb->format->format;
	config->standard = state->standard;
	config->range = state->range;
	config->y_hd_y2_stride = 0;
	config->y_pl_c2_stride = 0;
	config->c_hd_stride = 0;
	config->c_pl_stride = 0;

	config->addr[0] = exynos_drm_fb_dma_addr(fb, 0);

	if (has_all_bits(DRM_FORMAT_MOD_SAMSUNG_SBWC(0), fb->modifier)) {
		const struct dpu_fmt *fmt_info = dpu_find_fmt_info(config->format);
		bool is_10bpc = IS_10BPC(fmt_info);

		config->addr[0] += Y_PL_SIZE_SBWC(config->src.f_w,
				config->src.f_h, is_10bpc);
		config->y_hd_y2_stride = HD_STRIDE_SIZE_SBWC(config->src.f_w);

		config->addr[1] = exynos_drm_fb_dma_addr(fb, 0);
		config->y_pl_c2_stride = PL_STRIDE_SIZE_SBWC(config->src.f_w,
				is_10bpc);

		config->addr[2] = exynos_drm_fb_dma_addr(fb, 1) +
			UV_PL_SIZE_SBWC(config->src.f_w, config->src.f_h,
					is_10bpc);
		config->c_hd_stride = HD_STRIDE_SIZE_SBWC(config->src.f_w);

		config->addr[3] = exynos_drm_fb_dma_addr(fb, 1);
		config->c_pl_stride = PL_STRIDE_SIZE_SBWC(config->src.f_w,
				is_10bpc);
	} else {
		config->addr[1] = exynos_drm_fb_dma_addr(fb, 1);
		config->addr[2] = exynos_drm_fb_dma_addr(fb, 2);
		config->addr[3] = exynos_drm_fb_dma_addr(fb, 3);
	}

	/* TODO: blocking mode will be implemented later */
	config->is_block = false;
	/* TODO: very big count.. recovery will be not working... */
	config->rcv_num = 0x7FFFFFFF;

	pr_debug("%s -\n", __func__);
}

static int writeback_atomic_check(struct drm_encoder *encoder,
				struct drm_crtc_state *crtc_state,
				struct drm_connector_state *conn_state)
{
	const struct drm_framebuffer *fb;
	int i;

	conn_state->self_refresh_aware = true;

	if (!wb_check_job(conn_state))
		return 0;

	fb = conn_state->writeback_job->fb;

	for (i = 0; i < ARRAY_SIZE(writeback_formats); i++)
		if (fb->format->format == writeback_formats[i])
			break;

	if (i == ARRAY_SIZE(writeback_formats))
		return -EINVAL;

	return 0;
}

static void writeback_atomic_commit(struct drm_connector *connector,
		struct drm_atomic_state *atomic_state)
{
	struct drm_connector_state *state = drm_atomic_get_new_connector_state(atomic_state, connector);
	struct drm_writeback_connector *wb_conn = conn_to_wb_conn(connector);
	struct writeback_device *wb = conn_to_wb_dev(connector);
	struct dpp_params_info *config = &wb->win_config;

	pr_debug("%s +\n", __func__);

	if (wb->state == WB_STATE_OFF) {
		pr_info("writeback(dpp%d) disabled(%d)\n", wb->id, wb->state);
		return;
	}

	wb_convert_connector_state_to_config(config, to_exynos_wb_state(state));
	dpp_reg_configure_params(wb->id, config, wb->attr);
	drm_writeback_queue_job(wb_conn, state);

	DPU_EVENT_LOG(DPU_EVT_WB_ATOMIC_COMMIT, wb->decon_id, wb);

	pr_debug("%s -\n", __func__);
}

static const struct drm_connector_helper_funcs wb_connector_helper_funcs = {
	.get_modes = writeback_get_modes,
	.atomic_commit = writeback_atomic_commit,
};

/* TODO: check the func */
static struct drm_connector_state *
exynos_drm_writeback_duplicate_state(struct drm_connector *connector)
{
	struct exynos_drm_writeback_state *exynos_state;
	struct exynos_drm_writeback_state *copy;

	pr_debug("%s +\n", __func__);

	exynos_state = to_exynos_wb_state(connector->state);
	copy = kzalloc(sizeof(*exynos_state), GFP_KERNEL);
	if (!copy)
		return NULL;

	memcpy(copy, exynos_state, sizeof(*exynos_state));
	__drm_atomic_helper_connector_duplicate_state(connector, &copy->base);

	pr_debug("%s -\n", __func__);

	return &copy->base;
}

static void exynos_drm_writeback_destroy_state(struct drm_connector *connector,
		struct drm_connector_state *old_state)
{
	struct exynos_drm_writeback_state *old_exynos_state =
						to_exynos_wb_state(old_state);

	pr_debug("%s +\n", __func__);

	__drm_atomic_helper_connector_destroy_state(old_state);
	kfree(old_exynos_state);

	pr_debug("%s -\n", __func__);
}

static void exynos_drm_writeback_reset(struct drm_connector *connector)
{
	struct exynos_drm_writeback_state *exynos_state;

	pr_debug("%s +\n", __func__);

	if (connector->state) {
		exynos_drm_writeback_destroy_state(connector, connector->state);
		connector->state = NULL;
	}

	exynos_state = kzalloc(sizeof(*exynos_state), GFP_KERNEL);
	if (exynos_state) {
		connector->state = &exynos_state->base;
		connector->state->connector = connector;
		/*
		 * TODO: If it needs to initialize the specific property value,
		 * please add to here.
		 */
	}

	pr_debug("%s -\n", __func__);
}

static void exynos_drm_writeback_destroy(struct drm_connector *connector)
{
	struct writeback_device *wb = conn_to_wb_dev(connector);

	drm_connector_cleanup(connector);
	kfree(wb);
}

static int exynos_drm_writeback_set_property(struct drm_connector *connector,
				struct drm_connector_state *state,
				struct drm_property *property, uint64_t val)
{
	struct writeback_device *wb = conn_to_wb_dev(connector);
	struct exynos_drm_writeback_state *exynos_state =
						to_exynos_wb_state(state);

	if (property == wb->props.standard)
		exynos_state->standard = val;
	else if (property == wb->props.range)
		exynos_state->range = val;
	else
		return -EINVAL;

	return 0;
}

static int exynos_drm_writeback_get_property(struct drm_connector *connector,
				const struct drm_connector_state *state,
				struct drm_property *property, uint64_t *val)
{
	struct writeback_device *wb = conn_to_wb_dev(connector);
	struct exynos_drm_writeback_state *exynos_state =
						to_exynos_wb_state(state);

	if (property == wb->props.restriction)
		*val = exynos_state->blob_id_restriction;
	else if (property == wb->props.standard)
		*val = exynos_state->standard;
	else if (property == wb->props.range)
		*val = exynos_state->range;
	else
		return -EINVAL;

	return 0;

}

static const struct drm_connector_funcs wb_connector_funcs = {
	.reset = exynos_drm_writeback_reset,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = exynos_drm_writeback_destroy,
	.atomic_duplicate_state = exynos_drm_writeback_duplicate_state,
	.atomic_destroy_state = exynos_drm_writeback_destroy_state,
	.atomic_set_property = exynos_drm_writeback_set_property,
	.atomic_get_property = exynos_drm_writeback_get_property,
};

static void _writeback_enable(struct writeback_device *wb)
{
	dpp_reg_init(wb->id, wb->attr);
	enable_irq(wb->odma_irq);
}

static void writeback_enable(struct drm_encoder *encoder)
{
	struct writeback_device *wb = enc_to_wb_dev(encoder);
	const struct decon_device *decon;

	pr_debug("%s +\n", __func__);

	if (wb->state == WB_STATE_ON) {
		pr_info("wb(%d) already enabled(%d)\n", wb->id, wb->state);
		return;
	}

	_writeback_enable(wb);
	decon = wb_get_decon(wb);
	if (unlikely(!decon)) {
		pr_err("%s: unable to get decon\n", __func__);
		return;
	}

	wb->decon_id = decon->id;
	wb->state = WB_STATE_ON;
	DPU_EVENT_LOG(DPU_EVT_WB_ENABLE, wb->decon_id, wb);

	pr_debug("%s -\n", __func__);
}

void writeback_exit_hibernation(struct writeback_device *wb)
{
	if (wb->state != WB_STATE_HIBERNATION)
		return;

	_writeback_enable(wb);
	wb->state = WB_STATE_ON;
	DPU_EVENT_LOG(DPU_EVT_WB_EXIT_HIBERNATION, wb->decon_id, wb);
}

static void _writeback_disable(struct writeback_device *wb)
{
	disable_irq(wb->odma_irq);
	dpp_reg_deinit(wb->id, false, wb->attr);
}

static void writeback_disable(struct drm_encoder *encoder)
{
	struct writeback_device *wb = enc_to_wb_dev(encoder);

	pr_debug("%s +\n", __func__);

	if (wb->state == WB_STATE_OFF) {
		pr_info("writeback(dpp%d) already disabled(%d)\n",
				wb->id, wb->state);
		return;
	}

	_writeback_disable(wb);
	wb->state = WB_STATE_OFF;
	DPU_EVENT_LOG(DPU_EVT_WB_DISABLE, wb->decon_id, wb);

	wb->decon_id = -1;

	pr_debug("%s -\n", __func__);
}

void writeback_enter_hibernation(struct writeback_device *wb)
{
	if (wb->state != WB_STATE_ON)
		return;

	_writeback_disable(wb);
	wb->state = WB_STATE_HIBERNATION;
	DPU_EVENT_LOG(DPU_EVT_WB_ENTER_HIBERNATION, wb->decon_id, wb);
}

static const struct drm_encoder_helper_funcs wb_encoder_helper_funcs = {
	.enable = writeback_enable,
	.disable = writeback_disable,
	.atomic_check = writeback_atomic_check,
};

/* TODO : modify create property because same property is created at plane */
static int
exynos_drm_wb_conn_create_standard_property(struct drm_connector *connector)
{
	struct writeback_device *wb = conn_to_wb_dev(connector);
	struct drm_property *prop;

	static const struct drm_prop_enum_list standard_list[] = {
		{ EXYNOS_STANDARD_UNSPECIFIED, "Unspecified" },
		{ EXYNOS_STANDARD_BT709, "BT709" },
		{ EXYNOS_STANDARD_BT601_625, "BT601_625" },
		{ EXYNOS_STANDARD_BT601_625_UNADJUSTED, "BT601_625_UNADJUSTED"},
		{ EXYNOS_STANDARD_BT601_525, "BT601_525" },
		{ EXYNOS_STANDARD_BT601_525_UNADJUSTED, "BT601_525_UNADJUSTED"},
		{ EXYNOS_STANDARD_BT2020, "BT2020" },
		{ EXYNOS_STANDARD_BT2020_CONSTANT_LUMINANCE,
						"BT2020_CONSTANT_LUMINANCE"},
		{ EXYNOS_STANDARD_BT470M, "BT470M" },
		{ EXYNOS_STANDARD_FILM, "FILM" },
		{ EXYNOS_STANDARD_DCI_P3, "DCI-P3" },
		{ EXYNOS_STANDARD_ADOBE_RGB, "Adobe RGB" },
	};

	prop = drm_property_create_enum(connector->dev, 0, "standard",
				standard_list, ARRAY_SIZE(standard_list));
	if (!prop)
		return -ENOMEM;

	drm_object_attach_property(&connector->base, prop,
				EXYNOS_STANDARD_UNSPECIFIED);
	wb->props.standard = prop;

	return 0;
}

/* TODO : modify create property because same property is created at plane */
static int
exynos_drm_wb_conn_create_range_property(struct drm_connector *connector)
{
	struct writeback_device *wb = conn_to_wb_dev(connector);
	struct drm_property *prop;
	static const struct drm_prop_enum_list range_list[] = {
		{ EXYNOS_RANGE_UNSPECIFIED, "Unspecified" },
		{ EXYNOS_RANGE_FULL, "Full" },
		{ EXYNOS_RANGE_LIMITED, "Limited" },
		{ EXYNOS_RANGE_EXTENDED, "Extended" },
	};

	prop = drm_property_create_enum(connector->dev, 0, "range", range_list,
						ARRAY_SIZE(range_list));
	if (!prop)
		return -ENOMEM;

	drm_object_attach_property(&connector->base, prop,
				EXYNOS_RANGE_UNSPECIFIED);
	wb->props.range = prop;

	return 0;
}

/* TODO : modify create property because same property is created at plane */
static int
exynos_drm_wb_conn_create_restriction_property(struct drm_connector *connector)
{
	struct writeback_device *wb = conn_to_wb_dev(connector);
	struct drm_property *prop;
	struct drm_property_blob *blob;
	struct dpp_ch_restriction res;

	memcpy(&res.restriction, &wb->restriction,
				sizeof(struct dpp_restriction));
	res.id = wb->id;
	res.attr = wb->attr;

	blob = drm_property_create_blob(connector->dev, sizeof(res), &res);
	if (IS_ERR(blob))
		return PTR_ERR(blob);

	prop = drm_property_create(connector->dev,
				   DRM_MODE_PROP_IMMUTABLE | DRM_MODE_PROP_BLOB,
				   "hw restrictions", 0);
	if (!prop)
		return -ENOMEM;

	drm_object_attach_property(&connector->base, prop, blob->base.id);
	wb->props.restriction = prop;

	return 0;
}

static int writeback_bind(struct device *dev, struct device *master, void *data)
{
	struct writeback_device *wb = dev_get_drvdata(dev);
	struct drm_device *drm_dev = data;
	struct drm_connector *connector = &wb->writeback.base;
	int ret;

	pr_info("%s +\n", __func__);

	drm_connector_helper_add(connector, &wb_connector_helper_funcs);
	ret = drm_writeback_connector_init(drm_dev, &wb->writeback,
			&wb_connector_funcs, &wb_encoder_helper_funcs,
			wb->pixel_formats, wb->num_pixel_formats, 0);
	if (ret) {
		pr_err("%s: failed to init writeback connector\n", __func__);
		return ret;
	}

	exynos_drm_wb_conn_create_standard_property(connector);
	exynos_drm_wb_conn_create_range_property(connector);
	exynos_drm_wb_conn_create_restriction_property(connector);

	pr_info("%s -\n", __func__);

	return 0;
}

static void
writeback_unbind(struct device *dev, struct device *master, void *data)
{
	struct writeback_device *wb = dev_get_drvdata(dev);
	struct drm_connector *connector = &wb->writeback.base;

	pr_debug("%s +\n", __func__);
	drm_connector_cleanup(connector);
	pr_debug("%s -\n", __func__);
}

static const struct component_ops exynos_wb_component_ops = {
	.bind	= writeback_bind,
	.unbind	= writeback_unbind,
};

/* TODO : can be replaced by dpp_print_restriction */
static void wb_print_restriction(struct writeback_device *wb)
{
	struct dpp_restriction *res = &wb->restriction;

	pr_info("src_f_w[%d %d %d] src_f_h[%d %d %d]\n",
			res->src_f_w.min, res->src_f_w.max, res->src_f_w.align,
			res->src_f_h.min, res->src_f_h.max, res->src_f_h.align);
	pr_info("src_w[%d %d %d] src_h[%d %d %d] src_x_y_align[%d %d]\n",
			res->src_w.min, res->src_w.max, res->src_w.align,
			res->src_h.min, res->src_h.max, res->src_h.align,
			res->src_x_align, res->src_y_align);

	pr_info("dst_f_w[%d %d %d] dst_f_h[%d %d %d]\n",
			res->dst_f_w.min, res->dst_f_w.max, res->dst_f_w.align,
			res->dst_f_h.min, res->dst_f_h.max, res->dst_f_h.align);
	pr_info("dst_w[%d %d %d] dst_h[%d %d %d] dst_x_y_align[%d %d]\n",
			res->dst_w.min, res->dst_w.max, res->dst_w.align,
			res->dst_h.min, res->dst_h.max, res->dst_h.align,
			res->dst_x_align, res->dst_y_align);

	pr_info("blk_w[%d %d %d] blk_h[%d %d %d] blk_x_y_align[%d %d]\n",
			res->blk_w.min, res->blk_w.max, res->blk_w.align,
			res->blk_h.min, res->blk_h.max, res->blk_h.align,
			res->blk_x_align, res->blk_y_align);

	pr_info("src_h_rot_max[%d]\n", res->src_h_rot_max);

	pr_info("max scale up(%dx), down(1/%dx) ratio\n", res->scale_up,
			res->scale_down);
}

static int exynos_wb_parse_dt(struct writeback_device *wb,
				struct device_node *np)
{
	int ret;
	struct dpp_restriction *res = &wb->restriction;

	pr_debug("%s +\n", __func__);

	ret = of_property_read_u32(np, "dpp,id", &wb->id);
	if (ret)
		return ret;

	of_property_read_u32(np, "attr", (u32 *)&wb->attr);
	of_property_read_u32(np, "port", &wb->port);

	wb->pixel_formats = writeback_formats;
	wb->num_pixel_formats = ARRAY_SIZE(writeback_formats);

	of_property_read_u32(np, "scale_down", (u32 *)&res->scale_down);
	of_property_read_u32(np, "scale_up", (u32 *)&res->scale_up);

	pr_info("attr(0x%lx), port(%d)\n", wb->attr, wb->port);

	wb_print_restriction(wb);

	pr_debug("%s -\n", __func__);

	return ret;
}

static irqreturn_t odma_irq_handler(int irq, void *priv)
{
	struct writeback_device *wb = priv;
	u32 irqs;

	spin_lock(&wb->odma_slock);
	if (wb->state == WB_STATE_OFF)
		goto irq_end;

	irqs = odma_reg_get_irq_and_clear(wb->id);

	if (irqs & ODMA_STATUS_FRAMEDONE_IRQ || irqs & ODMA_INST_OFF_DONE_IRQ) {

		if (irqs & ODMA_STATUS_FRAMEDONE_IRQ)
			pr_debug("wb(%d) framedone irq occurs\n", wb->id);
		else
			pr_warn("wb(%d) instant off irq occurs\n", wb->id);

		drm_writeback_signal_completion(&wb->writeback, 0);
		DPU_EVENT_LOG(DPU_EVT_WB_FRAMEDONE, wb->decon_id, wb);
	}

irq_end:
	spin_unlock(&wb->odma_slock);
	return IRQ_HANDLED;
}

static int wb_init_resources(struct writeback_device *wb)
{
	struct resource res;
	int ret = 0;
	struct device *dev = wb->dev;
	struct device_node *np = dev->of_node;
	struct platform_device *pdev;
	int i;

	pr_debug("%s +\n", __func__);

	pdev = container_of(dev, struct platform_device, dev);

	i = of_property_match_string(np, "reg-names", "dma");
	if (of_address_to_resource(np, i, &res)) {
		pr_err("failed to get dma resource\n");
		return -EINVAL;
	}
	wb->regs.dma_base_regs = ioremap(res.start, resource_size(&res));
	if (!wb->regs.dma_base_regs) {
		pr_err("failed to remap DPU_DMA SFR region\n");
		return -EINVAL;
	}
	dpp_regs_desc_init(wb->regs.dma_base_regs, res.start, "dma", REGS_DMA, wb->id);

	wb->odma_irq = of_irq_get_byname(np, "dma");
	pr_info("dma irq no = %d\n", wb->odma_irq);
	ret = devm_request_irq(dev, wb->odma_irq, odma_irq_handler, 0,
			pdev->name, wb);
	if (ret) {
		pr_err("failed to install DPU DMA irq\n");
		return -EINVAL;
	}
	disable_irq(wb->odma_irq);

	if (test_bit(DPP_ATTR_DPP, &wb->attr)) {
		i = of_property_match_string(np, "reg-names", "dpp");
		if (of_address_to_resource(np, i, &res)) {
			pr_err("failed to get dpp resource\n");
			return -EINVAL;
		}
		wb->regs.dpp_base_regs = ioremap(res.start, resource_size(&res));
		if (!wb->regs.dpp_base_regs) {
			pr_err("failed to remap DPP SFR region\n");
			return -EINVAL;
		}
		dpp_regs_desc_init(wb->regs.dpp_base_regs, res.start, "dpp", REGS_DPP,
				wb->id);
	}

	pr_debug("%s -\n", __func__);

	return ret;
}

static int writeback_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device *dev = &pdev->dev;
	struct writeback_device *writeback;
	const struct dpp_restriction *restriction;

	writeback = devm_kzalloc(dev, sizeof(struct writeback_device),
			GFP_KERNEL);
	if (!writeback)
		return -ENOMEM;

	restriction = of_device_get_match_data(dev);
	memcpy(&writeback->restriction, restriction, sizeof(*restriction));

	writeback->dev = dev;
	ret = exynos_wb_parse_dt(writeback, dev->of_node);
	if (ret)
		goto fail;

	writeback->output_type = EXYNOS_DISPLAY_TYPE_VIDI;

	spin_lock_init(&writeback->odma_slock);

	writeback->state = WB_STATE_OFF;

	ret = wb_init_resources(writeback);
	if (ret)
		goto fail;

	/* dpp is not connected decon now */
	writeback->decon_id = -1;

	platform_set_drvdata(pdev, writeback);

	pr_info("writeback(dpp%d) successfully probe", writeback->id);

	ret = component_add(dev, &exynos_wb_component_ops);
	if (ret)
		goto fail;

	return ret;

fail:
	pr_err("writeback(dpp%d) probe failed", writeback->id);
	return ret;
}

static int writeback_remove(struct platform_device *pdev)
{
	struct writeback_device *wb = platform_get_drvdata(pdev);

	component_del(&pdev->dev, &exynos_wb_component_ops);

	if (test_bit(DPP_ATTR_DPP, &wb->attr))
		iounmap(wb->regs.dpp_base_regs);
	iounmap(wb->regs.dma_base_regs);

	return 0;
}

struct platform_driver writeback_driver = {
	.probe = writeback_probe,
	.remove = writeback_remove,
	.driver = {
		   .name = "exynos-writeback",
		   .owner = THIS_MODULE,
		   .of_match_table = wb_of_match,
	},
};

MODULE_AUTHOR("Wonyeong Choi <won0.choi@samsung.com>");
MODULE_DESCRIPTION("Samsung SoC Display WriteBack");
MODULE_LICENSE("GPL v2");
