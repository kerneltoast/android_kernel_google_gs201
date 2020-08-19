/*
 * drivers/media/platform/exynos/mfc/mfc_qos.c
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/err.h>
#ifdef CONFIG_MFC_USE_BTS
#include <soc/samsung/bts.h>
#endif

#include "mfc_qos.h"
#include "mfc_utils.h"

#ifdef CONFIG_MFC_USE_BUS_DEVFREQ
enum {
	MFC_QOS_ADD,
	MFC_QOS_UPDATE,
	MFC_QOS_REMOVE,
	MFC_QOS_BW,
};

enum {
	MFC_PERF_BOOST_DVFS	= (1 << 0),
	MFC_PERF_BOOST_MO	= (1 << 1),
	MFC_PERF_BOOST_CPU	= (1 << 2),
};

void mfc_perf_boost_enable(struct mfc_dev *dev)
{
	struct mfc_platdata *pdata = dev->pdata;
	struct mfc_qos_boost *qos_boost_table = pdata->qos_boost_table;
	int i;

	if (perf_boost_mode & MFC_PERF_BOOST_DVFS) {
		if (pdata->mfc_freq_control)
			pm_qos_add_request(&dev->qos_req_mfc, PM_QOS_MFC_THROUGHPUT,
					qos_boost_table->freq_mfc);
		pm_qos_add_request(&dev->qos_req_int, PM_QOS_DEVICE_THROUGHPUT,
				qos_boost_table->freq_int);
		pm_qos_add_request(&dev->qos_req_mif, PM_QOS_BUS_THROUGHPUT,
				qos_boost_table->freq_mif);
		mfc_dev_debug(3, "[QoS][BOOST] DVFS mfc: %d, int:%d, mif:%d\n",
				qos_boost_table->freq_mfc, qos_boost_table->freq_int,
				qos_boost_table->freq_mif);
	}

#ifdef CONFIG_MFC_USE_BTS
	if (perf_boost_mode & MFC_PERF_BOOST_MO) {
		if (pdata->mo_control) {
#ifdef CONFIG_MFC_NO_RENEWAL_BTS
			bts_update_scen(BS_MFC_UHD_10BIT, 1);
			mfc_dev_debug(3, "[QoS][BOOST] BTS(MO): UHD_10BIT\n");
#else
			bts_add_scenario(qos_boost_table->bts_scen_idx);
			mfc_dev_debug(3, "[QoS][BOOST] BTS(MO) add idx %d (%s)\n",
					qos_boost_table->bts_scen_idx,
					qos_boost_table->name);
#endif
		}
	}
#endif

	if (perf_boost_mode & MFC_PERF_BOOST_CPU) {
		for (i = 0; i < qos_boost_table->num_cluster; i++) {
			pm_qos_add_request(&dev->qos_req_cluster[i], PM_QOS_CLUSTER0_FREQ_MIN + (i * 2),
					qos_boost_table->freq_cluster[i]);
			mfc_dev_debug(3, "[QoS][BOOST] CPU cluster[%d]: %d\n",
					i, qos_boost_table->freq_cluster[i]);
		}
	}
}

void mfc_perf_boost_disable(struct mfc_dev *dev)
{
	struct mfc_platdata *pdata = dev->pdata;
	int i;

	if (perf_boost_mode & MFC_PERF_BOOST_DVFS) {
		if (pdata->mfc_freq_control)
			pm_qos_remove_request(&dev->qos_req_mfc);
		pm_qos_remove_request(&dev->qos_req_int);
		pm_qos_remove_request(&dev->qos_req_mif);
		mfc_dev_debug(3, "[QoS][BOOST] DVFS off\n");
	}

#ifdef CONFIG_MFC_USE_BTS
	if (perf_boost_mode & MFC_PERF_BOOST_MO) {
		if (pdata->mo_control) {
#ifdef CONFIG_MFC_NO_RENEWAL_BTS
			bts_update_scen(BS_MFC_UHD_10BIT, 0);
			mfc_dev_debug(3, "[QoS][BOOST] BTS(MO) off\n");
#else
			bts_del_scenario(pdata->qos_boost_table->bts_scen_idx);
			mfc_dev_debug(3, "[QoS][BOOST] BTS(MO) del idx %d (%s)\n",
					pdata->qos_boost_table->bts_scen_idx,
					pdata->qos_boost_table->name);
#endif
		}
	}
#endif

	if (perf_boost_mode & MFC_PERF_BOOST_CPU) {
		for (i = 0; i < pdata->qos_boost_table->num_cluster; i++) {
			pm_qos_remove_request(&dev->qos_req_cluster[i]);
			mfc_dev_debug(3, "[QoS][BOOST] CPU cluster[%d] off\n",
								i);
		}
	}
}

static void __mfc_qos_operate(struct mfc_dev *dev, int opr_type,
			int table_type, int idx)
{
	struct mfc_platdata *pdata = dev->pdata;
	struct mfc_qos *qos_table;
	int freq_mfc;

	if (table_type == MFC_QOS_TABLE_TYPE_ENCODER)
		qos_table = pdata->encoder_qos_table;
	else
		qos_table = pdata->default_qos_table;
	freq_mfc = qos_table[idx].freq_mfc;

	switch (opr_type) {
	case MFC_QOS_ADD:
		if (dev->mfc_freq_by_bps > freq_mfc) {
			mfc_dev_debug(2, "[QoS] mfc freq set to high %d -> %d by bps\n",
					freq_mfc, dev->mfc_freq_by_bps);
			freq_mfc = dev->mfc_freq_by_bps;
		}

		if (pdata->mfc_freq_control)
			pm_qos_add_request(&dev->qos_req_mfc,
					PM_QOS_MFC_THROUGHPUT,
					freq_mfc);
		pm_qos_add_request(&dev->qos_req_int,
				PM_QOS_DEVICE_THROUGHPUT,
				qos_table[idx].freq_int);
		pm_qos_add_request(&dev->qos_req_mif,
				PM_QOS_BUS_THROUGHPUT,
				qos_table[idx].freq_mif);

#ifdef CONFIG_MFC_USE_BTS
		if (pdata->mo_control) {
#ifdef CONFIG_MFC_NO_RENEWAL_BTS
			bts_update_scen(BS_MFC_UHD_ENC60,
					qos_table[idx].mo_uhd_enc60_value);
			bts_update_scen(BS_MFC_UHD_10BIT,
					qos_table[idx].mo_10bit_value);
			bts_update_scen(BS_MFC_UHD,
					qos_table[idx].mo_value);
			MFC_TRACE_DEV("MO add uhd:%d, 10bit:%d, enc60:%d\n",
					qos_table[idx].mo_value,
					qos_table[idx].mo_10bit_value,
					qos_table[idx].mo_uhd_enc60_value);
			mfc_dev_debug(2, "[QoS] BTS(MO) update - uhd:%d, uhd_10bit:%d, uhd_enc60:%d\n",
					qos_table[idx].mo_value,
					qos_table[idx].mo_10bit_value,
					qos_table[idx].mo_uhd_enc60_value);
#else
			bts_add_scenario(qos_table[idx].bts_scen_idx);
			dev->prev_bts_scen_idx = qos_table[idx].bts_scen_idx;
			MFC_TRACE_DEV("BTS(MO) add idx %d (%s)\n",
					qos_table[idx].bts_scen_idx,
					qos_table[idx].name);
			mfc_dev_debug(2, "[QoS] BTS(MO) add idx %d (%s)\n",
					qos_table[idx].bts_scen_idx,
					qos_table[idx].name);
#endif
		}
#endif

		atomic_set(&dev->qos_req_cur, idx + 1);
		MFC_TRACE_DEV("QoS add[%d] - mfc:%d(%s), int:%d, mif:%d\n",
				idx, freq_mfc,
				pdata->mfc_freq_control ? "used" : "un-used",
				qos_table[idx].freq_int,
				qos_table[idx].freq_mif);
		mfc_dev_debug(2, "[QoS] QoS add[%d] - mfc:%d(%s), int:%d, mif:%d\n",
				idx, freq_mfc,
				pdata->mfc_freq_control ? "used" : "un-used",
				 qos_table[idx].freq_int,
				 qos_table[idx].freq_mif);
		break;
	case MFC_QOS_UPDATE:
		if (dev->mfc_freq_by_bps > freq_mfc) {
			mfc_dev_debug(2, "[QoS] mfc freq set to high %d -> %d by bps\n",
					freq_mfc, dev->mfc_freq_by_bps);
			freq_mfc = dev->mfc_freq_by_bps;
		}

		if (pdata->mfc_freq_control)
			pm_qos_update_request(&dev->qos_req_mfc, freq_mfc);
		pm_qos_update_request(&dev->qos_req_int,
				qos_table[idx].freq_int);
		pm_qos_update_request(&dev->qos_req_mif,
				qos_table[idx].freq_mif);

#ifdef CONFIG_MFC_USE_BTS
		if (pdata->mo_control) {
#ifdef CONFIG_MFC_NO_RENEWAL_BTS
			bts_update_scen(BS_MFC_UHD_ENC60,
					qos_table[idx].mo_uhd_enc60_value);
			bts_update_scen(BS_MFC_UHD_10BIT,
					qos_table[idx].mo_10bit_value);
			bts_update_scen(BS_MFC_UHD,
					qos_table[idx].mo_value);
			MFC_TRACE_DEV("MO update uhd:%d, 10bit:%d, enc60:%d\n",
					qos_table[idx].mo_value,
					qos_table[idx].mo_10bit_value,
					qos_table[idx].mo_uhd_enc60_value);
			mfc_dev_debug(2, "[QoS] BTS(MO) update - uhd:%d, uhd_10bit:%d, uhd_enc60:%d\n",
					qos_table[idx].mo_value,
					qos_table[idx].mo_10bit_value,
					qos_table[idx].mo_uhd_enc60_value);
#else
			bts_add_scenario(qos_table[idx].bts_scen_idx);
			bts_del_scenario(dev->prev_bts_scen_idx);
			dev->prev_bts_scen_idx = qos_table[idx].bts_scen_idx;
			MFC_TRACE_DEV("BTS(MO) update idx %d (%s)\n",
					qos_table[idx].bts_scen_idx,
					qos_table[idx].name);
			mfc_dev_debug(2, "[QoS] BTS(MO) update idx %d (%s)\n",
					qos_table[idx].bts_scen_idx,
					qos_table[idx].name);
#endif
		}
#endif

		atomic_set(&dev->qos_req_cur, idx + 1);
		MFC_TRACE_DEV("QoS update[%d] - mfc:%d(%s), int:%d, mif:%d\n",
				idx, freq_mfc,
				pdata->mfc_freq_control ? "used" : "un-used",
				qos_table[idx].freq_int,
				qos_table[idx].freq_mif);
		mfc_dev_debug(2, "[QoS] QoS update[%d] - mfc:%d(%s), int:%d, mif:%d\n",
				idx, freq_mfc,
				pdata->mfc_freq_control ? "used" : "un-used",
				qos_table[idx].freq_int,
				qos_table[idx].freq_mif);
		break;
	case MFC_QOS_REMOVE:
		if (atomic_read(&dev->qos_req_cur) == 0) {
			MFC_TRACE_DEV("QoS already removed\n");
			mfc_dev_debug(2, "[QoS] QoS already removed\n");
			break;
		}

		if (pdata->mfc_freq_control)
			pm_qos_remove_request(&dev->qos_req_mfc);
		pm_qos_remove_request(&dev->qos_req_int);
		pm_qos_remove_request(&dev->qos_req_mif);

#ifdef CONFIG_MFC_USE_BTS
		if (pdata->mo_control) {
#ifdef CONFIG_MFC_NO_RENEWAL_BTS
			bts_update_scen(BS_MFC_UHD_ENC60, 0);
			bts_update_scen(BS_MFC_UHD_10BIT, 0);
			bts_update_scen(BS_MFC_UHD, 0);
#else
			bts_del_scenario(dev->prev_bts_scen_idx);
			MFC_TRACE_DEV("BTS(MO) del idx %d\n",
					dev->prev_bts_scen_idx);
			mfc_dev_debug(2, "[QoS] BTS(MO) del idx %d\n",
					dev->prev_bts_scen_idx);
#endif
		}

		if (pdata->bw_control) {
			dev->mfc_bw.peak = 0;
			dev->mfc_bw.read = 0;
			dev->mfc_bw.write = 0;
			bts_update_bw(dev->pdata->mfc_bw_index, dev->mfc_bw);
		}
#endif

		atomic_set(&dev->qos_req_cur, 0);
		MFC_TRACE_DEV("QoS remove\n");
		mfc_dev_debug(2, "[QoS] QoS remove\n");
		break;
	case MFC_QOS_BW:
#ifdef CONFIG_MFC_USE_BTS
		if (pdata->bw_control) {
			bts_update_bw(dev->pdata->mfc_bw_index, dev->mfc_bw);
			MFC_TRACE_DEV("BW update (p: %d, r: %d, w: %d)\n",
					dev->mfc_bw.peak, dev->mfc_bw.read,
					dev->mfc_bw.write);
			mfc_dev_debug(2, "[QoS] BTS(BW) update (peak: %d, read: %d, write: %d)\n",
					dev->mfc_bw.peak, dev->mfc_bw.read,
					dev->mfc_bw.write);
		}
#endif
		break;
	default:
		mfc_dev_err("[QoS] Unknown request for opr [%d]\n", opr_type);
		break;
	}
}

#ifdef CONFIG_MFC_USE_BTS
static void __mfc_qos_set(struct mfc_ctx *ctx, struct bts_bw *curr_mfc_bw,
			int table_type, int i)
#else
static void __mfc_qos_set(struct mfc_ctx *ctx, int table_type, int i)
#endif
{
	struct mfc_dev *dev = ctx->dev;
	struct mfc_platdata *pdata = dev->pdata;
	struct mfc_qos *qos_table;
	int num_qos_steps;
	unsigned int prev_mb;

	if (table_type == MFC_QOS_TABLE_TYPE_ENCODER) {
		num_qos_steps = pdata->num_encoder_qos_steps;
		qos_table = pdata->encoder_qos_table;
	} else {
		num_qos_steps = pdata->num_default_qos_steps;
		qos_table = pdata->default_qos_table;
	}

	prev_mb = qos_table[i + 1].threshold_mb;
	mfc_debug(2, "[QoS] %s table[%d] covered mb %d ~ %d (mfc: %d, int:%d, mif:%d)\n",
			table_type ? "enc" : "default", i,
			qos_table[i].threshold_mb,
			i == num_qos_steps - 1 ? pdata->max_mb : prev_mb,
			qos_table[i].freq_mfc, qos_table[i].freq_int,
			qos_table[i].freq_mif);

#ifdef CONFIG_MFC_USE_BTS
	if (curr_mfc_bw->peak != dev->mfc_bw.peak) {
		dev->mfc_bw.peak = curr_mfc_bw->peak;
		dev->mfc_bw.read = curr_mfc_bw->read;
		dev->mfc_bw.write = curr_mfc_bw->write;
		__mfc_qos_operate(dev, MFC_QOS_BW, table_type, i);
	}
#endif

	if (atomic_read(&dev->qos_req_cur) == 0) {
		__mfc_qos_operate(dev, MFC_QOS_ADD, table_type, i);
	} else {
		/*
		 * 1) QoS level is changed
		 * 2) MFC freq should be high regardless of QoS level
		 */
		if ((atomic_read(&dev->qos_req_cur) != (i + 1)) ||
				(dev->mfc_freq_by_bps > qos_table[i].freq_mfc))
			__mfc_qos_operate(dev, MFC_QOS_UPDATE, table_type, i);
	}
}

static inline unsigned long __mfc_qos_get_weighted_mb(struct mfc_ctx *ctx,
						unsigned long mb)
{
	struct mfc_enc *enc = ctx->enc_priv;
	struct mfc_dec *dec = ctx->dec_priv;
	struct mfc_enc_params *p;
	struct mfc_qos_weight *qos_weight = &ctx->dev->pdata->qos_weight;
	u32 num_planes = ctx->dst_fmt->num_planes;
	int weight = 1000;
	unsigned long weighted_mb;

	switch (ctx->codec_mode) {
	case MFC_REG_CODEC_H264_DEC:
	case MFC_REG_CODEC_H264_MVC_DEC:
	case MFC_REG_CODEC_H264_ENC:
	case MFC_REG_CODEC_H264_MVC_ENC:
		weight = (weight * 100) / qos_weight->weight_h264_hevc;
		mfc_debug(3, "[QoS] h264, hevc codec, weight: %d\n", weight / 10);
		if (num_planes == 3) {
			weight = (weight * 100) / qos_weight->weight_3plane;
			mfc_debug(3, "[QoS] 3 plane, weight: %d\n", weight / 10);
		}
		break;

	case MFC_REG_CODEC_VP8_DEC:
	case MFC_REG_CODEC_VP8_ENC:
		weight = (weight * 100) / qos_weight->weight_vp8_vp9;
		mfc_debug(3, "[QoS] vp8, vp9 codec, weight: %d\n", weight / 10);
		if (num_planes == 3) {
			weight = (weight * 100) / qos_weight->weight_3plane;
			mfc_debug(3, "[QoS] 3 plane, weight: %d\n", weight / 10);
		}
		break;

	case MFC_REG_CODEC_HEVC_DEC:
	case MFC_REG_CODEC_HEVC_ENC:
	case MFC_REG_CODEC_BPG_DEC:
	case MFC_REG_CODEC_BPG_ENC:
		weight = (weight * 100) / qos_weight->weight_h264_hevc;
		mfc_debug(3, "[QoS] h264, hevc codec, weight: %d\n", weight / 10);
		if (num_planes == 3) {
			weight = (weight * 100) / qos_weight->weight_3plane;
			mfc_debug(3, "[QoS] 3 plane, weight: %d\n", weight / 10);
		} else {
			if (ctx->is_422) {
				weight = (weight * 100) / qos_weight->weight_422;
				mfc_debug(3, "[QoS] 422foramt, weight: %d\n", weight / 10);
			} else if (ctx->is_10bit) {
				if (!ctx->mem_type_10bit && dec && dec->super64_bframe) {
					weight = (weight * 100) / qos_weight->weight_super64_bframe;
					mfc_debug(3, "[QoS] super64 B frame, weight: %d\n", weight / 10);
				} else {
					weight = (weight * 100) / qos_weight->weight_10bit;
					mfc_debug(3, "[QoS] 10bit, weight: %d\n", weight / 10);
				}
			}
		}
		break;

	case MFC_REG_CODEC_VP9_DEC:
	case MFC_REG_CODEC_VP9_ENC:
		weight = (weight * 100) / qos_weight->weight_vp8_vp9;
		mfc_debug(3, "[QoS] vp8, vp9 codec, weight: %d\n", weight / 10);

		if (num_planes == 3) {
			weight = (weight * 100) / qos_weight->weight_3plane;
			mfc_debug(3, "[QoS] 3 plane, weight: %d\n", weight / 10);
		} else {
			if (ctx->is_422) {
				weight = (weight * 100) / qos_weight->weight_422;
				mfc_debug(3, "[QoS] 422foramt, weight: %d\n", weight / 10);
			} else if (ctx->is_10bit) {
				weight = (weight * 100) / qos_weight->weight_10bit;
				mfc_debug(3, "[QoS] 10bit, weight: %d\n", weight / 10);
			}
		}
		break;

	case MFC_REG_CODEC_MPEG4_DEC:
	case MFC_REG_CODEC_FIMV1_DEC:
	case MFC_REG_CODEC_FIMV2_DEC:
	case MFC_REG_CODEC_FIMV3_DEC:
	case MFC_REG_CODEC_FIMV4_DEC:
	case MFC_REG_CODEC_H263_DEC:
	case MFC_REG_CODEC_VC1_RCV_DEC:
	case MFC_REG_CODEC_VC1_DEC:
	case MFC_REG_CODEC_MPEG2_DEC:
	case MFC_REG_CODEC_MPEG4_ENC:
	case MFC_REG_CODEC_H263_ENC:
		weight = (weight * 100) / qos_weight->weight_other_codec;
		mfc_debug(3, "[QoS] other codec, weight: %d\n", weight / 10);
		break;

	default:
		mfc_ctx_err("[QoS] wrong codec_mode (%d), no weight\n",
				ctx->codec_mode);
	}

	if (enc) {
		p = &enc->params;
		if ((IS_H264_ENC(ctx) || IS_HEVC_ENC(ctx)) && p->num_b_frame) {
			weight = (weight * 100) / qos_weight->weight_bframe;
			mfc_debug(3, "[QoS] B frame encoding, weight: %d\n", weight / 10);
		} else if ((IS_H264_ENC(ctx) || IS_HEVC_ENC(ctx) || IS_VP8_ENC(ctx) ||
					IS_VP9_ENC(ctx)) && (p->num_refs_for_p >= 2)) {
			weight = (weight * 100) / qos_weight->weight_num_of_ref;
			mfc_debug(3, "[QoS] num of ref >= 2, weight: %d\n", weight / 10);
		} else if (IS_HEVC_ENC(ctx) && p->codec.hevc.general_pb_enable) {
			weight = (weight * 100) / qos_weight->weight_gpb;
			mfc_debug(3, "[QoS] Genaral PB, weight: %d\n", weight / 10);
		}
	}
	if (dec) {
		if (dec->num_of_tile_over_4) {
			weight = (weight * 100) / qos_weight->weight_num_of_tile;
			mfc_debug(3, "[QoS] num of tile >= 4, weight: %d\n", weight / 10);
		}
	}

	weighted_mb = (mb * weight) / 1000;
	mfc_debug(3, "[QoS] weight: %d, codec: %d, num planes: %d, "
			"10bit: %d, 422format: %d (mb: %ld)\n",
			weight / 10, ctx->codec_mode,
			num_planes, ctx->is_10bit, ctx->is_422,
			weighted_mb);


	return weighted_mb;
}

static inline unsigned long __mfc_qos_get_mb_per_second(struct mfc_ctx *ctx)
{
	unsigned long mb_width, mb_height, fps, mb;

	mb_width = (ctx->crop_width + 15) / 16;
	mb_height = (ctx->crop_height + 15) / 16;
	fps = ctx->framerate / 1000;

	mb = mb_width * mb_height * fps;
	mfc_debug(4, "[QoS] ctx[%d:%s] %d x %d @ %ld fps (mb: %ld), %dKbps\n",
			ctx->num, ctx->type == MFCINST_ENCODER ? "ENC" : "DEC",
			ctx->crop_width, ctx->crop_height, fps, mb, ctx->Kbps);

	return __mfc_qos_get_weighted_mb(ctx, mb);
}

#ifdef CONFIG_MFC_USE_BTS
static void __mfc_qos_get_bw_per_second(struct mfc_ctx *ctx, struct bts_bw *curr_mfc_bw_ctx)
{
	struct mfc_bw_data bw_data;
	struct mfc_bw_info *bw_info = NULL;
	unsigned long mb_width, mb_height, fps, mb;
	unsigned long peak_bw_per_sec;
	unsigned long read_bw_per_sec;
	unsigned long write_bw_per_sec;
	unsigned long add_bw_per_sec = 0;
	unsigned long mb_count_per_uhd_frame = MB_COUNT_PER_UHD_FRAME;
	unsigned long max_fps_per_uhd_frame = MAX_FPS_PER_UHD_FRAME;

	mb_width = (ctx->crop_width + 15) / 16;
	mb_height = (ctx->crop_height + 15) / 16;
	fps = ctx->framerate / 1000;

	mb = mb_width * mb_height * fps;

	if (ctx->is_sbwc || ctx->is_sbwc_lossy)
		bw_info = &ctx->dev->pdata->mfc_bw_info_sbwc;
	else
		bw_info = &ctx->dev->pdata->mfc_bw_info;

	switch (ctx->codec_mode) {
	case MFC_REG_CODEC_H264_DEC:
	case MFC_REG_CODEC_H264_MVC_DEC:
		bw_data = bw_info->bw_dec_h264;
		break;
	case MFC_REG_CODEC_H264_ENC:
	case MFC_REG_CODEC_H264_MVC_ENC:
		bw_data = bw_info->bw_enc_h264;
		break;
	case MFC_REG_CODEC_HEVC_DEC:
	case MFC_REG_CODEC_BPG_DEC:
		if (ctx->is_10bit)
			bw_data = bw_info->bw_dec_hevc_10bit;
		else
			bw_data = bw_info->bw_dec_hevc;
		break;
	case MFC_REG_CODEC_HEVC_ENC:
	case MFC_REG_CODEC_BPG_ENC:
		if (ctx->is_10bit)
			bw_data = bw_info->bw_enc_hevc_10bit;
		else
			bw_data = bw_info->bw_enc_hevc;
		break;
	case MFC_REG_CODEC_MPEG4_DEC:
	case MFC_REG_CODEC_FIMV1_DEC:
	case MFC_REG_CODEC_FIMV2_DEC:
	case MFC_REG_CODEC_FIMV3_DEC:
	case MFC_REG_CODEC_FIMV4_DEC:
	case MFC_REG_CODEC_H263_DEC:
	case MFC_REG_CODEC_VC1_RCV_DEC:
	case MFC_REG_CODEC_VC1_DEC:
	case MFC_REG_CODEC_MPEG2_DEC:
		bw_data = bw_info->bw_dec_mpeg4;
		break;
	case MFC_REG_CODEC_VP8_DEC:
		bw_data = bw_info->bw_dec_vp8;
		break;
	case MFC_REG_CODEC_VP9_DEC:
		if (ctx->is_10bit)
			bw_data = bw_info->bw_dec_vp9_10bit;
		else
			bw_data = bw_info->bw_dec_vp9;
		break;
	case MFC_REG_CODEC_MPEG4_ENC:
	case MFC_REG_CODEC_H263_ENC:
		bw_data = bw_info->bw_enc_mpeg4;
		break;
	case MFC_REG_CODEC_VP8_ENC:
		bw_data = bw_info->bw_enc_vp8;
		break;
	case MFC_REG_CODEC_VP9_ENC:
		if (ctx->is_10bit)
			bw_data = bw_info->bw_enc_vp9_10bit;
		else
			bw_data = bw_info->bw_enc_vp9;
		break;
	default:
		bw_data.peak = 0;
		bw_data.read = 0;
		bw_data.write = 0;
		mfc_ctx_err("[QoS] wrong codec_mode (%d)\n", ctx->codec_mode);
	}

	if (mb > (mb_count_per_uhd_frame * max_fps_per_uhd_frame)) {
		mfc_debug(4, "[QoS] fix upper mb bound (mb: %ld, fps: %ld)\n", mb, fps);
		mb = mb_count_per_uhd_frame * max_fps_per_uhd_frame;
	}

	if (ctx->rgb_bpp > 12) {
		add_bw_per_sec = (((ctx->rgb_bpp - 12) / 8) *
			(ctx->crop_width * ctx->crop_height) * fps) /
			1024;
		mfc_debug(4, "[QoS] additional BW %ldKB for RGB format\n",
			add_bw_per_sec);
	}

	peak_bw_per_sec = ((bw_data.peak * mb) / mb_count_per_uhd_frame) +
				add_bw_per_sec;
	read_bw_per_sec = ((bw_data.read * mb) / mb_count_per_uhd_frame) +
				add_bw_per_sec;
	write_bw_per_sec = ((bw_data.write * mb) / mb_count_per_uhd_frame) +
				add_bw_per_sec;

	if (peak_bw_per_sec == 0) {
		mfc_debug(4, "[QoS] fix lower peak bound (mb: %ld, fps: %ld)\n", mb, fps);
		peak_bw_per_sec = MIN_BW_PER_SEC;
	}
	if (read_bw_per_sec == 0) {
		mfc_debug(4, "[QoS] fix lower read bound (mb: %ld, fps: %ld)\n", mb, fps);
		read_bw_per_sec = MIN_BW_PER_SEC;
	}
	if (write_bw_per_sec == 0) {
		mfc_debug(4, "[QoS] fix lower write bound (mb: %ld, fps: %ld)\n", mb, fps);
		write_bw_per_sec = MIN_BW_PER_SEC;
	}

	curr_mfc_bw_ctx->peak = (unsigned int)peak_bw_per_sec;
	curr_mfc_bw_ctx->read = (unsigned int)read_bw_per_sec;
	curr_mfc_bw_ctx->write = (unsigned int)write_bw_per_sec;
}
#endif

static int __mfc_qos_get_freq_by_bps(struct mfc_dev *dev,
		unsigned long total_bps)
{
	int i;

	if (total_bps > dev->pdata->max_Kbps[0]) {
		mfc_dev_debug(4, "[QoS] overspec bps %ld > %d\n",
				total_bps, dev->pdata->max_Kbps[0]);
		i = dev->pdata->num_mfc_freq - 1;
		return dev->bitrate_table[i].mfc_freq;
	}

	for (i = 0; i < dev->pdata->num_mfc_freq; i++) {
		if (total_bps <= dev->bitrate_table[i].bps_interval)
			return dev->bitrate_table[i].mfc_freq;
	}

	/* Not changed the MFC freq according to BPS */
	return 0;
}

void mfc_qos_on(struct mfc_ctx *ctx)
{
	struct mfc_dev *dev = ctx->dev;
	struct mfc_platdata *pdata = dev->pdata;
	struct mfc_qos *qos_table;
	struct mfc_ctx *qos_ctx;
	unsigned long hw_mb = 0, total_mb = 0, total_fps = 0, total_bps = 0;
	unsigned int fw_time, sw_time;
	int i, found = 0, dec_found = 0, num_qos_steps;
	int table_type = MFC_QOS_TABLE_TYPE_DEFAULT;
#ifdef CONFIG_MFC_USE_BTS
	struct bts_bw curr_mfc_bw, curr_mfc_bw_ctx;
#endif

	if (perf_boost_mode) {
		mfc_ctx_info("[QoS][BOOST] skip control\n");
		return;
	}

	mutex_lock(&dev->qos_mutex);
	list_for_each_entry(qos_ctx, &dev->qos_queue, qos_list)
		if (qos_ctx == ctx)
			found = 1;

	if (!found)
		list_add_tail(&ctx->qos_list, &dev->qos_queue);

#ifdef CONFIG_MFC_USE_BTS
	curr_mfc_bw.peak = 0;
	curr_mfc_bw.read = 0;
	curr_mfc_bw.write = 0;
#endif
	/* get the hw macroblock */
	list_for_each_entry(qos_ctx, &dev->qos_queue, qos_list) {
		if (qos_ctx->type == MFCINST_DECODER)
			dec_found += 1;
		hw_mb += __mfc_qos_get_mb_per_second(qos_ctx);
		total_fps += (qos_ctx->framerate / 1000);
		total_bps += qos_ctx->Kbps;
#ifdef CONFIG_MFC_USE_BTS
		__mfc_qos_get_bw_per_second(qos_ctx, &curr_mfc_bw_ctx);
		curr_mfc_bw.peak += curr_mfc_bw_ctx.peak;
		curr_mfc_bw.read += curr_mfc_bw_ctx.read;
		curr_mfc_bw.write += curr_mfc_bw_ctx.write;
#endif
	}

	if (dec_found) {
		/* default table */
		num_qos_steps = pdata->num_default_qos_steps;
		qos_table = pdata->default_qos_table;
		table_type = MFC_QOS_TABLE_TYPE_DEFAULT;
	} else {
		/* encoder only table */
		num_qos_steps = pdata->num_encoder_qos_steps;
		qos_table = pdata->encoder_qos_table;
		table_type = MFC_QOS_TABLE_TYPE_ENCODER;
	}

	/* search the suitable qos table */
	for (i = num_qos_steps - 1; i >= 0; i--) {
		fw_time = qos_table[i].time_fw;
		sw_time = (MFC_DRV_TIME + fw_time);

		if ((total_fps * sw_time) >= 1000000)
			total_mb = pdata->max_mb;
		else
			total_mb = ((1000000 * hw_mb) / (1000000 - (total_fps * sw_time)));

		mfc_debug(4, "[QoS] %s table[%d] fw_time: %dus, hw_mb: %ld, "
				"sw_time: %d, total_fps: %ld, total_mb: %ld\n",
				table_type ? "enc" : "default",
				i, fw_time, hw_mb, sw_time, total_fps, total_mb);

		if ((total_mb > qos_table[i].threshold_mb) || (i == 0))
			break;
	}

	if (total_mb > pdata->max_mb)
		mfc_debug(4, "[QoS] overspec mb %ld > %d\n", total_mb, pdata->max_mb);

	/* search the suitable independent mfc freq using bps */
	dev->mfc_freq_by_bps = __mfc_qos_get_freq_by_bps(dev, total_bps);

#ifdef CONFIG_MFC_USE_BTS
	__mfc_qos_set(ctx, &curr_mfc_bw, table_type, i);
#else
	__mfc_qos_set(ctx, table_type, i);
#endif
	mutex_unlock(&dev->qos_mutex);
}

void mfc_qos_off(struct mfc_ctx *ctx)
{
	struct mfc_dev *dev = ctx->dev;
	struct mfc_platdata *pdata = dev->pdata;
	struct mfc_qos *qos_table;
	struct mfc_ctx *qos_ctx;
	unsigned long hw_mb = 0, total_mb = 0, total_fps = 0, total_bps = 0;
	unsigned int fw_time, sw_time;
	int i, found = 0, dec_found = 0, num_qos_steps;
	int table_type = MFC_QOS_TABLE_TYPE_DEFAULT;
#ifdef CONFIG_MFC_USE_BTS
	struct bts_bw mfc_bw, mfc_bw_ctx;
#endif

	if (perf_boost_mode) {
		mfc_ctx_info("[QoS][BOOST] skip control\n");
		return;
	}

	mutex_lock(&dev->qos_mutex);
	if (list_empty(&dev->qos_queue)) {
		if (atomic_read(&dev->qos_req_cur) != 0) {
			mfc_ctx_err("[QoS] MFC request count is wrong!\n");
			__mfc_qos_operate(dev, MFC_QOS_REMOVE, table_type, 0);
		}
		mutex_unlock(&dev->qos_mutex);
		return;
	}

#ifdef CONFIG_MFC_USE_BTS
	mfc_bw.peak = 0;
	mfc_bw.read = 0;
	mfc_bw.write = 0;
#endif

	/* get the hw macroblock */
	list_for_each_entry(qos_ctx, &dev->qos_queue, qos_list) {
		if (qos_ctx == ctx) {
			found = 1;
			continue;
		}

		if (qos_ctx->type == MFCINST_DECODER)
			dec_found += 1;
		hw_mb += __mfc_qos_get_mb_per_second(qos_ctx);
		total_fps += (qos_ctx->framerate / 1000);
		total_bps += qos_ctx->Kbps;
#ifdef CONFIG_MFC_USE_BTS
		__mfc_qos_get_bw_per_second(qos_ctx, &mfc_bw_ctx);
		mfc_bw.peak += mfc_bw_ctx.peak;
		mfc_bw.read += mfc_bw_ctx.read;
		mfc_bw.write += mfc_bw_ctx.write;
#endif
	}
	if (found)
		list_del(&ctx->qos_list);

	if (dec_found) {
		/* default table */
		num_qos_steps = pdata->num_default_qos_steps;
		qos_table = pdata->default_qos_table;
		table_type = MFC_QOS_TABLE_TYPE_DEFAULT;
	} else {
		/* encoder only table */
		num_qos_steps = pdata->num_encoder_qos_steps;
		qos_table = pdata->encoder_qos_table;
		table_type = MFC_QOS_TABLE_TYPE_ENCODER;
	}

	/* search the suitable qos table */
	for (i = num_qos_steps - 1; i >= 0; i--) {
		fw_time = qos_table[i].time_fw;
		sw_time = (MFC_DRV_TIME + fw_time);

		if ((total_fps * sw_time) >= 1000000)
			total_mb = pdata->max_mb;
		else
			total_mb = ((1000000 * hw_mb) / (1000000 - (total_fps * sw_time)));

		mfc_debug(4, "[QoS] %s table[%d] fw_time: %dus, hw_mb: %ld, "
				"sw_time: %d, total_fps: %ld, total_mb: %ld\n",
				table_type ? "enc" : "default",
				i, fw_time, hw_mb, sw_time, total_fps, total_mb);

		if ((total_mb > qos_table[i].threshold_mb) || (total_mb == 0) || (i == 0))
			break;
	}

	if (total_mb > pdata->max_mb)
		mfc_debug(4, "[QoS] overspec mb %ld > %d\n", total_mb, pdata->max_mb);

	/* search the suitable independent mfc freq using bps */
	dev->mfc_freq_by_bps = __mfc_qos_get_freq_by_bps(dev, total_bps);

	if (list_empty(&dev->qos_queue) || total_mb == 0) {
		__mfc_qos_operate(dev, MFC_QOS_REMOVE, table_type, 0);
	} else {
#ifdef CONFIG_MFC_USE_BTS
		__mfc_qos_set(ctx, &mfc_bw, table_type, i);
#else
		__mfc_qos_set(ctx, table_type, i);
#endif
	}

	mutex_unlock(&dev->qos_mutex);
}

void __mfc_qos_off_all(struct mfc_dev *dev)
{
	struct mfc_ctx *qos_ctx, *tmp_ctx;

	mutex_lock(&dev->qos_mutex);
	if (list_empty(&dev->qos_queue)) {
		mfc_dev_err("[QoS][MFCIDLE] MFC QoS list already empty (%d)\n",
				atomic_read(&dev->qos_req_cur));
		mutex_unlock(&dev->qos_mutex);
		return;
	}

	/* Delete all of QoS list */
	list_for_each_entry_safe(qos_ctx, tmp_ctx, &dev->qos_queue, qos_list)
		list_del(&qos_ctx->qos_list);

	/* Select the opend ctx structure for QoS remove */
	__mfc_qos_operate(dev, MFC_QOS_REMOVE, MFC_QOS_TABLE_TYPE_DEFAULT, 0);
	mutex_unlock(&dev->qos_mutex);
}
#endif

void mfc_qos_idle_worker(struct work_struct *work)
{
	struct mfc_dev *dev;

	dev = container_of(work, struct mfc_dev, mfc_idle_work);

	mutex_lock(&dev->idle_qos_mutex);
	if (dev->idle_mode == MFC_IDLE_MODE_CANCEL) {
		mfc_change_idle_mode(dev, MFC_IDLE_MODE_NONE);
		mfc_dev_debug(2, "[QoS][MFCIDLE] idle mode is canceled\n");
		mutex_unlock(&dev->idle_qos_mutex);
		return;
	}

#ifdef CONFIG_MFC_USE_BUS_DEVFREQ
	__mfc_qos_off_all(dev);
#endif
	mfc_dev_info("[QoS][MFCIDLE] MFC go to QoS idle mode\n");

	mfc_change_idle_mode(dev, MFC_IDLE_MODE_IDLE);
	mutex_unlock(&dev->idle_qos_mutex);
}

#define COL_FRAME_RATE		0
#define COL_FRAME_INTERVAL	1

#define MFC_MAX_INTERVAL	(2 * USEC_PER_SEC)

/*
 * A framerate table determines framerate by the interval(us) of each frame.
 * Framerate is not accurate, just rough value to seperate overload section.
 * Base line of each section are selected from middle value.
 * 25fps(40000us), 40fps(25000us), 80fps(12500us), 144fps(6940us)
 * 144fps(6940us), 205fps(4860us), 320fps(3125us)
 *
 * interval(us)	 |0    3125    4860    6940    12500    25000	 40000	    |
 * framerate(fps)|  480  |  240  |  180  |  120  |   60   |   30   |   24   |
 */
static unsigned long framerate_table[][2] = {
	{  24000, 40000 },
	{  30000, 25000 },
	{  60000, 12500 },
	{ 120000,  6940 },
	{ 180000,  4860 },
	{ 240000,  3125 },
	{ 480000,     0 },
};

static inline unsigned long __mfc_qos_timespec64_diff(struct timespec64 *to,
						      struct timespec64 *from)
{
	return ((to->tv_sec * NSEC_PER_SEC + to->tv_nsec) -
		(from->tv_sec * NSEC_PER_SEC + from->tv_nsec)) /
	       NSEC_PER_USEC;
}

static unsigned long __mfc_qos_get_framerate_by_interval(int interval)
{
	unsigned long i;

	/* if the interval is too big (2sec), framerate set to 0 */
	if (interval > MFC_MAX_INTERVAL || interval < 0)
		return 0;

	for (i = 0; i < ARRAY_SIZE(framerate_table); i++) {
		if (interval > framerate_table[i][COL_FRAME_INTERVAL])
			return framerate_table[i][COL_FRAME_RATE];
	}

	return 0;
}

static int __mfc_qos_get_bps_section_by_bps(struct mfc_dev *dev, int Kbps)
{
	int i;

	if (Kbps > dev->pdata->max_Kbps[0]) {
		mfc_dev_debug(4, "[QoS] overspec bps %d > %d\n",
				Kbps, dev->pdata->max_Kbps[0]);
		return dev->pdata->num_mfc_freq - 1;
	}

	for (i = 0; i < dev->pdata->num_mfc_freq; i++) {
		if (Kbps <= dev->bitrate_table[i].bps_interval) {
			mfc_dev_debug(3, "[QoS] MFC freq lv%d, %dKHz is needed\n",
					i, dev->bitrate_table[i].mfc_freq);
			return i;
		}
	}

	return 0;
}

/* Return the minimum interval between previous and next entry */
static int __mfc_qos_get_interval(struct list_head *head, struct list_head *entry)
{
	int prev_interval = MFC_MAX_INTERVAL, next_interval = MFC_MAX_INTERVAL;
	struct mfc_timestamp *prev_ts, *next_ts, *curr_ts;

	curr_ts = list_entry(entry, struct mfc_timestamp, list);

	if (entry->prev != head) {
		prev_ts = list_entry(entry->prev, struct mfc_timestamp, list);
		prev_interval = __mfc_qos_timespec64_diff(&curr_ts->timestamp,
							  &prev_ts->timestamp);
	}

	if (entry->next != head) {
		next_ts = list_entry(entry->next, struct mfc_timestamp, list);
		next_interval = __mfc_qos_timespec64_diff(&next_ts->timestamp,
							  &curr_ts->timestamp);
	}

	return (prev_interval < next_interval ? prev_interval : next_interval);
}

static int __mfc_qos_add_timestamp(struct mfc_ctx *ctx,
				   struct timespec64 *time,
				   struct list_head *head)
{
	int replace_entry = 0;
	struct mfc_timestamp *curr_ts = &ctx->ts_array[ctx->ts_count];

	if (ctx->ts_is_full) {
		/* Replace the entry if list of array[ts_count] is same as entry */
		if (&curr_ts->list == head)
			replace_entry = 1;
		else
			list_del(&curr_ts->list);
	}

	memcpy(&curr_ts->timestamp, time, sizeof(*time));
	if (!replace_entry)
		list_add(&curr_ts->list, head);
	curr_ts->interval =
		__mfc_qos_get_interval(&ctx->ts_list, &curr_ts->list);
	curr_ts->index = ctx->ts_count;
	ctx->ts_count++;

	if (ctx->ts_count == MAX_TIME_INDEX) {
		ctx->ts_is_full = 1;
		ctx->ts_count %= MAX_TIME_INDEX;
	}

	return 0;
}

static unsigned long __mfc_qos_get_fps_by_timestamp(struct mfc_ctx *ctx,
						    struct timespec64 *time)
{
	struct list_head *head = &ctx->ts_list;
	struct mfc_timestamp *temp_ts;
	int found;
	int index = 0;
	int min_interval = MFC_MAX_INTERVAL;
	int time_diff;
	unsigned long max_framerate;

	if (debug_ts == 1) {
		/* Debug info */
		mfc_ctx_info("===================[TS]===================\n");
		mfc_ctx_info("[TS] New timestamp = %ld.%09ld, count = %d\n",
			time->tv_sec, time->tv_nsec, ctx->ts_count);
	}

	if (IS_BUFFER_BATCH_MODE(ctx)) {
		if (debug_ts == 1)
			mfc_ctx_info("[BUFCON][TS] Keep framerate if buffer batch mode is used, %ldfps\n",
					ctx->framerate);
		return ctx->framerate;
	}

	if (list_empty(&ctx->ts_list)) {
		__mfc_qos_add_timestamp(ctx, time, &ctx->ts_list);
		return __mfc_qos_get_framerate_by_interval(0);
	} else {
		found = 0;
		list_for_each_entry_reverse(temp_ts, &ctx->ts_list, list) {
			time_diff = __mfc_timespec64_compare(time,
							&temp_ts->timestamp);
			if (time_diff == 0) {
				/* Do not add if same timestamp already exists */
				found = 1;
				break;
			} else if (time_diff > 0) {
				/* Add this after temp_ts */
				__mfc_qos_add_timestamp(ctx, time, &temp_ts->list);
				found = 1;
				break;
			}
		}

		if (!found)	/* Add this at first entry */
			__mfc_qos_add_timestamp(ctx, time, &ctx->ts_list);
	}

	list_for_each_entry(temp_ts, &ctx->ts_list, list) {
		if (temp_ts->interval < min_interval)
			min_interval = temp_ts->interval;
	}

	max_framerate = __mfc_qos_get_framerate_by_interval(min_interval);

	if (debug_ts == 1) {
		/* Debug info */
		index = 0;
		list_for_each_entry(temp_ts, &ctx->ts_list, list) {
			mfc_ctx_info("[TS] [%d] timestamp [i:%d]: %ld.%09ld\n",
					index, temp_ts->index,
					temp_ts->timestamp.tv_sec,
					temp_ts->timestamp.tv_nsec);
			index++;
		}
		mfc_ctx_info("[TS] Min interval = %d, It is %ld fps\n",
				min_interval, max_framerate);
	}

	/* Calculation the last frame fps for drop control */
	temp_ts = list_entry(head->prev, struct mfc_timestamp, list);
	ctx->ts_last_interval = temp_ts->interval;

	if (!ctx->ts_is_full) {
		if (debug_ts == 1)
			mfc_ctx_info("[TS] ts doesn't full, keep %ld fps\n",
					ctx->framerate);
		return ctx->framerate;
	}

	return max_framerate;
}

static int __mfc_qos_get_bps_section(struct mfc_ctx *ctx, u32 bytesused)
{
	struct mfc_dev *dev = ctx->dev;
	struct list_head *head = &ctx->bitrate_list;
	struct mfc_bitrate *temp_bitrate;
	struct mfc_bitrate *new_bitrate;
	unsigned long sum_size = 0, avg_Kbits;
	int count = 0, bps_section = 0;

	if (ctx->bitrate_is_full) {
		temp_bitrate = list_entry(head->next, struct mfc_bitrate, list);
		list_del(&temp_bitrate->list);
	}

	new_bitrate = &ctx->bitrate_array[ctx->bitrate_index];
	new_bitrate->bytesused = bytesused;
	list_add_tail(&new_bitrate->list, head);

	list_for_each_entry(temp_bitrate, head, list) {
		mfc_debug(4, "[QoS][%d] strm_size %d\n",
				count, temp_bitrate->bytesused);
		sum_size += temp_bitrate->bytesused;
		count++;
	}

	avg_Kbits = ((sum_size * BITS_PER_BYTE) / count) / 1024;
	ctx->Kbps = (int)(avg_Kbits * (ctx->last_framerate / 1000));
	/* Standardization to high bitrate spec */
	if (!CODEC_HIGH_PERF(ctx))
		ctx->Kbps = dev->bps_ratio * ctx->Kbps;
	mfc_debug(3, "[QoS] %d Kbps, average %ld Kbits per frame\n",
			ctx->Kbps, avg_Kbits);

	ctx->bitrate_index++;
	if (ctx->bitrate_index == MAX_TIME_INDEX) {
		ctx->bitrate_is_full = 1;
		ctx->bitrate_index %= MAX_TIME_INDEX;
	}

	/*
	 * When there is a value of ts_is_full,
	 * we can trust fps(trusted fps calculated by timestamp diff).
	 * When fps information becomes reliable,
	 * we will start QoS handling by obtaining bps section.
	 */
	if (ctx->ts_is_full)
		bps_section = __mfc_qos_get_bps_section_by_bps(dev, ctx->Kbps);

	return bps_section;
}

void mfc_qos_update_framerate(struct mfc_ctx *ctx, u32 bytesused,
		int idle_trigger_only)
{
	struct mfc_dev *dev = ctx->dev;
	int bps_section;
	bool update_bps = false, update_idle = false;

	/* 1) Idle mode trigger */
	mutex_lock(&dev->idle_qos_mutex);
	if (dev->idle_mode == MFC_IDLE_MODE_IDLE) {
		mfc_debug(2, "[QoS][MFCIDLE] restart QoS control\n");
		mfc_change_idle_mode(dev, MFC_IDLE_MODE_NONE);
		update_idle = true;
	} else if (dev->idle_mode == MFC_IDLE_MODE_RUNNING) {
		mfc_debug(2, "[QoS][MFCIDLE] restart QoS control, cancel idle\n");
		mfc_change_idle_mode(dev, MFC_IDLE_MODE_CANCEL);
		update_idle = true;
	}
	mutex_unlock(&dev->idle_qos_mutex);

	if (idle_trigger_only)
		goto update_qos;

	/* 2) bitrate is updated */
	if (ctx->type == MFCINST_DECODER) {
		bps_section = __mfc_qos_get_bps_section(ctx, bytesused);
		if (ctx->last_bps_section != bps_section) {
			mfc_debug(2, "[QoS] bps section changed: %d -> %d\n",
					ctx->last_bps_section, bps_section);
			ctx->last_bps_section = bps_section;
			update_bps = true;
		}
	}

	/* 3) framerate is updated */
	if (ctx->last_framerate != 0 && ctx->last_framerate != ctx->framerate) {
		mfc_debug(2, "[QoS] fps changed: %ld -> %ld, qos ratio: %d\n",
				ctx->framerate, ctx->last_framerate, ctx->qos_ratio);
		ctx->framerate = ctx->last_framerate;
		ctx->update_framerate = true;
	}

update_qos:
	if (update_idle || update_bps || ctx->update_framerate)
		mfc_qos_on(ctx);
}

void mfc_qos_update_last_framerate(struct mfc_ctx *ctx, u64 timestamp)
{
	struct timespec64 time;

	time.tv_sec = timestamp / NSEC_PER_SEC;
	time.tv_nsec = (timestamp - (time.tv_sec * NSEC_PER_SEC));

	ctx->last_framerate = __mfc_qos_get_fps_by_timestamp(ctx, &time);
	if (ctx->last_framerate > MFC_MAX_FPS)
		ctx->last_framerate = MFC_MAX_FPS;
	ctx->last_framerate = (ctx->qos_ratio * ctx->last_framerate) / 100;
}
