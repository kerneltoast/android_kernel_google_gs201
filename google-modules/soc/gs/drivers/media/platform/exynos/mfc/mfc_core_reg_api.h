/*
 * drivers/media/platform/exynos/mfc/mfc_core_reg_api.h
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __MFC_CORE_REG_API_H
#define __MFC_CORE_REG_API_H __FILE__

#include "mfc_common.h"

#include "mfc_utils.h"

#define MFC_CORE_READL(offset)					\
	readl(core->regs_base + (offset))
#define MFC_CORE_WRITEL(data, offset)				\
	writel((data), core->regs_base + (offset))

#define MFC_CORE_RAW_READL(offset)				\
	__raw_readl(core->regs_base + (offset))
#define MFC_CORE_RAW_WRITEL(data, offset)			\
	__raw_writel((data), core->regs_base + (offset))

#define MFC_MMU0_READL(offset)		readl(core->sysmmu0_base + (offset))
#define MFC_MMU1_READL(offset)		readl(core->sysmmu1_base + (offset))

#define VOTF_WRITEL(data, offset)	writel((data), core->votf_base + (offset))
#define HWFC_WRITEL(data)	writel((data), core->hwfc_base)

#define MFC_SSMT0_WRITEL(data, offset)			\
	writel((data), core->ssmt0_base + (offset))
#define MFC_SSMT1_WRITEL(data, offset)			\
	writel((data), core->ssmt1_base + (offset))
#define MFC_SYSREG_WRITEL(data, offset)			\
	writel((data), core->sysreg_base + (offset))

/* version */
#define mfc_core_get_fimv_info()		((MFC_CORE_READL(MFC_REG_FW_VERSION)		\
						>> MFC_REG_FW_VER_INFO_SHFT)		\
						& MFC_REG_FW_VER_INFO_MASK)
#define mfc_core_get_fw_ver_year()	((MFC_CORE_READL(MFC_REG_FW_VERSION)		\
						>> MFC_REG_FW_VER_YEAR_SHFT)		\
						& MFC_REG_FW_VER_YEAR_MASK)
#define mfc_core_get_fw_ver_month()	((MFC_CORE_READL(MFC_REG_FW_VERSION)		\
						>> MFC_REG_FW_VER_MONTH_SHFT)		\
						& MFC_REG_FW_VER_MONTH_MASK)
#define mfc_core_get_fw_ver_date()	((MFC_CORE_READL(MFC_REG_FW_VERSION)		\
						>> MFC_REG_FW_VER_DATE_SHFT)		\
						& MFC_REG_FW_VER_DATE_MASK)
#define mfc_core_get_fw_ver_all()	((MFC_CORE_READL(MFC_REG_FW_VERSION)		\
						>> MFC_REG_FW_VER_ALL_SHFT)		\
						& MFC_REG_FW_VER_ALL_MASK)
#define mfc_core_get_mfc_version()	((MFC_CORE_READL(MFC_REG_MFC_VERSION)		\
						>> MFC_REG_MFC_VER_SHFT)		\
						& MFC_REG_MFC_VER_MASK)


/* decoding & display information */
#define mfc_core_get_dec_status()	(MFC_CORE_READL(MFC_REG_D_DECODED_STATUS)		\
						& MFC_REG_DEC_STATUS_DECODED_STATUS_MASK)
#define mfc_core_get_disp_status()	(MFC_CORE_READL(MFC_REG_D_DISPLAY_STATUS)		\
						& MFC_REG_DISP_STATUS_DISPLAY_STATUS_MASK)
#define mfc_core_get_res_change()	((MFC_CORE_READL(MFC_REG_D_DISPLAY_STATUS)		\
						>> MFC_REG_DISP_STATUS_RES_CHANGE_SHIFT)	\
						& MFC_REG_DISP_STATUS_RES_CHANGE_MASK)
#define mfc_core_get_black_bar_detection()	((MFC_CORE_READL(MFC_REG_D_DISPLAY_STATUS)		\
						>> MFC_REG_DISP_STATUS_BLACK_BAR_DETECT_SHIFT)	\
						& MFC_REG_DISP_STATUS_BLACK_BAR_DETECT_MASK)
#define mfc_core_get_dpb_change()	((MFC_CORE_READL(MFC_REG_D_DISPLAY_STATUS)		\
						>> MFC_REG_DISP_STATUS_NEED_DPB_CHANGE_SHIFT)	\
						& MFC_REG_DISP_STATUS_NEED_DPB_CHANGE_MASK)
#define mfc_core_get_scratch_change()	((MFC_CORE_READL(MFC_REG_D_DISPLAY_STATUS)		\
						>> MFC_REG_DISP_STATUS_NEED_SCRATCH_CHANGE_SHIFT)	\
						& MFC_REG_DISP_STATUS_NEED_SCRATCH_CHANGE_MASK)
#define mfc_core_get_uncomp()	((MFC_CORE_READL(MFC_REG_D_DISPLAY_STATUS)		\
						>> MFC_REG_DISP_STATUS_UNCOMP_SHIFT)	\
						& MFC_REG_DISP_STATUS_UNCOMP_MASK)
#define mfc_core_get_last_display()	((MFC_CORE_READL(MFC_REG_D_DISPLAY_STATUS)		\
						>> MFC_REG_DISP_STATUS_LAST_DISPLAY_FRAME_SHIFT) \
						& MFC_REG_DISP_STATUS_LAST_DISPLAY_FRAME_MASK)

#define mfc_core_get_disp_frame_type()	(MFC_CORE_READL(MFC_REG_D_DISPLAY_FRAME_TYPE)	\
						& MFC_REG_DISPLAY_FRAME_MASK)
#define mfc_core_get_dec_frame_type()	(MFC_CORE_READL(MFC_REG_D_DECODED_FRAME_TYPE)	\
						& MFC_REG_DECODED_FRAME_MASK)
#define mfc_core_get_disp_idr_flag()				\
	((MFC_CORE_READL(MFC_REG_D_DISPLAY_FRAME_TYPE)	\
	>> MFC_REG_DISPLAY_IDR_FLAG_SHIFT)		\
	& MFC_REG_DISPLAY_IDR_FLAG_MASK)
#define mfc_core_get_dec_idr_flag()				\
	((MFC_CORE_READL(MFC_REG_D_DECODED_FRAME_TYPE)	\
	>> MFC_REG_DECODED_IDR_FLAG_SHIFT)		\
	& MFC_REG_DECODED_IDR_FLAG_MASK)
#define mfc_core_get_dec_temporal_id()			\
	((MFC_CORE_READL(MFC_REG_D_H264_INFO)		\
	>> MFC_REG_D_H264_INFO_TEMPORAL_ID_SHIFT)	\
	& MFC_REG_D_H264_INFO_TEMPORAL_ID_MASK)
#define mfc_core_get_interlace_type()	((MFC_CORE_READL(MFC_REG_D_DISPLAY_FRAME_TYPE)	\
						>> MFC_REG_DISPLAY_TEMP_INFO_SHIFT)	\
						& MFC_REG_DISPLAY_TEMP_INFO_MASK)
#define mfc_core_is_interlace_picture()	((MFC_CORE_READL(MFC_REG_D_DISPLAY_STATUS)		\
						>> MFC_REG_DISP_STATUS_INTERLACE_SHIFT)\
						& MFC_REG_DISP_STATUS_INTERLACE_MASK)
#define mfc_core_is_mbaff_picture()	((MFC_CORE_READL(MFC_REG_D_H264_INFO)		\
						>> MFC_REG_D_H264_INFO_MBAFF_FRAME_FLAG_SHIFT)\
						& MFC_REG_D_H264_INFO_MBAFF_FRAME_FLAG_MASK)
#define mfc_core_is_sbwc_avail()		((MFC_CORE_READL(MFC_REG_D_DISPLAY_STATUS)		\
						>> MFC_REG_DISP_STATUS_COMP_SHIFT)\
						& MFC_REG_DISP_STATUS_COMP_MASK)

#define mfc_core_get_aspect_ratio()	MFC_CORE_READL(MFC_REG_D_DISPLAY_ASPECT_RATIO)
#define mfc_core_get_img_width()		MFC_CORE_READL(MFC_REG_D_DISPLAY_FRAME_WIDTH)
#define mfc_core_get_img_height()	MFC_CORE_READL(MFC_REG_D_DISPLAY_FRAME_HEIGHT)
#define mfc_core_get_disp_y_addr()	MFC_CORE_READL(MFC_REG_D_DISPLAY_LUMA_ADDR)
#define mfc_core_get_dec_y_addr()	MFC_CORE_READL(MFC_REG_D_DECODED_LUMA_ADDR)


/* kind of interrupt */
#define mfc_core_get_int_err()		MFC_CORE_READL(MFC_REG_ERROR_CODE)


/* additional information */
#define mfc_core_get_consumed_stream()		MFC_CORE_READL(MFC_REG_D_DECODED_NAL_SIZE)
#define mfc_core_get_dpb_count()			MFC_CORE_READL(MFC_REG_D_MIN_NUM_DPB)
#define mfc_core_get_min_dpb_size(x)		MFC_CORE_READL(MFC_REG_D_MIN_FIRST_PLANE_DPB_SIZE + (x * 4))
#define mfc_core_get_min_dpb_size_2bit(x)		MFC_CORE_READL(MFC_REG_D_MIN_FIRST_PLANE_2BIT_DPB_SIZE + (x * 4))
#define mfc_core_get_scratch_size()		MFC_CORE_READL(MFC_REG_D_MIN_SCRATCH_BUFFER_SIZE)
#define mfc_core_get_stride_size(x)		MFC_CORE_READL(MFC_REG_D_FIRST_PLANE_DPB_STRIDE_SIZE + (x * 4))
#define mfc_core_get_stride_size_2bit(x)		MFC_CORE_READL(MFC_REG_D_FIRST_PLANE_2BIT_DPB_STRIDE_SIZE + (x * 4))
#define mfc_core_get_mv_count()			MFC_CORE_READL(MFC_REG_D_MIN_NUM_MV)
#define mfc_core_get_inst_no()			MFC_CORE_READL(MFC_REG_RET_INSTANCE_ID)
#define mfc_core_get_enc_dpb_count()		MFC_CORE_READL(MFC_REG_E_NUM_DPB)
#define mfc_core_get_enc_scratch_size()		MFC_CORE_READL(MFC_REG_E_MIN_SCRATCH_BUFFER_SIZE)
#define mfc_core_get_enc_luma_size()		MFC_CORE_READL(MFC_REG_E_MIN_LUMA_DPB_SIZE)
#define mfc_core_get_enc_chroma_size()		MFC_CORE_READL(MFC_REG_E_MIN_CHROMA_DPB_SIZE)
#define mfc_core_get_enc_strm_size()		MFC_CORE_READL(MFC_REG_E_STREAM_SIZE)
#define mfc_core_get_enc_slice_type()		(MFC_CORE_READL(MFC_REG_E_SLICE_TYPE)		\
						& MFC_REG_E_SLICE_TYPE_MASK)
#define mfc_core_get_enc_pic_count()		MFC_CORE_READL(MFC_REG_E_PICTURE_COUNT)
#define mfc_core_get_sei_avail()			MFC_CORE_READL(MFC_REG_D_SEI_AVAIL)
#define mfc_core_get_sei_content_light()		MFC_CORE_READL(MFC_REG_D_CONTENT_LIGHT_LEVEL_INFO_SEI)
#define mfc_core_get_sei_mastering0()		MFC_CORE_READL(MFC_REG_D_MASTERING_DISPLAY_COLOUR_VOLUME_SEI_0)
#define mfc_core_get_sei_mastering1()		MFC_CORE_READL(MFC_REG_D_MASTERING_DISPLAY_COLOUR_VOLUME_SEI_1)
#define mfc_core_get_sei_mastering2()		MFC_CORE_READL(MFC_REG_D_MASTERING_DISPLAY_COLOUR_VOLUME_SEI_2)
#define mfc_core_get_sei_mastering3()		MFC_CORE_READL(MFC_REG_D_MASTERING_DISPLAY_COLOUR_VOLUME_SEI_3)
#define mfc_core_get_sei_mastering4()		MFC_CORE_READL(MFC_REG_D_MASTERING_DISPLAY_COLOUR_VOLUME_SEI_4)
#define mfc_core_get_sei_mastering5()		MFC_CORE_READL(MFC_REG_D_MASTERING_DISPLAY_COLOUR_VOLUME_SEI_5)
#define mfc_core_get_sei_avail_frame_pack()	(MFC_CORE_READL(MFC_REG_D_SEI_AVAIL)	\
						& MFC_REG_D_SEI_AVAIL_FRAME_PACK_MASK)
#define mfc_core_get_sei_avail_content_light()	((MFC_CORE_READL(MFC_REG_D_SEI_AVAIL)	\
						>> MFC_REG_D_SEI_AVAIL_CONTENT_LIGHT_SHIFT)	\
						& MFC_REG_D_SEI_AVAIL_CONTENT_LIGHT_MASK)
#define mfc_core_get_sei_avail_mastering_display()	((MFC_CORE_READL(MFC_REG_D_SEI_AVAIL)	\
						>> MFC_REG_D_SEI_AVAIL_MASTERING_DISPLAY_SHIFT)	\
						& MFC_REG_D_SEI_AVAIL_MASTERING_DISPLAY_MASK)
#define mfc_core_get_sei_avail_st_2094_40()		((MFC_CORE_READL(MFC_REG_D_SEI_AVAIL)	\
						>> MFC_REG_D_SEI_AVAIL_ST_2094_40_SHIFT)	\
						& MFC_REG_D_SEI_AVAIL_ST_2094_40_MASK)
#define mfc_core_get_sei_nal_meta_status()		((MFC_CORE_READL(MFC_REG_METADATA_STATUS)	\
						>> MFC_REG_SEI_NAL_STATUS_SHIFT)		\
						& MFC_REG_SEI_NAL_STATUS_MASK)

/* for AV1 Film Grain */
#define mfc_core_get_sei_avail_film_grain()		((MFC_CORE_READL(MFC_REG_D_SEI_AVAIL)	\
						>> MFC_REG_D_SEI_AVAIL_FILM_GRAIN_SHIFT)	\
						& MFC_REG_D_SEI_AVAIL_FILM_GRAIN_MASK)
#define mfc_core_get_video_signal_type()		((MFC_CORE_READL(MFC_REG_D_VIDEO_SIGNAL_TYPE)	\
						>> MFC_REG_D_VIDEO_SIGNAL_TYPE_FLAG_SHIFT)	\
						& MFC_REG_D_VIDEO_SIGNAL_TYPE_FLAG_MASK)
#define mfc_core_get_colour_description()	((MFC_CORE_READL(MFC_REG_D_VIDEO_SIGNAL_TYPE)	\
						>> MFC_REG_D_COLOUR_DESCRIPTION_FLAG_SHIFT)	\
						& MFC_REG_D_COLOUR_DESCRIPTION_FLAG_MASK)
#define mfc_core_get_primaries()		((MFC_CORE_READL(MFC_REG_D_VIDEO_SIGNAL_TYPE)	\
						>> MFC_REG_D_COLOUR_PRIMARIES_SHIFT)		\
						& MFC_REG_D_COLOUR_PRIMARIES_MASK)
#define mfc_core_get_transfer()			((MFC_CORE_READL(MFC_REG_D_VIDEO_SIGNAL_TYPE)	\
						>> MFC_REG_D_TRANSFER_CHARACTERISTICS_SHIFT)	\
						& MFC_REG_D_TRANSFER_CHARACTERISTICS_MASK)
#define mfc_core_get_matrix_coeff()		((MFC_CORE_READL(MFC_REG_D_VIDEO_SIGNAL_TYPE)	\
						>> MFC_REG_D_MATRIX_COEFFICIENTS_SHIFT)		\
						& MFC_REG_D_MATRIX_COEFFICIENTS_MASK)
#define mfc_core_get_black_bar_pos_x()		((MFC_CORE_READL(MFC_REG_D_BLACK_BAR_START_POS)	\
						>> MFC_REG_D_BLACK_BAR_START_X_SHIFT)		\
						& MFC_REG_D_BLACK_BAR_START_X_MASK)
#define mfc_core_get_black_bar_pos_y()		((MFC_CORE_READL(MFC_REG_D_BLACK_BAR_START_POS)	\
						>> MFC_REG_D_BLACK_BAR_START_Y_SHIFT)		\
						& MFC_REG_D_BLACK_BAR_START_Y_MASK)
#define mfc_core_get_black_bar_image_w()		((MFC_CORE_READL(MFC_REG_D_BLACK_BAR_IMAGE_SIZE)	\
						>> MFC_REG_D_BLACK_BAR_IMAGE_W_SHIFT)	\
						& MFC_REG_D_BLACK_BAR_IMAGE_W_MASK)
#define mfc_core_get_black_bar_image_h()		((MFC_CORE_READL(MFC_REG_D_BLACK_BAR_IMAGE_SIZE)	\
						>> MFC_REG_D_BLACK_BAR_IMAGE_H_SHIFT)	\
						& MFC_REG_D_BLACK_BAR_IMAGE_H_MASK)
#define mfc_core_get_mvc_disp_view_id()		(MFC_CORE_READL(MFC_REG_D_MVC_VIEW_ID)		\
						& MFC_REG_D_MVC_VIEW_ID_DISP_MASK)
#define mfc_core_get_profile()			(MFC_CORE_READL(MFC_REG_D_DECODED_PICTURE_PROFILE)	\
						& MFC_REG_D_DECODED_PIC_PROFILE_MASK)
#define mfc_core_get_level()				((MFC_CORE_READL(MFC_REG_D_DECODED_PICTURE_PROFILE)	\
						>> MFC_REG_D_PIC_LEVEL_SHIFT)	\
						& MFC_REG_D_PIC_LEVEL_MASK)
#define mfc_core_get_luma_bit_depth_minus8()	((MFC_CORE_READL(MFC_REG_D_DECODED_PICTURE_PROFILE)	\
						>> MFC_REG_D_BIT_DEPTH_LUMA_MINUS8_SHIFT)	\
						& MFC_REG_D_BIT_DEPTH_LUMA_MINUS8_MASK)
#define mfc_core_get_chroma_bit_depth_minus8()	((MFC_CORE_READL(MFC_REG_D_DECODED_PICTURE_PROFILE)	\
						>> MFC_REG_D_BIT_DEPTH_CHROMA_MINUS8_SHIFT)	\
						& MFC_REG_D_BIT_DEPTH_CHROMA_MINUS8_MASK)
#define mfc_core_get_display_delay()				\
	((MFC_CORE_READL(MFC_REG_D_DECODED_PICTURE_PROFILE)	\
	>> MFC_REG_D_DISPLAY_DELAY_SHIFT)		\
	& MFC_REG_D_DISPLAY_DELAY_MASK)
#define mfc_core_get_two_core_mode()				\
	((MFC_CORE_READL(MFC_REG_D_DECODED_PICTURE_PROFILE)	\
	>> MFC_REG_D_TWO_MFC_MODE_SHIFT)		\
	& MFC_REG_D_TWO_MFC_MODE_MASK)
#define mfc_core_get_dec_used_flag()		(((unsigned long)(MFC_CORE_READL(MFC_REG_D_USED_DPB_FLAG_UPPER)) << 32) |	\
						MFC_CORE_READL(MFC_REG_D_USED_DPB_FLAG_LOWER))
#define mfc_core_get_enc_idr_flag()				\
	((MFC_CORE_READL(MFC_REG_E_NAL_DONE_INFO)		\
	>> MFC_REG_E_NAL_DONE_INFO_IDR_SHIFT)			\
	& MFC_REG_E_NAL_DONE_INFO_IDR_MASK)
#define mfc_core_get_enc_comp_err()					\
	((MFC_CORE_READL(MFC_REG_E_NAL_DONE_INFO)		\
	>> MFC_REG_E_NAL_DONE_INFO_COMP_ERR_SHIFT)			\
	& MFC_REG_E_NAL_DONE_INFO_COMP_ERR_MASK)
#define mfc_core_get_chroma_format()		(MFC_CORE_READL(MFC_REG_D_CHROMA_FORMAT)		\
						& MFC_REG_D_CHROMA_FORMAT_MASK)
#define mfc_core_get_color_range()		((MFC_CORE_READL(MFC_REG_D_CHROMA_FORMAT)	\
						>> MFC_REG_D_COLOR_RANGE_SHIFT)	\
						& MFC_REG_D_COLOR_RANGE_MASK)
#define mfc_core_get_color_space()		((MFC_CORE_READL(MFC_REG_D_CHROMA_FORMAT)	\
						>> MFC_REG_D_COLOR_SPACE_SHIFT)	\
						& MFC_REG_D_COLOR_SPACE_MASK)
#define mfc_core_get_num_of_tile()		((MFC_CORE_READL(MFC_REG_D_DECODED_STATUS)		\
						>> MFC_REG_DEC_STATUS_NUM_OF_TILE_SHIFT)	\
						& MFC_REG_DEC_STATUS_NUM_OF_TILE_MASK)
#define mfc_core_get_lcu_size()			(MFC_CORE_READL(MFC_REG_D_HEVC_INFO)		\
						& MFC_REG_D_HEVC_INFO_LCU_SIZE_MASK)
#define mfc_core_get_disp_res_change()		((MFC_CORE_READL(MFC_REG_D_VP9_INFO)	\
						>> MFC_REG_D_VP9_INFO_DISP_RES_SHIFT)	\
						& MFC_REG_D_VP9_INFO_DISP_RES_MASK)
#define mfc_core_get_disp_res_change_av1()	((MFC_CORE_READL(MFC_REG_D_AV1_INFO)	\
						>> MFC_REG_D_AV1_INFO_DISP_RES_SHIFT)	\
						& MFC_REG_D_AV1_INFO_DISP_RES_MASK)
#define mfc_core_get_showable_frame()		((MFC_CORE_READL(MFC_REG_D_AV1_INFO)		\
						>> MFC_REG_D_AV1_INFO_SHOWABLE_FRAME_SHIFT)	\
						& MFC_REG_D_AV1_INFO_SHOWABLE_FRAME_MASK)
#define mfc_core_get_multiple_show_frame()	((MFC_CORE_READL(MFC_REG_D_AV1_INFO)		\
						>> MFC_REG_D_AV1_INFO_MULTIPLE_SHOW_SHIFT)	\
						& MFC_REG_D_AV1_INFO_MULTIPLE_SHOW_MASK)
#define mfc_core_get_av1_filmgrain_present() ((MFC_CORE_READL(MFC_REG_D_AV1_INFO)		\
						>> MFC_REG_D_AV1_INFO_FILMGRAIN_PRESENT_SHIFT)	\
						& MFC_REG_D_AV1_INFO_FILMGRAIN_PRESENT_MASK)
#define mfc_core_get_hevc_pic_output_flag()	((MFC_CORE_READL(MFC_REG_D_HEVC_INFO)		\
						>> MFC_REG_D_HEVC_INFO_PIC_OUTPUT_FLAG_SHIFT)	\
						& MFC_REG_D_HEVC_INFO_PIC_OUTPUT_FLAG_MASK)

/* nal queue information */
#define mfc_core_get_nal_q_input_count()		MFC_CORE_READL(MFC_REG_NAL_QUEUE_INPUT_COUNT)
#define mfc_core_get_nal_q_output_count()	MFC_CORE_READL(MFC_REG_NAL_QUEUE_OUTPUT_COUNT)
#define mfc_core_get_nal_q_input_exe_count()	MFC_CORE_READL(MFC_REG_NAL_QUEUE_INPUT_EXE_COUNT)
#define mfc_core_get_nal_q_info()		MFC_CORE_READL(MFC_REG_NAL_QUEUE_INFO)
#define mfc_core_get_nal_q_input_addr()		MFC_CORE_READL(MFC_REG_NAL_QUEUE_INPUT_ADDR)
#define mfc_core_get_nal_q_input_size()		MFC_CORE_READL(MFC_REG_NAL_QUEUE_INPUT_SIZE)
#define mfc_core_get_nal_q_output_addr()		MFC_CORE_READL(MFC_REG_NAL_QUEUE_OUTPUT_ADDR)
#define mfc_core_get_nal_q_output_ize()		MFC_CORE_READL(MFC_REG_NAL_QUEUE_OUTPUT_SIZE)

static inline void mfc_core_reset_nal_queue_registers(struct mfc_core *core)
{
	MFC_CORE_WRITEL(0x0, MFC_REG_NAL_QUEUE_INPUT_COUNT);
	MFC_CORE_WRITEL(0x0, MFC_REG_NAL_QUEUE_OUTPUT_COUNT);
	MFC_CORE_WRITEL(0x0, MFC_REG_NAL_QUEUE_INPUT_EXE_COUNT);
	MFC_CORE_WRITEL(0x0, MFC_REG_NAL_QUEUE_INFO);
}

static inline void mfc_core_update_nal_queue_input(struct mfc_core *core,
	dma_addr_t addr, unsigned int size)
{
	MFC_CORE_WRITEL(addr, MFC_REG_NAL_QUEUE_INPUT_ADDR);
	MFC_CORE_WRITEL(size, MFC_REG_NAL_QUEUE_INPUT_SIZE);
}

static inline void mfc_core_update_nal_queue_output(struct mfc_core *core,
	dma_addr_t addr, unsigned int size)
{
	MFC_CORE_WRITEL(addr, MFC_REG_NAL_QUEUE_OUTPUT_ADDR);
	MFC_CORE_WRITEL(size, MFC_REG_NAL_QUEUE_OUTPUT_SIZE);
}

static inline void mfc_core_update_nal_queue_input_count(struct mfc_core *core,
	unsigned int input_count)
{
	MFC_CORE_WRITEL(input_count, MFC_REG_NAL_QUEUE_INPUT_COUNT);
}

static inline void mfc_core_dec_get_crop_info(struct mfc_core *core,
					struct mfc_ctx *ctx)
{
	struct mfc_dec *dec = ctx->dec_priv;
	u32 left, right, top, bottom;

	left = MFC_CORE_READL(MFC_REG_D_DISPLAY_CROP_INFO1);
	right = left >> MFC_REG_D_SHARED_CROP_RIGHT_SHIFT;
	left = left & MFC_REG_D_SHARED_CROP_LEFT_MASK;
	top = MFC_CORE_READL(MFC_REG_D_DISPLAY_CROP_INFO2);
	bottom = top >> MFC_REG_D_SHARED_CROP_BOTTOM_SHIFT;
	top = top & MFC_REG_D_SHARED_CROP_TOP_MASK;

	dec->cr_left = left;
	dec->cr_right = right;
	dec->cr_top = top;
	dec->cr_bot = bottom;
}

static inline void mfc_core_clear_enc_res_change(struct mfc_core *core)
{
	unsigned int reg = 0;

	reg = MFC_CORE_READL(MFC_REG_E_PARAM_CHANGE);
	reg &= ~(0x7 << 6);
	MFC_CORE_WRITEL(reg, MFC_REG_E_PARAM_CHANGE);
}

static inline void mfc_core_clear_roi_enable(struct mfc_core *core)
{
	unsigned int reg = 0;

	reg = MFC_CORE_READL(MFC_REG_E_RC_ROI_CTRL);
	reg &= ~(0x1);
	MFC_CORE_WRITEL(reg, MFC_REG_E_RC_ROI_CTRL);
}

static inline void mfc_core_update_tag(struct mfc_core *core, struct mfc_ctx *ctx,
				int tag)
{
	if (MFC_CORE_READL(MFC_REG_D_PICTURE_TAG) != tag) {
		mfc_debug(2, "reused src's tag is different so update to %d\n",
					tag);
		MFC_CORE_WRITEL(tag, MFC_REG_D_PICTURE_TAG);
	}
}

static inline void mfc_core_set_enc_src_sbwc(struct mfc_core *core, int onoff)
{
	unsigned int reg = 0;

	reg = MFC_CORE_READL(MFC_REG_E_PARAM_CHANGE);
	reg &= ~(0xf << 14);
	reg |= (onoff << 14);
	MFC_CORE_WRITEL(reg, MFC_REG_E_PARAM_CHANGE);
}

static inline void mfc_core_clear_enc_src_sbwc(struct mfc_core *core)
{
	unsigned int reg = 0;

	reg = MFC_CORE_READL(MFC_REG_E_PARAM_CHANGE);
	reg &= ~(0xf << 14);
	MFC_CORE_WRITEL(reg, MFC_REG_E_PARAM_CHANGE);
}

static inline void mfc_core_set_migration_addr(struct mfc_core *core, struct mfc_ctx *ctx,
		dma_addr_t fw_addr, dma_addr_t common_ctx_addr)
{
	struct mfc_special_buf *ctx_buf = NULL;

	if (ctx->is_drm)
		ctx_buf = &core->drm_common_ctx_buf;
	else
		ctx_buf = &core->common_ctx_buf;

	MFC_CORE_WRITEL(fw_addr, MFC0_REG_RISC_BASE_ADDR);
	MFC_CORE_WRITEL(common_ctx_addr, MFC0_REG_COMMON_CONTEXT_MEM_ADDR);
	MFC_CORE_WRITEL(ctx_buf->daddr, MFC1_REG_COMMON_CONTEXT_MEM_ADDR);
	mfc_core_debug(2, "migration MFC0 FW base: %pad, context: %pad, MFC1 context: %pad\n",
			&fw_addr, &common_ctx_addr, &ctx_buf->daddr);
}

void mfc_core_enc_save_regression_result(struct mfc_core *core);
void mfc_core_dec_save_regression_result(struct mfc_core *core);

void mfc_core_dbg_enable(struct mfc_core *core);
void mfc_core_dbg_disable(struct mfc_core *core);
void mfc_core_dbg_set_addr(struct mfc_core *core);

void mfc_core_otf_set_frame_addr(struct mfc_core *core, struct mfc_ctx *ctx,
		int num_planes);
void mfc_core_otf_set_stream_size(struct mfc_core *core, struct mfc_ctx *ctx,
		unsigned int size);
void mfc_core_otf_set_votf_index(struct mfc_core *core, struct mfc_ctx *ctx,
		int job_id);
void mfc_core_otf_set_hwfc_index(struct mfc_core *core, struct mfc_ctx *ctx,
		int job_id);

unsigned int mfc_get_frame_error_type(struct mfc_ctx *ctx, unsigned int err);

int mfc_core_set_dec_codec_buffers(struct mfc_core_ctx *core_ctx);
int mfc_core_set_enc_codec_buffers(struct mfc_core_ctx *core_ctx);

int mfc_core_set_dec_stream_buffer(struct mfc_core *core, struct mfc_ctx *ctx,
		struct mfc_buf *mfc_buf, unsigned int start_num_byte,
		unsigned int buf_size);

void mfc_core_set_enc_frame_buffer(struct mfc_core *core, struct mfc_ctx *ctx,
		struct mfc_buf *mfc_buf, int num_planes);
int mfc_core_set_enc_stream_buffer(struct mfc_core *core, struct mfc_ctx *ctx,
		struct mfc_buf *mfc_buf);

void mfc_core_get_enc_frame_buffer(struct mfc_core *core, struct mfc_ctx *ctx,
		dma_addr_t addr[], int num_planes);
void mfc_core_set_enc_stride(struct mfc_core *core, struct mfc_ctx *ctx);
void mfc_core_set_enc_config_qp(struct mfc_core *core, struct mfc_ctx *ctx);

int mfc_core_set_dynamic_dpb(struct mfc_core *core, struct mfc_ctx *ctx,
		struct mfc_buf *dst_vb);

void mfc_core_get_img_size(struct mfc_core *core, struct mfc_ctx *ctx,
		enum mfc_get_img_size img_size);
void mfc_core_set_pixel_format(struct mfc_core *core, struct mfc_ctx *ctx,
		unsigned int format);

void mfc_core_print_hdr_plus_info(struct mfc_core *core, struct mfc_ctx *ctx,
		struct hdr10_plus_meta *sei_meta);
void mfc_core_get_hdr_plus_info(struct mfc_core *core, struct mfc_ctx *ctx,
		struct hdr10_plus_meta *sei_meta);
void mfc_core_set_hdr_plus_info(struct mfc_core *core, struct mfc_ctx *ctx,
		struct hdr10_plus_meta *sei_meta);

void mfc_core_set_dec_metadata_buffer(struct mfc_core *core, struct mfc_ctx *ctx);
void mfc_core_get_dec_metadata_sei_nal(struct mfc_core *core, struct mfc_ctx *ctx, int index);

void mfc_core_print_av1_film_grain_info(struct mfc_core *core, struct mfc_ctx *ctx,
		struct av1_film_grain_meta *sei_meta);
void mfc_core_get_av1_film_grain_info(struct mfc_core *core, struct mfc_ctx *ctx,
		struct av1_film_grain_meta *sei_meta);

#endif /* __MFC_CORE_REG_API_H */
