# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the drm device driver.  This driver provides support for the
# Direct Rendering Infrastructure (DRI) in XFree86 4.1.0 and higher.

subdir-ccflags-y += -I$(srctree)/$(src)/../common/include
subdir-ccflags-y += -I$(srctree)/$(src)/include/uapi
subdir-ccflags-y += -I$(KERNEL_SRC)/../private/google-modules/bms

ccflags-y += -I$(srctree)/$(src)/cal_common
ccflags-$(CONFIG_SOC_ZUMA) += -DCONFIG_DRM_LEGACY=1

ifneq ($(filter y, $(CONFIG_SOC_GS101) $(CONFIG_SOC_GS201)),)
ccflags-y += -I$(srctree)/$(src)/cal_9845
exynos-drm-y += cal_9845/decon_reg.o
exynos-drm-y += cal_9845/dsim_reg.o
exynos-drm-y += cal_9845/dpp_reg.o
exynos-drm-y += cal_9845/dqe_reg.o
exynos-drm-y += cal_9845/hdr_reg.o
endif

ccflags-$(CONFIG_SOC_GS201) += -I$(srctree)/$(src)/cal_9855
exynos-drm-$(CONFIG_SOC_GS201) += cal_9855/decon_reg.o
exynos-drm-$(CONFIG_SOC_GS201) += cal_9855/dpp_reg.o
exynos-drm-$(CONFIG_SOC_GS201) += cal_9855/dqe_reg.o
ifeq ($(CONFIG_DRM_SAMSUNG_DP),y)
exynos-drm-$(CONFIG_SOC_GS201) += cal_9855/dp_reg.o
endif

ccflags-$(CONFIG_SOC_ZUMA) += -I$(srctree)/$(src)/cal_9865
exynos-drm-$(CONFIG_SOC_ZUMA) += cal_9865/decon_reg.o
exynos-drm-$(CONFIG_SOC_ZUMA) += cal_9865/dsim_reg.o
exynos-drm-$(CONFIG_SOC_ZUMA) += cal_9865/dpp_reg.o
exynos-drm-$(CONFIG_SOC_ZUMA) += cal_9865/dqe_reg.o
exynos-drm-$(CONFIG_SOC_ZUMA) += cal_9865/hdr_reg.o
ifeq ($(CONFIG_DRM_SAMSUNG_DP),y)
exynos-drm-$(CONFIG_SOC_ZUMA) += cal_9865/dp_reg.o
exynos-drm-$(CONFIG_SOC_ZUMA) += displayport/dp_zuma.o
endif

exynos-drm-y += exynos_drm_drv.o
exynos-drm-y += exynos_drm_crtc.o
exynos-drm-y += exynos_drm_connector.o
exynos-drm-y += exynos_drm_fb.o
exynos-drm-y += exynos_drm_format.o
exynos-drm-y += exynos_drm_gem.o
exynos-drm-y += exynos_drm_plane.o

exynos-drm-y += exynos_drm_debug.o
exynos-drm-y += exynos_drm_dqe.o
exynos-drm-y += exynos_drm_hibernation.o
exynos-drm-y += exynos_drm_partial.o
exynos-drm-y += exynos_drm_recovery.o

exynos-drm-$(CONFIG_DRM_SAMSUNG_DECON)		+= exynos_drm_decon.o
exynos-drm-$(CONFIG_DRM_SAMSUNG_DPP)		+= exynos_drm_dpp.o
exynos-drm-$(CONFIG_DRM_SAMSUNG_DSI)		+= exynos_drm_dsim.o
exynos-drm-$(CONFIG_DRM_SAMSUNG_TUI)		+= exynos_drm_tui.o
exynos-drm-$(CONFIG_DRM_SAMSUNG_WB)		+= exynos_drm_writeback.o

ccflags-$(CONFIG_DRM_SAMSUNG_DP)		+= -I$(srctree)/drivers/usb
ccflags-$(CONFIG_DRM_SAMSUNG_DP)		+= -I$(srctree)/../private/google-modules/soc/gs/include

ifeq ($(CONFIG_SOC_ZUMA),y)
exynos-drm-$(CONFIG_DRM_SAMSUNG_DP)		+= exynos_drm_dp.o
else
exynos-drm-$(CONFIG_DRM_SAMSUNG_DP)		+= exynos_drm_dp-whi_placeholder.o
endif

exynos-drm-$(CONFIG_EXYNOS_BTS)			+= exynos_drm_bts.o

ccflags-$(CONFIG_DRM_SAMSUNG_DP_AUDIO) += -I$(srctree)/../private/google-modules/aoc/alsa
exynos-drm-audio-$(CONFIG_DRM_SAMSUNG_DP_AUDIO)	+= dp_audio/dp_dma.o

obj-$(CONFIG_DRM_SAMSUNG)			+= exynos-drm.o
ifeq ($(CONFIG_DRM_SAMSUNG_DP_AUDIO),y)
obj-$(CONFIG_DRM_SAMSUNG)			+= exynos-drm-audio.o
endif
obj-y	+= panel/
