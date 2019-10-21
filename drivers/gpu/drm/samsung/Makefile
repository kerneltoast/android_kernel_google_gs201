# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the drm device driver.  This driver provides support for the
# Direct Rendering Infrastructure (DRI) in XFree86 4.1.0 and higher.

ccflags-$(CONFIG_SOC_EXYNOS9820)		:= -I$(src)/cal_9820
ccflags-$(CONFIG_SOC_GS101)			:= -I$(src)/cal_9845
ccflags-$(CONFIG_DRM_PANEL_SAMSUNG_COMMON)	+= -I$(src)/../panel

subdir-ccflags-y			:= -I$(src)
subdir-ccflags-$(CONFIG_SOC_EXYNOS9820)	+= -I$(src)/cal_9820
subdir-ccflags-$(CONFIG_SOC_GS101)	+= -I$(src)/cal_9845

exynos-drm-y := exynos_drm_drv.o exynos_drm_crtc.o exynos_drm_fb.o	\
		exynos_drm_plane.o exynos_drm_format.o exynos_drm_bts.o	\
		exynos_drm_debug.o

exynos-drm-$(CONFIG_DRM_FBDEV_EMULATION) += exynos_drm_fbdev.o
exynos-drm-$(CONFIG_DRM_SAMSUNG_DECON)	+= exynos_drm_decon.o
exynos-drm-$(CONFIG_DRM_SAMSUNG_DPP)	+= exynos_drm_dpp.o
exynos-drm-$(CONFIG_DRM_SAMSUNG_DSI)	+= exynos_drm_dsim.o

exynos-drm-$(CONFIG_SOC_EXYNOS9820)	+= cal_9820/decon_reg.o cal_9820/dsim_reg.o cal_9820/dpp_reg.o
exynos-drm-$(CONFIG_SOC_GS101)		+= cal_9845/decon_reg.o cal_9845/dsim_reg.o cal_9845/dpp_reg.o

obj-$(CONFIG_DRM_SAMSUNG)		+= exynos-drm.o
