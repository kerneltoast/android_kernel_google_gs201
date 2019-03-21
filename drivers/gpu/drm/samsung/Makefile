# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the drm device driver.  This driver provides support for the
# Direct Rendering Infrastructure (DRI) in XFree86 4.1.0 and higher.

exynos-drm-y := exynos_drm_drv.o exynos_drm_crtc.o exynos_drm_fb.o	\
		exynos_drm_plane.o exynos_drm_format.o exynos_drm_bts.o	\
		exynos_drm_debug.o

exynos-drm-$(CONFIG_DRM_FBDEV_EMULATION) += exynos_drm_fbdev.o
exynos-drm-$(CONFIG_DRM_SAMSUNG_DECON)	+= exynos_drm_decon.o cal_9820/decon_reg.o
exynos-drm-$(CONFIG_DRM_SAMSUNG_DPP)	+= exynos_drm_dpp.o cal_9820/dpp_reg.o
exynos-drm-$(CONFIG_DRM_SAMSUNG_DSI)	+= exynos_drm_dsim.o cal_9820/dsim_reg.o

obj-$(CONFIG_DRM_SAMSUNG)		+= exynos-drm.o
