# SPDX-License-Identifier: GPL-2.0
#
# GS Clock specific Makefile
#

obj-$(CONFIG_EXYNOS_GS_COMMON_CLK)	+= clk_exynos_gs.o

clk_exynos_gs-objs			+= composite.o
clk_exynos_gs-$(CONFIG_SOC_GS101)	+= clk-gs101.o
clk_exynos_gs-$(CONFIG_SOC_GS201)	+= clk-gs201.o
clk_exynos_gs-$(CONFIG_SOC_ZUMA)	+= clk-zuma.o
