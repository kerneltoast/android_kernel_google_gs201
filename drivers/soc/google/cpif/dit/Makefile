# SPDX-License-Identifier: GPL-2.0
# Makefile of dit

ccflags-y += -Wformat
ccflags-y += -Wformat-zero-length

obj-$(CONFIG_EXYNOS_DIT) += exynos_dit.o
exynos_dit-y += dit.o dit_net.o dit_hal.o

ifeq ($(CONFIG_EXYNOS_DIT_VERSION),0x02020000)
  exynos_dit-y += dit_2_2_0.o
else
  exynos_dit-y += dit_2_1_0.o
endif
