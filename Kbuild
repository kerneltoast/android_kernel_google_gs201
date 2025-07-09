# SPDX-License-Identifier: GPL-2.0

subdir-ccflags-y += \
		-I$(KERNEL_SRC)/../private/google-modules/bms \

obj-$(CONFIG_MSM_BT_POWER) := btpower.o

ccflags-y += -I$(abspath $(KERNEL_SRC)/$(M)) -Werror -Wall
