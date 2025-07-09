ccflags-y += -I$(srctree)/$(src)/inc/
ccflags-y += -I$(src)

obj-$(CONFIG_SND_SOC_TAS25XX) += snd-soc-tas25xx.o

snd-soc-tas25xx-$(CONFIG_SND_SOC_TAS25XX) := \
	src/tas25xx-codec.o \
	src/tas25xx-regmap.o \
	src/tas25xx.o \
	src/tas25xx-logic.o \
	src/tas25xx-regbin-parser.o

ifneq ($(CONFIG_TAS25XX_ALGO),)
snd-soc-tas25xx-objs += algo/src/tas25xx-algo-intf.o \
	algo/src/tas25xx-calib.o \
	algo/src/tas25xx-algo-common.o
endif

ifneq ($(CONFIG_PLATFORM_MTK),)
snd-soc-tas25xx-objs += algo/platform/mtk/tas25xx-mtk-wrapper.o
endif

ifneq ($(CONFIG_TAS25XX_MISC),)
snd-soc-tas25xx-objs += src/tas25xx-misc.o
endif

ifneq ($(CONFIG_PLATFORM_QCOM),)
snd-soc-tas25xx-objs += algo/platform/qcom/tas25xx-algo-qdsp-intf.o
endif

ifneq ($(CONFIG_TISA_KBIN_INTF),)
snd-soc-tas25xx-objs += algo/src/tas25xx-algo-kbin-intf.o \
	algo/src/tas25xx-algo-bin-utils.o
endif

ifneq ($(CONFIG_TAS25XX_CALIB_VAL_BIG),)
snd-soc-tas25xx-objs += algo/src/tas25xx-calib-validation.o
endif

ifneq ($(CONFIG_TISA_SYSFS_INTF),)
snd-soc-tas25xx-objs += algo/src/tas25xx-sysfs-debugfs-utils.o \
	algo/src/smartpa-debug-common.o
endif

ifneq ($(CONFIG_TISA_BIN_INTF),)
snd-soc-tas25xx-objs += algo/src/tas25xx-algo-bin-intf.o
endif

ifneq ($(CONFIG_PLATFORM_EXYNOS),)
snd-soc-tas25xx-objs += algo/platform/exynos/tas25xx-algo-exynos-dsp-intf.o
snd-soc-tas25xx-objs += algo/platform/exynos/skinprot-sysfs-cb.o
endif

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)
CUSTOM_INCLUDE ?= -I$(shell pwd)/

EXTRA_CFLAGS += -DCONFIG_SND_SOC_TAS25XX=1
EXTRA_CFLAGS += -DCONFIG_TAS25XX_MISC=1
EXTRA_CFLAGS += -DCONFIG_TAS25XX_CODEC=1
#EXTRA_CFLAGS += -DCONFIG_TISA_KBIN_INTF=1
#EXTRA_CFLAGS += -DCONFIG_TAS25XX_ALGO=1
EXTRA_CFLAGS += -DCONFIG_TAS25XX_IRQ_BD=1
EXTRA_CFLAGS += $(CUSTOM_INCLUDE)

KBUILD_OPTIONS += CONFIG_SND_SOC_TAS25XX=m
KBUILD_OPTIONS += CONFIG_TAS25XX_MISC=m
#KBUILD_OPTIONS += CONFIG_TAS25XX_ALGO=m
#KBUILD_OPTIONS += CONFIG_TISA_KBIN_INTF=m
KBUILD_OPTIONS += CONFIG_TAS25XX_IRQ_BD=m

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) $(KBUILD_OPTIONS) EXTRA_CFLAGS="$(EXTRA_CFLAGS)" \
	KBUILD_EXTRA_SYMBOLS="$(EXTRA_SYMBOLS)" W=1 $(@)
