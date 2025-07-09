obj-$(CONFIG_SND_SOC_TAS256X) += snd-soc-tas256x.o

snd-soc-tas256x-objs := \
	os_layer/src/tas256x-regmap.o \
	os_layer/src/tas256x-codec.o \
	physical_layer/src/tas256x.o \
	logical_layer/src/tas256x-logic.o

ifneq ($(CONFIG_TAS256X_MISC),)
snd-soc-tas256x-objs += misc/tas256x-misc.o
endif

ifneq ($(CONFIG_TAS25XX_ALGO),)
snd-soc-tas256x-objs += algo/src/tas25xx-algo-intf.o \
	algo/src/tas25xx-calib.o \
	algo/src/tas25xx-algo-common.o
endif

ifneq ($(CONFIG_PLATFORM_QCOM),)
snd-soc-tas256x-objs += algo/platform/qcom/tas25xx-algo-qdsp-intf.o
endif

ifneq ($(CONFIG_TISA_KBIN_INTF),)
snd-soc-tas256x-objs += algo/src/tas25xx-algo-kbin-intf.o \
	algo/src/tas25xx-algo-bin-utils.o
endif

ifneq ($(CONFIG_TAS25XX_CALIB_VAL_BIG),)
snd-soc-tas256x-objs += algo/src/tas25xx-calib-validation.o
endif

ifneq ($(CONFIG_TISA_DEBUGFS_INTF),)
snd-soc-tas256x-objs += algo/src/tas25xx-algo-debugfs.o \
	logical_layer/src/smartpa-debug-common.o
endif

ifneq ($(CONFIG_TISA_BIN_INTF),)
snd-soc-tas256x-objs += algo/src/tas25xx-algo-bin-intf.o
endif

ifneq ($(CONFIG_TISA_KBIN_INTF),)
snd-soc-tas256x-objs += algo/platform/mtk/tas256x-mtk-wrapper.o
endif

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)
CUSTOM_INCLUDE ?= -I$(shell pwd)/

EXTRA_CFLAGS += -DCONFIG_SND_SOC_TAS256X=1
EXTRA_CFLAGS += -DCONFIG_TAS256X_MISC=1
EXTRA_CFLAGS += -DCONFIG_TAS256X_CODEC=1
EXTRA_CFLAGS += -DCONFIG_TAS256X_REGMAP=1
EXTRA_CFLAGS += $(CUSTOM_INCLUDE)

KBUILD_OPTIONS += CONFIG_SND_SOC_TAS256X=m \
		  CONFIG_TAS256X_MISC=m \
#		  CONFIG_TAS25XX_ALGO=m \
#		  CONFIG_TISA_KBIN_INTF=m \
#		  CONFIG_TAS25XX_CALIB_VAL_BIG=m \
#		  CONFIG_TISA_DEBUGFS_INTF=m \
#		  CONFIG_TISA_BIN_INTF=m \

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) $(KBUILD_OPTIONS) EXTRA_CFLAGS="$(EXTRA_CFLAGS)" W=1 $(@)

