KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build

# The Make variable $(M) must point to the directory that contains the module
# source code (which includes this Makefile). It can either be an absolute or a
# relative path. If it is a relative path, then it must be relative to the
# kernel source directory (KERNEL_SRC). An absolute path can be obtained very
# easily through $(shell pwd). Generating a path relative to KERNEL_SRC is
# difficult and we accept some outside help by letting the caller override the
# variable $(M). Allowing a relative path for $(M) enables us to have the build
# system put output/object files (.o, .ko.) into a directory different from the
# module source directory.
M ?= $(shell pwd)

ifeq ($(WLAN_ROOT),)
# WLAN_ROOT must contain an absolute path (i.e. not a relative path)
KBUILD_OPTIONS := WLAN_ROOT=$(srctree)/$(src)
KBUILD_OPTIONS += MODNAME?=wlan

#By default build for CLD
WLAN_SELECT := CONFIG_QCA_CLD_WLAN=m
WLAN_SELECT += CONFIG_CNSS_QCA6490=y
WLAN_SELECT += CONFIG_BUS_AUTO_SUSPEND=y
KBUILD_OPTIONS += CONFIG_QCA_WIFI_ISOC=0
KBUILD_OPTIONS += CONFIG_QCA_WIFI_2_0=1
KBUILD_OPTIONS += CONFIG_CNSS_OUT_OF_TREE=y
#KBUILD_EXTRA := KBUILD_EXTRA_SYMBOLS=$(O)/../google-modules/wlan/wcn6740/cnss2/Module.symvers
EXTRA_SYMBOLS +=$(OUT_DIR)/../private/google-modules/wlan/wcn6740/cnss2/Module.symvers
KBUILD_OPTIONS += CONFIG_ARCH_MSM=n
KBUILD_OPTIONS += CONFIG_ARCH_QCOM=n
KBUILD_OPTIONS += CONFIG_CNSS2=m
KBUILD_OPTIONS += CONFIG_CNSS2_DEBUG=y
KBUILD_OPTIONS += CONFIG_CNSS_GENL=m
KBUILD_OPTIONS += CONFIG_WCNSS_MEM_PRE_ALLOC=m
KBUILD_OPTIONS += CONFIG_CNSS_UTILS=m
KBUILD_OPTIONS += $(WLAN_SELECT)
KBUILD_OPTIONS += $(KBUILD_EXTRA) # Extra config if any
KBUILD_OPTIONS += TARGET_BUILD_VARIANT=user
KBUILD_OPTIONS += WLAN_COMMON_ROOT=../qca-wifi-host-cmn
endif

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) modules $(KBUILD_OPTIONS) KBUILD_EXTRA_SYMBOLS="$(EXTRA_SYMBOLS)"

modules_install:
	$(MAKE) INSTALL_MOD_STRIP=1 M=$(M) -C $(KERNEL_SRC) modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) clean $(KBUILD_OPTIONS)
