#
# Copyright (C) 2022, Broadcom.
#
#      Unless you and Broadcom execute a separate written software license
# agreement governing use of this software, this software is licensed to you
# under the terms of the GNU General Public License version 2 (the "GPL"),
# available at http://www.broadcom.com/licenses/GPLv2.php, with the
# following added to such license:
#
#      As a special exception, the copyright holders of this software give you
# permission to link this software with independent modules, and to copy and
# distribute the resulting executable under terms of your choice, provided that
# you also meet, for each linked independent module, the terms and conditions of
# the license of that module.  An independent module is a module which is not
# derived from this software.  The special exception does not apply to any
# modifications of the software.
#
#
# <<Broadcom-WL-IPTag/Open:>>
#
# bcmdhd

# Path to the module source
M ?= $(shell pwd)

KBUILD_OPTIONS += BCMDHD_ROOT=$(srctree)/$(src)

ifneq ($(CONFIG_WLAN_TRACKER),)
KBUILD_EXTRA_SYMBOLS=$(OUT_DIR)/../google-modules/wlan/wlan_ptracker/Module.symvers
endif

all:
	@echo "$(MAKE) -C $(KERNEL_SRC) M=$(M) modules $(KBUILD_OPTIONS)"
	@$(MAKE) -C $(KERNEL_SRC) M=$(M) modules $(KBUILD_OPTIONS)

modules_install:
	@echo "$(MAKE) INSTALL_MOD_STRIP=1 M=$(M) -C $(KERNEL_SRC) modules_install"
	@$(MAKE) INSTALL_MOD_STRIP=1 M=$(M) -C $(KERNEL_SRC) modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) clean $(KBUILD_OPTIONS)
