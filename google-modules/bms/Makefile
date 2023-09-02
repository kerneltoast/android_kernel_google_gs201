# SPDX-License-Identifier: GPL-2.0
#
# Makefile for Google Battery Management System
#
GBMS_MODULES =	GOOGLE_BMS \
		GOOGLE_BATTERY \
		GOOGLE_CHARGER \
		GOOGLE_CPM \
		GOOGLE_BEE \
		GOOGLE_DUAL_BATT_GAUGE \
		GOOGLE_DOCK \
		USB_OVERHEAT_MITIGATION \
		PMIC_MAX77729 \
		UIC_MAX77729 \
		CHARGER_MAX77729 \
		CHARGER_MAX77759 \
		MAXQ_MAX77759 \
		CHARGER_P9221 \
		MAX1720X_BATTERY \
		MAX_M5 \
		PCA9468 \
		MAX20339


obj-$(CONFIG_GOOGLE_BMS)	+= google-bms.o
google-bms-objs += google_bms.o
google-bms-objs += gbms_storage.o
# TODO(166536889): enable bee only on the devices supporting it. This will
# require a change in the API since right now storage call into eeprom that
# calls back into storage.
# KBUILD_OPTIONS += CONFIG_GOOGLE_BEE=m \
# obj-$(CONFIG_GOOGLE_BEE)	+= google_eeprom.o
google-bms-objs += google_eeprom.o
google-bms-objs += google_eeprom_01.o
google-bms-objs += gs101_usecase.o

# Battery
obj-$(CONFIG_GOOGLE_BATTERY) += google-battery.o
google-battery-objs += google_battery.o
google-battery-objs += google_ttf.o

# google_charger
obj-$(CONFIG_GOOGLE_CHARGER) += google-charger.o
google-charger-objs += google_charger.o
google-charger-objs += google_dc_pps.o

# google_dual_batt_gauge
obj-$(CONFIG_GOOGLE_DUAL_BATT_GAUGE)	+= google_dual_batt_gauge.o

# charging policy manager, for devices that have more HW chargers
# requires google_dc_pps
obj-$(CONFIG_GOOGLE_CPM)	+= google-cpm.o
google-cpm-objs += google_cpm.o
google-cpm-objs += google_dc_pps.o

# google_dock
obj-$(CONFIG_GOOGLE_DOCK)	+= google_dock.o

# Overheat mitigation driver
obj-$(CONFIG_USB_OVERHEAT_MITIGATION)	+= overheat_mitigation.o

# max7729f drivers for the single SSID
obj-$(CONFIG_PMIC_MAX77729)	+= max77729-pmic.o
max77729-pmic-objs += max77729_pmic.o
max77729-pmic-objs += max77759_maxq.o

obj-$(CONFIG_UIC_MAX77729)	+= max77729_uic.o
obj-$(CONFIG_CHARGER_MAX77729)	+= max77729_charger.o
# Muirwoods drivers for the single SSID (max77729_pmic is shared)
obj-$(CONFIG_CHARGER_MAX77759)	+= max77759_charger.o

# Wireless charging
obj-$(CONFIG_CHARGER_P9221)	+= p9221.o
p9221-objs += p9221_charger.o
p9221-objs += p9221_chip.o

# Standalone for pca9468
obj-$(CONFIG_PCA9468)		+= pca9468.o
pca9468-objs += pca9468_charger.o
pca9468-objs += pca9468_gbms_pps.o
pca9468-objs += google_dc_pps.o

obj-$(CONFIG_PCA9468_GOOGLE)  += pca9468-google.o
pca9468-google-objs += pca_charger.o
pca9468-google-objs += pca9468_gbms_pps.o
pca9468-google-objs += google_dc_pps.o

# Alternate (untested) standalone for max77729f sans FG
obj-$(CONFIG_MAX77729)		+= max77729.o
max77729-objs += max77729_pmic.o
max77729-objs += max77729_uic.o
max77729-objs += max77729_charger.o

# Alternate (untested) standalone for max77759 sans FG
obj-$(CONFIG_MAX77759)		+= max77759.o
max77759-objs += max77729_pmic.o
max77759-objs += max77729_uic.o
max77759-objs += max77729_charger.o
max77759-objs += max77759_maxq.o

obj-$(CONFIG_MAX1720X_BATTERY)  += max1720x-battery.o
max1720x-battery-objs += max1720x_battery.o
max1720x-battery-objs += max1720x_outliers.o
max1720x-battery-objs += max_m5.o

# OVP
obj-$(CONFIG_MAX20339)	+= max20339.o

# prevent warnings
WENUMS=-Wno-enum-conversion -Wno-switch

CFLAGS_max77759_charger.o += -Wno-unused-function $(WENUMS)
CFLAGS_max77729_charger.o += -Wno-unused-function $(WENUMS)
CFLAGS_max1720x_battery.o += $(WENUMS)
CFLAGS_pca9468_charger.o += $(WENUMS)
CFLAGS_pca9468_gbms_pps.o += $(WENUMS)
CFLAGS_pca_charger.o += $(WENUMS)
CFLAGS_google_battery.o += $(WENUMS)
CFLAGS_google_ttf.o += -Wno-format
CFLAGS_google_charger.o += -Wno-enum-conversion
CFLAGS_google_bms.o += -Wno-enum-conversion
CFLAGS_google_cpm.o += $(WENUMS)
CFLAGS_google_dual_batt_gauge.o += $(WENUMS)
CFLAGS_google_dock.o += $(WENUMS)
CFLAGS_p9221_charger.o += $(WENUMS)

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)


KBUILD_OPTIONS += $(foreach m,$(GBMS_MODULES),CONFIG_$(m)=m )

modules:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) W=1 $(KBUILD_OPTIONS) \
		EXTRA_CFLAGS="-DDYNAMIC_DEBUG_MODULE $(foreach m,$(GBMS_MODULES),-DCONFIG_$(m)_MODULE)" \
		$(@)

modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) W=1 $(KBUILD_OPTIONS) $(@)

print-%:
	@echo $* = $($*)

value-%:
	@echo $($*)
