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
		GOOGLE_CCD \
		USB_OVERHEAT_MITIGATION \
		PMIC_MAX77729 \
		UIC_MAX77729 \
		CHARGER_MAX77729 \
		CHARGER_MAX77759 \
		CHARGER_MAX77779 \
		CHARGER_MAX77779_I2C \
		MAXQ_MAX77759 \
		MAX77779_SP \
		MAX77779_SP_I2C \
		CHARGER_P9221 \
		MAX1720X_BATTERY \
		MAX_M5 \
		PCA9468 \
		MAX20339 \
		MAX77779_I2CM \
		MAX77779_I2CM_I2C \
		MAX77779_PMIC \
		MAX77779_PMIC_I2C \
		MAX77779_PMIC_IRQ \
		MAX77779_PMIC_PINCTRL \
		MAX77779_PMIC_SGPIO \
		STWLC98 \
		STWC68 \
		LN8411 \
		FG_MAX77779 \
		FG_MAX77779_I2C \
		VIMON_MAX77779 \
		VIMON_MAX77779_I2C \
		FWUPDATE_MAX77779 \
		CHARGER_RT9471 \
		HL7132

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
google-bms-objs += google_eeprom_02.o

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

# google_ccd
obj-$(CONFIG_GOOGLE_CCD)	+= google_ccd.o

# max7729f drivers for the single SSID
obj-$(CONFIG_PMIC_MAX77729)	+= max77729-pmic.o
max77729-pmic-objs += max77729_pmic.o
max77729-pmic-objs += max77759_maxq.o

obj-$(CONFIG_UIC_MAX77729)	+= max77729_uic.o
obj-$(CONFIG_CHARGER_MAX77729)	+= max77729_charger.o

# max77759 if-pmic based device (todo use the standalone)
obj-$(CONFIG_CHARGER_MAX77759)	+= max77759-charger.o
max77759-charger-objs += max77759_charger.o
max77759-charger-objs += max77759_usecase.o

# max77779 if-pmic based device (todo use the standalone)
obj-$(CONFIG_CHARGER_MAX77779)  += max77779-charger.o
max77779-charger-objs += max77779_charger.o
max77779-charger-objs += max77779_usecase.o

obj-$(CONFIG_CHARGER_MAX77779_I2C) += max77779-charger-i2c.o
max77779-charger-i2c-objs += max77779_charger_i2c.o

obj-$(CONFIG_VIMON_MAX77779)  += max77779-vimon.o
max77779-vimon-objs += max77779_vimon.o

obj-$(CONFIG_VIMON_MAX77779_I2C)  += max77779-vimon-i2c.o
max77779-vimon-i2c-objs += max77779_vimon_i2c.o

# Wireless charging
obj-$(CONFIG_CHARGER_P9221)	+= p9221.o
p9221-objs += p9221_charger.o
p9221-objs += p9221_chip.o

# Standalone for pca9468
obj-$(CONFIG_PCA9468)		+= pca9468.o
pca9468-objs += pca9468_charger.o
pca9468-objs += pca9468_gbms_pps.o
pca9468-objs += google_dc_pps.o

# Alternate (untested) standalone for max77729f sans FG
# needs the usecases
obj-$(CONFIG_MAX77729)		+= max77729.o
max77729-objs += max77729_pmic.o
max77729-objs += max77729_uic.o
max77729-objs += max77729_charger.o

# Alternate (untested) standalone for max77759 sans FG
# needs the usecases
obj-$(CONFIG_MAX77759)		+= max77759.o
max77759-objs += max77729_pmic.o
max77759-objs += max77729_uic.o
max77759-objs += max77759_charger.o
max77759-objs += max77759_maxq.o

# Alternate (untested) standalone for max77779 sans FG
# needs the usecases
obj-$(CONFIG_MAX77779)		+= max77779.o
max77779-objs += max77779_charger.o

obj-$(CONFIG_MAX1720X_BATTERY)  += max1720x-battery.o
max1720x-battery-objs += max1720x_battery.o
max1720x-battery-objs += max1720x_outliers.o
max1720x-battery-objs += max_m5.o
max1720x-battery-objs += maxfg_common.o

obj-$(CONFIG_FG_MAX77779)  += max77779-fg.o
max77779-fg-objs += max77779_fg.o
max77779-fg-objs += max77779_fg_model.o
max77779-fg-objs += maxfg_common.o

obj-$(CONFIG_FG_MAX77779_I2C)  += max77779-fg-i2c.o
max77779-fg-i2c-objs += max77779_fg_i2c.o

# OVP
obj-$(CONFIG_MAX20339)	+= max20339.o

# max77779 Scratch Space
obj-$(CONFIG_MAX77779_SP)	+= max77779-sp.o
max77779-sp-objs += max77779_sp.o

obj-$(CONFIG_MAX77779_SP_I2C)	+= max77779-sp-i2c.o
max77779-sp-i2c-objs += max77779_sp_i2c.o

# MAX77779 PMIC
obj-$(CONFIG_MAX77779_I2CM) += max77779_i2cm.o
obj-$(CONFIG_MAX77779_I2CM_I2C) += max77779_i2cm_i2c.o

obj-$(CONFIG_MAX77779_PMIC) += max77779_pmic.o
obj-$(CONFIG_MAX77779_PMIC_IRQ) += max77779_pmic_irq.o
obj-$(CONFIG_MAX77779_PMIC_PINCTRL) += max77779_pmic_pinctrl.o
obj-$(CONFIG_MAX77779_PMIC_SGPIO) += max77779_pmic_sgpio.o

obj-$(CONFIG_MAX77779_PMIC_I2C) += max77779_pmic_i2c.o

# LN8411 DC Charge pump
obj-$(CONFIG_LN8411)	+= ln8411.o
ln8411-objs += ln8411_driver.o
ln8411-objs += ln8411_gbms_pps.o
ln8411-objs += google_dc_pps.o

# MAX77779 Firmware Update
obj-$(CONFIG_FWUPDATE_MAX77779) += max77779-fwupdate.o
max77779-fwupdate-objs += max77779_fwupdate.o

# RT9471 Charger
obj-$(CONFIG_CHARGER_RT9471) += rt9471_charger.o

# Standalone for hl7132
obj-$(CONFIG_HL7132)		+= hl7132.o
hl7132-objs += hl7132_charger.o
hl7132-objs += hl7132_gbms_pps.o
hl7132-objs += google_dc_pps.o

# prevent warnings
WENUMS=-Wno-enum-conversion -Wno-switch

CFLAGS_max77759_charger.o += -Wno-unused-function $(WENUMS)
CFLAGS_max77779_charger.o += -Wno-unused-function $(WENUMS)
CFLAGS_max77779_vimon.o += -Wno-unused-function $(WENUMS)
CFLAGS_max77729_charger.o += -Wno-unused-function $(WENUMS)
CFLAGS_max77779_fg.o += -Wno-unused-function $(WENUMS)
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
CFLAGS_google_ccd.o += $(WENUMS)
CFLAGS_p9221_charger.o += $(WENUMS)
CFLAGS_max77779_sp.o += -Wno-unused-function $(WENUMS)
CFLAGS_ln8411_driver.o += $(WENUMS)
CFLAGS_hl7132_charger.o += $(WENUMS)

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)

subdir-ccflags-y += \
		-I$(KERNEL_SRC)/../private/google-modules/bms \

KBUILD_OPTIONS += $(foreach m,$(GBMS_MODULES),CONFIG_$(m)=m )

EXTRA_CFLAGS += -DDYNAMIC_DEBUG_MODULE
EXTRA_CFLAGS += $(foreach m,$(GBMS_MODULES),-DCONFIG_$(m)_MODULE)

EXTRA_SYMBOLS += $(OUT_DIR)/../private/google-modules/bms/misc/Module.symvers
include $(KERNEL_SRC)/../private/google-modules/soc/gs/Makefile.include

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) W=1 \
	$(KBUILD_OPTIONS) EXTRA_CFLAGS="$(EXTRA_CFLAGS)" KBUILD_EXTRA_SYMBOLS="$(EXTRA_SYMBOLS)" $(@)

print-%:
	@echo $* = $($*)

value-%:
	@echo $($*)
