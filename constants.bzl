# SPDX-License-Identifier: GPL-2.0-only

"""
GS201 constants.
"""

GS201_DTBS = [
    # keep sorted
    "gs201-a0.dtb",
    "gs201-b0.dtb",
    "gs201-b0_v2-ipop.dtb",
]

GS201_MODULE_OUTS = [
    # keep sorted
    "drivers/gpu/drm/display/drm_display_helper.ko",
    "drivers/i2c/i2c-dev.ko",
    "drivers/misc/eeprom/at24.ko",
    "drivers/perf/arm_dsu_pmu.ko",
    "drivers/scsi/sg.ko",
    "drivers/spi/spidev.ko",
    "drivers/watchdog/softdog.ko",
    "net/mac80211/mac80211.ko",
    "net/wireless/cfg80211.ko",
]

GS201_DEVICE_MODULE_OUTS = [
    # keep sorted
    "drivers/leds/leds-pwm.ko",
    "drivers/regulator/rt4801-regulator.ko",
    "drivers/regulator/tps65132-regulator.ko",
    "drivers/video/backlight/lp855x_bl.ko",
]
