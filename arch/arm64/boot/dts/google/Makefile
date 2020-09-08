dtb-$(CONFIG_SOC_GS101) += \
	gs101.dtb

dtbo-$(CONFIG_BOARD_SLIDER) += \
	gs101-slider.dtbo \
	gs101-slider2.dtbo \
	gs101-whitefin.dtbo \
	gs101-whitefin2.dtbo \
	gs101-whitefin2v2.dtbo

gs101-slider.dtbo-base := gs101.dtb
gs101-slider2.dtbo-base := gs101.dtb
gs101-whitefin.dtbo-base := gs101.dtb
gs101-whitefin2.dtbo-base := gs101.dtb
gs101-whitefin2v2.dtbo-base := gs101.dtb

dtbo-$(CONFIG_BOARD_PASSPORT) += \
	gs101-passport-dev.dtbo

gs101-passport-dev.dtbo-base := gs101.dtb

MKDTIMG_FLAGS += --page_size=4096  --id=/:board_id --rev=/:board_rev
