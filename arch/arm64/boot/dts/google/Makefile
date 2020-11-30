dtb-$(CONFIG_SOC_GS101) += \
	gs101-a0.dtb \
	gs101-b0.dtb

dtbo-$(CONFIG_BOARD_SLIDER) += \
	gs101-slider.dtbo \
	gs101-slider2.dtbo \
	gs101-whitefin.dtbo \
	gs101-whitefin2.dtbo \
	gs101-whitefin2v2.dtbo \
	gs101-raven.dtbo \
	gs101-oriole.dtbo

gs101-slider.dtbo-base := gs101-a0.dtb
gs101-slider2.dtbo-base := gs101-a0.dtb
gs101-whitefin.dtbo-base := gs101-a0.dtb
gs101-whitefin2.dtbo-base := gs101-a0.dtb
gs101-whitefin2v2.dtbo-base := gs101-a0.dtb
gs101-raven.dtbo-base := gs101-a0.dtb
gs101-oriole.dtbo-base := gs101-a0.dtb

MKDTIMG_FLAGS += --page_size=4096  --id=/:board_id --rev=/:board_rev
