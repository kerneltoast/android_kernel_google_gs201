dtb-$(CONFIG_SOC_GS101) += \
	gs101-a0.dtb \
	gs101-b0.dtb

dtb-$(CONFIG_SOC_GS201) += \
	gs201-a0.dtb \

soc-dtbo-$(CONFIG_SOC_GS101) += \
	gs101-dpm-eng.dtbo \
	gs101-dpm-user.dtbo \

DTC_FLAGS_gs101-dpm-eng += -a 4096
DTC_FLAGS_gs101-dpm-user += -a 4096

dtbo-$(CONFIG_BOARD_SLIDER) += \
	gs101-slider.dtbo \
	gs101-slider2.dtbo \
	gs101-slider2-o6r4.dtbo \
	gs101-whitefin.dtbo \
	gs101-whitefin2.dtbo \
	gs101-whitefin2v2.dtbo \
	gs101-oriole.dtbo \
	gs101-oriole-proto1.dtbo \
	gs101-raven.dtbo \
	gs101-raven-proto1.dtbo

dtbo-$(CONFIG_BOARD_GS201_EMULATOR) += \
	gs201-emulator.dtbo

dtbo-$(CONFIG_BOARD_GS201_HYBRID) += \
	gs201-hybrid.dtbo

gs101-slider.dtbo-base := gs101-a0.dtb
gs101-slider2.dtbo-base := gs101-a0.dtb
gs101-slider2-o6r4.dtbo-base := gs101-a0.dtb
gs101-whitefin.dtbo-base := gs101-a0.dtb
gs101-whitefin2.dtbo-base := gs101-a0.dtb
gs101-whitefin2v2.dtbo-base := gs101-a0.dtb
gs101-oriole.dtbo-base := gs101-b0.dtb
gs101-oriole-proto1.dtbo-base := gs101-a0.dtb
gs101-raven.dtbo-base := gs101-b0.dtb
gs101-raven-proto1.dtbo-base := gs101-b0.dtb

gs101-dpm-eng.dtbo-base := gs101-b0.dtb
gs101-dpm-user.dtbo-base := gs101-b0.dtb

gs201-emulator.dtbo-base := gs201-a0.dtb
gs201-hybrid.dtbo-base := gs201-a0.dtb

MKDTIMG_FLAGS += --page_size=4096  --id=/:board_id --rev=/:board_rev
