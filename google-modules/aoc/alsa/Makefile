#
# ASoC-based ALSA driver gluing all the audio amplifier drivers
#
obj-$(CONFIG_AOC_DRIVER) += aoc_alsa_dev.o
obj-$(CONFIG_AOC_DRIVER) += aoc_alsa_dev_util.o
aoc_alsa_dev_util-objs := aoc_alsa_path.o aoc_alsa_drv.o \
                                aoc_alsa_pcm.o aoc_alsa_voice.o \
                                aoc_alsa_compr.o aoc_alsa_ctl.o aoc_alsa_hw.o \
                                aoc_alsa_nohost.o aoc_alsa_incall.o  aoc_alsa_voip.o
aoc_alsa_dev-objs := aoc_alsa_card.o

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)
KBUILD_OPTIONS += CONFIG_AOC_DRIVER=m
EXTRA_CFLAGS=-I$(KERNEL_SRC)/../google-modules/aoc_ipc -I$(KERNEL_SRC)/../google-modules/aoc

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) W=1 $(KBUILD_OPTIONS) $(EXTRA_CFLAGS) KBUILD_EXTRA_SYMBOLS=$(OUT_DIR)/../google-modules/aoc/Module.symvers $(@)
