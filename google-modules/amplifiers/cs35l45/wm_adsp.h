/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * wm_adsp.h  --  Wolfson ADSP support
 *
 * Copyright 2012 Wolfson Microelectronics plc
 *
 * Author: Mark Brown <broonie@opensource.wolfsonmicro.com>
 */

#ifndef __WM_ADSP_H
#define __WM_ADSP_H

#include "cs_dsp.h"
#include "wmfw.h"

#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/compress_driver.h>

/* Return values for wm_adsp_compr_handle_irq */
#define WM_ADSP_COMPR_OK                 0
#define WM_ADSP_COMPR_VOICE_TRIGGER      1

struct wm_adsp_compr;
struct wm_adsp_compr_buf;

struct wm_adsp {
	struct cs_dsp cs_dsp;
	const char *part;
	const char *fwf_name;
	const char *system_name;
	struct snd_soc_component *component;

	unsigned int sys_config_size;

	int fw;

	struct work_struct boot_work;

	bool preloaded;
	bool fatal_error;
	bool tuning_has_prefix;

	struct list_head compr_list;
	struct list_head buffer_list;

	/*
	 * Flag indicating the preloader widget only needs power toggled
	 * on state change rather than held on for the duration of the
	 * preload, useful for devices that can retain firmware memory
	 * across power down.
	 */
	bool toggle_preload;
};

#define WM_ADSP1(wname, num) \
	SND_SOC_DAPM_PGA_E(wname, SND_SOC_NOPM, num, 0, NULL, 0, \
		wm_adsp1_event_v1, SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD)

#define WM_ADSP2_PRELOAD_SWITCH(wname, num) \
	SOC_SINGLE_EXT(wname " Preload Switch", SND_SOC_NOPM, num, 1, 0, \
		wm_adsp2_preloader_get_v1, wm_adsp2_preloader_put_v1)

#define WM_ADSP2(wname, num, event_fn) \
	SND_SOC_DAPM_SPK(wname " Preload", NULL), \
{	.id = snd_soc_dapm_supply, .name = wname " Preloader", \
	.reg = SND_SOC_NOPM, .shift = num, .event = event_fn, \
	.event_flags = SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD, \
	.subseq = 100, /* Ensure we run after SYSCLK supply widget */ }, \
{	.id = snd_soc_dapm_out_drv, .name = wname, \
	.reg = SND_SOC_NOPM, .shift = num, .event = wm_adsp_event_v1, \
	.event_flags = SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD }

#define WM_ADSP_FW_CONTROL(dspname, num) \
	SOC_ENUM_EXT(dspname " Firmware", wm_adsp_fw_enum_v1[num], \
		     wm_adsp_fw_get_v1, wm_adsp_fw_put_v1)

extern const struct soc_enum wm_adsp_fw_enum_v1[];

int wm_adsp1_init_v1(struct wm_adsp *dsp);
int wm_adsp2_init_v1(struct wm_adsp *dsp);
void wm_adsp2_remove_v1(struct wm_adsp *dsp);
int wm_adsp2_component_probe_v1(struct wm_adsp *dsp, struct snd_soc_component *component);
int wm_adsp2_component_remove_v1(struct wm_adsp *dsp, struct snd_soc_component *component);
int wm_halo_init_v1(struct wm_adsp *dsp);

int wm_adsp1_event_v1(struct snd_soc_dapm_widget *w,
		   struct snd_kcontrol *kcontrol, int event);

int wm_adsp_early_event_v1(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event);

irqreturn_t wm_adsp2_bus_error_v1(int irq, void *data);
irqreturn_t wm_halo_bus_error_v1(int irq, void *data);
irqreturn_t wm_halo_wdt_expire_v1(int irq, void *data);

int wm_adsp_event_v1(struct snd_soc_dapm_widget *w,
		  struct snd_kcontrol *kcontrol, int event);

int wm_adsp2_set_dspclk_v1(struct snd_soc_dapm_widget *w, unsigned int freq);

int wm_adsp2_preloader_get_v1(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol);
int wm_adsp2_preloader_put_v1(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol);
int wm_adsp_fw_get_v1(struct snd_kcontrol *kcontrol,
		   struct snd_ctl_elem_value *ucontrol);
int wm_adsp_fw_put_v1(struct snd_kcontrol *kcontrol,
		   struct snd_ctl_elem_value *ucontrol);

int wm_adsp_compr_open_v1(struct wm_adsp *dsp, struct snd_compr_stream *stream);
int wm_adsp_compr_free_v1(struct snd_soc_component *component,
		       struct snd_compr_stream *stream);
int wm_adsp_compr_set_params_v1(struct snd_soc_component *component,
			     struct snd_compr_stream *stream,
			     struct snd_compr_params *params);
int wm_adsp_compr_get_caps_v1(struct snd_soc_component *component,
			   struct snd_compr_stream *stream,
			   struct snd_compr_caps *caps);
int wm_adsp_compr_trigger_v1(struct snd_soc_component *component,
			  struct snd_compr_stream *stream, int cmd);
int wm_adsp_compr_handle_irq_v1(struct wm_adsp *dsp);
int wm_adsp_compr_pointer_v1(struct snd_soc_component *component,
			  struct snd_compr_stream *stream,
			  struct snd_compr_tstamp *tstamp);
int wm_adsp_compr_copy_v1(struct snd_soc_component *component,
		       struct snd_compr_stream *stream,
		       char __user *buf, size_t count);
int wm_adsp_write_ctl_v1(struct wm_adsp *dsp, const char *name,  int type,
		      unsigned int alg, void *buf, size_t len);
int wm_adsp_read_ctl_v1(struct wm_adsp *dsp, const char *name,  int type,
		      unsigned int alg, void *buf, size_t len);

#endif
