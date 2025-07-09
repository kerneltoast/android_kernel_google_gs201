/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * DP audio API header file.
 *
 * Copyright (c) 2023 Google LLC
 *
 */

#ifndef AOC_DP_AUDIO_API_H
#define AOC_DP_AUDIO_API_H

typedef int (*fb_cb_t) (struct snd_pcm_substream *, unsigned int, unsigned int);

void dp_dma_register_fill_buffer_cb(struct snd_pcm_substream *substream, fb_cb_t cb);

#endif /* SAMSUNG_DP_AUDIO_API_H */
