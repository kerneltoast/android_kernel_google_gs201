/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef SAMSUNG_DP_DMA_H
#define SAMSUNG_DP_DMA_H

#define USE_AOC

#define DP_AUDIO_PARAM_CHANNEL_OFFSET 0
#define DP_AUDIO_PARAM_WIDTH_OFFSET 16
#define DP_AUDIO_PARAM_RATE_OFFSET 19

enum audio_param {
	DPADO_RATE,
	DPADO_WIDTH,
	DPADO_CHANNEL,
	DPADO_PARAM_COUNT,
};

struct dp_audio_pdata {
	u32 id;
	u32 fifo_addr;
	u32 rate;
	u32 width;
	u32 channel;
	int idle_ip_index;
	char *ch_name;
	struct runtime_data *prtd;
};

void dp_ado_switch_set_state(int state);

#endif /* SAMSUNG_DP_DMA_H */
