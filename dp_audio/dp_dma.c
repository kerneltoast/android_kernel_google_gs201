// SPDX-License-Identifier: GPL-2.0-only
/*
 * dp_dma.c  --  ALSA Soc Audio Layer
 *
 * Copyright (c) 2018 Samsung Electronics Co. Ltd.
 *
 */

#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/iommu.h>
#include <linux/dma/dma-pl330.h>
#include <linux/extcon-provider.h>
#include <linux/sysfs.h>

#include <sound/soc.h>
#include <sound/pcm_params.h>

/* switch device header */
#if IS_ENABLED(CONFIG_SWITCH)
#include <linux/switch.h>
#endif /* CONFIG_SWITCH */

#include <soc/google/exynos-cpupm.h>

#include "dp_audio.h"
#include "dp_dma.h"
#include "../../../../drivers/extcon/extcon.h"
#include "../exynos_drm_dp.h"

#ifdef USE_AOC
#include "google-aoc-enum.h"
#endif

#define DPAUDIO_SAMPLING_RATES (SNDRV_PCM_RATE_KNOT)
#define DPAUDIO_SAMPLE_FORMATS (SNDRV_PCM_FMTBIT_S16 \
				| SNDRV_PCM_FMTBIT_S24 \
				| SNDRV_PCM_FMTBIT_S32)

#define PERIOD_MIN		4
#define ST_RUNNING		BIT(0)
#define ST_OPENED		BIT(1)

#define RX_SRAM_SIZE		(0x2000)	/* 8 KB */
#define MAX_DEEPBUF_SIZE	(0xA000)	/* 40 KB */

static void __iomem *dp_debug_sfr;
static struct device *dp_dma_devs[2];
static int dp_dma_count;
static void *dp_ado_reserved_mem = NULL;

#if IS_ENABLED(CONFIG_EXYNOS_MIPI_DISPLAYPORT) || \
	IS_ENABLED(CONFIG_EXYNOS_DISPLAYPORT) || \
	IS_ENABLED(CONFIG_DRM_SAMSUNG_DP)
extern struct blocking_notifier_head dp_ado_notifier_head;
#endif

static const struct snd_pcm_hardware dma_hardware = {
	.info			= SNDRV_PCM_INFO_INTERLEAVED |
				  SNDRV_PCM_INFO_BLOCK_TRANSFER |
				  SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID,
	.formats		= SNDRV_PCM_FMTBIT_S24_LE |
				  SNDRV_PCM_FMTBIT_U24_LE |
				  SNDRV_PCM_FMTBIT_S20_3LE |
				  SNDRV_PCM_FMTBIT_U20_3LE |
				  SNDRV_PCM_FMTBIT_S16_LE |
				  SNDRV_PCM_FMTBIT_U16_LE,
	.channels_min		= 1,
	.channels_max		= 8,
	.buffer_bytes_max	= 1024 * 1024,
	.period_bytes_min	= 128,
	.period_bytes_max	= 256 * 1024,
	.periods_min		= 2,
	.periods_max		= 128,
	.fifo_size		= 32,
};

struct s3c_dma_params {
	struct s3c2410_dma_client *client;	/* stream identifier */
	int channel;				/* Channel ID */
	dma_addr_t dma_addr;
	int dma_size;				/* Size of the DMA transfer */
	unsigned long ch;
	struct samsung_dma_ops *ops;
	struct device *sec_dma_dev; 		/* stream identifier */
	char *ch_name;
	bool esa_dma;
	bool compr_dma;
};

struct runtime_data {
	spinlock_t lock;
	int state;
	unsigned int dma_loaded;
	unsigned int dma_period;
	dma_addr_t dma_start;
	dma_addr_t dma_pos;
	dma_addr_t dma_end;
	struct s3c_dma_params *params;
	struct snd_pcm_hardware hw;
	struct dp_audio_config dp_config;
	dma_addr_t irq_pos;
	u32 irq_cnt;
	fb_cb_t fillbuffer_cb;
};

#ifdef USE_AOC
static int remap_dai_id(struct device *dev, int id)
{
	switch(id) {
	case DP_DMA_RX:
		return 0;
	default:
		dev_err(dev, "unsupport dai id %x\n", id);
		return -1;
	}
}
#else
static int remap_dai_id(struct device *dev, int id)
{
	return id;
}
#endif

void dp_dma_register_fill_buffer_cb(struct snd_pcm_substream *substream, fb_cb_t cb)
{
	struct runtime_data *prtd = substream->runtime->private_data;
	struct device *dev = substream->pcm->card->dev;

	if (prtd != NULL) {
		dev_dbg(dev, "cb = %pK\n", cb);
		prtd->fillbuffer_cb = cb;
	}
}
EXPORT_SYMBOL_GPL(dp_dma_register_fill_buffer_cb);

#if IS_ENABLED(CONFIG_SND_SAMSUNG_IOMMU)
struct dma_iova {
	dma_addr_t		iova;
	dma_addr_t		pa;
	unsigned char		*va;
	struct list_head	node;
};

static LIST_HEAD(iova_list);
#endif

static struct reserved_mem *dp_ado_rmem;

static void *dp_ado_rmem_vmap(struct reserved_mem *rmem)
{
	phys_addr_t phys = rmem->base;
	size_t size = rmem->size;
	unsigned int num_pages = (unsigned int)DIV_ROUND_UP(size, PAGE_SIZE);
	pgprot_t prot = pgprot_writecombine(PAGE_KERNEL);
	struct page **pages, **page;
	void *vaddr = NULL;
	pr_debug("%s: rmem=%pK\n", __func__, rmem);
	pages = kcalloc(num_pages, sizeof(pages[0]), GFP_KERNEL);
	if (!pages) {
		goto out;
	}

	for (page = pages; (page - pages < num_pages); page++) {
		*page = phys_to_page(phys);
		phys += PAGE_SIZE;
	}

	vaddr = vmap(pages, num_pages, VM_MAP, prot);
	kfree(pages);
out:
	pr_debug("%s: vaddr=%pK\n", __func__, vaddr);
	return vaddr;
}

static int __init dp_ado_rmem_setup(struct reserved_mem *rmem)
{
	pr_info("%s: base=%pa, size=%pa\n", __func__, &rmem->base, &rmem->size);
	dp_ado_rmem = rmem;
	return 0;
}

RESERVEDMEM_OF_DECLARE(dp_ado_rmem, "exynos,dp_ado_rmem", dp_ado_rmem_setup);

#if !IS_ENABLED(CONFIG_SWITCH) && !IS_ENABLED(CONFIG_EXTCON)
#error "need to select SWITCH or EXTCON"
#endif

static void dp_audio_stop(struct runtime_data *prtd)
{
	if (prtd == NULL || prtd->state != ST_RUNNING)
		return;
	pr_info("dp audio is disconnected\n");
	prtd->state &= ~ST_RUNNING;
	prtd->dp_config.audio_state = DP_AUDIO_WAIT_BUF_FULL;
	dp_audio_config(&prtd->dp_config);
}

#if IS_ENABLED(CONFIG_SWITCH)
struct switch_dev dp_ado_switch;
void dp_ado_switch_set_state(int state)
{
	pr_info("dp audio switch event = %d\n", state);
	switch_set_state(&dp_ado_switch, (state  < 0) ? 1 : 1);
}

EXPORT_SYMBOL_GPL(dp_ado_switch_set_state);

#elif IS_ENABLED(CONFIG_EXTCON)
static const unsigned int extcon_id[] = {
	EXTCON_DISP_HDMI,
	EXTCON_NONE,
};

struct extcon_dev *dp_ado_extcon;
void dp_ado_switch_set_state(int state)
{
	struct dp_audio_pdata *pdata;
	int i;

	pr_info("dp audio switch event = %x\n", state);

	if (state < 0) {
		for (i = 0; i < dp_dma_count; i++) {
			pdata = dev_get_drvdata(dp_dma_devs[i]);
			pdata->channel = 0;
			pdata->width = 0;
			pdata->rate = 0;
			dp_audio_stop(pdata->prtd);
		}
	} else {
		for (i = 0; i < dp_dma_count; i++) {
			pdata = dev_get_drvdata(dp_dma_devs[i]);
			pdata->channel = ((state >> DP_AUDIO_PARAM_CHANNEL_OFFSET) & 0xff);
			pdata->width = ((state >> DP_AUDIO_PARAM_WIDTH_OFFSET) & 0x7);
			pdata->rate = ((state >> DP_AUDIO_PARAM_RATE_OFFSET) & 0x7f);
		}
	}

	extcon_set_state_sync(dp_ado_extcon, EXTCON_DISP_HDMI, (state  < 0) ? 0 : 1);
}

EXPORT_SYMBOL_GPL(dp_ado_switch_set_state);

static bool dp_ado_extcon_is_avail(void)
{
	int ret;

	ret = extcon_get_state(dp_ado_extcon, EXTCON_DISP_HDMI);
	if (ret <= 0) {
		pr_info("dp audio is disconnected\n");
		return false;
	}

	return true;
}
#endif


static void audio_buffdone(void *data);

/* dma_enqueue
 *
 * place a dma buffer onto the queue for the dma system
 * to handle.
 */
static void dma_enqueue(struct snd_pcm_substream *substream)
{
	struct runtime_data *prtd = substream->runtime->private_data;
	struct device *dev = substream->pcm->card->dev;
	dma_addr_t pos = prtd->dma_pos;
	unsigned long limit;
	struct samsung_dma_prep dma_info;

	dev_dbg(dev, "Entered %s\n", __func__);

	limit = (prtd->dma_end - prtd->dma_start) / prtd->dma_period;

	dev_dbg(dev, "%s: loaded %d, limit %lu\n",
				__func__, prtd->dma_loaded, limit);

	dma_info.cap = DMA_CYCLIC;
	dma_info.direction = DMA_MEM_TO_DEV;
	dma_info.fp = audio_buffdone;
	dma_info.fp_param = substream;
	dma_info.period = prtd->dma_period;
	dma_info.len = prtd->dma_period*limit;

	if (prtd->params->esa_dma || samsung_dma_has_infiniteloop()) {
		dma_info.buf = prtd->dma_pos;
		dma_info.infiniteloop = (unsigned int)limit;
		prtd->params->ops->prepare(prtd->params->ch, &dma_info);
	} else {
		dma_info.infiniteloop = 0;
		while (prtd->dma_loaded < limit) {
			dev_dbg(dev, "dma_loaded: %d\n", prtd->dma_loaded);

			if ((pos + dma_info.period) > prtd->dma_end) {
				dma_info.period  = prtd->dma_end - pos;
				dev_dbg(dev, "%s: corrected dma len %ld\n",
						__func__, dma_info.period);
			}

			dma_info.buf = pos;
			prtd->params->ops->prepare(prtd->params->ch, &dma_info);

			prtd->dma_loaded++;
			pos += prtd->dma_period;
			if (pos >= prtd->dma_end)
				pos = prtd->dma_start;
		}
		prtd->dma_pos = pos;
	}
}

static void audio_buffdone(void *data)
{
	struct snd_pcm_substream *substream = data;
	struct runtime_data *prtd;
	struct device *dev = substream->pcm->card->dev;
	dma_addr_t src, dst, pos;
#ifdef USE_AOC
	int buf_index_to_fill;
#endif
	dev_dbg(dev, "Entered %s\n", __func__);

	if (!substream)
		return;

	prtd = substream->runtime->private_data;
	if (prtd->state & ST_RUNNING) {
		prtd->params->ops->getposition(prtd->params->ch, &src, &dst);
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
			pos = dst - prtd->dma_start;
		else
			pos = src - prtd->dma_start;

		prtd->irq_cnt++;
		prtd->irq_pos = pos;
		pos /= prtd->dma_period;
#ifdef USE_AOC
		buf_index_to_fill = (pos > 0)? pos - 1:
			((prtd->dma_end - prtd->dma_start) / prtd->dma_period) - 1;
#endif
		pos = prtd->dma_start + (pos * prtd->dma_period);
		if (pos >= prtd->dma_end)
			pos = prtd->dma_start;

		prtd->dma_pos = pos;
#ifdef USE_AOC
		if (prtd->fillbuffer_cb) {
			prtd->fillbuffer_cb(substream,
				(unsigned int)buf_index_to_fill * prtd->dma_period,
				prtd->dma_period);
		}
#else
		snd_pcm_period_elapsed(substream);
#endif
		if (!prtd->params->esa_dma && !samsung_dma_has_circular()) {
			spin_lock(&prtd->lock);
			prtd->dma_loaded--;
			if (!samsung_dma_has_infiniteloop())
				dma_enqueue(substream);
			spin_unlock(&prtd->lock);
		}
	}
}

static int dma_hw_params(struct snd_soc_component *component,
		struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct runtime_data *prtd = runtime->private_data;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	unsigned long totbytes = params_buffer_bytes(params);
	int burst_len;
	struct samsung_dma_req req;
	struct samsung_dma_config config;
	struct snd_soc_dai *dai = asoc_rtd_to_cpu(rtd, 0);
	struct dp_audio_pdata *pdata;
	struct device *dev = component->dev;
	int dai_id = remap_dai_id(dev, dai->id);

	dev_dbg(dev, "Entered %s prtd = %pK\n", __func__, prtd);

	if (dai_id < 0 || dai_id >= ARRAY_SIZE(dp_dma_devs)) {
		dev_err(dev, "unknown dai->id %x\n", dai->id);
		return -EINVAL;
	}

	pdata = dev_get_drvdata(dp_dma_devs[dai_id]);

	if (!dp_ado_extcon_is_avail())
		return -EIO;

	burst_len = snd_pcm_format_physical_width(params_format(params)) *
			params_channels(params) / 32;

	/* this may get called several times by oss emulation
	 * with different params -HW */
	if (prtd->params == NULL) {
		prtd->params = kzalloc(sizeof(struct s3c_dma_params), GFP_KERNEL);
		if (prtd->params == NULL) {
			return -ENOMEM;
		}

		dev_dbg(dev, "params %pK, client %pK, channel %d\n", prtd->params,
			prtd->params->client, prtd->params->channel);

		prtd->params->ops = samsung_dma_get_ops();
		req.cap = DMA_CYCLIC;
		req.client = prtd->params->client;

		config.direction = DMA_MEM_TO_DEV;
		config.width = 4;
		config.maxburst = burst_len;
		config.fifo = pdata->fifo_addr;

		prtd->params->ch_name = pdata->ch_name;
		prtd->params->ch = prtd->params->ops->request(prtd->params->channel,
				&req, dp_dma_devs[dai_id], prtd->params->ch_name);

		dev_dbg(dev, "samsung_dma_has_circular=%d\n", samsung_dma_has_circular());
		dev_dbg(dev, "samsung_dma_has_infiniteloop=%d\n", samsung_dma_has_infiniteloop());
		dev_dbg(dev, "dma_request: ch %d, req %pK, dev %pK, ch_name [%s]\n",
			prtd->params->channel, &req, dp_dma_devs[dai_id],
			prtd->params->ch_name);
		prtd->params->ops->config(prtd->params->ch, &config);
	}

	if (params != NULL) {
		runtime->access = params_access(params);
		runtime->format = params_format(params);
		runtime->subformat = params_subformat(params);
		runtime->period_size = params_period_bytes(params);
		runtime->rate = params_rate(params);
		runtime->channels = params_channels(params);
		runtime->sample_bits = snd_pcm_format_width(params_format(params));
	}

	dev_dbg(dev, "period_size: %lu\n", runtime->period_size);
	dev_dbg(dev, "rate: %u\n", runtime->rate);
	dev_dbg(dev, "channels: %u\n", runtime->channels);
	dev_dbg(dev, "sample_bits: %u\n", runtime->sample_bits);
	dev_dbg(dev, "burst_len: %u\n", burst_len);

	switch (runtime->rate) {
	case 32000:
		prtd->dp_config.audio_fs = FS_32KHZ;
		break;
	case 44100:
		prtd->dp_config.audio_fs = FS_44KHZ;
		break;
	case 48000:
		prtd->dp_config.audio_fs = FS_48KHZ;
		break;
	case 88200:
		prtd->dp_config.audio_fs = FS_88KHZ;
		break;
	case 96000:
		prtd->dp_config.audio_fs = FS_96KHZ;
		break;
	case 176400:
		prtd->dp_config.audio_fs = FS_176KHZ;
		break;
	case 192000:
		prtd->dp_config.audio_fs = FS_192KHZ;
		break;
	default:
		pr_debug("Not supported sample rate: %u\n", runtime->rate);
		return -EINVAL;
	}

	switch (runtime->sample_bits) {
	case 16:
		prtd->dp_config.audio_bit = AUDIO_16_BIT;
		break;
	case 20:
		prtd->dp_config.audio_bit = AUDIO_20_BIT;
		break;
	case 24:
		prtd->dp_config.audio_bit = AUDIO_24_BIT;
		break;
	default:
		pr_debug("Not supported sample bits: %u\n", runtime->sample_bits);
		return -EINVAL;
	}

	prtd->dp_config.num_audio_ch = runtime->channels;
	prtd->dp_config.audio_word_length = burst_len - 1;

	if (snd_pcm_format_physical_width(params_format(params)) == 32)
		prtd->dp_config.audio_packed_mode = NORMAL_MODE;
	else
		prtd->dp_config.audio_packed_mode = PACKED_MODE2;

	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = dp_dma_devs[dai_id];
	buf->private_data = NULL;
	buf->addr = (dma_addr_t)((unsigned long)dp_ado_rmem->base + (SZ_1M * dai_id));
	buf->area = (unsigned char *)((unsigned long)dp_ado_reserved_mem + (SZ_1M * dai_id));

	snd_pcm_set_runtime_buffer(substream, buf);

	runtime->dma_bytes = totbytes;

	spin_lock_irq(&prtd->lock);
	prtd->dma_loaded = 0;
	prtd->dma_period = params_period_bytes(params);
	prtd->dma_start = runtime->dma_addr;
	prtd->dma_pos = prtd->dma_start;
	prtd->dma_end = prtd->dma_start + totbytes;
	while (prtd->dma_period != 0 && (totbytes / prtd->dma_period) < PERIOD_MIN)
		prtd->dma_period >>= 1;
	spin_unlock_irq(&prtd->lock);
	dev_info(dev, "ADMA:%c:DmaAddr=@%pad Total=%d PrdSz=%d(%d) #Prds=%d dma_area=0x%pad\n",
		(substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? 'P' : 'C',
		&prtd->dma_start, (u32)runtime->dma_bytes,
		params_period_bytes(params),(u32) prtd->dma_period,
		params_periods(params), runtime->dma_area);
	return 0;
}

static int dma_hw_free(struct snd_soc_component *component,
		struct snd_pcm_substream *substream)
{
	struct runtime_data *prtd = substream->runtime->private_data;

	snd_pcm_set_runtime_buffer(substream, NULL);

	if (prtd->params) {
		prtd->params->ops->flush(prtd->params->ch);
		prtd->params->ops->release(prtd->params->ch,
					prtd->params->client);
		kfree(prtd->params);
		prtd->params = NULL;
	}

	return 0;
}

static int dma_prepare(struct snd_soc_component *component,
		struct snd_pcm_substream *substream)
{
	struct runtime_data *prtd = substream->runtime->private_data;
	int ret = 0;

	/* return if this is a bufferless transfer e.g.
	 * codec <--> BT codec or GSM modem -- lg FIXME */
	if (!prtd->params)
		return 0;

	/* flush the DMA channel */
	prtd->params->ops->flush(prtd->params->ch);
	prtd->dma_loaded = 0;
	prtd->dma_pos = prtd->dma_start;
	prtd->irq_pos = prtd->dma_start;
	prtd->irq_cnt = 0;

	/* enqueue dma buffers */
	dma_enqueue(substream);

	return ret;
}

static int dma_trigger(struct snd_soc_component *component,
		struct snd_pcm_substream *substream, int cmd)
{
	struct runtime_data *prtd = substream->runtime->private_data;
	struct device *dev = component->dev;
	int ret = 0;

	spin_lock(&prtd->lock);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		prtd->state |= ST_RUNNING;
		dev_info(dev, "Start DP DMA request\n");
#ifdef DP_DMA_DEBUG
		dev_info(dev, "Start DP DMA request initial status = 0x%08x\n",
			readl(dp_debug_sfr + 0x580C));
#endif
		prtd->dp_config.audio_state = DP_AUDIO_START;
		dp_audio_config(&prtd->dp_config);
#ifdef DP_DMA_DEBUG
		dev_info(dev, "Start DP DMA request Low status = 0x%08x\n",
			readl(dp_debug_sfr + 0x580C));
#endif
		prtd->params->ops->trigger(prtd->params->ch);
#ifdef DP_DMA_DEBUG
		dev_info(dev, "Start DP DMA request DMA On status = 0x%08x\n",
			readl(dp_debug_sfr + 0x580C));
#endif
		prtd->dp_config.audio_state = DP_AUDIO_REQ_BUF_READ;
		dp_audio_config(&prtd->dp_config);
#ifdef DP_DMA_DEBUG
		dev_info(dev, "Start DP DMA request DP Audio En status = 0x%08x\n",
			readl(dp_debug_sfr + 0x580C));
#endif
		break;

	case SNDRV_PCM_TRIGGER_STOP:
		prtd->state &= ~ST_RUNNING;
		prtd->dp_config.audio_state = DP_AUDIO_WAIT_BUF_FULL;
		dev_info(dev, "Stop DP DMA request\n");
#ifdef DP_DMA_DEBUG
		dev_info(dev, "Stop DP DMA request WAIT BUF FULL status = 0x%08x\n",
			readl(dp_debug_sfr + 0x580C));
#endif
		dp_audio_config(&prtd->dp_config);
#ifdef DP_DMA_DEBUG
		dev_info(dev, "Stop DP DMA request DMA Off status = 0x%08x\n",
			readl(dp_debug_sfr + 0x580C));
#endif
		prtd->params->ops->stop(prtd->params->ch);
#ifdef DP_DMA_DEBUG
		dev_info(dev, "Stop DP DMA request DP Audio Dis status = 0x%08x\n",
			readl(dp_debug_sfr + 0x580C));
#endif
		prtd->dp_config.audio_state = DP_AUDIO_DISABLE;
		dp_audio_config(&prtd->dp_config);
#ifdef DP_DMA_DEBUG
		dev_info(dev, "Stop DP DMA request End status = 0x%08x\n",
			readl(dp_debug_sfr + 0x580C));
#endif
		break;

	default:
		ret = -EINVAL;
		break;
	}

	spin_unlock(&prtd->lock);

	return ret;
}

static snd_pcm_uframes_t dma_pointer(struct snd_soc_component *component,
		struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct runtime_data *prtd = runtime->private_data;
	struct device *dev = component->dev;
	unsigned long res;

	res = prtd->dma_pos - prtd->dma_start;

	dev_dbg(dev, "Pointer offset: %lu\n", res);

	/*
	 * We seem to be getting the odd error from the pcm library due
	 * to out-of-bounds pointers. This is maybe due to the dma engine
	 * not having loaded the new values for the channel before being
	 * called... (todo - fix )
	 */

	if (res >= snd_pcm_lib_buffer_bytes(substream)) {
		if (res == snd_pcm_lib_buffer_bytes(substream))
			res = 0;
	}

	return bytes_to_frames(substream->runtime, res);
}

static int dma_open(struct snd_soc_component *component,
		struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct runtime_data *prtd;
	struct dp_audio_pdata *pdata;
	struct device *dev = component->dev;

	if (!dp_ado_extcon_is_avail())
		return -EIO;

	prtd = kzalloc(sizeof(struct runtime_data), GFP_KERNEL);
	if (prtd == NULL)
		return -ENOMEM;

	spin_lock_init(&prtd->lock);

	memcpy(&prtd->hw, &dma_hardware, sizeof(struct snd_pcm_hardware));

	snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);
	runtime->private_data = prtd;
	snd_soc_set_runtime_hwparams(substream, &prtd->hw);

	pdata = snd_soc_component_get_drvdata(component);
	exynos_update_ip_idle_status(pdata->idle_ip_index, 0);

	prtd->dp_config.audio_state = DP_AUDIO_ENABLE;
	dp_audio_config(&prtd->dp_config);

	pdata->prtd = prtd;
	dev_dbg(dev, "%s: prtd = %pK\n", __func__, prtd);

	return 0;
}

static int dma_close(struct snd_soc_component *component,
		struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct runtime_data *prtd = runtime->private_data;
	struct dp_audio_pdata *pdata;
	struct device *dev = component->dev;

	if (!prtd) {
		dev_dbg(dev, "dma_close called with prtd == NULL\n");
		return 0;
	}

	dev_dbg(dev, "%s: prtd = %pK, irq_cnt %u\n",
			__func__, prtd, prtd->irq_cnt);

	prtd->dp_config.audio_state = DP_AUDIO_DISABLE;
	dp_audio_config(&prtd->dp_config);

	pdata = snd_soc_component_get_drvdata(component);
	pdata->prtd = NULL;
	exynos_update_ip_idle_status(pdata->idle_ip_index, 1);

#ifdef DP_DMA_DEBUG
	if (prtd->irq_cnt == 0) {
		pr_info("=== DisplayPort SFR DUMP ===\n");
		print_hex_dump(KERN_INFO, "", DUMP_PREFIX_ADDRESS, 32, 4,
				dp_debug_sfr, 0x30, false);
		print_hex_dump(KERN_INFO, "", DUMP_PREFIX_ADDRESS, 32, 4,
				dp_debug_sfr + 0x100, 0x10, false);
		print_hex_dump(KERN_INFO, "", DUMP_PREFIX_ADDRESS, 32, 4,
				dp_debug_sfr + 0x200, 0x8, false);
		print_hex_dump(KERN_INFO, "", DUMP_PREFIX_ADDRESS, 32, 4,
				dp_debug_sfr + 0x1000, 0x204, false);
		print_hex_dump(KERN_INFO, "", DUMP_PREFIX_ADDRESS, 32, 4,
				dp_debug_sfr + 0x2000, 0x68, false);
		print_hex_dump(KERN_INFO, "", DUMP_PREFIX_ADDRESS, 32, 4,
				dp_debug_sfr + 0x3000, 0x24, false);
		print_hex_dump(KERN_INFO, "", DUMP_PREFIX_ADDRESS, 32, 4,
				dp_debug_sfr + 0x3100, 0x14, false);
		print_hex_dump(KERN_INFO, "", DUMP_PREFIX_ADDRESS, 32, 4,
				dp_debug_sfr + 0x4000, 0x68, false);
		print_hex_dump(KERN_INFO, "", DUMP_PREFIX_ADDRESS, 32, 4,
				dp_debug_sfr + 0x4400, 0x94, false);
		print_hex_dump(KERN_INFO, "", DUMP_PREFIX_ADDRESS, 32, 4,
				dp_debug_sfr + 0x4500, 0x8, false);
		print_hex_dump(KERN_INFO, "", DUMP_PREFIX_ADDRESS, 32, 4,
				dp_debug_sfr + 0x5000, 0x108, false);
		print_hex_dump(KERN_INFO, "", DUMP_PREFIX_ADDRESS, 32, 4,
				dp_debug_sfr + 0x5400, 0x70, false);
		print_hex_dump(KERN_INFO, "", DUMP_PREFIX_ADDRESS, 32, 4,
				dp_debug_sfr + 0x5800, 0xF0, false);
		print_hex_dump(KERN_INFO, "", DUMP_PREFIX_ADDRESS, 32, 4,
				dp_debug_sfr + 0x5900, 0x4, false);
		print_hex_dump(KERN_INFO, "", DUMP_PREFIX_ADDRESS, 32, 4,
				dp_debug_sfr + 0x5C00, 0xC0, false);
		print_hex_dump(KERN_INFO, "", DUMP_PREFIX_ADDRESS, 32, 4,
				dp_debug_sfr + 0x5d00, 0x84, false);
		print_hex_dump(KERN_INFO, "", DUMP_PREFIX_ADDRESS, 32, 4,
				dp_debug_sfr + 0x6000, 0x108, false);
		print_hex_dump(KERN_INFO, "", DUMP_PREFIX_ADDRESS, 32, 4,
				dp_debug_sfr + 0x6400, 0x70, false);
		print_hex_dump(KERN_INFO, "", DUMP_PREFIX_ADDRESS, 32, 4,
				dp_debug_sfr + 0x6800, 0xF0, false);
		print_hex_dump(KERN_INFO, "", DUMP_PREFIX_ADDRESS, 32, 4,
				dp_debug_sfr + 0x6900, 0x4, false);
		print_hex_dump(KERN_INFO, "", DUMP_PREFIX_ADDRESS, 32, 4,
				dp_debug_sfr + 0x6C00, 0xC0, false);
		print_hex_dump(KERN_INFO, "", DUMP_PREFIX_ADDRESS, 32, 4,
				dp_debug_sfr + 0x6D00, 0x84, false);
	}
#endif

	kfree(prtd);

	return 0;
}

static int dma_mmap(struct snd_soc_component *component,
		struct snd_pcm_substream *substream, struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	dma_addr_t dma_pa = runtime->dma_addr;
#if IS_ENABLED(CONFIG_SND_SAMSUNG_IOMMU)
	struct dma_iova *di;
#endif

#if IS_ENABLED(CONFIG_SND_SAMSUNG_IOMMU)
	list_for_each_entry(di, &iova_list, node) {
		if (di->iova == runtime->dma_addr)
			dma_pa = di->pa;
	}
#endif
	return dma_mmap_wc(substream->pcm->card->dev, vma,
				     runtime->dma_area, dma_pa,
				     runtime->dma_bytes);
}

static void dma_free_dma_buffers(struct snd_soc_component *component, struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

	for (stream = 0; stream < 2; stream++) {
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;

		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;

		dma_free_coherent(pcm->card->dev, buf->bytes,
					buf->area, buf->addr);

		buf->area = NULL;
		buf->bytes = 0;
	}
}

#if IS_ENABLED(CONFIG_ZONE_DMA)
static u64 dma_mask = DMA_BIT_MASK(32);
#else
static u64 dma_mask = DMA_BIT_MASK(36);
#endif

static int dma_new(struct snd_soc_component *component, struct snd_soc_pcm_runtime *rtd)
{
	struct snd_card *card = rtd->card->snd_card;
	int ret = 0;

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &dma_mask;
	if (!card->dev->coherent_dma_mask)
#if IS_ENABLED(CONFIG_ZONE_DMA)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(32);
#else
		card->dev->coherent_dma_mask = DMA_BIT_MASK(36);
#endif
	dma_set_mask(card->dev, DMA_BIT_MASK(36));

	return ret;
}

static int dp_ado_hw_params_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_soc_kcontrol_component(kcontrol);
	struct device *dev = cmpnt->dev;
	struct dp_audio_pdata *pdata = dev_get_drvdata(dev);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	enum audio_param param = mc->reg;
	long value;

	switch (param) {
	case DPADO_RATE:
		value = pdata->rate;
		break;
	case DPADO_WIDTH:
		value = pdata->width;
		break;
	case DPADO_CHANNEL:
		value = pdata->channel;
		break;
	default:
		return -EINVAL;
	}

	dev_dbg(dev, "param(%d), value(%ld)\n", param, value);
	ucontrol->value.integer.value[0] = value;

	return 0;
}

static int dp_ado_hw_params_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	/* Nothing to do */
	return 0;
}

static const struct snd_kcontrol_new dp_ado_controls[] = {
	SOC_SINGLE_EXT("Rate", DPADO_RATE, 0, 384000, 0,
			dp_ado_hw_params_get, dp_ado_hw_params_put),
	SOC_SINGLE_EXT("Width", DPADO_WIDTH, 0, 24, 0,
			dp_ado_hw_params_get, dp_ado_hw_params_put),
	SOC_SINGLE_EXT("Channel", DPADO_CHANNEL, 0, 8, 0,
			dp_ado_hw_params_get, dp_ado_hw_params_put),
};

static int dma_ioctl(struct snd_soc_component *component,
		struct snd_pcm_substream *substream, unsigned int cmd, void *arg)
{
	int ret;

	ret = snd_pcm_lib_ioctl(substream, cmd, arg);

	return ret;
}

static const struct snd_soc_component_driver dp_dma_cmpnt_drv = {
	.controls		= dp_ado_controls,
	.num_controls		= ARRAY_SIZE(dp_ado_controls),
	.pcm_construct		= dma_new,
	.pcm_destruct		= dma_free_dma_buffers,
	.open			= dma_open,
	.close			= dma_close,
	.ioctl			= dma_ioctl,
	.hw_params		= dma_hw_params,
	.hw_free		= dma_hw_free,
	.prepare		= dma_prepare,
	.trigger		= dma_trigger,
	.pointer		= dma_pointer,
	.mmap			= dma_mmap,
};

static const struct snd_soc_dai_driver dp_dma_dai_drv = {
	.name = "dp0-dma",
	.id = 0,
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 8,
		.rates = DPAUDIO_SAMPLING_RATES,
		.rate_min = 8000,
		.rate_max = 384000,
		.formats = DPAUDIO_SAMPLE_FORMATS,
	},
};

static int samsung_dp_dma_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct dp_audio_pdata *pdata;
	int result;

	dev_dbg(dev, "%s \n", __func__);
	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	platform_set_drvdata(pdev, pdata);

	pdata->idle_ip_index = exynos_get_idle_ip_index(dev_name(&pdev->dev));
	exynos_update_ip_idle_status(pdata->idle_ip_index, 1);

	result = of_property_read_u32_index(np, "id", 0, &pdata->id);
	if (result < 0) {
		dev_err(dev, "id property reading fail\n");
		return result;
	}
	if (pdata->id >= ARRAY_SIZE(dp_dma_devs))
		return -EFAULT;

	result = of_property_read_u32_index(np, "samsung,fifo_addr", 0, &pdata->fifo_addr);
	if (result < 0) {
		dev_err(dev, "fifo_addr property reading fail\n");
		return result;
	}

	result = of_property_read_string(np, "dma-names", (const char**)&pdata->ch_name);
	if (result < 0) {
		dev_err(dev, "dma-names property reading fail\n");
		return result;
	}

	result = devm_snd_soc_register_component(dev, &dp_dma_cmpnt_drv,
			(struct snd_soc_dai_driver *)&dp_dma_dai_drv, 1);
	if (result < 0)
		return result;

	if (dp_dma_count <= pdata->id)
		dp_dma_count = pdata->id + 1;
	dp_dma_devs[pdata->id] = dev;

	dev_dbg(dev, "Probed successfully\n");

	return 0;
}

static int samsung_dp_dma_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id samsung_dp_dma_match[] = {
	{
		.compatible = "samsung,dp-dma",
	},
	{},
};
MODULE_DEVICE_TABLE(of, samsung_dp_dma_match);

static struct platform_driver samsung_dp_dma_driver = {
	.probe  = samsung_dp_dma_probe,
	.remove = samsung_dp_dma_remove,
	.driver = {
		.name = "samsung-dp-dma",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(samsung_dp_dma_match),
	},
};

#if IS_ENABLED(CONFIG_EXYNOS_MIPI_DISPLAYPORT) || \
	IS_ENABLED(CONFIG_EXYNOS_DISPLAYPORT) || \
	IS_ENABLED(CONFIG_DRM_SAMSUNG_DP)
static int dp_ado_notifier(struct notifier_block *nb,
				      unsigned long event, void *data)
{
	dp_ado_switch_set_state((int)event);
	return 0;
}

static struct notifier_block dp_ado_nb = {
	.notifier_call = dp_ado_notifier,
};
#endif

static int dp_ado_component_probe(struct snd_soc_component *component)
{
	return 0;
}

static const struct snd_soc_component_driver dp_ado_cmpnt_drv = {
	.probe = dp_ado_component_probe,
};

static struct snd_soc_dai_driver dp_ado_dai_drv[] = {
	{
		.name = "dp0-ado",
		.id = 0,
		.playback = {
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = 8,
			.rates = DPAUDIO_SAMPLING_RATES,
			.rate_min = 8000,
			.rate_max = 384000,
			.formats = DPAUDIO_SAMPLE_FORMATS,
		},
	},
	{
		.name = "dp1-ado",
		.id = 1,
		.playback = {
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = 8,
			.rates = DPAUDIO_SAMPLING_RATES,
			.rate_min = 8000,
			.rate_max = 384000,
			.formats = DPAUDIO_SAMPLE_FORMATS,
		},
	},
};


static int samsung_display_adma_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *np_tmp;
	u32 dp_tx_addr = 0;
	int ret;
#if IS_ENABLED(CONFIG_SWITCH)
	dp_ado_switch.name = "hdmi_audio";
	ret = switch_dev_register(&dp_ado_switch);
	if (ret) {
		dev_err(dev, "Failed to register dp audio switch\n");
		return -EINVAL;
	}
#elif IS_ENABLED(CONFIG_EXTCON)
	dp_ado_extcon = devm_extcon_dev_allocate(dev, extcon_id);
	if (IS_ERR(dp_ado_extcon)) {
		dev_err(dev, "Failed to allocate dp audio extcon\n");
		return -EINVAL;
	}
	dp_ado_extcon->dev.init_name = "hdmi_audio";
	ret = devm_extcon_dev_register(dev, dp_ado_extcon);
	if (ret) {
		dev_err(dev, "Failed to register dp audio extcon\n");
		return -EINVAL;
	}
#endif

	ret = snd_soc_register_component(&pdev->dev, &dp_ado_cmpnt_drv,
						dp_ado_dai_drv, ARRAY_SIZE(dp_ado_dai_drv));
	if (ret < 0) {
		dev_err(dev, "Failed to register ASoC component\n");
#if IS_ENABLED(CONFIG_SWITCH)
		switch_dev_unregister(&dp_ado_switch);
#endif
		return -EINVAL;
	}

	if (!dp_ado_rmem) {
		np_tmp = of_parse_phandle(np, "memory-region", 0);
		if (!np_tmp) {
#if IS_ENABLED(CONFIG_SWITCH)
			switch_dev_unregister(&dp_ado_switch);
#endif
			return -ENODEV;
		}
		dp_ado_rmem = of_reserved_mem_lookup(np_tmp);
	}

	if (!dp_ado_rmem) {
		dev_err(dev, "dp audio no memory\n");
#if IS_ENABLED(CONFIG_SWITCH)
		switch_dev_unregister(&dp_ado_switch);
#endif
		return -ENOMEM;
	}

#if IS_ENABLED(CONFIG_EXYNOS_MIPI_DISPLAYPORT) || \
	IS_ENABLED(CONFIG_EXYNOS_DISPLAYPORT) || \
	IS_ENABLED(CONFIG_DRM_SAMSUNG_DP)
	blocking_notifier_chain_register(&dp_ado_notifier_head, &dp_ado_nb);
#endif

	dp_ado_reserved_mem = dp_ado_rmem_vmap(dp_ado_rmem);
	pr_info("dp_ado_reserved_mem = %pK\n", dp_ado_reserved_mem);

	if (of_property_read_u32_index(np, "samsung,dp_tx_addr", 0, &dp_tx_addr) < 0)
		dev_warn(dev, "dp_tx_addr property reading fail\n");
	else
		dp_debug_sfr = devm_ioremap(dev, dp_tx_addr, 0x7000);

	platform_driver_register(&samsung_dp_dma_driver);
	return of_platform_populate(np, NULL, NULL, dev);
}

static int samsung_display_adma_remove(struct platform_device *pdev)
{
	vunmap(dp_ado_reserved_mem);
	dp_ado_reserved_mem = NULL;
	snd_soc_unregister_component(&pdev->dev);
	return 0;
}

static const struct of_device_id samsung_display_adma_match[] = {
	{
		.compatible = "samsung,displayport-adma",
	},
	{},
};
MODULE_DEVICE_TABLE(of, samsung_display_adma_match);

static struct platform_driver samsung_display_adma_driver = {
	.probe	= samsung_display_adma_probe,
	.remove	= samsung_display_adma_remove,
	.driver = {
		.name = "samsung-displayport-adma",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(samsung_display_adma_match),
	},
};

module_platform_driver(samsung_display_adma_driver);

MODULE_AUTHOR("Ben Dooks, <ben@simtec.co.uk>");
MODULE_DESCRIPTION("Samsung Display Port Audio DMA Driver");
MODULE_ALIAS("platform:samsung-display-adma");
MODULE_LICENSE("GPL");
