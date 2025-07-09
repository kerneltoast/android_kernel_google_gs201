/*
 * Copyright (c) 2018 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Copyright 2022 Google LLC
 *   pcsaszar@ - Migrated the driver files from sysfs to debugfs
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of_platform.h>
#include <linux/clk.h>
#include <linux/suspend.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/of_reserved_mem.h>
#include <linux/sched/clock.h>
#include <linux/debugfs.h>

#include <asm/tlbflush.h>
#include <asm/cacheflush.h>

#include <soc/google/debug-snapshot.h>
#include <soc/google/exynos-adv-tracer.h>
#include <soc/google/exynos-bcm_dbg.h>
#include <soc/google/exynos-bcm_dbg-dt.h>
#include <soc/google/exynos-bcm_dbg-dump.h>
#include <soc/google/exynos-pd.h>
#include <soc/google/cal-if.h>
#include <soc/google/exynos-itmon.h>

#define BCM_FILE_ENTRY_RO(name)		{ #name, 0440, show_ ## name, NULL }
#define BCM_FILE_ENTRY_WR(name)		{ #name, 0640, show_ ## name, store_ ## name }

struct bcm_file_entry {
	char *fname;
	umode_t mode;
	ssize_t (*read) (struct file *, char __user *, size_t, loff_t *);
	ssize_t (*write) (struct file *, const char __user *, size_t, loff_t *);
};

static struct file_operations *bcm_dbg_file_fops;

static struct exynos_bcm_dbg_data *bcm_dbg_data = NULL;
static bool pd_sync_init;

#if IS_ENABLED(CONFIG_EXYNOS_ADV_TRACER)
static enum exynos_bcm_err_code exynos_bcm_dbg_ipc_err_handle(unsigned int cmd)
{
	enum exynos_bcm_err_code err_code;

	err_code = BCM_CMD_GET(cmd, BCM_ERR_MASK, BCM_ERR_SHIFT);
	if (err_code)
		BCM_ERR("%s: BCM IPC error return(%u)\n", __func__, err_code);

	return err_code;
}
#endif

static int exynos_bcm_ip_validate(unsigned int ip_range, unsigned int ip_index,
					unsigned int bcm_ip_nr)
{
	if (ip_range >= BCM_RANGE_MAX) {
		BCM_ERR("%s: Invalid ip range(%u)\n", __func__, ip_range);
		BCM_ERR("%s: BCM_EACH(%d), BCM_ALL(%d)\n",
				__func__, BCM_EACH, BCM_ALL);
		return -EINVAL;
	}

	if (ip_index >= bcm_ip_nr) {
		BCM_ERR("%s: Invalid ip index(%u), ip_max_nr(%u)\n",
			__func__, ip_index, bcm_ip_nr - 1);
		return -EINVAL;
	}

	return 0;
}

static int exynos_bcm_is_running(unsigned int run_state)
{
	if (run_state == BCM_RUN) {
		BCM_ERR("%s: do not set when bcm is running(%u)\n",
				__func__, run_state);
		return -EBUSY;
	}

	return 0;
}

static int __exynos_bcm_dbg_ipc_send_data(enum exynos_bcm_dbg_ipc_type ipc_type,
				struct exynos_bcm_dbg_data *data,
				unsigned int *cmd)
{
#if IS_ENABLED(CONFIG_EXYNOS_ADV_TRACER)
	int i, ret = 0;
	struct adv_tracer_ipc_cmd config;
	enum exynos_bcm_err_code ipc_err;
	unsigned int *bcm_cmd;

	if ((ipc_type < IPC_BCM_DBG_EVENT) ||
		(ipc_type >= IPC_BCM_DBG_MAX)) {
		BCM_ERR("%s: Invalid IPC Type: %d\n", __func__, ipc_type);
		ret = -EINVAL;
		return ret;
	}

	config.cmd_raw.cmd = BCM_CMD_SET(ipc_type, BCM_CMD_ID_MASK, BCM_CMD_ID_SHIFT);
	bcm_cmd = cmd;
	memcpy(&config.buffer[1], bcm_cmd, sizeof(unsigned int) * CMD_DATA_MAX);

	ret = adv_tracer_ipc_send_data_polling(data->ipc_ch_num, &config);
	if (ret) {
		BCM_ERR("%s: Failed to send IPC(%d:%u) data to dbgc (rv=%d)\n",
			__func__, ipc_type, data->ipc_ch_num, ret);
		return ret;
	}

	for (i = 0; i < data->ipc_size; i++)
		BCM_DBG("%s: received data[%d]: 0x%08x\n",
				__func__, i, config.buffer[i]);

	memcpy(bcm_cmd, &config.buffer[1], sizeof(unsigned int) * CMD_DATA_MAX);

	ipc_err = exynos_bcm_dbg_ipc_err_handle(config.cmd_raw.cmd);
	if (ipc_err) {
		ret = -EBADMSG;
		return ret;
	}
#endif

	return 0;
}

int exynos_bcm_dbg_ipc_send_data(enum exynos_bcm_dbg_ipc_type ipc_type,
				struct exynos_bcm_dbg_data *data,
				unsigned int *cmd)
{
	int ret;
	unsigned long flags;

	if (!data) {
		BCM_ERR("%s: data is not ready!\n", __func__);
		return -ENODEV;
	}

	spin_lock_irqsave(&data->lock, flags);
	ret = __exynos_bcm_dbg_ipc_send_data(ipc_type, data, cmd);
	spin_unlock_irqrestore(&data->lock, flags);

	return ret;
}
EXPORT_SYMBOL(exynos_bcm_dbg_ipc_send_data);

#if IS_ENABLED(CONFIG_EXYNOS_ADV_TRACER)
static void adv_tracer_bcm_dbg_handler(struct adv_tracer_ipc_cmd *cmd, unsigned int len)
{
	return;
}

static int exynos_bcm_dbg_ipc_channel_request(struct exynos_bcm_dbg_data *data)
{
	int ret = 0;

	ret = adv_tracer_ipc_request_channel(data->ipc_node,
				adv_tracer_bcm_dbg_handler,
				&data->ipc_ch_num, &data->ipc_size);
	if (ret) {
		BCM_ERR("%s: adv tracer request channel is failed\n", __func__);
		return ret;
	}

	BCM_INFO("ipc channel info: ch_num(%u), size(%u)\n",
				data->ipc_ch_num, data->ipc_size);

	return ret;
}

static void exynos_bcm_dbg_ipc_channel_release(struct exynos_bcm_dbg_data *data)
{
	adv_tracer_ipc_release_channel(data->ipc_ch_num);
}
#else
static inline
int exynos_bcm_dbg_ipc_channel_request(struct exynos_bcm_dbg_data *data)
{
	return 0;
}

static inline
void exynos_bcm_dbg_ipc_channel_release(struct exynos_bcm_dbg_data *data)
{
	return;
}
#endif

static int exynos_bcm_dbg_early_pd_sync(unsigned int cal_pdid, bool on)
{
	unsigned int cmd[4] = {0, };
	unsigned long flags;
	struct exynos_bcm_pd_info *bcm_pd_info = NULL;
	int i, ret = 0;

	spin_lock_irqsave(&bcm_dbg_data->lock, flags);

	for (i = 0; i < bcm_dbg_data->pd_size; i++) {
		if (bcm_dbg_data->pd_info[i]->cal_pdid == cal_pdid) {
			bcm_pd_info = bcm_dbg_data->pd_info[i];
			break;
		}
	}

	if (!bcm_pd_info) {
		ret = -EINVAL;
		goto out;
	}

	if (on ^ bcm_pd_info->on) {
		bcm_pd_info->on = on;
		/* Generate IPC command for PD sync */
		cmd[0] |= BCM_CMD_SET(bcm_pd_info->pd_index, BCM_PD_INFO_MASK,
					BCM_PD_INFO_SHIFT);
		cmd[0] |= BCM_CMD_SET((unsigned int)on, BCM_ONE_BIT_MASK,
					BCM_PD_ON_SHIFT);
		cmd[1] = 0;
		cmd[2] = 0;
		cmd[3] = 0;

		/* send command for PD sync */
		ret = __exynos_bcm_dbg_ipc_send_data(IPC_BCM_DBG_PD,
							bcm_dbg_data, cmd);
		if (ret) {
			BCM_ERR("%s: Failed send data for pd sync\n", __func__);
			goto out;
		}
	}

out:
	spin_unlock_irqrestore(&bcm_dbg_data->lock, flags);

	return ret;
}
int exynos_bcm_dbg_pd_sync(unsigned int cal_pdid, bool on)
{
	unsigned int cmd[4] = {0, };
	unsigned long flags;
	struct exynos_bcm_pd_info *bcm_pd_info = NULL;
	int i, ret = 0;

	if (!bcm_dbg_data || !pd_sync_init) {
		BCM_DBG("%s: do not pd_sync_init(%s)\n",
			__func__, pd_sync_init ? "true" : "false");
		return 0;
	}

	spin_lock_irqsave(&bcm_dbg_data->lock, flags);

	for (i = 0; i < bcm_dbg_data->pd_size; i++) {
		if (bcm_dbg_data->pd_info[i]->cal_pdid == cal_pdid) {
			bcm_pd_info = bcm_dbg_data->pd_info[i];
			break;
		}
	}

	if (!bcm_pd_info) {
		ret = -EINVAL;
		goto out;
	}

	if (on ^ bcm_pd_info->on) {
		bcm_pd_info->on = on;
		/* Generate IPC command for PD sync */
		cmd[0] |= BCM_CMD_SET(bcm_pd_info->pd_index, BCM_PD_INFO_MASK,
					BCM_PD_INFO_SHIFT);
		cmd[0] |= BCM_CMD_SET((unsigned int)on, BCM_ONE_BIT_MASK,
					BCM_PD_ON_SHIFT);
		cmd[1] = 0;
		cmd[2] = 0;
		cmd[3] = 0;

		/* send command for PD sync */
		ret = __exynos_bcm_dbg_ipc_send_data(IPC_BCM_DBG_PD,
							bcm_dbg_data, cmd);
		if (ret) {
			BCM_ERR("%s: Failed send data for pd sync\n", __func__);
			goto out;
		}
	}

out:
	spin_unlock_irqrestore(&bcm_dbg_data->lock, flags);

	return ret;
}
EXPORT_SYMBOL(exynos_bcm_dbg_pd_sync);

static int exynos_bcm_dbg_pd_sync_init(struct exynos_bcm_dbg_data *data)
{
	struct exynos_pm_domain *exynos_pd;
	unsigned int pd_index, pd_size;
	int ret = 0;

	if (pd_sync_init) {
		BCM_ERR("%s: already pd_sync_init(%s)\n",
			__func__, pd_sync_init ? "true" : "false");
		return -EINVAL;
	}

	pd_size = data->pd_size;
	for (pd_index = 0; pd_index < pd_size; pd_index++) {
		exynos_pd = NULL;
		data->pd_info[pd_index]->on = false;
		exynos_pd = exynos_pd_lookup_name(data->pd_info[pd_index]->pd_name);
		if (exynos_pd) {
			mutex_lock(&exynos_pd->access_lock);
			exynos_pd->bcm = data->pd_info[pd_index];
			data->pd_info[pd_index]->cal_pdid = exynos_pd->cal_pdid;
			if (cal_pd_status(exynos_pd->cal_pdid)) {
				ret = exynos_bcm_dbg_early_pd_sync(exynos_pd->cal_pdid, true);
				if (ret) {
					mutex_unlock(&exynos_pd->access_lock);
					return ret;
				}
			}
			mutex_unlock(&exynos_pd->access_lock);
		} else {
			ret = exynos_bcm_dbg_early_pd_sync(data->pd_info[pd_index]->cal_pdid, true);
			if (ret)
				return ret;
		}
	}

	set_exynos_cal_pd_bcm_sync(exynos_bcm_dbg_pd_sync);
	pd_sync_init = true;

	return ret;
}

static int exynos_bcm_dbg_pd_sync_exit(struct exynos_bcm_dbg_data *data)
{
	struct exynos_pm_domain *exynos_pd;
	unsigned int pd_index, pd_size;
	int ret = 0;

	if (!pd_sync_init) {
		BCM_ERR("%s: already pd_sync_exit(%s)\n",
			__func__, pd_sync_init ? "true" : "false");
		return -EINVAL;
	}

	pd_size = data->pd_size;
	for (pd_index = 0; pd_index < pd_size; pd_index++) {
		exynos_pd = exynos_pd_lookup_name(data->pd_info[pd_index]->pd_name);
		if (exynos_pd) {
			mutex_lock(&exynos_pd->access_lock);
			exynos_pd->bcm = NULL;
			ret = exynos_bcm_dbg_pd_sync(data->pd_info[pd_index]->cal_pdid, false);
			if (ret) {
				mutex_unlock(&exynos_pd->access_lock);
				return ret;
			}
			mutex_unlock(&exynos_pd->access_lock);
		} else {
			ret = exynos_bcm_dbg_pd_sync(data->pd_info[pd_index]->cal_pdid, false);
			if (ret)
				return ret;
		}
	}

	clear_exynos_cal_pd_bcm_sync();
	pd_sync_init = false;

	return ret;
}

void exynos_bcm_dbg_set_base_info(
				struct exynos_bcm_ipc_base_info *ipc_base_info,
				enum exynos_bcm_event_id event_id,
				enum exynos_bcm_event_dir direction,
				enum exynos_bcm_ip_range ip_range)
{
	ipc_base_info->event_id = event_id;
	ipc_base_info->ip_range = ip_range;
	ipc_base_info->direction = direction;
}
EXPORT_SYMBOL(exynos_bcm_dbg_set_base_info);

static void exynos_bcm_dbg_set_base_cmd(unsigned int *cmd,
				struct exynos_bcm_ipc_base_info *ipc_base_info)
{
	cmd[0] = 0;
	cmd[0] |= BCM_CMD_SET(ipc_base_info->event_id, BCM_EVT_ID_MASK,
					BCM_EVT_ID_SHIFT);
	cmd[0] |= BCM_CMD_SET(ipc_base_info->ip_range, BCM_ONE_BIT_MASK,
					BCM_IP_RANGE_SHIFT);
	cmd[0] |= BCM_CMD_SET(ipc_base_info->direction, BCM_ONE_BIT_MASK,
					BCM_EVT_DIR_SHIFT);
}

int exynos_bcm_dbg_event_ctrl(struct exynos_bcm_ipc_base_info *ipc_base_info,
					struct exynos_bcm_event *bcm_event,
					unsigned int bcm_ip_index,
					struct exynos_bcm_dbg_data *data)
{
	unsigned int cmd[4] = {0, 0, 0, 0};
	int i, ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&data->lock, flags);

	if (!ipc_base_info || !bcm_event) {
		BCM_ERR("%s: pointer is NULL\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	exynos_bcm_dbg_set_base_cmd(cmd, ipc_base_info);

	if (ipc_base_info->event_id != BCM_EVT_PRE_DEFINE &&
		ipc_base_info->event_id != BCM_EVT_EVENT) {
		BCM_ERR("%s: Invalid Event ID(%d)\n", __func__,
					ipc_base_info->event_id);
		ret = -EINVAL;
		goto out;
	}

	if (ipc_base_info->ip_range == BCM_EACH)
		cmd[0] |= BCM_CMD_SET(bcm_ip_index, BCM_IP_MASK, BCM_IP_SHIFT);

	if (ipc_base_info->direction == BCM_EVT_SET) {
		cmd[0] |= BCM_CMD_SET(bcm_event->index, BCM_EVT_PRE_DEFINE_MASK,
					BCM_EVT_PRE_DEFINE_SHIFT);

		for (i = 0; i < (BCM_EVT_EVENT_MAX / 2); i++) {
			cmd[1] |= BCM_CMD_SET(bcm_event->event[i], BCM_EVT_EVENT_MASK,
						BCM_EVT_EVENT_SHIFT(i));
			cmd[2] |= BCM_CMD_SET(bcm_event->event[i + 4], BCM_EVT_EVENT_MASK,
						BCM_EVT_EVENT_SHIFT(i + 4));
		}
	}

	/* send command for BCM Event */
	ret = __exynos_bcm_dbg_ipc_send_data(IPC_BCM_DBG_EVENT, data, cmd);
	if (ret) {
		BCM_ERR("%s: Failed send data\n", __func__);
		goto out;
	}

	if (ipc_base_info->direction == BCM_EVT_GET) {
		bcm_event->index = BCM_CMD_GET(cmd[0], BCM_EVT_PRE_DEFINE_MASK,
						BCM_EVT_PRE_DEFINE_SHIFT);

		for (i = 0; i < (BCM_EVT_EVENT_MAX / 2); i++) {
			bcm_event->event[i] = BCM_CMD_GET(cmd[1], BCM_EVT_EVENT_MASK,
							BCM_EVT_EVENT_SHIFT(i));
			bcm_event->event[i + 4] = BCM_CMD_GET(cmd[2], BCM_EVT_EVENT_MASK,
							BCM_EVT_EVENT_SHIFT(i + 4));
		}
	}

out:
	spin_unlock_irqrestore(&data->lock, flags);

	return ret;
}
EXPORT_SYMBOL(exynos_bcm_dbg_event_ctrl);

static int exynos_bcm_dbg_filter_id_ctrl(struct exynos_bcm_ipc_base_info *ipc_base_info,
					struct exynos_bcm_filter_id *filter_id,
					unsigned int bcm_ip_index,
					struct exynos_bcm_dbg_data *data)
{
	unsigned int cmd[4] = {0, 0, 0, 0};
	int i, ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&data->lock, flags);

	if (!ipc_base_info || !filter_id) {
		BCM_ERR("%s: pointer is NULL\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	exynos_bcm_dbg_set_base_cmd(cmd, ipc_base_info);

	if (ipc_base_info->event_id != BCM_EVT_EVENT_FLT_ID) {
		BCM_ERR("%s: Invalid Event ID(%d)\n", __func__,
					ipc_base_info->event_id);
		ret = -EINVAL;
		goto out;
	}

	if (ipc_base_info->ip_range == BCM_EACH)
		cmd[0] |= BCM_CMD_SET(bcm_ip_index, BCM_IP_MASK, BCM_IP_SHIFT);

	if (ipc_base_info->direction == BCM_EVT_SET) {
		/*
		 * check bcm running state
		 * When bcm is running, value can not set
		 */
		ret = exynos_bcm_is_running(data->bcm_run_state);
		if (ret)
			goto out;

		cmd[1] = filter_id->sm_id_mask;
		cmd[2] = filter_id->sm_id_value;
		for (i = 0; i < BCM_EVT_EVENT_MAX; i++)
			cmd[3] |= BCM_CMD_SET(filter_id->sm_id_active[i],
					BCM_ONE_BIT_MASK, BCM_EVT_FLT_ACT_SHIFT(i));
	}

	/* send command for BCM Filter ID */
	ret = __exynos_bcm_dbg_ipc_send_data(IPC_BCM_DBG_EVENT, data, cmd);
	if (ret) {
		BCM_ERR("%s: Failed send data\n", __func__);
		goto out;
	}

	if (ipc_base_info->direction == BCM_EVT_GET) {
		filter_id->sm_id_mask = cmd[1];
		filter_id->sm_id_value = cmd[2];
		for (i = 0; i < BCM_EVT_EVENT_MAX; i++)
			filter_id->sm_id_active[i] = BCM_CMD_GET(cmd[3],
							BCM_ONE_BIT_MASK,
							BCM_EVT_FLT_ACT_SHIFT(i));
	}

out:
	spin_unlock_irqrestore(&data->lock, flags);

	return ret;
}

static int exynos_bcm_dbg_filter_others_ctrl(
					struct exynos_bcm_ipc_base_info *ipc_base_info,
					struct exynos_bcm_filter_others *filter_others,
					unsigned int bcm_ip_index,
					struct exynos_bcm_dbg_data *data)
{
	unsigned int cmd[4] = {0, 0, 0, 0};
	int i, ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&data->lock, flags);

	if (!ipc_base_info || !filter_others) {
		BCM_ERR("%s: pointer is NULL\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	exynos_bcm_dbg_set_base_cmd(cmd, ipc_base_info);

	if (ipc_base_info->event_id != BCM_EVT_EVENT_FLT_OTHERS) {
		BCM_ERR("%s: Invalid Event ID(%d)\n", __func__,
					ipc_base_info->event_id);
		ret = -EINVAL;
		goto out;
	}

	if (ipc_base_info->ip_range == BCM_EACH)
		cmd[0] |= BCM_CMD_SET(bcm_ip_index, BCM_IP_MASK, BCM_IP_SHIFT);

	if (ipc_base_info->direction == BCM_EVT_SET) {
		/*
		 * check bcm running state
		 * When bcm is running, value can not set
		 */
		ret = exynos_bcm_is_running(data->bcm_run_state);
		if (ret)
			goto out;

		for (i = 0; i < BCM_EVT_FLT_OTHR_MAX; i++) {
			cmd[1] |= BCM_CMD_SET(filter_others->sm_other_type[i],
						BCM_EVT_FLT_OTHR_TYPE_MASK,
						BCM_EVT_FLT_OTHR_TYPE_SHIFT(i));
			cmd[1] |= BCM_CMD_SET(filter_others->sm_other_mask[i],
						BCM_EVT_FLT_OTHR_MASK_MASK,
						BCM_EVT_FLT_OTHR_MASK_SHIFT(i));
			cmd[1] |= BCM_CMD_SET(filter_others->sm_other_value[i],
						BCM_EVT_FLT_OTHR_VALUE_MASK,
						BCM_EVT_FLT_OTHR_VALUE_SHIFT(i));
		}

		for (i = 0; i < BCM_EVT_EVENT_MAX; i++)
			cmd[2] |= BCM_CMD_SET(filter_others->sm_other_active[i],
					BCM_ONE_BIT_MASK, BCM_EVT_FLT_ACT_SHIFT(i));
	}

	/* send command for BCM Filter Others */
	ret = __exynos_bcm_dbg_ipc_send_data(IPC_BCM_DBG_EVENT, data, cmd);
	if (ret) {
		BCM_ERR("%s: Failed send data\n", __func__);
		goto out;
	}

	if (ipc_base_info->direction == BCM_EVT_GET) {
		for (i = 0; i < BCM_EVT_FLT_OTHR_MAX; i++) {
			filter_others->sm_other_type[i] =
				BCM_CMD_GET(cmd[1], BCM_EVT_FLT_OTHR_TYPE_MASK,
						BCM_EVT_FLT_OTHR_TYPE_SHIFT(i));
			filter_others->sm_other_mask[i] =
				BCM_CMD_GET(cmd[1], BCM_EVT_FLT_OTHR_MASK_MASK,
						BCM_EVT_FLT_OTHR_MASK_SHIFT(i));
			filter_others->sm_other_value[i] =
				BCM_CMD_GET(cmd[1], BCM_EVT_FLT_OTHR_VALUE_MASK,
						BCM_EVT_FLT_OTHR_VALUE_SHIFT(i));
		}

		for (i = 0; i < BCM_EVT_EVENT_MAX; i++)
			filter_others->sm_other_active[i] =
				BCM_CMD_GET(cmd[2], BCM_ONE_BIT_MASK,
						BCM_EVT_FLT_ACT_SHIFT(i));
	}

out:
	spin_unlock_irqrestore(&data->lock, flags);

	return ret;
}

int exynos_bcm_dbg_sample_id_ctrl(struct exynos_bcm_ipc_base_info *ipc_base_info,
					struct exynos_bcm_sample_id *sample_id,
					unsigned int bcm_ip_index,
					struct exynos_bcm_dbg_data *data)
{
	unsigned int cmd[4] = {0, 0, 0, 0};
	int i, ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&data->lock, flags);

	if (!ipc_base_info || !sample_id) {
		BCM_ERR("%s: pointer is NULL\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	exynos_bcm_dbg_set_base_cmd(cmd, ipc_base_info);

	if (ipc_base_info->event_id != BCM_EVT_EVENT_SAMPLE_ID) {
		BCM_ERR("%s: Invalid Event ID(%d)\n", __func__,
					ipc_base_info->event_id);
		ret = -EINVAL;
		goto out;
	}

	if (ipc_base_info->ip_range == BCM_EACH)
		cmd[0] |= BCM_CMD_SET(bcm_ip_index, BCM_IP_MASK, BCM_IP_SHIFT);

	if (ipc_base_info->direction == BCM_EVT_SET) {
		cmd[1] = sample_id->peak_mask;
		cmd[2] = sample_id->peak_id;
		for (i = 0; i < BCM_EVT_EVENT_MAX; i++)
			cmd[3] |= BCM_CMD_SET(sample_id->peak_enable[i],
					BCM_ONE_BIT_MASK, BCM_EVT_FLT_ACT_SHIFT(i));
	}

	/* send command for BCM Sample ID */
	ret = __exynos_bcm_dbg_ipc_send_data(IPC_BCM_DBG_EVENT, data, cmd);
	if (ret) {
		BCM_ERR("%s: Failed send data\n", __func__);
		goto out;
	}

	if (ipc_base_info->direction == BCM_EVT_GET) {
		sample_id->peak_mask = cmd[1];
		sample_id->peak_id = cmd[2];
		for (i = 0; i < BCM_EVT_EVENT_MAX; i++)
			sample_id->peak_enable[i] = BCM_CMD_GET(cmd[3],
							BCM_ONE_BIT_MASK,
							BCM_EVT_FLT_ACT_SHIFT(i));
	}

out:
	spin_unlock_irqrestore(&data->lock, flags);

	return ret;
}
EXPORT_SYMBOL(exynos_bcm_dbg_sample_id_ctrl);

static int exynos_bcm_dbg_event_sm_user_ctrl(struct exynos_bcm_ipc_base_info *ipc_base_info,
					     struct exynos_bcm_event_sm_user_info *event_sm_user,
					     unsigned int bcm_ip_index,
					     struct exynos_bcm_dbg_data *data)
{
	unsigned int cmd[4] = {0, 0, 0, 0};
	int ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&data->lock, flags);

	if (!ipc_base_info || !event_sm_user) {
		BCM_ERR("%s: pointer is NULL\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	exynos_bcm_dbg_set_base_cmd(cmd, ipc_base_info);

	if (ipc_base_info->event_id != BCM_EVT_EVENT_SM_USER) {
		BCM_ERR("%s: Invalid Event ID(%d)\n", __func__, ipc_base_info->event_id);
		ret = -EINVAL;
		goto out;
	}

	if (ipc_base_info->ip_range == BCM_EACH)
		cmd[0] |= BCM_CMD_SET(bcm_ip_index, BCM_IP_MASK, BCM_IP_SHIFT);

	if (ipc_base_info->direction == BCM_EVT_SET) {
		cmd[1] = event_sm_user->sm_user_mask;
		cmd[2] = event_sm_user->sm_user_value;
	}

	/* send command for BCM event_sm_user control */
	ret = __exynos_bcm_dbg_ipc_send_data(IPC_BCM_DBG_EVENT, data, cmd);
	if (ret) {
		BCM_ERR("%s: Failed send data\n", __func__);
		goto out;
	}

	if (ipc_base_info->direction == BCM_EVT_GET) {
		event_sm_user->sm_user_mask = cmd[1];
		event_sm_user->sm_user_value = cmd[2];
	}

out:
	spin_unlock_irqrestore(&data->lock, flags);

	return ret;
}

int exynos_bcm_dbg_run_ctrl(struct exynos_bcm_ipc_base_info *ipc_base_info,
					unsigned int *bcm_run,
					struct exynos_bcm_dbg_data *data)
{
	unsigned int cmd[4] = {0, 0, 0, 0};
	unsigned int run, low_ktime, high_ktime;
	int ret = 0;
	u64 ktime;
	unsigned long flags;

	spin_lock_irqsave(&data->lock, flags);

	if (!ipc_base_info) {
		BCM_ERR("%s: pointer is NULL\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	exynos_bcm_dbg_set_base_cmd(cmd, ipc_base_info);

	if (ipc_base_info->event_id != BCM_EVT_RUN_CONT) {
		BCM_ERR("%s: Invalid Event ID(%d)\n", __func__,
					ipc_base_info->event_id);
		ret = -EINVAL;
		goto out;
	}

	if (ipc_base_info->direction == BCM_EVT_SET) {
		run = *bcm_run;

		if (!(run ^ data->bcm_run_state)) {
			BCM_INFO("%s: same run control command(%u)"
				"bcm_run_state(%u)\n", __func__,
				run, data->bcm_run_state);
			goto out;
		}

		cmd[0] |= BCM_CMD_SET(run, BCM_ONE_BIT_MASK,
					BCM_EVT_RUN_CONT_SHIFT);

		if (run == BCM_STOP) {
			ktime = sched_clock();
			low_ktime = (unsigned int)(ktime & EXYNOS_BCM_U64_LOW_MASK);
			high_ktime = (unsigned int)((ktime & EXYNOS_BCM_U64_HIGH_MASK)
							>> EXYNOS_BCM_32BIT_SHIFT);
			cmd[1] = low_ktime;
			cmd[2] = high_ktime;
		}
	}

	/* send command for BCM Run */
	ret = __exynos_bcm_dbg_ipc_send_data(IPC_BCM_DBG_EVENT, data, cmd);
	if (ret) {
		BCM_ERR("%s: Failed send data\n", __func__);
		goto out;
	}

	if (ipc_base_info->direction == BCM_EVT_GET) {
		run = BCM_CMD_GET(cmd[0], BCM_ONE_BIT_MASK,
					BCM_EVT_RUN_CONT_SHIFT);
		*bcm_run = run;
	} else if (ipc_base_info->direction == BCM_EVT_SET) {
		data->bcm_run_state = run;
	}

	spin_unlock_irqrestore(&data->lock, flags);

	/* dumping data from buffer */
	if (run == BCM_STOP && ipc_base_info->direction == BCM_EVT_SET) {
		if (data->dump_klog || data->dump_file)
			exynos_bcm_dbg_buffer_dump(data);
	}

	return ret;

out:
	spin_unlock_irqrestore(&data->lock, flags);

	return ret;
}
EXPORT_SYMBOL(exynos_bcm_dbg_run_ctrl);

int exynos_bcm_dbg_period_ctrl(struct exynos_bcm_ipc_base_info *ipc_base_info,
					unsigned int *bcm_period,
					struct exynos_bcm_dbg_data *data)
{
	unsigned int cmd[4] = {0, 0, 0, 0};
	unsigned int period;
	int ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&data->lock, flags);

	if (!ipc_base_info) {
		BCM_ERR("%s: pointer is NULL\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	exynos_bcm_dbg_set_base_cmd(cmd, ipc_base_info);

	if (ipc_base_info->event_id != BCM_EVT_PERIOD_CONT) {
		BCM_ERR("%s: Invalid Event ID(%d)\n", __func__,
					ipc_base_info->event_id);
		ret = -EINVAL;
		goto out;
	}

	if (ipc_base_info->direction == BCM_EVT_SET) {
		/*
		 * check bcm running state
		 * When bcm is running, value can not set
		 */
		ret = exynos_bcm_is_running(data->bcm_run_state);
		if (ret)
			goto out;

		period = *bcm_period;

		/* valid check for period range */
		if (!(period >= BCM_TIMER_PERIOD_MIN &&
			period <= BCM_TIMER_PERIOD_MAX)) {
			BCM_ERR("%s: Invalid period range(%uusec),(%d ~ %dusec)\n",
					__func__, period,
					BCM_TIMER_PERIOD_MIN, BCM_TIMER_PERIOD_MAX);
			ret = -EINVAL;
			goto out;
		}

		cmd[1] |= BCM_CMD_SET(period, BCM_EVT_PERIOD_CONT_MASK,
					BCM_EVT_PERIOD_CONT_SHIFT);
	}

	/* send command for BCM Period */
	ret = __exynos_bcm_dbg_ipc_send_data(IPC_BCM_DBG_EVENT, data, cmd);
	if (ret) {
		BCM_ERR("%s: Failed send data\n", __func__);
		goto out;
	}

	if (ipc_base_info->direction == BCM_EVT_GET) {
		period = BCM_CMD_GET(cmd[1], BCM_EVT_PERIOD_CONT_MASK,
					BCM_EVT_PERIOD_CONT_SHIFT);
		*bcm_period = period;
	}

out:
	spin_unlock_irqrestore(&data->lock, flags);

	return ret;
}
EXPORT_SYMBOL(exynos_bcm_dbg_period_ctrl);

int exynos_bcm_dbg_mode_ctrl(struct exynos_bcm_ipc_base_info *ipc_base_info,
					unsigned int *bcm_mode,
					struct exynos_bcm_dbg_data *data)
{
	unsigned int cmd[4] = {0, 0, 0, 0};
	unsigned int mode;
	int ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&data->lock, flags);

	if (!ipc_base_info) {
		BCM_ERR("%s: pointer is NULL\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	exynos_bcm_dbg_set_base_cmd(cmd, ipc_base_info);

	if (ipc_base_info->event_id != BCM_EVT_MODE_CONT) {
		BCM_ERR("%s: Invalid Event ID(%d)\n", __func__,
					ipc_base_info->event_id);
		ret = -EINVAL;
		goto out;
	}

	if (ipc_base_info->direction == BCM_EVT_SET) {
		/*
		 * check bcm running state
		 * When bcm is running, value can not set
		 */
		ret = exynos_bcm_is_running(data->bcm_run_state);
		if (ret)
			goto out;

		mode = *bcm_mode;

		if (mode >= BCM_MODE_MAX) {
			BCM_ERR("%s: Invalid BCM mode(%u), BCM mode max(%d)\n",
					__func__, mode, BCM_MODE_MAX);
			ret = -EINVAL;
			goto out;
		}

		cmd[0] |= BCM_CMD_SET(mode, BCM_EVT_MODE_CONT_MASK,
					BCM_EVT_MODE_CONT_SHIFT);
	}

	/* send command for BCM Mode */
	ret = __exynos_bcm_dbg_ipc_send_data(IPC_BCM_DBG_EVENT, data, cmd);
	if (ret) {
		BCM_ERR("%s: Failed send data\n", __func__);
		goto out;
	}

	if (ipc_base_info->direction == BCM_EVT_GET) {
		mode = BCM_CMD_GET(cmd[0], BCM_EVT_MODE_CONT_MASK,
					BCM_EVT_MODE_CONT_SHIFT);
		*bcm_mode = mode;
	}

out:
	spin_unlock_irqrestore(&data->lock, flags);

	return ret;
}
EXPORT_SYMBOL(exynos_bcm_dbg_mode_ctrl);

static int exynos_bcm_dbg_str_ctrl(struct exynos_bcm_ipc_base_info *ipc_base_info,
					unsigned int *ap_suspend,
					struct exynos_bcm_dbg_data *data)
{
	unsigned int cmd[4] = {0, 0, 0, 0};
	unsigned int suspend;
	int ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&data->lock, flags);

	if (!ipc_base_info) {
		BCM_ERR("%s: pointer is NULL\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	exynos_bcm_dbg_set_base_cmd(cmd, ipc_base_info);

	if (ipc_base_info->event_id != BCM_EVT_STR_STATE) {
		BCM_ERR("%s: Invalid Event ID(%d)\n", __func__,
					ipc_base_info->event_id);
		ret = -EINVAL;
		goto out;
	}

	if (ipc_base_info->direction == BCM_EVT_SET) {
		suspend = *ap_suspend;
		cmd[0] |= BCM_CMD_SET(suspend, BCM_ONE_BIT_MASK,
					BCM_EVT_STR_STATE_SHIFT);
	}

	/* send command for BCM STR state */
	ret = __exynos_bcm_dbg_ipc_send_data(IPC_BCM_DBG_EVENT, data, cmd);
	if (ret) {
		BCM_ERR("%s: Failed send data\n", __func__);
		goto out;
	}

	if (ipc_base_info->direction == BCM_EVT_GET) {
		suspend = BCM_CMD_GET(cmd[0], BCM_ONE_BIT_MASK,
					BCM_EVT_STR_STATE_SHIFT);
		*ap_suspend = suspend;
	}

out:
	spin_unlock_irqrestore(&data->lock, flags);

	return ret;
}

int exynos_bcm_dbg_ip_ctrl(struct exynos_bcm_ipc_base_info *ipc_base_info,
					unsigned int *bcm_ip_enable,
					unsigned int bcm_ip_index,
					struct exynos_bcm_dbg_data *data)
{
	unsigned int cmd[4] = {0, 0, 0, 0};
	unsigned int ip_enable;
	int ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&data->lock, flags);

	if (!ipc_base_info) {
		BCM_ERR("%s: pointer is NULL\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	exynos_bcm_dbg_set_base_cmd(cmd, ipc_base_info);

	if (ipc_base_info->event_id != BCM_EVT_IP_CONT) {
		BCM_ERR("%s: Invalid Event ID(%d)\n", __func__,
					ipc_base_info->event_id);
		ret = -EINVAL;
		goto out;
	}

	if (ipc_base_info->ip_range == BCM_EACH)
		cmd[0] |= BCM_CMD_SET(bcm_ip_index, BCM_IP_MASK, BCM_IP_SHIFT);

	if (ipc_base_info->direction == BCM_EVT_SET) {
		ip_enable = *bcm_ip_enable;
		cmd[0] |= BCM_CMD_SET(ip_enable, BCM_ONE_BIT_MASK,
					BCM_EVT_IP_CONT_SHIFT);
	}

	/* send command for BCM IP control */
	ret = __exynos_bcm_dbg_ipc_send_data(IPC_BCM_DBG_EVENT, data, cmd);
	if (ret) {
		BCM_ERR("%s: Failed send data\n", __func__);
		goto out;
	}

	if (ipc_base_info->direction == BCM_EVT_GET) {
		ip_enable = BCM_CMD_GET(cmd[0], BCM_ONE_BIT_MASK,
					BCM_EVT_IP_CONT_SHIFT);
		*bcm_ip_enable = ip_enable;
	}

out:
	spin_unlock_irqrestore(&data->lock, flags);

	return ret;
}
EXPORT_SYMBOL(exynos_bcm_dbg_ip_ctrl);

int exynos_bcm_dbg_dump_accumulators_ctrl(
			struct exynos_bcm_ipc_base_info *ipc_base_info,
			char *buf, size_t *buf_len, loff_t off, size_t size,
			struct exynos_bcm_dbg_data *data)
{
	unsigned int cmd[4] = {0, 0, 0, 0};
	int ret = 0;
	unsigned long flags;

	if (off != 0)
		goto dump;

	spin_lock_irqsave(&data->lock, flags);

	if (!ipc_base_info) {
		BCM_ERR("%s: pointer is NULL\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	exynos_bcm_dbg_set_base_cmd(cmd, ipc_base_info);

	if (ipc_base_info->event_id != BCM_EVT_DUMP_ACCUMULATORS) {
		BCM_ERR("%s: Invalid Event ID(%d)\n", __func__,
					ipc_base_info->event_id);
		ret = -EINVAL;
		goto out;
	}

	if (ipc_base_info->direction != BCM_EVT_SET) {
		ret = -EINVAL;
		goto out;
	}

	cmd[0] |= BCM_CMD_SET(1, BCM_ONE_BIT_MASK, BCM_EVT_DUMP_ACC_CONT_SHIFT);

	/* send command for BCM IP control */
	ret = __exynos_bcm_dbg_ipc_send_data(IPC_BCM_DBG_EVENT, data, cmd);
	if (ret) {
		BCM_ERR("%s: Failed send data\n", __func__);
		goto out;
	}

	spin_unlock_irqrestore(&data->lock, flags);

dump:
	/* print accumulator data from buffer */
	return exynos_bcm_dbg_print_accumulators(data, data->dump_klog, buf,
						 buf_len, off, size);

out:
	spin_unlock_irqrestore(&data->lock, flags);

	return ret;
}
EXPORT_SYMBOL(exynos_bcm_dbg_dump_accumulators_ctrl);

#define PPMU_VER_LINE_MAX_LEN	80		/* Includes PPMU IP names */
#define PPMU_VER_LEN		8		/* "nn.nn.nn" */

static ssize_t show_ppmu_ver(struct file *fp, char __user *ubuf, size_t size, loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	unsigned int cmd[4] = {0, 0, 0, 0};
	int ppmu_ver;
	char line_buf[PPMU_VER_LINE_MAX_LEN + 1];
	char ppmu_ver_buf[PPMU_VER_LEN + 1];
	char ppmu_text_buf[PPMU_VER_LEN + 16];
	ssize_t count = 0;
	int ppmu_idx;
	int ret = 0;

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	/* Handle the first round */
	if (*ppos == 0)
		data->bcm_cnt_nr = 0;

	/* Get the PPMU version values */
	for (ppmu_idx = data->bcm_cnt_nr;
			ppmu_idx < data->bcm_ip_nr &&
			size - count > PPMU_VER_LINE_MAX_LEN;
			ppmu_idx++) {
		/*
		 * Note: ">" instead of ">=": Leave room for the final
		 *       scnprintf's '\0'
		 */

		if (data->initial_run_ip[ppmu_idx]) {

			cmd[0] = ppmu_idx;

			ret = exynos_bcm_dbg_ipc_send_data(
					IPC_BCM_DBG_GET_PPMU_VER,
					data, cmd);
			if (ret) {
				BCM_ERR("%s: Failed send data\n", __func__);
				goto out;
			}

			ppmu_ver = cmd[0];

			if (ppmu_ver) {
				scnprintf(ppmu_ver_buf, sizeof(ppmu_ver_buf),
						"%u.%u.%u",
						(ppmu_ver >> 16) & 0xf,
						(ppmu_ver >> 12) & 0xf,
						(ppmu_ver >> 8) & 0xf);

				scnprintf(ppmu_text_buf, sizeof(ppmu_text_buf),
						"Arch: %-*s  RTL: %-2u",
						PPMU_VER_LEN,
						ppmu_ver_buf,
						ppmu_ver & 0xf);
			} else {
				scnprintf(ppmu_text_buf, sizeof(ppmu_text_buf),
						"%-*s",
						PPMU_VER_LEN + 15,
						"Power domain off");
			}

		} else {
			scnprintf(ppmu_text_buf, sizeof(ppmu_text_buf),
					"%-*s",
					PPMU_VER_LEN + 15,
					"Not available");
		}

		scnprintf(line_buf, sizeof(line_buf),
				"PPMU %3u:  %s  Name: %s",
				ppmu_idx,
				ppmu_text_buf,
				data->bcm_ip_names[ppmu_idx]);

		count += scnprintf(buf + count, size - count, "%s\n", line_buf);
	}

	data->bcm_cnt_nr = ppmu_idx;

	ret = count;
	if (copy_to_user(ubuf, buf, ret)) {
		ret = -EFAULT;
		goto out;
	}
	*ppos += ret;
out:
	kfree(buf);

	return ret;
}

#if IS_ENABLED(CONFIG_DEBUG_SNAPSHOT)
static int exynos_bcm_dbg_dump_addr_ctrl(struct exynos_bcm_ipc_base_info *ipc_base_info,
					struct exynos_bcm_dump_addr *dump_addr,
					struct exynos_bcm_dbg_data *data)
{
	unsigned int cmd[4] = {0, 0, 0, 0};
	int ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&data->lock, flags);

	if (!ipc_base_info) {
		BCM_ERR("%s: pointer is NULL\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	exynos_bcm_dbg_set_base_cmd(cmd, ipc_base_info);

	if (ipc_base_info->event_id != BCM_EVT_DUMP_ADDR) {
		BCM_ERR("%s: Invalid Event ID(%d)\n", __func__,
					ipc_base_info->event_id);
		ret = -EINVAL;
		goto out;
	}

	if (ipc_base_info->direction == BCM_EVT_SET) {
		/*
		 * check bcm running state
		 * When bcm is running, value can not set
		 */
		ret = exynos_bcm_is_running(data->bcm_run_state);
		if (ret)
			goto out;

		if (!dump_addr->p_addr || !dump_addr->buff_size) {
			BCM_ERR("%s: No dump address info: p_addr(0x%08x), buff_size(0x%08x)\n",
					__func__, dump_addr->p_addr, dump_addr->buff_size);
			ret = -EINVAL;
			goto out;
		}

		cmd[1] = (unsigned int)dump_addr->p_addr;
		cmd[2] = (unsigned int)dump_addr->buff_size;
	}

	/* send command for BCM Dump address info */
	ret = __exynos_bcm_dbg_ipc_send_data(IPC_BCM_DBG_EVENT, data, cmd);
	if (ret) {
		BCM_ERR("%s: Failed send data\n", __func__);
		goto out;
	}

out:
	spin_unlock_irqrestore(&data->lock, flags);

	return ret;
}
#endif

static int exynos_bcm_dbg_early_init(struct exynos_bcm_dbg_data *data)
{
	struct exynos_bcm_ipc_base_info ipc_base_info;
	struct exynos_bcm_event bcm_event;
	struct exynos_bcm_filter_id filter_id;
	struct exynos_bcm_filter_others filter_others;
	struct exynos_bcm_sample_id sample_id;
	unsigned int default_event;
	int ev_cnt, othr_cnt, ip_cnt;
	int ret = 0;

	/* pre-defined event set */
	exynos_bcm_dbg_set_base_info(&ipc_base_info, BCM_EVT_PRE_DEFINE,
					BCM_EVT_SET, BCM_ALL);

	default_event = data->default_define_event;
	bcm_event.index = data->define_event[default_event].index;
	for (ev_cnt = 0; ev_cnt < BCM_EVT_EVENT_MAX; ev_cnt++)
		bcm_event.event[ev_cnt] =
			data->define_event[default_event].event[ev_cnt];

	ret = exynos_bcm_dbg_event_ctrl(&ipc_base_info, &bcm_event, 0, data);
	if (ret) {
		BCM_ERR("%s: failed set event\n", __func__);
		return ret;
	}

	/* default filter id set */
	exynos_bcm_dbg_set_base_info(&ipc_base_info, BCM_EVT_EVENT_FLT_ID,
					BCM_EVT_SET, BCM_ALL);

	filter_id.sm_id_mask = data->define_filter_id[default_event].sm_id_mask;
	filter_id.sm_id_value = data->define_filter_id[default_event].sm_id_value;
	for (ev_cnt = 0; ev_cnt < BCM_EVT_EVENT_MAX; ev_cnt++)
		filter_id.sm_id_active[ev_cnt] =
			data->define_filter_id[default_event].sm_id_active[ev_cnt];

	ret = exynos_bcm_dbg_filter_id_ctrl(&ipc_base_info, &filter_id,
						0, data);
	if (ret) {
		BCM_ERR("%s: failed set filter ID\n", __func__);
		return ret;
	}

	/* default filter others set */
	exynos_bcm_dbg_set_base_info(&ipc_base_info, BCM_EVT_EVENT_FLT_OTHERS,
					BCM_EVT_SET, BCM_ALL);

	for (othr_cnt = 0; othr_cnt < BCM_EVT_FLT_OTHR_MAX; othr_cnt++) {
		filter_others.sm_other_type[othr_cnt] =
			data->define_filter_others[default_event].sm_other_type[othr_cnt];
		filter_others.sm_other_mask[othr_cnt] =
			data->define_filter_others[default_event].sm_other_mask[othr_cnt];
		filter_others.sm_other_value[othr_cnt] =
			data->define_filter_others[default_event].sm_other_value[othr_cnt];
	}

	for (ev_cnt = 0; ev_cnt < BCM_EVT_EVENT_MAX; ev_cnt++)
		filter_others.sm_other_active[ev_cnt] =
			data->define_filter_others[default_event].sm_other_active[ev_cnt];

	ret = exynos_bcm_dbg_filter_others_ctrl(&ipc_base_info,
					&filter_others, 0, data);
	if (ret) {
		BCM_ERR("%s: failed set filter others\n", __func__);
		return ret;
	}

	/* default sample id set */
	exynos_bcm_dbg_set_base_info(&ipc_base_info, BCM_EVT_EVENT_SAMPLE_ID,
					BCM_EVT_SET, BCM_ALL);

	sample_id.peak_mask = data->define_sample_id[default_event].peak_mask;
	sample_id.peak_id = data->define_sample_id[default_event].peak_id;
	for (ev_cnt = 0; ev_cnt < BCM_EVT_EVENT_MAX; ev_cnt++)
		sample_id.peak_enable[ev_cnt] =
			data->define_sample_id[default_event].peak_enable[ev_cnt];

	ret = exynos_bcm_dbg_sample_id_ctrl(&ipc_base_info, &sample_id,
						0, data);
	if (ret) {
		BCM_ERR("%s: failed set sample ID\n", __func__);
		return ret;
	}

	/* default period set */
	exynos_bcm_dbg_set_base_info(&ipc_base_info, BCM_EVT_PERIOD_CONT,
					BCM_EVT_SET, 0);

	ret = exynos_bcm_dbg_period_ctrl(&ipc_base_info,
					&data->initial_period, data);
	if (ret) {
		BCM_ERR("%s: failed set period\n", __func__);
		return ret;
	}

	/* default mode set */
	exynos_bcm_dbg_set_base_info(&ipc_base_info, BCM_EVT_MODE_CONT,
					BCM_EVT_SET, 0);

	ret = exynos_bcm_dbg_mode_ctrl(&ipc_base_info,
					&data->initial_bcm_mode, data);
	if (ret) {
		BCM_ERR("%s: failed set mode\n", __func__);
		return ret;
	}

	/* default run ip set */
	for (ip_cnt = 0; ip_cnt < data->bcm_ip_nr; ip_cnt++) {
		exynos_bcm_dbg_set_base_info(&ipc_base_info, BCM_EVT_IP_CONT,
						BCM_EVT_SET, BCM_EACH);

		ret = exynos_bcm_dbg_ip_ctrl(&ipc_base_info,
				&data->initial_run_ip[ip_cnt], ip_cnt, data);
		if (ret) {
			BCM_ERR("%s: failed set IP control\n", __func__);
			return ret;
		}
	}

	return 0;
}

static int exynos_bcm_dbg_run(unsigned int bcm_run,
				struct exynos_bcm_dbg_data *data)
{
	struct exynos_bcm_ipc_base_info ipc_base_info;
	int ret;

	exynos_bcm_dbg_set_base_info(&ipc_base_info, BCM_EVT_RUN_CONT,
					BCM_EVT_SET, 0);

	ret = exynos_bcm_dbg_run_ctrl(&ipc_base_info, &bcm_run, data);
	if (ret) {
		BCM_ERR("%s: failed set Run state\n", __func__);
		return ret;
	}

	return 0;
}

void exynos_bcm_dbg_start(void)
{
	int ret;

	if (!bcm_dbg_data) {
		BCM_ERR("%s: bcm_dbg_data is not ready!\n", __func__);
		return;
	}

	ret = exynos_bcm_dbg_run(BCM_RUN, bcm_dbg_data);
	if (ret) {
		BCM_ERR("%s: failed to bcm start\n", __func__);
		return;
	}

	BCM_INFO("%s\n", __func__);
}
EXPORT_SYMBOL(exynos_bcm_dbg_start);

void exynos_bcm_dbg_stop(unsigned int bcm_stop_owner)
{
	int ret;

	if (!bcm_dbg_data) {
		BCM_ERR("%s: bcm_dbg_data is not ready!\n", __func__);
		return;
	}

	if (bcm_stop_owner >= STOP_OWNER_MAX) {
		BCM_ERR("Invalid stop owner (%u)\n", bcm_stop_owner);
		return;
	}

	if (!bcm_dbg_data->available_stop_owner[bcm_stop_owner]) {
		BCM_ERR("Have not stop permission (%u)\n", bcm_stop_owner);
		return;
	}

	ret = exynos_bcm_dbg_run(BCM_STOP, bcm_dbg_data);
	if (ret) {
		BCM_ERR("%s: failed to bcm stop\n", __func__);
		return;
	}

	BCM_INFO("%s\n", __func__);
}
EXPORT_SYMBOL(exynos_bcm_dbg_stop);

static int exynos_bcm_dbg_str(unsigned int suspend,
				struct exynos_bcm_dbg_data *data)
{
	struct exynos_bcm_ipc_base_info ipc_base_info;
	int ret;

	exynos_bcm_dbg_set_base_info(&ipc_base_info, BCM_EVT_STR_STATE,
					BCM_EVT_SET, 0);

	ret = exynos_bcm_dbg_str_ctrl(&ipc_base_info, &suspend, data);
	if (ret) {
		BCM_ERR("%s:failed set str state\n", __func__);
		return ret;
	}

	return 0;
}

void exynos_bcm_dbg_set_dump(bool enable_klog, bool enable_file,
			     struct exynos_bcm_dbg_data *data)
{
	data->dump_klog = enable_klog;
#if IS_ENABLED(CONFIG_EXYNOS_BCM_DBG_DUMP_FILE)
	data->dump_file = enable_file;
#else
	data->dump_file = false;
#endif
}
EXPORT_SYMBOL(exynos_bcm_dbg_set_dump);

/* Debugfs Interface */
static ssize_t show_ip_power_domains(struct file *fp, char __user *ubuf, size_t size, loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	ssize_t ret;
	int i;
	ssize_t count = 0;

	if (*ppos > 0)
		return 0;

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	count += scnprintf(buf, size - count, "=== IPC node info ===\n");

	count += scnprintf(buf + count, size - count, "IPC node name: %s\n",
					data->ipc_node->name);

	count += scnprintf(buf + count, size - count,
				"\n=== Local Power Domain info ===\n");
	count += scnprintf(buf + count, size - count,
				"pd_size: %u, pd_sync_init: %s\n",
				data->pd_size,
				pd_sync_init ? "true" : "false");

	for (i = 0; i < data->pd_size; i++)
		count += scnprintf(buf + count, size - count,
				   "pd_name: %12s, pd_index: %2u, pd_on: %s, "
				   "cal_pdid: 0x%08x\n",
				   data->pd_info[i]->pd_name,
				   data->pd_info[i]->pd_index,
				   data->pd_info[i]->on ? "true" : "false",
				   data->pd_info[i]->cal_pdid);

	ret = simple_read_from_buffer(ubuf, size, ppos, buf, count);
	kfree(buf);

	return ret;
}

static ssize_t show_predefined_events(struct file *fp, char __user *ubuf, size_t size, loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	ssize_t ret;
	int i, j;
	ssize_t count = 0;

	if (*ppos > 0)
		return 0;

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	count += scnprintf(buf + count, size - count,
				"\n=== Pre-defined Event info ===\n");
	for (i = 0; i < data->define_event_max; i++) {
		count += scnprintf(buf + count, size - count,
				  "Pre-defined Event index: %2u\n",
				  data->define_event[i].index);
		for (j = 0; j < BCM_EVT_EVENT_MAX; j++)
			count += scnprintf(buf + count, size - count,
					  " Event[%d]: 0x%02x\n", j,
					  data->define_event[i].event[j]);
	}

	count += scnprintf(buf + count, size - count,
				"Default Pre-defined Event index: %2u\n",
				data->default_define_event);
	count += scnprintf(buf + count, size - count,
				"Pre-defined Event Max: %2u\n",
				data->define_event_max);

	ret = simple_read_from_buffer(ubuf, size, ppos, buf, count);
	kfree(buf);

	return ret;
}

static ssize_t show_predefined_filters(struct file *fp, char __user *ubuf, size_t size,
		loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	ssize_t ret;
	int i, j;
	ssize_t count = 0;

	if (*ppos > 0)
		return 0;

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	count += scnprintf(buf + count, size - count,
			  "\n=== Filter ID info ===\n");
	for (i = 0; i < data->define_event_max; i++) {
		count += scnprintf(buf + count, size - count,
				   "Pre-defined Event index: %2u\n",
				   data->define_event[i].index);
		count += scnprintf(buf + count, size - count,
				   " Filter ID mask: 0x%08x\n",
				   data->define_filter_id[i].sm_id_mask);
		count += scnprintf(buf + count, size - count,
				   " Filter ID value: 0x%08x\n",
				   data->define_filter_id[i].sm_id_value);
		count += scnprintf(buf + count, size - count,
				   " Filter ID active\n");

		for (j = 0; j < BCM_EVT_EVENT_MAX; j++)
			count += scnprintf(buf + count, size - count,
					   "  Event[%d]: %u\n", j,
					   data->define_filter_id[i].sm_id_active[j]);
	}

	count += scnprintf(buf + count, size - count,
			  "\n=== Filter Others info ===\n");
	for (i = 0; i < data->define_event_max; i++) {
		count += scnprintf(buf + count, size - count,
				"Pre-defined Event index: %2u\n",
				data->define_event[i].index);

		for (j = 0; j < BCM_EVT_FLT_OTHR_MAX; j++) {
			count += scnprintf(buf + count, size - count,
					   " Filter Others type[%d]: 0x%02x\n",
					   j, data->define_filter_others[i].sm_other_type[j]);
			count += scnprintf(buf + count, size - count,
					   " Filter Others mask[%d]: 0x%02x\n",
					   j, data->define_filter_others[i].sm_other_mask[j]);
			count += scnprintf(buf + count, size - count,
					   " Filter Others value[%d]: 0x%02x\n",
					   j, data->define_filter_others[i].sm_other_value[j]);
		}

		count += scnprintf(buf + count, size - count,
				   " Filter Others active\n");

		for (j = 0; j < BCM_EVT_EVENT_MAX; j++)
			count += scnprintf(buf + count, size - count,
					   "  Event[%d]: %u\n", j,
					   data->define_filter_others[i].sm_other_active[j]);
	}

	ret = simple_read_from_buffer(ubuf, size, ppos, buf, count);
	kfree(buf);

	return ret;
}

static ssize_t show_predefined_sample_mask(struct file *fp, char __user *ubuf, size_t size,
		loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	ssize_t ret;
	int i, j;
	ssize_t count = 0;

	if (*ppos > 0)
		return 0;

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	count += scnprintf(buf + count, size - count,
			   "\n=== Sample ID info ===\n");
	for (i = 0; i < data->define_event_max; i++) {
		count += scnprintf(buf + count, size - count,
				   "Pre-defined Event index: %2u\n",
				   data->define_event[i].index);

		count += scnprintf(buf + count, size - count,
				   " Sample ID: peak_mask: 0x%08x\n",
				   data->define_sample_id[i].peak_mask);
		count += scnprintf(buf + count, size - count,
				   " Sample ID: peak_id: 0x%08x\n",
				   data->define_sample_id[i].peak_id);
		count += scnprintf(buf + count, size - count,
				   " Sample ID active\n");

		for (j = 0; j < BCM_EVT_EVENT_MAX; j++)
			count += scnprintf(buf + count, size - count,
					   "  Event[%d]: %u\n", j,
					   data->define_sample_id[i].peak_enable[j]);
	}

	ret = simple_read_from_buffer(ubuf, size, ppos, buf, count);
	kfree(buf);

	return ret;
}

static ssize_t show_boot_config(struct file *fp, char __user *ubuf, size_t size, loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	ssize_t ret;
	ssize_t count = 0;
	static int ip_cnt;

	if (ip_cnt >= data->bcm_ip_nr) {
		ip_cnt = 0;
		return 0;
	}

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	if (*ppos == 0) {
		count += scnprintf(buf + count, size - count,
				  "\n=== Ctrl Attr info ===\n");
		count += scnprintf(buf + count, size - count,
				  "Initial BCM run: %s\n",
				  data->initial_bcm_run ? "true" : "false");
		count += scnprintf(buf + count, size - count,
				  "Initial monitor period: %u usec\n",
				  data->initial_period);
		count += scnprintf(buf + count, size - count,
				  "Initial BCM mode: %u\n",
				  data->initial_bcm_mode);
		count += scnprintf(buf + count, size - count,
				  "Initial Run IPs\n");
	}

	do {
		count += scnprintf(buf + count, size - count,
				  " BCM IP[%d]: %s\n", ip_cnt,
				  data->initial_run_ip[ip_cnt] ?
				  "true" : "false");
		ip_cnt++;
	} while ((ip_cnt < data->bcm_ip_nr) &&
		 (ip_cnt % data->bcm_ip_print_nr));

	*ppos = 0;
	ret = simple_read_from_buffer(ubuf, size, ppos, buf, count);
	kfree(buf);

	return ret;
}

static ssize_t show_event_ctrl(struct file *fp, char __user *ubuf, size_t size, loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	struct exynos_bcm_ipc_base_info ipc_base_info;
	struct exynos_bcm_event bcm_event;
	ssize_t count = 0;
	int ev_cnt, ret;
	static int ip_cnt;

	if (ip_cnt >= data->bcm_ip_nr) {
		ip_cnt = 0;
		return 0;
	}

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	exynos_bcm_dbg_set_base_info(&ipc_base_info, BCM_EVT_EVENT,
					BCM_EVT_GET, BCM_EACH);

	do {
		ret = exynos_bcm_dbg_event_ctrl(&ipc_base_info, &bcm_event,
						ip_cnt, data);
		if (ret) {
			BCM_ERR("%s: failed get event(ip:%d)\n",
					__func__, ip_cnt);
			ip_cnt = 0;
			goto out;
		}

		count += scnprintf(buf + count, size - count,
				   "bcm[%2d]: def(%2u),",
				   ip_cnt, bcm_event.index);
		for (ev_cnt = 0; ev_cnt < BCM_EVT_EVENT_MAX; ev_cnt++)
			count += scnprintf(buf + count, size - count,
					   " (0x%02x),",
					   bcm_event.event[ev_cnt]);
		count += scnprintf(buf + count, size - count, "\n");
		ip_cnt++;
	} while ((ip_cnt < data->bcm_ip_nr) &&
		 (ip_cnt % data->bcm_ip_print_nr));

	*ppos = 0;
	ret = simple_read_from_buffer(ubuf, size, ppos, buf, count);
out:
	kfree(buf);

	return ret;
}

static ssize_t show_event_ctrl_help(struct file *fp, char __user *ubuf, size_t size, loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	ssize_t ret;
	ssize_t count = 0;

	if (*ppos > 0)
		return 0;

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	/* help show_event_ctrl */
	count += scnprintf(buf + count, size - count,
				"\n= event_ctrl get help =\n");
	count += scnprintf(buf + count, size - count, "Usage:\n");
	count += scnprintf(buf + count, size - count,
				"cat event_ctrl\n"
				"bcm[ip_index]: def(define_index), [ev0], [ev1], [ev2], [ev3], [ev4], [ev5], [ev6], [ev7]\n");

	/* help store_event_ctrl */
	count += scnprintf(buf + count, size - count,
			   "\n= event_ctrl set help =\n");
	count += scnprintf(buf + count, size - count, "Usage:\n");
	count += scnprintf(buf + count, size - count,
			   "echo [ip_range] [ip_index] [define_index] "
			   "[ev0] [ev1] [ev2] [ev3] [ev4] [ev5] [ev6] [ev7] > "
			   "event_ctrl\n");
	count += scnprintf(buf + count, size - count,
				"\nip_range: BCM_EACH(%d), BCM_ALL(%d)\n",
			   BCM_EACH, BCM_ALL);
	count += scnprintf(buf + count, size - count,
				"ip_index: number of bcm ip (0 ~ %u)\n"
				"          (if ip_range is all, set to 0)\n",
			   data->bcm_ip_nr - 1);
	count += scnprintf(buf + count, size - count,
				"define_index: index of pre-defined event (0 ~ %u)\n"
				"              0 means no pre-defined event\n",
			   data->define_event_max - 1);
	count += scnprintf(buf + count, size - count,
				"evX: event value of counter (if define_index is not 0, set to 0\n"
				"     event value should be in hex\n");

	ret = simple_read_from_buffer(ubuf, size, ppos, buf, count);
	kfree(buf);

	return ret;
}

static ssize_t store_event_ctrl(struct file *fp, const char __user *ubuf, size_t size, loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	struct exynos_bcm_ipc_base_info ipc_base_info;
	struct exynos_bcm_event bcm_event;
	unsigned int bcm_ip_index;
	unsigned int event_id, ip_range;
	unsigned int defined_index;
	unsigned int event[BCM_EVT_EVENT_MAX];
	int ev_cnt, dfd_cnt, ret;

	if (*ppos != 0)
		return size;

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	simple_write_to_buffer(buf, size, ppos, ubuf, size);

	ret = sscanf(buf, "%u %u %u %x %x %x %x %x %x %x %x",
			&ip_range, &bcm_ip_index, &defined_index,
			&event[0], &event[1], &event[2], &event[3],
			&event[4], &event[5], &event[6], &event[7]);
	kfree(buf);
	if (ret != 11) {
		BCM_ERR("%s: Invalid command string\n", __func__);
		return -EINVAL;
	}

	ret = exynos_bcm_ip_validate(ip_range, bcm_ip_index, data->bcm_ip_nr);
	if (ret)
		return ret;

	if (defined_index >= data->define_event_max) {
		BCM_ERR("%s: Invalid defined index(%u),"
			" defined_max_nr(%u)\n", __func__,
				defined_index, data->define_event_max - 1);
		return -EINVAL;
	}

	if (ip_range == BCM_ALL)
		bcm_ip_index = 0;

	if (defined_index != NO_PRE_DEFINE_EVT) {
		event_id = BCM_EVT_PRE_DEFINE;
		for (dfd_cnt = 1; dfd_cnt < data->define_event_max; dfd_cnt++) {
			if (defined_index ==
				data->define_event[dfd_cnt].index)
				break;
		}

		bcm_event.index = data->define_event[dfd_cnt].index;
		for (ev_cnt = 0; ev_cnt < BCM_EVT_EVENT_MAX; ev_cnt++)
			bcm_event.event[ev_cnt] =
				data->define_event[dfd_cnt].event[ev_cnt];
	} else {
		event_id = BCM_EVT_EVENT;
		bcm_event.index = NO_PRE_DEFINE_EVT;
		for (ev_cnt = 0; ev_cnt < BCM_EVT_EVENT_MAX; ev_cnt++)
			bcm_event.event[ev_cnt] = event[ev_cnt];
	}

	exynos_bcm_dbg_set_base_info(&ipc_base_info, event_id,
					BCM_EVT_SET, ip_range);

	ret = exynos_bcm_dbg_event_ctrl(&ipc_base_info, &bcm_event,
						bcm_ip_index, data);
	if (ret) {
		BCM_ERR("%s:failed set event\n", __func__);
		return ret;
	}

	return size;
}

static ssize_t show_filter_id_ctrl(struct file *fp, char __user *ubuf, size_t size, loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	struct exynos_bcm_ipc_base_info ipc_base_info;
	struct exynos_bcm_filter_id filter_id;
	ssize_t count = 0;
	int ret;
	static int ip_cnt;

	if (ip_cnt >= data->bcm_ip_nr) {
		ip_cnt = 0;
		return 0;
	}

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	exynos_bcm_dbg_set_base_info(&ipc_base_info, BCM_EVT_EVENT_FLT_ID,
					BCM_EVT_GET, BCM_EACH);

	do {
		ret = exynos_bcm_dbg_filter_id_ctrl(&ipc_base_info,
						&filter_id, ip_cnt, data);
		if (ret) {
			BCM_ERR("%s: failed get filter id(ip:%d)\n",
					__func__, ip_cnt);
			ip_cnt = 0;
			goto out;
		}

		count += scnprintf(buf + count, size - count,
				   "bcm[%2d]: mask(0x%08x), value(0x%08x)\n",
				   ip_cnt, filter_id.sm_id_mask,
				   filter_id.sm_id_value);
		ip_cnt++;
	} while ((ip_cnt < data->bcm_ip_nr) &&
		 (ip_cnt % data->bcm_ip_print_nr));

	*ppos = 0;
	ret = simple_read_from_buffer(ubuf, size, ppos, buf, count);
out:
	kfree(buf);

	return ret;
}

static ssize_t show_filter_id_active(struct file *fp, char __user *ubuf, size_t size, loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	struct exynos_bcm_ipc_base_info ipc_base_info;
	struct exynos_bcm_filter_id filter_id;
	ssize_t count = 0;
	int ev_cnt, ret;
	static int ip_cnt;

	if (ip_cnt >= data->bcm_ip_nr) {
		ip_cnt = 0;
		return 0;
	}

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	exynos_bcm_dbg_set_base_info(&ipc_base_info, BCM_EVT_EVENT_FLT_ID,
					BCM_EVT_GET, BCM_EACH);

	do {
		ret = exynos_bcm_dbg_filter_id_ctrl(&ipc_base_info,
						&filter_id, ip_cnt, data);
		if (ret) {
			BCM_ERR("%s: failed get filter id(ip:%d)\n",
					__func__, ip_cnt);
			ip_cnt = 0;
			goto out;
		}

		count += scnprintf(buf + count, size - count, "bcm[%2d]:",
				   ip_cnt);
		for (ev_cnt = 0; ev_cnt < BCM_EVT_EVENT_MAX; ev_cnt++)
			count += scnprintf(buf + count, size - count,
					" ev%d %u,", ev_cnt,
					filter_id.sm_id_active[ev_cnt]);
		count += scnprintf(buf + count, size - count, "\n");
		ip_cnt++;
	} while ((ip_cnt < data->bcm_ip_nr) &&
		 (ip_cnt % data->bcm_ip_print_nr));

	*ppos = 0;
	ret = simple_read_from_buffer(ubuf, size, ppos, buf, count);
out:
	kfree(buf);

	return ret;
}

static ssize_t show_filter_id_ctrl_help(struct file *fp, char __user *ubuf, size_t size,
		loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	ssize_t ret;
	ssize_t count = 0;

	if (*ppos > 0)
		return 0;

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	/* help show_filter_id_ctrl */
	count += scnprintf(buf + count, size - count,
				"\n= filter_id_ctrl get help =\n");
	count += scnprintf(buf + count, size - count, "Usage:\n");
	count += scnprintf(buf + count, size - count,
				"cat filter_id_ctrl\n"
				"bcm[ip_index]: [mask], [value]\n");

	/* help store_filter_id_ctrl */
	count += scnprintf(buf + count, size - count,
			   "\n= filter_id_ctrl set help =\n");
	count += scnprintf(buf + count, size - count, "Usage:\n");
	count += scnprintf(buf + count, size - count,
			   "echo [ip_range] [ip_index] [define_index] [mask] "
			   "[value] [ev0] [ev1] [ev2] [ev3] [ev4] [ev5] [ev6] "
			   "[ev7] > filter_id_ctrl\n");
	count += scnprintf(buf + count, size - count,
				"\nip_range: BCM_EACH(%d), BCM_ALL(%d)\n",
			   BCM_EACH, BCM_ALL);
	count += scnprintf(buf + count, size - count,
				"ip_index: number of bcm ip (0 ~ %u)\n"
				"          (if ip_range is all, set to 0)\n",
			   data->bcm_ip_nr - 1);
	count += scnprintf(buf + count, size - count,
				"define_index: index of pre-defined event (0 ~ %u)\n"
				"              0 means no pre-defined event\n",
			   data->define_event_max - 1);
	count += scnprintf(buf + count, size - count,
				"mask: masking for filter id (if define_index is not 0, set to 0)\n"
				"      mask value should be in hex\n");
	count += scnprintf(buf + count, size - count,
				"value: value of filter id (if define_index is not 0, set to 0)\n"
				"       value should be in hex\n");
	count += scnprintf(buf + count, size - count,
				"evX: event counter alloc for filter id (if define_index is not 0, set to 0)\n"
				"     value should be 0 or 1\n");

	ret = simple_read_from_buffer(ubuf, size, ppos, buf, count);
	kfree(buf);

	return ret;
}

static ssize_t store_filter_id_ctrl(struct file *fp, const char __user *ubuf, size_t size,
		loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	struct exynos_bcm_ipc_base_info ipc_base_info;
	struct exynos_bcm_filter_id filter_id;
	unsigned int bcm_ip_index, ip_range;
	unsigned int defined_index;
	unsigned int sm_id_mask, sm_id_value;
	unsigned int sm_id_active[BCM_EVT_EVENT_MAX];
	int ev_cnt, ret;

	if (*ppos != 0)
		return size;

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	simple_write_to_buffer(buf, size, ppos, ubuf, size);

	ret = sscanf(buf, "%u %u %u %x %x %u %u %u %u %u %u %u %u",
			&ip_range, &bcm_ip_index, &defined_index, &sm_id_mask, &sm_id_value,
			&sm_id_active[0], &sm_id_active[1], &sm_id_active[2],
			&sm_id_active[3], &sm_id_active[4], &sm_id_active[5],
			&sm_id_active[6], &sm_id_active[7]);
	kfree(buf);
	if (ret != 13) {
		BCM_ERR("%s: Invalid command string\n", __func__);
		return -EINVAL;
	}

	ret = exynos_bcm_ip_validate(ip_range, bcm_ip_index, data->bcm_ip_nr);
	if (ret)
		return ret;

	for (ev_cnt = 0; ev_cnt < BCM_EVT_EVENT_MAX; ev_cnt++) {
		if (sm_id_active[ev_cnt])
			sm_id_active[ev_cnt] = true;
	}

	if (defined_index >= data->define_event_max) {
		BCM_ERR("%s: Invalid defined index(%u),"
			" defined_max_nr(%u)\n", __func__,
				defined_index, data->define_event_max - 1);
		return -EINVAL;
	}

	if (ip_range == BCM_ALL)
		bcm_ip_index = 0;

	if (defined_index != NO_PRE_DEFINE_EVT) {
		filter_id.sm_id_mask =
			data->define_filter_id[defined_index].sm_id_mask;
		filter_id.sm_id_value =
			data->define_filter_id[defined_index].sm_id_value;
		for (ev_cnt = 0; ev_cnt < BCM_EVT_EVENT_MAX; ev_cnt++)
			filter_id.sm_id_active[ev_cnt] =
				data->define_filter_id[defined_index].sm_id_active[ev_cnt];
	} else {
		filter_id.sm_id_mask = sm_id_mask;
		filter_id.sm_id_value = sm_id_value;
		for (ev_cnt = 0; ev_cnt < BCM_EVT_EVENT_MAX; ev_cnt++)
			filter_id.sm_id_active[ev_cnt] = sm_id_active[ev_cnt];
	}

	exynos_bcm_dbg_set_base_info(&ipc_base_info, BCM_EVT_EVENT_FLT_ID,
					BCM_EVT_SET, ip_range);

	ret = exynos_bcm_dbg_filter_id_ctrl(&ipc_base_info, &filter_id,
						bcm_ip_index, data);
	if (ret) {
		BCM_ERR("%s:failed set filter ID\n", __func__);
		return ret;
	}

	return size;
}

static ssize_t show_filter_others_ctrl(struct file *fp, char __user *ubuf, size_t size,
		loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	struct exynos_bcm_ipc_base_info ipc_base_info;
	struct exynos_bcm_filter_others filter_others;
	ssize_t count = 0;
	int othr_cnt, ret;
	static int ip_cnt;

	if (ip_cnt >= data->bcm_ip_nr) {
		ip_cnt = 0;
		return 0;
	}

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	exynos_bcm_dbg_set_base_info(&ipc_base_info, BCM_EVT_EVENT_FLT_OTHERS,
					BCM_EVT_GET, BCM_EACH);

	do {
		ret = exynos_bcm_dbg_filter_others_ctrl(&ipc_base_info,
					&filter_others, ip_cnt, data);
		if (ret) {
			BCM_ERR("%s: failed get filter others(ip:%d)\n",
					__func__, ip_cnt);
			ip_cnt = 0;
			goto out;
		}

		count += scnprintf(buf + count, size - count, "bcm[%2d]:",
				   ip_cnt);
		for (othr_cnt = 0; othr_cnt < BCM_EVT_FLT_OTHR_MAX; othr_cnt++)
			count += scnprintf(buf + count, size - count,
				" type%d(0x%02x), mask%d(0x%02x), "
				"value%d(0x%02x),",
				othr_cnt, filter_others.sm_other_type[othr_cnt],
				othr_cnt, filter_others.sm_other_mask[othr_cnt],
				othr_cnt,
				filter_others.sm_other_value[othr_cnt]);
		count += scnprintf(buf + count, size - count, "\n");
		ip_cnt++;
	} while ((ip_cnt < data->bcm_ip_nr) &&
		 (ip_cnt % data->bcm_ip_print_nr));

	*ppos = 0;
	ret = simple_read_from_buffer(ubuf, size, ppos, buf, count);
out:
	kfree(buf);

	return ret;
}

static ssize_t show_filter_others_active(struct file *fp, char __user *ubuf, size_t size,
		loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	struct exynos_bcm_ipc_base_info ipc_base_info;
	struct exynos_bcm_filter_others filter_others;
	ssize_t count = 0;
	int ev_cnt, ret;
	static int ip_cnt;

	if (ip_cnt >= data->bcm_ip_nr) {
		ip_cnt = 0;
		return 0;
	}

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	exynos_bcm_dbg_set_base_info(&ipc_base_info, BCM_EVT_EVENT_FLT_OTHERS,
					BCM_EVT_GET, BCM_EACH);

	do {
		ret = exynos_bcm_dbg_filter_others_ctrl(&ipc_base_info,
					&filter_others, ip_cnt, data);
		if (ret) {
			BCM_ERR("%s: failed get filter others(ip:%d)\n",
				__func__, ip_cnt);
			ip_cnt = 0;
			goto out;
		}

		count += scnprintf(buf + count, size - count, "bcm[%2d]:",
				   ip_cnt);
		for (ev_cnt = 0; ev_cnt < BCM_EVT_EVENT_MAX; ev_cnt++)
			count += scnprintf(buf + count, size - count,
					" ev%d %u,", ev_cnt,
					filter_others.sm_other_active[ev_cnt]);
		count += scnprintf(buf + count, size - count, "\n");
		ip_cnt++;
	} while ((ip_cnt < data->bcm_ip_nr) &&
		 (ip_cnt % data->bcm_ip_print_nr));

	*ppos = 0;
	ret = simple_read_from_buffer(ubuf, size, ppos, buf, count);
out:
	kfree(buf);

	return ret;
}

static ssize_t show_filter_others_ctrl_help(struct file *fp, char __user *ubuf, size_t size,
		loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	ssize_t ret;
	ssize_t count = 0;
	int othr_cnt;

	if (*ppos > 0)
		return 0;

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	/* help show_filter_others_ctrl */
	count += scnprintf(buf + count, size - count,
			   "\n= filter_others_ctrl get help =\n");
	count += scnprintf(buf + count, size - count, "Usage:\n");
	count += scnprintf(buf + count, size - count,
				"cat filter_other_ctrl\n"
				"bcm[ip_index]: [type0], [mask0], [value0], [type1], [mask1], [value1]\n");

	/* help store_filter_others_ctrl */
	count += scnprintf(buf + count, size - count,
			   "\n= filter_others_ctrl set help =\n");
	count += scnprintf(buf + count, size - count, "Usage:\n");
	count += scnprintf(buf + count, size - count,
			   "echo [ip_range] [ip_index] [define_index] "
			   "[type0] [mask0] [value0] [type1] [mask1] [value1] "
			   "[ev0] [ev1] [ev2] [ev3] [ev4] [ev5] [ev6] [ev7] > "
			   "filter_others_ctrl\n");
	count += scnprintf(buf + count, size - count,
			   " ip_range: BCM_EACH(%d), BCM_ALL(%d)\n",
			   BCM_EACH, BCM_ALL);
	count += scnprintf(buf + count, size - count,
			   " ip_index: number of bcm ip (0 ~ %u)\n"
			   "           (if ip_range is all, set to 0)\n",
			   data->bcm_ip_nr - 1);
	count += scnprintf(buf + count, size - count,
			   " define_index: index of pre-defined event (0 ~ %u)\n"
			   "               0 means no pre-defined event\n",
			   data->define_event_max - 1);
	for (othr_cnt = 0; othr_cnt < BCM_EVT_FLT_OTHR_MAX; othr_cnt++) {
		count += scnprintf(buf + count, size - count,
				   " type%d: type%d for filter others"
				   " (if define_index is not 0, set to 0)\n"
				   "         type%d value should be in hex\n",
				   othr_cnt, othr_cnt, othr_cnt);
		count += scnprintf(buf + count, size - count,
				   " mask%d: mask%d for filter others"
				   " (if define_index is not 0, set to 0)\n"
				   "         mask%d value should be in hex\n",
				   othr_cnt, othr_cnt, othr_cnt);
		count += scnprintf(buf + count, size - count,
				   " value%d: value%d of filter others"
				   " (if define_index is not 0, set to 0)\n"
				   "          value%d should be in hex\n",
				   othr_cnt, othr_cnt, othr_cnt);
	}
	count += scnprintf(buf + count, size - count,
			   " evX: event counter alloc for filter others"
			   " (if define_index is not 0, set to 0)\n"
			   "      value should be 0 or 1\n");

	ret = simple_read_from_buffer(ubuf, size, ppos, buf, count);
	kfree(buf);

	return ret;
}

static ssize_t store_filter_others_ctrl(struct file *fp, const char __user *ubuf, size_t size,
		loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	struct exynos_bcm_ipc_base_info ipc_base_info;
	struct exynos_bcm_filter_others filter_others;
	unsigned int bcm_ip_index, ip_range;
	unsigned int defined_index;
	unsigned int sm_other_type[BCM_EVT_FLT_OTHR_MAX];
	unsigned int sm_other_mask[BCM_EVT_FLT_OTHR_MAX];
	unsigned int sm_other_value[BCM_EVT_FLT_OTHR_MAX];
	unsigned int sm_other_active[BCM_EVT_EVENT_MAX];
	int ev_cnt, othr_cnt, ret;

	if (*ppos != 0)
		return size;

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	simple_write_to_buffer(buf, size, ppos, ubuf, size);

	ret = sscanf(buf, "%u %u %u %x %x %x %x %x %x %u %u %u %u %u %u %u %u",
			&ip_range, &bcm_ip_index, &defined_index,
			&sm_other_type[0], &sm_other_mask[0], &sm_other_value[0],
			&sm_other_type[1], &sm_other_mask[1], &sm_other_value[1],
			&sm_other_active[0], &sm_other_active[1], &sm_other_active[2],
			&sm_other_active[3], &sm_other_active[4], &sm_other_active[5],
			&sm_other_active[6], &sm_other_active[7]);
	kfree(buf);
	if (ret != 17) {
		BCM_ERR("%s: Invalid command string\n", __func__);
		return -EINVAL;
	}

	ret = exynos_bcm_ip_validate(ip_range, bcm_ip_index, data->bcm_ip_nr);
	if (ret)
		return ret;

	for (ev_cnt = 0; ev_cnt < BCM_EVT_EVENT_MAX; ev_cnt++) {
		if (sm_other_active[ev_cnt])
			sm_other_active[ev_cnt] = true;
	}

	if (defined_index >= data->define_event_max) {
		BCM_ERR("%s: Invalid defined index(%u),"
			" defined_max_nr(%u)\n", __func__,
				defined_index, data->define_event_max - 1);
		return -EINVAL;
	}

	if (ip_range == BCM_ALL)
		bcm_ip_index = 0;

	if (defined_index != NO_PRE_DEFINE_EVT) {
		for (othr_cnt = 0; othr_cnt < BCM_EVT_FLT_OTHR_MAX; othr_cnt++) {
			filter_others.sm_other_type[othr_cnt] =
				data->define_filter_others[defined_index].sm_other_type[othr_cnt];
			filter_others.sm_other_mask[othr_cnt] =
				data->define_filter_others[defined_index].sm_other_mask[othr_cnt];
			filter_others.sm_other_value[othr_cnt] =
				data->define_filter_others[defined_index].sm_other_value[othr_cnt];
		}

		for (ev_cnt = 0; ev_cnt < BCM_EVT_EVENT_MAX; ev_cnt++)
			filter_others.sm_other_active[ev_cnt] =
				data->define_filter_others[defined_index].sm_other_active[ev_cnt];
	} else {
		for (othr_cnt = 0; othr_cnt < BCM_EVT_FLT_OTHR_MAX;
		     othr_cnt++) {
			filter_others.sm_other_type[othr_cnt] =
				sm_other_type[othr_cnt];
			filter_others.sm_other_mask[othr_cnt] =
				sm_other_mask[othr_cnt];
			filter_others.sm_other_value[othr_cnt] =
				sm_other_value[othr_cnt];
		}

		for (ev_cnt = 0; ev_cnt < BCM_EVT_EVENT_MAX; ev_cnt++)
			filter_others.sm_other_active[ev_cnt] =
				sm_other_active[ev_cnt];
	}

	exynos_bcm_dbg_set_base_info(&ipc_base_info, BCM_EVT_EVENT_FLT_OTHERS,
					BCM_EVT_SET, ip_range);

	ret = exynos_bcm_dbg_filter_others_ctrl(&ipc_base_info,
				&filter_others, bcm_ip_index, data);
	if (ret) {
		BCM_ERR("%s:failed set filter others\n", __func__);
		return ret;
	}

	return size;
}

static ssize_t show_event_sm_user_ctrl(struct file *fp, char __user *ubuf, size_t size,
				       loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	struct exynos_bcm_ipc_base_info ipc_base_info;
	struct exynos_bcm_event_sm_user_info event_sm_user;
	ssize_t count = 0;
	int ret;
	static int ip_cnt;

	if (ip_cnt >= data->bcm_ip_nr) {
		ip_cnt = 0;
		return 0;
	}

	buf = kmalloc(size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	exynos_bcm_dbg_set_base_info(&ipc_base_info, BCM_EVT_EVENT_SM_USER, BCM_EVT_GET, BCM_EACH);

	do {
		ret = exynos_bcm_dbg_event_sm_user_ctrl(&ipc_base_info, &event_sm_user, ip_cnt,
							data);
		if (ret) {
			BCM_ERR("%s: failed get event_sm_user(ip:%d)\n", __func__, ip_cnt);
			ip_cnt = 0;
			goto out;
		}

		count += scnprintf(buf + count, size - count, "bcm[%2d]: %x/%x\n", ip_cnt,
				   event_sm_user.sm_user_value, event_sm_user.sm_user_mask);
		ip_cnt++;
	} while ((ip_cnt < data->bcm_ip_nr) &&
		 (ip_cnt % data->bcm_ip_print_nr));

	*ppos = 0;
	ret = simple_read_from_buffer(ubuf, size, ppos, buf, count);
out:
	kfree(buf);

	return ret;
}

static ssize_t store_event_sm_user_ctrl(struct file *fp, const char __user *ubuf, size_t size,
					loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	struct exynos_bcm_ipc_base_info ipc_base_info;
	struct exynos_bcm_event_sm_user_info event_sm_user;
	unsigned int ip_range, bcm_ip_index;
	int ret;

	if (*ppos != 0)
		return size;

	buf = kmalloc(size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	simple_write_to_buffer(buf, size, ppos, ubuf, size);

	ret = sscanf(buf, "%u %u %x %x", &ip_range, &bcm_ip_index,
		     &event_sm_user.sm_user_value, &event_sm_user.sm_user_mask);
	kfree(buf);
	if (ret != 4) {
		BCM_ERR("%s: Invalid command string\n", __func__);
		return -EINVAL;
	}

	ret = exynos_bcm_ip_validate(ip_range, bcm_ip_index, data->bcm_ip_nr);
	if (ret)
		return ret;

	if (ip_range == BCM_ALL)
		bcm_ip_index = 0;

	exynos_bcm_dbg_set_base_info(&ipc_base_info, BCM_EVT_EVENT_SM_USER, BCM_EVT_SET, ip_range);

	ret = exynos_bcm_dbg_event_sm_user_ctrl(&ipc_base_info, &event_sm_user, bcm_ip_index, data);
	if (ret) {
		BCM_ERR("%s:failed set event sm user\n", __func__);
		return ret;
	}

	return size;
}

static ssize_t show_event_sm_user_ctrl_help(struct file *fp, char __user *ubuf, size_t size,
					    loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	ssize_t ret;
	ssize_t count = 0;

	if (*ppos > 0)
		return 0;

	buf = kmalloc(size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	/* help event_sm_user_ctrl */
	count += scnprintf(buf + count, size - count,
		"\n"
		"= event_sm_user_ctrl get help =\n"
		"Usage:\n"
		"cat event_sm_user_ctrl\n"
		" bcm[ip_index]: [sm_user_value]/[sm_user_mask]\n"
		"\n"
		"= event_sm_user_ctrl set help =\n"
		"Usage:\n"
		"echo [ip_range] [ip_index] [sm_user_value] [sm_user_mask] > event_sm_user_ctrl\n"
		" ip_range: BCM_EACH(%d), BCM_ALL(%d)\n"
		" ip_index: number of bcm ip (0 ~ %u)\n"
		"           (if ip_range is all, set to 0)\n",
		BCM_EACH, BCM_ALL, data->bcm_ip_nr - 1);

	ret = simple_read_from_buffer(ubuf, size, ppos, buf, count);
	kfree(buf);

	return ret;
}

static ssize_t show_sample_id_ctrl(struct file *fp, char __user *ubuf, size_t size, loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	struct exynos_bcm_ipc_base_info ipc_base_info;
	struct exynos_bcm_sample_id sample_id;
	ssize_t count = 0;
	int ret;
	static int ip_cnt;

	if (ip_cnt >= data->bcm_ip_nr) {
		ip_cnt = 0;
		return 0;
	}

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	exynos_bcm_dbg_set_base_info(&ipc_base_info, BCM_EVT_EVENT_SAMPLE_ID,
					BCM_EVT_GET, BCM_EACH);

	do {
		ret = exynos_bcm_dbg_sample_id_ctrl(&ipc_base_info,
						&sample_id, ip_cnt, data);
		if (ret) {
			BCM_ERR("%s: failed get sample id(ip:%d)\n",
				__func__, ip_cnt);
			ip_cnt = 0;
			goto out;
		}

		count += scnprintf(buf + count, size - count,
				   "bcm[%2d]: mask(0x%08x), id(0x%08x)\n",
				   ip_cnt, sample_id.peak_mask,
				   sample_id.peak_id);
		ip_cnt++;
	} while ((ip_cnt < data->bcm_ip_nr) &&
		 (ip_cnt % data->bcm_ip_print_nr));

	*ppos = 0;
	ret = simple_read_from_buffer(ubuf, size, ppos, buf, count);
out:
	kfree(buf);

	return ret;
}

static ssize_t show_sample_id_active(struct file *fp, char __user *ubuf, size_t size, loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	struct exynos_bcm_ipc_base_info ipc_base_info;
	struct exynos_bcm_sample_id sample_id;
	ssize_t count = 0;
	int ev_cnt, ret;
	static int ip_cnt;

	if (ip_cnt >= data->bcm_ip_nr) {
		ip_cnt = 0;
		return 0;
	}

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	exynos_bcm_dbg_set_base_info(&ipc_base_info, BCM_EVT_EVENT_SAMPLE_ID,
					BCM_EVT_GET, BCM_EACH);

	do {
		ret = exynos_bcm_dbg_sample_id_ctrl(&ipc_base_info,
						&sample_id, ip_cnt, data);
		if (ret) {
			BCM_ERR("%s: failed get sample id(ip:%d)\n",
				__func__, ip_cnt);
			ip_cnt = 0;
			goto out;
		}

		count += scnprintf(buf + count, size - count, "bcm[%2d]:",
				   ip_cnt);
		for (ev_cnt = 0; ev_cnt < BCM_EVT_EVENT_MAX; ev_cnt++)
			count += scnprintf(buf + count, size - count,
					   " ev%d %u,", ev_cnt,
					   sample_id.peak_enable[ev_cnt]);
		count += scnprintf(buf + count, size - count, "\n");
		ip_cnt++;
	} while ((ip_cnt < data->bcm_ip_nr) &&
		 (ip_cnt % data->bcm_ip_print_nr));

	*ppos = 0;
	ret = simple_read_from_buffer(ubuf, size, ppos, buf, count);
out:
	kfree(buf);

	return ret;
}

static ssize_t show_sample_id_ctrl_help(struct file *fp, char __user *ubuf, size_t size,
		loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	ssize_t ret;
	ssize_t count = 0;

	if (*ppos > 0)
		return 0;

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	/* help show_sample_id_ctrl */
	count += scnprintf(buf + count, size - count,
			   "\n= sample_id_ctrl get help =\n");
	count += scnprintf(buf + count, size - count, "Usage:\n");
	count += scnprintf(buf + count, size - count,
				"cat sample_id_ctrl\n"
				"bcm[ip_index]: [mask], [value]\n");

	/* help store_sample_id_ctrl */
	count += scnprintf(buf + count, size - count,
			   "\n= sample_id_ctrl set help =\n");
	count += scnprintf(buf + count, size - count, "Usage:\n");
	count += scnprintf(buf + count, size - count,
			   "echo [ip_range] [ip_index] [define_index] [mask] "
			   "[id] [ev0] [ev1] [ev2] [ev3] [ev4] [ev5] [ev6] "
			   "[ev7] > sample_id_ctrl\n");
	count += scnprintf(buf + count, size - count,
				"\nip_range: BCM_EACH(%d), BCM_ALL(%d)\n",
				BCM_EACH, BCM_ALL);
	count += scnprintf(buf + count, size - count,
				"ip_index: number of bcm ip (0 ~ %u)\n"
				"          (if ip_range is all, set to 0)\n",
			   data->bcm_ip_nr - 1);
	count += scnprintf(buf + count, size - count,
				"define_index: index of pre-defined event (0 ~ %u)\n"
				"              0 means no pre-defined event\n",
			   data->define_event_max - 1);
	count += scnprintf(buf + count, size - count,
				"mask: masking for sample id (if define_index is not 0, set to 0)\n"
				"      mask value should be in hex\n");
	count += scnprintf(buf + count, size - count,
				"id: id of sample id (if define_index is not 0, set to 0)\n"
				"    id should be in hex\n");
	count += scnprintf(buf + count, size - count,
				"evX: event counter enable for sample id (if define_index is not 0, set to 0)\n"
				"     value should be 0 or 1\n");

	ret = simple_read_from_buffer(ubuf, size, ppos, buf, count);
	kfree(buf);

	return ret;
}

static ssize_t store_sample_id_ctrl(struct file *fp, const char __user *ubuf, size_t size,
		loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	struct exynos_bcm_ipc_base_info ipc_base_info;
	struct exynos_bcm_sample_id sample_id;
	unsigned int bcm_ip_index, ip_range;
	unsigned int defined_index;
	unsigned int peak_mask, peak_id;
	unsigned int peak_enable[BCM_EVT_EVENT_MAX];
	int ev_cnt, ret;

	if (*ppos != 0)
		return size;

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	simple_write_to_buffer(buf, size, ppos, ubuf, size);

	ret = sscanf(buf, "%u %u %u %x %x %u %u %u %u %u %u %u %u",
			&ip_range, &bcm_ip_index, &defined_index,
			&peak_mask, &peak_id,
			&peak_enable[0], &peak_enable[1], &peak_enable[2],
			&peak_enable[3], &peak_enable[4], &peak_enable[5],
			&peak_enable[6], &peak_enable[7]);
	kfree(buf);
	if (ret != 13) {
		BCM_ERR("%s: Invalid command string\n", __func__);
		return -EINVAL;
	}

	ret = exynos_bcm_ip_validate(ip_range, bcm_ip_index, data->bcm_ip_nr);
	if (ret)
		return ret;

	for (ev_cnt = 0; ev_cnt < BCM_EVT_EVENT_MAX; ev_cnt++) {
		if (peak_enable[ev_cnt])
			peak_enable[ev_cnt] = true;
	}

	if (defined_index >= data->define_event_max) {
		BCM_ERR("%s: Invalid defined index(%u),"
			" defined_max_nr(%u)\n", __func__,
			defined_index, data->define_event_max - 1);
		return -EINVAL;
	}

	if (ip_range == BCM_ALL)
		bcm_ip_index = 0;

	if (defined_index != NO_PRE_DEFINE_EVT) {
		sample_id.peak_mask =
			data->define_sample_id[defined_index].peak_mask;
		sample_id.peak_id =
			data->define_sample_id[defined_index].peak_id;
		for (ev_cnt = 0; ev_cnt < BCM_EVT_EVENT_MAX; ev_cnt++)
			sample_id.peak_enable[ev_cnt] =
				data->define_sample_id[defined_index].peak_enable[ev_cnt];
	} else {
		sample_id.peak_mask = peak_mask;
		sample_id.peak_id = peak_id;
		for (ev_cnt = 0; ev_cnt < BCM_EVT_EVENT_MAX; ev_cnt++)
			sample_id.peak_enable[ev_cnt] = peak_enable[ev_cnt];
	}

	exynos_bcm_dbg_set_base_info(&ipc_base_info, BCM_EVT_EVENT_SAMPLE_ID,
					BCM_EVT_SET, ip_range);

	ret = exynos_bcm_dbg_sample_id_ctrl(&ipc_base_info, &sample_id,
						bcm_ip_index, data);
	if (ret) {
		BCM_ERR("%s:failed set sample ID\n", __func__);
		return ret;
	}

	return size;
}

static ssize_t show_run_ctrl(struct file *fp, char __user *ubuf, size_t size, loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	struct exynos_bcm_ipc_base_info ipc_base_info;
	unsigned int bcm_run;
	ssize_t count = 0;
	int ret;

	if (*ppos > 0)
		return 0;

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	exynos_bcm_dbg_set_base_info(&ipc_base_info, BCM_EVT_RUN_CONT,
					BCM_EVT_GET, 0);

	ret = exynos_bcm_dbg_run_ctrl(&ipc_base_info, &bcm_run, data);
	if (ret)
		count += scnprintf(buf + count, size - count, "failed get run state\n");
	else
		count += scnprintf(buf + count, size - count,
				"run state: raw state(%s), sw state(%s)\n",
				bcm_run ? "run" : "stop",
				data->bcm_run_state ? "run" : "stop");

	ret = simple_read_from_buffer(ubuf, size, ppos, buf, count);
	kfree(buf);

	return ret;
}

static ssize_t show_run_ctrl_help(struct file *fp, char __user *ubuf, size_t size, loff_t *ppos)
{
	char *buf;
	ssize_t ret;
	ssize_t count = 0;

	if (*ppos > 0)
		return 0;

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	/* help show_run_ctrl */
	count += scnprintf(buf + count, size - count,
				"\n= run_ctrl get help =\n");
	count += scnprintf(buf + count, size - count, "Usage:\n");
	count += scnprintf(buf + count, size - count,
				"cat run_ctrl\n"
				"run state: raw state([run_state]), sw state([run_state])\n");

	/* help store_run_ctrl */
	count += scnprintf(buf + count, size - count,
			   "\n= run_ctrl set help =\n");
	count += scnprintf(buf + count, size - count, "Usage:\n");
	count += scnprintf(buf + count, size - count,
			   "echo [run_state] > run_ctrl\n");
	count += scnprintf(buf + count, size - count,
				"\nrun_state: BCM_RUN(%d), BCM_STOP(%d)\n",
				BCM_RUN, BCM_STOP);

	ret = simple_read_from_buffer(ubuf, size, ppos, buf, count);
	kfree(buf);

	return ret;
}

static ssize_t store_run_ctrl(struct file *fp, const char __user *ubuf, size_t size, loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	unsigned int bcm_run;
	int ret;

	if (*ppos != 0)
		return size;

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	simple_write_to_buffer(buf, size, ppos, ubuf, size);

	ret = kstrtouint(buf, 0, &bcm_run);
	kfree(buf);
	if (ret) {
		BCM_ERR("%s: Invalid command string\n", __func__);
		return ret;
	}

	if (!(bcm_run == 0 || bcm_run == 1)) {
		BCM_ERR("%s: invalid parameter (%u)\n", __func__, bcm_run);
		return -EINVAL;
	}

	if (bcm_run)
		bcm_run = true;

	ret = exynos_bcm_dbg_run(bcm_run, data);
	if (ret) {
		BCM_ERR("%s:failed set Run state\n", __func__);
		return ret;
	}

	return size;
}

static ssize_t show_period_ctrl(struct file *fp, char __user *ubuf, size_t size, loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	struct exynos_bcm_ipc_base_info ipc_base_info;
	unsigned int period;
	ssize_t count = 0;
	int ret;

	if (*ppos > 0)
		return 0;

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	exynos_bcm_dbg_set_base_info(&ipc_base_info, BCM_EVT_PERIOD_CONT,
					BCM_EVT_GET, 0);

	ret = exynos_bcm_dbg_period_ctrl(&ipc_base_info, &period, data);
	if (ret)
		count += scnprintf(buf + count, size - count, "failed get period\n");
	else
		count += scnprintf(buf + count, size - count, "monitor period: %u usec\n", period);

	ret = simple_read_from_buffer(ubuf, size, ppos, buf, count);
	kfree(buf);

	return ret;
}

static ssize_t show_period_ctrl_help(struct file *fp, char __user *ubuf, size_t size, loff_t *ppos)
{
	char *buf;
	ssize_t ret;
	ssize_t count = 0;

	if (*ppos > 0)
		return 0;

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	/* help show_period_ctrl */
	count += scnprintf(buf + count, size - count,
				"\n= period_ctrl get help =\n");
	count += scnprintf(buf + count, size - count, "Usage:\n");
	count += scnprintf(buf + count, size - count,
				"cat period_ctrl\n"
				"monitor period: [period] usec\n");

	/* help store_period_ctrl */
	count += scnprintf(buf + count, size - count,
			   "\n= period_ctrl set help =\n");
	count += scnprintf(buf + count, size - count, "Usage:\n");
	count += scnprintf(buf + count, size - count,
			   "echo [period] > period_ctrl\n");
	count += scnprintf(buf + count, size - count,
				"\nperiod: monitor period (unit: usec),\n"
				"          min(%d usec) ~ max(%d usec)\n",
			   BCM_TIMER_PERIOD_MIN, BCM_TIMER_PERIOD_MAX);

	ret = simple_read_from_buffer(ubuf, size, ppos, buf, count);
	kfree(buf);

	return ret;
}

static ssize_t store_period_ctrl(struct file *fp, const char __user *ubuf, size_t size,
		loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	struct exynos_bcm_ipc_base_info ipc_base_info;
	unsigned int period;
	int ret;

	if (*ppos != 0)
		return size;

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	simple_write_to_buffer(buf, size, ppos, ubuf, size);

	ret = kstrtouint(buf, 0, &period);
	kfree(buf);
	if (ret) {
		BCM_ERR("%s: Invalid command string\n", __func__);
		return ret;
	}

	exynos_bcm_dbg_set_base_info(&ipc_base_info, BCM_EVT_PERIOD_CONT,
					BCM_EVT_SET, 0);

	ret = exynos_bcm_dbg_period_ctrl(&ipc_base_info, &period, data);
	if (ret) {
		BCM_ERR("%s:failed set period\n", __func__);
		return ret;
	}

	return size;
}

static ssize_t show_mode_ctrl(struct file *fp, char __user *ubuf, size_t size, loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	struct exynos_bcm_ipc_base_info ipc_base_info;
	unsigned int bcm_mode;
	ssize_t count = 0;
	int ret;

	if (*ppos > 0)
		return 0;

	exynos_bcm_dbg_set_base_info(&ipc_base_info, BCM_EVT_MODE_CONT,
					BCM_EVT_GET, 0);

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	ret = exynos_bcm_dbg_mode_ctrl(&ipc_base_info, &bcm_mode, data);
	if (ret)
		count += scnprintf(buf + count, size - count, "failed get mode\n");
	else
		count += scnprintf(buf + count, size - count,
			   "mode: %d (%d:Interval, %d:Once, %d:User_ctrl, %d:Accumulator)\n",
			   bcm_mode, BCM_MODE_INTERVAL, BCM_MODE_ONCE,
			   BCM_MODE_USERCTRL, BCM_MODE_ACCUMULATOR);

	ret = simple_read_from_buffer(ubuf, size, ppos, buf, count);
	kfree(buf);

	return ret;
}

static ssize_t show_mode_ctrl_help(struct file *fp, char __user *ubuf, size_t size, loff_t *ppos)
{
	char *buf;
	ssize_t ret;
	ssize_t count = 0;

	if (*ppos > 0)
		return 0;

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	/* help show_mode_ctrl */
	count += scnprintf(buf + count, size - count,
				"\n= mode_ctrl get help =\n");
	count += scnprintf(buf + count, size - count, "Usage:\n");
	count += scnprintf(buf + count, size - count,
			"cat mode_ctrl\n"
			"mode: [mode] (%d:Interval, %d:Once, %d:User_ctrl, %d:Accumulator)\n",
			BCM_MODE_INTERVAL, BCM_MODE_ONCE, BCM_MODE_USERCTRL,
			BCM_MODE_ACCUMULATOR);

	/* help store_mode_ctrl */
	count += scnprintf(buf + count, size - count,
			   "\n= mode_ctrl set help =\n");
	count += scnprintf(buf + count, size - count, "Usage:\n");
	count += scnprintf(buf + count, size - count,
			   "echo [mode] > mode_ctrl\n");
	count += scnprintf(buf + count, size - count,
			   "\nmode: %d:Interval, %d:Once, %d:User_ctrl, %d:Accumulator\n",
			   BCM_MODE_INTERVAL, BCM_MODE_ONCE, BCM_MODE_USERCTRL,
			   BCM_MODE_ACCUMULATOR);

	ret = simple_read_from_buffer(ubuf, size, ppos, buf, count);
	kfree(buf);

	return ret;
}

static ssize_t store_mode_ctrl(struct file *fp, const char __user *ubuf, size_t size, loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	struct exynos_bcm_ipc_base_info ipc_base_info;
	unsigned int bcm_mode;
	int ret;

	if (*ppos != 0)
		return size;

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	simple_write_to_buffer(buf, size, ppos, ubuf, size);

	ret = kstrtouint(buf, 0, &bcm_mode);
	kfree(buf);
	if (ret) {
		BCM_ERR("%s: Invalid command string\n", __func__);
		return ret;
	}

	exynos_bcm_dbg_set_base_info(&ipc_base_info, BCM_EVT_MODE_CONT,
					BCM_EVT_SET, 0);

	ret = exynos_bcm_dbg_mode_ctrl(&ipc_base_info, &bcm_mode, data);
	if (ret) {
		BCM_ERR("%s:failed set mode\n", __func__);
		return ret;
	}

	return size;
}

static ssize_t show_str_ctrl(struct file *fp, char __user *ubuf, size_t size, loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	struct exynos_bcm_ipc_base_info ipc_base_info;
	unsigned int suspend;
	ssize_t count = 0;
	int ret;

	if (*ppos > 0)
		return 0;

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	exynos_bcm_dbg_set_base_info(&ipc_base_info, BCM_EVT_STR_STATE,
					BCM_EVT_GET, 0);

	ret = exynos_bcm_dbg_str_ctrl(&ipc_base_info, &suspend, data);
	if (ret)
		count += scnprintf(buf + count, size - count, "failed get str state\n");
	else
		count += scnprintf(buf + count, size - count,
			   "str state: %s\n", suspend ? "suspend" : "resume");

	ret = simple_read_from_buffer(ubuf, size, ppos, buf, count);
	kfree(buf);

	return ret;
}

static ssize_t show_str_ctrl_help(struct file *fp, char __user *ubuf, size_t size, loff_t *ppos)
{
	char *buf;
	ssize_t ret;
	ssize_t count = 0;

	if (*ppos > 0)
		return 0;

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	/* help show_str_ctrl */
	count += scnprintf(buf + count, size - count,
				"\n= str_ctrl get help =\n");
	count += scnprintf(buf + count, size - count, "Usage:\n");
	count += scnprintf(buf + count, size - count,
				"cat str_ctrl\n"
				"str state: [str_state]\n");

	/* help store_str_ctrl */
	count += scnprintf(buf + count, size - count,
			   "\n= str_ctrl set help =\n");
	count += scnprintf(buf + count, size - count, "Usage:\n");
	count += scnprintf(buf + count, size - count,
			   "echo [str_state] > str_ctrl\n");
	count += scnprintf(buf + count, size - count,
				"\nstr_state: suspend(1), resume(0)\n");

	ret = simple_read_from_buffer(ubuf, size, ppos, buf, count);
	kfree(buf);

	return ret;
}

static ssize_t store_str_ctrl(struct file *fp, const char __user *ubuf, size_t size, loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	unsigned int suspend;
	int ret;

	if (*ppos != 0)
		return size;

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	simple_write_to_buffer(buf, size, ppos, ubuf, size);

	ret = kstrtouint(buf, 0, &suspend);
	kfree(buf);
	if (ret) {
		BCM_ERR("%s: Invalid command string\n", __func__);
		return ret;
	}

	if (suspend)
		suspend = true;

	ret = exynos_bcm_dbg_str(suspend, data);
	if (ret) {
		BCM_ERR("%s:failed set str state\n", __func__);
		return ret;
	}

	return size;
}

static ssize_t show_ip_ctrl(struct file *fp, char __user *ubuf, size_t size, loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	struct exynos_bcm_ipc_base_info ipc_base_info;
	unsigned int ip_enable;
	ssize_t count = 0;
	int ret;
	static int ip_cnt;

	if (ip_cnt >= data->bcm_ip_nr) {
		ip_cnt = 0;
		return 0;
	}

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	exynos_bcm_dbg_set_base_info(&ipc_base_info, BCM_EVT_IP_CONT,
					BCM_EVT_GET, BCM_EACH);

	do {
		ret = exynos_bcm_dbg_ip_ctrl(&ipc_base_info,
						&ip_enable, ip_cnt, data);
		if (ret) {
			BCM_ERR("%s: failed get ip_enable state(ip:%d)\n",
				__func__, ip_cnt);
			ip_cnt = 0;
			goto out;
		}

		count += scnprintf(buf + count, size - count,
				   "bcm[%2d]: enabled (%s)\n",
				   ip_cnt, ip_enable ? "true" : "false");
		ip_cnt++;
	} while ((ip_cnt < data->bcm_ip_nr) &&
		 (ip_cnt % data->bcm_ip_print_nr));

	*ppos = 0;
	ret = simple_read_from_buffer(ubuf, size, ppos, buf, count);
out:
	kfree(buf);

	return ret;
}

static ssize_t show_ip_ctrl_help(struct file *fp, char __user *ubuf, size_t size, loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	ssize_t ret;
	ssize_t count = 0;

	if (*ppos > 0)
		return 0;

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	/* help show_ip_ctrl */
	count += scnprintf(buf + count, size - count,
				"\n= ip_ctrl get help =\n");
	count += scnprintf(buf + count, size - count, "Usage:\n");
	count += scnprintf(buf + count, size - count,
				"cat ip_ctrl\n"
				"bcm[ip_index]: enabled ([enable])\n");

	/* help store_ip_ctrl */
	count += scnprintf(buf + count, size - count,
				"\n= ip_ctrl set help =\n");
	count += scnprintf(buf + count, size - count, "Usage:\n");
	count += scnprintf(buf + count, size - count,
				"echo [ip_index] [enable] > ip_ctrl\n");
	count += scnprintf(buf + count, size - count,
				"\nip_index: number of bcm ip (0 ~ %u)\n",
			   data->bcm_ip_nr - 1);
	count += scnprintf(buf + count, size - count,
				"enable: ip enable state (1:enable, 0:disable)\n");

	ret = simple_read_from_buffer(ubuf, size, ppos, buf, count);
	kfree(buf);

	return ret;
}

static ssize_t store_ip_ctrl(struct file *fp, const char __user *ubuf, size_t size, loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	struct exynos_bcm_ipc_base_info ipc_base_info;
	unsigned int bcm_ip_index, ip_enable;
	int ret;

	if (*ppos != 0)
		return size;

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	simple_write_to_buffer(buf, size, ppos, ubuf, size);

	ret = sscanf(buf, "%u %u", &bcm_ip_index, &ip_enable);
	kfree(buf);
	if (ret != 2) {
		BCM_ERR("%s: Invalid command string\n", __func__);
		return -EINVAL;
	}

	ret = exynos_bcm_ip_validate(BCM_EACH, bcm_ip_index, data->bcm_ip_nr);
	if (ret)
		return ret;

	if (ip_enable)
		ip_enable = true;

	exynos_bcm_dbg_set_base_info(&ipc_base_info, BCM_EVT_IP_CONT,
					BCM_EVT_SET, BCM_EACH);

	ret = exynos_bcm_dbg_ip_ctrl(&ipc_base_info, &ip_enable,
						bcm_ip_index, data);
	if (ret) {
		BCM_ERR("%s:failed set IP control\n", __func__);
		return ret;
	}

	return size;
}

#if IS_ENABLED(CONFIG_DEBUG_SNAPSHOT)
static int exynos_bcm_dbg_set_dump_info(struct exynos_bcm_dbg_data *data)
{
	struct exynos_bcm_ipc_base_info ipc_base_info;
	int ret;

	if (data->dump_addr.buff_size == 0 ||
		data->dump_addr.buff_size > data->dump_addr.p_size)
		data->dump_addr.buff_size = data->dump_addr.p_size;

	BCM_INFO("%s: virtual address for reserved memory: v_addr = 0x%p\n",
		 __func__, data->dump_addr.v_addr);
	BCM_INFO("%s: buffer size for reserved memory: buff_size = 0x%x\n",
		 __func__, data->dump_addr.buff_size);

	/* send physical address info to BCM plugin */
	exynos_bcm_dbg_set_base_info(&ipc_base_info, BCM_EVT_DUMP_ADDR,
					BCM_EVT_SET, 0);

	ret = exynos_bcm_dbg_dump_addr_ctrl(&ipc_base_info, &data->dump_addr,
					    data);
	if (ret) {
		BCM_ERR("%s: failed set dump address info\n", __func__);
		return ret;
	}

	return 0;
}

static ssize_t show_dump_addr_info(struct file *fp, char __user *ubuf, size_t size, loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	ssize_t ret;
	ssize_t count = 0;

	if (*ppos > 0)
		return 0;

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	count += scnprintf(buf + count, size - count,
			   "\n= BCM dump address info =\n");
	count += scnprintf(buf + count, size - count,
			   "physical address = 0x%08x\n",
			   data->dump_addr.p_addr);
	count += scnprintf(buf + count, size - count,
			   "virtual address = 0x%p\n",
			   data->dump_addr.v_addr);
	count += scnprintf(buf + count, size - count,
			   "dump region size = 0x%08x\n",
			   data->dump_addr.p_size);
	count += scnprintf(buf + count, size - count,
			   "actual use size = 0x%08x\n",
			   data->dump_addr.buff_size);

	ret = simple_read_from_buffer(ubuf, size, ppos, buf, count);
	kfree(buf);

	return ret;
}

static ssize_t store_dump_addr_info(struct file *fp, const char __user *ubuf, size_t size,
		loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	unsigned int buff_size;
	int ret;

	if (*ppos != 0)
		return size;

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	simple_write_to_buffer(buf, size, ppos, ubuf, size);

	ret = kstrtouint(buf, 16, &buff_size);
	kfree(buf);
	if (ret) {
		BCM_ERR("%s: Invalid command string\n", __func__);
		return ret;
	}

	data->dump_addr.buff_size = buff_size;

	ret = exynos_bcm_dbg_set_dump_info(data);
	if (ret) {
		BCM_ERR("%s: failed set dump info\n", __func__);
		return ret;
	}

	return size;
}
#endif

#if IS_ENABLED(CONFIG_EXYNOS_BCM_DBG_DUMP)
static ssize_t show_bcm_dump(struct file *fp, char __user *ubuf, size_t size, loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	ssize_t ret;

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	ret = exynos_bcm_dbg_dump(data, buf, size, *ppos);
	if (ret < 0) {
		BCM_ERR("%s: failed to complete\n", __func__);
		goto out;
	}

	if (copy_to_user(ubuf, buf, ret)) {
		ret = -EFAULT;
		goto out;
	}
	*ppos += ret;
out:
	kfree(buf);

	return ret;
}
#endif

static ssize_t show_enable_dump_klog(struct file *fp, char __user *ubuf, size_t size, loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	ssize_t ret;
	ssize_t count = 0;

	if (*ppos > 0)
		return 0;

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	count += scnprintf(buf + count, size - count,
			   "\n= BCM dump to kernel log =\n");
	count += scnprintf(buf + count, size - count, "%s\n",
			   data->dump_klog ? "enabled" : "disabled");

	ret = simple_read_from_buffer(ubuf, size, ppos, buf, count);
	kfree(buf);

	return ret;
}

static ssize_t store_enable_dump_klog(struct file *fp, const char __user *ubuf, size_t size,
		loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	unsigned int enable;
	int ret;

	if (*ppos != 0)
		return size;

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	simple_write_to_buffer(buf, size, ppos, ubuf, size);

	ret = kstrtouint(buf, 0, &enable);
	kfree(buf);
	if (ret) {
		BCM_ERR("%s: Invalid command string\n", __func__);
		return ret;
	}

	if (enable)
		data->dump_klog = true;
	else
		data->dump_klog = false;

	return size;
}

#if IS_ENABLED(CONFIG_EXYNOS_BCM_DBG_DUMP_FILE)
static ssize_t show_enable_dump_file(struct file *fp, char __user *ubuf, size_t size, loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	ssize_t ret;
	ssize_t count = 0;

	if (*ppos > 0)
		return 0;

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	count += scnprintf(buf + count, size - count,
			   "\n= BCM dump to file =\n");
	count += scnprintf(buf + count, size - count, "%s\n",
			   data->dump_file ? "enabled" : "disabled");

	ret = simple_read_from_buffer(ubuf, size, ppos, buf, count);
	kfree(buf);

	return ret;
}

static ssize_t store_enable_dump_file(struct file *fp, const char __user *ubuf, size_t size,
		loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	unsigned int enable;
	int ret;

	if (*ppos != 0)
		return size;

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	simple_write_to_buffer(buf, size, ppos, ubuf, size);

	ret = kstrtouint(buf, 0, &enable);
	kfree(buf);
	if (ret) {
		BCM_ERR("%s: Invalid command string\n", __func__);
		return ret;
	}

	data->dump_file = enable;

	return size;
}
#endif

static ssize_t show_enable_stop_owner(struct file *fp, char __user *ubuf, size_t size, loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	ssize_t ret;
	ssize_t count = 0;
	int i;

	if (*ppos > 0)
		return 0;

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	count += scnprintf(buf + count, size - count,
			   "\n= BCM Available stop owner =\n");
	for (i = 0; i < STOP_OWNER_MAX; i++)
		count += scnprintf(buf + count, size - count,
				   " stop owner[%d]: %s\n",
				   i, data->available_stop_owner[i] ?
				   "true" : "false");

	ret = simple_read_from_buffer(ubuf, size, ppos, buf, count);
	kfree(buf);

	return ret;
}

static ssize_t store_enable_stop_owner(struct file *fp, const char __user *ubuf, size_t size,
		loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	unsigned int owner_index, enable;
	int ret;

	if (*ppos != 0)
		return size;

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	simple_write_to_buffer(buf, size, ppos, ubuf, size);

	ret = sscanf(buf, "%u %u", &owner_index, &enable);
	kfree(buf);
	if (ret) {
		BCM_ERR("%s: Invalid command string\n", __func__);
		return ret;
	}

	if (owner_index >= STOP_OWNER_MAX) {
		BCM_ERR("Invalid stop owner (%u)\n", owner_index);
		return -EINVAL;
	}

	if (enable)
		data->available_stop_owner[owner_index] = true;
	else
		data->available_stop_owner[owner_index] = false;

	return size;
}

static ssize_t show_dump_accumulators(struct file *fp, char __user *ubuf, size_t size, loff_t *ppos)
{
	struct exynos_bcm_dbg_data *data = fp->private_data;
	char *buf;
	struct exynos_bcm_ipc_base_info ipc_base_info;
	ssize_t count = 0;
	int ret;

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	exynos_bcm_dbg_set_base_info(&ipc_base_info, BCM_EVT_DUMP_ACCUMULATORS,
					BCM_EVT_SET, BCM_EACH);

	ret = exynos_bcm_dbg_dump_accumulators_ctrl(&ipc_base_info, buf,
						    &count, *ppos, size, data);
	if (ret) {
		BCM_ERR("%s: failed to dump accumulators\n", __func__);
		goto out;
	}

	ret = simple_read_from_buffer(ubuf, size, ppos, buf, count);
out:
	kfree(buf);

	return ret;
}

static ssize_t show_dump_accumulators_help(struct file *fp, char __user *ubuf, size_t size,
		loff_t *ppos)
{
	char *buf;
	ssize_t ret;
	ssize_t count = 0;

	if (*ppos > 0)
		return 0;

	buf = kmalloc(size, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	/* help show_dump_accumulators_help */
	count += scnprintf(buf + count, size - count,
				"\n= dump_accumulators get help =\n");
	count += scnprintf(buf + count, size - count, "Usage:\n");
	count += scnprintf(buf + count, size - count, "cat dump_accumulators\n");
	count += scnprintf(buf + count, size - count,
			"[seq_no], [ip_index], [define_event], [time], [ccnt], [pmcnt0], ");
	count += scnprintf(buf + count, size - count,
			"[pmcnt1], [pmcnt2], [pmcnt3], [pmcnt4], [pmcnt5], [pmcnt6], [pmcnt7]\n");

	ret = simple_read_from_buffer(ubuf, size, ppos, buf, count);
	kfree(buf);

	return ret;
}

#if IS_ENABLED(CONFIG_DEBUG_SNAPSHOT)
static int exynos_bcm_dbg_dump_config(struct exynos_bcm_dbg_data *data)
{
	int ret;

	data->dump_addr.p_addr = dbg_snapshot_get_item_paddr(BCM_DSS_NAME);
	data->dump_addr.p_size = dbg_snapshot_get_item_size(BCM_DSS_NAME);
	data->dump_addr.v_addr =
		(void __iomem *)dbg_snapshot_get_item_vaddr(BCM_DSS_NAME);

	ret = exynos_bcm_dbg_set_dump_info(data);
	if (ret) {
		BCM_ERR("%s: failed set dump info\n", __func__);
		return ret;
	}

	return 0;
}
#endif

#if IS_ENABLED(CONFIG_EXYNOS_ITMON)
static int exynos_bcm_dbg_itmon_notifier(struct notifier_block *nb,
					unsigned long val, void *v)
{
	struct itmon_notifier *itmon_info = (struct itmon_notifier *)v;

	BCM_INFO("%s: itmon error code %u\n", __func__, itmon_info->errcode);

	if (itmon_info->errcode == ERRCODE_ITMON_TIMEOUT) {
		BCM_INFO("%s: Note: It can occurred be IPC timeout "
			 "because can be trying access to timeout block "
			 "from BCMDBG plugin\n", __func__);
		exynos_bcm_dbg_stop(ITMON_HANDLE);
	}

	return NOTIFY_OK;
}
#endif

static int exynos_bcm_dbg_pm_suspend(struct device *dev)
{
	unsigned int suspend = true;
	int ret;

	ret = exynos_bcm_dbg_str(suspend, bcm_dbg_data);
	if (ret) {
		BCM_ERR("%s: failed set str state\n", __func__);
		return ret;
	}

	return 0;
}

static int exynos_bcm_dbg_pm_resume(struct device *dev)
{
	unsigned int suspend = false;
	int ret;

	ret = exynos_bcm_dbg_str(suspend, bcm_dbg_data);
	if (ret) {
		BCM_ERR("%s: failed set str state\n", __func__);
		return ret;
	}

	return 0;
}

static struct dev_pm_ops exynos_bcm_dbg_pm_ops = {
	.suspend	= exynos_bcm_dbg_pm_suspend,
	.resume		= exynos_bcm_dbg_pm_resume,
};

static struct bcm_file_entry bcm_dbg_file_entries[] = {
#if IS_ENABLED(CONFIG_EXYNOS_BCM_DBG_DUMP)
	BCM_FILE_ENTRY_RO(bcm_dump),
#endif
	BCM_FILE_ENTRY_RO(boot_config),
	BCM_FILE_ENTRY_RO(dump_accumulators),
	BCM_FILE_ENTRY_RO(dump_accumulators_help),
#if IS_ENABLED(CONFIG_DEBUG_SNAPSHOT)
	BCM_FILE_ENTRY_WR(dump_addr_info),
#endif
#if IS_ENABLED(CONFIG_EXYNOS_BCM_DBG_DUMP_FILE)
	BCM_FILE_ENTRY_WR(enable_dump_file),
#endif
	BCM_FILE_ENTRY_WR(enable_dump_klog),
	BCM_FILE_ENTRY_WR(enable_stop_owner),
	BCM_FILE_ENTRY_WR(event_ctrl),
	BCM_FILE_ENTRY_RO(event_ctrl_help),
	BCM_FILE_ENTRY_RO(filter_id_active),
	BCM_FILE_ENTRY_WR(filter_id_ctrl),
	BCM_FILE_ENTRY_RO(filter_id_ctrl_help),
	BCM_FILE_ENTRY_RO(filter_others_active),
	BCM_FILE_ENTRY_WR(filter_others_ctrl),
	BCM_FILE_ENTRY_RO(filter_others_ctrl_help),
	BCM_FILE_ENTRY_WR(event_sm_user_ctrl),
	BCM_FILE_ENTRY_RO(event_sm_user_ctrl_help),
	BCM_FILE_ENTRY_WR(ip_ctrl),
	BCM_FILE_ENTRY_RO(ip_ctrl_help),
	BCM_FILE_ENTRY_RO(ip_power_domains),
	BCM_FILE_ENTRY_WR(mode_ctrl),
	BCM_FILE_ENTRY_RO(mode_ctrl_help),
	BCM_FILE_ENTRY_WR(period_ctrl),
	BCM_FILE_ENTRY_RO(period_ctrl_help),
	BCM_FILE_ENTRY_RO(ppmu_ver),
	BCM_FILE_ENTRY_RO(predefined_events),
	BCM_FILE_ENTRY_RO(predefined_filters),
	BCM_FILE_ENTRY_RO(predefined_sample_mask),
	BCM_FILE_ENTRY_WR(run_ctrl),
	BCM_FILE_ENTRY_RO(run_ctrl_help),
	BCM_FILE_ENTRY_RO(sample_id_active),
	BCM_FILE_ENTRY_WR(sample_id_ctrl),
	BCM_FILE_ENTRY_RO(sample_id_ctrl_help),
	BCM_FILE_ENTRY_WR(str_ctrl),
	BCM_FILE_ENTRY_RO(str_ctrl_help),
};

static void exynos_bcm_dbg_init_debugfs(struct exynos_bcm_dbg_data *data)
{
	int f, num_files = ARRAY_SIZE(bcm_dbg_file_entries);
	struct dentry *bcm_dbg_dentry;

	bcm_dbg_file_fops = kzalloc(num_files * sizeof(struct file_operations), GFP_KERNEL);
	if (bcm_dbg_file_fops == NULL)
		return;

	bcm_dbg_dentry = debugfs_create_dir("bcm_attr", NULL);

	for (f = 0; f < num_files; f++) {
		bcm_dbg_file_fops[f].open = simple_open;
		bcm_dbg_file_fops[f].read = bcm_dbg_file_entries[f].read;
		bcm_dbg_file_fops[f].write = bcm_dbg_file_entries[f].write;
		bcm_dbg_file_fops[f].llseek = default_llseek;
		debugfs_create_file(bcm_dbg_file_entries[f].fname, bcm_dbg_file_entries[f].mode,
				bcm_dbg_dentry, data, &bcm_dbg_file_fops[f]);
	}
}

static int __init exynos_bcm_dbg_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct exynos_bcm_dbg_data *data;

	data = kzalloc(sizeof(struct exynos_bcm_dbg_data), GFP_KERNEL);
	if (data == NULL) {
		ret = -ENOMEM;
		goto err_data;
	}

	bcm_dbg_data = data;
	data->dev = &pdev->dev;

	spin_lock_init(&data->lock);

	/* parsing dts data for BCM debug */
	ret = exynos_bcm_dbg_parse_dt(data->dev->of_node, data);
	if (ret) {
		BCM_ERR("%s: failed to parse private data\n", __func__);
		goto err_parse_dt;
	}

	/* Request IPC channel */
	ret = exynos_bcm_dbg_ipc_channel_request(data);
	if (ret) {
		BCM_ERR("%s: failed to ipc channel request\n", __func__);
		goto err_ipc_channel;
	}

	/* Initalize BCM Plugin */
	ret = exynos_bcm_dbg_early_init(data);
	if (ret) {
		BCM_ERR("%s: failed to early bcm initialize\n", __func__);
		goto err_early_init;
	}

	/* initial Local Power Domain sync-up */
	ret = exynos_bcm_dbg_pd_sync_init(data);
	if (ret) {
		BCM_ERR("%s: failed to pd_sync_init\n", __func__);
		goto err_pd_sync_init;
	}

#if IS_ENABLED(CONFIG_DEBUG_SNAPSHOT)
	ret = exynos_bcm_dbg_dump_config(data);
	if (ret) {
		BCM_ERR("%s: failed to dump config\n", __func__);
		goto err_dump_config;
	}
#endif
	data->dump_klog = false;
#if IS_ENABLED(CONFIG_EXYNOS_BCM_DBG_DUMP_FILE)
	data->dump_file = true;  /* for backward compatibility */
#else
	data->dump_file = false;
#endif

	/* BCM plugin run */
	if (data->initial_bcm_run) {
		ret = exynos_bcm_dbg_run(data->initial_bcm_run, data);
		if (ret) {
			BCM_ERR("%s: failed to bcm initial run\n", __func__);
			goto err_initial_run;
		}
	}

	platform_set_drvdata(pdev, data);
	exynos_bcm_dbg_init_debugfs(data);

#if IS_ENABLED(CONFIG_EXYNOS_ITMON)
	data->itmon_notifier.notifier_call = exynos_bcm_dbg_itmon_notifier;
	itmon_notifier_chain_register(&data->itmon_notifier);
#endif

#if IS_ENABLED(CONFIG_EXYNOS_BCM_DBG_PPMU)
	ret = exynos_bcm_dbg_ppmu_init(pdev);
	if (ret)
		BCM_ERR("%s: failed to initialize Platform PMU\n", __func__);
#endif

	BCM_INFO("%s: exynos bcm is initialized!!\n", __func__);

	return 0;

err_initial_run:
#if IS_ENABLED(CONFIG_DEBUG_SNAPSHOT)
err_dump_config:
#endif
	exynos_bcm_dbg_pd_sync_exit(data);
err_pd_sync_init:
err_early_init:
	exynos_bcm_dbg_ipc_channel_release(data);
err_ipc_channel:
err_parse_dt:
	kfree(data);
	data = NULL;
	bcm_dbg_data = NULL;
err_data:
	return ret;
}

static int exynos_bcm_dbg_remove(struct platform_device *pdev)
{
	struct exynos_bcm_dbg_data *data =
					platform_get_drvdata(pdev);
	int ret;

	debugfs_remove_recursive(debugfs_lookup("bcm_attr", NULL));
	kfree(bcm_dbg_file_fops);
	platform_set_drvdata(pdev, NULL);
	ret = exynos_bcm_dbg_pd_sync_exit(data);
	if (ret) {
		BCM_ERR("%s: failed to pd_sync_exit\n", __func__);
		return ret;
	}

	exynos_bcm_dbg_ipc_channel_release(data);
	kfree(data);
	data = NULL;

#if IS_ENABLED(CONFIG_EXYNOS_BCM_DBG_PPMU)
	exynos_bcm_dbg_ppmu_exit(pdev);
#endif

	BCM_INFO("%s: exynos bcm is removed!!\n", __func__);

	return 0;
}

static struct platform_device_id exynos_bcm_dbg_driver_ids[] = {
	{ .name = EXYNOS_BCM_DBG_MODULE_NAME, },
	{},
};
MODULE_DEVICE_TABLE(platform, exynos_bcm_dbg_driver_ids);

static const struct of_device_id exynos_bcm_dbg_match[] = {
	{ .compatible = "samsung,exynos-bcm_dbg", },
	{},
};

static struct platform_driver exynos_bcm_dbg_driver = {
	.remove = exynos_bcm_dbg_remove,
	.id_table = exynos_bcm_dbg_driver_ids,
	.driver = {
		.name = EXYNOS_BCM_DBG_MODULE_NAME,
		.owner = THIS_MODULE,
		.pm = &exynos_bcm_dbg_pm_ops,
		.of_match_table = exynos_bcm_dbg_match,
	},
};

module_platform_driver_probe(exynos_bcm_dbg_driver, exynos_bcm_dbg_probe);

MODULE_AUTHOR("Taekki Kim <taekki.kim@samsung.com>");
MODULE_DESCRIPTION("Samsung BCM Debug driver");
MODULE_LICENSE("GPL");
