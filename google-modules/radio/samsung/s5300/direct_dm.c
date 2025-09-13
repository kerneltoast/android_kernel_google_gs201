// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 Samsung Electronics.
 *
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/usb/f_dm.h>

#include "direct_dm.h"
#include "modem_utils.h"

static struct direct_dm_ctrl *_dc;

static void direct_dm_rx_func(unsigned long arg);

/* RX timer */
static inline void direct_dm_start_rx_timer(struct direct_dm_ctrl *dc,
		struct hrtimer *timer)
{
	unsigned long flags;

	if (!dc) {
		mif_err_limited("dc is null\n");
		return;
	}

	if (!dc->use_rx_timer) {
		mif_err_limited("use_rx_timer is not set\n");
		return;
	}

	spin_lock_irqsave(&dc->rx_timer_lock, flags);
	if (!hrtimer_is_queued(timer)) {
		ktime_t ktime = ktime_set(0, dc->rx_timer_period_msec * NSEC_PER_MSEC);

		dc->stat.rx_timer_req++;
		hrtimer_start(timer, ktime, HRTIMER_MODE_REL);
	}
	spin_unlock_irqrestore(&dc->rx_timer_lock, flags);
}

static enum hrtimer_restart direct_dm_rx_timer(struct hrtimer *timer)
{
	mif_info("run rx func by timer\n");

	_dc->stat.rx_timer_expire++;

	if (_dc->use_rx_task)
		tasklet_hi_schedule(&_dc->rx_task);
	else
		direct_dm_rx_func((unsigned long)_dc);

	return HRTIMER_NORESTART;
}

/* RX func */
static int direct_dm_send_to_upper_layer(struct direct_dm_ctrl *dc,
	struct direct_dm_desc *curr_desc, void *addr)
{
	struct sk_buff *skb = NULL;
	struct mem_link_device *mld;
	int ch_id = 0;

	if (!dc || !curr_desc || !addr) {
		mif_err_limited("null addr\n");
		return -ENOMEM;
	}

	dc->stat.upper_layer_req_cnt++;
	skb = dev_alloc_skb(curr_desc->length);
	if (unlikely(!skb)) {
		mif_err_limited("mem_alloc_skb() error pos:%d\n",
			dc->curr_desc_pos);
		dc->stat.err_upper_layer_req++;
		return -ENOMEM;
	}
	skb_put(skb, curr_desc->length);
	skb_copy_to_linear_data(skb, addr, curr_desc->length);

	switch (dc->ld->protocol) {
	case PROTOCOL_SIPC:
		ch_id = SIPC_CH_ID_CPLOG1;
		break;
	case PROTOCOL_SIT:
		ch_id = EXYNOS_CH_ID_CPLOG;
		break;
	default:
		mif_err_limited("protocol error:%d\n", dc->ld->protocol);
		return -EINVAL;
	}

	skbpriv(skb)->lnk_hdr = 0;
	skbpriv(skb)->sipc_ch = ch_id;
	skbpriv(skb)->iod = link_get_iod_with_channel(dc->ld, ch_id);
	skbpriv(skb)->ld = dc->ld;
	skbpriv(skb)->napi = NULL;

	mld = to_mem_link_device(dc->ld);
	mld->pass_skb_to_demux(mld, skb);

	dc->desc_rgn[dc->curr_desc_pos].status &= ~BIT(DDM_DESC_S_DONE);

	dc->curr_desc_pos = circ_new_ptr(dc->num_desc,
		dc->curr_desc_pos, 1);
	dc->curr_done_pos = circ_new_ptr(dc->num_desc,
		dc->curr_done_pos, 1);

	return 0;
}

static int direct_dm_send_to_usb(struct direct_dm_ctrl *dc,
	struct direct_dm_desc *curr_desc, void *addr)
{
	int ret = 0;

	if (!dc || !curr_desc || !addr) {
		mif_err_limited("null addr\n");
		return -ENOMEM;
	}

	dc->stat.usb_req_cnt++;
	ret = usb_dm_request(addr, curr_desc->length);
	if (ret) {
		mif_info_limited("usb_dm_request() ret:%d pos:%d\n",
			ret, dc->curr_desc_pos);

		dc->usb_req_failed = true;
		dc->stat.err_usb_req++;
		if (dc->use_rx_timer) {
			if (unlikely(dc->enable_debug))
				mif_info("start timer\n");

			direct_dm_start_rx_timer(dc, &dc->rx_timer);
		}

		return ret;
	}
	dc->usb_req_failed = false;

	dc->curr_desc_pos = circ_new_ptr(dc->num_desc,
		dc->curr_desc_pos, 1);

	return 0;
}

static void direct_dm_rx_func(unsigned long arg)
{
	struct direct_dm_ctrl *dc = (struct direct_dm_ctrl *)arg;
	unsigned long paddr;
	void *addr;
	int ret;
	int i;
	bool upper_layer_req = false;
	int rcvd = 0;
	unsigned long flags;

	if (!dc) {
		mif_err_limited("dc is null\n");
		return;
	}

	spin_lock_irqsave(&dc->rx_lock, flags);

	upper_layer_req = dc->info_rgn->silent_log;
	if (!upper_layer_req && !dc->usb_active) {
		mif_info_limited("usb is not activated\n");
		spin_unlock_irqrestore(&dc->rx_lock, flags);
		return;
	}

	for (i = 0; i < dc->num_desc; i++) {
		struct direct_dm_desc curr_desc = dc->desc_rgn[dc->curr_desc_pos];

		if (!(curr_desc.status & BIT(DDM_DESC_S_DONE))) {
			if (unlikely(dc->enable_debug))
				mif_info("DDM_DESC_S_DONE is not set %d 0x%llx\n",
					 dc->curr_desc_pos, curr_desc.cp_buff_paddr);

			break;
		}

		if (curr_desc.status & BIT(DDM_DESC_S_TOUT)) {
			mif_err_limited("DDM_DESC_S_TOUT is set %d 0x%llx\n",
					dc->curr_desc_pos, curr_desc.cp_buff_paddr);
			dc->stat.err_desc_tout++;
		}

		/* TODO: support compressed log of DM log mover H/W */
		if (curr_desc.status & BIT(DDM_DESC_S_COMPRESSED))
			mif_err_limited("DDM_DESC_S_COMPRESSED is set %d 0x%llx\n",
					dc->curr_desc_pos, curr_desc.cp_buff_paddr);

		if (!curr_desc.length || (curr_desc.length > dc->max_packet_size)) {
			mif_err_limited("length error:%d\n", curr_desc.length);
			dc->stat.err_length++;
			break;
		}

		paddr = curr_desc.cp_buff_paddr -
			dc->buff_rgn_offset - dc->cp_ddm_pbase + dc->buff_pbase;
		addr = phys_to_virt(paddr);

		if (dc->buff_rgn_cached && !dc->hw_iocc)
			dma_sync_single_for_cpu(dc->dev, paddr,
				dc->max_packet_size, DMA_FROM_DEVICE);

		if (unlikely(dc->enable_debug))
			mif_info("pos:%d len:%d a:%pK/0x%lx/0x%llx upper:%d done:%d\n",
				 dc->curr_desc_pos, curr_desc.length,
				 addr, paddr, curr_desc.cp_buff_paddr,
				 upper_layer_req, dc->desc_rgn[dc->curr_desc_pos].status);

		if (unlikely(upper_layer_req)) {
			ret = direct_dm_send_to_upper_layer(dc, &curr_desc, addr);
			if (ret)
				break;

			rcvd++;
		} else {
			ret = direct_dm_send_to_usb(dc, &curr_desc, addr);
			if (ret)
				break;

			rcvd++;

			if (dc->curr_desc_pos == dc->curr_done_pos) {
				if (unlikely(dc->enable_debug))
					mif_info("prev desc is enqueued:%d\n",
						dc->curr_done_pos);
				break;
			}
		}
	}

	if (unlikely(dc->enable_debug))
		mif_info("rcvd:%d\n", rcvd);

	spin_unlock_irqrestore(&dc->rx_lock, flags);
}

static void direct_dm_run_rx_func(struct direct_dm_ctrl *dc)
{
	if (!dc) {
		mif_err_limited("dc is null\n");
		return;
	}

	if (dc->use_rx_timer && hrtimer_active(&dc->rx_timer))
		hrtimer_cancel(&dc->rx_timer);

	if (dc->use_rx_task)
		tasklet_hi_schedule(&dc->rx_task);
	else
		direct_dm_rx_func((unsigned long)dc);
}

/* IRQ handler */
static irqreturn_t direct_dm_irq_handler(int irq, void *arg)
{
	struct direct_dm_ctrl *dc = (struct direct_dm_ctrl *)arg;

	if (!dc) {
		mif_err_limited("dc is null\n");
		return IRQ_HANDLED;
	}

	direct_dm_run_rx_func(dc);

	return IRQ_HANDLED;
}

/* Callback from USB driver */
static void direct_dm_usb_active_noti(void *arg)
{
	struct direct_dm_ctrl *dc = (struct direct_dm_ctrl *)arg;
	unsigned long flags;

	if (!dc) {
		mif_err_limited("dc is null\n");
		return;
	}

	spin_lock_irqsave(&dc->rx_lock, flags);

	mif_info("usb is activated\n");
	dc->usb_active = true;

	spin_unlock_irqrestore(&dc->rx_lock, flags);

	if (dc->usb_req_failed) {
		mif_info("run rx func\n");
		direct_dm_run_rx_func(dc);
	}
}

static void direct_dm_usb_disable_noti(void *arg)
{
	struct direct_dm_ctrl *dc = (struct direct_dm_ctrl *)arg;
	unsigned long flags;

	if (!dc) {
		mif_err_limited("dc is null\n");
		return;
	}

	spin_lock_irqsave(&dc->rx_lock, flags);

	mif_info("usb is deactivated\n");
	dc->usb_active = false;
	dc->usb_req_failed = true;

	spin_unlock_irqrestore(&dc->rx_lock, flags);

	if (dc->use_rx_timer && hrtimer_active(&dc->rx_timer)) {
		mif_info("cancel rx timer\n");
		hrtimer_cancel(&dc->rx_timer);
	}
}

static void direct_dm_usb_completion_noti(void *addr, int length, void *arg)
{
	struct direct_dm_ctrl *dc = (struct direct_dm_ctrl *)arg;
	unsigned long paddr;
	u32 pos;
	unsigned long flags;

	if (!dc) {
		mif_err_limited("dc is null\n");
		return;
	}

	dc->stat.usb_complete_cnt++;

	paddr = virt_to_phys(addr);
	if ((paddr < dc->buff_pbase) ||
		(paddr >= (dc->buff_pbase + dc->buff_rgn_size))) {
		mif_err("addr error:%pK 0x%lx 0x%lx 0x%x\n",
			addr, paddr, dc->buff_pbase, dc->buff_rgn_offset);
		dc->stat.err_usb_complete++;
		return;
	}

	if (dc->buff_rgn_cached && !dc->hw_iocc)
		dma_sync_single_for_device(dc->dev, paddr,
			dc->max_packet_size, DMA_FROM_DEVICE);

	spin_lock_irqsave(&dc->rx_lock, flags);
	pos = (paddr - dc->buff_pbase) / dc->max_packet_size;
	dc->desc_rgn[pos].status &= ~BIT(DDM_DESC_S_DONE);

	if (dc->curr_done_pos != pos)
		mif_err("pos error! pos:%d done:%d len:%d a:%pK/0x%lx\n",
			pos, dc->curr_done_pos, length, addr, paddr);

	if ((length <= 0) || (length > dc->max_packet_size)) {
		mif_err_limited("length error:%d\n", length);
		dc->usb_req_failed = true;
	}

	if (unlikely(dc->enable_debug))
		mif_info("pos:%d done:%d len:%d a:%pK/0x%lx\n",
			 pos, dc->curr_done_pos, length, addr, paddr);

	dc->curr_done_pos = circ_new_ptr(dc->num_desc,
		dc->curr_done_pos, 1);

	spin_unlock_irqrestore(&dc->rx_lock, flags);
}

/* sysfs */
static ssize_t ctrl_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct direct_dm_ctrl *dc = _dc;
	ssize_t count = 0;

	if (!dc) {
		mif_err("dc is null\n");
		return count;
	}

	count += scnprintf(&buf[count], PAGE_SIZE - count,
		"version:%d shm_rgn_index:%d\n",
		dc->version, dc->shm_rgn_index);
	count += scnprintf(&buf[count], PAGE_SIZE - count,
		"hw_iocc:%d info_desc_rgn_cached:%d buff_rgn_cached:%d\n",
		dc->hw_iocc, dc->info_desc_rgn_cached, dc->buff_rgn_cached);
	count += scnprintf(&buf[count], PAGE_SIZE - count,
		"info_rgn_offset:0x%08x info_rgn_size:0x%08x\n",
		dc->info_rgn_offset, dc->info_rgn_size);
	count += scnprintf(&buf[count], PAGE_SIZE - count,
		"desc_rgn_offset:0x%08x desc_rgn_size:0x%08x num_desc:%d\n",
		dc->desc_rgn_offset, dc->desc_rgn_size, dc->num_desc);
	count += scnprintf(&buf[count], PAGE_SIZE - count,
		"buff_rgn_offset:0x%08x buff_rgn_size:0x%08x\n",
		dc->buff_rgn_offset, dc->buff_rgn_size);
	count += scnprintf(&buf[count], PAGE_SIZE - count,
		"max_packet_size:%d usb_req_num:%d irq_index:%d\n",
		dc->max_packet_size, dc->usb_req_num, dc->irq_index);
	count += scnprintf(&buf[count], PAGE_SIZE - count,
		"cp_ddm_pbase:0x%08x\n", dc->cp_ddm_pbase);
	count += scnprintf(&buf[count], PAGE_SIZE - count,
		"curr_desc_pos:%d\n", dc->curr_desc_pos);
	count += scnprintf(&buf[count], PAGE_SIZE - count,
		"curr_done_pos:%d\n", dc->curr_done_pos);
	count += scnprintf(&buf[count], PAGE_SIZE - count,
		"use_rx_task:%d\n", dc->use_rx_task);
	count += scnprintf(&buf[count], PAGE_SIZE - count,
		"use_rx_timer:%d\n", dc->use_rx_timer);
	count += scnprintf(&buf[count], PAGE_SIZE - count,
		"rx_timer_period_msec:%d\n", dc->rx_timer_period_msec);
	count += scnprintf(&buf[count], PAGE_SIZE - count,
		"enable_debug:%d\n", dc->enable_debug);
	count += scnprintf(&buf[count], PAGE_SIZE - count,
		"usb_req_failed:%d\n", dc->usb_req_failed);

	return count;
}

static ssize_t stat_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct direct_dm_ctrl *dc = _dc;
	ssize_t count = 0;

	if (!dc) {
		mif_err("dc is null\n");
		return count;
	}

	count += scnprintf(&buf[count], PAGE_SIZE - count,
		"err_desc_tout:%lld\n", dc->stat.err_desc_tout);
	count += scnprintf(&buf[count], PAGE_SIZE - count,
		"err_length:%lld\n", dc->stat.err_length);

	count += scnprintf(&buf[count], PAGE_SIZE - count, "\n");
	count += scnprintf(&buf[count], PAGE_SIZE - count,
		"usb_req_cnt:%lld\n", dc->stat.usb_req_cnt);
	count += scnprintf(&buf[count], PAGE_SIZE - count,
		"err_usb_req:%lld\n", dc->stat.err_usb_req);
	count += scnprintf(&buf[count], PAGE_SIZE - count,
		"usb_complete_cnt:%lld\n", dc->stat.usb_complete_cnt);
	count += scnprintf(&buf[count], PAGE_SIZE - count,
		"err_usb_complete:%lld\n", dc->stat.err_usb_complete);

	count += scnprintf(&buf[count], PAGE_SIZE - count, "\n");
	count += scnprintf(&buf[count], PAGE_SIZE - count,
		"upper_layer_req_cnt:%lld\n", dc->stat.upper_layer_req_cnt);
	count += scnprintf(&buf[count], PAGE_SIZE - count,
		"err_upper_layer_req:%lld\n", dc->stat.err_upper_layer_req);

	count += scnprintf(&buf[count], PAGE_SIZE - count, "\n");
	count += scnprintf(&buf[count], PAGE_SIZE - count,
		"rx_timer_req:%lld\n", dc->stat.rx_timer_req);
	count += scnprintf(&buf[count], PAGE_SIZE - count,
		"rx_timer_expire:%lld\n", dc->stat.rx_timer_expire);

	return count;
}

static ssize_t info_rgn_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct direct_dm_ctrl *dc = _dc;
	ssize_t count = 0;

	if (!dc) {
		mif_err("dc is null\n");
		return count;
	}

	count += scnprintf(&buf[count], PAGE_SIZE - count,
		"version:%d max_packet_size:%d\n",
		dc->info_rgn->version, dc->info_rgn->max_packet_size);
	count += scnprintf(&buf[count], PAGE_SIZE - count,
		"silent_log:%d\n",
		dc->info_rgn->silent_log);
	count += scnprintf(&buf[count], PAGE_SIZE - count,
		"cp_desc_pbase:0x%llx cp_buff_pbase:0x%llx\n",
		dc->info_rgn->cp_desc_pbase, dc->info_rgn->cp_buff_pbase);

	return count;
}

static ssize_t desc_rgn_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct direct_dm_ctrl *dc = _dc;
	ssize_t count = 0;
	int i;

	if (!dc) {
		mif_err("dc is null\n");
		return count;
	}

	count += scnprintf(&buf[count], PAGE_SIZE - count,
		"curr_desc_pos:%d\n", dc->curr_desc_pos);
	count += scnprintf(&buf[count], PAGE_SIZE - count,
		"curr_done_pos:%d\n", dc->curr_done_pos);

	for (i = 0; i < dc->num_desc; i++) {
		count += scnprintf(&buf[count], PAGE_SIZE - count,
			"%d 0x%llx 0x%x 0x%x %d\n",
			i, dc->desc_rgn[i].cp_buff_paddr, dc->desc_rgn[i].control,
			dc->desc_rgn[i].status, dc->desc_rgn[i].length);
	}

	return count;
}

static ssize_t enable_debug_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct direct_dm_ctrl *dc = _dc;
	int val;
	int ret;

	if (!dc) {
		mif_err("dc is null\n");
		return count;
	}

	ret = kstrtoint(buf, 0, &val);
	if (ret) {
		mif_err("kstrtoint() error:%d\n", ret);
		return ret;
	}

	val ? (dc->enable_debug = true) : (dc->enable_debug = false);
	mif_info("enable_debug:%d\n", dc->enable_debug);

	return count;
}

static ssize_t enable_debug_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct direct_dm_ctrl *dc = _dc;
	ssize_t count = 0;

	if (!dc) {
		mif_err("dc is null\n");
		return count;
	}

	count += scnprintf(&buf[count], PAGE_SIZE - count,
		"enable_debug:%d\n",
		dc->enable_debug);

	return count;
}

static ssize_t use_rx_task_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct direct_dm_ctrl *dc = _dc;
	int val;
	int ret;

	if (!dc) {
		mif_err("dc is null\n");
		return count;
	}

	ret = kstrtoint(buf, 0, &val);
	if (ret) {
		mif_err("kstrtoint() error:%d\n", ret);
		return ret;
	}

	val ? (dc->use_rx_task = true) : (dc->use_rx_task = false);
	mif_info("use_rx_task:%d\n", dc->use_rx_task);

	return count;
}

static ssize_t use_rx_task_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct direct_dm_ctrl *dc = _dc;
	ssize_t count = 0;

	if (!dc) {
		mif_err("dc is null\n");
		return count;
	}

	count += scnprintf(&buf[count], PAGE_SIZE - count,
		"use_rx_task:%d\n",
		dc->use_rx_task);

	return count;
}

static ssize_t use_rx_timer_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct direct_dm_ctrl *dc = _dc;
	int val;
	int ret;

	if (!dc) {
		mif_err("dc is null\n");
		return count;
	}

	ret = kstrtoint(buf, 0, &val);
	if (ret) {
		mif_err("kstrtoint() error:%d\n", ret);
		return ret;
	}

	val ? (dc->use_rx_timer = true) : (dc->use_rx_timer = false);
	mif_info("use_rx_timer:%d\n", dc->use_rx_timer);

	return count;
}

static ssize_t use_rx_timer_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct direct_dm_ctrl *dc = _dc;
	ssize_t count = 0;

	if (!dc) {
		mif_err("dc is null\n");
		return count;
	}

	count += scnprintf(&buf[count], PAGE_SIZE - count,
		"use_rx_timer:%d\n",
		dc->use_rx_timer);

	return count;
}

static ssize_t rx_timer_period_msec_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct direct_dm_ctrl *dc = _dc;
	int val;
	int ret;

	if (!dc) {
		mif_err("dc is null\n");
		return count;
	}

	if (!dc->use_rx_timer) {
		mif_err("use_rx_timer is not set\n");
		return count;
	}

	ret = kstrtoint(buf, 0, &val);
	if (ret) {
		mif_err("kstrtoint() error:%d\n", ret);
		return ret;
	}

	dc->rx_timer_period_msec = val;
	mif_info("rx_timer_period_msec:%d\n", dc->rx_timer_period_msec);

	return count;
}

static ssize_t rx_timer_period_msec_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct direct_dm_ctrl *dc = _dc;
	ssize_t count = 0;

	if (!dc) {
		mif_err("dc is null\n");
		return count;
	}

	if (!dc->use_rx_timer) {
		mif_err("use_rx_timer is not set\n");
		return count;
	}

	count += scnprintf(&buf[count], PAGE_SIZE - count,
		"rx_timer_period_msec:%d\n",
		dc->rx_timer_period_msec);

	return count;
}

static u32 _test_dm_desc_pos;
static ssize_t test_dm_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct direct_dm_ctrl *dc = _dc;
	void *addr;
	int val;
	int ret;
	int i;

	if (!dc) {
		mif_err("dc is null\n");
		return count;
	}

	ret = kstrtoint(buf, 0, &val);
	if (ret) {
		mif_err("kstrtoint() error:%d\n", ret);
		return ret;
	}

	if (!val || (val > dc->num_desc)) {
		mif_err("val error:%d\n", val);
		return count;
	}

	for (i = 0; i < val; i++) {
		addr = dc->desc_rgn[_test_dm_desc_pos].cp_buff_paddr -
			dc->buff_rgn_offset - dc->cp_ddm_pbase + dc->buff_vbase;

		mif_info("pos:%d a:%pK/0x%llx\n",
			 _test_dm_desc_pos, addr,
			 dc->desc_rgn[_test_dm_desc_pos].cp_buff_paddr);

		if (dc->desc_rgn[_test_dm_desc_pos].status & BIT(DDM_DESC_S_DONE)) {
			mif_err("DDM_DESC_S_DONE is already set. pos:%d\n",
				_test_dm_desc_pos);
			break;
		}

		memset(addr, _test_dm_desc_pos, dc->max_packet_size);
		dc->desc_rgn[_test_dm_desc_pos].length = dc->max_packet_size;
		dc->desc_rgn[_test_dm_desc_pos].status |= BIT(DDM_DESC_S_DONE);

		if (dc->desc_rgn[_test_dm_desc_pos].control & BIT(DDM_DESC_C_END))
			_test_dm_desc_pos = 0;
		else
			_test_dm_desc_pos++;
	}

	direct_dm_irq_handler(dc->irq_index, dc);

	return count;
}

static ssize_t test_silent_log_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct direct_dm_ctrl *dc = _dc;
	int val;
	int ret;

	if (!dc) {
		mif_err("dc is null\n");
		return count;
	}

	ret = kstrtoint(buf, 0, &val);
	if (ret) {
		mif_err("kstrtoint() error:%d\n", ret);
		return ret;
	}

	val ? (dc->info_rgn->silent_log = 1) :
		(dc->info_rgn->silent_log = 0);
	mif_info("silent_log:%d\n", dc->info_rgn->silent_log);

	return count;
}

static DEVICE_ATTR_RO(ctrl_status);
static DEVICE_ATTR_RO(stat);
static DEVICE_ATTR_RO(info_rgn);
static DEVICE_ATTR_RO(desc_rgn);
static DEVICE_ATTR_RW(enable_debug);
static DEVICE_ATTR_RW(use_rx_task);
static DEVICE_ATTR_RW(use_rx_timer);
static DEVICE_ATTR_RW(rx_timer_period_msec);
static DEVICE_ATTR_WO(test_dm);
static DEVICE_ATTR_WO(test_silent_log);

static struct attribute *direct_dm_attrs[] = {
	&dev_attr_ctrl_status.attr,
	&dev_attr_stat.attr,
	&dev_attr_info_rgn.attr,
	&dev_attr_desc_rgn.attr,
	&dev_attr_enable_debug.attr,
	&dev_attr_use_rx_task.attr,
	&dev_attr_use_rx_timer.attr,
	&dev_attr_rx_timer_period_msec.attr,
	&dev_attr_test_dm.attr,
	&dev_attr_test_silent_log.attr,
	NULL,
};
ATTRIBUTE_GROUPS(direct_dm);

/* Initialize */
int direct_dm_init(struct link_device *ld)
{
	struct direct_dm_ctrl *dc = _dc;
	int i;

	if (!dc) {
		mif_err("dc is null\n");
		return -EPERM;
	}

	dc->ld = ld;

	dc->info_rgn->version = dc->version;
	dc->info_rgn->max_packet_size = dc->max_packet_size;
	dc->info_rgn->cp_desc_pbase = dc->cp_ddm_pbase + dc->desc_rgn_offset;
	dc->info_rgn->cp_buff_pbase = dc->cp_ddm_pbase + dc->buff_rgn_offset;
	mif_info("version:%d max_packet_size:%d\n",
		 dc->info_rgn->version, dc->info_rgn->max_packet_size);
	mif_info("cp_desc_pbase:0x%llx cp_buff_pbase:0x%llx\n",
		 dc->info_rgn->cp_desc_pbase, dc->info_rgn->cp_buff_pbase);

	memset(dc->desc_vbase, 0, dc->desc_rgn_size);
	for (i = 0; i < dc->num_desc; i++) {
		dc->desc_rgn[i].cp_buff_paddr = dc->cp_ddm_pbase +
			dc->buff_rgn_offset + (dc->max_packet_size * i);

		dc->desc_rgn[i].control |= BIT(DDM_DESC_C_INT);

		if (i == (dc->num_desc - 1))
			dc->desc_rgn[i].control |= BIT(DDM_DESC_C_END);
	}

	if (unlikely(dc->enable_debug)) {
		for (i = 0; i < dc->num_desc; i++) {
			mif_info("%d a:0x%llx c:0x%x s:0x%x l:%d\n",
				 i, dc->desc_rgn[i].cp_buff_paddr, dc->desc_rgn[i].control,
				 dc->desc_rgn[i].status, dc->desc_rgn[i].length);
		}
	}

	memset(dc->buff_vbase, 0, dc->buff_rgn_size);

	dc->curr_desc_pos = 0;
	dc->curr_done_pos = 0;
	_test_dm_desc_pos = 0;

	dc->usb_req_failed = false;

	memset(&dc->stat, 0, sizeof(dc->stat));

	return 0;
}
EXPORT_SYMBOL(direct_dm_init);

int direct_dm_deinit(void)
{
	return 0;
}
EXPORT_SYMBOL(direct_dm_deinit);

/* Create */
static int direct_dm_setup_region(struct direct_dm_ctrl *dc)
{
	unsigned long cp_pbase;
	unsigned long ddm_pbase;
	u32 ddm_rgn_size;

	if (!dc) {
		mif_err("dc is null\n");
		return -EPERM;
	}

	cp_pbase = cp_shmem_get_base(0, SHMEM_CP);
	if (!cp_pbase) {
		mif_err("cp_pbase is null\n");
		return -ENOMEM;
	}
	ddm_pbase = cp_shmem_get_base(0, dc->shm_rgn_index);
	if (!ddm_pbase) {
		mif_err("ddm_pbase is null\n");
		return -ENOMEM;
	}
	ddm_rgn_size = cp_shmem_get_size(0, dc->shm_rgn_index);
	if (!ddm_rgn_size) {
		mif_err("ddm_rgn_size is null\n");
		return -ENOMEM;
	}
	mif_info("cp_pbase:0x%lx ddm_pbase:0x%lx ddm_rgn_size:0x%08x\n",
		cp_pbase, ddm_pbase, ddm_rgn_size);

	/* TODO: support slim modem */
	dc->cp_ddm_pbase = ddm_pbase - cp_pbase + CP_CPU_BASE_ADDRESS;
	mif_info("cp_ddm_pbase:0x%08x\n", dc->cp_ddm_pbase);

	if (dc->info_desc_rgn_cached) {
		dc->info_vbase = phys_to_virt(ddm_pbase + dc->info_rgn_offset);
		dc->desc_vbase = phys_to_virt(ddm_pbase + dc->desc_rgn_offset);
		if (!dc->hw_iocc) {
			mif_err("cached region is not supported yet without hw_iocc\n");
			return -EINVAL;
		}
	} else {
		dc->info_vbase = cp_shmem_get_nc_region(
			ddm_pbase + dc->info_rgn_offset,
			dc->info_rgn_size + dc->desc_rgn_size);
		if (!dc->info_vbase) {
			mif_err("dc->info_base error\n");
			return -ENOMEM;
		}
		dc->desc_vbase = dc->info_vbase + dc->info_rgn_size;
	}
	memset(dc->info_vbase, 0, dc->info_rgn_size + dc->desc_rgn_size);
	mif_info("info_rgn_size:0x%08x desc_rgn_size:0x%08x\n",
		dc->info_rgn_size, dc->desc_rgn_size);

	dc->buff_rgn_size = ddm_rgn_size -
		(dc->info_rgn_size + dc->desc_rgn_size);
	dc->buff_pbase = ddm_pbase + dc->buff_rgn_offset;
	if (dc->buff_rgn_cached) {
		dc->buff_vbase = phys_to_virt(dc->buff_pbase);
	} else {
		dc->buff_vbase = cp_shmem_get_nc_region(
			dc->buff_pbase, dc->buff_rgn_size);
		if (!dc->buff_vbase) {
			mif_err("dc->buff_vbase error\n");
			return -ENOMEM;
		}
	}
	memset(dc->buff_vbase, 0, dc->buff_rgn_size);
	dc->num_desc = dc->buff_rgn_size / dc->max_packet_size;
	if (dc->buff_rgn_cached && !dc->hw_iocc)
		dma_sync_single_for_device(dc->dev, dc->buff_pbase,
			dc->buff_rgn_size, DMA_FROM_DEVICE);
	mif_info("buff_rgn_size:0x%08x num_desc:%d\n",
		dc->buff_rgn_size, dc->num_desc);

	dc->info_rgn = (struct direct_dm_info_rgn *)dc->info_vbase;
	dc->desc_rgn = (struct direct_dm_desc *)dc->desc_vbase;

	return 0;
}

static int direct_dm_register_irq(struct direct_dm_ctrl *dc)
{
	int ret;

	ret = cp_mbox_register_handler(dc->irq_index, 0,
		direct_dm_irq_handler, dc);
	if (ret) {
		mif_err("cp_mbox_register_handler() error:%d\n", ret);
		goto error;
	}

error:
	return ret;
}

static int direct_dm_read_dt(struct device_node *np,
	struct direct_dm_ctrl *dc)
{
	mif_dt_read_u32(np, "version", dc->version);
	mif_dt_read_u32(np, "shm_rgn_index", dc->shm_rgn_index);
	mif_dt_read_u32(np, "hw_iocc", dc->hw_iocc);
	mif_dt_read_u32(np, "info_desc_rgn_cached", dc->info_desc_rgn_cached);
	mif_dt_read_u32(np, "buff_rgn_cached", dc->buff_rgn_cached);

	mif_dt_read_u32(np, "info_rgn_offset", dc->info_rgn_offset);
	mif_dt_read_u32(np, "info_rgn_size", dc->info_rgn_size);
	mif_dt_read_u32(np, "desc_rgn_offset", dc->desc_rgn_offset);
	mif_dt_read_u32(np, "desc_rgn_size", dc->desc_rgn_size);
	mif_dt_read_u32(np, "buff_rgn_offset", dc->buff_rgn_offset);

	mif_dt_read_u32(np, "max_packet_size", dc->max_packet_size);
	mif_dt_read_u32(np, "usb_req_num", dc->usb_req_num);
	mif_dt_read_u32(np, "irq_index", dc->irq_index);

	mif_dt_read_bool(np, "use_rx_task", dc->use_rx_task);
	mif_dt_read_bool(np, "use_rx_timer", dc->use_rx_timer);
	mif_dt_read_u32(np, "rx_timer_period_msec", dc->rx_timer_period_msec);

	mif_info("version:%d shm_rgn_index:%d\n",
		dc->version, dc->shm_rgn_index);
	mif_info("hw_iocc:%d info_desc_rgn_cached:%d buff_rgn_cached:%d\n",
		dc->hw_iocc, dc->info_desc_rgn_cached, dc->buff_rgn_cached);

	mif_info("info_rgn_offset:0x%08x info_rgn_size:0x%08x\n",
		dc->info_rgn_offset, dc->info_rgn_size);
	mif_info("desc_rgn_offset:0x%08x desc_rgn_size:0x%08x\n",
		dc->desc_rgn_offset, dc->desc_rgn_size);
	mif_info("buff_rgn_offset:0x%08x\n", dc->buff_rgn_offset);

	mif_info("max_packet_size:%d usb_req_num:%d irq_index:%d\n",
		dc->max_packet_size, dc->usb_req_num, dc->irq_index);
	mif_info("use_rx_timer:%d rx_timer_period_msec:%d\n",
		dc->use_rx_timer, dc->rx_timer_period_msec);


	return 0;
}

int direct_dm_create(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct direct_dm_ctrl *dc = NULL;
	int ret;

	mif_info("+++\n");

	if (!np) {
		mif_err("of_node is null\n");
		ret = -EINVAL;
		goto err;
	}

	_dc = devm_kzalloc(dev, sizeof(struct direct_dm_ctrl), GFP_KERNEL);
	if (!_dc) {
		mif_err_limited("devm_kzalloc() error\n");
		ret = -ENOMEM;
		goto err;
	}
	dc = _dc;

	dc->dev = dev;

	ret = direct_dm_read_dt(np, dc);
	if (ret) {
		mif_err("direct_dm_read_dt() error:%d\n", ret);
		goto err_setup;
	}

	dma_set_mask_and_coherent(dev, DMA_BIT_MASK(36));

	ret = direct_dm_register_irq(dc);
	if (ret) {
		mif_err("direct_dm_register_irq() error:%d\n", ret);
		goto err_setup;
	}

	ret = direct_dm_setup_region(dc);
	if (ret) {
		mif_err("direct_dm_setup_region() error:%d\n", ret);
		goto err_setup;
	}

	ret = init_dm_direct_path(dc->usb_req_num,
		direct_dm_usb_completion_noti,
		direct_dm_usb_active_noti,
		direct_dm_usb_disable_noti,
		(void *)dc);
	if (ret) {
		mif_err("init_dm_direct_path() error:%d\n", ret);
		goto err_setup;
	}

	spin_lock_init(&dc->rx_lock);

	tasklet_init(&dc->rx_task, direct_dm_rx_func, (unsigned long)dc);

	spin_lock_init(&dc->rx_timer_lock);
	hrtimer_init(&dc->rx_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	dc->rx_timer.function = direct_dm_rx_timer;

	dev_set_drvdata(dev, dc);

	ret = sysfs_create_groups(&dev->kobj, direct_dm_groups);
	if (ret != 0) {
		mif_err("sysfs_create_group() error:%d\n", ret);
		goto err_setup;
	}

	mif_info("---\n");

	return 0;

err_setup:
	devm_kfree(dev, dc);
	dc = NULL;

err:
	panic("Direct DM driver probe failed\n");
	mif_err("xxx\n");

	return ret;
}
EXPORT_SYMBOL(direct_dm_create);

/* Platform driver */
static int direct_dm_probe(struct platform_device *pdev)
{
	return direct_dm_create(pdev);
}

static int direct_dm_remove(struct platform_device *pdev)
{
	return 0;
}

static int direct_dm_suspend(struct device *dev)
{
	return 0;
}

static int direct_dm_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops direct_dm_pm_ops = {
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(direct_dm_suspend, direct_dm_resume)
};

static const struct of_device_id direct_dm_dt_match[] = {
	{ .compatible = "samsung,cpif-direct-dm", },
	{},
};
MODULE_DEVICE_TABLE(of, direct_dm_dt_match);

static struct platform_driver direct_dm_driver = {
	.probe = direct_dm_probe,
	.remove = direct_dm_remove,
	.driver = {
		.name = "direct_dm",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(direct_dm_dt_match),
		.pm = &direct_dm_pm_ops,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(direct_dm_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Samsung direct DM driver");
