// SPDX-License-Identifier: GPL-2.0
/* linux/drivers/modem/modem.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2010 Samsung Electronics.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/if_arp.h>
#include <linux/device.h>

#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/mfd/syscon.h>
#include <linux/of_reserved_mem.h>
#include <linux/dma-map-ops.h>
#include <uapi/linux/in.h>
#include <linux/inet.h>
#include <net/ipv6.h>
#include <soc/google/exynos-modem-ctrl.h>
#include <soc/google/modem_notifier.h>

#if IS_ENABLED(CONFIG_S5910)
#include <linux/s5910.h>
#endif

#if IS_ENABLED(CONFIG_LINK_DEVICE_SHMEM)
#include <linux/shm_ipc.h>
#include "mcu_ipc.h"
#endif

#include "exynos-modem-ctrl.h"
#include "modem_prj.h"
#include "modem_variation.h"
#include "modem_utils.h"
#include "modem_toe_device.h"

#if IS_ENABLED(CONFIG_MODEM_IF_LEGACY_QOS)
#include "cpif_qos_info.h"
#endif

#define FMT_WAKE_TIME   (msecs_to_jiffies(300))
#define RAW_WAKE_TIME   (HZ*6)
#define NET_WAKE_TIME	(HZ/2)

static struct modem_shared *create_modem_shared_data(
				struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct modem_shared *msd;
	int size = MAX_MIF_BUFF_SIZE;

	msd = devm_kzalloc(dev, sizeof(struct modem_shared), GFP_KERNEL);
	if (!msd)
		return NULL;

	/* initialize link device list */
	INIT_LIST_HEAD(&msd->link_dev_list);
	INIT_LIST_HEAD(&msd->activated_ndev_list);

	/* initialize tree of io devices */
	msd->iodevs_tree_fmt = RB_ROOT;

	msd->storage.cnt = 0;
	msd->storage.addr = devm_kcalloc(dev, MAX_MIF_BUFF_SIZE +
		(MAX_MIF_SEPA_SIZE * 2), sizeof(*msd->storage.addr), GFP_KERNEL);
	if (!msd->storage.addr) {
		mif_err("IPC logger buff alloc failed!!\n");
		devm_kfree(dev, msd);
		return NULL;
	}
	memset(msd->storage.addr, 0, size + (MAX_MIF_SEPA_SIZE * 2));
	memcpy(msd->storage.addr, MIF_SEPARATOR, strlen(MIF_SEPARATOR));
	msd->storage.addr += MAX_MIF_SEPA_SIZE;
	memcpy(msd->storage.addr, &size, sizeof(int));
	msd->storage.addr += MAX_MIF_SEPA_SIZE;
	spin_lock_init(&msd->lock);
	spin_lock_init(&msd->active_list_lock);

	return msd;
}

static struct modem_ctl *create_modemctl_device(struct platform_device *pdev,
		struct modem_shared *msd)
{
	struct device *dev = &pdev->dev;
	struct modem_data *pdata = pdev->dev.platform_data;
	struct modem_ctl *modemctl;
	int ret;

	/* create modem control device */
	modemctl = devm_kzalloc(dev, sizeof(struct modem_ctl), GFP_KERNEL);
	if (!modemctl) {
		mif_err("%s: modemctl devm_kzalloc fail\n", pdata->name);
		mif_err("%s: xxx\n", pdata->name);
		return NULL;
	}

	modemctl->dev = dev;
	modemctl->name = pdata->name;
	modemctl->mdm_data = pdata;

	modemctl->msd = msd;

	modemctl->phone_state = STATE_OFFLINE;

	INIT_LIST_HEAD(&modemctl->modem_state_notify_list);
	spin_lock_init(&modemctl->lock);
	spin_lock_init(&modemctl->tx_timer_lock);
	init_completion(&modemctl->init_cmpl);
	init_completion(&modemctl->off_cmpl);

	/* init modemctl device for getting modemctl operations */
	ret = call_modem_init_func(modemctl, pdata);
	if (ret) {
		mif_err("%s: call_modem_init_func fail (err %d)\n",
			pdata->name, ret);
		mif_err("%s: xxx\n", pdata->name);
		devm_kfree(dev, modemctl);
		return NULL;
	}

	mif_info("%s is created!!!\n", pdata->name);

	return modemctl;
}

static struct io_device *create_io_device(struct platform_device *pdev,
		struct modem_io_t *io_t, struct modem_shared *msd,
		struct modem_ctl *modemctl, struct modem_data *pdata)
{
	int ret;
	struct device *dev = &pdev->dev;
	struct io_device *iod;

	iod = devm_kzalloc(dev, sizeof(struct io_device), GFP_KERNEL);
	if (!iod) {
		mif_err("iod == NULL\n");
		return NULL;
	}

	INIT_LIST_HEAD(&iod->list);
	RB_CLEAR_NODE(&iod->node_fmt);

	iod->name = io_t->name;
	iod->ch = io_t->ch;
	iod->format = io_t->format;
	iod->io_typ = io_t->io_type;
	iod->link_type = io_t->link_type;
	iod->attrs = io_t->attrs;
	iod->max_tx_size = io_t->ul_buffer_size;
	iod->ipc_version = pdata->ipc_version;
	atomic_set(&iod->opened, 0);
	spin_lock_init(&iod->info_id_lock);
	spin_lock_init(&iod->clat_lock);

	/* link between io device and modem control */
	iod->mc = modemctl;

	switch (pdata->protocol) {
	case PROTOCOL_SIPC:
		if (iod->format == IPC_FMT && iod->ch == SIPC5_CH_ID_FMT_0)
			modemctl->iod = iod;
		break;
	case PROTOCOL_SIT:
		if (iod->format == IPC_FMT && iod->ch == EXYNOS_CH_ID_FMT_0)
			modemctl->iod = iod;
		break;
	default:
		mif_err("protocol error\n");
		return NULL;
	}

	if (iod->format == IPC_BOOT) {
		modemctl->bootd = iod;
		mif_info("BOOT device = %s\n", iod->name);
	}

	/* link between io device and modem shared */
	iod->msd = msd;

	/* add iod to rb_tree */
	if (iod->format != IPC_RAW)
		insert_iod_with_format(msd, iod->format, iod);

	switch (pdata->protocol) {
	case PROTOCOL_SIPC:
		if (sipc5_is_not_reserved_channel(iod->ch))
			insert_iod_with_channel(msd, iod->ch, iod);
		break;
	case PROTOCOL_SIT:
		insert_iod_with_channel(msd, iod->ch, iod);
		break;
	default:
		mif_err("protocol error\n");
		return NULL;
	}

	/* register misc device or net device */
	ret = sipc5_init_io_device(iod, pdata->mld);
	if (ret) {
		devm_kfree(dev, iod);
		mif_err("sipc5_init_io_device fail (%d)\n", ret);
		return NULL;
	}

	mif_info("%s created. attrs:0x%08x\n", iod->name, iod->attrs);
	return iod;
}

static int attach_devices(struct io_device *iod, struct device *dev)
{
	struct modem_shared *msd = iod->msd;
	struct link_device *ld;

	/* find link type for this io device */
	list_for_each_entry(ld, &msd->link_dev_list, list) {
		if (IS_CONNECTED(iod, ld)) {
			mif_debug("set %s->%s\n", iod->name, ld->name);
			set_current_link(iod, ld);

			if (iod->io_typ == IODEV_NET && iod->ndev) {
				struct vnet *vnet;

				vnet = netdev_priv(iod->ndev);
				vnet->hiprio_ack_only = ld->hiprio_ack_only;
			}
		}
	}

	if (!get_current_link(iod)) {
		mif_err("%s->link == NULL\n", iod->name);
		BUG();
	}

	iod->ws = cpif_wake_lock_register(dev, iod->name);
	if (iod->ws == NULL) {
		mif_err("%s: wakeup_source_register fail\n", iod->name);
		return -EINVAL;
	}

	switch (iod->ch) {
	case SIPC5_CH_ID_FMT_0 ... SIPC5_CH_ID_FMT_9:
		iod->waketime = FMT_WAKE_TIME;
		break;

	case SIPC5_CH_ID_BOOT_0 ... SIPC5_CH_ID_DUMP_9:
		iod->waketime = RAW_WAKE_TIME;
		break;

#if IS_ENABLED(CONFIG_CH_EXTENSION)
	case SIPC_CH_EX_ID_PDP_0 ... SIPC_CH_EX_ID_PDP_MAX:
	case SIPC_CH_ID_BT_DUN ... SIPC_CH_ID_CIQ_DATA:
	case SIPC_CH_ID_CPLOG1 ... SIPC_CH_ID_LOOPBACK2:
#else
	case SIPC_CH_ID_PDP_0 ... SIPC_CH_ID_LOOPBACK2:
#endif
		iod->waketime = NET_WAKE_TIME;
		break;

	default:
		iod->waketime = 0;
		break;
	}

	switch (iod->format) {
	case IPC_FMT:
		iod->waketime = FMT_WAKE_TIME;
		break;

	case IPC_BOOT ... IPC_DUMP:
		iod->waketime = RAW_WAKE_TIME;
		break;

	default:
		break;
	}

	return 0;
}

static int parse_dt_common_pdata(struct device_node *np,
				 struct modem_data *pdata)
{
	mif_dt_read_string(np, "mif,name", pdata->name);
	mif_dt_read_u32(np, "mif,cp_num", pdata->cp_num);

	mif_dt_read_enum(np, "mif,modem_type", pdata->modem_type);
	mif_dt_read_enum(np, "mif,ipc_version", pdata->ipc_version);

	mif_dt_read_u32_noerr(np, "mif,protocol", pdata->protocol);
	mif_info("protocol:%d\n", pdata->protocol);

	mif_dt_read_u32(np, "mif,link_type", pdata->link_type);
	mif_dt_read_string(np, "mif,link_name", pdata->link_name);
	mif_dt_read_u32(np, "mif,link_attrs", pdata->link_attrs);
	mif_dt_read_u32(np, "mif,interrupt_types", pdata->interrupt_types);

	mif_dt_read_u32_noerr(np, "mif,capability_check", pdata->capability_check);
	mif_info("capability_check:%d\n", pdata->capability_check);

	mif_dt_read_u32_noerr(np, "mif,cp2ap_active_not_alive", pdata->cp2ap_active_not_alive);
	mif_info("cp2ap_active_not_alive:%d\n", pdata->cp2ap_active_not_alive);

	mif_dt_read_u32_noerr(np, "mif_off_during_volte", pdata->mif_off_during_volte);
	mif_info("mif_off_during_volte:%d\n", pdata->mif_off_during_volte);

	return 0;
}

static int parse_dt_mbox_pdata(struct device *dev, struct device_node *np,
					struct modem_data *pdata)
{
	struct modem_mbox *mbox;
	int ret = 0;

	if ((pdata->link_type != LINKDEV_SHMEM) &&
			(pdata->link_type != LINKDEV_PCIE)) {
		mif_err("mbox: link type error:0x%08x\n", pdata->link_type);
		return ret;
	}

	mbox = (struct modem_mbox *)devm_kzalloc(dev, sizeof(struct modem_mbox), GFP_KERNEL);
	if (!mbox) {
		mif_err("mbox: failed to alloc memory\n");
		return -ENOMEM;
	}
	pdata->mbx = mbox;

	mif_dt_read_u32(np, "mif,int_ap2cp_msg", mbox->int_ap2cp_msg);
	mif_dt_read_u32(np, "mif,int_ap2cp_wakeup", mbox->int_ap2cp_wakeup);
	mif_dt_read_u32(np, "mif,int_ap2cp_status", mbox->int_ap2cp_status);
	mif_dt_read_u32(np, "mif,int_ap2cp_active", mbox->int_ap2cp_active);
#if IS_ENABLED(CONFIG_CP_LCD_NOTIFIER)
	mif_dt_read_u32(np, "mif,int_ap2cp_lcd_status",
			mbox->int_ap2cp_lcd_status);
#endif
#if IS_ENABLED(CONFIG_CP_PKTPROC_CLAT)
	mif_dt_read_u32(np, "mif,int_ap2cp_clatinfo_send", mbox->int_ap2cp_clatinfo_send);
#endif
#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE)
	mif_dt_read_u32(np, "mif,int_ap2cp_pcie_link_ack", mbox->int_ap2cp_pcie_link_ack);
#endif
	mif_dt_read_u32(np, "mif,int_ap2cp_uart_noti", mbox->int_ap2cp_uart_noti);

	mif_dt_read_u32(np, "mif,irq_cp2ap_msg", mbox->irq_cp2ap_msg);
	mif_dt_read_u32(np, "mif,irq_cp2ap_status", mbox->irq_cp2ap_status);
	mif_dt_read_u32(np, "mif,irq_cp2ap_active", mbox->irq_cp2ap_active);
#if IS_ENABLED(CONFIG_CP_PKTPROC_CLAT)
	mif_dt_read_u32(np, "mif,irq_cp2ap_clatinfo_ack", mbox->irq_cp2ap_clatinfo_ack);
#endif
	mif_dt_read_u32(np, "mif,irq_cp2ap_wakelock", mbox->irq_cp2ap_wakelock);
	mif_dt_read_u32(np, "mif,irq_cp2ap_ratmode", mbox->irq_cp2ap_rat_mode);

	return ret;
}

static int parse_dt_ipc_region_pdata(struct device *dev, struct device_node *np,
					struct modem_data *pdata)
{
	int ret = 0;

	/* legacy buffer (fmt, raw) setting */
	mif_dt_read_u32(np, "legacy_fmt_head_tail_offset",
			pdata->legacy_fmt_head_tail_offset);
	mif_dt_read_u32(np, "legacy_fmt_buffer_offset", pdata->legacy_fmt_buffer_offset);
	mif_dt_read_u32(np, "legacy_fmt_txq_size", pdata->legacy_fmt_txq_size);
	mif_dt_read_u32(np, "legacy_fmt_rxq_size", pdata->legacy_fmt_rxq_size);
	mif_dt_read_u32(np, "legacy_raw_head_tail_offset",
			pdata->legacy_raw_head_tail_offset);
	mif_dt_read_u32(np, "legacy_raw_buffer_offset", pdata->legacy_raw_buffer_offset);
	mif_dt_read_u32(np, "legacy_raw_txq_size", pdata->legacy_raw_txq_size);
	mif_dt_read_u32(np, "legacy_raw_rxq_size", pdata->legacy_raw_rxq_size);
	mif_dt_read_u32(np, "legacy_raw_rx_buffer_cached",
			pdata->legacy_raw_rx_buffer_cached);

	mif_dt_read_u32_noerr(np, "offset_ap_version", pdata->offset_ap_version);
	mif_dt_read_u32_noerr(np, "offset_cp_version", pdata->offset_cp_version);
	mif_dt_read_u32_noerr(np, "offset_cmsg_offset", pdata->offset_cmsg_offset);
	mif_dt_read_u32_noerr(np, "offset_srinfo_offset", pdata->offset_srinfo_offset);
	mif_dt_read_u32_noerr(np, "offset_clk_table_offset", pdata->offset_clk_table_offset);
	mif_dt_read_u32_noerr(np, "offset_buff_desc_offset", pdata->offset_buff_desc_offset);
	mif_dt_read_u32_noerr(np, "offset_capability_offset", pdata->offset_capability_offset);

#if IS_ENABLED(CONFIG_MODEM_IF_LEGACY_QOS)
	/* legacy priority queue setting */
	mif_dt_read_u32(np, "legacy_raw_qos_head_tail_offset",
			pdata->legacy_raw_qos_head_tail_offset);
	mif_dt_read_u32(np, "legacy_raw_qos_buffer_offset", pdata->legacy_raw_qos_buffer_offset);
	mif_dt_read_u32(np, "legacy_raw_qos_txq_size", pdata->legacy_raw_qos_txq_size);
	mif_dt_read_u32(np, "legacy_raw_qos_rxq_size", pdata->legacy_raw_qos_rxq_size);
#endif

	/* control message offset setting (optional)*/
	mif_dt_read_u32_noerr(np, "cmsg_offset", pdata->cmsg_offset);
	/* srinfo settings */
	mif_dt_read_u32(np, "srinfo_offset", pdata->srinfo_offset);
	mif_dt_read_u32(np, "srinfo_size", pdata->srinfo_size);
	/* clk_table offset (optional)*/
	mif_dt_read_u32_noerr(np, "clk_table_offset", pdata->clk_table_offset);
	/* offset setting for new SIT buffer descriptors (optional) */
	mif_dt_read_u32_noerr(np, "buff_desc_offset", pdata->buff_desc_offset);

	/* offset setting for capability */
	if (pdata->capability_check) {
		mif_dt_read_u32(np, "capability_offset", pdata->capability_offset);
		mif_dt_read_u32(np, "ap_capability_0", pdata->ap_capability[0]);
		mif_dt_read_u32(np, "ap_capability_1", pdata->ap_capability[1]);
	}

	of_property_read_u32_array(np, "ap2cp_msg", pdata->ap2cp_msg, 2);
	of_property_read_u32_array(np, "cp2ap_msg", pdata->cp2ap_msg, 2);
	of_property_read_u32_array(np, "cp2ap_united_status", pdata->cp2ap_united_status, 2);
	of_property_read_u32_array(np, "ap2cp_united_status", pdata->ap2cp_united_status, 2);
#if IS_ENABLED(CONFIG_CP_PKTPROC_CLAT)
	mif_dt_count_u32_array(np, "ap2cp_clatinfo_xlat_v4_addr",
			pdata->ap2cp_clatinfo_xlat_v4_addr, 2);
	mif_dt_count_u32_array(np, "ap2cp_clatinfo_xlat_addr_0",
			pdata->ap2cp_clatinfo_xlat_addr_0, 2);
	mif_dt_count_u32_array(np, "ap2cp_clatinfo_xlat_addr_1",
			pdata->ap2cp_clatinfo_xlat_addr_1, 2);
	mif_dt_count_u32_array(np, "ap2cp_clatinfo_xlat_addr_2",
			pdata->ap2cp_clatinfo_xlat_addr_2, 2);
	mif_dt_count_u32_array(np, "ap2cp_clatinfo_xlat_addr_3",
			pdata->ap2cp_clatinfo_xlat_addr_3, 2);
	mif_dt_count_u32_array(np, "ap2cp_clatinfo_index",
			pdata->ap2cp_clatinfo_index, 2);
#endif
	of_property_read_u32_array(np, "ap2cp_kerneltime", pdata->ap2cp_kerneltime, 2);
	of_property_read_u32_array(np, "ap2cp_kerneltime_sec", pdata->ap2cp_kerneltime_sec, 2);
	of_property_read_u32_array(np, "ap2cp_kerneltime_usec", pdata->ap2cp_kerneltime_usec, 2);
	of_property_read_u32_array(np, "ap2cp_handover_block_info",
			pdata->ap2cp_handover_block_info, 2);

	/* Status Bit Info */
	mif_dt_read_u32(np, "sbi_lte_active_mask", pdata->sbi_lte_active_mask);
	mif_dt_read_u32(np, "sbi_lte_active_pos", pdata->sbi_lte_active_pos);
	mif_dt_read_u32(np, "sbi_cp_status_mask", pdata->sbi_cp_status_mask);
	mif_dt_read_u32(np, "sbi_cp_status_pos", pdata->sbi_cp_status_pos);
	mif_dt_read_u32(np, "sbi_cp_rat_mode_mask", pdata->sbi_cp2ap_rat_mode_mask);
	mif_dt_read_u32(np, "sbi_cp_rat_mode_pos", pdata->sbi_cp2ap_rat_mode_pos);
	mif_dt_read_u32(np, "sbi_cp2ap_wakelock_mask", pdata->sbi_cp2ap_wakelock_mask);
	mif_dt_read_u32(np, "sbi_cp2ap_wakelock_pos", pdata->sbi_cp2ap_wakelock_pos);
	mif_dt_read_u32(np, "sbi_pda_active_mask", pdata->sbi_pda_active_mask);
	mif_dt_read_u32(np, "sbi_pda_active_pos", pdata->sbi_pda_active_pos);
	mif_dt_read_u32(np, "sbi_ap_status_mask", pdata->sbi_ap_status_mask);
	mif_dt_read_u32(np, "sbi_ap_status_pos", pdata->sbi_ap_status_pos);
	mif_dt_read_u32(np, "sbi_crash_type_mask", pdata->sbi_crash_type_mask);
	mif_dt_read_u32(np, "sbi_crash_type_pos", pdata->sbi_crash_type_pos);
#if IS_ENABLED(CONFIG_CP_LCD_NOTIFIER)
	mif_dt_read_u32(np, "sbi_lcd_status_mask", pdata->sbi_lcd_status_mask);
	mif_dt_read_u32(np, "sbi_lcd_status_pos", pdata->sbi_lcd_status_pos);
#endif
	mif_dt_read_u32(np, "sbi_uart_noti_mask", pdata->sbi_uart_noti_mask);
	mif_dt_read_u32(np, "sbi_uart_noti_pos", pdata->sbi_uart_noti_pos);
	mif_dt_read_u32(np, "sbi_ds_det_mask", pdata->sbi_ds_det_mask);
	mif_dt_read_u32(np, "sbi_ds_det_pos", pdata->sbi_ds_det_pos);
	mif_dt_read_u32_noerr(np, "sbi_ap2cp_kerneltime_sec_mask",
			pdata->sbi_ap2cp_kerneltime_sec_mask);
	mif_dt_read_u32_noerr(np, "sbi_ap2cp_kerneltime_sec_pos",
			pdata->sbi_ap2cp_kerneltime_sec_pos);
	mif_dt_read_u32_noerr(np, "sbi_ap2cp_kerneltime_usec_mask",
			pdata->sbi_ap2cp_kerneltime_usec_mask);
	mif_dt_read_u32_noerr(np, "sbi_ap2cp_kerneltime_usec_pos",
			pdata->sbi_ap2cp_kerneltime_usec_pos);

	/* Check pktproc use 36bit addr */
	mif_dt_read_u32(np, "pktproc_use_36bit_addr",
			pdata->pktproc_use_36bit_addr);

	return ret;
}

static int parse_dt_iodevs_pdata(struct device *dev, struct device_node *np,
				 struct modem_data *pdata)
{
	struct device_node *child = NULL;

	for_each_child_of_node(np, child) {
		struct modem_io_t *p_iod = NULL;
		struct modem_io_t *iod;
		unsigned int ch_count = 0;
		char *name;

		do {
			iod = devm_kzalloc(dev, sizeof(struct modem_io_t), GFP_KERNEL);
			if (!iod) {
				mif_err("failed to alloc iodev\n");
				return -ENOMEM;
			}

			if (!p_iod) {
				mif_dt_read_string(child, "iod,name", name);
				mif_dt_read_u32(child, "iod,ch", iod->ch);
				mif_dt_read_enum(child, "iod,format", iod->format);
				mif_dt_read_enum(child, "iod,io_type", iod->io_type);
				mif_dt_read_u32(child, "iod,link_type", iod->link_type);
				mif_dt_read_u32(child, "iod,attrs", iod->attrs);
				mif_dt_read_u32_noerr(child, "iod,max_tx_size",
						      iod->ul_buffer_size);

				if (iod->attrs & IO_ATTR_SBD_IPC) {
					mif_dt_read_u32(child, "iod,ul_num_buffers",
							iod->ul_num_buffers);
					mif_dt_read_u32(child, "iod,ul_buffer_size",
							iod->ul_buffer_size);
					mif_dt_read_u32(child, "iod,dl_num_buffers",
							iod->dl_num_buffers);
					mif_dt_read_u32(child, "iod,dl_buffer_size",
							iod->dl_buffer_size);
				}

				if (iod->attrs & IO_ATTR_OPTION_REGION)
					mif_dt_read_string(child, "iod,option_region",
							   iod->option_region);

				if (iod->attrs & IO_ATTR_MULTI_CH)
					mif_dt_read_u32(child, "iod,ch_count", iod->ch_count);

				p_iod = iod;
			} else {
				memcpy(iod, p_iod, sizeof(struct modem_io_t));
			}

			if (ch_count < iod->ch_count) {
				scnprintf(iod->name, sizeof(iod->name), "%s%d", name, ch_count);
				iod->ch = p_iod->ch + ch_count;
			} else {
				scnprintf(iod->name, sizeof(iod->name), "%s", name);
			}

			pdata->iodevs[pdata->num_iodevs] = iod;
			pdata->num_iodevs++;
		} while (++ch_count < iod->ch_count);
	}

	mif_info("num_iodevs:%d\n", pdata->num_iodevs);

	return 0;
}

static struct modem_data *modem_if_parse_dt_pdata(struct device *dev)
{
	struct modem_data *pdata;
	struct device_node *iodevs_node = NULL;

	pdata = devm_kzalloc(dev, sizeof(struct modem_data), GFP_KERNEL);
	if (!pdata) {
		mif_err("modem_data: alloc fail\n");
		return ERR_PTR(-ENOMEM);
	}

	if (parse_dt_common_pdata(dev->of_node, pdata)) {
		mif_err("DT error: failed to parse common\n");
		goto error;
	}

	if (parse_dt_mbox_pdata(dev, dev->of_node, pdata)) {
		mif_err("DT error: failed to parse mbox\n");
		goto error;
	}

	if (parse_dt_ipc_region_pdata(dev, dev->of_node, pdata)) {
		mif_err("DT error: failed to parse control messages\n");
		goto error;
	}

	iodevs_node = of_get_child_by_name(dev->of_node, "iodevs");
	if (!iodevs_node) {
		mif_err("DT error: failed to get child node\n");
		goto error;
	}

	if (parse_dt_iodevs_pdata(dev, iodevs_node, pdata)) {
		mif_err("DT error: failed to parse iodevs\n");
		goto error;
	}

	dev->platform_data = pdata;
	mif_info("DT parse complete!\n");

	return pdata;

error:
	if (pdata) {
		unsigned int id;

		for (id = 0; id < ARRAY_SIZE(pdata->iodevs); id++) {
			if (pdata->iodevs[id])
				devm_kfree(dev, pdata->iodevs[id]);
		}

		devm_kfree(dev, pdata);
	}
	return ERR_PTR(-EINVAL);
}

static const struct of_device_id cpif_dt_match[] = {
	{ .compatible = "samsung,exynos-cp", },
	{},
};
MODULE_DEVICE_TABLE(of, cpif_dt_match);

enum mif_sim_mode {
	MIF_SIM_NONE = 0,
	MIF_SIM_SINGLE,
	MIF_SIM_DUAL,
	MIF_SIM_TRIPLE,
};

static int simslot_count(struct seq_file *m, void *v)
{
	enum mif_sim_mode mode = (enum mif_sim_mode)(uintptr_t)(m->private);

	seq_printf(m, "%u\n", mode);
	return 0;
}

static int simslot_count_open(struct inode *inode, struct file *file)
{
	return single_open(file, simslot_count, pde_data(inode));
}

static const struct file_operations __maybe_unused simslot_count_fops = {
	.open	= simslot_count_open,
	.read	= seq_read,
	.llseek	= seq_lseek,
	.release = single_release,
};

#if IS_ENABLED(CONFIG_GPIO_DS_DETECT)
static enum mif_sim_mode get_sim_mode(struct device_node *of_node)
{
	enum mif_sim_mode mode = MIF_SIM_SINGLE;
	int gpio_ds_det;
	int retval;

	gpio_ds_det = of_get_named_gpio(of_node, "mif,gpio_ds_det", 0);
	if (!gpio_is_valid(gpio_ds_det)) {
		mif_err("DT error: failed to get sim mode\n");
		goto make_proc;
	}

	retval = gpio_request(gpio_ds_det, "DS_DET");
	if (retval) {
		mif_err("Failed to request GPIO(%d)\n", retval);
		goto make_proc;
	} else {
		gpio_direction_input(gpio_ds_det);
	}

	retval = gpio_get_value(gpio_ds_det);
	if (retval)
		mode = MIF_SIM_DUAL;

	mif_info("sim_mode: %d\n", mode);
	gpio_free(gpio_ds_det);

make_proc:
	if (!proc_create_data("simslot_count", 0444, NULL, &simslot_count_fops,
			(void *)(long)mode)) {
		mif_err("Failed to create proc\n");
		mode = MIF_SIM_SINGLE;
	}

	return mode;
}
#else
static enum mif_sim_mode get_sim_mode(struct device_node *of_node)
{
	return MIF_SIM_SINGLE;
}
#endif

static ssize_t do_cp_crash_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	modem_force_crash_exit_ext(buf);

	return count;
}

static ssize_t modem_state_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct modem_ctl *mc = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%s\n", cp_state_str(mc->phone_state));
}

static DEVICE_ATTR_WO(do_cp_crash);
static DEVICE_ATTR_RO(modem_state);

static struct attribute *modem_attrs[] = {
	&dev_attr_do_cp_crash.attr,
	&dev_attr_modem_state.attr,
	NULL,
};
ATTRIBUTE_GROUPS(modem);

static int cpif_cdev_alloc_region(struct modem_data *pdata, struct modem_shared *msd)
{
	int ret;

	ret = alloc_chrdev_region(&msd->cdev_major, 0, pdata->num_iodevs, "cpif");
	if (ret < 0) {
		mif_err("alloc_chrdev_region() failed:%d\n", ret);
		return ret;
	}

	msd->cdev_class = class_create(THIS_MODULE, "cpif");
	if (IS_ERR(msd->cdev_class)) {
		mif_err("class_create() failed:%ld\n", PTR_ERR(msd->cdev_class));
		ret = -ENOMEM;
		unregister_chrdev_region(MAJOR(msd->cdev_major), pdata->num_iodevs);
		return ret;
	}

	return 0;
}

static int cpif_probe(struct platform_device *pdev)
{
	int i;
	struct device *dev = &pdev->dev;
	struct modem_data *pdata = NULL;
	struct modem_shared *msd;
	struct modem_ctl *modemctl;
	struct io_device **iod;
	struct link_device *ld;
	enum mif_sim_mode sim_mode;
	int err;

	mif_info("Exynos CP interface driver %s\n", get_cpif_driver_version());
	mif_info("%s: +++ (%s)\n", pdev->name, CONFIG_OPTION_REGION);

	err = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(36));
	if (err) {
		mif_err("dma_set_mask_and_coherent() error:%d\n", err);
		goto fail;
	}

	if (dev->of_node) {
		pdata = modem_if_parse_dt_pdata(dev);
		if (IS_ERR(pdata)) {
			mif_err("MIF DT parse error!\n");
			err = PTR_ERR(pdata);
			goto fail;
		}
	} else {
		if (!pdata) {
			mif_err("Non-DT, incorrect pdata!\n");
			err = -EINVAL;
			goto fail;
		}
	}

	msd = create_modem_shared_data(pdev);
	if (!msd) {
		mif_err("%s: msd == NULL\n", pdata->name);
		err = -ENOMEM;
		goto fail;
	}

	modemctl = create_modemctl_device(pdev, msd);
	if (!modemctl) {
		mif_err("%s: modemctl == NULL\n", pdata->name);
		devm_kfree(dev, msd);
		err = -ENOMEM;
		goto fail;
	}

	modemctl->log = logbuffer_register("cpif");
	if (IS_ERR_OR_NULL(modemctl->log)) {
		mif_err("Failed to register logbuffer!\n");
		modemctl->log = NULL;
	}

#if IS_ENABLED(CONFIG_S5910)
	/* get the s5910 node pointer */
	modemctl->s5910_dev = NULL;
	if (dev->of_node) {
		struct device_node *np;
		struct device *dp;
		struct device_link *link;

		np = of_parse_phandle(dev->of_node, "google,clk-buffer", 0);
		if (np) {
			dp = s5910_get_device(np);
			if (dp) {
				modemctl->s5910_dev = dp;
				link = device_link_add(dev, dp, 0);
				if (link)
					mif_info("%s s5910 linked\n",
							pdata->name);
			}
		}
	}
#endif

	if (toe_dev_create(pdev)) {
		mif_err("%s: toe dev not created\n", pdata->name);
		goto free_mc;
	}

	/* create link device */
	ld = call_link_init_func(pdev, pdata->link_type);
	if (!ld)
		goto free_mc;

	mif_info("%s: %s link created\n", pdata->name, ld->name);

	ld->mc = modemctl;
	ld->msd = msd;
	list_add(&ld->list, &msd->link_dev_list);

	/* get sim mode */
	sim_mode = get_sim_mode(dev->of_node);

	/* char device */
	err = cpif_cdev_alloc_region(pdata, msd);
	if (err) {
		mif_err("cpif_cdev_alloc_region() err:%d\n", err);
		goto free_mc;
	}

	/* create io deivces and connect to modemctl device */
	iod = kcalloc(pdata->num_iodevs, sizeof(*iod), GFP_KERNEL);
	if (!iod) {
		mif_err("kcalloc() err\n");
		goto free_chrdev;
	}

	for (i = 0; i < pdata->num_iodevs; i++) {
		if (sim_mode < MIF_SIM_DUAL &&
			pdata->iodevs[i]->attrs & IO_ATTR_DUALSIM)
			continue;

		if (pdata->iodevs[i]->attrs & IO_ATTR_OPTION_REGION &&
		    strcmp(pdata->iodevs[i]->option_region, CONFIG_OPTION_REGION))
			continue;

		iod[i] = create_io_device(pdev, pdata->iodevs[i], msd,
					  modemctl, pdata);
		if (!iod[i]) {
			mif_err("%s: iod[%d] == NULL\n", pdata->name, i);
			goto free_iod;
		}

		/* Basically, iods of IPC_FMT and IPC_BOOT will receive the state */
		if (iod[i]->format == IPC_FMT || iod[i]->format == IPC_BOOT ||
		    iod[i]->attrs & IO_ATTR_STATE_RESET_NOTI)
			list_add_tail(&iod[i]->list, &modemctl->modem_state_notify_list);

		attach_devices(iod[i], dev);
	}

	cp_btl_create(&pdata->btl, dev);

	platform_set_drvdata(pdev, modemctl);

	kfree(iod);

	if (sysfs_create_groups(&dev->kobj, modem_groups))
		mif_err("failed to create modem groups node\n");

#if IS_ENABLED(CONFIG_MODEM_IF_LEGACY_QOS)
	err = cpif_qos_init_list();
	if (err < 0)
		mif_err("failed to initialize hiprio list(%d)\n", err);
#endif

#if IS_ENABLED(CONFIG_CPIF_VENDOR_HOOK)
	err = hook_init();
	if (err)
		mif_err("failed to register vendor hook\n");
#endif

	mif_info("%s: done ---\n", pdev->name);
	return 0;

free_iod:
	for (i = 0; i < pdata->num_iodevs; i++) {
		if (iod[i]) {
			sipc5_deinit_io_device(iod[i]);
			devm_kfree(dev, iod[i]);
		}
	}
	kfree(iod);

free_chrdev:
	class_destroy(msd->cdev_class);
	unregister_chrdev_region(MAJOR(msd->cdev_major), pdata->num_iodevs);

free_mc:
	if (modemctl) {
		call_modem_uninit_func(modemctl, pdata);
		devm_kfree(dev, modemctl);
	}

	if (msd)
		devm_kfree(dev, msd);

	err = -ENOMEM;

fail:
	mif_err("%s: xxx\n", pdev->name);

	panic("CP interface driver probe failed\n");
	return err;
}

static void cpif_shutdown(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct modem_ctl *mc = dev_get_drvdata(dev);

	if (mc->ops.power_shutdown)
		mc->ops.power_shutdown(mc);

	mc->phone_state = STATE_OFFLINE;

	mif_err("%s\n", mc->name);
}

static int modem_suspend(struct device *pdev)
{
	int ret = 0;
	struct modem_ctl *mc = dev_get_drvdata(pdev);

	if (mc->ops.suspend)
		ret = mc->ops.suspend(mc);

	if (!ret) {
#if defined(CPIF_WAKEPKT_SET_MARK)
		atomic_set(&mc->mark_skb_wakeup, 1);
#endif
		set_wakeup_packet_log(true);
	}

	return ret;
}

static int modem_resume(struct device *pdev)
{
	struct modem_ctl *mc = dev_get_drvdata(pdev);

	set_wakeup_packet_log(false);

	if (mc->ops.resume)
		mc->ops.resume(mc);

	return 0;
}

static const struct dev_pm_ops cpif_pm_ops = {
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(modem_suspend, modem_resume)
};

static struct platform_driver cpif_driver = {
	.probe = cpif_probe,
	.shutdown = cpif_shutdown,
	.driver = {
		.name = "cp_interface",
		.owner = THIS_MODULE,
		.pm = &cpif_pm_ops,
		.suppress_bind_attrs = true,
#if IS_ENABLED(CONFIG_OF)
		.of_match_table = of_match_ptr(cpif_dt_match),
#endif
	},
};

module_platform_driver(cpif_driver);

MODULE_DESCRIPTION("Exynos CP interface Driver");
MODULE_LICENSE("GPL");
