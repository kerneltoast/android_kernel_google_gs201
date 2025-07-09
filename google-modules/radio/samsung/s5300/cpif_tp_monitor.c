// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019-2020, Samsung Electronics.
 *
 */

#include <linux/ip.h>
#include <linux/ipv6.h>
#include "modem_prj.h"
#include "modem_utils.h"
#include "modem_ctrl.h"
#include "link_device_memory.h"
#include "cpif_tp_monitor.h"
#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE)
#include <linux/exynos-pci-ctrl.h>
#include "s51xx_pcie.h"
#endif
#if IS_ENABLED(CONFIG_EXYNOS_DIT)
#include "dit.h"
#endif
#if IS_ENABLED(CONFIG_EXYNOS_BTS)
#include <soc/google/bts.h>
#endif

static struct cpif_tpmon _tpmon;

/*
 * Get data
 */
/* RX speed */
static u32 tpmon_get_rx_speed_mbps(struct tpmon_data *data)
{
	unsigned long speed = 0;

	if (!data->enable)
		return 0;

	switch (data->proto) {
	case TPMON_PROTO_TCP:
		speed = data->tpmon->rx_tcp.rx_mbps;
		break;
	case TPMON_PROTO_UDP:
		speed = data->tpmon->rx_udp.rx_mbps;
		break;
	case TPMON_PROTO_OTHERS:
		speed = data->tpmon->rx_others.rx_mbps;
		break;
	case TPMON_PROTO_ALL:
	default:
		speed = data->tpmon->rx_total.rx_mbps;
		break;
	}

	return (u32)speed;
}

static int tpmon_calc_rx_speed_internal(
	struct cpif_tpmon *tpmon, struct cpif_rx_data *rx_data, bool check_stat)
{
	u64 rx_bytes;
	u64 delta_msec;
	ktime_t curr_time;
	unsigned long flags;

	curr_time = ktime_get();

	delta_msec = ktime_ms_delta(curr_time, rx_data->prev_time);
	if (delta_msec < tpmon->trigger_msec_min)
		return -EIO;

	rx_data->prev_time = curr_time;

	spin_lock_irqsave(&tpmon->lock, flags);
	rx_bytes = rx_data->rx_bytes;
	rx_data->rx_bytes = 0;
	spin_unlock_irqrestore(&tpmon->lock, flags);

	if (!check_stat && (delta_msec > tpmon->trigger_msec_max)) {
		rx_data->rx_mbps = 0;
		return -EIO;
	}

	rx_data->rx_mbps = rx_bytes * 8 / delta_msec / 1000;

	return 0;
}

static void tpmon_stat_rx_speed(struct cpif_tpmon *tpmon)
{
	tpmon_calc_rx_speed_internal(tpmon, &tpmon->rx_total_stat, true);
	tpmon_calc_rx_speed_internal(tpmon, &tpmon->rx_tcp_stat, true);
	tpmon_calc_rx_speed_internal(tpmon, &tpmon->rx_udp_stat, true);
	tpmon_calc_rx_speed_internal(tpmon, &tpmon->rx_others_stat, true);
}

static void tpmon_calc_rx_speed(struct cpif_tpmon *tpmon)
{
	int ret = 0;

	ret = tpmon_calc_rx_speed_internal(tpmon, &tpmon->rx_total, false);
	tpmon_calc_rx_speed_internal(tpmon, &tpmon->rx_tcp, false);
	tpmon_calc_rx_speed_internal(tpmon, &tpmon->rx_udp, false);
	tpmon_calc_rx_speed_internal(tpmon, &tpmon->rx_others, false);

	if (tpmon->debug_print && tpmon->rx_total.rx_mbps && !ret)
		mif_info("%ldMbps(%ld/%ld/%ld)\n",
			tpmon->rx_total.rx_mbps, tpmon->rx_tcp.rx_mbps,
			tpmon->rx_udp.rx_mbps, tpmon->rx_others.rx_mbps);

	if (!tpmon_check_active())
		tpmon_stat_rx_speed(tpmon);
}

/* Queue status */
static u32 tpmon_get_q_status(struct tpmon_data *data)
{
	u32 usage = 0;

	if (!data->enable)
		return 0;

	switch (data->measure) {
	case TPMON_MEASURE_NETDEV_Q:
		usage = data->tpmon->q_status_netdev_backlog;
		break;
	case TPMON_MEASURE_PKTPROC_DL_Q:
		usage = data->tpmon->q_status_pktproc_dl;
		break;
	case TPMON_MEASURE_DIT_SRC_Q:
		usage = data->tpmon->q_status_dit_src;
		break;
	default:
		mif_err_limited("measure %d is not valid\n", data->measure);
		break;
	}

	return usage;
}

static int tpmon_calc_q_status_pktproc_dl(struct cpif_tpmon *tpmon)
{
	struct mem_link_device *mld = ld_to_mem_link_device(tpmon->ld);
	struct pktproc_adaptor *ppa = &mld->pktproc;
	u32 usage = 0;
	int i;

	if (!pktproc_check_support(ppa))
		return 0;

	for (i = 0; i < mld->pktproc.num_queue; i++) {
		if (!pktproc_check_active(ppa, i))
			continue;

		if (pktproc_get_usage_fore_rear(ppa->q[i]) > 0)
			usage += pktproc_get_usage_fore_rear(ppa->q[i]);
	}

	tpmon->q_status_pktproc_dl = usage;

	return 0;
}

static int tpmon_calc_q_status_dit_src(struct cpif_tpmon *tpmon)
{
	u32 usage = 0;

#if IS_ENABLED(CONFIG_EXYNOS_DIT)
	int ret = dit_get_src_usage(DIT_DIR_RX, &usage);

	if (ret && (ret != -EPERM)) {
		mif_err_limited("dit_get_src_usage() error:%d\n", ret);
		return ret;
	}
#endif

	tpmon->q_status_dit_src = usage;

	return 0;
}

static int tpmon_calc_q_status_netdev_backlog(struct cpif_tpmon *tpmon)
{
	struct softnet_data *sd = NULL;
	u32 usage = 0;
	int i;
	int num_cpu;

#if defined(CONFIG_VENDOR_NR_CPUS)
	num_cpu = CONFIG_VENDOR_NR_CPUS;
#else
	num_cpu = 8;
#endif
	for (i = 0; i < num_cpu; i++) {
		sd = &per_cpu(softnet_data, i);
		if (sd->input_queue_tail > sd->input_queue_head)
			usage += sd->input_queue_tail - sd->input_queue_head;
	}

	tpmon->q_status_netdev_backlog = usage;

	return 0;
}

static void tpmon_calc_q_status(struct cpif_tpmon *tpmon)
{
	tpmon_calc_q_status_pktproc_dl(tpmon);
	tpmon_calc_q_status_dit_src(tpmon);
	tpmon_calc_q_status_netdev_backlog(tpmon);
}

/* Inforamtion */
static void tpmon_print_stat(struct cpif_tpmon *tpmon)
{
	mif_info_limited("DL:%ldMbps(%ld/%ld/%ld) Q:%d/%d/%d/%d tcp_rmem:%d/%d/%d\n",
		tpmon->rx_total_stat.rx_mbps,
		tpmon->rx_tcp_stat.rx_mbps,
		tpmon->rx_udp_stat.rx_mbps,
		tpmon->rx_others_stat.rx_mbps,
		tpmon->q_status_pktproc_dl,
		tpmon->q_status_dit_src,
		tpmon->q_status_netdev_backlog,
		tpmon->legacy_packet_count,
		init_net.ipv4.sysctl_tcp_rmem[0], init_net.ipv4.sysctl_tcp_rmem[1],
		init_net.ipv4.sysctl_tcp_rmem[2]);

	tpmon->legacy_packet_count = 0;
}

/* Check boost/unboost */
static bool tpmon_check_to_boost(struct tpmon_data *data)
{
	int usage = 0;
	int i;
	struct cpif_tpmon *tpmon = data->tpmon;
	struct tpmon_data *all_data = NULL;

	if (!data->enable)
		return false;

	if (!data->get_data) {
		mif_err_limited("get_data is null:%s\n", data->name);
		return false;
	}

	list_for_each_entry(all_data, &tpmon->all_data_list, data_node) {
		if ((data->target == all_data->target) &&
			(data->measure != all_data->measure) &&
			(data->curr_level_pos < all_data->curr_level_pos)) {
			return false;
		}
	}

	usage = data->get_data(data);
	if (usage < 0) {
		mif_err_limited("get_data(%s) error:%d\n", data->name, usage);
		return false;
	}

	for (i = 0; i < data->num_threshold; i++)
		if (usage < data->threshold[i])
			break;

	if (i <= data->curr_level_pos)
		return false;

	if (i >= data->num_level) {
		mif_err_limited("Invalid level:%s %d %d\n",
			data->name, i, data->num_level);
		return false;
	}

	data->prev_level_pos = data->curr_level_pos;
	data->curr_level_pos = i;

	data->prev_threshold_pos = data->curr_threshold_pos;
	if (data->curr_level_pos)
		data->curr_threshold_pos = data->curr_level_pos - 1;
	else
		data->curr_threshold_pos = 0;

	data->need_boost = true;

	mif_info("%s %d->%d (usage:%d unboost@%dMbps)\n",
		data->name, data->prev_level_pos, data->curr_level_pos,
		usage, data->unboost_threshold_mbps[data->curr_threshold_pos]);

	return true;
}

static bool tpmon_check_to_unboost(struct tpmon_data *data)
{
	ktime_t curr_time;
	u64 delta_msec;

	if (!data->enable)
		return false;

	if (!data->curr_level_pos)
		return false;

	curr_time = ktime_get();
	if (!data->prev_unboost_time) {
		data->prev_unboost_time = curr_time;
		return false;
	}

	if ((tpmon_get_rx_speed_mbps(data) >=
		data->unboost_threshold_mbps[data->curr_threshold_pos])) {
		data->prev_unboost_time = curr_time;
		return false;
	}

	delta_msec = ktime_ms_delta(curr_time, data->prev_unboost_time);
	if (delta_msec < data->tpmon->boost_hold_msec)
		return false;

	data->prev_level_pos = data->curr_level_pos;
	if (data->curr_level_pos > 0)
		data->curr_level_pos--;

	data->prev_threshold_pos = data->curr_threshold_pos;
	if (data->curr_threshold_pos > 0)
		data->curr_threshold_pos--;

	mif_info("%s %d->%d (%ldMbps < %dMbps)\n",
		data->name, data->prev_level_pos, data->curr_level_pos,
		data->tpmon->rx_total.rx_mbps,
		data->unboost_threshold_mbps[data->prev_threshold_pos]);

	data->prev_unboost_time = 0;

	return true;
}

static void tpmon_get_cpu_per_queue(u32 mask, u32 *q, unsigned int q_num,
				    bool get_mask)
{
	u32 cur_mask = mask, bit_pos = 0;
	bool masks_lt_qnum = false;
	unsigned int idx = 0;

	if (!mask)
		return;

	while (cur_mask || idx < q_num) {
		u32 bit_mask;

		if (!cur_mask) {
			cur_mask = mask;
			bit_pos = 0;

			if (idx < q_num)
				masks_lt_qnum = true;
		}

		bit_mask = (u32)BIT(bit_pos);

		if (bit_mask & cur_mask) {
			cur_mask &= ~bit_mask;
			if (get_mask)
				q[idx % q_num] |= bit_mask;
			else
				q[idx % q_num] = bit_pos;
			idx++;
		}

		if (masks_lt_qnum && idx == q_num)
			cur_mask = 0;

		bit_pos++;
	}
}

/*
 * Target
 */
/* RPS */
#if IS_ENABLED(CONFIG_RPS)
/* From net/core/net-sysfs.c */
static ssize_t tpmon_store_rps_map(struct netdev_rx_queue *queue,
						const char *buf, ssize_t len)
{
	struct rps_map *old_map, *map;
	cpumask_var_t mask;
	int err, cpu, i;
	static DEFINE_MUTEX(rps_map_mutex);

	if (!alloc_cpumask_var(&mask, GFP_KERNEL))
		return -ENOMEM;

	err = bitmap_parse(buf, len, cpumask_bits(mask), nr_cpumask_bits);
	if (err) {
		free_cpumask_var(mask);
		return err;
	}

	map = kzalloc(max_t(unsigned int,
			    RPS_MAP_SIZE(cpumask_weight(mask)), L1_CACHE_BYTES),
		      GFP_KERNEL);
	if (!map) {
		free_cpumask_var(mask);
		return -ENOMEM;
	}

	i = 0;
	for_each_cpu_and(cpu, mask, cpu_online_mask)
		map->cpus[i++] = cpu;

	if (i) {
		map->len = i;
	} else {
		kfree(map);
		map = NULL;
	}

	mutex_lock(&rps_map_mutex);
	old_map = rcu_dereference_protected(queue->rps_map,
					    mutex_is_locked(&rps_map_mutex));
	rcu_assign_pointer(queue->rps_map, map);

	if (map)
		static_branch_inc(&rps_needed);
	if (old_map)
		static_branch_dec(&rps_needed);

	mutex_unlock(&rps_map_mutex);

	if (old_map)
		kfree_rcu(old_map, rcu);

	free_cpumask_var(mask);
	return len;
}

static void tpmon_set_rps(struct tpmon_data *data)
{
#if IS_ENABLED(CONFIG_CP_PKTPROC)
	struct mem_link_device *mld = container_of(data->tpmon->ld,
						   struct mem_link_device, link_dev);
	struct pktproc_adaptor *ppa = &mld->pktproc;
#endif
	struct io_device *iod;
	unsigned int num_queue = 1;
	unsigned int i, len;
	u32 val, *rxq_mask;

	if (!data->enable)
		return;

#if IS_ENABLED(CONFIG_CP_PKTPROC)
	if (ppa->use_exclusive_irq)
		num_queue = ppa->num_queue;
#endif

	rxq_mask = kzalloc(sizeof(u32) * num_queue, GFP_KERNEL);
	if (!rxq_mask)
		return;

	val = tpmon_get_curr_level(data);
	tpmon_get_cpu_per_queue(val, rxq_mask, num_queue, true);

	list_for_each_entry(iod, &data->tpmon->net_node_list, node_all_ndev) {
		char mask[MAX_RPS_STRING] = {};
		unsigned long flags;
		int ret;

		if (!iod->name)
			continue;

		if (!iod->ndev)
			continue;

		for (i = 0; i < num_queue; i++) {
			len = scnprintf(mask, MAX_RPS_STRING, "%x",
					rxq_mask[i]);

			ret = (int)tpmon_store_rps_map(&iod->ndev->_rx[i],
					mask, len);
			if (ret < 0) {
				mif_err("tpmon_store_rps_map() error:%d\n", ret);
				goto out;
			}
		}

		spin_lock_irqsave(&iod->clat_lock, flags);
		if (!iod->clat_ndev) {
			spin_unlock_irqrestore(&iod->clat_lock, flags);
			continue;
		}

		dev_hold(iod->clat_ndev);
		spin_unlock_irqrestore(&iod->clat_lock, flags);

		len = scnprintf(mask, MAX_RPS_STRING, "%x", val);
		ret = (int)tpmon_store_rps_map(&(iod->clat_ndev->_rx[0]),
			mask, len);
		dev_put(iod->clat_ndev);

		if (ret < 0) {
			mif_err("tpmon_store_rps_map() clat error:%d\n", ret);
			break;
		}
	}

	for (i = 0; i < num_queue; i++)
		mif_info("%s (rxq[%u] mask:0x%02x)\n", data->name, i, rxq_mask[i]);

out:
	kfree(rxq_mask);
}
#endif

/* GRO flush timeout */
static void tpmon_set_gro(struct tpmon_data *data)
{
	struct mem_link_device *mld = container_of(data->tpmon->ld,
			struct mem_link_device, link_dev);
	long timeout;

#if IS_ENABLED(CONFIG_CP_PKTPROC)
	struct pktproc_adaptor *ppa = &mld->pktproc;
	int i;
#endif

	if (!data->enable)
		return;

	timeout = tpmon_get_curr_level(data);

#if IS_ENABLED(CONFIG_CP_PKTPROC)
	if (ppa->use_exclusive_irq) {
		for (i = 0; i < ppa->num_queue; i++) {
			struct pktproc_queue *q = ppa->q[i];

			q->netdev.gro_flush_timeout = timeout;
		}
	} else {
		mld->dummy_net.gro_flush_timeout = timeout;
	}
#else
	mld->dummy_net.gro_flush_timeout = timeout;
#endif

#if IS_ENABLED(CONFIG_EXYNOS_DIT)
	if (dit_get_netdev()) {
		dit_get_netdev()->gro_flush_timeout = timeout;
		mld->dummy_net.gro_flush_timeout = 0;
	}
#endif

	mif_info("%s (flush timeout:%ld)\n", data->name, timeout);
}

/* IRQ affinity */
#if IS_ENABLED(CONFIG_MCU_IPC)
static void tpmon_set_irq_affinity_mbox(struct tpmon_data *data)
{
#if IS_ENABLED(CONFIG_CP_PKTPROC)
	struct mem_link_device *mld = ld_to_mem_link_device(data->tpmon->ld);
	struct pktproc_adaptor *ppa = &mld->pktproc;
#endif
	unsigned int num_queue = 1;
	unsigned int i;
	u32 val, *q_cpu;

	if (!data->enable)
		return;

#if IS_ENABLED(CONFIG_CP_PKTPROC)
	if (ppa->use_exclusive_irq)
		num_queue = ppa->num_queue;
#endif

	q_cpu = kzalloc(sizeof(u32) * num_queue, GFP_KERNEL);
	if (!q_cpu)
		return;
	}

	val = tpmon_get_curr_level(data);
	tpmon_get_cpu_per_queue(val, q_cpu, num_queue, false);

	for (i = 0; i < num_queue; i++) {
		int irq_idx = data->extra_idx;

#if IS_ENABLED(CONFIG_CP_PKTPROC)
		if (ppa->use_exclusive_irq)
			irq_idx = ppa->q[i]->irq_idx;
#endif

		if (cp_mbox_get_affinity(irq_idx) == q_cpu[i]) {
			mif_info("skip to set same cpu_num for %s (CPU:%u)\n",
				 data->name, q_cpu[i]);
			continue;
		}

		mif_info("%s (CPU:%u)\n", data->name, val);
		cp_mbox_set_affinity(irq_idx, q_cpu[i]);
	}

	kfree(q_cpu);
}
#endif

#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE)
static void tpmon_set_irq_affinity_pcie(struct tpmon_data *data)
{
	struct mem_link_device *mld = ld_to_mem_link_device(data->tpmon->ld);
	struct modem_ctl *mc = data->tpmon->ld->mc;
#if IS_ENABLED(CONFIG_CP_PKTPROC)
	struct pktproc_adaptor *ppa = &mld->pktproc;
	unsigned int num_queue = 1;
	unsigned int i;
	u32 val, *q_cpu;
#endif

	if (!data->enable)
		return;

#if IS_ENABLED(CONFIG_CP_PKTPROC)
	if (ppa->use_exclusive_irq)
		num_queue = ppa->num_queue;

	q_cpu = kzalloc(sizeof(u32) * num_queue, GFP_KERNEL);
	if (!q_cpu)
		return;

	val = tpmon_get_curr_level(data);
	tpmon_get_cpu_per_queue(val, q_cpu, num_queue, false);

	for (i = 0; i < num_queue; i++) {
		if (!ppa->q[i]->irq)
			break;

		if (!ppa->use_exclusive_irq)
			q_cpu[i] = data->extra_idx;

		mif_info("%s (q[%u] cpu:%u)\n", data->name, i, q_cpu[i]);
		mld->msi_irq_q_cpu[i] = q_cpu[i];
	}

	kfree(q_cpu);
#endif

	/* The affinity of msi_irq_base is fixed, use the extra_idx */
	mld->msi_irq_base_cpu = data->extra_idx;
	s5100_set_pcie_irq_affinity(mc);
}
#endif

#if IS_ENABLED(CONFIG_EXYNOS_DIT)
static void tpmon_set_irq_affinity_dit(struct tpmon_data *data)
{
	u32 val, cpu[1];

	if (!data->enable)
		return;

	val = tpmon_get_curr_level(data);
	tpmon_get_cpu_per_queue(val, cpu, 1, false);

	if (dit_get_irq_affinity() == cpu[0]) {
		mif_info("skip to set same cpu_num for %s (CPU:%u)\n",
			 data->name, cpu[0]);
		return;
	}

	mif_info("%s (CPU:%u)\n", data->name, cpu[0]);
	dit_set_irq_affinity(cpu[0]);
}
#endif

/* Frequency */
#if IS_ENABLED(CONFIG_EXYNOS_PM_QOS)
static void tpmon_set_exynos_pm_qos(struct tpmon_data *data)
{
	u32 val;

	if (!data->enable)
		return;

	if (!data->extra_data)
		return;

	val = tpmon_get_curr_level(data);

	mif_info("%s (freq:%d)\n", data->name, val);

	exynos_pm_qos_update_request(data->extra_data, val);
}
#endif

#if IS_ENABLED(CONFIG_CPU_FREQ)
static void tpmon_set_cpu_freq(struct tpmon_data *data)
{
	u32 val;

	if (!data->enable)
		return;

	if (!data->extra_data)
		return;

	val = tpmon_get_curr_level(data);

	mif_info("%s (freq:%d)\n", data->name, val);

	freq_qos_update_request((struct freq_qos_request *)data->extra_data, val);
}

static int tpmon_cpufreq_nb(struct notifier_block *nb,
		unsigned long event, void *arg)
{
	struct cpufreq_policy *policy = arg;
	struct cpif_tpmon *tpmon = &_tpmon;
	struct tpmon_data *data;

	if (event != CPUFREQ_CREATE_POLICY)
		return NOTIFY_OK;

	list_for_each_entry(data, &tpmon->all_data_list, data_node) {
		switch (data->target) {
		case TPMON_TARGET_CPU_CL0:
		case TPMON_TARGET_CPU_CL1:
		case TPMON_TARGET_CPU_CL2:
			if (policy->cpu == data->extra_idx) {
				mif_info("freq_qos_add_request for cpu%d min\n", policy->cpu);
#if IS_ENABLED(CONFIG_ARM_FREQ_QOS_TRACER)
				freq_qos_tracer_add_request(&policy->constraints,
					data->extra_data, FREQ_QOS_MIN, PM_QOS_DEFAULT_VALUE);
#else
				freq_qos_add_request(&policy->constraints,
					data->extra_data, FREQ_QOS_MIN, PM_QOS_DEFAULT_VALUE);
#endif
			}
			break;
		case TPMON_TARGET_CPU_CL0_MAX:
		case TPMON_TARGET_CPU_CL1_MAX:
		case TPMON_TARGET_CPU_CL2_MAX:
			if (policy->cpu == data->extra_idx) {
				mif_info("freq_qos_add_request for cpu%d max\n", policy->cpu);
#if IS_ENABLED(CONFIG_ARM_FREQ_QOS_TRACER)
				freq_qos_tracer_add_request(&policy->constraints,
					data->extra_data, FREQ_QOS_MAX, PM_QOS_DEFAULT_VALUE);
#else
				freq_qos_add_request(&policy->constraints,
					data->extra_data, FREQ_QOS_MAX, PM_QOS_DEFAULT_VALUE);
#endif
			}
			break;
		default:
			break;
		}
	}

	return NOTIFY_OK;
}
#endif

/* PCIe power */
#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE)
static void tpmon_set_pci_low_power(struct tpmon_data *data)
{
	struct modem_ctl *mc = data->tpmon->ld->mc;
	u32 val;

	if (!data->enable)
		return;

	mutex_lock(&mc->pcie_check_lock);
	if (!mc->pcie_powered_on || s51xx_check_pcie_link_status(mc->pcie_ch_num) == 0)
		goto out;

	val = tpmon_get_curr_level(data);
	mif_info("%s (enable:%u)\n", data->name, val);
	s51xx_pcie_l1ss_ctrl((int)val, mc->pcie_ch_num);

out:
	mutex_unlock(&mc->pcie_check_lock);
}
#endif

/* Bus */
#if IS_ENABLED(CONFIG_EXYNOS_BTS)
static void tpmon_set_bts(struct tpmon_data *data)
{
	u32 val;

	if (!data->enable)
		return;

	val = tpmon_get_curr_level(data);

	mif_info("%s (val:%d)\n", data->name, val);

	if (val)
		bts_add_scenario(data->tpmon->bts_scen_index);
	else
		bts_del_scenario(data->tpmon->bts_scen_index);
}
#endif

/*
 * Work
 */
/* Monitor work */
static void tpmon_monitor_work(struct work_struct *ws)
{
	struct cpif_tpmon *tpmon = container_of(ws,
		struct cpif_tpmon, monitor_dwork.work);
	struct tpmon_data *data;
	ktime_t curr_time;
	u64 delta_msec;

	if (tpmon_check_active()) {
		tpmon_stat_rx_speed(tpmon);
		tpmon_calc_q_status(tpmon);
		tpmon_print_stat(tpmon);
	}

	if (tpmon->use_user_level)
		goto run_again;

	list_for_each_entry(data, &tpmon->all_data_list, data_node) {
		if (atomic_read(&tpmon->need_init)) {
			data->set_data(data);
			continue;
		}

		if (tpmon_check_to_unboost(data))
			data->set_data(data);
	}

	if (atomic_read(&tpmon->need_init)) {
		atomic_set(&tpmon->need_init, 0);
		return;
	}

	curr_time = ktime_get();
	if (!tpmon->prev_monitor_time)
		tpmon->prev_monitor_time = curr_time;

	if (tpmon->rx_total_stat.rx_mbps >= tpmon->monitor_stop_mbps) {
		tpmon->prev_monitor_time = 0;
		goto run_again;
	}

	delta_msec = ktime_ms_delta(curr_time, tpmon->prev_monitor_time);
	if (delta_msec < tpmon->monitor_hold_msec)
		goto run_again;

	if (tpmon_check_active())
		tpmon_stop();

	mif_info("monitor is stopped\n");

	return;

run_again:
	queue_delayed_work(tpmon->monitor_wq, &tpmon->monitor_dwork,
		msecs_to_jiffies(tpmon->monitor_interval_msec));
}

/* Boost work */
static void tpmon_boost_work(struct work_struct *ws)
{
	struct cpif_tpmon *tpmon = container_of(ws,
		struct cpif_tpmon, boost_dwork.work);
	struct tpmon_data *data;

	list_for_each_entry(data, &tpmon->all_data_list, data_node) {
		if (!data->set_data) {
			mif_err_limited("set_data is null:%s\n", data->name);
			continue;
		}

		if (data->need_boost) {
			mif_info("set data name:%s\n", data->name);
			data->set_data(data);
			data->need_boost = false;
		}
	}

	if (!tpmon_check_active()) {
		mif_info("start monitor\n");
		atomic_set(&tpmon->active, 1);
		queue_delayed_work(tpmon->monitor_wq, &tpmon->monitor_dwork, 0);
	}
}

/*
 * Control
 */
void tpmon_add_rx_bytes(struct sk_buff *skb)
{
	struct cpif_tpmon *tpmon = &_tpmon;
	u16 proto = 0;
	unsigned long flags;

	switch (ip_hdr(skb)->version) {
	case 4:
		proto = ip_hdr(skb)->protocol;
		break;
	case 6:
		proto = ipv6_hdr(skb)->nexthdr;
		if (proto == IPPROTO_FRAGMENT)
			proto = skb->data[sizeof(struct ipv6hdr)];
		break;
	default:
		mif_err_limited("Non IPv4/IPv6 packet:0x%x\n",
			ip_hdr(skb)->version);
		break;
	}

	spin_lock_irqsave(&tpmon->lock, flags);
	tpmon->rx_total.rx_bytes += skb->len;
	tpmon->rx_total_stat.rx_bytes += skb->len;
	switch (proto) {
	case IPPROTO_TCP:
		tpmon->rx_tcp.rx_bytes += skb->len;
		tpmon->rx_tcp_stat.rx_bytes += skb->len;
		break;
	case IPPROTO_UDP:
		tpmon->rx_udp.rx_bytes += skb->len;
		tpmon->rx_udp_stat.rx_bytes += skb->len;
		break;
	default:
		tpmon->rx_others.rx_bytes += skb->len;
		tpmon->rx_others_stat.rx_bytes += skb->len;
		break;
	}
	spin_unlock_irqrestore(&tpmon->lock, flags);
}
EXPORT_SYMBOL(tpmon_add_rx_bytes);

void tpmon_add_legacy_packet_count(u32 count)
{
	struct cpif_tpmon *tpmon = &_tpmon;

	tpmon->legacy_packet_count += count;
}
EXPORT_SYMBOL(tpmon_add_legacy_packet_count);

void tpmon_add_net_node(struct list_head *node)
{
	struct cpif_tpmon *tpmon = &_tpmon;
	unsigned long flags;

	spin_lock_irqsave(&tpmon->lock, flags);

	list_add_tail(node, &tpmon->net_node_list);

	spin_unlock_irqrestore(&tpmon->lock, flags);
}
EXPORT_SYMBOL(tpmon_add_net_node);

void tpmon_reset_data(char *name)
{
	struct cpif_tpmon *tpmon = &_tpmon;
	struct tpmon_data *data;

	list_for_each_entry(data, &tpmon->all_data_list, data_node) {
		if (strncmp(data->name, name, strlen(name)) == 0) {
			data->set_data(data);
			break;
		}
	}
}
EXPORT_SYMBOL(tpmon_reset_data);

/* Init */
static int tpmon_init_params(struct cpif_tpmon *tpmon)
{
	struct tpmon_data *data;

	memset(&tpmon->rx_total, 0, sizeof(struct cpif_rx_data));
	memset(&tpmon->rx_tcp, 0, sizeof(struct cpif_rx_data));
	memset(&tpmon->rx_udp, 0, sizeof(struct cpif_rx_data));
	memset(&tpmon->rx_others, 0, sizeof(struct cpif_rx_data));

	memset(&tpmon->rx_total_stat, 0, sizeof(struct cpif_rx_data));
	memset(&tpmon->rx_tcp_stat, 0, sizeof(struct cpif_rx_data));
	memset(&tpmon->rx_udp_stat, 0, sizeof(struct cpif_rx_data));
	memset(&tpmon->rx_others_stat, 0, sizeof(struct cpif_rx_data));

	tpmon->q_status_pktproc_dl = 0;
	tpmon->q_status_netdev_backlog = 0;
	tpmon->q_status_dit_src = 0;
	tpmon->legacy_packet_count = 0;

	tpmon->prev_monitor_time = 0;

	list_for_each_entry(data, &tpmon->all_data_list, data_node) {
		data->curr_threshold_pos = 0;
		data->prev_threshold_pos = 0;
		data->curr_level_pos = 0;
		data->prev_level_pos = 0;
		data->prev_unboost_time = 0;
		data->need_boost = false;
	}

	return 0;
}

static void tpmon_check_q_status(struct cpif_tpmon *tpmon)
{
	struct tpmon_data *data;
	bool run_work = false;

	list_for_each_entry(data, &tpmon->q_status_list, q_status_node) {
		if (data->need_boost)
			continue;

		if (!tpmon_check_to_boost(data))
			continue;

		mif_debug("need to run work for %s\n", data->name);
		run_work = true;
	}

	if (run_work)
		queue_delayed_work(tpmon->boost_wq, &tpmon->boost_dwork, 0);
}

static void tpmon_check_tp_status(struct cpif_tpmon *tpmon)
{
	struct tpmon_data *data;
	bool run_work = false;

	list_for_each_entry(data, &tpmon->tp_node_list, tp_node) {
		if (data->need_boost)
			continue;

		if (!tpmon_check_to_boost(data))
			continue;

		mif_debug("need to run work for %s\n", data->name);
		run_work = true;
	}

	if (run_work)
		queue_delayed_work(tpmon->boost_wq, &tpmon->boost_dwork, 0);
}

int tpmon_start(void)
{
	struct cpif_tpmon *tpmon = &_tpmon;

	if (tpmon->use_user_level)
		return 0;

	tpmon_calc_q_status(tpmon);
	tpmon_check_q_status(tpmon);

	tpmon_calc_rx_speed(tpmon);
	tpmon_check_tp_status(tpmon);

	return 0;
}
EXPORT_SYMBOL(tpmon_start);

int tpmon_stop(void)
{
	struct cpif_tpmon *tpmon = &_tpmon;
	struct tpmon_data *data;

	if (!tpmon_check_active())
		return 0;

	cancel_delayed_work(&tpmon->boost_dwork);
	cancel_delayed_work(&tpmon->monitor_dwork);

	atomic_set(&tpmon->active, 0);

	list_for_each_entry(data, &tpmon->all_data_list, data_node) {
		if (data->curr_level_pos != 0) {
			atomic_set(&tpmon->need_init, 1);
			break;
		}
	}

	tpmon_init_params(tpmon);

	if (atomic_read(&tpmon->need_init))
		queue_delayed_work(tpmon->monitor_wq, &tpmon->monitor_dwork, 0);

	return 0;
}
EXPORT_SYMBOL(tpmon_stop);

int tpmon_init(void)
{
	struct cpif_tpmon *tpmon = &_tpmon;

	if (tpmon->use_user_level) {
		mif_info("enable use_user_level again if you want to set user level\n");
		tpmon->use_user_level = 0;
	}

	if (tpmon_check_active())
		tpmon_stop();

	tpmon_init_params(tpmon);

	mif_info("set initial level\n");
	atomic_set(&tpmon->need_init, 1);
	queue_delayed_work(tpmon->monitor_wq, &tpmon->monitor_dwork, 0);

	return 0;
}
EXPORT_SYMBOL(tpmon_init);

int tpmon_check_active(void)
{
	struct cpif_tpmon *tpmon = &_tpmon;

	return atomic_read(&tpmon->active);
}
EXPORT_SYMBOL(tpmon_check_active);

/*
 * sysfs
 */
static ssize_t dt_level_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cpif_tpmon *tpmon = &_tpmon;
	struct tpmon_data *data;
	ssize_t len = 0;
	int i = 0;

	list_for_each_entry(data, &tpmon->all_data_list, data_node) {
		len += scnprintf(buf + len, PAGE_SIZE - len,
			"%s threshold: ", data->name);

		for (i = 0; i < data->num_threshold; i++)
			len += scnprintf(buf + len, PAGE_SIZE - len,
			"%d ", data->threshold[i]);

		len += scnprintf(buf + len, PAGE_SIZE - len, "\n");
		len += scnprintf(buf + len, PAGE_SIZE - len,
			"%s level: ", data->name);

		for (i = 0; i < data->num_level; i++)
			len += scnprintf(buf + len, PAGE_SIZE - len, "0x%x(%d) ",
				data->level[i], data->level[i]);

		len += scnprintf(buf + len, PAGE_SIZE - len, "\n\n");
	}

	return len;
}
static DEVICE_ATTR_RO(dt_level);

static ssize_t curr_level_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cpif_tpmon *tpmon = &_tpmon;
	struct tpmon_data *data;
	ssize_t len = 0;
	int i;

	len += scnprintf(buf + len, PAGE_SIZE - len,
			"trigger min:%dmsec max:%dmsec\n",
			tpmon->trigger_msec_min, tpmon->trigger_msec_max);
	len += scnprintf(buf + len, PAGE_SIZE - len,
			"monitor interval:%dmsec hold:%dmsec stop:%dMbps\n",
			tpmon->monitor_interval_msec,
			tpmon->monitor_hold_msec,
			tpmon->monitor_stop_mbps);
	len += scnprintf(buf + len, PAGE_SIZE - len,
			"boost hold:%dmsec\n", tpmon->boost_hold_msec);

	list_for_each_entry(data, &tpmon->all_data_list, data_node) {
		len += scnprintf(buf + len, PAGE_SIZE - len, "\n");
		len += scnprintf(buf + len, PAGE_SIZE - len,
			"name:%s measure:%d target:%d enable:%d extra_idx:%d proto:%d\n",
			data->name, data->measure,
			data->target, data->enable, data->extra_idx, data->proto);

		len += scnprintf(buf + len, PAGE_SIZE - len,
			"num_threshold:%d\n", data->num_threshold);
		for (i = 0; i < data->num_threshold; i++)
			len += scnprintf(buf + len, PAGE_SIZE - len,
				"%d(unboost@%dMbps) ",
				data->threshold[i], data->unboost_threshold_mbps[i]);
		len += scnprintf(buf + len, PAGE_SIZE - len, "\n");

		len += scnprintf(buf + len, PAGE_SIZE - len,
			"num_level:%d\n", data->num_level);
		for (i = 0; i < data->num_level; i++)
			len += scnprintf(buf + len, PAGE_SIZE - len,
				"%d ", data->level[i]);
		len += scnprintf(buf + len, PAGE_SIZE - len, "\n");

		len += scnprintf(buf + len, PAGE_SIZE - len,
			"curr_level_pos:%d user_level:%d\n",
			data->curr_level_pos, data->user_level);
	}

	return len;
}
static DEVICE_ATTR_RO(curr_level);

static ssize_t status_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cpif_tpmon *tpmon = &_tpmon;
	struct tpmon_data *data;
	ssize_t len = 0;
	u32 val = 0;

	len += scnprintf(buf + len, PAGE_SIZE - len,
			"rx_total %ldbytes %ldMbps\n",
			tpmon->rx_total.rx_bytes, tpmon->rx_total.rx_mbps);
	len += scnprintf(buf + len, PAGE_SIZE - len,
			"rx_tcp %ldbytes %ldMbps\n",
			tpmon->rx_tcp.rx_bytes, tpmon->rx_tcp.rx_mbps);
	len += scnprintf(buf + len, PAGE_SIZE - len,
			"rx_udp %ldbytes %ldMbps\n",
			tpmon->rx_udp.rx_bytes, tpmon->rx_udp.rx_mbps);
	len += scnprintf(buf + len, PAGE_SIZE - len,
			"rx_others %ldbytes %ldMbps\n",
			tpmon->rx_others.rx_bytes, tpmon->rx_others.rx_mbps);

	len += scnprintf(buf + len, PAGE_SIZE - len,
			"queue status pktproc:%d dit:%d netdev:%d legacy:%d\n",
			tpmon->q_status_pktproc_dl,
			tpmon->q_status_dit_src,
			tpmon->q_status_netdev_backlog,
			tpmon->legacy_packet_count);
	len += scnprintf(buf + len, PAGE_SIZE - len,
			"use_user_level:%d debug_print:%d\n",
			tpmon->use_user_level,
			tpmon->debug_print);

	list_for_each_entry(data, &tpmon->all_data_list, data_node) {
		len += scnprintf(buf + len, PAGE_SIZE - len, "%s: enable:%d",
			data->name, data->enable);

		if (!data->enable) {
			len += scnprintf(buf + len, PAGE_SIZE - len, "\n");
			continue;
		}

		val = tpmon_get_curr_level(data);
		len += scnprintf(buf + len, PAGE_SIZE - len, " val:%d(0x%x)",
			val, val);

		if (tpmon->use_user_level)
			len += scnprintf(buf + len, PAGE_SIZE - len, "\n");
		else
			len += scnprintf(buf + len, PAGE_SIZE - len,
				" pos:%d unboost@%dMbps\n",
				data->curr_level_pos,
				data->curr_level_pos ?
					data->unboost_threshold_mbps[data->curr_threshold_pos] : 0);
	}

	return len;
}
static DEVICE_ATTR_RO(status);

static ssize_t use_user_level_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cpif_tpmon *tpmon = &_tpmon;

	return sysfs_emit(buf, "use_user_level:%d\n", tpmon->use_user_level);
}

static ssize_t use_user_level_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct cpif_tpmon *tpmon = &_tpmon;
	struct tpmon_data *data;
	int ret;
	int level;

	ret = kstrtoint(buf, 0, &level);
	if (ret != 0) {
		mif_err("invalid level:%d with %d\n", level, ret);
		return -EINVAL;
	}

	tpmon->use_user_level = level;
	mif_info("use_user_level:%d\n", tpmon->use_user_level);

	list_for_each_entry(data, &tpmon->all_data_list, data_node) {
		if (data->set_data) {
			data->user_level = data->level[data->curr_level_pos];
		}
	}

	return count;
}
static DEVICE_ATTR_RW(use_user_level);

static ssize_t debug_print_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cpif_tpmon *tpmon = &_tpmon;

	return sysfs_emit(buf, "debug pring enable:%d\n", tpmon->debug_print);
}

static ssize_t debug_print_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct cpif_tpmon *tpmon = &_tpmon;
	int ret;
	int level;

	ret = kstrtoint(buf, 0, &level);
	if (ret != 0) {
		mif_err("invalid level:%d with %d\n", level, ret);
		return -EINVAL;
	}

	tpmon->debug_print = level;

	mif_info("debug pring enable:%d\n", tpmon->debug_print);

	return count;
}
static DEVICE_ATTR_RW(debug_print);

static ssize_t set_user_level_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct cpif_tpmon *tpmon = &_tpmon;
	struct tpmon_data *data;
	char name[20];
	int level;
	int ret;

	if (!tpmon->use_user_level) {
		mif_info("use_user_level is not set\n");
		return count;
	}

	ret = sscanf(buf, "%19s %i", name, &level);
	if (ret < 1)
		return -EINVAL;

	mif_info("Change %s to %d(0x%x)\n", name, level, level);

	list_for_each_entry(data, &tpmon->all_data_list, data_node) {
		if (strcmp(data->name, name) == 0) {
			data->user_level = level;
			data->set_data(data);
		}
	}

	return count;
}
static DEVICE_ATTR_WO(set_user_level);

static struct attribute *tpmon_attrs[] = {
	&dev_attr_dt_level.attr,
	&dev_attr_curr_level.attr,
	&dev_attr_status.attr,
	&dev_attr_use_user_level.attr,
	&dev_attr_debug_print.attr,
	&dev_attr_set_user_level.attr,
	NULL,
};

static const struct attribute_group tpmon_group = {
	.attrs = tpmon_attrs,
	.name = "tpmon",
};

/*
 * Init
 */
static int tpmon_set_cpufreq(struct tpmon_data *data)
{
#if IS_ENABLED(CONFIG_CPU_FREQ)
	struct cpif_tpmon *tpmon = data->tpmon;
	struct cpufreq_policy *policy;
	int qos_type;

	switch (data->target) {
	case TPMON_TARGET_CPU_CL0:
		data->extra_data = (void *)&tpmon->qos_req_cpu_cl0;
		data->set_data = tpmon_set_cpu_freq;
		qos_type = FREQ_QOS_MIN;
		break;
	case TPMON_TARGET_CPU_CL0_MAX:
		data->extra_data = (void *)&tpmon->qos_req_cpu_cl0_max;
		data->set_data = tpmon_set_cpu_freq;
		qos_type = FREQ_QOS_MAX;
		break;
	case TPMON_TARGET_CPU_CL1:
		data->extra_data = (void *)&tpmon->qos_req_cpu_cl1;
		data->set_data = tpmon_set_cpu_freq;
		qos_type = FREQ_QOS_MIN;
		break;
	case TPMON_TARGET_CPU_CL1_MAX:
		data->extra_data = (void *)&tpmon->qos_req_cpu_cl1_max;
		data->set_data = tpmon_set_cpu_freq;
		qos_type = FREQ_QOS_MAX;
		break;
	case TPMON_TARGET_CPU_CL2:
		data->extra_data = (void *)&tpmon->qos_req_cpu_cl2;
		data->set_data = tpmon_set_cpu_freq;
		qos_type = FREQ_QOS_MIN;
		break;
	case TPMON_TARGET_CPU_CL2_MAX:
		data->extra_data = (void *)&tpmon->qos_req_cpu_cl2_max;
		data->set_data = tpmon_set_cpu_freq;
		qos_type = FREQ_QOS_MAX;
		break;
	default:
		mif_err_limited("no target\n");
		return -EINVAL;
	}

	if (tpmon->cpufreq_nb.notifier_call) {
		mif_info("notifier_call is registered\n");
		return 0;
	}

	policy = cpufreq_cpu_get(data->extra_idx);
	if (!policy) {
		mif_err_limited("cpufreq_cpu_get() error\n");
		return -EINVAL;
	}

	if (policy->cpu == data->extra_idx) {
		mif_info("freq_qos_add_request for cpu%d %d\n",
			policy->cpu, qos_type);
#if IS_ENABLED(CONFIG_ARM_FREQ_QOS_TRACER)
		freq_qos_tracer_add_request(&policy->constraints,
			data->extra_data, qos_type, PM_QOS_DEFAULT_VALUE);
#else
		freq_qos_add_request(&policy->constraints,
			data->extra_data, qos_type, PM_QOS_DEFAULT_VALUE);
#endif
	}
#endif /* CONFIG_CPU_FREQ */

	return 0;
}

static int tpmon_set_target(struct tpmon_data *data)
{
	struct cpif_tpmon *tpmon = data->tpmon;
	int ret = 0;

	switch (data->target) {
#if IS_ENABLED(CONFIG_RPS)
	case TPMON_TARGET_RPS:
		data->set_data = tpmon_set_rps;
		break;
#endif

	case TPMON_TARGET_GRO:
		data->set_data = tpmon_set_gro;
		break;

#if IS_ENABLED(CONFIG_EXYNOS_PM_QOS)
	case TPMON_TARGET_MIF:
		data->extra_data = (void *)&tpmon->qos_req_mif;
		data->set_data = tpmon_set_exynos_pm_qos;
		break;
	case TPMON_TARGET_MIF_MAX:
		data->extra_data = (void *)&tpmon->qos_req_mif_max;
		data->set_data = tpmon_set_exynos_pm_qos;
		break;
	case TPMON_TARGET_INT_FREQ:
		data->extra_data = (void *)&tpmon->qos_req_int;
		data->set_data = tpmon_set_exynos_pm_qos;
		break;
	case TPMON_TARGET_INT_FREQ_MAX:
		data->extra_data = (void *)&tpmon->qos_req_int_max;
		data->set_data = tpmon_set_exynos_pm_qos;
		break;
#endif

#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE)
	case TPMON_TARGET_PCIE_LOW_POWER:
		data->set_data = tpmon_set_pci_low_power;
		break;
	case TPMON_TARGET_IRQ_PCIE:
		data->set_data = tpmon_set_irq_affinity_pcie;
		break;
#endif

#if IS_ENABLED(CONFIG_MCU_IPC)
	case TPMON_TARGET_IRQ_MBOX:
		data->set_data = tpmon_set_irq_affinity_mbox;
		break;
#endif

#if IS_ENABLED(CONFIG_EXYNOS_DIT)
	case TPMON_TARGET_IRQ_DIT:
		data->set_data = tpmon_set_irq_affinity_dit;
		break;
#endif

#if IS_ENABLED(CONFIG_EXYNOS_BTS)
	case TPMON_TARGET_BTS:
		data->set_data = tpmon_set_bts;
		break;
#endif

#if IS_ENABLED(CONFIG_CPU_FREQ)
	case TPMON_TARGET_CPU_CL0:
	case TPMON_TARGET_CPU_CL0_MAX:
	case TPMON_TARGET_CPU_CL1:
	case TPMON_TARGET_CPU_CL1_MAX:
	case TPMON_TARGET_CPU_CL2:
	case TPMON_TARGET_CPU_CL2_MAX:
		ret = tpmon_set_cpufreq(data);
		if (ret) {
			mif_err("tpmon_set_cpufreq() error:%d\n", ret);
			return ret;
		}
		break;
#endif

	default:
		mif_err("%s target error:%d\n", data->name, data->target);
		return -EINVAL;
	}

	return 0;
}

static int tpmon_parse_dt(struct device_node *np, struct cpif_tpmon *tpmon)
{
	struct device_node *tpmon_np = NULL;
	struct device_node *child_np = NULL;
	struct device_node *boost_np = NULL;
	struct tpmon_data *data = NULL;
	int ret = 0;
	u32 count = 0;
	unsigned long flags;

	tpmon_np = of_get_child_by_name(np, "cpif_tpmon");
	if (!tpmon_np) {
		mif_err("tpmon_np is null\n");
		return -ENODEV;
	}

	mif_dt_read_u32(tpmon_np, "trigger_msec_min", tpmon->trigger_msec_min);
	mif_dt_read_u32(tpmon_np, "trigger_msec_max", tpmon->trigger_msec_max);
	mif_info("trigger min:%dmsec max:%dmsec\n",
		tpmon->trigger_msec_min, tpmon->trigger_msec_max);

	mif_dt_read_u32(tpmon_np, "monitor_interval_msec",
		tpmon->monitor_interval_msec);
	mif_dt_read_u32(tpmon_np, "monitor_hold_msec",
		tpmon->monitor_hold_msec);
	mif_dt_read_u32(tpmon_np, "monitor_stop_mbps",
		tpmon->monitor_stop_mbps);
	mif_info("monitor interval:%dmsec hold:%dmsec stop:%dmbps\n",
		tpmon->monitor_interval_msec, tpmon->monitor_hold_msec,
		tpmon->monitor_stop_mbps);

	mif_dt_read_u32(tpmon_np, "boost_hold_msec", tpmon->boost_hold_msec);
	mif_info("boost hold:%dmsec\n", tpmon->boost_hold_msec);

	for_each_child_of_node(tpmon_np, child_np) {
		struct tpmon_data child_data = {};

		mif_dt_read_string(child_np, "boost_name", child_data.name);
		mif_dt_read_u32(child_np, "target", child_data.target);
		mif_dt_read_u32(child_np, "extra_idx", child_data.extra_idx);
		mif_dt_count_u32_elems(child_np, "level", child_data.num_level);
		mif_dt_count_u32_array(child_np, "level",
			child_data.level, child_data.num_level);

		/* boost */
		for_each_child_of_node(child_np, boost_np) {
			if (count >= MAX_TPMON_DATA) {
				mif_err("count is full:%d\n", count);
				return -EINVAL;
			}

			data = &tpmon->data[count];
			memcpy(data, &child_data, sizeof(child_data));
			data->tpmon = tpmon;

			/* check enabled */
			mif_dt_read_u32(boost_np, "enable", data->enable);
			if (!data->enable)
				continue;

			/* threshold */
			mif_dt_count_u32_elems(boost_np, "boost_threshold",
				data->num_threshold);
			mif_dt_count_u32_array(boost_np, "boost_threshold",
				data->threshold, data->num_threshold);
			mif_dt_count_u32_array(boost_np, "unboost_threshold_mbps",
				data->unboost_threshold_mbps, data->num_threshold);

			/* target */
			ret = tpmon_set_target(data);
			if (ret) {
				mif_err("tpmon_set_target() error:%d\n", ret);
				continue;
			}

			/* measure */
			mif_dt_read_u32(boost_np, "proto", data->proto);
			mif_dt_read_u32(boost_np, "measure", data->measure);
			spin_lock_irqsave(&tpmon->lock, flags);
			switch (data->measure) {
			case TPMON_MEASURE_TP:
				data->get_data = tpmon_get_rx_speed_mbps;
				list_add_tail(&data->tp_node, &tpmon->tp_node_list);
				break;
			case TPMON_MEASURE_NETDEV_Q:
			case TPMON_MEASURE_PKTPROC_DL_Q:
			case TPMON_MEASURE_DIT_SRC_Q:
				data->get_data = tpmon_get_q_status;
				list_add_tail(&data->q_status_node, &tpmon->q_status_list);
				break;
			default:
				mif_err("%s measure error:%d %d\n",
					data->name, count, data->measure);
				spin_unlock_irqrestore(&tpmon->lock, flags);
				return -EINVAL;
			}
			list_add_tail(&data->data_node, &tpmon->all_data_list);
			spin_unlock_irqrestore(&tpmon->lock, flags);

			mif_info("name:%s measure:%d target:%d extra_idx:%d level:%d/%d proto:%d\n",
				 data->name, data->measure, data->target, data->extra_idx,
				 data->num_threshold, data->num_level, data->proto);

			count++;
		}
	}

	return 0;
}

int tpmon_create(struct platform_device *pdev, struct link_device *ld)
{
	struct device_node *np = pdev->dev.of_node;
	struct cpif_tpmon *tpmon = &_tpmon;
	struct mem_link_device *mld = ld_to_mem_link_device(ld);
	int ret = 0;
#if IS_ENABLED(CONFIG_CPU_FREQ)
	struct cpufreq_policy pol;
#endif

	if (!np) {
		mif_err("np is null\n");
		ret = -EINVAL;
		goto create_error;
	}
	if (!ld) {
		mif_err("ld is null\n");
		ret = -EINVAL;
		goto create_error;
	}

	tpmon->ld = ld;
	tpmon->use_user_level = 0;
	tpmon->debug_print = 0;
	mld->tpmon = &_tpmon;

	spin_lock_init(&tpmon->lock);
	atomic_set(&tpmon->active, 0);

	INIT_LIST_HEAD(&tpmon->all_data_list);
	INIT_LIST_HEAD(&tpmon->tp_node_list);
	INIT_LIST_HEAD(&tpmon->q_status_list);
	INIT_LIST_HEAD(&tpmon->net_node_list);

#if IS_ENABLED(CONFIG_CPU_FREQ)
	if (cpufreq_get_policy(&pol, 0) != 0) {
		mif_info("register cpufreq notifier\n");
		tpmon->cpufreq_nb.notifier_call = tpmon_cpufreq_nb;
		cpufreq_register_notifier(&tpmon->cpufreq_nb, CPUFREQ_POLICY_NOTIFIER);
	}
#endif

#if IS_ENABLED(CONFIG_EXYNOS_PM_QOS)
	exynos_pm_qos_add_request(&tpmon->qos_req_mif,
		PM_QOS_BUS_THROUGHPUT, 0);
	exynos_pm_qos_add_request(&tpmon->qos_req_mif_max,
		PM_QOS_BUS_THROUGHPUT_MAX,
		PM_QOS_BUS_THROUGHPUT_MAX_DEFAULT_VALUE);
	exynos_pm_qos_add_request(&tpmon->qos_req_int,
		PM_QOS_DEVICE_THROUGHPUT, 0);
	exynos_pm_qos_add_request(&tpmon->qos_req_int_max,
		PM_QOS_DEVICE_THROUGHPUT_MAX,
		PM_QOS_DEVICE_THROUGHPUT_MAX_DEFAULT_VALUE);
#endif

#if IS_ENABLED(CONFIG_EXYNOS_BTS)
	tpmon->bts_scen_index = bts_get_scenindex("cp_throughput");
#endif

	ret = tpmon_parse_dt(np, tpmon);
	if (ret) {
		mif_err("tpmon_parse_dt() error:%d\n", ret);
		goto create_error;
	}

	tpmon->monitor_wq = alloc_workqueue("cpif_tpmon_monitor_wq",
					__WQ_LEGACY | WQ_MEM_RECLAIM | WQ_UNBOUND, 1);
	if (!tpmon->monitor_wq) {
		mif_err("create_workqueue() monitor_wq error\n");
		ret = -EINVAL;
		goto create_error;
	}
	INIT_DELAYED_WORK(&tpmon->monitor_dwork, tpmon_monitor_work);

	tpmon->boost_wq = alloc_workqueue("cpif_tpmon_boost_wq",
					__WQ_LEGACY | WQ_MEM_RECLAIM | WQ_UNBOUND, 1);
	if (!tpmon->boost_wq) {
		mif_err("create_workqueue() boost_wq error\n");
		ret = -EINVAL;
		goto create_error;
	}
	INIT_DELAYED_WORK(&tpmon->boost_dwork, tpmon_boost_work);

	tpmon->start = tpmon_start;
	tpmon->stop = tpmon_stop;
	tpmon->add_rx_bytes = tpmon_add_rx_bytes;
	tpmon->check_active = tpmon_check_active;
	tpmon->reset_data = tpmon_reset_data;

	if (sysfs_create_group(&pdev->dev.kobj, &tpmon_group))
		mif_err("failed to create cpif tpmon groups node\n");

	return ret;

create_error:
	mif_err("Error:%d\n", ret);

	return ret;
}
EXPORT_SYMBOL(tpmon_create);
