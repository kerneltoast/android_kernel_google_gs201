// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019-2020, Samsung Electronics.
 *
 */

#include "modem_prj.h"
#include "modem_utils.h"
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
static u32 tpmon_get_pktproc_queue_status(struct tpmon_data *data)
{
	if (!data->enable)
		return 0;

	return data->tpmon->pktproc_queue_status;
}

static u32 tpmon_get_netdev_backlog_queue_status(struct tpmon_data *data)
{
	if (!data->enable)
		return 0;

	return data->tpmon->netdev_backlog_queue_status;
}

#if IS_ENABLED(CONFIG_EXYNOS_DIT)
static u32 tpmon_get_dit_src_queue_status(struct tpmon_data *data)
{
	if (!data->enable)
		return 0;

	return data->tpmon->dit_src_queue_status;
}
#endif

static u32 tpmon_get_rx_speed_mbps(struct tpmon_data *data)
{
	if (!data->enable)
		return 0;

	return (u32)data->tpmon->rx_mbps;
}

/* Queue status */
static int tpmon_calc_pktproc_queue_status(struct cpif_tpmon *tpmon)
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

		if (pktproc_get_usage(ppa->q[i]) > 0)
			usage += pktproc_get_usage(ppa->q[i]);
	}

	tpmon->pktproc_queue_status = usage;

	return 0;
}

static int tpmon_calc_netdev_backlog_queue_status(struct cpif_tpmon *tpmon)
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

	tpmon->netdev_backlog_queue_status = usage;

	return 0;
}

#if IS_ENABLED(CONFIG_EXYNOS_DIT)
static int tpmon_calc_dit_src_queue_status(struct cpif_tpmon *tpmon)
{
	u32 usage = 0;
	int ret = 0;

	ret = dit_get_src_usage(DIT_DIR_RX, &usage);
	if (ret) {
		mif_err_limited("dit_get_src_usage() error:%d\n", ret);
		return ret;
	}

	tpmon->dit_src_queue_status = usage;

	return 0;
}
#else
static int tpmon_calc_dit_src_queue_status(struct cpif_tpmon *tpmon)
{
	return 0;
}
#endif

/* RX speed */
static void tpmon_calc_rx_speed(struct cpif_tpmon *tpmon)
{
	unsigned long divider_mbps, divider_kbps;
	unsigned long rx_bytes;
	static unsigned long idx;

	/* mbps 131072 = 1024 * 1024 / 8 */
	/* kbps 128 = 1024 / 8 */
	if (tpmon->monitor_interval_msec >= 1000) {
		divider_mbps = 131072 * tpmon->monitor_interval_msec / 1000;
		divider_kbps = 128 * tpmon->monitor_interval_msec / 1000;
	} else {
		divider_mbps = 131072;
		divider_kbps = 128;
	}

	rx_bytes = tpmon->rx_bytes;
	tpmon->rx_bytes = 0;

	tpmon->rx_sum -= tpmon->rx_bytes_data[idx];
	tpmon->rx_sum += rx_bytes;
	tpmon->rx_bytes_data[idx] = rx_bytes;

	idx++;
	if (tpmon->monitor_interval_msec >= 1000)
		idx = 0;
	else
		idx %= (1000 / tpmon->monitor_interval_msec);

	if (tpmon->rx_sum < divider_mbps)
		tpmon->rx_mbps = 0;
	else
		tpmon->rx_mbps = tpmon->rx_sum / divider_mbps;

	if (tpmon->rx_sum < divider_kbps)
		tpmon->rx_kbps = 0;
	else
		tpmon->rx_kbps = tpmon->rx_sum / divider_kbps;
}

/* Inforamtion */
static void tpmon_print_info(struct cpif_tpmon *tpmon)
{
	static unsigned long idx;

	if (!tpmon->debug_print)
		return;

	idx++;
	if (tpmon->monitor_interval_msec >= 1000)
		idx = 0;
	else
		idx %= (1000 / tpmon->monitor_interval_msec);

	if (idx == 0)
		mif_info("DL:%ldMbps pktproc:%d dit:%d netdev:%d\n",
			tpmon->rx_mbps,
			tpmon->pktproc_queue_status,
			tpmon->dit_src_queue_status,
			tpmon->netdev_backlog_queue_status);
}

/* Check speed changing */
static bool tpmon_check_to_boost(struct tpmon_data *data)
{
	int usage = 0;
	u32 prev_value_pos = 0;
	int i;

	if (!data->enable)
		return false;

	if (!data->get_data) {
		mif_err_limited("get_data is null:%s\n", data->name);
		return false;
	}

	for (i = 0; i < data->num_threshold; i++) {
		usage = data->get_data(data);
		if (usage < 0) {
			mif_err_limited("get_data(%s) error:%d\n", data->name, usage);
			return false;
		}

		if (usage < data->threshold[i])
			break;
	}

	if (i <= data->curr_value_pos)
		return false;

	if (i >= data->num_values) {
		mif_err_limited("Invalid value:%s %d %d\n",
			data->name, i, data->num_values);
		return false;
	}

	data->jiffies_to_unboost = get_jiffies_64();
	prev_value_pos = data->curr_value_pos;
	data->curr_value_pos = i;
	for (i = 0; i < data->curr_value_pos; i++) {
		if (!data->unboost_tp_mbps[i])
			data->unboost_tp_mbps[i] = data->tpmon->rx_mbps *
				data->tpmon->unboost_tp_percent / 100;
	}

	mif_info("%s %d->%d (usage:%d unboost@%dMbps)\n",
		data->name, prev_value_pos, data->curr_value_pos,
		usage, data->unboost_tp_mbps[data->curr_value_pos]);

	return true;
}

static bool tpmon_check_to_unboost(struct tpmon_data *data)
{
	u64 jiffies_curr;
	u64 delta_msec;
	u32 prev_value_pos = 0;

	if (!data->enable)
		return false;

	if (!data->curr_value_pos)
		return false;

	jiffies_curr = get_jiffies_64();
	if (!data->jiffies_to_unboost) {
		data->jiffies_to_unboost = jiffies_curr;
		return false;
	}

	if (jiffies_curr > data->jiffies_to_unboost)
		delta_msec = jiffies64_to_msecs(jiffies_curr - data->jiffies_to_unboost);
	else
		delta_msec = jiffies64_to_msecs(data->jiffies_to_unboost - jiffies_curr);

	if (delta_msec < data->tpmon->boost_hold_msec)
		return false;

	if (data->tpmon->rx_mbps >= data->unboost_tp_mbps[data->curr_value_pos - 1])
		return false;

	data->jiffies_to_unboost = jiffies_curr;

	data->unboost_tp_mbps[data->curr_value_pos] = 0;
	prev_value_pos = data->curr_value_pos;
	if (data->curr_value_pos > 0)
		data->curr_value_pos--;

	mif_info("%s %d->%d (%ldMbps < %dMbps)\n",
		data->name, prev_value_pos, data->curr_value_pos,
		data->tpmon->rx_mbps, data->unboost_tp_mbps[data->curr_value_pos]);

	return true;
}

/*
 * Set data
 */
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

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0))
	if (map)
		static_branch_inc(&rps_needed);
	if (old_map)
		static_branch_dec(&rps_needed);
#else
	if (map)
		static_key_slow_inc(&rps_needed);
	if (old_map)
		static_key_slow_dec(&rps_needed);
#endif

	mutex_unlock(&rps_map_mutex);

	if (old_map)
		kfree_rcu(old_map, rcu);

	free_cpumask_var(mask);
	return len;
}

static void tpmon_set_rps(struct tpmon_data *data)
{
	struct io_device *iod;
	struct mem_link_device *mld;
	struct pktproc_adaptor *ppa;
	int ret = 0;
	char mask[MAX_RPS_STRING];
	u32 rps_value;
	int q_stat;
	int i;

	if (!data->enable)
		return;

	rps_value = data->tpmon->use_user_value ?
			data->user_value : data->values[data->curr_value_pos];
	snprintf(mask, MAX_RPS_STRING, "%x", rps_value);

	mld = to_mem_link_device(data->tpmon->ld);
	ppa = &mld->pktproc;
	for (i = 0; i < ppa->num_queue; i++) {
		if (ppa->use_napi)
			napi_disable(&ppa->q[i]->napi);
		if (ppa->use_exclusive_irq)
			ppa->q[i]->disable_irq(ppa->q[i]);
	}

	for (i = 0; i < 1000; i++) {
		tpmon_calc_netdev_backlog_queue_status(data->tpmon);
		q_stat = data->tpmon->netdev_backlog_queue_status;
		if (q_stat == 0)
			break;

		udelay(100);
	}
	if (q_stat)
		mif_info("can not clear q_stat:%d\n", q_stat);

	list_for_each_entry(iod, &data->tpmon->net_node_list, node_all_ndev) {
		if (!iod->name)
			continue;

		ret = (int)tpmon_store_rps_map(&(iod->ndev->_rx[0]), mask, strlen(mask));
		if (ret < 0) {
			mif_err("tpmon_store_rps_map() error:%d\n", ret);
			break;
		}
	}

	for (i = 0; i < ppa->num_queue; i++) {
		if (ppa->use_napi)
			napi_enable(&ppa->q[i]->napi);
		if (ppa->use_exclusive_irq)
			ppa->q[i]->enable_irq(ppa->q[i]);
	}

	mif_info("%s (mask:0x%s)\n", data->name, mask);
}
#endif

#if IS_ENABLED(CONFIG_MODEM_IF_NET_GRO)
static void tpmon_set_gro(struct tpmon_data *data)
{
	struct mem_link_device *mld = container_of(data->tpmon->ld,
			struct mem_link_device, link_dev);
#if IS_ENABLED(CONFIG_CP_PKTPROC)
	struct pktproc_adaptor *ppa = &mld->pktproc;
	int i;
#endif

	if (!data->enable)
		return;

	gro_flush_time = data->tpmon->use_user_value ?
			data->user_value : data->values[data->curr_value_pos];

	mld->dummy_net.gro_flush_timeout = gro_flush_time;
#if IS_ENABLED(CONFIG_CP_PKTPROC)
	for (i = 0; i > ppa->num_queue; i++) {
		struct pktproc_queue *q = ppa->q[i];

		q->netdev.gro_flush_timeout = gro_flush_time;
	}
#endif

	mif_info("%s (flush time:%u)\n", data->name, gro_flush_time);
}
#endif

/* IRQ affinity */
static int tpmon_stop_napi_poll(struct tpmon_data *data)
{
	struct mem_link_device *mld = ld_to_mem_link_device(data->tpmon->ld);
#if IS_ENABLED(CONFIG_CP_PKTPROC)
	struct pktproc_adaptor *ppa = &mld->pktproc;
	int i;
#endif

	atomic_set(&mld->stop_napi_poll, 1);

#if IS_ENABLED(CONFIG_CP_PKTPROC)
	for (i = 0; i < ppa->num_queue; i++)
		pktproc_stop_napi_poll(ppa, i);
#endif

	return 0;
}

#if IS_ENABLED(CONFIG_LINK_DEVICE_SHMEM) && IS_ENABLED(CONFIG_MCU_IPC)
static void tpmon_set_irq_affinity_mbox(struct tpmon_data *data)
{
	u32 cpu_num;

	if (!data->enable)
		return;

	cpu_num = data->tpmon->use_user_value ? data->user_value :
				data->values[data->curr_value_pos];

	mif_info("%s (CPU:%d)\n", data->name, cpu_num);

	tpmon_stop_napi_poll(data);
	cp_mbox_set_affinity(data->extra_idx, cpu_num);
}
#endif

#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE)
static void tpmon_set_irq_affinity_pcie(struct tpmon_data *data)
{
	struct modem_ctl *mc = data->tpmon->ld->mc;
	u32 cpu_num;

	if (!mc)
		return;

	if (!data->enable)
		return;

	cpu_num = data->tpmon->use_user_value ? data->user_value :
				data->values[data->curr_value_pos];

	mif_info("%s (CPU:%d)\n", data->name, cpu_num);

	tpmon_stop_napi_poll(data);
	exynos_pcie_rc_set_affinity(mc->pcie_ch_num, cpu_num);
}
#endif

#if IS_ENABLED(CONFIG_EXYNOS_DIT)
static void tpmon_set_irq_affinity_dit(struct tpmon_data *data)
{
	u32 cpu_num;

	if (!data->enable)
		return;

	cpu_num = data->tpmon->use_user_value ? data->user_value :
				data->values[data->curr_value_pos];

	mif_info("%s (CPU:%d)\n", data->name, cpu_num);

	dit_set_irq_affinity(cpu_num);
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

	val = data->tpmon->use_user_value ?
			data->user_value : data->values[data->curr_value_pos];

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

	val = data->tpmon->use_user_value ?
			data->user_value : data->values[data->curr_value_pos];

	mif_info("%s (freq:%d)\n", data->name, val);

	freq_qos_update_request((struct freq_qos_request *)data->extra_data, val);
}

static int tpmon_cpufreq_nb(struct notifier_block *nb,
		unsigned long event, void *arg)
{
	struct cpufreq_policy *policy = (struct cpufreq_policy *)arg;
	struct cpif_tpmon *tpmon = &_tpmon;
	struct tpmon_data *data;

	if (event != CPUFREQ_CREATE_POLICY)
		return NOTIFY_OK;

	list_for_each_entry(data, &tpmon->data_list, data_node) {
		switch (data->target) {
		case TPMON_TARGET_CPU_CL0:
		case TPMON_TARGET_CPU_CL1:
		case TPMON_TARGET_CPU_CL2:
			if (policy->cpu == data->extra_idx) {
				mif_info("freq_qos_add_request for cpu%d\n", policy->cpu);
				freq_qos_add_request(&policy->constraints,
					data->extra_data, FREQ_QOS_MIN, PM_QOS_DEFAULT_VALUE);
			}
			break;
		default:
			break;
		}
	}

	return NOTIFY_OK;
}
#endif

#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE)
static void tpmon_set_pci_low_power(struct tpmon_data *data)
{
	struct modem_ctl *mc = data->tpmon->ld->mc;
	u32 pci_low_power_value;

	if (!data->enable)
		return;

	if (!mc || !mc->pcie_powered_on)
		return;

	pci_low_power_value = data->tpmon->use_user_value ?
			data->user_value : data->values[data->curr_value_pos];

	s51xx_pcie_l1ss_ctrl((int)pci_low_power_value, mc->pcie_ch_num);

	mif_info("%s (enable:%u)\n", data->name, pci_low_power_value);
}
#endif

#if IS_ENABLED(CONFIG_EXYNOS_BTS)
static void tpmon_set_bts(struct tpmon_data *data)
{
	u32 val;

	if (!data->enable)
		return;

	val = data->tpmon->use_user_value ?
			data->user_value : data->values[data->curr_value_pos];

	mif_info("%s (val:%d)\n", data->name, val);

	if (val)
		bts_add_scenario(data->tpmon->bts_scen_index);
	else
		bts_del_scenario(data->tpmon->bts_scen_index);
}
#endif

/* Monitor work */
static void tpmon_monitor_work(struct work_struct *ws)
{
	struct cpif_tpmon *tpmon = container_of(ws, struct cpif_tpmon, monitor_dwork.work);
	struct tpmon_data *data;
	u64 jiffies_curr;
	u64 delta_msec;

	if (tpmon_check_active()) {
		tpmon_calc_rx_speed(tpmon);
		tpmon_calc_pktproc_queue_status(tpmon);
		tpmon_calc_netdev_backlog_queue_status(tpmon);
		tpmon_calc_dit_src_queue_status(tpmon);
		tpmon_print_info(tpmon);
	}

	list_for_each_entry(data, &tpmon->data_list, data_node) {
		if (!data->set_data) {
			mif_err_limited("set_data is null:%s\n", data->name);
			continue;
		}

		if (atomic_read(&tpmon->need_init)) {
			data->set_data(data);
			continue;
		}

		if (tpmon_check_to_boost(data)) {
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

	jiffies_curr = get_jiffies_64();
	if (!tpmon->jiffies_to_trigger)
		tpmon->jiffies_to_trigger = jiffies_curr;

	if (tpmon->rx_mbps >= tpmon->trigger_mbps) {
		tpmon->jiffies_to_trigger = jiffies_curr;
		goto run_again;
	}

	if (jiffies_curr > tpmon->jiffies_to_trigger)
		delta_msec = jiffies64_to_msecs(jiffies_curr - tpmon->jiffies_to_trigger);
	else
		delta_msec = jiffies64_to_msecs(tpmon->jiffies_to_trigger - jiffies_curr);

	if (delta_msec < tpmon->monitor_hold_msec)
		goto run_again;

	if (tpmon_check_active())
		tpmon_stop();

	return;

run_again:
	queue_delayed_work_on(3, tpmon->monitor_wq, &tpmon->monitor_dwork,
		msecs_to_jiffies(tpmon->monitor_interval_msec));
}

/*
 * Control
 */
void tpmon_add_rx_bytes(unsigned long bytes)
{
	struct cpif_tpmon *tpmon = &_tpmon;

	tpmon->rx_bytes += bytes;
}
EXPORT_SYMBOL(tpmon_add_rx_bytes);

void tpmon_add_net_node(struct list_head *node)
{
#if IS_ENABLED(CONFIG_RPS)
	struct cpif_tpmon *tpmon = &_tpmon;
	unsigned long flags;

	spin_lock_irqsave(&tpmon->lock, flags);

	list_add_tail(node, &tpmon->net_node_list);

	spin_unlock_irqrestore(&tpmon->lock, flags);
#endif
}
EXPORT_SYMBOL(tpmon_add_net_node);

static int tpmon_init_params(struct cpif_tpmon *tpmon)
{
	struct tpmon_data *data;

	tpmon->rx_bytes = 0;
	tpmon->rx_mbps = 0;
	tpmon->rx_kbps = 0;
	tpmon->rx_sum = 0;
	memset(tpmon->rx_bytes_data, 0, sizeof(tpmon->rx_bytes_data));

	tpmon->pktproc_queue_status = 0;
	tpmon->netdev_backlog_queue_status = 0;
	tpmon->dit_src_queue_status = 0;

	tpmon->jiffies_to_trigger = 0;

	list_for_each_entry(data, &tpmon->data_list, data_node) {
		data->curr_value_pos = 0;
		data->jiffies_to_unboost = 0;
		memset(data->unboost_tp_mbps, 0, sizeof(data->unboost_tp_mbps));
	}

	return 0;
}

static bool tpmon_check_to_start(struct cpif_tpmon *tpmon)
{
	u64 jiffies_curr;
	u64 delta_msec;
	unsigned long mbps, kbps;

	jiffies_curr = get_jiffies_64();
	if (!tpmon->jiffies_to_trigger) {
		tpmon->jiffies_to_trigger = jiffies_curr;
		return false;
	}

	if (jiffies_curr > tpmon->jiffies_to_trigger)
		delta_msec = jiffies64_to_msecs(jiffies_curr - tpmon->jiffies_to_trigger);
	else
		delta_msec = jiffies64_to_msecs(tpmon->jiffies_to_trigger - jiffies_curr);

	if (delta_msec > tpmon->trigger_msec_max) {
		tpmon->jiffies_to_trigger = jiffies_curr;
		tpmon->rx_bytes = 0;
		return false;
	}

	if (delta_msec < tpmon->trigger_msec_min)
		return false;

	if (delta_msec >= 1000) {
		mbps = tpmon->rx_bytes / (131072 * delta_msec / 1000);
		kbps = tpmon->rx_bytes / (128 * delta_msec / 1000);
	} else {
		mbps = (tpmon->rx_bytes * 1000 / delta_msec) / 131072;
		kbps = (tpmon->rx_bytes * 1000 / delta_msec) / 128;
	}

	if (tpmon->debug_print)
		mif_info_limited("%ldMbps %ldKbps delta:%ldmsec rx_bytes:%ld\n",
			mbps, kbps, delta_msec, tpmon->rx_bytes);

	tpmon->jiffies_to_trigger = jiffies_curr;

	if (mbps < tpmon->trigger_mbps) {
		tpmon->rx_bytes = 0;
		return false;
	}

	return true;
}

int tpmon_start(void)
{
	struct cpif_tpmon *tpmon = &_tpmon;

	if (tpmon_check_active())
		return 0;

	if (!tpmon_check_to_start(tpmon))
		return 0;

	atomic_set(&tpmon->active, 1);

	queue_delayed_work_on(3, tpmon->monitor_wq, &tpmon->monitor_dwork, 0);

	if (tpmon->debug_print)
		mif_info("started\n");

	return 0;
}
EXPORT_SYMBOL(tpmon_start);

int tpmon_stop(void)
{
	struct cpif_tpmon *tpmon = &_tpmon;
	struct tpmon_data *data;

	if (!tpmon_check_active())
		return 0;

	cancel_delayed_work(&tpmon->monitor_dwork);

	atomic_set(&tpmon->active, 0);

	list_for_each_entry(data, &tpmon->data_list, data_node) {
		if (data->curr_value_pos != 0) {
			atomic_set(&tpmon->need_init, 1);
			break;
		}
	}

	tpmon_init_params(tpmon);

	if (atomic_read(&tpmon->need_init))
		queue_delayed_work_on(3, tpmon->monitor_wq, &tpmon->monitor_dwork, 0);

	if (tpmon->debug_print)
		mif_info("stopped\n");

	return 0;
}
EXPORT_SYMBOL(tpmon_stop);

int tpmon_init(void)
{
	struct cpif_tpmon *tpmon = &_tpmon;

	if (tpmon_check_active())
		tpmon_stop();

	tpmon_init_params(tpmon);

	mif_info("set initial values\n");
	atomic_set(&tpmon->need_init, 1);
	queue_delayed_work_on(3, tpmon->monitor_wq, &tpmon->monitor_dwork, 0);

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
static ssize_t dt_value_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cpif_tpmon *tpmon = &_tpmon;
	struct tpmon_data *data;
	ssize_t len = 0;
	int i = 0;

	list_for_each_entry(data, &tpmon->data_list, data_node) {
		len += scnprintf(buf + len, PAGE_SIZE - len, "%s threshold: ", data->name);

		for (i = 0; i < data->num_threshold; i++)
			len += scnprintf(buf + len, PAGE_SIZE - len, "%d ", data->threshold[i]);

		len += scnprintf(buf + len, PAGE_SIZE - len, "\n");
		len += scnprintf(buf + len, PAGE_SIZE - len, "%s values: ", data->name);

		for (i = 0; i < data->num_values; i++)
			len += scnprintf(buf + len, PAGE_SIZE - len, "0x%x(%d) ",
				data->values[i], data->values[i]);

		len += scnprintf(buf + len, PAGE_SIZE - len, "\n\n");
	}

	return len;
}
static DEVICE_ATTR_RO(dt_value);

static ssize_t curr_value_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cpif_tpmon *tpmon = &_tpmon;
	struct tpmon_data *data;
	ssize_t len = 0;
	int ret = 0;

	list_for_each_entry(data, &tpmon->data_list, data_node) {
		ret = tpmon->use_user_value ? data->user_value : data->values[data->curr_value_pos];

		len += scnprintf(buf + len, PAGE_SIZE - len, "%s: 0x%x(%d) pos:%d\n",
			data->name, ret, ret, data->curr_value_pos);
	}

	return len;
}
static DEVICE_ATTR_RO(curr_value);

static ssize_t status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cpif_tpmon *tpmon = &_tpmon;
	struct tpmon_data *data;
	ssize_t len = 0;
	int i;

	len += scnprintf(buf + len, PAGE_SIZE - len,
			"start at %dMbps min:%dmsec max:%dmsec\n",
			tpmon->trigger_mbps, tpmon->trigger_msec_min,
			tpmon->trigger_msec_max);
	len += scnprintf(buf + len, PAGE_SIZE - len,
			"monitor interval:%dmsec hold:%dmsec\n",
			tpmon->monitor_interval_msec,
			tpmon->monitor_hold_msec);
	len += scnprintf(buf + len, PAGE_SIZE - len,
			"boost hold:%dmsec unboost_percent:%d\n",
			tpmon->boost_hold_msec,
			tpmon->unboost_tp_percent);
	len += scnprintf(buf + len, PAGE_SIZE - len,
			"rx %ldbytes %ldMbps %ldKbps sum:%d\n",
			tpmon->rx_bytes, tpmon->rx_mbps, tpmon->rx_kbps, tpmon->rx_sum);
	len += scnprintf(buf + len, PAGE_SIZE - len,
			"queue status pktproc:%d dit:%d netdev:%d\n",
			tpmon->pktproc_queue_status,
			tpmon->dit_src_queue_status,
			tpmon->netdev_backlog_queue_status);
	len += scnprintf(buf + len, PAGE_SIZE - len,
			"use_user_value:%d debug_print:%d\n",
			tpmon->use_user_value,
			tpmon->debug_print);

	list_for_each_entry(data, &tpmon->data_list, data_node) {
		len += scnprintf(buf + len, PAGE_SIZE - len, "\n");
		len += scnprintf(buf + len, PAGE_SIZE - len,
			"name:%s measure:%d target:%d enable:%d extra_idx:%d\n",
			data->name, data->measure,
			data->target, data->enable, data->extra_idx);

		len += scnprintf(buf + len, PAGE_SIZE - len,
			"num_threshold:%d\n", data->num_threshold);
		for (i = 0; i < data->num_threshold; i++)
			len += scnprintf(buf + len, PAGE_SIZE - len,
				"%d(unboost@%dMbps) ",
				data->threshold[i], data->unboost_tp_mbps[i]);
		len += scnprintf(buf + len, PAGE_SIZE - len, "\n");

		len += scnprintf(buf + len, PAGE_SIZE - len,
			"num_values:%d\n", data->num_values);
		for (i = 0; i < data->num_values; i++)
			len += scnprintf(buf + len, PAGE_SIZE - len,
				"%d ", data->values[i]);
		len += scnprintf(buf + len, PAGE_SIZE - len, "\n");

		len += scnprintf(buf + len, PAGE_SIZE - len,
			"curr_value_pos:%d user_value:%d\n",
			data->curr_value_pos, data->user_value);
	}

	return len;
}
static DEVICE_ATTR_RO(status);

static ssize_t use_user_value_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cpif_tpmon *tpmon = &_tpmon;

	return scnprintf(buf, PAGE_SIZE, "enable use user value: %d\n", tpmon->use_user_value);
}

static ssize_t use_user_value_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cpif_tpmon *tpmon = &_tpmon;
	struct tpmon_data *data;
	int ret;
	int value;

	ret = kstrtoint(buf, 0, &value);
	if (ret != 0) {
		mif_err("invalid value:%d with %d\n", value, ret);
		return -EINVAL;
	}

	tpmon->use_user_value = value;

	mif_info("enable use user value: %d\n", tpmon->use_user_value);

	if (tpmon->use_user_value) {
		tpmon_stop();

		list_for_each_entry(data, &tpmon->data_list, data_node) {
			if (data->set_data) {
				data->user_value = data->values[0];
				data->set_data(data);
			}
		}

		return count;
	}

	tpmon_start();

	return count;
}
static DEVICE_ATTR_RW(use_user_value);

static ssize_t debug_print_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cpif_tpmon *tpmon = &_tpmon;

	return scnprintf(buf, PAGE_SIZE, "debug pring enable:%d\n", tpmon->debug_print);
}

static ssize_t debug_print_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cpif_tpmon *tpmon = &_tpmon;
	int ret;
	int value;

	ret = kstrtoint(buf, 0, &value);
	if (ret != 0) {
		mif_err("invalid value:%d with %d\n", value, ret);
		return -EINVAL;
	}

	tpmon->debug_print = value;

	mif_info("debug pring enable:%d\n", tpmon->debug_print);

	return count;
}
static DEVICE_ATTR_RW(debug_print);

static ssize_t set_user_value_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cpif_tpmon *tpmon = &_tpmon;
	struct tpmon_data *data;
	char name[20];
	int value;
	int ret;

	ret = sscanf(buf, "%s %x", name, &value);
	if (ret < 1)
		return -EINVAL;

	mif_info("%s %d 0x%x\n", name, value, value);

	list_for_each_entry(data, &tpmon->data_list, data_node) {
		if (strcmp(data->name, name) == 0) {
			data->user_value = value;
			data->set_data(data);
		}
	}

	return count;
}
static DEVICE_ATTR_WO(set_user_value);

static struct attribute *tpmon_attrs[] = {
	&dev_attr_dt_value.attr,
	&dev_attr_curr_value.attr,
	&dev_attr_status.attr,
	&dev_attr_use_user_value.attr,
	&dev_attr_debug_print.attr,
	&dev_attr_set_user_value.attr,
	NULL,
};

static const struct attribute_group tpmon_group = {
	.attrs = tpmon_attrs,
	.name = "tpmon",
};

/*
 * Init
 */
static int tpmon_parse_dt(struct device_node *np, struct cpif_tpmon *tpmon)
{
	struct device_node *tpmon_np = NULL;
	struct device_node *child_np = NULL;
	struct tpmon_data *data = NULL;
	int ret = 0;
	u32 count = 0;
	unsigned long flags;

	tpmon_np = of_get_child_by_name(np, "cpif_tpmon");
	if (!tpmon_np) {
		mif_err("tpmon_np is null\n");
		return -ENODEV;
	}

	mif_dt_read_u32(tpmon_np, "tpmon_trigger_mbps",
			tpmon->trigger_mbps);
	mif_dt_read_u32(tpmon_np, "tpmon_trigger_msec_min",
			tpmon->trigger_msec_min);
	mif_dt_read_u32(tpmon_np, "tpmon_trigger_msec_max",
			tpmon->trigger_msec_max);
	mif_info("trigger:%dMbps min:%dmsec max:%dmsec\n",
			tpmon->trigger_mbps, tpmon->trigger_msec_min,
			tpmon->trigger_msec_max);

	mif_dt_read_u32(tpmon_np, "tpmon_monitor_interval_msec",
			tpmon->monitor_interval_msec);
	mif_dt_read_u32(tpmon_np, "tpmon_monitor_hold_msec",
			tpmon->monitor_hold_msec);
	mif_info("monitor interval:%dmsec hold:%dmsec\n",
			tpmon->monitor_interval_msec, tpmon->monitor_hold_msec);

	mif_dt_read_u32(tpmon_np, "tpmon_boost_hold_msec",
			tpmon->boost_hold_msec);
	mif_dt_read_u32(tpmon_np, "tpmon_unboost_tp_percent",
			tpmon->unboost_tp_percent);
	mif_info("boost hold:%dmsec unboost percent:%d\n",
			tpmon->monitor_interval_msec, tpmon->boost_hold_msec,
			tpmon->unboost_tp_percent);

	for_each_child_of_node(tpmon_np, child_np) {
		if (count >= MAX_TPMON_DATA) {
			mif_err("count is full:%d\n", count);
			return -EINVAL;
		}

		data = &tpmon->data[count];
		memset(data, 0, sizeof(struct tpmon_data));

		/* name */
		ret = of_property_read_string(child_np, "tpmon,name",
						(const char **)&data->name);
		if (ret < 0) {
			mif_info("can not get %d tpmon,name:%s\n", count, ret);
			return ret;
		}

		/* enable */
		ret = of_property_read_u32(child_np, "tpmon,enable", &data->enable);
		if (ret || !data->enable) {
			mif_info("%s is not enabled:%d %d\n",
					data->name, ret, data->enable);
			continue;
		}

		/* measure */
		ret = of_property_read_u32(child_np, "tpmon,measure", &data->measure);
		if (ret) {
			mif_info("can not get tpmon,measure:%s %d %d %d\n",
					data->name, count, ret, data->measure);
			return ret;
		}

		switch (data->measure) {
		case TPMON_MEASURE_TP:
			data->get_data = tpmon_get_rx_speed_mbps;
			break;
		case TPMON_MEASURE_NETDEV_Q:
			data->get_data = tpmon_get_netdev_backlog_queue_status;
			break;
		case TPMON_MEASURE_PKTPROC_DL_Q:
			data->get_data = tpmon_get_pktproc_queue_status;
			break;
		case TPMON_MEASURE_DIT_SRC_Q:
#if IS_ENABLED(CONFIG_EXYNOS_DIT)
			data->get_data = tpmon_get_dit_src_queue_status;
#endif
			break;
		default:
			mif_err("%s measure error:%d %d\n", data->name, count, data->measure);
			return -EINVAL;
		}

		/* target */
		ret = of_property_read_u32(child_np, "tpmon,target", &data->target);
		if (ret) {
			mif_info("can not get tpmon,target:%s %d %d %d\n",
					data->name, count, ret, data->target);
			return ret;
		}

		switch (data->target) {
		case TPMON_TARGET_RPS:
#if IS_ENABLED(CONFIG_RPS)
			data->set_data = tpmon_set_rps;
#else
			mif_info("rps config is disabled\n");
			continue;
#endif
			break;
		case TPMON_TARGET_GRO:
#if IS_ENABLED(CONFIG_MODEM_IF_NET_GRO)
			data->set_data = tpmon_set_gro;
#else
			mif_info("gro config is disabled\n");
			continue;
#endif
			break;
		case TPMON_TARGET_MIF:
#if IS_ENABLED(CONFIG_EXYNOS_PM_QOS)
			data->extra_data = (void *)&tpmon->qos_req_mif;
			data->set_data = tpmon_set_exynos_pm_qos;
#else
			mif_info("pm qos config is disabled\n");
			continue;
#endif
			break;
		case TPMON_TARGET_PCIE_LOW_POWER:
#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE)
			data->set_data = tpmon_set_pci_low_power;
#else
			mif_info("pcie config is disabled\n");
			continue;
#endif
			break;
		case TPMON_TARGET_IRQ_MBOX:
#if IS_ENABLED(CONFIG_LINK_DEVICE_SHMEM) && IS_ENABLED(CONFIG_MCU_IPC)
			data->set_data = tpmon_set_irq_affinity_mbox;
#else
			mif_info("mailbox config is disabled\n");
			continue;
#endif
			break;
		case TPMON_TARGET_IRQ_PCIE:
#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE)
			data->set_data = tpmon_set_irq_affinity_pcie;
#else
			mif_info("pcie config is disabled\n");
			continue;
#endif
			break;
		case TPMON_TARGET_IRQ_DIT:
#if IS_ENABLED(CONFIG_EXYNOS_DIT)
			data->set_data = tpmon_set_irq_affinity_dit;
#else
			mif_info("dit config is disabled\n");
			continue;
#endif
			break;
		case TPMON_TARGET_INT_FREQ:
#if IS_ENABLED(CONFIG_EXYNOS_PM_QOS)
			data->extra_data = (void *)&tpmon->qos_req_int;
			data->set_data = tpmon_set_exynos_pm_qos;
#else
			mif_info("pm qos config is disabled\n");
			continue;
#endif
			break;
		case TPMON_TARGET_BTS:
#if IS_ENABLED(CONFIG_EXYNOS_BTS)
			data->set_data = tpmon_set_bts;
#else
			mif_info("bts config is disabled\n");
			continue;
#endif
			break;
		case TPMON_TARGET_CPU_CL0:
#if IS_ENABLED(CONFIG_EXYNOS_PM_QOS)
			data->extra_data = (void *)&tpmon->qos_req_cpu_cl0;
			data->set_data = tpmon_set_cpu_freq;
#else
			mif_info("pm qos config is disabled\n");
			continue;
#endif
			break;
		case TPMON_TARGET_CPU_CL1:
#if IS_ENABLED(CONFIG_EXYNOS_PM_QOS)
			data->extra_data = (void *)&tpmon->qos_req_cpu_cl1;
			data->set_data = tpmon_set_cpu_freq;
#else
			mif_info("pm qos config is disabled\n");
			continue;
#endif
			break;
		case TPMON_TARGET_CPU_CL2:
#if IS_ENABLED(CONFIG_EXYNOS_PM_QOS)
			data->extra_data = (void *)&tpmon->qos_req_cpu_cl2;
			data->set_data = tpmon_set_cpu_freq;
#else
			mif_info("pm qos config is disabled\n");
			continue;
#endif
			break;
		default:
			mif_err("%s target error:%d %d\n", data->name, count, data->target);
			return -EINVAL;
		}

		/* extra_idx */
		ret = of_property_read_u32(child_np, "tpmon,idx",
				&data->extra_idx);
		if (ret) {
			mif_info("can not get tpmon,extra_idx:%s, %d %d %d %d\n",
					data->name, data->measure, data->target,
					ret, data->extra_idx);
			return ret;
		}

		/* threshold */
		ret = of_property_count_u32_elems(child_np, "tpmon,threshold");
		if (ret < 0) {
			mif_err("can not get num_threshold:%s %d %d %d\n",
					data->name, data->measure, data->target, ret);
			return ret;
		}
		data->num_threshold = ret;
		if (data->num_threshold > MAX_TPMON_THRESHOLD) {
			mif_err("num_threshold is over max:%s %d %d %d\n",
					data->name, data->measure, data->target,
					data->num_threshold);
			return -EINVAL;
		}
		ret = of_property_read_u32_array(child_np, "tpmon,threshold",
						data->threshold, data->num_threshold);
		if (ret) {
			mif_err("can not get threshold:%s %d %d %d\n",
					data->name, data->measure, data->target, ret);
			return ret;
		}

		/* values */
		ret = of_property_count_u32_elems(child_np, "tpmon,values");
		if (ret < 0) {
			mif_err("can not get num_values:%s %d %d %d\n",
					data->name, data->measure, data->target, ret);
			return -EINVAL;
		}
		data->num_values = ret;
		if (data->num_values > MAX_TPMON_VALUES) {
			mif_err("num_values is over max:%s %d %d %d\n",
					data->name, data->measure, data->target, data->num_values);
			return -EINVAL;
		}
		ret = of_property_read_u32_array(child_np, "tpmon,values",
							data->values, data->num_values);
		if (ret) {
			mif_err("can not get values:%s %d %d %d\n",
					data->name, data->measure, data->target, ret);
			return ret;
		}

		data->tpmon = tpmon;

		spin_lock_irqsave(&tpmon->lock, flags);
		list_add_tail(&data->data_node, &tpmon->data_list);
		spin_unlock_irqrestore(&tpmon->lock, flags);

		mif_info("name:%s measure:%d target:%d enable:%d idx:%d num:%d/%d\n",
				data->name, data->measure, data->target, data->enable,
				data->extra_idx, data->num_threshold, data->num_values);

		count++;
	}

	return 0;
}

int tpmon_create(struct platform_device *pdev, struct link_device *ld)
{
	struct device_node *np = pdev->dev.of_node;
	struct cpif_tpmon *tpmon = &_tpmon;
	struct mem_link_device *mld = ld_to_mem_link_device(ld);
	int ret = 0;

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
	tpmon->use_user_value = 0;
	tpmon->debug_print = 0;
	mld->tpmon = &_tpmon;

	spin_lock_init(&tpmon->lock);
	atomic_set(&tpmon->active, 0);

	INIT_LIST_HEAD(&tpmon->data_list);

	ret = tpmon_parse_dt(np, tpmon);
	if (ret) {
		mif_err("tpmon_parse_dt() error:%d\n", ret);
		goto create_error;
	}

#if IS_ENABLED(CONFIG_RPS)
	INIT_LIST_HEAD(&tpmon->net_node_list);
#endif

#if IS_ENABLED(CONFIG_EXYNOS_PM_QOS)
	exynos_pm_qos_add_request(&tpmon->qos_req_mif, PM_QOS_BUS_THROUGHPUT, 0);
	exynos_pm_qos_add_request(&tpmon->qos_req_int, PM_QOS_DEVICE_THROUGHPUT, 0);
#endif

#if IS_ENABLED(CONFIG_CPU_FREQ)
	tpmon->cpufreq_nb.notifier_call = tpmon_cpufreq_nb;
	cpufreq_register_notifier(&tpmon->cpufreq_nb, CPUFREQ_POLICY_NOTIFIER);
#endif

#if IS_ENABLED(CONFIG_EXYNOS_BTS)
	tpmon->bts_scen_index = bts_get_scenindex("cp_throughput");
#endif

	tpmon->monitor_wq = create_workqueue("cpif_tpmon_monitor_wq");
	if (!tpmon->monitor_wq) {
		mif_err("create_workqueue() error\n");
		ret = -EINVAL;
		goto create_error;
	}
	INIT_DELAYED_WORK(&tpmon->monitor_dwork, tpmon_monitor_work);

	tpmon->start = tpmon_start;
	tpmon->stop = tpmon_stop;
	tpmon->add_rx_bytes = tpmon_add_rx_bytes;
	tpmon->check_active = tpmon_check_active;

	if (sysfs_create_group(&pdev->dev.kobj, &tpmon_group))
		mif_err("failed to create cpif tpmon groups node\n");

	return ret;

create_error:
	mif_err("Error:%d\n", ret);

	return ret;
}
EXPORT_SYMBOL(tpmon_create);
