// SPDX-License-Identifier: GPL-2.0-only
/*
 * GS101 SoC Exynos ACME (A Cpufreq that Meets Every chipset) driver
 *
 * Copyright (c) 2019 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * GS101 based board files should include this file.
 *
 */

#define pr_fmt(fmt)	KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/cpufreq.h>
#include <linux/tick.h>
#include <linux/pm_opp.h>
#include <linux/suspend.h>
#include <linux/platform_device.h>

#include <soc/google/cal-if.h>
#include <soc/google/ect_parser.h>
#include <soc/google/exynos-cpupm.h>
#include <soc/google/exynos_cpu_cooling.h>
#include <soc/google/debug-snapshot.h>
#include <soc/google/gs_tmu.h>

#include <trace/events/power.h>

#include "exynos-acme.h"
#include "../soc/google/vh/kernel/systrace.h"

#if IS_ENABLED(CONFIG_PIXEL_EM)
#include "../soc/google/vh/include/pixel_em.h"
struct pixel_em_profile **exynos_acme_pixel_em_profile;
EXPORT_SYMBOL_GPL(exynos_acme_pixel_em_profile);
#endif

/*
 * list head of cpufreq domain
 */
static LIST_HEAD(domains);

/*
 * list head of units which have cpufreq policy dependency
 */
static LIST_HEAD(ready_list);

/*********************************************************************
 *                          HELPER FUNCTION                          *
 *********************************************************************/
static struct exynos_cpufreq_domain *find_domain(unsigned int cpu)
{
	struct exynos_cpufreq_domain *domain;

	list_for_each_entry(domain, &domains, list)
		if (cpumask_test_cpu(cpu, &domain->cpus))
			return domain;

	pr_err("cannot find cpufreq domain by cpu\n");
	return NULL;
}

static void enable_domain(struct exynos_cpufreq_domain *domain)
{
	mutex_lock(&domain->lock);
	domain->enabled = true;
	mutex_unlock(&domain->lock);
}

static void disable_domain(struct exynos_cpufreq_domain *domain)
{
	mutex_lock(&domain->lock);
	domain->enabled = false;
	mutex_unlock(&domain->lock);
}

/*********************************************************************
 *                   PRE/POST HANDLING FOR SCALING                   *
 *********************************************************************/
static int pre_scale(struct cpufreq_freqs *freqs)
{
	return 0;
}

static int post_scale(struct cpufreq_freqs *freqs)
{
	return 0;
}

/*********************************************************************
 *                         FREQUENCY SCALING                         *
 *********************************************************************/
/*
 * Depending on cluster structure, it cannot be possible to get/set
 * cpu frequency while cluster is off. For this, disable cluster-wide
 * power mode while getting/setting frequency.
 */
static unsigned int get_freq(struct exynos_cpufreq_domain *domain)
{
	int wakeup_flag = 0;
	unsigned int freq;
	struct cpumask temp;

	cpumask_and(&temp, &domain->cpus, cpu_online_mask);

	if (cpumask_empty(&temp))
		return domain->old;

	if (domain->need_awake) {
		if (likely(domain->old))
			return domain->old;

		wakeup_flag = 1;
		disable_power_mode(cpumask_any(&domain->cpus), POWERMODE_TYPE_CLUSTER);
	}

	freq = (unsigned int)cal_dfs_get_rate(domain->cal_id);
	if (!freq) {
		/* On changing state, CAL returns 0 */
		freq = domain->old;
	}

	if (unlikely(wakeup_flag))
		enable_power_mode(cpumask_any(&domain->cpus), POWERMODE_TYPE_CLUSTER);

	return freq;
}

static int set_freq(struct exynos_cpufreq_domain *domain,
		    unsigned int target_freq)
{
	int err;

	dbg_snapshot_printk("ID %d: %d -> %d (%d)\n",
			    domain->id, domain->old, target_freq, DSS_FLAG_IN);

	if (domain->need_awake)
		disable_power_mode(cpumask_any(&domain->cpus), POWERMODE_TYPE_CLUSTER);

	err = cal_dfs_set_rate(domain->cal_id, target_freq);
	if (err < 0) {
		pr_err("failed to scale frequency of domain%d (%d -> %d)\n",
		       domain->id, domain->old, target_freq);
	} else {
		trace_clock_set_rate(domain->dn->full_name, target_freq,
				     raw_smp_processor_id());
	}
	if (domain->need_awake)
		enable_power_mode(cpumask_any(&domain->cpus), POWERMODE_TYPE_CLUSTER);

	dbg_snapshot_printk("ID %d: %d -> %d (%d)\n", domain->id, domain->old,
			    target_freq, DSS_FLAG_OUT);

	return err;
}

static int scale(struct exynos_cpufreq_domain *domain,
		 struct cpufreq_policy *policy,
		 unsigned int target_freq)
{
	int ret;
	struct cpufreq_freqs freqs = {
		.policy		= policy,
		.old		= domain->old,
		.new		= target_freq,
		.flags		= 0,
	};

	cpufreq_freq_transition_begin(policy, &freqs);
	dbg_snapshot_freq(domain->id, domain->old, target_freq, DSS_FLAG_IN);

	ret = pre_scale(&freqs);
	if (ret)
		goto fail_scale;

	/* Scale frequency by hooked function, set_freq() */
	ret = set_freq(domain, target_freq);
	if (ret)
		goto fail_scale;

	ret = post_scale(&freqs);
	if (ret)
		goto fail_scale;

fail_scale:
	/* In scaling failure case, logs -1 to exynos snapshot */
	dbg_snapshot_freq(domain->id, domain->old, target_freq,
			  ret < 0 ? ret : DSS_FLAG_OUT);
	cpufreq_freq_transition_end(policy, &freqs, ret);

	return ret;
}

/*********************************************************************
 *                         THERMAL PRESSURE                         *
 *********************************************************************/
/*
 * When device thermals throttle the CPUs, we notify the scheduler of
 * capacity change using the thermal pressure APIs
 */
static void update_thermal_pressure(struct exynos_cpufreq_domain *domain, int dfs_count_change)
{
	cpumask_t *maskp = &domain->cpus;
	struct cpufreq_policy *policy = cpufreq_cpu_get(cpumask_first(maskp));
	unsigned long max_capacity, min_capacity, capacity;
#if IS_ENABLED(CONFIG_PIXEL_EM)
	struct pixel_em_profile **profile_ptr_snapshot, *profile = NULL;
	struct pixel_em_cluster *em_cluster;
	int i;
#endif

	if (!policy)
		return;

	max_capacity = arch_scale_cpu_capacity(cpumask_first(maskp));
	min_capacity = (policy->cpuinfo.min_freq * max_capacity) / (policy->cpuinfo.max_freq);
	capacity     = (policy->max * max_capacity) / (policy->cpuinfo.max_freq);

	spin_lock(&domain->thermal_update_lock);
	domain->dfs_throttle_count += dfs_count_change;

	BUG_ON(domain->dfs_throttle_count < 0);
	BUG_ON(domain->dfs_throttle_count > domain->max_dfs_count);

#if IS_ENABLED(CONFIG_PIXEL_EM)
	profile_ptr_snapshot = READ_ONCE(exynos_acme_pixel_em_profile);

	if (profile_ptr_snapshot)
		profile = READ_ONCE(*profile_ptr_snapshot);

	if (profile) {
		em_cluster = profile->cpu_to_cluster[cpumask_first(maskp)];
		for (i = 0; i < em_cluster->num_opps - 1; i++) {
			if (em_cluster->opps[i].freq >= policy->max)
				break;
		}
		capacity = (domain->dfs_throttle_count > 0) ?
			em_cluster->opps[0].capacity : em_cluster->opps[i].capacity;
	} else {
		capacity = (domain->dfs_throttle_count > 0) ? min_capacity : capacity;
	}
#else
	capacity = (domain->dfs_throttle_count > 0) ? min_capacity : capacity;
#endif

	arch_set_thermal_pressure(maskp, max_capacity - capacity);
	spin_unlock(&domain->thermal_update_lock);

	cpufreq_cpu_put(policy);
}

static void exynos_cpufreq_set_thermal_dfs_cb(cpumask_t *maskp, bool is_dfs_throttled)
{
	unsigned int cpu;
	cpumask_t cpu_per_domain = CPU_MASK_NONE;

	/* create a mask with one cpu per domain */
	for_each_cpu_and(cpu, maskp, cpu_possible_mask) {
		struct exynos_cpufreq_domain *domain = find_domain(cpu);
		cpumask_set_cpu(cpumask_first(&domain->cpus), &cpu_per_domain);
	}

	/* apply thermal pressure for each domain */
	for_each_cpu(cpu, &cpu_per_domain) {
		update_thermal_pressure(find_domain(cpu), (is_dfs_throttled ? 1 : -1));
	}
}

/*********************************************************************
 *                   EXYNOS CPUFREQ DRIVER INTERFACE                 *
 *********************************************************************/
static int exynos_cpufreq_init(struct cpufreq_policy *policy)
{
	struct exynos_cpufreq_domain *domain = find_domain(policy->cpu);

	if (!domain)
		return -EINVAL;

	policy->freq_table = domain->freq_table;
	policy->cur = get_freq(domain);
	policy->cpuinfo.transition_latency = TRANSITION_LATENCY;
	policy->dvfs_possible_from_any_cpu = true;
	cpumask_copy(policy->cpus, &domain->cpus);

	pr_info("CPUFREQ domain%d registered\n", domain->id);

	return 0;
}

static unsigned int exynos_cpufreq_resolve_freq(struct cpufreq_policy *policy,
						unsigned int target_freq)
{
	unsigned int index;

	index = cpufreq_frequency_table_target(policy, target_freq, CPUFREQ_RELATION_L);
	if (index < 0) {
		pr_err("target frequency(%d) out of range\n", target_freq);
		return 0;
	}

	return policy->freq_table[index].frequency;
}

static int exynos_cpufreq_online(struct cpufreq_policy *policy)
{
	struct exynos_cpufreq_domain *domain;

	/*
	 * CPU frequency is not changed before cpufreq_resume() is called.
	 * Therefore, if it is called by enable_nonboot_cpus(),
	 * it is ignored.
	 */
	if (cpuhp_tasks_frozen)
		return 0;

	domain = find_domain(policy->cpu);
	if (!domain)
		return 0;

	enable_domain(domain);
	freq_qos_update_request(&domain->max_qos_req, domain->max_freq);

	return 0;
}

static int exynos_cpufreq_offline(struct cpufreq_policy *policy)
{
	struct exynos_cpufreq_domain *domain;

	/*
	 * CPU frequency is not changed after cpufreq_suspend() is called.
	 * Therefore, if it is called by disable_nonboot_cpus(),
	 * it is ignored.
	 */
	if (cpuhp_tasks_frozen)
		return 0;

	domain = find_domain(policy->cpu);
	if (!domain)
		return 0;

	freq_qos_update_request(&domain->max_qos_req, domain->min_freq);
	disable_domain(domain);

	return 0;
}

static int exynos_cpufreq_verify(struct cpufreq_policy_data *new_policy)
{
	struct exynos_cpufreq_domain *domain = find_domain(new_policy->cpu);
	struct cpufreq_policy policy;
	unsigned int min_freq, max_freq;
	int index, ret;

	if (!domain)
		return -EINVAL;

	policy.freq_table = new_policy->freq_table;

	index = cpufreq_table_find_index_ah(&policy, new_policy->max);
	if (index == -1) {
		pr_err("%s : failed to find a proper max frequency\n", __func__);
		return -EINVAL;
	}
	max_freq = policy.freq_table[index].frequency;

	index = cpufreq_table_find_index_al(&policy, new_policy->min);
	if (index == -1) {
		pr_err("%s : failed to find a proper min frequency\n", __func__);
		return -EINVAL;
	}
	min_freq = policy.freq_table[index].frequency;

	domain->max_freq_qos = max_freq;
	domain->min_freq_qos = min_freq;

	new_policy->max = domain->max_freq_qos;
	new_policy->min = domain->min_freq_qos;

	policy_update_call_to_DM(domain->dm_type,
				 domain->min_freq_qos,
				 domain->max_freq_qos);

	ret = cpufreq_frequency_table_verify(new_policy, domain->freq_table);
	if (!ret) {
		update_thermal_pressure(domain, 0);
	}
	return ret;
}

static int __exynos_cpufreq_target(struct cpufreq_policy *policy,
				   unsigned int target_freq,
				   unsigned int relation)
{
	struct exynos_cpufreq_domain *domain = find_domain(policy->cpu);
	int ret = 0;

	if (!domain)
		return -EINVAL;
	ATRACE_BEGIN(__func__);
	mutex_lock(&domain->lock);

	if (!domain->enabled) {
		ret = -EINVAL;
		goto out;
	}

	/* Target is same as current, skip scaling */
	if (domain->old == target_freq)
		goto out;

	ret = scale(domain, policy, target_freq);
	if (ret)
		goto out;

	pr_debug("CPUFREQ domain%d frequency change %u kHz -> %u kHz\n",
		 domain->id, domain->old, target_freq);

	domain->old = target_freq;

out:
	mutex_unlock(&domain->lock);
	ATRACE_END();
	return ret;
}

static int exynos_cpufreq_target(struct cpufreq_policy *policy,
				 unsigned int target_freq,
				 unsigned int relation)
{
	struct exynos_cpufreq_domain *domain = find_domain(policy->cpu);
	unsigned long freq;
	int ret = 0;

	ATRACE_BEGIN(__func__);
	if (!domain || !domain->enabled) {
		ret = -EINVAL;
		goto out;
	}

	mutex_lock(&domain->lock);
	freq = cpufreq_driver_resolve_freq(policy, target_freq);
	// Always go to DM_CALL even with `domain->old == freq` to update dm->governor_freq
	if (!freq) {
		mutex_unlock(&domain->lock);
		goto out;
	}
	mutex_unlock(&domain->lock);

	if (list_empty(&domain->dm_list)) {
		ret = __exynos_cpufreq_target(policy, target_freq, relation);
		goto out;
	}

	freq = (unsigned long)target_freq;

	ret = DM_CALL(domain->dm_type, &freq);

out:
	ATRACE_END();
	return ret;
}

static unsigned int exynos_cpufreq_get(unsigned int cpu)
{
	struct exynos_cpufreq_domain *domain = find_domain(cpu);

	if (!domain)
		return 0;

	return get_freq(domain);
}

static int __exynos_cpufreq_suspend(struct cpufreq_policy *policy,
				    struct exynos_cpufreq_domain *domain)
{
	unsigned int freq;
	struct work_struct *update_work = &policy->update;

	if (!domain)
		return 0;

	freq = domain->resume_freq;

	freq_qos_update_request(policy->min_freq_req, freq);
	freq_qos_update_request(policy->max_freq_req, freq);

	flush_work(update_work);

	return 0;
}

static int exynos_cpufreq_suspend(struct cpufreq_policy *policy)
{
	return __exynos_cpufreq_suspend(policy, find_domain(policy->cpu));
}

static int __exynos_cpufreq_resume(struct cpufreq_policy *policy,
				   struct exynos_cpufreq_domain *domain)
{
	if (!domain)
		return -EINVAL;

	freq_qos_update_request(policy->max_freq_req, domain->max_freq);
	freq_qos_update_request(policy->min_freq_req, domain->min_freq);

	return 0;
}

static int exynos_cpufreq_resume(struct cpufreq_policy *policy)
{
	return __exynos_cpufreq_resume(policy, find_domain(policy->cpu));
}

static void exynos_cpufreq_ready(struct cpufreq_policy *policy)
{
	struct exynos_cpufreq_ready_block *ready_block;

	list_for_each_entry(ready_block, &ready_list, list) {
		if (ready_block->update)
			ready_block->update(policy);
		if (ready_block->get_target)
			ready_block->get_target(policy, exynos_cpufreq_target);
	}
}

static int exynos_cpufreq_pm_notifier(struct notifier_block *notifier,
				      unsigned long pm_event, void *v)
{
	struct exynos_cpufreq_domain *domain;
	struct cpufreq_policy *policy;

	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
		list_for_each_entry_reverse(domain, &domains, list) {
			policy = cpufreq_cpu_get(cpumask_any(&domain->cpus));
			if (!policy)
				continue;
			if (__exynos_cpufreq_suspend(policy, domain))
				return NOTIFY_BAD;
		}
		break;
	case PM_POST_SUSPEND:
		list_for_each_entry(domain, &domains, list) {
			policy = cpufreq_cpu_get(cpumask_any(&domain->cpus));
			if (!policy)
				continue;
			if (__exynos_cpufreq_resume(policy, domain))
				return NOTIFY_BAD;
		}
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block exynos_cpufreq_pm = {
	.notifier_call = exynos_cpufreq_pm_notifier,
	.priority = INT_MAX,
};

static struct cpufreq_driver exynos_driver = {
	.name		= "exynos_cpufreq",
	.flags		= CPUFREQ_STICKY | CPUFREQ_HAVE_GOVERNOR_PER_POLICY |
				CPUFREQ_NEED_UPDATE_LIMITS, // force update dm->governor_freq
	.init		= exynos_cpufreq_init,
	.verify		= exynos_cpufreq_verify,
	.target		= exynos_cpufreq_target,
	.get		= exynos_cpufreq_get,
	.resolve_freq	= exynos_cpufreq_resolve_freq,
	.online		= exynos_cpufreq_online,
	.offline	= exynos_cpufreq_offline,
	.suspend	= exynos_cpufreq_suspend,
	.resume		= exynos_cpufreq_resume,
	.ready		= exynos_cpufreq_ready,
	.attr		= cpufreq_generic_attr,
};

/*********************************************************************
 *                       CPUFREQ SYSFS			             *
 *********************************************************************/
static ssize_t freq_qos_min_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	ssize_t count = 0;
	struct exynos_cpufreq_domain *domain;

	list_for_each_entry(domain, &domains, list)
		count += snprintf(buf + count, 30, "cpu%d: qos_min: %d\n",
				  cpumask_first(&domain->cpus),
				  domain->user_min_qos_req.pnode.prio);
	return count;
}

static ssize_t freq_qos_min_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	int freq, cpu, ret;
	struct exynos_cpufreq_domain *domain;

	ret = sscanf(buf, "%d %8d", &cpu, &freq);
	if (!ret)
		return -EINVAL;

	if (cpu < 0 || cpu >= num_possible_cpus() || freq < 0)
		return -EINVAL;

	domain = find_domain(cpu);
	if (!domain)
		return -EINVAL;

	freq_qos_update_request(&domain->user_min_qos_req, freq);

	return count;
}

static ssize_t freq_qos_max_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	ssize_t count = 0;
	struct exynos_cpufreq_domain *domain;

	list_for_each_entry(domain, &domains, list)
		count += snprintf(buf + count, 30, "cpu%d: qos_max: %d\n",
				  cpumask_first(&domain->cpus),
				  domain->user_max_qos_req.pnode.prio);
	return count;
}

static ssize_t freq_qos_max_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	int freq, cpu, ret;
	struct exynos_cpufreq_domain *domain;

	ret = sscanf(buf, "%d %8d", &cpu, &freq);
	if (!ret)
		return -EINVAL;

	if (cpu < 0 || cpu >= num_possible_cpus() || freq < 0)
		return -EINVAL;

	domain = find_domain(cpu);
	if (!domain)
		return -EINVAL;

	freq_qos_update_request(&domain->user_max_qos_req, freq);

	return count;
}

static ssize_t min_freq_qos_list_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int total_requests = 0;
	struct exynos_cpufreq_domain *domain;
	struct plist_node *min_freq_pos;

	list_for_each_entry(domain, &domains, list) {
		total_requests = 0;
		list_for_each_entry(min_freq_pos,
				    &domain->min_qos_req.qos->min_freq.list.node_list, node_list) {
			total_requests += 1;
			len += sysfs_emit_at(buf, len, "cpu%d: total_requests: %d,"
					     " min_freq_qos: %d\n",
					     cpumask_first(&domain->cpus),
					     total_requests, min_freq_pos->prio);
		}
	}
	return len;
}

static ssize_t max_freq_qos_list_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int total_requests = 0;
	struct exynos_cpufreq_domain *domain;
	struct plist_node *max_freq_pos;

	list_for_each_entry(domain, &domains, list) {
		total_requests = 0;
		list_for_each_entry(max_freq_pos,
				    &domain->max_qos_req.qos->max_freq.list.node_list, node_list) {
			total_requests += 1;
			len += sysfs_emit_at(buf, len, "cpu%d: total_requests: %d,"
					     " max_freq_qos: %d\n",
					     cpumask_first(&domain->cpus),
					     total_requests, max_freq_pos->prio);
		}
	}
	return len;
}

static DEVICE_ATTR_RW(freq_qos_max);
static DEVICE_ATTR_RW(freq_qos_min);
static DEVICE_ATTR_RO(min_freq_qos_list);
static DEVICE_ATTR_RO(max_freq_qos_list);

/*********************************************************************
 *                       CPUFREQ DEV FOPS                            *
 *********************************************************************/

static ssize_t cpufreq_fops_write(struct file *filp, const char __user *buf,
				  size_t count, loff_t *f_pos)
{
	s32 value;
	struct freq_qos_request *req = filp->private_data;

	if (count == sizeof(s32)) {
		if (copy_from_user(&value, buf, sizeof(s32)))
			return -EFAULT;
	} else {
		int ret;

		ret = kstrtos32_from_user(buf, count, 16, &value);
		if (ret)
			return ret;
	}

	freq_qos_update_request(req, value);

	return count;
}

static ssize_t cpufreq_fops_read(struct file *filp, char __user *buf,
				 size_t count, loff_t *f_pos)
{
	s32 value = 0;

	return simple_read_from_buffer(buf, count, f_pos, &value, sizeof(s32));
}

static int cpufreq_fops_open(struct inode *inode, struct file *filp)
{
	int ret;
	struct exynos_cpufreq_file_operations *fops;
	struct freq_qos_request *req;

	fops = container_of(filp->f_op, struct exynos_cpufreq_file_operations, fops);
	req = kzalloc(sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	filp->private_data = req;
	ret = freq_qos_add_request(fops->freq_constraints, req,
				   fops->req_type,
				   fops->default_value);
	if (ret)
		return ret;

	return 0;
}

static int cpufreq_fops_release(struct inode *inode, struct file *filp)
{
	struct freq_qos_request *req = filp->private_data;

	freq_qos_remove_request(req);
	kfree(req);

	return 0;
}

/*********************************************************************
 *                       EXTERNAL REFERENCE APIs                     *
 *********************************************************************/
void exynos_cpufreq_ready_list_add(struct exynos_cpufreq_ready_block *rb)
{
	if (!rb)
		return;

	list_add(&rb->list, &ready_list);
}
EXPORT_SYMBOL_GPL(exynos_cpufreq_ready_list_add);

/*********************************************************************
 *                      SUPPORT for DVFS MANAGER                     *
 *********************************************************************/
static void
validate_dm_constraint_table(struct exynos_dm_freq *table, int table_size,
			     int driver_cal_id, int constraint_cal_id)
{
	unsigned long *ect_table;
	int ect_size, index, ect_index;

	if (!driver_cal_id)
		goto validate_constraint;

	/* validate driver frequency */
	ect_size = cal_dfs_get_lv_num(driver_cal_id);
	ect_table = kcalloc(ect_size, sizeof(unsigned long), GFP_KERNEL);
	if (!ect_table)
		return;

	cal_dfs_get_rate_table(driver_cal_id, ect_table);

	for (index = 0; index < table_size; index++) {
		unsigned int freq = table[index].driver_freq;

		ect_index = ect_size;
		while (--ect_index >= 0) {
			if (freq <= ect_table[ect_index]) {
				table[index].driver_freq = ect_table[ect_index];
				break;
			}
		}
	}

	kfree(ect_table);

validate_constraint:
	if (!constraint_cal_id)
		return;

	/* validate constraint frequency */
	ect_size = cal_dfs_get_lv_num(constraint_cal_id);
	ect_table = kcalloc(ect_size, sizeof(unsigned long), GFP_KERNEL);
	if (!ect_table)
		return;

	cal_dfs_get_rate_table(constraint_cal_id, ect_table);

	for (index = 0; index < table_size; index++) {
		unsigned int freq = table[index].constraint_freq;

		ect_index = ect_size;
		while (--ect_index >= 0) {
			if (freq <= ect_table[ect_index]) {
				table[index].constraint_freq = ect_table[ect_index];
				break;
			}
		}
	}

	kfree(ect_table);
}

static int init_constraint_table_dt(struct exynos_cpufreq_dm *dm,
				    struct device_node *dn)
{
	struct exynos_dm_freq *table;
	int size, table_size, index, c_index;

	/*
	 * A DM constraint table row consists of driver and constraint frequency
	 * value, the size of a row is 64bytes. Divide size in half when
	 * table is allocated.
	 */
	size = of_property_count_u32_elems(dn, "table");
	if (size < 0)
		return size;

	table_size = size / 2;
	table = kcalloc(table_size, sizeof(*table), GFP_KERNEL);
	if (!table)
		return -ENOMEM;

	of_property_read_u32_array(dn, "table", (unsigned int *)table, size);

	validate_dm_constraint_table(table, table_size,
				     dm->driver_cal_id, dm->constraint_cal_id);

	for (index = 0; index < dm->c.table_length; index++) {
		unsigned int freq = dm->c.freq_table[index].driver_freq;

		if (freq > table[0].driver_freq)
			continue;

		for (c_index = 0; c_index < table_size; c_index++) {
			if (freq >= table[c_index].driver_freq) {
				dm->c.freq_table[index].constraint_freq =
					table[c_index].constraint_freq;
				break;
			}

			if (c_index == table_size - 1)
				dm->c.freq_table[index].constraint_freq =
					table[c_index].constraint_freq;
		}
	}

	kfree(table);
	return 0;
}

static int dm_scaler(int dm_type, void *devdata, unsigned int target_freq,
		     unsigned int relation)
{
	struct exynos_cpufreq_domain *domain = devdata;
	struct cpufreq_policy *policy;
	struct cpumask mask;
	int ret;

	/* Skip scaling if all cpus of domain are hotplugged out */
	cpumask_and(&mask, &domain->cpus, cpu_online_mask);
	if (cpumask_empty(&mask))
		return -ENODEV;

	policy = cpufreq_cpu_get(cpumask_first(&mask));
	if (!policy) {
		pr_err("%s: failed get cpufreq policy\n", __func__);
		return -ENODEV;
	}

	ret = __exynos_cpufreq_target(policy, target_freq, relation);

	cpufreq_cpu_put(policy);

	return ret;
}

static int init_dm(struct exynos_cpufreq_domain *domain,
		   struct device_node *dn)
{
	struct exynos_cpufreq_dm *dm;
	struct device_node *root;
	struct of_phandle_iterator iter;
	int ret, err;

	ret = of_property_read_u32(dn, "dm-type", &domain->dm_type);
	if (ret)
		return ret;

	ret = exynos_dm_data_init(domain->dm_type, domain, domain->min_freq,
				  domain->max_freq, domain->old);
	if (ret)
		return ret;

	/* Initialize list head of DVFS Manager constraints */
	INIT_LIST_HEAD(&domain->dm_list);

	/*
	 * Initialize DVFS Manager constraints
	 * - constraint_type : minimum or maximum constraint
	 * - constraint_dm_type : cpu/mif/int/.. etc
	 * - guidance : constraint from chipset characteristic
	 * - freq_table : constraint table
	 */
	root = of_get_child_by_name(dn, "dm-constraints");
	of_for_each_phandle(&iter, err, root, "list", NULL, 0) {
		int index, r_index;

		/* allocate DM constraint */
		dm = kzalloc(sizeof(*dm), GFP_KERNEL);
		if (!dm)
			goto init_fail;

		list_add_tail(&dm->list, &domain->dm_list);

		of_property_read_u32(iter.node, "const-type", &dm->c.constraint_type);
		of_property_read_u32(iter.node, "dm-constraint", &dm->c.dm_constraint);
		of_property_read_u32(iter.node, "driver-cal-id", &dm->driver_cal_id);
		of_property_read_u32(iter.node, "constraint-cal-id", &dm->constraint_cal_id);

		/* allocate DM constraint table */
		dm->c.freq_table = kcalloc(domain->table_size,
					   sizeof(*dm->c.freq_table),
					   GFP_KERNEL);
		if (!dm->c.freq_table)
			goto init_fail;

		dm->c.table_length = domain->table_size;

		/*
		 * fill driver freq, domain frequency table is in ascending
		 * order, but DM constraint table must be in descending
		 * order.
		 */
		index = 0;
		r_index = domain->table_size - 1;
		while (r_index >= 0) {
			dm->c.freq_table[index].driver_freq =
				domain->freq_table[r_index].frequency;
			index++;
			r_index--;
		}

		/* fill constraint freq */
		if (init_constraint_table_dt(dm, iter.node))
			continue;

		/* register DM constraint */
		ret = register_exynos_dm_constraint_table(domain->dm_type, &dm->c);
		if (ret)
			goto init_fail;
	}

	return register_exynos_dm_freq_scaler(domain->dm_type, dm_scaler);

init_fail:
	while (!list_empty(&domain->dm_list)) {
		dm = list_last_entry(&domain->dm_list, struct exynos_cpufreq_dm, list);
		list_del(&dm->list);
		kfree(dm->c.freq_table);
		kfree(dm);
	}

	return 0;
}

/*********************************************************************
 *                  INITIALIZE EXYNOS CPUFREQ DRIVER                 *
 *********************************************************************/
static void print_domain_info(struct exynos_cpufreq_domain *domain)
{
	int i;
	char buf[10];

	pr_info("CPUFREQ of domain%d cal-id : %#x\n", domain->id, domain->cal_id);

	scnprintf(buf, sizeof(buf), "%*pbl", cpumask_pr_args(&domain->cpus));
	pr_info("CPUFREQ of domain%d sibling cpus : %s\n", domain->id, buf);

	pr_info("CPUFREQ of domain%d boot freq = %d kHz, resume freq = %d kHz\n",
		domain->id, domain->boot_freq, domain->resume_freq);

	pr_info("CPUFREQ of domain%d max freq : %d kHz, min freq : %d kHz\n",
		domain->id,
		domain->max_freq, domain->min_freq);

	pr_info("CPUFREQ of domain%d table size = %d\n",
		domain->id, domain->table_size);

	for (i = 0; i < domain->table_size; i++) {
		if (domain->freq_table[i].frequency == CPUFREQ_ENTRY_INVALID)
			continue;

		pr_info("CPUFREQ of domain%d : L%-2d  %7d kHz\n",
			domain->id,
			domain->freq_table[i].driver_data,
			domain->freq_table[i].frequency);
	}
}

static void freq_qos_release(struct work_struct *work)
{
	struct exynos_cpufreq_domain *domain = container_of(to_delayed_work(work),
							    struct exynos_cpufreq_domain,
							    work);

	freq_qos_update_request(&domain->min_qos_req, domain->min_freq);
	freq_qos_update_request(&domain->max_qos_req, domain->max_freq);
}

static int
init_user_freq_qos(struct exynos_cpufreq_domain *domain, struct cpufreq_policy *policy)
{
	int ret = freq_qos_add_request(&policy->constraints, &domain->user_min_qos_req,
				       FREQ_QOS_MIN, domain->min_freq);
	if (ret < 0)
		return ret;

	ret = freq_qos_add_request(&policy->constraints, &domain->user_max_qos_req,
				   FREQ_QOS_MAX, domain->soft_max_freq);
	return ret;
}

static int
init_freq_qos(struct exynos_cpufreq_domain *domain, struct cpufreq_policy *policy)
{
	unsigned int boot_qos, val;
	struct device_node *dn = domain->dn;
	int ret;

	ret = freq_qos_add_request(&policy->constraints, &domain->min_qos_req,
				   FREQ_QOS_MIN, domain->min_freq);
	if (ret < 0)
		return ret;

	ret = freq_qos_add_request(&policy->constraints, &domain->max_qos_req,
				   FREQ_QOS_MAX, domain->max_freq);
	if (ret < 0)
		return ret;

	/* Skip pm_qos if setting exists in device tree */
	if (of_property_read_bool(dn, "skip-boot-pmqos")) {
		pr_info("Skipping boot pm_qos domain%d\n", domain->id);
		return 0;
	}

	/*
	 * Basically booting pm_qos is set to max frequency of domain.
	 * But if pm_qos-booting exists in device tree,
	 * booting pm_qos is selected to smaller one
	 * between max frequency of domain and the value defined in device tree.
	 */
	boot_qos = domain->max_freq;
	if (!of_property_read_u32(dn, "pm_qos-booting", &val))
		boot_qos = min(boot_qos, val);

	freq_qos_update_request(&domain->min_qos_req, boot_qos);
	freq_qos_update_request(&domain->max_qos_req, boot_qos);

	/* booting boost, it is expired after 40s */
	INIT_DELAYED_WORK(&domain->work, freq_qos_release);
	schedule_delayed_work(&domain->work, msecs_to_jiffies(40000));
	pr_info("Set boot pm_qos domain%d to %d for %ld\n", domain->id,
		boot_qos, 40 * USEC_PER_SEC);
	return 0;
}

static int
init_fops(struct exynos_cpufreq_domain *domain, struct cpufreq_policy *policy)
{
	char *node_name_buffer;
	int ret, buffer_size;

	buffer_size = sizeof(char [64]);
	node_name_buffer = kzalloc(buffer_size, GFP_KERNEL);
	if (!node_name_buffer)
		return -ENOMEM;

	snprintf(node_name_buffer, buffer_size,
		 "cluster%d_freq_min", domain->id);

	domain->min_qos_fops.fops.write		= cpufreq_fops_write;
	domain->min_qos_fops.fops.read		= cpufreq_fops_read;
	domain->min_qos_fops.fops.open		= cpufreq_fops_open;
	domain->min_qos_fops.fops.release	= cpufreq_fops_release;
	domain->min_qos_fops.fops.llseek	= noop_llseek;

	domain->min_qos_fops.miscdev.minor	= MISC_DYNAMIC_MINOR;
	domain->min_qos_fops.miscdev.name	= node_name_buffer;
	domain->min_qos_fops.miscdev.fops	= &domain->min_qos_fops.fops;

	domain->min_qos_fops.freq_constraints	= &policy->constraints;
	domain->min_qos_fops.default_value	= FREQ_QOS_MIN_DEFAULT_VALUE;
	domain->min_qos_fops.req_type		= FREQ_QOS_MIN;

	ret = misc_register(&domain->min_qos_fops.miscdev);
	if (ret) {
		pr_err("CPUFREQ couldn't register misc device min for domain %d", domain->id);
		kfree(node_name_buffer);
		return ret;
	}

	node_name_buffer = kzalloc(buffer_size, GFP_KERNEL);
	if (!node_name_buffer)
		return -ENOMEM;

	snprintf(node_name_buffer, buffer_size,
		 "cluster%d_freq_max", domain->id);

	domain->max_qos_fops.fops.write		= cpufreq_fops_write;
	domain->max_qos_fops.fops.read		= cpufreq_fops_read;
	domain->max_qos_fops.fops.open		= cpufreq_fops_open;
	domain->max_qos_fops.fops.release	= cpufreq_fops_release;
	domain->max_qos_fops.fops.llseek	= noop_llseek;

	domain->max_qos_fops.miscdev.minor	= MISC_DYNAMIC_MINOR;
	domain->max_qos_fops.miscdev.name	= node_name_buffer;
	domain->max_qos_fops.miscdev.fops	= &domain->max_qos_fops.fops;

	domain->max_qos_fops.freq_constraints	= &policy->constraints;
	domain->max_qos_fops.default_value	= FREQ_QOS_MAX_DEFAULT_VALUE;
	domain->max_qos_fops.req_type		= FREQ_QOS_MAX;

	ret = misc_register(&domain->max_qos_fops.miscdev);
	if (ret) {
		pr_err("CPUFREQ couldn't register misc device max for domain %d", domain->id);
		kfree(node_name_buffer);
		return ret;
	}

	return 0;
}

static int init_domain(struct exynos_cpufreq_domain *domain,
		       struct device_node *dn)
{
	unsigned int val, orig_table_size;
	int index, r_index;
	unsigned long *freq_table;
	unsigned int *volt_table;
	const char *buf;
	struct device *cpu_dev;
	int ret;
	unsigned int resume_freq = 0;

	/* Get CAL ID */
	ret = of_property_read_u32(dn, "cal-id", &domain->cal_id);
	if (ret)
		return ret;

	/*
	 * Set min/max frequency.
	 * If max-freq property exists in device tree, max frequency is
	 * selected to smaller one between the value defined in device
	 * tree and CAL. In case of min-freq, min frequency is selected
	 * to bigger one.
	 */
	domain->max_freq = cal_dfs_get_max_freq(domain->cal_id);
	domain->soft_max_freq = domain->max_freq;
	domain->min_freq = cal_dfs_get_min_freq(domain->cal_id);

	if (!of_property_read_u32(dn, "max-freq", &val))
		domain->max_freq = val;
	if (!of_property_read_u32(dn, "min-freq", &val))
		domain->min_freq = max(domain->min_freq, val);
	if (!of_property_read_u32(dn, "resume-freq", &val))
		resume_freq = max(domain->min_freq, val);
	if (!of_property_read_u32(dn, "soft-max-freq", &val))
		domain->soft_max_freq = min(domain->max_freq, val);
	domain->soft_max_freq = max(domain->soft_max_freq, domain->min_freq);

	domain->max_freq_qos = domain->max_freq;
	domain->min_freq_qos = domain->min_freq;

	/*
	 * Allocate temporary frequency and voltage tables
	 * to get DVFS table from CAL.
	 */
	orig_table_size = cal_dfs_get_lv_num(domain->cal_id);

	freq_table = kcalloc(orig_table_size, sizeof(unsigned long),
			     GFP_KERNEL);
	if (!freq_table)
		return -ENOMEM;
	cal_dfs_get_rate_table(domain->cal_id, freq_table);

	volt_table = kzalloc(sizeof(unsigned int) * orig_table_size,
			     GFP_KERNEL);
	if (!volt_table) {
		kfree(freq_table);
		return -ENOMEM;
	}
	cal_dfs_get_asv_table(domain->cal_id, volt_table);

	/*
	 * A voltage-based cap can be used to find the max frequency
	 * from the given ASV table.
	 */
	if (!of_property_read_u32(dn, "max-volt", &val)) {
		for (index = 0; index < orig_table_size; index++) {
			if (volt_table[index] <= val)
				break;
		}
		domain->max_freq = min(domain->max_freq, (unsigned int)freq_table[index]);
		domain->max_freq_qos = domain->max_freq;
	}

	resume_freq = min(resume_freq, domain->max_freq);

	/*
	 * Set frequency table size.
	 */
	domain->table_size = 0;
	for (index = 0; index < orig_table_size; index++) {
		if (freq_table[index] > domain->max_freq)
			continue;
		if (freq_table[index] < domain->min_freq)
			continue;

		domain->table_size++;
	}

	/*
	 * Allocate frequency table.
	 * Last row of frequency table must be set to CPUFREQ_TABLE_END.
	 * Table size should be one larger than real table size.
	 */
	domain->freq_table = kcalloc(domain->table_size + 1,
				     sizeof(*domain->freq_table),
				     GFP_KERNEL);
	if (!domain->freq_table) {
		kfree(freq_table);
		return -ENOMEM;
	}

	/*
	 * Initialize frequency table.
	 * The frequency table obtained from ECT is in descending order, but
	 * the frequency table of domain is organized in ascending order for
	 * Android BatteryStat service.
	 */
	index = 0;
	r_index = orig_table_size;
	while (--r_index >= 0) {
		if (freq_table[r_index] > domain->max_freq)
			continue;
		if (freq_table[r_index] < domain->min_freq)
			continue;

		domain->freq_table[index].driver_data = index;
		domain->freq_table[index].frequency = freq_table[r_index];
		index++;
	}

	domain->freq_table[index].driver_data = index;
	domain->freq_table[index].frequency = CPUFREQ_TABLE_END;

	/*
	 * Get cpumask which belongs to domain.
	 */
	ret = of_property_read_string(dn, "sibling-cpus", &buf);
	if (ret) {
		kfree(freq_table);
		return ret;
	}
	cpulist_parse(buf, &domain->cpus);
	cpumask_and(&domain->cpus, &domain->cpus, cpu_possible_mask);
	if (cpumask_weight(&domain->cpus) == 0) {
		kfree(freq_table);
		return -ENODEV;
	}

	/*
	 * Add OPP table for thermal.
	 * Thermal CPU cooling is based on the OPP table.
	 */
	for (index = 0; index < orig_table_size; index++) {
		int cpu;

		if (freq_table[index] > domain->max_freq)
			continue;
		if (freq_table[index] < domain->min_freq)
			continue;

		for_each_cpu_and(cpu, &domain->cpus, cpu_possible_mask)
			dev_pm_opp_add(get_cpu_device(cpu),
				       freq_table[index] * 1000,
				       volt_table[index]);
	}

	kfree(freq_table);
	kfree(volt_table);

	/*
	 * Initialize other items.
	 */
	if (of_property_read_bool(dn, "need-awake"))
		domain->need_awake = true;

	domain->boot_freq = cal_dfs_get_boot_freq(domain->cal_id);
	domain->resume_freq = resume_freq ? resume_freq :
					    cal_dfs_get_resume_freq(domain->cal_id);
	domain->old = get_freq(domain);
	if (domain->old < domain->min_freq || domain->max_freq < domain->old) {
		pr_info("Out-of-range freq(%dkhz) returned for domain%d in init time\n",
		     domain->old, domain->id);
		domain->old = domain->boot_freq;
	}

	mutex_init(&domain->lock);
	spin_lock_init(&domain->thermal_update_lock);

	/*
	 * Initialize CPUFreq DVFS Manager
	 * DVFS Manager is the optional function, it does not check return value
	 */
	init_dm(domain, dn);

	cpu_dev = get_cpu_device(cpumask_first(&domain->cpus));
	dev_pm_opp_of_register_em(cpu_dev, &domain->cpus);

	/* Get max-dfs-count per domain. Set to zero, if not configured*/
	if (of_property_read_u32(dn, "max-dfs-count", &domain->max_dfs_count)) {
		pr_info("max-dfs-count not set for cpufreq-domain:%d, defaulting to 0\n", domain->id);
		domain->max_dfs_count = 0;
	}

	pr_info("Complete to initialize cpufreq-domain%d\n", domain->id);

	return 0;
}

static int exynos_cpufreq_probe(struct platform_device *pdev)
{
	struct device_node *dn;
	struct exynos_cpufreq_domain *domain;
	unsigned int domain_id = 0;
	int ret = 0;

	/*
	 * Pre-initialization.
	 *
	 * allocate and initialize cpufreq domain
	 */
	for_each_child_of_node(pdev->dev.of_node, dn) {
		domain = kzalloc(sizeof(*domain), GFP_KERNEL);
		if (!domain)
			return -ENOMEM;

		domain->id = domain_id++;
		if (init_domain(domain, dn)) {
			pr_err("failed to initialize cpufreq domain%d\n", domain->id);
			kfree(domain->freq_table);
			kfree(domain);
			continue;
		}

		domain->dn = dn;
		list_add_tail(&domain->list, &domains);

		print_domain_info(domain);
	}

	if (!domain_id) {
		pr_err("Failed to initialize cpufreq driver\n");
		return -ENOMEM;
	}

	/* Register cpufreq driver */
	ret = cpufreq_register_driver(&exynos_driver);
	if (ret) {
		pr_err("failed to register cpufreq driver\n");
		return ret;
	}

	/*
	 * Post-initialization
	 *
	 * 1. create sysfs to control frequency min/max
	 * 2. enable frequency scaling of each domain
	 * 3. initialize freq qos of each domain
	 * 4. register notifier bloack
	 */
	ret = sysfs_create_file(&pdev->dev.kobj, &dev_attr_freq_qos_max.attr);
	if (ret) {
		pr_err("failed to create user_max node\n");
		return ret;
	}

	ret = sysfs_create_file(&pdev->dev.kobj, &dev_attr_freq_qos_min.attr);
	if (ret) {
		pr_err("failed to create user_min node\n");
		return ret;
	}

	ret = sysfs_create_file(&pdev->dev.kobj, &dev_attr_min_freq_qos_list.attr);
	if (ret) {
		pr_err("failed to create min_freq_qos_list node\n");
		return ret;
	}

	ret = sysfs_create_file(&pdev->dev.kobj, &dev_attr_max_freq_qos_list.attr);
	if (ret) {
		pr_err("failed to create max_freq_qos_list node\n");
		return ret;
	}

	list_for_each_entry(domain, &domains, list) {
		struct cpufreq_policy *policy;

		policy = cpufreq_cpu_get_raw(cpumask_first(&domain->cpus));
		if (!policy) {
			pr_err("failed to find domain policy!\n");
			return -ENODEV;
		}

		ret = init_user_freq_qos(domain, policy);
		if (ret < 0) {
			pr_err("Failed to set max user qos vote!\n");
			return ret;
		}

		enable_domain(domain);

#if IS_ENABLED(CONFIG_EXYNOS_CPU_THERMAL)
		exynos_cpufreq_cooling_register(domain->dn, policy);
#endif
		ret = init_freq_qos(domain, policy);
		if (ret) {
			pr_info("failed to init pm_qos with err %d\n", ret);
			return ret;
		}
		ret = init_fops(domain, policy);
		if (ret) {
			pr_info("failed to init fops with err %d\n", ret);
			return ret;
		}
	}

	register_pm_notifier(&exynos_cpufreq_pm);
	register_dfs_throttle_cb(exynos_cpufreq_set_thermal_dfs_cb);

	pr_info("Initialized Exynos cpufreq driver\n");

	return ret;
}

static const struct of_device_id of_exynos_cpufreq_match[] = {
	{ .compatible = "samsung,exynos-acme", },
	{ },
};
MODULE_DEVICE_TABLE(of, of_exynos_cpufreq_match);

static struct platform_driver exynos_cpufreq_driver = {
	.driver = {
		.name	= "exynos-acme",
		.owner = THIS_MODULE,
		.of_match_table = of_exynos_cpufreq_match,
	},
	.probe		= exynos_cpufreq_probe,
};

module_platform_driver(exynos_cpufreq_driver);

MODULE_DESCRIPTION("Exynos ACME");
MODULE_LICENSE("GPL");
