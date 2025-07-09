// SPDX-License-Identifier: GPL-2.0-only
/* pixel_em.c
 *
 * Support for runtime-customizable table-based Energy Model
 *
 * Copyright 2022 Google LLC
 */

#define pr_fmt(fmt) "pixel-em: " fmt

#include <linux/arch_topology.h>
#include <linux/bitops.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/energy_model.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>

#include "../../include/pixel_em.h"

#if IS_ENABLED(CONFIG_VH_SCHED)
extern struct pixel_em_profile **vendor_sched_pixel_em_profile;
extern struct pixel_idle_em *vendor_sched_pixel_idle_em;
extern raw_spinlock_t vendor_sched_pixel_em_lock;
extern void vh_arch_set_freq_scale_pixel_mod(void *data,
					     const struct cpumask *cpus,
					     unsigned long freq,
					     unsigned long max,
					     unsigned long *scale);

extern bool update_thermal_freq_cap(unsigned int cpu);
#endif

#if IS_ENABLED(CONFIG_EXYNOS_CPU_THERMAL)
extern struct pixel_em_profile **exynos_cpu_cooling_pixel_em_profile;
#endif

extern int pixel_cpu_num;
extern int pixel_cluster_num;
extern int *pixel_cluster_start_cpu;
extern bool pixel_cpu_init;

static struct mutex profile_list_lock;
static LIST_HEAD(profile_list);
static struct pixel_em_profile *active_profile;
static struct pixel_idle_em *idle_profile;

static struct mutex sysfs_lock; // Synchronize sysfs calls.
static struct kobject *primary_sysfs_folder;
static struct kobject *profiles_sysfs_folder;

static struct platform_device *platform_dev;

static struct pixel_em_profile *generate_default_em_profile(const char *);
static void pixel_em_free_profile(struct pixel_em_profile *);
static int pixel_em_publish_profile(struct pixel_em_profile *);
static void pixel_em_unpublish_profile(struct pixel_em_profile *);

#if IS_ENABLED(CONFIG_VH_SCHED)
static void pixel_em_free_idle(struct pixel_idle_em *);
#endif

static int pixel_em_init_cpu_layout(void)
{
	int i;

	if (!pixel_cpu_init)
		return -EPROBE_DEFER;

	for (i = 0; i < pixel_cluster_num; i++) {
		struct em_perf_domain *pd = em_cpu_get(pixel_cluster_start_cpu[i]);

		if (!pd)
			return -EPROBE_DEFER;
	}

	return 0;
}

static bool verify_profile_name(char *name)
{
	char *iter = name;

	if (*name == '\0') {
		pr_err("Empty profile name!\n");
		return false;
	}

	while (*iter) {
		if ((*iter >= 'a' && *iter <= 'z') ||
		    (*iter >= 'A' && *iter <= 'Z') ||
		    (*iter >= '0' && *iter <= '9') ||
		    (*iter == '-' || *iter == '_') ) {
			iter++;
		} else {
			pr_err("Invalid character '%c' in profile name!\n", *iter);
			return false;
		}
	}

	return true;
}

static struct pixel_em_profile *find_profile(const char *name)
{
	struct pixel_em_profile *profile;
	struct list_head *pos;

	mutex_lock(&profile_list_lock);
	list_for_each(pos, &profile_list) {
		profile = list_entry(pos, struct pixel_em_profile, list);
		if (strcmp(name, profile->name) == 0) {
			mutex_unlock(&profile_list_lock);
			return profile;
		}
	}
	mutex_unlock(&profile_list_lock);

	return NULL;
}

/*
 * We assume opps[@opp_id] was updated when this function gets called, which
 * potentially could make the previous opp_id ineffiicent now, so check for
 * this.
 */
static void update_inefficient_prev_opp(struct pixel_em_opp opps[], int opp_id)
{
	if (!opp_id)
		return;

	opps[opp_id-1].inefficient = opps[opp_id-1].cost >= opps[opp_id].cost;
}


static void apply_profile(struct pixel_em_profile *profile)
{
	int cluster_id;

	pr_info("Switching to profile %s...\n", profile->name);

	WRITE_ONCE(active_profile, profile);

	for (cluster_id = 0; cluster_id < profile->num_clusters; cluster_id++) {
		struct pixel_em_cluster *cluster = &profile->clusters[cluster_id];
		int cluster_cap = cluster->opps[cluster->num_opps - 1].capacity;
		int cpu;
		struct cpufreq_policy *policy;

		cpu = cpumask_first(&cluster->cpus);
		policy = cpufreq_cpu_get(cpu);
		if (policy) {
			unsigned int cur_freq;

			spin_lock(&policy->transition_lock);

			cur_freq = policy->cur;

			for_each_cpu(cpu, &cluster->cpus) {
				WRITE_ONCE(per_cpu(cpu_scale, cpu), cluster_cap);
			}

#if IS_ENABLED(CONFIG_VH_SCHED)
			{
				unsigned int max_freq = cluster->opps[cluster->num_opps - 1].freq;
				unsigned long new_freq_scale;
				vh_arch_set_freq_scale_pixel_mod(NULL,
								 &cluster->cpus,
								 cur_freq,
								 max_freq,
								 &new_freq_scale);
				for_each_cpu(cpu, &cluster->cpus) {
					WRITE_ONCE(per_cpu(arch_freq_scale, cpu), new_freq_scale);
				}
			}
#endif
			spin_unlock(&policy->transition_lock);

			schedule_work(&policy->update);
			cpufreq_cpu_put(policy);
		} else {
			pr_err("Could not find cpufreq policy for CPU %d!\n", cpu);
		}

		update_thermal_freq_cap(cpu);
	}
}

static bool update_em_entry(struct pixel_em_profile *profile,
			    int cpu,
			    unsigned int freq,
			    unsigned int cap,
			    unsigned int power)
{
	int cluster_id;
	int opp_id;

	for (cluster_id = 0; cluster_id < profile->num_clusters; cluster_id++) {
		struct pixel_em_cluster *cluster = &profile->clusters[cluster_id];

		if (!cpumask_test_cpu(cpu, &cluster->cpus))
			continue;

		for (opp_id = 0; opp_id < cluster->num_opps; opp_id++) {
			if (cluster->opps[opp_id].freq == freq) {
				cluster->opps[opp_id].capacity = cap;
				cluster->opps[opp_id].power = power;
				cluster->opps[opp_id].cost = power / cap;
				update_inefficient_prev_opp(cluster->opps, opp_id);
				return true;
			}
		}
	}

	pr_err("Could not find OPP for CPU %d, freq %u in profile '%s'!\n",
	       cpu,
	       freq,
	       profile->name);

	return false;
}

#if IS_ENABLED(CONFIG_VH_SCHED)
static bool update_idle_em_entry(struct pixel_idle_em *idle_em,
			    int cpu,
				unsigned int freq,
				unsigned int energy)
{
	int cluster_id;
	int opp_id;

	WARN_ON(energy >= UINT_MAX/2);

	for (cluster_id = 0; cluster_id < idle_em->num_clusters; cluster_id++) {
		struct pixel_em_cluster *cluster = &idle_em->clusters[cluster_id];

		if (!cpumask_test_cpu(cpu, &cluster->cpus))
			continue;

		for (opp_id = 0; opp_id < cluster->num_opps; opp_id++) {
			if (cluster->idle_opps[opp_id].freq == freq) {
				cluster->idle_opps[opp_id].energy = energy;
				return true;
			}
		}
	}

	pr_err("Could not find OPP for CPU %d, freq %u in idle em!\n",
	       cpu,
	       freq);

	return false;
}
#endif

static void update_profile(struct pixel_em_profile *dst, const struct pixel_em_profile *src)
{
	int cluster_id;
	int opp_id;

	if (dst->num_clusters != src->num_clusters) {
		pr_err("Cannot update incompatible profiles (different num_clusters)!\n");
		return;
	}

	for (cluster_id = 0; cluster_id < dst->num_clusters; cluster_id++) {
		struct pixel_em_cluster *dst_cluster = &dst->clusters[cluster_id];
		struct pixel_em_cluster *src_cluster = &src->clusters[cluster_id];

		if (dst_cluster->num_opps != src_cluster->num_opps) {
			pr_err("Cannot update incompatible profiles (different num_opps)!\n");
			return;
		}

		if (!cpumask_equal(&dst_cluster->cpus, &src_cluster->cpus)) {
			pr_err("Cannot update incompatible profiles (different CPU masks)!\n");
			return;
		}

		for (opp_id = 0; opp_id < src_cluster->num_opps; opp_id++) {
			if (dst_cluster->opps[opp_id].freq != src_cluster->opps[opp_id].freq) {
				pr_err("Cannot update incompatible profiles (different CPU freqs)!\n");
				return;
			}
			dst_cluster->opps[opp_id].capacity = src_cluster->opps[opp_id].capacity;
			dst_cluster->opps[opp_id].power = src_cluster->opps[opp_id].power;
			dst_cluster->opps[opp_id].cost = src_cluster->opps[opp_id].cost;
			dst_cluster->opps[opp_id].inefficient = src_cluster->opps[opp_id].inefficient;
		}
	}
}

// Checks that frequencies, capacities and powers are ascending on every cluster.
static bool check_profile_consistency(const struct pixel_em_profile *profile)
{
	int cluster_id;
	int opp_id;

	for (cluster_id = 0; cluster_id < profile->num_clusters; cluster_id++) {
		struct pixel_em_cluster *cluster = &profile->clusters[cluster_id];
		for (opp_id = 1; opp_id < cluster->num_opps; opp_id++) {
			if (cluster->opps[opp_id].freq <= cluster->opps[opp_id -1].freq) {
				pr_err("Non-ascending frequency in profile (freq: %u KHz)!\n",
				       cluster->opps[opp_id].freq);
				return false;
			}
			if (cluster->opps[opp_id].capacity <= cluster->opps[opp_id -1].capacity) {
				pr_err("Non-ascending capacity in profile (capacity: %u)!\n",
				       cluster->opps[opp_id].capacity);
				return false;
			}
			if (cluster->opps[opp_id].power <= cluster->opps[opp_id -1].power) {
				pr_err("Non-ascending power in profile (power: %u mW)!\n",
				       cluster->opps[opp_id].power);
				return false;
			}
		}
	}

	return true;
}

static void scale_profile_capacities(struct pixel_em_profile *profile)
{
	int cluster_id;
	int opp_id;
	unsigned int orig_max_cap = 0;
	const unsigned int scaling_target = 1024;

	for (cluster_id = 0; cluster_id < profile->num_clusters; cluster_id++) {
		struct pixel_em_cluster *cluster = &profile->clusters[cluster_id];
		orig_max_cap = max(orig_max_cap, cluster->opps[cluster->num_opps - 1].capacity);
	}

	for (cluster_id = 0; cluster_id < profile->num_clusters; cluster_id++) {
		struct pixel_em_cluster *cluster = &profile->clusters[cluster_id];
		for (opp_id = 0; opp_id < cluster->num_opps; opp_id++) {
			cluster->opps[opp_id].capacity *= scaling_target;
			cluster->opps[opp_id].capacity /= orig_max_cap;
		}
	}
}

static int parse_profile(const char *profile_input, int profile_input_length)
{
	char *profile_input_dup = kstrndup(profile_input, profile_input_length, GFP_KERNEL);
	char *cur_line;
	char *sep_iterator = profile_input_dup;
	char *profile_name;
	struct pixel_em_profile *profile;
	struct pixel_em_profile *pre_existing_profile;
	int current_cpu_id = -1;
	int res = profile_input_length;

	if (!profile_input_dup) {
		res = -ENOMEM;
		goto early_return;
	}

	profile_name = strsep(&sep_iterator, "\n");
	if (!profile_name || !verify_profile_name(profile_name)) {
		res = -EINVAL;
		goto early_return;
	}

	profile = generate_default_em_profile(profile_name);
	if (!profile) {
		res = -EINVAL;
		goto early_return;
	}

	pre_existing_profile = find_profile(profile->name);
	if (pre_existing_profile) {
		pr_info("Updating profile %s...\n", profile->name);
	}

	while ((cur_line = strsep(&sep_iterator, "\n"))) {
		char *skipped_blanks = skip_spaces(cur_line);

		if (skipped_blanks[0] == '\0' || skipped_blanks[0] == '}') {
			continue;
		} else if (strncasecmp(skipped_blanks, "cpu", 3) == 0) {
			// Expecting a CPU line here...
			if (sscanf(skipped_blanks + 3, "%d", &current_cpu_id) != 1) {
				pr_err("Error when parsing '%s'!\n", skipped_blanks);
				res = -EINVAL;
				goto early_return;
			}
			if (current_cpu_id < 0 || current_cpu_id >= pixel_cpu_num) {
				pr_err("Invalid CPU specified on line '%s'!\n", skipped_blanks);
				res = -EINVAL;
				goto early_return;
			}
			pr_debug("Setting active CPU to %d...\n", current_cpu_id);
		} else if (skipped_blanks[0] != '\0' && skipped_blanks[0] != '}') {
			unsigned int freq = 0;
			unsigned int cap = 0;
			unsigned int power = 0;

			if (current_cpu_id == -1) {
				pr_err("Error: no CPU id specified before parsing '%s'!\n",
				       skipped_blanks);
				res = -EINVAL;
				goto early_return;
			}
			if (sscanf(skipped_blanks, "%u %u %u", &freq, &cap, &power) != 3) {
				pr_err("Error when parsing '%s'!\n", skipped_blanks);
				res = -EINVAL;
				goto early_return;
			}
			if (freq == 0 || cap == 0 || power == 0) {
				pr_err("Illegal freq/cap/power combination specified: %u, %u, %u.\n",
				       freq,
				       cap,
				       power);
				res = -EINVAL;
				goto early_return;
			}

			update_em_entry(profile, current_cpu_id, freq, cap, power);
		}
	}

	if (!check_profile_consistency(profile)) {
		res = -EINVAL;
		goto early_return;
	}

	scale_profile_capacities(profile);

	if (!pre_existing_profile) {
		int file_res = pixel_em_publish_profile(profile);
		if (file_res) {
			pixel_em_free_profile(profile);
			res = file_res;
			goto early_return;
		}
	} else {
		update_profile(pre_existing_profile, profile);
		pixel_em_free_profile(profile);
		profile = pre_existing_profile;
		if (profile == active_profile)
			apply_profile(profile);
	}

early_return:
	kfree(profile_input_dup);
	if (res < 0) {
		pixel_em_free_profile(profile);
	} else {
		pr_info("Successfully created/updated profile '%s'!\n", profile->name);
	}

	return res;
}

static bool generate_em_cluster(struct pixel_em_cluster *dst, struct em_perf_domain *pd)
{
    int first_cpu = cpumask_first(em_span_cpus(pd));
    int cpu_scale = topology_get_cpu_scale(first_cpu);
	int max_freq_index = pd->nr_perf_states - 1;
	unsigned long max_freq = pd->table[max_freq_index].frequency;
	int opp_id;

	cpumask_copy(&dst->cpus, em_span_cpus(pd));

	dst->num_opps = pd->nr_perf_states;

	dst->opps = kcalloc(dst->num_opps, sizeof(*dst->opps), GFP_KERNEL);
	if (!dst->opps)
		return false;

	for (opp_id = 0; opp_id < pd->nr_perf_states; opp_id++) {
		dst->opps[opp_id].freq = pd->table[opp_id].frequency;
		dst->opps[opp_id].power = pd->table[opp_id].power;
		dst->opps[opp_id].capacity = (dst->opps[opp_id].freq * cpu_scale) / max_freq;
		dst->opps[opp_id].cost = pd->table[opp_id].power / dst->opps[opp_id].capacity;
		update_inefficient_prev_opp(dst->opps, opp_id);
	}

	return true;
}

static void deallocate_em_cluster(struct pixel_em_cluster *dst)
{
	kfree(dst->opps);
	dst->opps = NULL;
}

// Returns a valid pixel_em_profile based on default system parameters. This
// profile is NOT yet registered in the profile list, nor associated to sysfs.
static struct pixel_em_profile *generate_default_em_profile(const char *name)
{
	struct pixel_em_profile *res;
	cpumask_t unmatched_cpus;
	int current_cluster_id = 0;

	res = kzalloc(sizeof(*res), GFP_KERNEL);
	if (!res)
		goto failed_res_allocation;

	res->name = kstrdup(name, GFP_KERNEL);
	if (!res->name)
		goto failed_name_allocation;

	res->num_clusters = pixel_cluster_num;

	res->clusters = kcalloc(res->num_clusters, sizeof(*res->clusters), GFP_KERNEL);
	if (!res->clusters)
		goto failed_clusters_allocation;

	res->cpu_to_cluster = kcalloc(pixel_cpu_num,
				      sizeof(*res->cpu_to_cluster),
				      GFP_KERNEL);
	if (!res->cpu_to_cluster)
		goto failed_cpu_to_cluster_allocation;

	cpumask_copy(&unmatched_cpus, cpu_possible_mask);

	while (!cpumask_empty(&unmatched_cpus)) {
		int first_cpu = cpumask_first(&unmatched_cpus);
		struct em_perf_domain *pd = em_cpu_get(first_cpu);
		// pd is guaranteed not to be NULL, as pixel_em_count_clusters completed earlier.
		int pd_cpu;

		if (!generate_em_cluster(&res->clusters[current_cluster_id], pd)) {
			do {
				deallocate_em_cluster(&res->clusters[current_cluster_id]);
			} while (--current_cluster_id >= 0);
			goto failed_cluster_generation;
		}

		for_each_cpu(pd_cpu, em_span_cpus(pd)) {
			res->cpu_to_cluster[pd_cpu] = &res->clusters[current_cluster_id];
		}

		cpumask_xor(&unmatched_cpus, &unmatched_cpus, em_span_cpus(pd));
		current_cluster_id++;
	}

	INIT_LIST_HEAD(&res->list);

	return res;

failed_cluster_generation:
	kfree(res->cpu_to_cluster);

failed_cpu_to_cluster_allocation:
	kfree(res->clusters);

failed_clusters_allocation:
	kfree(res->name);

failed_name_allocation:
	kfree(res);

failed_res_allocation:
	return NULL;
}

#if IS_ENABLED(CONFIG_VH_SCHED)

static bool generate_idle_em_cluster(struct pixel_em_cluster *dst, struct em_perf_domain *pd)
{
	int opp_id;

	cpumask_copy(&dst->cpus, em_span_cpus(pd));

	dst->num_opps = pd->nr_perf_states;

	dst->idle_opps = kcalloc(dst->num_opps, sizeof(*dst->idle_opps), GFP_KERNEL);
	if (!dst->idle_opps)
		return false;

	for (opp_id = 0; opp_id < pd->nr_perf_states; opp_id++) {
		dst->idle_opps[opp_id].freq = pd->table[opp_id].frequency;
		dst->idle_opps[opp_id].energy = 0;
	}

	return true;
}

static void deallocate_idle_em_cluster(struct pixel_em_cluster *dst)
{
	kfree(dst->idle_opps);
	dst->idle_opps = NULL;
}

static struct pixel_idle_em *generate_idle_em(void)
{
	struct pixel_idle_em *idle_em;
	cpumask_t unmatched_cpus;
	int current_cluster_id = 0;

	idle_em = kzalloc(sizeof(*idle_em), GFP_KERNEL);
	if (!idle_em)
		goto failed_idle_em_allocation;

	idle_em->num_clusters = pixel_cluster_num;

	idle_em->clusters = kcalloc(pixel_cluster_num, sizeof(*idle_em->clusters), GFP_KERNEL);
	if (!idle_em->clusters)
		goto failed_clusters_allocation;

	idle_em->cpu_to_cluster = kcalloc(pixel_cpu_num, sizeof(*idle_em->cpu_to_cluster),
					  GFP_KERNEL);
	if (!idle_em->cpu_to_cluster)
		goto failed_cpu_to_cluster_allocation;

	cpumask_copy(&unmatched_cpus, cpu_possible_mask);

	while (!cpumask_empty(&unmatched_cpus)) {
		int first_cpu = cpumask_first(&unmatched_cpus);
		struct em_perf_domain *pd = em_cpu_get(first_cpu);
		// pd is guaranteed not to be NULL, as pixel_em_count_clusters completed earlier.
		int pd_cpu;

		if (!generate_idle_em_cluster(&idle_em->clusters[current_cluster_id], pd)) {
			do {
				deallocate_idle_em_cluster(&idle_em->clusters[current_cluster_id]);
			} while (--current_cluster_id >= 0);
			goto failed_cluster_generation;
		}

		for_each_cpu(pd_cpu, em_span_cpus(pd)) {
			idle_em->cpu_to_cluster[pd_cpu] = &idle_em->clusters[current_cluster_id];
		}

		cpumask_xor(&unmatched_cpus, &unmatched_cpus, em_span_cpus(pd));
		current_cluster_id++;
	}

	return idle_em;

failed_cluster_generation:
	kfree(idle_em->cpu_to_cluster);

failed_cpu_to_cluster_allocation:
	kfree(idle_em->clusters);

failed_clusters_allocation:
	kfree(idle_em);

failed_idle_em_allocation:
	return NULL;

}

static bool parse_idle_em_body(struct pixel_idle_em *idle_em, const char *idle_em_body,
			       int idle_em_body_length)
{
	char *idle_em_body_dup = kstrndup(idle_em_body, idle_em_body_length, GFP_KERNEL);
	char *cur_line;
	char *sep_iterator = idle_em_body_dup;
	int current_cpu_id = -1;
	int ret = idle_em_body_length;

	if (!idle_em_body_dup) {
		ret = -ENOMEM;
		goto early_return;
	}

	while ((cur_line = strsep(&sep_iterator, "\n"))) {
		char *skipped_blanks = skip_spaces(cur_line);

		if (skipped_blanks[0] == '\0' || skipped_blanks[0] == '}') {
			continue;
		} else if (strncasecmp(skipped_blanks, "cpu", 3) == 0) {
			// Expecting a CPU line here...
			if (sscanf(skipped_blanks + 3, "%d", &current_cpu_id) != 1) {
				pr_err("Error when parsing '%s'!\n", skipped_blanks);
				ret = -EINVAL;
				goto early_return;
			}
			if (current_cpu_id < 0 || current_cpu_id >= pixel_cpu_num) {
				pr_err("Invalid CPU specified on line '%s'!\n", skipped_blanks);
				ret = -EINVAL;
				goto early_return;
			}
			pr_debug("Setting active CPU to %d...\n", current_cpu_id);
		} else if (skipped_blanks[0] != '\0' && skipped_blanks[0] != '}') {
			unsigned int freq = 0;
			unsigned int energy = 0;

			if (current_cpu_id == -1) {
				pr_err("Error: no CPU id specified before parsing '%s'!\n",
				       skipped_blanks);
				ret = -EINVAL;
				goto early_return;
			}
			if (sscanf(skipped_blanks, "%u %u", &freq, &energy) != 2) {
				pr_err("Error when parsing '%s'!\n", skipped_blanks);
				ret = -EINVAL;
				goto early_return;
			}
			if (freq == 0 || energy == 0) {
				pr_err("Illegal freq/power combination specified: %u, %u.\n",
				       freq,
				       energy);
				ret = -EINVAL;
				goto early_return;
			}

			update_idle_em_entry(idle_em, current_cpu_id, freq, energy);
		}
	}


early_return:
	kfree(idle_em_body_dup);
	if (ret > 0)
		pr_info("Successfully parsed idle em!\n");

	return (ret > 0);
}

static int parse_idle_em(struct pixel_idle_em *idle_em, const struct device_node *np)
{
	int ret;
	const char *idle_em_body;

	ret = of_property_read_string(np, "idle_energy", &idle_em_body);

	if (ret == 0) {
		ret = parse_idle_em_body(idle_em, idle_em_body, strlen(idle_em_body));
	}

	return ret;
}
#endif

static ssize_t sysfs_write_profile_store(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 const char *buf,
					 size_t count)
{
	int parse_result;

	mutex_lock(&sysfs_lock);
	parse_result = parse_profile(buf, count);
	mutex_unlock(&sysfs_lock);

	return parse_result;
}

static struct kobj_attribute write_profile_attr = __ATTR(write_profile,
							 0220,
							 NULL,
							 sysfs_write_profile_store);

static ssize_t sysfs_active_profile_show(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 char *buf)
{
	ssize_t res = 0;
	struct pixel_em_profile *profile_snapshot;

	mutex_lock(&sysfs_lock);

	profile_snapshot = READ_ONCE(active_profile);

	res = profile_snapshot
		? sysfs_emit(buf, "%s\n", profile_snapshot->name)
		: -EINVAL;

	mutex_unlock(&sysfs_lock);
	return res;
}

static ssize_t sysfs_active_profile_store(struct kobject *kobj,
					  struct kobj_attribute *attr,
					  const char *buf,
					  size_t count)
{
	int res = count;
	char *profile_name = kstrndup(buf, count, GFP_KERNEL);
	char *iter = profile_name;
	struct pixel_em_profile *profile;

	if (!profile_name)
		return -ENOMEM;

	while (*iter) {
		if (*iter == '\n') {
			*iter = '\0';
			break;
		}
		iter++;
	}

	mutex_lock(&sysfs_lock);
	profile = find_profile(profile_name);
	if (profile)
		apply_profile(profile);
	else
		res = -EINVAL;
	mutex_unlock(&sysfs_lock);

	kfree(profile_name);
	return res;
}

static struct kobj_attribute active_profile_attr = __ATTR(active_profile,
							  0664,
							  sysfs_active_profile_show,
							  sysfs_active_profile_store);

#if IS_ENABLED(CONFIG_VH_SCHED)

static ssize_t sysfs_idle_profile_show(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 char *buf)
{
	ssize_t res = 0;
	int cluster_id;

	mutex_lock(&sysfs_lock);

	res = sysfs_emit(buf, "idle\n");

	if (idle_profile != NULL) {
		for (cluster_id = 0; cluster_id < idle_profile->num_clusters; cluster_id++) {
			int opp_id;
			int first_cpu = cpumask_first(&idle_profile->clusters[cluster_id].cpus);

			res += sysfs_emit_at(buf, res, "cpu%d {\n", first_cpu);

			for (opp_id = 0;
				opp_id < idle_profile->clusters[cluster_id].num_opps;
				opp_id++)
				res += sysfs_emit_at(buf,
							res,
							"%u %u\n",
							idle_profile->clusters[cluster_id].idle_opps[opp_id].freq,
							idle_profile->clusters[cluster_id].idle_opps[opp_id].energy);
		}

		res += sysfs_emit_at(buf, res, "}\n");
	}

	mutex_unlock(&sysfs_lock);
	return res;
}

static ssize_t sysfs_idle_profile_store(struct kobject *kobj,
					  struct kobj_attribute *attr,
					  const char *buf,
					  size_t count)
{
	int res = 0;
	bool parse_result;
	struct pixel_idle_em *new_idle_profile = NULL;

	mutex_lock(&sysfs_lock);

	new_idle_profile = generate_idle_em();
	if (new_idle_profile == NULL) {
		pr_err("Pixel idle em not generated!\n");
		res = -ENOMEM;
		goto unlock;
	}
	parse_result = parse_idle_em_body(new_idle_profile, buf, count);
	if (parse_result) {
		struct pixel_idle_em *old_idle_profile = idle_profile;
		unsigned long flags;
		idle_profile = new_idle_profile;
		raw_spin_lock_irqsave(&vendor_sched_pixel_em_lock, flags);
		WRITE_ONCE(vendor_sched_pixel_idle_em, idle_profile);
		raw_spin_unlock_irqrestore(&vendor_sched_pixel_em_lock, flags);
		pixel_em_free_idle(old_idle_profile);
		res = count;
	} else {
		res = -EINVAL;
	}

unlock:
	mutex_unlock(&sysfs_lock);

	return res;
}

static struct kobj_attribute idle_profile_attr = __ATTR(idle_profile,
							  0664,
							  sysfs_idle_profile_show,
							  sysfs_idle_profile_store);


static ssize_t sysfs_idle_profile_enable_show(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 char *buf)
{
	ssize_t res = 0;

	mutex_lock(&sysfs_lock);

	if (READ_ONCE(vendor_sched_pixel_idle_em) != NULL) {
		res = sysfs_emit(buf, "1\n");
	} else {
		res = sysfs_emit(buf, "0\n");
	}

	mutex_unlock(&sysfs_lock);
	return res;
}

static ssize_t sysfs_idle_profile_enable_store(struct kobject *kobj,
					  struct kobj_attribute *attr,
					  const char *buf,
					  size_t count)
{
	unsigned long flags;
	bool enable;
	int res = kstrtobool(buf, &enable);
	if (res) {
		return res;
	}

	mutex_lock(&sysfs_lock);

	raw_spin_lock_irqsave(&vendor_sched_pixel_em_lock, flags);
	if (enable) {
		WRITE_ONCE(vendor_sched_pixel_idle_em, idle_profile);
	} else {
		WRITE_ONCE(vendor_sched_pixel_idle_em, NULL);
	}
	raw_spin_unlock_irqrestore(&vendor_sched_pixel_em_lock, flags);

	mutex_unlock(&sysfs_lock);

	return count;
}

static struct kobj_attribute idle_profile_enable_attr = __ATTR(idle_profile_enable,
							  0664,
							  sysfs_idle_profile_enable_show,
							  sysfs_idle_profile_enable_store);
#endif

struct profile_sysfs_helper {
	struct kobj_attribute kobj_attr;
	struct pixel_em_profile *profile;
};

static ssize_t sysfs_profile_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	ssize_t res = 0;
	int cluster_id;
	struct pixel_em_profile *profile = ((struct profile_sysfs_helper *) attr)->profile;

	mutex_lock(&sysfs_lock);

	res += sysfs_emit_at(buf, res, "%s\n", profile->name);

	for (cluster_id = 0; cluster_id < profile->num_clusters; cluster_id++) {
		int opp_id;
		int first_cpu = cpumask_first(&profile->clusters[cluster_id].cpus);

		res += sysfs_emit_at(buf, res, "cpu%d {\n", first_cpu);

		for (opp_id = 0;
		     opp_id < profile->clusters[cluster_id].num_opps;
		     opp_id++)
			res += sysfs_emit_at(buf,
					     res,
					     "%u %u %u %lu %d\n",
					     profile->clusters[cluster_id].opps[opp_id].freq,
					     profile->clusters[cluster_id].opps[opp_id].capacity,
					     profile->clusters[cluster_id].opps[opp_id].power,
					     profile->clusters[cluster_id].opps[opp_id].cost,
					     profile->clusters[cluster_id].opps[opp_id].inefficient
					     );

		res += sysfs_emit_at(buf, res, "}\n");
	}
	mutex_unlock(&sysfs_lock);
	return res;
}

// Creates a sysfs file for the target profile (in the profiles/ folder), and also adds the
// profile to the profiles list.
static int pixel_em_publish_profile(struct pixel_em_profile *profile)
{
	profile->sysfs_helper = kzalloc(sizeof(*profile->sysfs_helper), GFP_KERNEL);
	if (!profile->sysfs_helper)
		return -ENOMEM;

	profile->sysfs_helper->profile = profile;
	sysfs_attr_init(&profile->sysfs_helper->kobj_attr.attr);
	profile->sysfs_helper->kobj_attr.attr.name = profile->name;
	profile->sysfs_helper->kobj_attr.attr.mode = 0664;
	profile->sysfs_helper->kobj_attr.show = sysfs_profile_show;

	if (sysfs_create_file(profiles_sysfs_folder, &profile->sysfs_helper->kobj_attr.attr)) {
		pr_err("Failed to create profile file for '%s'!\n", profile->name);
		kfree(profile->sysfs_helper);
		profile->sysfs_helper = NULL;
		return -EINVAL;
	}

	mutex_lock(&profile_list_lock);
	list_add(&profile->list, &profile_list);
	mutex_unlock(&profile_list_lock);
	return 0;
}

static void pixel_em_unpublish_profile(struct pixel_em_profile *profile)
{
	if (!profile->sysfs_helper)
		return;

	mutex_lock(&profile_list_lock);
	list_del(&profile->list);
	mutex_unlock(&profile_list_lock);

	sysfs_remove_file(profiles_sysfs_folder, &profile->sysfs_helper->kobj_attr.attr);
	kfree(profile->sysfs_helper);
	profile->sysfs_helper = NULL;
}

static void pixel_em_free_profile(struct pixel_em_profile *profile)
{
	int cluster_id;

	if (!profile)
		return;

	if (profile->sysfs_helper) {
		// When a profile was published (i.e. got sysfs files / was inserted in
		// the profiles list), we cannot guarantee that no driver client retains
		// a reference to it: the sysfs file can be removed, but the rest of the
		// profile cannot be deallocated.
		pixel_em_unpublish_profile(profile);
		return;
	}

	kfree(profile->name);

	for (cluster_id = 0; cluster_id < profile->num_clusters; cluster_id++) {
		deallocate_em_cluster(&profile->clusters[cluster_id]);
	}
	kfree(profile->clusters);
	kfree(profile->cpu_to_cluster);
	kfree(profile);
}

#if IS_ENABLED(CONFIG_VH_SCHED)
static void pixel_em_free_idle(struct pixel_idle_em *idle_em)
{
	int cluster_id;

	if (!idle_em)
		return;

	for (cluster_id = 0; cluster_id < idle_em->num_clusters; cluster_id++) {
		deallocate_idle_em_cluster(&idle_em->clusters[cluster_id]);
	}
	kfree(idle_em->clusters);
	kfree(idle_em->cpu_to_cluster);
	kfree(idle_em);
}
#endif

static void pixel_em_clean_up_sysfs_nodes(void)
{
	if (!primary_sysfs_folder)
		return;

#if IS_ENABLED(CONFIG_VH_SCHED)
	sysfs_remove_file(primary_sysfs_folder, &idle_profile_attr.attr);
	sysfs_remove_file(primary_sysfs_folder, &idle_profile_enable_attr.attr);
#endif
	sysfs_remove_file(primary_sysfs_folder, &active_profile_attr.attr);
	sysfs_remove_file(primary_sysfs_folder, &write_profile_attr.attr);

	if (profiles_sysfs_folder) {
		struct pixel_em_profile *profile;
		struct list_head *pos, *tmp;

		list_for_each_safe(pos, tmp, &profile_list) {
			profile = list_entry(pos, struct pixel_em_profile, list);
			pixel_em_free_profile(profile);
		}
		kobject_put(profiles_sysfs_folder);
		profiles_sysfs_folder = NULL;
	}

	kobject_put(primary_sysfs_folder);
	primary_sysfs_folder = NULL;
}

static int pixel_em_initialize_sysfs_nodes(void)
{
	if (primary_sysfs_folder) {
		pr_err("Sysfs nodes already initialized!\n");
		return -EINVAL;
	}

	primary_sysfs_folder = kobject_create_and_add("pixel_em", kernel_kobj);
	if (!primary_sysfs_folder) {
		pr_err("Failed to create primary sysfs folder!\n");
		return -EINVAL;
	}

	profiles_sysfs_folder = kobject_create_and_add("profiles", primary_sysfs_folder);
	if (!profiles_sysfs_folder) {
		pr_err("Failed to create profiles sysfs folder!\n");
		return -EINVAL;
	}


	if (sysfs_create_file(primary_sysfs_folder, &write_profile_attr.attr)) {
		pr_err("Failed to create write_profile file!\n");
		return -EINVAL;
	}

	if (sysfs_create_file(primary_sysfs_folder, &active_profile_attr.attr)) {
		pr_err("Failed to create active_profile file!\n");
		return -EINVAL;
	}

#if IS_ENABLED(CONFIG_VH_SCHED)
	if (sysfs_create_file(primary_sysfs_folder, &idle_profile_attr.attr)) {
		pr_err("Failed to create idle_profile file!\n");
		return -EINVAL;
	}

	if (sysfs_create_file(primary_sysfs_folder, &idle_profile_enable_attr.attr)) {
		pr_err("Failed to create idle_profile_enable file!\n");
		return -EINVAL;
	}
#endif

	return 0;
}

static void pixel_em_drv_undo_probe(void)
{
	// Note: removing/unloading this driver after a successful probe is not expected to ever
	// happen (other than debugging).

	pixel_em_clean_up_sysfs_nodes();
#if IS_ENABLED(CONFIG_VH_SCHED)
	pixel_em_free_idle(idle_profile);
	idle_profile = NULL;
	vendor_sched_pixel_idle_em = NULL;
#endif

	if (!platform_dev) {
		// 'platform_dev' gets set when probing is successful. When that point is reached,
		// there is no way to know whether freeing cpu_to_em_pd or em_pds is safe (as
		// the pointers may have been shared with other drivers without reference tracking).
		// => If platform_dev is NULL, free these pointers (if they're not NULL themselves).
		//    Otherwise, set them to NULL without freeing.
	}

	platform_dev = NULL;
}

static int pixel_em_drv_probe(struct platform_device *dev)
{
	int res;
	struct pixel_em_profile *default_profile;
	int num_dt_profiles;
	unsigned long flags;
	int i;

	mutex_init(&sysfs_lock);
	mutex_init(&profile_list_lock);
	INIT_LIST_HEAD(&profile_list);

	res = pixel_em_init_cpu_layout();
	if (res < 0) {
		pixel_em_drv_undo_probe();
		return res;
	}

	default_profile = generate_default_em_profile("default");
	if (default_profile == NULL) {
		pixel_em_drv_undo_probe();
		return -ENOMEM;
	}

#if IS_ENABLED(CONFIG_VH_SCHED)
	idle_profile = generate_idle_em();
	if (idle_profile == NULL)
		pr_warn("Pixel idle em not generated!\n");

	if (idle_profile &&
	    parse_idle_em(idle_profile, dev->dev.of_node) < 0) {
		pixel_em_free_idle(idle_profile);
		idle_profile = NULL;
		pr_warn("Pixel idle em not parsed!\n");
	}
	raw_spin_lock_irqsave(&vendor_sched_pixel_em_lock, flags);
	WRITE_ONCE(vendor_sched_pixel_idle_em, idle_profile);
	raw_spin_unlock_irqrestore(&vendor_sched_pixel_em_lock, flags);
#endif

	res = pixel_em_initialize_sysfs_nodes();
	if (res < 0) {
		pixel_em_drv_undo_probe();
		return res;
	}

	res = pixel_em_publish_profile(default_profile);
	if (res) {
		pixel_em_drv_undo_probe();
		return res;
	}

	num_dt_profiles = of_property_count_strings(dev->dev.of_node, "profiles");
	if (num_dt_profiles >= 0)
		pr_info("Loading %d profile(s).\n", num_dt_profiles);

	for (i = 0; i < num_dt_profiles; i++) {
		const char *profile_body;
		int res = of_property_read_string_index(dev->dev.of_node,
							"profiles",
							i,
							&profile_body);
		if (!res) {
			res = parse_profile(profile_body, strlen(profile_body));
			if (res <= 0) {
				pr_err("Error parsing profile #%d.\n", i);
				pixel_em_drv_undo_probe();
				return res;
			}
		} else {
			pr_err("Error retrieving profile #%d.\n", i);
			pixel_em_drv_undo_probe();
			return res;
		}
	}

	apply_profile(default_profile);

	// Probe is successful => do not attempt to free cpu_to_em_pd.
	platform_dev = dev;

	// Register EM table to all needed drivers here.
#if IS_ENABLED(CONFIG_VH_SCHED)
	pr_info("Publishing EM profile to vh_sched!\n");
	WRITE_ONCE(vendor_sched_pixel_em_profile, &active_profile);
#endif

#if IS_ENABLED(CONFIG_EXYNOS_CPU_THERMAL)
	pr_info("Publishing EM profile to exynos_cpu_cooling!\n");
	WRITE_ONCE(exynos_cpu_cooling_pixel_em_profile, &active_profile);
#endif

	return 0;
}

static int pixel_em_drv_remove(struct platform_device *dev)
{
	pixel_em_drv_undo_probe();

	return 0;
}

static const struct of_device_id pixel_em_of_match[] = {
	{
		.compatible = "google,pixel-em",
	},
	{}
};

static struct platform_driver pixel_em_platform_driver = {
	.probe = pixel_em_drv_probe,
	.remove = pixel_em_drv_remove,
	.driver = {
		.name = "pixel-em",
		.owner = THIS_MODULE,
		.of_match_table = pixel_em_of_match,
	},
};

static int __init pixel_em_init(void)
{
	if (platform_driver_register(&pixel_em_platform_driver))
		pr_err("Error when registering driver!\n");

	pr_info("Registered! :D\n");

	return 0;
}

static void __exit pixel_em_exit(void)
{
	pixel_em_drv_undo_probe();
	pr_info("Unregistered! :(\n");
}

module_init(pixel_em_init);
module_exit(pixel_em_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Vincent Palomares");
MODULE_DESCRIPTION("Pixel Energy Model Driver");
MODULE_DEVICE_TABLE(of, pixel_em_of_match);
