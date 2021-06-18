// SPDX-License-Identifier: GPL-2.0-only
/* pixel_em.c
 *
 * Support for runtime-customizable table-based Energy Model
 *
 * Copyright 2022 Google LLC
 */

#define pr_fmt(fmt) "pixel-em: " fmt

#include <linux/bitops.h>
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

#if IS_ENABLED(CONFIG_VH_SCHED)
extern struct em_perf_domain **vendor_sched_cpu_to_em_pd;
#endif

#if IS_ENABLED(CONFIG_EXYNOS_CPU_THERMAL)
extern struct em_perf_domain **exynos_cpu_cooling_cpu_to_em_pd;
#endif

static int pixel_em_max_cpu;
static int pixel_em_num_cpu_pds;

static struct em_perf_domain **em_pds;
static struct em_perf_domain **em_pds_backup;
static struct em_perf_domain **cpu_to_em_pd;
static int *em_pd_to_cpu; // Map em_pd indice to the first CPU of a PD.

static struct mutex sysfs_em_pd_lock; // Synchronize sysfs EM table accesses.
static struct kobject *primary_sysfs_folder;

static struct platform_device *platform_dev;

static int pixel_em_count_cpu_pds(void)
{
	int res = 0;

	cpumask_t unmatched_cpus;

	cpumask_copy(&unmatched_cpus, cpu_possible_mask);

	while (!cpumask_empty(&unmatched_cpus)) {
		int first_cpu = cpumask_first(&unmatched_cpus);
		struct em_perf_domain *pd = em_cpu_get(first_cpu);

		if (!pd)
			return -EPROBE_DEFER;

		cpumask_xor(&unmatched_cpus, &unmatched_cpus, em_span_cpus(pd));
		res++;
	}

	return res;
}

static struct em_perf_domain *clone_pd(const struct em_perf_domain *original_pd)
{
	struct em_perf_domain *pd;

	pd = kzalloc(sizeof(*pd) + cpumask_size(), GFP_KERNEL);
	if (!pd)
		return NULL;

	cpumask_copy(em_span_cpus(pd), em_span_cpus(original_pd));

	pd->nr_perf_states = original_pd->nr_perf_states;
	pd->milliwatts = original_pd->milliwatts;

	pd->table = kcalloc(original_pd->nr_perf_states, sizeof(struct em_perf_state), GFP_KERNEL);
	if (!pd->table) {
		kfree(pd);
		return NULL;
	}

	memcpy(pd->table, original_pd->table,
	       sizeof(struct em_perf_state) * original_pd->nr_perf_states);

	return pd;
}

static void copy_pd_table(struct em_perf_domain *dest_pd, const struct em_perf_domain *orig_pd)
{
	int perf_state_id;

	if (dest_pd->nr_perf_states != orig_pd->nr_perf_states) {
		pr_err("Trying to copy PDs of different sizes %d and %d!\n",
		       dest_pd->nr_perf_states,
		       orig_pd->nr_perf_states);
		return;
	}

	for (perf_state_id = 0; perf_state_id < orig_pd->nr_perf_states; perf_state_id++) {
		dest_pd->table[perf_state_id].power = orig_pd->table[perf_state_id].power;
		dest_pd->table[perf_state_id].cost = orig_pd->table[perf_state_id].cost;
	}
}

static void free_pd(const struct em_perf_domain *pd)
{
	if (!pd)
		return;

	kfree(pd->table);
	kfree(pd);
}

static int pixel_em_init_cpu_layout(void)
{
	cpumask_t unmatched_cpus;
	int current_pd_id = 0;
	int num_cpu_pds;

	num_cpu_pds = pixel_em_count_cpu_pds();
	if (num_cpu_pds <= 0)
		return num_cpu_pds;
	pixel_em_num_cpu_pds = num_cpu_pds;

	pixel_em_max_cpu = cpumask_last(cpu_possible_mask);

	cpu_to_em_pd = kcalloc(pixel_em_max_cpu + 1, sizeof(*cpu_to_em_pd), GFP_KERNEL);
	if (!cpu_to_em_pd)
		return -ENOMEM;

	em_pd_to_cpu = kcalloc(num_cpu_pds, sizeof(*em_pd_to_cpu), GFP_KERNEL);
	if (!em_pd_to_cpu)
		return -ENOMEM;

	em_pds = kcalloc(num_cpu_pds, sizeof(*em_pds), GFP_KERNEL);
	if (!em_pds)
		return -ENOMEM;

	em_pds_backup = kcalloc(num_cpu_pds, sizeof(*em_pds_backup), GFP_KERNEL);
	if (!em_pds_backup)
		return -ENOMEM;

	cpumask_copy(&unmatched_cpus, cpu_possible_mask);

	while (!cpumask_empty(&unmatched_cpus)) {
		int first_cpu = cpumask_first(&unmatched_cpus);
		int pd_cpu;
		struct em_perf_domain *pd = em_cpu_get(first_cpu);
		// pd is guaranteed not to be NULL, as pixel_em_count_cpu_pds completed earlier.

		em_pd_to_cpu[current_pd_id] = first_cpu;

		em_pds[current_pd_id] = clone_pd(pd);
		if (!em_pds[current_pd_id])
			return -ENOMEM;

		em_pds_backup[current_pd_id] = clone_pd(em_pds[current_pd_id]);
		if (!em_pds_backup[current_pd_id])
			return -ENOMEM;

		for_each_cpu(pd_cpu, em_span_cpus(pd)) {
			pr_debug("For CPU %d's domain, seeing CPU %d.\n", first_cpu, pd_cpu);
			cpu_to_em_pd[pd_cpu] = em_pds[current_pd_id];
		}

		cpumask_xor(&unmatched_cpus, &unmatched_cpus, em_span_cpus(pd));
		current_pd_id++;
	}

	return 0;
}

static ssize_t sysfs_profile_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	ssize_t res = 0;
	int pd_id;

	mutex_lock(&sysfs_em_pd_lock);
	for (pd_id = 0; pd_id < pixel_em_num_cpu_pds; pd_id++) {
		int perf_state_id;

		res += sysfs_emit_at(buf, res, "cpu%d {\n", em_pd_to_cpu[pd_id]);

		for (perf_state_id = 0;
		     perf_state_id < em_pds[pd_id]->nr_perf_states;
		     perf_state_id++)
			res += sysfs_emit_at(buf,
					     res,
					     "%lu %lu\n",
					     em_pds[pd_id]->table[perf_state_id].frequency,
					     em_pds[pd_id]->table[perf_state_id].power);

		res += sysfs_emit_at(buf, res, "}\n");
	}
	mutex_unlock(&sysfs_em_pd_lock);
	return res;
}

static void update_em_entry(int cpu_id, unsigned long freq, unsigned long power)
{
	int target_opp_index = 0;
	int max_freq_index;
	unsigned long max_freq;

	while (target_opp_index < cpu_to_em_pd[cpu_id]->nr_perf_states) {
		if (cpu_to_em_pd[cpu_id]->table[target_opp_index].frequency >= freq)
			break;
		target_opp_index++;
	}

	max_freq_index = cpu_to_em_pd[cpu_id]->nr_perf_states - 1;
	max_freq = cpu_to_em_pd[cpu_id]->table[max_freq_index].frequency;

	cpu_to_em_pd[cpu_id]->table[target_opp_index].power = power;
	cpu_to_em_pd[cpu_id]->table[target_opp_index].cost = div64_u64(max_freq * power, freq);
	pr_info("Updating cpu_em[%d][%luKHz] to {%lu mW, %lu cost}.\n",
		cpu_id,
		freq,
		power,
		cpu_to_em_pd[cpu_id]->table[target_opp_index].cost);
}

static int parse_profile(const char *profile, int profile_length)
{
	char *profile_dup = kstrndup(profile, profile_length, GFP_KERNEL);
	char *cur_line;
	char *sep_iterator = profile_dup;

	int current_cpu_id = -1;

	int res = 0;

	if (!profile_dup)
		return -ENOMEM;

	while ((cur_line = strsep(&sep_iterator, "\n"))) {
		char *skipped_blanks = skip_spaces(cur_line);

		if (skipped_blanks[0] == '\0' || skipped_blanks[0] == '}') {
			continue;
		} else if (strncasecmp(skipped_blanks, "cpu", 3) == 0) {
			// Expecting a CPU line here...
			if (sscanf(skipped_blanks + 3, "%d", &current_cpu_id) != 1) {
				pr_err("Error when parsing '%s'!\n", skipped_blanks);
				res = -EINVAL;
				break;
			}
			if (current_cpu_id < 0 || current_cpu_id > pixel_em_max_cpu) {
				pr_err("Invalid CPU specified on line '%s'!\n", skipped_blanks);
				res = -EINVAL;
				break;
			}
			pr_debug("Setting active CPU to %d...\n", current_cpu_id);
		} else if (skipped_blanks[0] != '\0' && skipped_blanks[0] != '}') {
			unsigned long freq = 0;
			unsigned long power = 0;

			if (current_cpu_id == -1) {
				pr_err("Error: no CPU id specified before parsing '%s'!\n",
				       skipped_blanks);
				res = -EINVAL;
				break;
			}
			if (sscanf(skipped_blanks, "%lu %lu", &freq, &power) != 2) {
				pr_err("Error when parsing '%s'!\n", skipped_blanks);
				res = -EINVAL;
				break;
			}
			pr_info("Scanned freq %luKHz, power %lumW for CPU%d.\n",
				freq,
				power,
				current_cpu_id);
			if (freq == 0 || power == 0) {
				pr_err("Illegal freq/power combination specified: %lu, %lu.\n",
				       freq,
				       power);
				res = -EINVAL;
				break;
			}

			update_em_entry(current_cpu_id, freq, power);
		}
	}

	kfree(profile_dup);
	return res;
}

static ssize_t sysfs_profile_store(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   const char *buf,
				   size_t count)
{
	bool parse_successful;

	if (strncasecmp(buf, "default", sizeof("default") - 1) == 0) {
		int pd_id;

		mutex_lock(&sysfs_em_pd_lock);
		pr_info("Restoring default profile.\n");
		for (pd_id = 0; pd_id < pixel_em_num_cpu_pds; pd_id++)
			copy_pd_table(em_pds[pd_id], em_pds_backup[pd_id]);
		mutex_unlock(&sysfs_em_pd_lock);
		return count;
	}

	mutex_lock(&sysfs_em_pd_lock);
	parse_successful = parse_profile(buf, count);
	mutex_unlock(&sysfs_em_pd_lock);

	return parse_successful ? count : -EINVAL;
}

static struct kobj_attribute profile_attr = __ATTR(profile,
						   0660,
						   sysfs_profile_show,
						   sysfs_profile_store);

static void pixel_em_clean_up_sysfs_nodes(void)
{
	if (!primary_sysfs_folder)
		return;

	sysfs_remove_file(primary_sysfs_folder, &profile_attr.attr);

	kobject_put(primary_sysfs_folder);
	primary_sysfs_folder = NULL;
}

static int pixel_em_initialize_sysfs_nodes(void)
{
	if (primary_sysfs_folder) {
		pr_err("Sysfs nodes already initialized!");
		return -EINVAL;
	}

	primary_sysfs_folder = kobject_create_and_add("pixel_em", kernel_kobj);
	if (!primary_sysfs_folder) {
		pr_err("Failed to create primary sysfs folder!");
		return -EINVAL;
	}

	if (sysfs_create_file(primary_sysfs_folder, &profile_attr.attr)) {
		pr_err("Failed to create profile file!\n");
		return -EINVAL;
	}

	return 0;
}

static void pixel_em_drv_undo_probe(void)
{
	// Note: removing/unloading this driver after a successful probe is not expected to ever
	// happen (other than debugging).

	pixel_em_clean_up_sysfs_nodes();

	if (!platform_dev) {
		// 'platform_dev' gets set when probing is successful. When that point is reached,
		// there is no way to know whether freeing cpu_to_em_pd or em_pds is safe (as
		// the pointers may have been shared with other drivers without reference tracking).
		// => If platform_dev is NULL, free these pointers (if they're not NULL themselves).
		//    Otherwise, set them to NULL without freeing.
		kfree(cpu_to_em_pd);

		if (em_pds) {
			int i;

			for (i = 0; i < pixel_em_num_cpu_pds; i++) {
				if (em_pds[i])
					free_pd(em_pds[i]);
			}
			kfree(em_pds);
		}
	}

	kfree(em_pd_to_cpu);
	em_pd_to_cpu = NULL;

	if (em_pds_backup) {
		int i;

		for (i = 0; i < pixel_em_num_cpu_pds; i++) {
			if (em_pds_backup[i])
				free_pd(em_pds_backup[i]);
		}
		kfree(em_pds_backup);
		em_pds_backup = NULL;
	}

	cpu_to_em_pd = NULL;
	em_pds = NULL;
	platform_dev = NULL;
}

static int pixel_em_drv_probe(struct platform_device *dev)
{
	int res;
	const char *dt_profile;

	res = pixel_em_init_cpu_layout();
	if (res < 0) {
		pixel_em_drv_undo_probe();
		return res;
	}

	if (of_property_read_string(dev->dev.of_node, "profile", &dt_profile)) {
		pr_info("Could not find EM profile in device tree.\n");
	} else {
		int pd_id;

		pr_info("Loading profile from DT.\n");
		parse_profile(dt_profile, strlen(dt_profile));

		// Override backup values with DT profile.
		for (pd_id = 0; pd_id < pixel_em_num_cpu_pds; pd_id++)
			copy_pd_table(em_pds_backup[pd_id], em_pds[pd_id]);
	}

	res = pixel_em_initialize_sysfs_nodes();
	if (res < 0) {
		pixel_em_drv_undo_probe();
		return res;
	}

	// Probe is successful => do not attempt to free pixel_em_max_cpu or cpu_to_em_pd.
	platform_dev = dev;

	// Register EM table to all needed drivers here.
#if IS_ENABLED(CONFIG_VH_SCHED)
	pr_info("Publishing PDs to vh_sched!\n");
	WRITE_ONCE(vendor_sched_cpu_to_em_pd, cpu_to_em_pd);
#endif

#if IS_ENABLED(CONFIG_EXYNOS_CPU_THERMAL)
	pr_info("Publishing PDs to exynos_cpu_cooling!\n");
	WRITE_ONCE(exynos_cpu_cooling_cpu_to_em_pd, cpu_to_em_pd);
#endif

	return res;
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
	mutex_init(&sysfs_em_pd_lock);
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
