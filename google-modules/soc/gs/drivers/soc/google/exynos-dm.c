// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/sched/clock.h>
#if IS_ENABLED(CONFIG_DEBUG_SNAPSHOT)
#include <soc/google/debug-snapshot.h>
#endif
#include <trace/hooks/systrace.h>
#if IS_ENABLED(CONFIG_GS_ACPM)
#include "acpm/acpm.h"
#include "acpm/acpm_ipc.h"
#include "cal-if/acpm_dvfs.h"
#include "cal-if/cmucal.h"
#endif

#include <soc/google/exynos-dm.h>

#define DM_EMPTY	0xFF
static struct exynos_dm_device *exynos_dm;

/*
 * SYSFS for Debugging
 */
static ssize_t available_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct exynos_dm_device *dm = platform_get_drvdata(pdev);
	ssize_t count = 0;
	int i;

	for (i = 0; i < dm->domain_count; i++) {
		if (!dm->dm_data[i].available)
			continue;

		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "dm_type: %d(%s), available = %s\n",
				   dm->dm_data[i].dm_type, dm->dm_data[i].dm_type_name,
				   dm->dm_data[i].available ? "true" : "false");
	}

	return count;
}

static ssize_t show_constraint_table(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct list_head *constraint_list;
	struct exynos_dm_constraint *constraint;
	struct exynos_dm_data *dm;
	struct exynos_dm_attrs *dm_attrs;
	ssize_t count = 0;
	int i;

	dm_attrs = container_of(attr, struct exynos_dm_attrs, attr);
	dm = container_of(dm_attrs, struct exynos_dm_data, constraint_table_attr);

	if (!dm->available) {
		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "This dm_type is not available\n");
		return count;
	}

	count += scnprintf(buf + count, PAGE_SIZE - count, "dm_type: %s\n",
			   dm->dm_type_name);

	constraint_list = &dm->min_constraints;
	if (list_empty(constraint_list)) {
		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "This dm_type have not min constraint tables\n\n");
		goto next;
	}

	list_for_each_entry(constraint, constraint_list, driver_domain) {
		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "-------------------------------------------------\n");
		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "constraint_dm_type = %s\n", constraint->dm_type_name);
		count += scnprintf(buf + count, PAGE_SIZE - count, "constraint_type: %s\n",
				   constraint->constraint_type ? "MAX" : "MIN");
		count += scnprintf(buf + count, PAGE_SIZE - count, "guidance: %s\n",
				   constraint->guidance ? "true" : "false");
		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "const_freq = %u, gov_freq =%u\n",
				   constraint->const_freq, constraint->gov_freq);
		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "driver_freq\t constraint_freq\n");
		for (i = 0; i < constraint->table_length; i++)
			count += scnprintf(buf + count, PAGE_SIZE - count, "%10u\t %10u\n",
					   constraint->freq_table[i].driver_freq,
					   constraint->freq_table[i].constraint_freq);
		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "-------------------------------------------------\n");
	}

next:
	constraint_list = &dm->max_constraints;
	if (list_empty(constraint_list)) {
		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "This dm_type have not max constraint tables\n\n");
		return count;
	}

	list_for_each_entry(constraint, constraint_list, driver_domain) {
		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "-------------------------------------------------\n");
		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "constraint_dm_type = %s\n", constraint->dm_type_name);
		count += scnprintf(buf + count, PAGE_SIZE - count, "constraint_type: %s\n",
				   constraint->constraint_type ? "MAX" : "MIN");
		count += scnprintf(buf + count, PAGE_SIZE - count, "guidance: %s\n",
				   constraint->guidance ? "true" : "false");
		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "max_freq =%u\n",
				   constraint->const_freq);
		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "driver_freq\t constraint_freq\n");
		for (i = 0; i < constraint->table_length; i++)
			count += scnprintf(buf + count, PAGE_SIZE - count, "%10u\t %10u\n",
					   constraint->freq_table[i].driver_freq,
					   constraint->freq_table[i].constraint_freq);
		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "-------------------------------------------------\n");
	}

	return count;
}

static ssize_t show_dm_policy(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct list_head *constraint_list;
	struct exynos_dm_constraint *constraint;
	struct exynos_dm_data *dm;
	struct exynos_dm_attrs *dm_attrs;
	ssize_t count = 0;
	u32 find;
	int i;

	dm_attrs = container_of(attr, struct exynos_dm_attrs, attr);
	dm = container_of(dm_attrs, struct exynos_dm_data, dm_policy_attr);

	if (!dm->available) {
		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "This dm_type is not available\n");
		return count;
	}

	count += scnprintf(buf + count, PAGE_SIZE - count, "dm_type: %s\n",
			   dm->dm_type_name);

	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "policy_min = %u, policy_max = %u\n",
			   dm->policy_min, dm->policy_max);
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "const_min = %u, const_max = %u\n",
			   dm->const_min, dm->const_max);
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "gov_min = %u, governor_freq = %u\n", dm->gov_min, dm->governor_freq);
	count += scnprintf(buf + count, PAGE_SIZE - count, "current_freq = %u\n", dm->cur_freq);
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "-------------------------------------------------\n");
	count += scnprintf(buf + count, PAGE_SIZE - count, "min constraint by\n");
	find = 0;

	for (i = 0; i < exynos_dm->domain_count; i++) {
		if (!exynos_dm->dm_data[i].available)
			continue;

		constraint_list = &exynos_dm->dm_data[i].min_constraints;
		if (list_empty(constraint_list))
			continue;
		list_for_each_entry(constraint, constraint_list, driver_domain) {
			if (constraint->dm_constraint == dm->dm_type) {
				count += scnprintf(buf + count, PAGE_SIZE - count,
					"%s ---> %s\n"
					"policy_min(%u), const_min(%u) ---> const_freq(%u)\n"
					"gov_min(%u), gov_freq(%u) ---> gov_freq(%u)\n",
					exynos_dm->dm_data[i].dm_type_name,
					dm->dm_type_name,
					exynos_dm->dm_data[i].policy_min,
					exynos_dm->dm_data[i].const_min,
					constraint->const_freq,
					exynos_dm->dm_data[i].gov_min,
					exynos_dm->dm_data[i].governor_freq,
					constraint->gov_freq);
				if (constraint->guidance)
					count += scnprintf(buf + count, PAGE_SIZE - count,
							   " [guidance]\n");
				else
					count += scnprintf(buf + count, PAGE_SIZE - count, "\n");
				find = max(find, constraint->const_freq);
			}
		}
	}
	if (find == 0)
		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "There is no min constraint\n\n");
	else
		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "min constraint freq = %u\n", find);
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "-------------------------------------------------\n");
	count += scnprintf(buf + count, PAGE_SIZE - count, "max constraint by\n");
	find = INT_MAX;

	for (i = 0; i < exynos_dm->domain_count; i++) {
		if (!exynos_dm->dm_data[i].available)
			continue;

		constraint_list = &exynos_dm->dm_data[i].max_constraints;
		if (list_empty(constraint_list))
			continue;
		list_for_each_entry(constraint, constraint_list, driver_domain) {
			if (constraint->dm_constraint == dm->dm_type) {
				count += scnprintf(buf + count, PAGE_SIZE - count,
					"%s ---> %s\n"
					"policy_min(%u), const_min(%u) ---> const_freq(%u)\n",
					exynos_dm->dm_data[i].dm_type_name,
					dm->dm_type_name,
					exynos_dm->dm_data[i].policy_max,
					exynos_dm->dm_data[i].const_max,
					constraint->const_freq);
				if (constraint->guidance)
					count += scnprintf(buf + count, PAGE_SIZE - count,
							   " [guidance]\n");
				else
					count += scnprintf(buf + count, PAGE_SIZE - count, "\n");
				find = min(find, constraint->const_freq);
			}
		}
	}
	if (find == INT_MAX)
		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "There is no max constraint\n\n");
	else
		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "max constraint freq = %u\n", find);
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "-------------------------------------------------\n");
	return count;
}

static DEVICE_ATTR_RO(available);

static struct attribute *exynos_dm_sysfs_entries[] = {
	&dev_attr_available.attr,
	NULL,
};

static struct attribute_group exynos_dm_attr_group = {
	.name	= "exynos_dm",
	.attrs	= exynos_dm_sysfs_entries,
};

/*
 * SYSFS for Debugging end
 */

static void print_available_dm_data(struct exynos_dm_device *dm)
{
	int i;

	for (i = 0; i < dm->domain_count; i++) {
		if (!dm->dm_data[i].available)
			continue;

		dev_info(dm->dev, "dm_type: %d(%s), available = %s\n",
			 dm->dm_data[i].dm_type, dm->dm_data[i].dm_type_name,
			 dm->dm_data[i].available ? "true" : "false");
	}
}

static int exynos_dm_index_validate(int index)
{
	if (index < 0) {
		dev_err(exynos_dm->dev, "invalid dm_index (%d)\n", index);
		return -EINVAL;
	}

	return 0;
}

#ifdef CONFIG_OF
static int exynos_dm_parse_dt(struct device_node *np, struct exynos_dm_device *dm)
{
	struct device_node *child_np, *domain_np = NULL;
	const char *name;
	int ret = 0;

	if (!np)
		return -ENODEV;

	domain_np = of_get_child_by_name(np, "dm_domains");
	if (!domain_np)
		return -ENODEV;

	dm->domain_count = of_get_child_count(domain_np);
	if (!dm->domain_count)
		return -ENODEV;

	dm->dm_data = kcalloc(dm->domain_count, sizeof(*dm->dm_data), GFP_KERNEL);
	if (!dm->dm_data)
		return -ENOMEM;

	dm->domain_order = kcalloc(dm->domain_count, sizeof(int), GFP_KERNEL);
	if (!dm->domain_order)
		return -ENOMEM;

	for_each_child_of_node(domain_np, child_np) {
		int index;
		const char *available;
#if IS_ENABLED(CONFIG_GS_ACPM)
		const char *policy_use;
#endif
		if (of_property_read_u32(child_np, "dm-index", &index))
			return -ENODEV;

		ret = exynos_dm_index_validate(index);
		if (ret)
			return ret;

		if (of_property_read_string(child_np, "available", &available))
			return -ENODEV;

		if (!strcmp(available, "true")) {
			dm->dm_data[index].dm_type = index;
			dm->dm_data[index].available = true;

			if (!of_property_read_string(child_np, "dm_type_name", &name)) {
				strncpy(dm->dm_data[index].dm_type_name,
					name, EXYNOS_DM_TYPE_NAME_LEN);
			}

			INIT_LIST_HEAD(&dm->dm_data[index].min_constraints);
			INIT_LIST_HEAD(&dm->dm_data[index].max_constraints);
			INIT_LIST_HEAD(&dm->dm_data[index].min_drivers);
			INIT_LIST_HEAD(&dm->dm_data[index].max_drivers);
		} else {
			dm->dm_data[index].available = false;
		}
#if IS_ENABLED(CONFIG_GS_ACPM)
		if (of_property_read_string(child_np, "policy_use", &policy_use)) {
			dev_info(dm->dev, "This doesn't need to send policy to ACPM\n");
		} else {
			if (!strcmp(policy_use, "true"))
				dm->dm_data[index].policy_use = true;
		}

		if (of_property_read_u32(child_np, "cal_id", &dm->dm_data[index].cal_id))
			return -ENODEV;
#endif
	}

	return ret;
}
#else
static int exynos_dm_parse_dt(struct device_node *np, struct exynos_dm_device *dm)
{
	return -ENODEV;
}
#endif

/*
 * This function should be called from each DVFS drivers
 * before DVFS driver registration to DVFS framework.
 * Initialize sequence Step.1
 */
int exynos_dm_data_init(int dm_type, void *data,
			u32 min_freq, u32 max_freq, u32 cur_freq)
{
	struct exynos_dm_data *dm;
	int ret = 0;

	ret = exynos_dm_index_validate(dm_type);
	if (ret)
		return ret;

	mutex_lock(&exynos_dm->lock);

	dm = &exynos_dm->dm_data[dm_type];

	if (!dm->available) {
		dev_err(exynos_dm->dev,
			"This dm type(%d) is not available\n", dm_type);
		ret = -ENODEV;
		goto out;
	}

	dm->policy_max = max_freq;
	dm->const_max = max_freq;
	dm->const_min = min_freq;
	dm->policy_min = min_freq;
	dm->gov_min = min_freq;
	dm->cur_freq = cur_freq;
	dm->governor_freq = cur_freq;

	dm->devdata = data;

out:
	mutex_unlock(&exynos_dm->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(exynos_dm_data_init);

/*
 * Initialize sequence Step.2
 */
static void exynos_dm_topological_sort(void)
{
	int *indegree;
	int *search_queue;
	int *result_queue = exynos_dm->domain_order;
	int s_head = 0, r_head = 0, s_rear = 0;
	int poll, i;
	struct exynos_dm_constraint *t = NULL;

	indegree = kmalloc_array(exynos_dm->domain_count, sizeof(int), GFP_KERNEL);
	if (WARN_ON(!indegree))
		return;

	search_queue = kmalloc_array(exynos_dm->domain_count, sizeof(int), GFP_KERNEL);
	if (WARN_ON(!search_queue))
		goto free_indegree;

	for (i = 0; i < exynos_dm->domain_count; i++) {
		/* calculate Indegree of each domain */
		struct exynos_dm_data *dm = &exynos_dm->dm_data[i];

		if (!dm->available)
			continue;

		if (list_empty(&dm->min_constraints) && list_empty(&dm->max_constraints) &&
		    list_empty(&dm->min_drivers) && list_empty(&dm->max_drivers))
			continue;

		indegree[i] = dm->indegree;

		/*
		 * In indegree is 0, input to search_queue.
		 * It means that the domain is not constraint of any other domains.
		 */
		if (indegree[i] == 0)
			search_queue[s_head++] = i;
	}

	while (s_head != s_rear) {
		/* Pick the first item of search_queue */
		t = NULL;
		poll = search_queue[s_rear++];
		/* Push the item which has 0 indegree into result_queue */
		result_queue[r_head++] = poll;
		exynos_dm->dm_data[poll].my_order = r_head - 1;
		/* Decrease the indegree which indecated by poll item. */
		list_for_each_entry(t, &exynos_dm->dm_data[poll].min_constraints, driver_domain) {
			if (--indegree[t->dm_constraint] == 0)
				search_queue[s_head++] = t->dm_constraint;
		}
	}

	/* Size of result queue means the number of domains which has constraint */
	exynos_dm->constraint_domain_count = r_head;

	kfree(search_queue);
free_indegree:
	kfree(indegree);
}

int register_exynos_dm_constraint_table(int dm_type,
					struct exynos_dm_constraint *constraint_list)
{
	struct exynos_dm_constraint *sub_constraint_list;
	struct exynos_dm_data *driver = &exynos_dm->dm_data[dm_type];
	struct exynos_dm_data *constraint = &exynos_dm->dm_data[constraint_list->dm_constraint];
	int i, ret = 0;

	ret = exynos_dm_index_validate(dm_type);
	if (ret)
		return ret;

	if (!constraint_list) {
		dev_err(exynos_dm->dev, "constraint is not valid\n");
		return -EINVAL;
	}

	/* check member invalid */
	if (constraint_list->constraint_type < CONSTRAINT_MIN ||
	    constraint_list->constraint_type > CONSTRAINT_MAX) {
		dev_err(exynos_dm->dev, "constraint_type is invalid\n");
		return -EINVAL;
	}

	ret = exynos_dm_index_validate(constraint_list->dm_constraint);
	if (ret)
		return ret;

	if (!constraint_list->freq_table) {
		dev_err(exynos_dm->dev, "No frequency table for constraint\n");
		return -EINVAL;
	}

	mutex_lock(&exynos_dm->lock);

	strncpy(constraint_list->dm_type_name,
		exynos_dm->dm_data[constraint_list->dm_constraint].dm_type_name,
		EXYNOS_DM_TYPE_NAME_LEN);
	constraint_list->const_freq = 0;
	constraint_list->gov_freq = 0;
	constraint_list->dm_driver = dm_type;

	if (constraint_list->constraint_type == CONSTRAINT_MIN) {
		/*
		 * In domain, min/max constraints are list of constraint conditions
		 * In constraint, driver_domain means that which work as driver.
		 * All min/max constraints in domains are linked with driver_domain in constraint,
		 * and min/max drivers in domains are linked with constraint_domain in constraint.
		 */
		list_add(&constraint_list->driver_domain, &driver->min_constraints);
		list_add(&constraint_list->constraint_domain, &constraint->min_drivers);
		constraint->indegree++;
	} else if (constraint_list->constraint_type == CONSTRAINT_MAX) {
		list_add(&constraint_list->driver_domain, &driver->max_constraints);
		list_add(&constraint_list->constraint_domain, &constraint->max_drivers);
	}

	/* check guidance and sub constraint table generations */
	if (constraint_list->guidance && constraint_list->constraint_type == CONSTRAINT_MIN) {
		sub_constraint_list = kzalloc(sizeof(*sub_constraint_list), GFP_KERNEL);
		if (!sub_constraint_list) {
			ret = -ENOMEM;
			goto err_sub_const;
		}

		// Initialize member variables
		sub_constraint_list->dm_driver = constraint_list->dm_constraint;
		sub_constraint_list->dm_constraint = constraint_list->dm_driver;
		sub_constraint_list->table_length = constraint_list->table_length;
		sub_constraint_list->constraint_type = CONSTRAINT_MAX;
		sub_constraint_list->guidance = true;
		sub_constraint_list->const_freq = UINT_MAX;
		sub_constraint_list->gov_freq = UINT_MAX;

		strncpy(sub_constraint_list->dm_type_name,
			exynos_dm->dm_data[sub_constraint_list->dm_constraint].dm_type_name,
			EXYNOS_DM_TYPE_NAME_LEN);

		sub_constraint_list->freq_table =
			kcalloc(sub_constraint_list->table_length,
				sizeof(struct exynos_dm_freq), GFP_KERNEL);
		if (!sub_constraint_list->freq_table) {
			ret = -ENOMEM;
			goto err_freq_table;
		}

		/* generation table */
		for (i = 0; i < constraint_list->table_length; i++) {
			sub_constraint_list->freq_table[i].driver_freq =
					constraint_list->freq_table[i].constraint_freq;
			sub_constraint_list->freq_table[i].constraint_freq =
					constraint_list->freq_table[i].driver_freq;
		}

		list_add(&sub_constraint_list->driver_domain, &constraint->max_constraints);
		list_add(&sub_constraint_list->constraint_domain, &driver->max_drivers);

		/* linked sub constraint */
		constraint_list->sub_constraint = sub_constraint_list;
	}

	exynos_dm_topological_sort();

	mutex_unlock(&exynos_dm->lock);

	return 0;

err_freq_table:
	kfree(sub_constraint_list);
err_sub_const:
	list_del(&constraint_list->driver_domain);
	list_del(&constraint_list->constraint_domain);

	mutex_unlock(&exynos_dm->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(register_exynos_dm_constraint_table);

int unregister_exynos_dm_constraint_table(int dm_type,
					  struct exynos_dm_constraint *constraint_list)
{
	struct exynos_dm_constraint *sub_constraint_list;
	int ret = 0;

	ret = exynos_dm_index_validate(dm_type);
	if (ret)
		return ret;

	if (!constraint_list) {
		dev_err(exynos_dm->dev, "constraint is not valid\n");
		return -EINVAL;
	}

	mutex_lock(&exynos_dm->lock);

	if (constraint_list->sub_constraint) {
		sub_constraint_list = constraint_list->sub_constraint;
		list_del(&sub_constraint_list->driver_domain);
		list_del(&sub_constraint_list->constraint_domain);
		kfree(sub_constraint_list->freq_table);
		kfree(sub_constraint_list);
	}

	list_del(&constraint_list->driver_domain);
	list_del(&constraint_list->constraint_domain);

	mutex_unlock(&exynos_dm->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(unregister_exynos_dm_constraint_table);

/*
 * This function should be called from each DVFS driver registration function
 * before return to corresponding DVFS drvier.
 * Initialize sequence Step.3
 */
int register_exynos_dm_freq_scaler(int dm_type,
				   int (*scaler_func)(int dm_type, void *devdata,
						      u32 target_freq, unsigned int relation))
{
	int ret = 0;

	ret = exynos_dm_index_validate(dm_type);
	if (ret)
		return ret;

	if (!scaler_func) {
		dev_err(exynos_dm->dev, "function is not valid\n");
		return -EINVAL;
	}

	mutex_lock(&exynos_dm->lock);

	if (!exynos_dm->dm_data[dm_type].available) {
		dev_err(exynos_dm->dev,
			"This dm type(%d) is not available\n", dm_type);
		ret = -ENODEV;
		goto out;
	}

	if (!exynos_dm->dm_data[dm_type].freq_scaler)
		exynos_dm->dm_data[dm_type].freq_scaler = scaler_func;

out:
	mutex_unlock(&exynos_dm->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(register_exynos_dm_freq_scaler);

int unregister_exynos_dm_freq_scaler(int dm_type)
{
	int ret = 0;

	ret = exynos_dm_index_validate(dm_type);
	if (ret)
		return ret;

	mutex_lock(&exynos_dm->lock);

	if (!exynos_dm->dm_data[dm_type].available) {
		dev_err(exynos_dm->dev,
			"This dm type(%d) is not available\n", dm_type);
		ret = -ENODEV;
		goto out;
	}

	if (exynos_dm->dm_data[dm_type].freq_scaler)
		exynos_dm->dm_data[dm_type].freq_scaler = NULL;

out:
	mutex_unlock(&exynos_dm->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(unregister_exynos_dm_freq_scaler);

/*
 * Policy Updater
 *
 * @dm_type: DVFS domain type for updating policy
 * @min_freq: Minimum frequency decided by policy
 * @max_freq: Maximum frequency decided by policy
 *
 * In this function, policy_min_freq and policy_max_freq will be changed.
 * After that, DVFS Manager will decide min/max freq. of current domain
 * and check dependent domains whether update is necessary.
 */

/* DM Algorithm */
static int update_constraint_min(struct exynos_dm_constraint *constraint, u32 driver_min)
{
	struct exynos_dm_data *dm = &exynos_dm->dm_data[constraint->dm_constraint];
	struct exynos_dm_freq *const_table = constraint->freq_table;
	struct exynos_dm_constraint *t;
	int i;

	/* Find constraint condition for min relationship */
	for (i = constraint->table_length - 1; i >= 0; i--) {
		if (const_table[i].driver_freq >= driver_min)
			break;
	}

	/* If i is lesser than 0, there is no constraint condition. */
	if (i < 0)
		i = 0;

	constraint->const_freq = const_table[i].constraint_freq;
	dm->const_min = 0;

	/* Find min constraint frequency from driver domains */
	list_for_each_entry(t, &dm->min_drivers, constraint_domain) {
		dm->const_min = max(t->const_freq, dm->const_min);
	}

	return 0;
}

static int update_constraint_max(struct exynos_dm_constraint *constraint, u32 driver_max)
{
	struct exynos_dm_data *dm = &exynos_dm->dm_data[constraint->dm_constraint];
	struct exynos_dm_freq *const_table = constraint->freq_table;
	struct exynos_dm_constraint *t;
	int i;

	/* Find constraint condition for max relationship */
	for (i = 0; i < constraint->table_length; i++) {
		if (const_table[i].driver_freq <= driver_max)
			break;
	}

	/* If not found in the table, assuming lowest constraint condition. */
	if (i == constraint->table_length)
		i = constraint->table_length - 1;

	constraint->const_freq = const_table[i].constraint_freq;
	dm->const_max = UINT_MAX;

	/* Find max constraint frequency from driver domains */
	list_for_each_entry(t, &dm->max_drivers, constraint_domain) {
		dm->const_max = min(t->const_freq, dm->const_max);
	}

	return 0;
}

int policy_update_call_to_DM(int dm_type, u32 min_freq, u32 max_freq)
{
	struct exynos_dm_data *dm;
	u64 pre, before, after;
#if IS_ENABLED(CONFIG_GS_ACPM) && !IS_ENABLED(CONFIG_SOC_ZUMA)
	struct ipc_config config;
	unsigned int cmd[4];
	int size, ch_num;
#endif
	s32 time = 0, pre_time = 0;
	struct exynos_dm_data *domain;
	u32 prev_min, prev_max, new_min, new_max;
	int ret = 0, i;
	struct exynos_dm_constraint *t;

#if IS_ENABLED(CONFIG_DEBUG_SNAPSHOT)
	dbg_snapshot_dm((int)dm_type, min_freq, max_freq, pre_time, time);
#endif

	pre = sched_clock();
	mutex_lock(&exynos_dm->lock);
	before = sched_clock();

	dm = &exynos_dm->dm_data[dm_type];

	/* Return if there has no min/max freq update */
	if (max_freq == 0 && min_freq == 0) {
		ret = -EINVAL;
		goto out;
	}

	prev_min = max(dm->policy_min, dm->const_min);
	prev_max = min(dm->policy_max, dm->const_max);

	if (dm->policy_max == max_freq && dm->policy_min == min_freq)
		goto out;

	if (max_freq == 0)
		max_freq = dm->policy_max;

	if (min_freq == 0)
		min_freq = dm->policy_min;

	dm->policy_max = max_freq;
	dm->policy_min = min_freq;

#if IS_ENABLED(CONFIG_GS_ACPM) && !IS_ENABLED(CONFIG_SOC_ZUMA)
       /* Send policy to FVP */
	if (dm->policy_use) {
		ret = acpm_ipc_request_channel(exynos_dm->dev->of_node, NULL, &ch_num, &size);
		if (ret) {
			dev_err(exynos_dm->dev,
				"acpm request channel is failed, id:%u, size:%u\n", ch_num, size);
			goto out;
		}
		config.cmd = cmd;
		config.response = true;
		config.cmd[0] = GET_IDX(dm->cal_id);
		config.cmd[1] = max_freq;
		config.cmd[2] = POLICY_REQ;

		ret = acpm_ipc_send_data(ch_num, &config);
		if (ret) {
			dev_err(exynos_dm->dev, "Failed to send policy data to FVP");
			goto out;
		}
	}
#endif

	if (list_empty(&dm->min_constraints) && list_empty(&dm->max_constraints))
		goto out;

	new_min = max(dm->policy_min, dm->const_min);
	new_max = min(dm->policy_max, dm->const_max);
	new_min = min(new_min, new_max);

	if (new_min != prev_min) {
		int min_freq, max_freq;

		for (i = dm->my_order; i < exynos_dm->constraint_domain_count; i++) {
			domain = &exynos_dm->dm_data[exynos_dm->domain_order[i]];
			min_freq = max(domain->policy_min, domain->const_min);
			max_freq = min(domain->policy_max, domain->const_max);
			min_freq = min(min_freq, max_freq);
			list_for_each_entry(t, &domain->min_constraints, driver_domain) {
				update_constraint_min(t, min_freq);
			}
		}
	}

	if (new_max != prev_max) {
		int max_freq;

		for (i = dm->my_order; i >= 0; i--) {
			domain = &exynos_dm->dm_data[exynos_dm->domain_order[i]];
			max_freq = min(domain->policy_max, domain->const_max);
			list_for_each_entry(t, &domain->max_constraints, driver_domain) {
				update_constraint_max(t, max_freq);
			}
		}
	}
out:
	after = sched_clock();
	mutex_unlock(&exynos_dm->lock);

	pre_time = (unsigned int)(before - pre);
	time = (unsigned int)(after - before);

#if IS_ENABLED(CONFIG_DEBUG_SNAPSHOT)
	dbg_snapshot_dm((int)dm_type, min_freq, max_freq, pre_time, time);
#endif

	return 0;
}
EXPORT_SYMBOL_GPL(policy_update_call_to_DM);

static void update_new_target(int dm_type)
{
	u32 min_freq, max_freq;
	struct exynos_dm_data *dm = &exynos_dm->dm_data[dm_type];

	if (dm->policy_max > dm->const_max)
		max_freq = dm->policy_max;

	min_freq = max3(dm->policy_min, dm->const_min, dm->gov_min);
	min_freq = max(min_freq, dm->governor_freq);
	max_freq = min(dm->policy_max, dm->const_max);
	exynos_dm->dm_data[dm_type].next_target_freq = min(min_freq, max_freq);
}

static int update_gov_min(struct exynos_dm_constraint *constraint, u32 driver_freq)
{
	struct exynos_dm_data *dm = &exynos_dm->dm_data[constraint->dm_constraint];
	struct exynos_dm_freq *const_table = constraint->freq_table;
	struct exynos_dm_constraint *t;
	int i;

	/* Find constraint condition for min relationship */
	for (i = constraint->table_length - 1; i >= 0; i--) {
		if (const_table[i].driver_freq >= driver_freq)
			break;
	}

	/* If i is lesser than 0, there is no constraint condition. */
	if (i < 0)
		i = 0;

	constraint->gov_freq = const_table[i].constraint_freq;
	dm->gov_min = 0;

	/* Find gov_min frequency from driver domains */
	list_for_each_entry(t, &dm->min_drivers, constraint_domain) {
		dm->gov_min = max(t->gov_freq, dm->gov_min);
	}

	return 0;
}

/*
 * DM CALL
 */
int DM_CALL(int dm_type, unsigned long *target_freq)
{
	struct exynos_dm_data *target_dm;
	struct exynos_dm_data *dm;
	struct exynos_dm_constraint *t;
	u32 max_freq, min_freq;
	int i, ret = 0;
	unsigned int relation = EXYNOS_DM_RELATION_L;
	u64 pre, before, after;
	s32 time = 0, pre_time = 0;

	ATRACE_BEGIN(__func__);
#if IS_ENABLED(CONFIG_DEBUG_SNAPSHOT)
	dbg_snapshot_dm((int)dm_type, *target_freq, 1, pre_time, time);
#endif

	pre = sched_clock();
	mutex_lock(&exynos_dm->lock);
	before = sched_clock();

	target_dm = &exynos_dm->dm_data[dm_type];

	target_dm->governor_freq = *target_freq;
	// trace governor voted freq
	__ATRACE_INT_PID(1, target_dm->dm_type_name, target_dm->governor_freq);

	/* Determine min/max frequency */
	max_freq = min(target_dm->const_max, target_dm->policy_max);
	min_freq = max(target_dm->const_min, target_dm->policy_min);

	if (*target_freq < min_freq)
		*target_freq = min_freq;

	if (*target_freq >= max_freq) {
		*target_freq = max_freq;
		relation = EXYNOS_DM_RELATION_H;
	}

	if (target_dm->cur_freq == *target_freq)
		goto out;

	if (list_empty(&target_dm->max_constraints) && list_empty(&target_dm->min_constraints) &&
	    target_dm->freq_scaler) {
		update_new_target(target_dm->dm_type);
		ret = target_dm->freq_scaler(target_dm->dm_type, target_dm->devdata,
					     target_dm->next_target_freq, relation);
		if (!ret)
			target_dm->cur_freq = target_dm->next_target_freq;
		goto out;
	}

	/* Propagate the influence of new terget frequencies */
	for (i = 0; i < exynos_dm->constraint_domain_count; i++) {
		dm = &exynos_dm->dm_data[exynos_dm->domain_order[i]];

		/* Update new target frequency from all min, max restricts. */
		update_new_target(dm->dm_type);

		/* Travers all constraint domains to update the gov_min value. */
		list_for_each_entry(t, &dm->min_constraints, driver_domain) {
			update_gov_min(t, dm->next_target_freq);
		}

		/* Perform frequency down scaling */
		if (dm->cur_freq > dm->next_target_freq && dm->freq_scaler) {
			ret = dm->freq_scaler(dm->dm_type, dm->devdata,
					      dm->next_target_freq, relation);
			if (!ret)
				dm->cur_freq = dm->next_target_freq;
		}
	}

	/* Perform frequency up scaling */
	for (i = exynos_dm->constraint_domain_count - 1; i >= 0; i--) {
		dm = &exynos_dm->dm_data[exynos_dm->domain_order[i]];
		if (dm->cur_freq < dm->next_target_freq && dm->freq_scaler) {
			ret = dm->freq_scaler(dm->dm_type, dm->devdata,
					      dm->next_target_freq, relation);
			if (!ret)
				dm->cur_freq = dm->next_target_freq;
		}
	}

out:
	after = sched_clock();
	mutex_unlock(&exynos_dm->lock);

	pre_time = (unsigned int)(before - pre);
	time = (unsigned int)(after - before);

	*target_freq = target_dm->cur_freq;

#if IS_ENABLED(CONFIG_DEBUG_SNAPSHOT)
	dbg_snapshot_dm((int)dm_type, *target_freq, 3, pre_time, time);
#endif
	ATRACE_END();
	return ret;
}
EXPORT_SYMBOL_GPL(DM_CALL);

static int exynos_dm_suspend(struct device *dev)
{
	/* Suspend callback function might be registered if necessary */

	return 0;
}

static int exynos_dm_resume(struct device *dev)
{
	/* Resume callback function might be registered if necessary */

	return 0;
}

static int exynos_dm_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct exynos_dm_device *dm;
	int i;

	dm = kzalloc(sizeof(*dm), GFP_KERNEL);
	if (!dm) {
		ret = -ENOMEM;
		goto err_device;
	}

	dm->dev = &pdev->dev;

	mutex_init(&dm->lock);

	/* parsing devfreq dts data for exynos-dvfs-manager */
	ret = exynos_dm_parse_dt(dm->dev->of_node, dm);
	if (ret) {
		dev_err(dm->dev, "failed to parse private data\n");
		goto err_parse_dt;
	}

	print_available_dm_data(dm);

	ret = sysfs_create_group(&dm->dev->kobj, &exynos_dm_attr_group);
	if (ret)
		dev_warn(dm->dev, "failed create sysfs for DVFS Manager\n");

	for (i = 0; i < dm->domain_count; i++) {
		if (!dm->dm_data[i].available)
			continue;

		snprintf(dm->dm_data[i].dm_policy_attr.name, EXYNOS_DM_ATTR_NAME_LEN,
			 "dm_policy_%s", dm->dm_data[i].dm_type_name);
		sysfs_attr_init(&dm->dm_data[i].dm_policy_attr.attr.attr);
		dm->dm_data[i].dm_policy_attr.attr.attr.name =
			dm->dm_data[i].dm_policy_attr.name;
		dm->dm_data[i].dm_policy_attr.attr.attr.mode = 0440;
		dm->dm_data[i].dm_policy_attr.attr.show = show_dm_policy;

		ret = sysfs_add_file_to_group(&dm->dev->kobj,
					      &dm->dm_data[i].dm_policy_attr.attr.attr,
					      exynos_dm_attr_group.name);
		if (ret)
			dev_warn(dm->dev, "failed create sysfs for DM policy %s\n",
				 dm->dm_data[i].dm_type_name);

		snprintf(dm->dm_data[i].constraint_table_attr.name, EXYNOS_DM_ATTR_NAME_LEN,
			 "constaint_table_%s", dm->dm_data[i].dm_type_name);
		sysfs_attr_init(&dm->dm_data[i].constraint_table_attr.attr.attr);
		dm->dm_data[i].constraint_table_attr.attr.attr.name =
			dm->dm_data[i].constraint_table_attr.name;
		dm->dm_data[i].constraint_table_attr.attr.attr.mode = 0440;
		dm->dm_data[i].constraint_table_attr.attr.show = show_constraint_table;

		ret = sysfs_add_file_to_group(&dm->dev->kobj,
					      &dm->dm_data[i].constraint_table_attr.attr.attr,
					      exynos_dm_attr_group.name);
		if (ret)
			dev_warn(dm->dev, "failed create sysfs for constraint_table %s\n",
				 dm->dm_data[i].dm_type_name);
	}

	exynos_dm = dm;
	platform_set_drvdata(pdev, dm);

	return 0;

err_parse_dt:
	mutex_destroy(&dm->lock);
	kfree(dm);
err_device:

	return ret;
}

static int exynos_dm_remove(struct platform_device *pdev)
{
	struct exynos_dm_device *dm = platform_get_drvdata(pdev);

	sysfs_remove_group(&dm->dev->kobj, &exynos_dm_attr_group);
	mutex_destroy(&dm->lock);
	kfree(dm);

	return 0;
}

static struct platform_device_id exynos_dm_driver_ids[] = {
	{
		.name		= EXYNOS_DM_MODULE_NAME,
	},
	{},
};
MODULE_DEVICE_TABLE(platform, exynos_dm_driver_ids);

static const struct of_device_id exynos_dm_match[] = {
	{
		.compatible	= "samsung,exynos-dvfs-manager",
	},
	{},
};
MODULE_DEVICE_TABLE(of, exynos_dm_match);

static const struct dev_pm_ops exynos_dm_pm_ops = {
	.suspend	= exynos_dm_suspend,
	.resume		= exynos_dm_resume,
};

static struct platform_driver exynos_dm_driver = {
	.probe		= exynos_dm_probe,
	.remove		= exynos_dm_remove,
	.id_table	= exynos_dm_driver_ids,
	.driver	= {
		.name	= EXYNOS_DM_MODULE_NAME,
		.owner	= THIS_MODULE,
		.pm	= &exynos_dm_pm_ops,
		.of_match_table = exynos_dm_match,
	},
};

static int exynos_dm_init(void)
{
	return platform_driver_register(&exynos_dm_driver);
}
subsys_initcall(exynos_dm_init);

static void __exit exynos_dm_exit(void)
{
	platform_driver_unregister(&exynos_dm_driver);
}
module_exit(exynos_dm_exit);

MODULE_DESCRIPTION("Exynos DVFS Manager driver");
MODULE_LICENSE("GPL");
