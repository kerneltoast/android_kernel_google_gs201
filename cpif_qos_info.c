// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019-2020, Samsung Electronics.
 *
 */
#include <linux/slab.h>

#include "cpif_qos_info.h"
#include "modem_v1.h"

static struct hiprio_uid_list g_hiprio_uid_list;

struct hiprio_uid_list *cpif_qos_get_list(void)
{
	return &g_hiprio_uid_list;
}

struct hiprio_uid *cpif_qos_get_node(u32 uid)
{
	struct hiprio_uid *node;

	hash_for_each_possible(g_hiprio_uid_list.uid_map, node, h_node, uid) {
		if (node->uid == uid)
			return node;
	}

	return NULL;
}

bool cpif_qos_add_uid(u32 uid)
{
	struct hiprio_uid *new_uid;

	if (cpif_qos_get_node(uid)) {
		mif_err("---- uid(%d) already exists in the list\n", uid);
		return false;
	}
	new_uid = kzalloc(sizeof(struct hiprio_uid), GFP_ATOMIC);
	if (!new_uid)
		return false;

	new_uid->uid = uid;
	hash_add(g_hiprio_uid_list.uid_map, &new_uid->h_node, uid);

	return true;
}

bool cpif_qos_remove_uid(u32 uid)
{
	struct hiprio_uid *node = cpif_qos_get_node(uid);

	if (!node) {
		mif_err("---- uid(%d) does not exist in the list\n", uid);
		return false;
	}
	hash_del(&node->h_node);
	kfree(node);

	return true;
}

/* sysfs */
static ssize_t hiprio_uid_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	struct hiprio_uid_list *hiprio_list;
	struct hiprio_uid *node;
	ssize_t count = 0;
	int i = 0;

	hiprio_list = cpif_qos_get_list();
	if (!hiprio_list) {
		mif_err("-- hiprio uid list does not exist\n");
		return -EINVAL;
	}

	if (hash_empty(hiprio_list->uid_map)) {
		mif_err("-- there is no hiprio uid\n");
		return count;
	}

	mif_info("-- uid list --\n");
	hash_for_each(hiprio_list->uid_map, i, node, h_node) {
		count += scnprintf(buf + count, PAGE_SIZE - count, "%d\n", node->uid);
		mif_info("index %d: %d\n", i, node->uid);
	}

	return count;
}

static ssize_t hiprio_uid_store(struct kobject *kobj,
		struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	long uid = 0;

	mif_info("cpif_qos command input:  %s\n", buf);

	if (strstr(buf, "add")) {
		if (kstrtol(buf + 4, 10, &uid)) {
			mif_err("-- failed to parse uid\n");
			return -EINVAL;
		}
		mif_info("-- user requires addition of uid: %ld\n", uid);
		if (!cpif_qos_add_uid((u32)uid)) {
			mif_err("-- Adding uid %ld to hiprio list failed\n", uid);
			return -EINVAL;
		}
	} else if (strstr(buf, "rm")) {
		if (kstrtol(buf + 3, 10, &uid)) {
			mif_err("-- failed to parse uid\n");
			return -EINVAL;
		}
		mif_info("-- user requires removal of uid: %ld\n", uid);
		if (!cpif_qos_remove_uid((u32)uid)) {
			mif_err("-- Removing uid %ld from hiprio list failed\n", uid);
			return -EINVAL;
		}
	} else {
		mif_err("-- command not valid\n");
		return -EINVAL;
	}

	return count;
}

static struct kobject *cpif_qos_kobject;
static struct kobj_attribute hiprio_uid_attribute = {
	.attr = {.name = "hiprio_uid", .mode = 0660},
	.show = hiprio_uid_show,
	.store = hiprio_uid_store,
};
static struct attribute *cpif_qos_attrs[] = {
	&hiprio_uid_attribute.attr,
	NULL,
};
ATTRIBUTE_GROUPS(cpif_qos);

/* Init */
int cpif_qos_init_list(void)
{
	cpif_qos_kobject = kobject_create_and_add("cpif_qos", kernel_kobj);
	if (!cpif_qos_kobject) {
		mif_err("kobject_create_and_add() error\n");
		return -EINVAL;
	}

	if (sysfs_create_groups(cpif_qos_kobject, cpif_qos_groups)) {
		mif_err("sysfs_create_groups() error\n");
		return -EINVAL;
	}

	hash_init(g_hiprio_uid_list.uid_map);

	return 0;
}
