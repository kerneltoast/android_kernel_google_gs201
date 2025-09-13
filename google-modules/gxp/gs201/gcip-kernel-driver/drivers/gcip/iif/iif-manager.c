// SPDX-License-Identifier: GPL-2.0-only
/*
 * The manager of inter-IP fences.
 *
 * It manages the pool of fence IDs. The IIF driver device will initialize a manager and each IP
 * driver will fetch the manager from the IIF device.
 *
 * Copyright (C) 2023-2024 Google LLC
 */

#define pr_fmt(fmt) "iif: " fmt

#include <linux/container_of.h>
#include <linux/export.h>
#include <linux/idr.h>
#include <linux/kref.h>
#include <linux/of.h>
#include <linux/rwsem.h>
#include <linux/slab.h>

#include <iif/iif-fence-table.h>
#include <iif/iif-fence.h>
#include <iif/iif-manager.h>
#include <iif/iif-shared.h>

static void iif_manager_destroy(struct kref *kref)
{
	struct iif_manager *mgr = container_of(kref, struct iif_manager, kref);

	ida_destroy(&mgr->idp);
	kfree(mgr);
}

struct iif_manager *iif_manager_init(const struct device_node *np)
{
	struct iif_manager *mgr;
	int ret;

	mgr = kzalloc(sizeof(*mgr), GFP_KERNEL);
	if (!mgr)
		return ERR_PTR(-ENOMEM);

	ret = iif_fence_table_init(np, &mgr->fence_table);
	if (ret) {
		kfree(mgr);
		return ERR_PTR(ret);
	}

	kref_init(&mgr->kref);
	ida_init(&mgr->idp);
	init_rwsem(&mgr->ops_sema);

	return mgr;
}

struct iif_manager *iif_manager_get(struct iif_manager *mgr)
{
	kref_get(&mgr->kref);
	return mgr;
}

void iif_manager_put(struct iif_manager *mgr)
{
	kref_put(&mgr->kref, iif_manager_destroy);
}

int iif_manager_register_ops(struct iif_manager *mgr, enum iif_ip_type ip,
			     const struct iif_manager_ops *ops, void *data)
{
	if (!ops || !ops->fence_unblocked || ip >= IIF_IP_NUM)
		return -EINVAL;

	down_write(&mgr->ops_sema);

	mgr->ops[ip] = ops;
	mgr->data[ip] = data;

	up_write(&mgr->ops_sema);

	return 0;
}

void iif_manager_unregister_ops(struct iif_manager *mgr, enum iif_ip_type ip)
{
	if (ip >= IIF_IP_NUM)
		return;

	down_write(&mgr->ops_sema);

	mgr->ops[ip] = NULL;
	mgr->data[ip] = NULL;

	up_write(&mgr->ops_sema);
}

int iif_manager_acquire_block_wakelock(struct iif_manager *mgr, enum iif_ip_type ip)
{
	int ret = 0;

	if (ip >= IIF_IP_NUM)
		return -EINVAL;

	down_read(&mgr->ops_sema);

	if (mgr->ops[ip] && mgr->ops[ip]->acquire_block_wakelock)
		ret = mgr->ops[ip]->acquire_block_wakelock(mgr->data[ip]);

	up_read(&mgr->ops_sema);

	return ret;
}

void iif_manager_release_block_wakelock(struct iif_manager *mgr, enum iif_ip_type ip)
{
	if (ip >= IIF_IP_NUM)
		return;

	down_read(&mgr->ops_sema);

	if (mgr->ops[ip] && mgr->ops[ip]->release_block_wakelock)
		mgr->ops[ip]->release_block_wakelock(mgr->data[ip]);

	up_read(&mgr->ops_sema);
}

void iif_manager_broadcast_fence_unblocked(struct iif_manager *mgr, struct iif_fence *fence)
{
	enum iif_ip_type ip;
	unsigned int tmp;

	down_read(&mgr->ops_sema);

	for_each_waiting_ip(&mgr->fence_table, fence->id, ip, tmp) {
		if (!mgr->ops[ip] || !mgr->ops[ip]->fence_unblocked) {
			pr_warn("IP driver hasn't registered fence_unblocked, ip=%d", ip);
			continue;
		}
		mgr->ops[ip]->fence_unblocked(fence, mgr->data[ip]);
	}

	up_read(&mgr->ops_sema);
}
