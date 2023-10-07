// SPDX-License-Identifier: GPL-2.0
/*
 * Implements methods common to the family of EdgeTPUs for mobile devices to retrieve host side
 * debug dump segments and report them to SSCD.
 *
 * Copyright (C) 2021-2022 Google LLC
 */

#include <linux/atomic.h>
#include <linux/bits.h>
#include <linux/mutex.h>
#include <linux/platform_data/sscoredump.h>
#include <linux/platform_device.h>
#include <linux/rbtree.h>
#include <linux/slab.h>

#include "edgetpu-config.h"
#include "edgetpu-device-group.h"
#include "edgetpu-dump-info.h"
#include "edgetpu-internal.h"
#include "edgetpu-mailbox.h"
#include "edgetpu-mapping.h"
#include "edgetpu-mobile-platform.h"
#include "edgetpu-wakelock.h"
#include "mobile-debug-dump.h"

#include "edgetpu-debug-dump.c"

#define SET_FIELD(info, obj, __field) ((info)->__field = (obj)->__field)

/* Helper structure to hold the segments to be reported to SSCD. */
struct sscd_segments_context {
	size_t n_segs; /* current number of recorded segments */
	size_t capacity; /* number of segments allocated */
	struct sscd_segment *segs;
	/*
	 * Array with the same length as @segs, indicates whether segs[i].addr should be freed on
	 * context releasing.
	 */
	bool *free_on_release;
	struct mobile_sscd_info *sscd_info;
};

static int sscd_ctx_init(struct sscd_segments_context *ctx, struct mobile_sscd_info *sscd_info)
{
	struct sscd_platform_data *pdata = sscd_info->pdata;

	if (!pdata->sscd_report)
		return -ENOENT;
	ctx->n_segs = 0;
	ctx->capacity = 0;
	ctx->segs = NULL;
	ctx->free_on_release = NULL;
	ctx->sscd_info = sscd_info;
	return 0;
}

static void sscd_ctx_release(struct sscd_segments_context *ctx)
{
	int i;

	for (i = 0; i < ctx->n_segs; i++)
		if (ctx->free_on_release[i])
			kfree(ctx->segs[i].addr);
	kfree(ctx->segs);
	kfree(ctx->free_on_release);
}

/*
 * Pushes the segment.
 *
 * If @free_on_release is true, kfree(@seg->addr) is called when releasing @ctx.
 *
 * Returns 0 on success.
 */
static int sscd_ctx_push_segment(struct sscd_segments_context *ctx, struct sscd_segment *seg,
				 bool free_on_release)
{
	void *ptr1, *ptr2;
	size_t new_cap;

	if (ctx->n_segs >= ctx->capacity) {
		new_cap = ctx->capacity << 1;
		if (!new_cap)
			new_cap = 1;
		ptr1 = krealloc(ctx->segs, new_cap * sizeof(*ctx->segs), GFP_KERNEL);
		if (!ptr1)
			return -ENOMEM;
		ptr2 = krealloc(ctx->free_on_release, new_cap * sizeof(*ctx->free_on_release),
				GFP_KERNEL);
		if (!ptr2) {
			kfree(ptr1);
			return -ENOMEM;
		}
		ctx->segs = ptr1;
		ctx->free_on_release = ptr2;
		ctx->capacity = new_cap;
	}

	ctx->segs[ctx->n_segs] = *seg;
	ctx->free_on_release[ctx->n_segs] = free_on_release;
	ctx->n_segs++;
	return 0;
}

/*
 * Passes dump data to SSCD daemon and releases @ctx.
 *
 * Returns what sscd_report returned. Note that @ctx is always released no matter what is returned.
 */
static int sscd_ctx_report_and_release(struct sscd_segments_context *ctx, const char *crash_info)
{
	struct sscd_platform_data *pdata = ctx->sscd_info->pdata;
	struct platform_device *sscd_dev = ctx->sscd_info->dev;
	int ret;

	ret = pdata->sscd_report(sscd_dev, ctx->segs, ctx->n_segs, SSCD_FLAGS_ELFARM64HDR,
				 crash_info);
	sscd_ctx_release(ctx);
	return ret;
}

static void sscd_release(struct device *dev)
{
	pr_debug(DRIVER_NAME " release\n");
}

static struct sscd_platform_data sscd_pdata;
static struct platform_device sscd_dev;

static int mobile_sscd_collect_mappings_info(struct edgetpu_mapping_root *root, u32 workload_id,
					     u8 type, struct sscd_segments_context *ctx)
{
	int ret = 0;
	struct edgetpu_dump_segment *seg_hdr;
	struct edgetpu_mapping_info_header *hdr;
	struct edgetpu_mapping_info *info;
	size_t seg_size;
	void *buffer = NULL;
	struct rb_node *node;

	mutex_lock(&root->lock);

	if (!root->count)
		goto out_unlock;
	seg_size = sizeof(*seg_hdr) + sizeof(*hdr) + sizeof(*info) * root->count;
	buffer = kzalloc(seg_size, GFP_KERNEL);
	if (!buffer) {
		ret = -ENOMEM;
		goto out_unlock;
	}

	seg_hdr = buffer;
	seg_hdr->type = BIT_ULL(DUMP_TYPE_KERNEL_MAPPINGS_BIT);
	seg_hdr->size = seg_size - sizeof(*seg_hdr);
	hdr = (typeof(hdr))(seg_hdr + 1);
	hdr->n_mappings = root->count;
	hdr->group_workload_id = workload_id;
	hdr->mapping_type = type;
	info = hdr->mappings;
	for (node = rb_first(&root->rb); node; node = rb_next(node)) {
		struct edgetpu_mapping *map = container_of(node, struct edgetpu_mapping, node);

		SET_FIELD(info, map, host_address);
		SET_FIELD(info, map, device_address);
		SET_FIELD(info, map, flags);
		SET_FIELD(info, map, dir);
		info->size = (u64)map->map_size;
		info++;
	}

out_unlock:
	mutex_unlock(&root->lock);
	if (buffer) {
		struct sscd_segment seg = {
			.addr = buffer,
			.size = seg_size,
		};

		ret = sscd_ctx_push_segment(ctx, &seg, true);
		if (ret)
			kfree(buffer);
	}
	return ret;
}

/*
 * For each group, collects the mappings information include host mapping and dmabuf mapping buffers
 * and records to @ctx.
 *
 * Returns a negative errno in case of failure.
 */
static int mobile_sscd_collect_group_mappings_info(struct edgetpu_device_group **groups,
						   size_t num_groups,
						   struct sscd_segments_context *ctx)
{
	int i, ret;
	struct edgetpu_device_group *group;

	for (i = 0; i < num_groups; i++) {
		group = groups[i];
		ret = mobile_sscd_collect_mappings_info(&group->host_mappings, group->workload_id,
							MAPPING_TYPE_HOST, ctx);
		if (ret)
			return ret;
		ret = mobile_sscd_collect_mappings_info(&group->dmabuf_mappings, group->workload_id,
							MAPPING_TYPE_DMABUF, ctx);
		if (ret)
			return ret;
	}
	return 0;
}

static int mobile_sscd_collect_etdev_info(struct edgetpu_dev *etdev, struct sscd_segments_context *ctx)
{
	struct edgetpu_dump_segment *seg_hdr;
	struct edgetpu_dev_info *info;
	const size_t seg_size = sizeof(*seg_hdr) + sizeof(*info);
	void *buffer;
	struct sscd_segment seg = {
		.size = seg_size,
	};

	buffer = kzalloc(seg_size, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;
	seg.addr = buffer;
	seg_hdr = buffer;
	seg_hdr->type = BIT_ULL(DUMP_TYPE_KERNEL_ETDEV_BIT);
	seg_hdr->size = seg_size - sizeof(*seg_hdr);
	info = (typeof(info))(seg_hdr + 1);
	SET_FIELD(info, etdev, state);
	SET_FIELD(info, etdev, vcid_pool);
	info->job_count = atomic_read(&etdev->job_count);
	SET_FIELD(info, etdev, firmware_crash_count);
	SET_FIELD(info, etdev, watchdog_timeout_count);
	return sscd_ctx_push_segment(ctx, &seg, true);
}

static int mobile_sscd_collect_clients_info(struct edgetpu_client **clients, size_t num_clients,
					    struct sscd_segments_context *ctx)
{
	int i;
	struct edgetpu_dump_segment *seg_hdr;
	struct edgetpu_client_info_header *hdr;
	struct edgetpu_client_info *info;
	struct edgetpu_client *client;
	const size_t seg_size = sizeof(*seg_hdr) + sizeof(*hdr) + sizeof(*info) * num_clients;
	void *buffer;
	struct sscd_segment seg = {
		.size = seg_size,
	};

	if (!num_clients)
		return 0;
	buffer = kzalloc(seg_size, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;
	seg.addr = buffer;
	seg_hdr = buffer;
	seg_hdr->type = BIT_ULL(DUMP_TYPE_KERNEL_CLIENTS_BIT);
	seg_hdr->size = seg_size - sizeof(*seg_hdr);
	hdr = (typeof(hdr))(seg_hdr + 1);
	info = hdr->clients;
	for (i = 0; i < num_clients; i++) {
		client = clients[i];
		SET_FIELD(info, client, pid);
		SET_FIELD(info, client, tgid);
		SET_FIELD(info, client, perdie_events);
		info->wakelock_req_count =
			NO_WAKELOCK(client->wakelock) ? ~0u : client->wakelock->req_count;
		mutex_lock(&client->group_lock);
		info->group_workload_id = client->group ? client->group->workload_id : ~0u;
		mutex_unlock(&client->group_lock);
		info++;
	}
	hdr->n_clients = num_clients;
	return sscd_ctx_push_segment(ctx, &seg, true);
}

static int mobile_sscd_collect_groups_info(struct edgetpu_device_group **groups, size_t num_groups,
					   struct sscd_segments_context *ctx)
{
	int i;
	struct edgetpu_dump_segment *seg_hdr;
	struct edgetpu_group_info_header *hdr;
	struct edgetpu_group_info *info;
	struct edgetpu_device_group *group;
	const size_t seg_size = sizeof(*seg_hdr) + sizeof(*hdr) + sizeof(*info) * num_groups;
	void *buffer;
	struct sscd_segment seg = {
		.size = seg_size,
	};

	if (!num_groups)
		return 0;
	buffer = kzalloc(seg_size, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;
	seg.addr = buffer;
	seg_hdr = buffer;
	seg_hdr->type = BIT_ULL(DUMP_TYPE_KERNEL_GROUPS_BIT);
	seg_hdr->size = seg_size - sizeof(*seg_hdr);
	hdr = (typeof(hdr))(seg_hdr + 1);
	info = hdr->groups;
	for (i = 0; i < num_groups; i++) {
		group = groups[i];
		SET_FIELD(info, group, workload_id);
		SET_FIELD(info, group, vcid);
		SET_FIELD(info, group, status);
		SET_FIELD(info, group, context_id);
		info->size_host_mappings = edgetpu_mappings_total_size(&group->host_mappings);
		info->size_dmabuf_mappings = edgetpu_mappings_total_size(&group->dmabuf_mappings);
		mutex_lock(&group->lock);
		info->queues_attached = edgetpu_group_finalized_and_attached(group);
		mutex_unlock(&group->lock);
		info++;
	}
	hdr->n_groups = num_groups;
	return sscd_ctx_push_segment(ctx, &seg, true);
}

static struct edgetpu_client **edgetpu_get_clients(struct edgetpu_dev *etdev, size_t *p_num_clients)
{
	struct edgetpu_client **clients;
	struct edgetpu_list_device_client *lc;
	size_t num_clients = 0, i = 0;

	mutex_lock(&etdev->clients_lock);
	for_each_list_device_client(etdev, lc)
		num_clients++;
	clients = kmalloc_array(num_clients, sizeof(*clients), GFP_KERNEL);
	if (!clients) {
		mutex_unlock(&etdev->clients_lock);
		return ERR_PTR(-ENOMEM);
	}

	for_each_list_device_client(etdev, lc)
		clients[i++] = edgetpu_client_get(lc->client);
	mutex_unlock(&etdev->clients_lock);
	*p_num_clients = num_clients;
	return clients;
}

static struct edgetpu_device_group **edgetpu_get_groups(struct edgetpu_dev *etdev,
							size_t *p_num_groups)
{
	struct edgetpu_device_group **groups;
	struct edgetpu_device_group *group;
	struct edgetpu_list_group *g;
	size_t num_groups = 0;

	mutex_lock(&etdev->groups_lock);
	groups = kmalloc_array(etdev->n_groups, sizeof(*groups), GFP_KERNEL);
	if (!groups) {
		mutex_unlock(&etdev->groups_lock);
		return ERR_PTR(-ENOMEM);
	}

	etdev_for_each_group(etdev, g, group)
		groups[num_groups++] = edgetpu_device_group_get(group);
	mutex_unlock(&etdev->groups_lock);
	*p_num_groups = num_groups;
	return groups;
}

static int mobile_collect_device_info(struct edgetpu_dev *etdev, struct sscd_segments_context *ctx)
{
	struct edgetpu_device_group **groups;
	struct edgetpu_client **clients;
	size_t num_groups = 0, num_clients = 0;
	int i, ret;

	clients = edgetpu_get_clients(etdev, &num_clients);
	if (IS_ERR(clients))
		return PTR_ERR(clients);
	groups = edgetpu_get_groups(etdev, &num_groups);
	if (IS_ERR(groups)) {
		ret = PTR_ERR(groups);
		goto out_put_clients;
	}

	ret = mobile_sscd_collect_etdev_info(etdev, ctx);
	if (ret)
		goto out_put_groups;
	ret = mobile_sscd_collect_clients_info(clients, num_clients, ctx);
	if (ret)
		goto out_put_groups;
	ret = mobile_sscd_collect_groups_info(groups, num_groups, ctx);
	if (ret)
		goto out_put_groups;
	ret = mobile_sscd_collect_group_mappings_info(groups, num_groups, ctx);

out_put_groups:
	for (i = 0; i < num_groups; i++)
		edgetpu_device_group_put(groups[i]);
	kfree(groups);
out_put_clients:
	for (i = 0; i < num_clients; i++)
		edgetpu_client_put(clients[i]);
	kfree(clients);
	return ret;
}

static int mobile_sscd_generate_coredump(void *p_etdev, void *p_dump_setup)
{
	struct edgetpu_dev *etdev;
	struct edgetpu_debug_dump_setup *dump_setup;
	struct edgetpu_mobile_platform_dev *pdev;
	struct sscd_segments_context sscd_ctx;
	struct edgetpu_debug_dump *debug_dump;
	struct edgetpu_crash_reason *crash_reason;
	struct edgetpu_dump_segment *dump_seg;
	char crash_info[128];
	int i, ret;
	u64 offset;

	if (!p_etdev || !p_dump_setup)
		return -EINVAL;

	etdev = (struct edgetpu_dev *)p_etdev;
	dump_setup = (struct edgetpu_debug_dump_setup *)p_dump_setup;
	pdev = to_mobile_dev(etdev);
	ret = sscd_ctx_init(&sscd_ctx, &pdev->sscd_info);
	if (ret)
		goto err;

	debug_dump = (struct edgetpu_debug_dump *)(dump_setup + 1);

	/* Populate crash reason */
	crash_reason =
		(struct edgetpu_crash_reason *)((u8 *)dump_setup + debug_dump->crash_reason_offset);
	scnprintf(crash_info, sizeof(crash_info), "[edgetpu_coredump] error code: %#llx",
		  crash_reason->code);

	/* Populate sscd segments */
	dump_seg = (struct edgetpu_dump_segment *)((u8 *)dump_setup +
						   debug_dump->dump_segments_offset);
	offset = debug_dump->dump_segments_offset;
	for (i = 0; i < debug_dump->dump_segments_num; i++) {
		struct sscd_segment seg = {
			.addr = dump_seg,
			.size = sizeof(struct edgetpu_dump_segment) + dump_seg->size,
			.paddr = (void *)(etdev->debug_dump_mem.tpu_addr + offset),
			.vaddr = (void *)(etdev->debug_dump_mem.vaddr + offset),
		};

		ret = sscd_ctx_push_segment(&sscd_ctx, &seg, false);
		if (ret)
			goto err_release;
		offset += sizeof(struct edgetpu_dump_segment) + dump_seg->size;
		dump_seg = (struct edgetpu_dump_segment *)((u8 *)dump_setup +
							   ALIGN(offset, sizeof(uint64_t)));
	}

	ret = mobile_collect_device_info(etdev, &sscd_ctx);
	if (ret)
		goto err_release;

	ret = sscd_ctx_report_and_release(&sscd_ctx, crash_info);
	if (ret)
		goto err;

	return 0;

err_release:
	sscd_ctx_release(&sscd_ctx);
err:
	etdev_err(etdev, "failed to generate coredump: %d", ret);
	return ret;
}

int edgetpu_debug_dump_init(struct edgetpu_dev *etdev)
{
	size_t size;
	int ret;
	struct edgetpu_debug_dump_setup *dump_setup;
	struct edgetpu_mobile_platform_dev *pdev;

	pdev = to_mobile_dev(etdev);

	size = EDGETPU_DEBUG_DUMP_MEM_SIZE;

	sscd_dev = (struct platform_device) {
		.name = DRIVER_NAME,
		.driver_override = SSCD_NAME,
		.id = PLATFORM_DEVID_NONE,
		.dev = {
			.platform_data = &sscd_pdata,
			.release = sscd_release,
		},
	};
	/* Register SSCD platform device */
	ret = platform_device_register(&sscd_dev);
	if (ret) {
		etdev_err(etdev, "SSCD platform device registration failed: %d", ret);
		return ret;
	}
	/*
	 * Allocate a buffer for various dump segments
	 */
	ret = edgetpu_alloc_coherent(etdev, size, &etdev->debug_dump_mem, EDGETPU_CONTEXT_KCI);
	if (ret) {
		etdev_err(etdev, "Debug dump seg alloc failed");
		etdev->debug_dump_mem.vaddr = NULL;
		goto out_unregister_platform;
	}
	dump_setup = (struct edgetpu_debug_dump_setup *)etdev->debug_dump_mem.vaddr;
	memset(dump_setup, 0, size);
	dump_setup->dump_mem_size = size;

	/*
	 * Allocate memory for debug dump handlers
	 */
	etdev->debug_dump_handlers =
		kcalloc(DUMP_REASON_NUM, sizeof(*etdev->debug_dump_handlers), GFP_KERNEL);
	if (!etdev->debug_dump_handlers)
		return -ENOMEM;
	etdev->debug_dump_handlers[DUMP_REASON_REQ_BY_USER] = mobile_sscd_generate_coredump;
	etdev->debug_dump_handlers[DUMP_REASON_RECOVERABLE_FAULT] = mobile_sscd_generate_coredump;
	etdev->debug_dump_handlers[DUMP_REASON_FW_CHECKPOINT] = mobile_sscd_generate_coredump;

	pdev->sscd_info.pdata = &sscd_pdata;
	pdev->sscd_info.dev = &sscd_dev;
	edgetpu_setup_debug_dump_fs(etdev);
	return ret;
out_unregister_platform:
	platform_device_unregister(&sscd_dev);
	return ret;
}

void edgetpu_debug_dump_exit(struct edgetpu_dev *etdev)
{
	if (!etdev->debug_dump_mem.vaddr) {
		etdev_dbg(etdev, "Debug dump not allocated");
		return;
	}
	/*
	 * Free the memory assigned for debug dump
	 */
	edgetpu_free_coherent(etdev, &etdev->debug_dump_mem, EDGETPU_CONTEXT_KCI);
	kfree(etdev->debug_dump_handlers);
	platform_device_unregister(&sscd_dev);
}
