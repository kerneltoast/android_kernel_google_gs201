// SPDX-License-Identifier: GPL-2.0
/*
 * memory_group_manager.c
 *
 * C) COPYRIGHT 2019 ARM Limited. All rights reserved.
 * C) COPYRIGHT 2019-2021 Google LLC
 *
 */

#include <linux/atomic.h>
#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif
#include <linux/fs.h>
#include <linux/kobject.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/limits.h>

#include <linux/memory_group_manager.h>

#include <soc/google/pt.h>

#include <uapi/gpu/arm/midgard/platform/pixel/pixel_memory_group_manager.h>


#define ORDER_SMALL_PAGE 0
#define ORDER_LARGE_PAGE 9

/**
 * Borr does not have "real" PBHA support. However, since we only use a 36-bit PA on the bus,
 * AxADDR[39:36] is wired up to the GPU AxUSER[PBHA] field seen by the rest of the system.
 * Those AxADDR bits come from [39:36] in the page descriptor.
 *
 * Odin and Turse have "real" PBHA support using a dedicated output signal and page descriptor field.
 * The AxUSER[PBHA] field is driven by the GPU's PBHA signal, and AxADDR[39:36] is dropped.
 * The page descriptor PBHA field is [62:59].
 *
 * We could write to both of these locations, as each SoC only reads from its respective PBHA
 * location with the other being ignored or dropped.
 *
 * b/148988078 contains confirmation of the above description.
 */
#if IS_ENABLED(CONFIG_SOC_GS101)
#define PBHA_BIT_POS  (36)
#else
#define PBHA_BIT_POS  (59)
#endif
#define PBHA_BIT_MASK (0xf)

#define MGM_PBHA_DEFAULT 0

#define MGM_SENTINEL_PT_SIZE U64_MAX

#define INVALID_GROUP_ID(group_id) \
	(WARN_ON((group_id) < 0) || \
	WARN_ON((group_id) >= MEMORY_GROUP_MANAGER_NR_GROUPS))

#if (KERNEL_VERSION(4, 20, 0) > LINUX_VERSION_CODE)
static inline vm_fault_t vmf_insert_pfn_prot(struct vm_area_struct *vma,
			unsigned long addr, unsigned long pfn, pgprot_t pgprot)
{
	int err = vm_insert_pfn_prot(vma, addr, pfn, pgprot);

	if (unlikely(err == -ENOMEM))
		return VM_FAULT_OOM;
	if (unlikely(err < 0 && err != -EBUSY))
		return VM_FAULT_SIGBUS;

	return VM_FAULT_NOPAGE;
}
#endif

/**
 * struct mgm_group - Structure to keep track of the number of allocated
 *                    pages per group
 *
 * @size:  The number of allocated small(4KB) pages
 * @lp_size:  The number of allocated large(2MB) pages
 * @insert_pfn: The number of calls to map pages for CPU access.
 * @update_gpu_pte: The number of calls to update GPU page table entries.
 * @ptid: The active partition ID for this group
 * @pbha: The PBHA bits assigned to this group,
 * @base_pt: The base partition ID available to this group.
 * @pt_num: The number of partitions available to this group.
 * @active_pt_idx: The relative index for the partition backing the group.
 *                 Different from the absolute ptid.
 * @state: The lifecycle state of the partition associated with this group
 * This structure allows page allocation information to be displayed via
 * debugfs. Display is organized per group with small and large sized pages.
 */
struct mgm_group {
	atomic_t size;
	atomic_t lp_size;
	atomic_t insert_pfn;
	atomic_t update_gpu_pte;

	ptid_t ptid;
	ptpbha_t pbha;

	u32 base_pt;
	u32 pt_num;
	u32 active_pt_idx;
	enum {
		MGM_GROUP_STATE_NEW = 0,
		MGM_GROUP_STATE_ENABLED = 10,
		MGM_GROUP_STATE_DISABLED_NOT_FREED = 20,
		MGM_GROUP_STATE_DISABLED = 30,
	} state;
};

/**
 * struct partition_stats - Structure for tracking sizing of a partition
 *
 * @capacity: The total capacity of each partition
 * @size: The current size of each partition
 */
struct partition_stats {
	u64 capacity;
	atomic64_t size;
};

/**
 * struct mgm_groups - Structure for groups of memory group manager
 *
 * @groups: To keep track of the number of allocated pages of all groups
 * @ngroups: Number of groups actually used
 * @npartitions: Number of partitions used by all groups combined
 * @pt_stats: The sizing info for each partition
 * @dev: device attached
 * @pt_handle: Link to SLC partition data
 * @kobj: &sruct kobject used for linking to pixel_stats_sysfs node
 * @mgm_debugfs_root: debugfs root directory of memory group manager
 *
 * This structure allows page allocation information to be displayed via
 * debugfs. Display is organized per group with small and large sized pages.
 */
struct mgm_groups {
	struct mgm_group groups[MEMORY_GROUP_MANAGER_NR_GROUPS];
	size_t ngroups;
	size_t npartitions;
	struct partition_stats *pt_stats;
	struct device *dev;
	struct pt_handle *pt_handle;
	struct kobject kobj;
#ifdef CONFIG_DEBUG_FS
	struct dentry *mgm_debugfs_root;
#endif
};

/*
 * DebugFS
 */

#ifdef CONFIG_DEBUG_FS

static int mgm_debugfs_state_get(void *data, u64 *val)
{
	struct mgm_group *group = data;
	*val = (int)group->state;
	return 0;
}

static int mgm_debugfs_size_get(void *data, u64 *val)
{
	struct mgm_group *group = data;
	*val = atomic_read(&group->size);
	return 0;
}

static int mgm_debugfs_lp_size_get(void *data, u64 *val)
{
	struct mgm_group *group = data;
	*val = atomic_read(&group->lp_size);
	return 0;
}

static int mgm_debugfs_insert_pfn_get(void *data, u64 *val)
{
	struct mgm_group *group = data;
	*val = atomic_read(&group->insert_pfn);
	return 0;
}

static int mgm_debugfs_update_gpu_pte_get(void *data, u64 *val)
{
	struct mgm_group *group = data;
	*val = atomic_read(&group->update_gpu_pte);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(fops_mgm_state, mgm_debugfs_state_get,
	NULL, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(fops_mgm_size, mgm_debugfs_size_get,
	NULL, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(fops_mgm_lp_size, mgm_debugfs_lp_size_get,
	NULL, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(fops_mgm_insert_pfn, mgm_debugfs_insert_pfn_get,
	NULL, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(fops_mgm_update_gpu_pte, mgm_debugfs_update_gpu_pte_get,
	NULL, "%llu\n");

static void mgm_debugfs_term(struct mgm_groups *data)
{
	debugfs_remove_recursive(data->mgm_debugfs_root);
}

#define MGM_DEBUGFS_GROUP_NAME_MAX 10

/*
 * attribs - An array of the debug fs files present for each group
 */
static struct {
	const char *name;
	const struct file_operations *fops;
} attribs[] = {
	{ "state", &fops_mgm_state},
	{ "size", &fops_mgm_size},
	{ "lp_size", &fops_mgm_lp_size},
	{ "insert_pfn", &fops_mgm_insert_pfn},
	{ "update_gpu_pte", &fops_mgm_update_gpu_pte},
};

static int mgm_debugfs_init(struct mgm_groups *mgm_data)
{
	int i, j;
	struct dentry *e, *g;
	char debugfs_group_name[MGM_DEBUGFS_GROUP_NAME_MAX];

	/*
	 * Create root directory of memory-group-manager
	 */
	mgm_data->mgm_debugfs_root =
		debugfs_create_dir("physical-memory-group-manager", NULL);
	if (IS_ERR(mgm_data->mgm_debugfs_root)) {
		dev_err(mgm_data->dev,
			"debugfs: Failed to create root directory\n");
		return -ENODEV;
	}

	/*
	 * Create debugfs files per group
	 */
	for (i = 0; i < MEMORY_GROUP_MANAGER_NR_GROUPS; i++) {
		scnprintf(debugfs_group_name, MGM_DEBUGFS_GROUP_NAME_MAX,
				"group_%02d", i);
		g = debugfs_create_dir(debugfs_group_name,
				mgm_data->mgm_debugfs_root);
		if (IS_ERR(g)) {
			dev_err(mgm_data->dev,
				"debugfs: Couldn't create group[%d]\n", i);
			goto remove_debugfs;
		}

		for (j=0; j < ARRAY_SIZE(attribs); j++) {
			e = debugfs_create_file(attribs[j].name, 0444, g,
				&mgm_data->groups[i], attribs[j].fops);

			if (IS_ERR(e)) {
				dev_err(mgm_data->dev,
					"debugfs: Couldn't create %s[%d]\n",
					attribs[j].name, i);
				goto remove_debugfs;
			}
		}
	}

	return 0;

remove_debugfs:
	mgm_debugfs_term(mgm_data);
	return -ENODEV;
}

#else

static void mgm_debugfs_term(struct mgm_groups *data)
{
}

static int mgm_debugfs_init(struct mgm_groups *mgm_data)
{
	return 0;
}

#endif /* CONFIG_DEBUG_FS */

/*
 * Pixel Stats sysfs
 */
#ifdef CONFIG_MALI_PIXEL_STATS

extern struct kobject *pixel_stat_gpu_kobj;

#define MGM_ATTR_RO(_name) \
	static struct kobj_attribute _name##_attr = __ATTR_RO(_name)

static ssize_t total_page_count_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	struct mgm_groups *data = container_of(kobj, struct mgm_groups, kobj);
	int i, pages = 0;

	/* count pages as 4K unit */
	for (i = 0; i < MEMORY_GROUP_MANAGER_NR_GROUPS; i++)
		pages += (atomic_read(&data->groups[i].size) << ORDER_SMALL_PAGE) +
			 (atomic_read(&data->groups[i].lp_size) << ORDER_LARGE_PAGE);

	return sysfs_emit(buf, "%d\n", pages);
}
MGM_ATTR_RO(total_page_count);

static ssize_t small_page_count_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	struct mgm_groups *data = container_of(kobj, struct mgm_groups, kobj);
	int i, pages = 0;

	for (i = 0; i < MEMORY_GROUP_MANAGER_NR_GROUPS; i++)
		pages += atomic_read(&data->groups[i].size);

	return sysfs_emit(buf, "%d\n", pages);
}
MGM_ATTR_RO(small_page_count);

static ssize_t large_page_count_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	struct mgm_groups *data = container_of(kobj, struct mgm_groups, kobj);
	int i, pages = 0;

	for (i = 0; i < MEMORY_GROUP_MANAGER_NR_GROUPS; i++)
		pages += atomic_read(&data->groups[i].lp_size);

	return sysfs_emit(buf, "%d\n", pages);
}
MGM_ATTR_RO(large_page_count);

static struct attribute *mgm_attrs[] = {
	&total_page_count_attr.attr,
	&small_page_count_attr.attr,
	&large_page_count_attr.attr,
	NULL,
};
ATTRIBUTE_GROUPS(mgm);

static void mgm_kobj_release(struct kobject *kobj)
{
	/* Nothing to be done */
}

static struct kobj_type mgm_ktype = {
	.release = mgm_kobj_release,
	.sysfs_ops = &kobj_sysfs_ops,
	.default_groups = mgm_groups,
};

static int mgm_sysfs_init(struct mgm_groups *data)
{
	int ret;
	struct kobject *pixel_gpu_stat = pixel_stat_gpu_kobj;

	WARN_ON(pixel_gpu_stat == NULL);

	ret = kobject_init_and_add(&data->kobj, &mgm_ktype, pixel_gpu_stat, "mem");
	if (ret) {
		kobject_put(&data->kobj);
		return ret;
	}

	return 0;
}

static void mgm_sysfs_term(struct mgm_groups *data)
{
	kobject_put(&data->kobj);
}

#else /* CONFIG_MALI_PIXEL_STATS */

static int mgm_sysfs_init(struct mgm_groups *data)
{
	return 0;
}

static void mgm_sysfs_term(struct mgm_groups *data)
{}

#endif /* CONFIG_MALI_PIXEL_STATS */

static int group_pt_id(struct mgm_groups *data, enum pixel_mgm_group_id group_id, int pt_index)
{
	struct mgm_group *group = &data->groups[group_id];
	if (WARN_ON_ONCE(pt_index >= group->pt_num))
		return 0;

	return group->base_pt + pt_index;
}

static int group_active_pt_id(struct mgm_groups *data, enum pixel_mgm_group_id group_id)
{
	return group_pt_id(data, group_id, data->groups[group_id].active_pt_idx);
}

static atomic64_t total_gpu_pages = ATOMIC64_INIT(0);

static void update_size(struct memory_group_manager_device *mgm_dev, int
		group_id, int order, bool alloc)
{
	static DEFINE_RATELIMIT_STATE(gpu_alloc_rs, 10*HZ, 1);
	struct mgm_groups *data = mgm_dev->data;

	switch (order) {
	case ORDER_SMALL_PAGE:
		if (alloc) {
			atomic_inc(&data->groups[group_id].size);
			atomic64_inc(&total_gpu_pages);
		} else {
			WARN_ON(atomic_read(&data->groups[group_id].size) == 0);
			atomic_dec(&data->groups[group_id].size);
			atomic64_dec(&total_gpu_pages);
		}
	break;

	case ORDER_LARGE_PAGE:
		if (alloc) {
			atomic_inc(&data->groups[group_id].lp_size);
			atomic64_add(1 << ORDER_LARGE_PAGE, &total_gpu_pages);
		} else {
			WARN_ON(atomic_read(
				&data->groups[group_id].lp_size) == 0);
			atomic_dec(&data->groups[group_id].lp_size);
			atomic64_sub(1 << ORDER_LARGE_PAGE, &total_gpu_pages);
		}
	break;

	default:
		dev_err(data->dev, "Unknown order(%d)\n", order);
	break;
	}

	if (atomic64_read(&total_gpu_pages) >= (4 << (30 - PAGE_SHIFT)) &&
			  __ratelimit(&gpu_alloc_rs))
		pr_warn("total_gpu_pages %lld\n", atomic64_read(&total_gpu_pages));
}

static void pt_size_invalidate(struct mgm_groups* data, int pt_idx)
{
	/* Set the size to a known sentinel value so that we can later detect an update */
	atomic64_set(&data->pt_stats[pt_idx].size, MGM_SENTINEL_PT_SIZE);
}

static void pt_size_init(struct mgm_groups* data, int pt_idx, size_t size)
{
	/* The resize callback may have already been executed, which would have set
	 * the correct size. Only update the size if this has not happened.
	 * We can tell that no resize took place if the size is still a sentinel.
	 */
	atomic64_cmpxchg(&data->pt_stats[pt_idx].size, MGM_SENTINEL_PT_SIZE, size);
}

static void validate_ptid(struct mgm_groups* data, enum pixel_mgm_group_id group_id, int ptid)
{
	if (ptid == -EINVAL)
		dev_err(data->dev, "Failed to get partition for group: %d\n", group_id);
	else
		dev_info(data->dev, "pt_client_mutate returned ptid=%d for group=%d", ptid, group_id);
}

static void update_group(struct mgm_groups* data,
                         enum pixel_mgm_group_id group_id,
                         int ptid,
                         int relative_pt_idx)
{
	int const abs_pt_idx = group_pt_id(data, group_id, relative_pt_idx);
	int const pbha = pt_pbha(data->dev->of_node, abs_pt_idx);

	if (pbha == PT_PBHA_INVALID)
		dev_err(data->dev, "Failed to get PBHA for group: %d\n", group_id);
	else
		dev_info(data->dev, "pt_pbha returned PBHA=%d for group=%d", pbha, group_id);

	data->groups[group_id].ptid = ptid;
	data->groups[group_id].pbha = pbha;
	data->groups[group_id].state = MGM_GROUP_STATE_ENABLED;
	data->groups[group_id].active_pt_idx = relative_pt_idx;
}

static void disable_partition(struct mgm_groups* data, enum pixel_mgm_group_id group_id)
{
	int const active_idx = group_active_pt_id(data, group_id);

	/* Skip if not already enabled */
	if (data->groups[group_id].state != MGM_GROUP_STATE_ENABLED)
		return;

	pt_client_disable_no_free(data->pt_handle, active_idx);
	data->groups[group_id].state = MGM_GROUP_STATE_DISABLED_NOT_FREED;

	pt_size_invalidate(data, active_idx);
	pt_size_init(data, active_idx, 0);
}

static void enable_partition(struct mgm_groups* data, enum pixel_mgm_group_id group_id)
{
	int ptid;
	size_t size = 0;
	int const active_idx = group_active_pt_id(data, group_id);

	/* Skip if already enabled */
	if (data->groups[group_id].state == MGM_GROUP_STATE_ENABLED)
		return;

	pt_size_invalidate(data, active_idx);

	ptid = pt_client_enable_size(data->pt_handle, active_idx, &size);

	validate_ptid(data, group_id, ptid);

	update_group(data, group_id, ptid, data->groups[group_id].active_pt_idx);

	pt_size_init(data, active_idx, size);
}

static void set_group_partition(struct mgm_groups* data,
                                enum pixel_mgm_group_id group_id,
                                int new_pt_index)
{
	int ptid;
	size_t size = 0;
	int const active_idx = group_active_pt_id(data, group_id);
	int const new_idx = group_pt_id(data, group_id, new_pt_index);

	/* Early out if no changes are needed */
	if (new_idx == active_idx)
		return;

	pt_size_invalidate(data, new_idx);

	ptid = pt_client_mutate_size(data->pt_handle, active_idx, new_idx, &size);

	validate_ptid(data, group_id, ptid);

	update_group(data, group_id, ptid, new_pt_index);

	pt_size_init(data, new_idx, size);
	/* Reset old partition size */
	atomic64_set(&data->pt_stats[active_idx].size, data->pt_stats[active_idx].capacity);
}

u64 pixel_mgm_query_group_size(struct memory_group_manager_device* mgm_dev,
                               enum pixel_mgm_group_id group_id)
{
	struct mgm_groups *data;
	struct mgm_group *group;
	u64 size = 0;

	/* Early out if the group doesn't exist */
	if (INVALID_GROUP_ID(group_id))
		goto done;

	data = mgm_dev->data;
	group = &data->groups[group_id];

	/* Early out if the group has no partitions */
	if (group->pt_num == 0)
		goto done;

	size = atomic64_read(&data->pt_stats[group_active_pt_id(data, group_id)].size);

done:
	return size;
}
EXPORT_SYMBOL(pixel_mgm_query_group_size);

void pixel_mgm_resize_group_to_fit(struct memory_group_manager_device* mgm_dev,
                                  enum pixel_mgm_group_id group_id,
                                  u64 demand)
{
	struct mgm_groups *data;
	struct mgm_group *group;
	s64 diff, cur_size, min_diff = S64_MAX;
	int pt_idx;

	/* Early out if the group doesn't exist */
	if (INVALID_GROUP_ID(group_id))
		goto done;

	data = mgm_dev->data;
	group = &data->groups[group_id];

	/* Early out if the group has no partitions */
	if (group->pt_num == 0)
		goto done;

	/* We can disable the partition if there's no demand */
	if (demand == 0)
	{
		disable_partition(data, group_id);
		goto done;
	}

	/* Calculate best partition to use, by finding the nearest capacity */
	for (pt_idx = 0; pt_idx < group->pt_num; ++pt_idx)
	{
		cur_size = data->pt_stats[group_pt_id(data, group_id, pt_idx)].capacity;
		diff = abs(demand - cur_size);

		if (diff > min_diff)
			break;

		min_diff = diff;
	}

	/* Ensure the partition is enabled before trying to mutate it */
	enable_partition(data, group_id);
	set_group_partition(data, group_id, pt_idx - 1);

done:
	dev_dbg(data->dev, "%s: resized memory_group_%d for demand: %lldB", __func__, group_id, demand);

	return;
}
EXPORT_SYMBOL(pixel_mgm_resize_group_to_fit);

static struct page *mgm_alloc_page(
	struct memory_group_manager_device *mgm_dev, int group_id,
	gfp_t gfp_mask, unsigned int order)
{
	struct mgm_groups *const data = mgm_dev->data;
	struct page *p;

	dev_dbg(data->dev,
		"%s(mgm_dev=%p, group_id=%d gfp_mask=0x%x order=%u\n",
		__func__, (void *)mgm_dev, group_id, gfp_mask, order);

	if (INVALID_GROUP_ID(group_id))
		return NULL;

	if (WARN_ON_ONCE((group_id != MGM_RESERVED_GROUP_ID) &&
			 (group_active_pt_id(data, group_id) >= data->npartitions)))
		return NULL;

	/* We don't expect to be allocting pages into the group used for
	 * external or imported memory
	 */
	if (WARN_ON(group_id == MGM_IMPORTED_MEMORY_GROUP_ID))
		return NULL;

	/* If we are allocating a page in this group for the first time then
	 *  ensure that we have enabled the relevant partitions for it.
	 */
	if (group_id != MGM_RESERVED_GROUP_ID) {
		switch (data->groups[group_id].state) {
		case MGM_GROUP_STATE_NEW:
			enable_partition(data, group_id);
			break;
		case MGM_GROUP_STATE_ENABLED:
		case MGM_GROUP_STATE_DISABLED_NOT_FREED:
		case MGM_GROUP_STATE_DISABLED:
			/* Everything should already be set up*/
			break;
		default:
			dev_err(data->dev, "Group %d in invalid state %d\n",
				group_id, data->groups[group_id].state);
		}
	}

	p = alloc_pages(gfp_mask, order);

	if (p) {
		update_size(mgm_dev, group_id, order, true);
	} else {
		struct mgm_groups *data = mgm_dev->data;
		dev_err(data->dev, "alloc_pages failed\n");
	}

	return p;
}

static void mgm_free_page(
	struct memory_group_manager_device *mgm_dev, int group_id,
	struct page *page, unsigned int order)
{
	struct mgm_groups *const data = mgm_dev->data;

	dev_dbg(data->dev, "%s(mgm_dev=%p, group_id=%d page=%p order=%u\n",
		__func__, (void *)mgm_dev, group_id, (void *)page, order);

	if (INVALID_GROUP_ID(group_id))
		return;

	__free_pages(page, order);

	/* TODO: Determine the logic of when we disable a partition depending
	 *       on when pages in that group drop to zero? Or after a timeout?
	 */

	update_size(mgm_dev, group_id, order, false);
}

static int mgm_get_import_memory_id(
	struct memory_group_manager_device *mgm_dev,
	struct memory_group_manager_import_data *import_data)
{
	struct mgm_groups *const data = mgm_dev->data;

	dev_dbg(data->dev, "%s(mgm_dev=%p, import_data=%p (type=%d)\n",
		__func__, (void *)mgm_dev, (void *)import_data,
		(int)import_data->type);

	if (!WARN_ON(!import_data)) {
		WARN_ON(!import_data->u.dma_buf);

		WARN_ON(import_data->type !=
				MEMORY_GROUP_MANAGER_IMPORT_TYPE_DMA_BUF);
	}

	return MGM_IMPORTED_MEMORY_GROUP_ID;
}

static u64 mgm_update_gpu_pte(
	struct memory_group_manager_device *const mgm_dev, int const group_id,
	int const mmu_level, u64 pte)
{
	struct mgm_groups *const data = mgm_dev->data;
	unsigned int pbha;

	dev_dbg(data->dev,
		"%s(mgm_dev=%p, group_id=%d, mmu_level=%d, pte=0x%llx)\n",
		__func__, (void *)mgm_dev, group_id, mmu_level, pte);

	if (INVALID_GROUP_ID(group_id))
		return pte;

	/* Clear any bits set in the PBHA range */
	if (pte & ((u64)PBHA_BIT_MASK << PBHA_BIT_POS)) {
		dev_warn(data->dev,
			"%s: updating pte with bits already set in PBHA range",
			__func__);
		pte &= ~((u64)PBHA_BIT_MASK << PBHA_BIT_POS);
	}

	switch (group_id) {
	case MGM_RESERVED_GROUP_ID:
	case MGM_IMPORTED_MEMORY_GROUP_ID:
		/* The reserved group doesn't set PBHA bits */
		/* TODO: Determine what to do with imported memory */
		break;
	default:
		/* All other groups will have PBHA bits */
		if (data->groups[group_id].state > MGM_GROUP_STATE_NEW) {
			u64 old_pte = pte;
			pbha = data->groups[group_id].pbha;

			pte |= ((u64)pbha & PBHA_BIT_MASK) << PBHA_BIT_POS;

			dev_dbg(data->dev,
				"%s: group_id=%d pbha=%d "
				"pte=0x%llx -> 0x%llx\n",
				__func__, group_id, pbha, old_pte, pte);

		} else {
			dev_err(data->dev,
				"Tried to get PBHA of uninitialized group=%d",
				group_id);
		}
	}

	atomic_inc(&data->groups[group_id].update_gpu_pte);

	return pte;
}

static u64 mgm_pte_to_original_pte(struct memory_group_manager_device *mgm_dev, int group_id,
				int mmu_level, u64 pte)
{
	struct mgm_groups *const data = mgm_dev->data;
	u64 old_pte;

	if (INVALID_GROUP_ID(group_id))
		return pte;

	switch (group_id) {
	case MGM_RESERVED_GROUP_ID:
	case MGM_IMPORTED_MEMORY_GROUP_ID:
		/* The reserved group doesn't set PBHA bits */
		/* TODO: Determine what to do with imported memory */
		break;
	default:
		/* All other groups will have PBHA bits, so clear them */
		old_pte = pte;
		pte &= ~((u64)PBHA_BIT_MASK << PBHA_BIT_POS);
		dev_dbg(data->dev, "%s: group_id=%d pte=0x%llx -> 0x%llx\n", __func__, group_id,
			old_pte, pte);
	}

	return pte;
}

static vm_fault_t mgm_vmf_insert_pfn_prot(
	struct memory_group_manager_device *const mgm_dev, int const group_id,
	struct vm_area_struct *const vma, unsigned long const addr,
	unsigned long const pfn, pgprot_t const prot)
{
	struct mgm_groups *const data = mgm_dev->data;
	vm_fault_t fault;

	dev_dbg(data->dev,
		"%s(mgm_dev=%p, group_id=%d, vma=%p, addr=0x%lx, pfn=0x%lx,"
		" prot=0x%llx)\n",
		__func__, (void *)mgm_dev, group_id, (void *)vma, addr, pfn,
		pgprot_val(prot));

	if (INVALID_GROUP_ID(group_id))
		return VM_FAULT_SIGBUS;

	fault = vmf_insert_pfn_prot(vma, addr, pfn, prot);

	if (fault == VM_FAULT_NOPAGE)
		atomic_inc(&data->groups[group_id].insert_pfn);
	else
		dev_err(data->dev, "vmf_insert_pfn_prot failed\n");

	return fault;
}

static void mgm_resize_callback(void *data, int id, size_t size_allocated)
{
	struct mgm_groups *const mgm_data = (struct mgm_groups *)data;
	dev_dbg(mgm_data->dev, "Resize callback called, size_allocated: %zu\n", size_allocated);
	/* Update the partition size for the group */
	atomic64_set(&mgm_data->pt_stats[id].size, size_allocated);
}

static int mgm_initialize_data(struct mgm_groups *mgm_data)
{
	int i, ret;

	/* +1 to include the required default group */
	const int ngroups = of_property_count_strings(mgm_data->dev->of_node, "groups") + 1;
	if (WARN_ON(ngroups < 0) ||
	    WARN_ON(ngroups > MEMORY_GROUP_MANAGER_NR_GROUPS)) {
		mgm_data->ngroups = 0;
	} else {
		mgm_data->ngroups = ngroups;
	}
	mgm_data->npartitions = of_property_count_strings(mgm_data->dev->of_node, "pt_id");

	mgm_data->pt_stats = kzalloc(mgm_data->npartitions * sizeof(struct partition_stats), GFP_KERNEL);
	if (mgm_data->pt_stats == NULL) {
		dev_err(mgm_data->dev, "failed to allocate space for pt_stats");
		ret = -ENOMEM;
		goto out_err;
	}

	for (i = 0; i < mgm_data->npartitions; i++) {
		struct partition_stats* stats;
		u32 capacity_kb;
		ret = of_property_read_u32_index(mgm_data->dev->of_node, "pt_size", i, &capacity_kb);
		if (ret) {
			dev_err(mgm_data->dev, "failed to read pt_size[%d]", i);
			continue;
		}

		stats = &mgm_data->pt_stats[i];
		// Convert from KB to bytes
		stats->capacity = (u64)capacity_kb << 10;
		atomic64_set(&stats->size, stats->capacity);
	}

	for (i = 0; i < MEMORY_GROUP_MANAGER_NR_GROUPS; i++) {
		atomic_set(&mgm_data->groups[i].size, 0);
		atomic_set(&mgm_data->groups[i].lp_size, 0);
		atomic_set(&mgm_data->groups[i].insert_pfn, 0);
		atomic_set(&mgm_data->groups[i].update_gpu_pte, 0);

		mgm_data->groups[i].pbha = MGM_PBHA_DEFAULT;
		mgm_data->groups[i].base_pt = 0;
		mgm_data->groups[i].pt_num = 0;
		mgm_data->groups[i].active_pt_idx = 0;
		mgm_data->groups[i].state = MGM_GROUP_STATE_NEW;
	}

	/* Discover the partitions belonging to each memory group, skipping the reserved group */
	for (i = 1; i < mgm_data->ngroups; i++) {
		/* Device tree has no description for the reserved group */
		int const dt_idx = i - 1;

		int err = of_property_read_u32_index(
		    mgm_data->dev->of_node, "group_base_pt", dt_idx, &mgm_data->groups[i].base_pt);
		if (err) {
			dev_warn(mgm_data->dev, "failed to read base pt index for group %d", i);
			continue;
		}

		err = of_property_read_u32_index(
		    mgm_data->dev->of_node, "group_pt_num", dt_idx, &mgm_data->groups[i].pt_num);
		if (err)
			dev_warn(mgm_data->dev, "failed to read pt number for group %d", i);
	}

	/*
	 * Initialize SLC partitions. We don't enable partitions until
	 * we actually allocate memory to the corresponding memory
	 * group
	 */
	mgm_data->pt_handle =
	    pt_client_register(mgm_data->dev->of_node, (void*)mgm_data, &mgm_resize_callback);

	if (IS_ERR(mgm_data->pt_handle)) {
		ret = PTR_ERR(mgm_data->pt_handle);
		dev_err(mgm_data->dev, "pt_client_register returned %d\n", ret);
		goto out_err;
	}

	/* We don't use PBHA bits for the reserved memory group, and so
	 * it is effectively already initialized.
	 */
	mgm_data->groups[MGM_RESERVED_GROUP_ID].state = MGM_GROUP_STATE_ENABLED;

	if ((ret = mgm_debugfs_init(mgm_data)))
		goto out_err;

	if ((ret = mgm_sysfs_init(mgm_data)))
		goto out_err;

#ifdef CONFIG_MALI_PIXEL_GPU_SLC
	/* We enable the SLC partition by default to support dynamic SLC caching.
	 * Enabling will initialize the partition, by querying the pbha and assigning a ptid.
	 * We then immediately disable the partition, effectively resizing the group to zero,
	 * whilst still retaining other properties such as pbha.
	 */
	enable_partition(mgm_data, MGM_SLC_GROUP_ID);
	disable_partition(mgm_data, MGM_SLC_GROUP_ID);
#endif

	return ret;

out_err:
	kfree(mgm_data->pt_stats);
	return ret;
}

static void mgm_term_data(struct mgm_groups *data)
{
	int i;
	struct mgm_group *group;

	for (i = 0; i < MEMORY_GROUP_MANAGER_NR_GROUPS; i++) {
		group = &data->groups[i];

		/* Shouldn't have outstanding page allocations at this stage*/
		if (atomic_read(&group->size) != 0)
			dev_warn(data->dev,
				"%zu 0-order pages in group(%d) leaked\n",
				(size_t)atomic_read(&group->size), i);
		if (atomic_read(&group->lp_size) != 0)
			dev_warn(data->dev,
				"%zu 9 order pages in group(%d) leaked\n",
				(size_t)atomic_read(&group->lp_size), i);

		/* Disable partition indices and free the partition */
		switch (group->state) {

		case MGM_GROUP_STATE_NEW:
		case MGM_GROUP_STATE_DISABLED:
			/* Nothing to do */
			break;

		case MGM_GROUP_STATE_ENABLED:
			pt_client_disable(data->pt_handle, group_active_pt_id(data, i));
			break;
		case MGM_GROUP_STATE_DISABLED_NOT_FREED:
			pt_client_free(data->pt_handle, group_active_pt_id(data, i));
			break;

		default:
			dev_err(data->dev, "Group %d in invalid state %d\n",
				i, group->state);
		}
	}

	pt_client_unregister(data->pt_handle);

	mgm_debugfs_term(data);
	mgm_sysfs_term(data);
}

static int memory_group_manager_probe(struct platform_device *pdev)
{
	struct memory_group_manager_device *mgm_dev;
	struct mgm_groups *mgm_data;
	int ret;

	mgm_dev = kzalloc(sizeof(*mgm_dev), GFP_KERNEL);
	if (!mgm_dev)
		return -ENOMEM;

	mgm_dev->owner = THIS_MODULE;
	mgm_dev->ops = (struct memory_group_manager_ops){
		.mgm_alloc_page = mgm_alloc_page,
		.mgm_free_page = mgm_free_page,
		.mgm_get_import_memory_id = mgm_get_import_memory_id,
		.mgm_update_gpu_pte = mgm_update_gpu_pte,
		.mgm_pte_to_original_pte = mgm_pte_to_original_pte,
		.mgm_vmf_insert_pfn_prot = mgm_vmf_insert_pfn_prot,
	};

	mgm_data = kzalloc(sizeof(*mgm_data), GFP_KERNEL);
	if (!mgm_data) {
		kfree(mgm_dev);
		return -ENOMEM;
	}

	mgm_dev->data = mgm_data;
	mgm_data->dev = &pdev->dev;

	ret = mgm_initialize_data(mgm_data);
	if (ret) {
		kfree(mgm_data);
		kfree(mgm_dev);
		return ret;
	}

	platform_set_drvdata(pdev, mgm_dev);
	dev_info(&pdev->dev, "Memory group manager probed successfully\n");

	return 0;
}

static int memory_group_manager_remove(struct platform_device *pdev)
{
	struct memory_group_manager_device *mgm_dev =
		platform_get_drvdata(pdev);
	struct mgm_groups *mgm_data = mgm_dev->data;

	mgm_term_data(mgm_data);
	kfree(mgm_data);

	kfree(mgm_dev);

	dev_info(&pdev->dev, "Memory group manager removed successfully\n");

	return 0;
}

static const struct of_device_id memory_group_manager_dt_ids[] = {
	{ .compatible = "arm,physical-memory-group-manager" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, memory_group_manager_dt_ids);

struct platform_driver memory_group_manager_driver = {
	.probe = memory_group_manager_probe,
	.remove = memory_group_manager_remove,
	.driver = {
		.name = "mali-mgm",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(memory_group_manager_dt_ids),
		/*
		 * Prevent the mgm_dev from being unbound and freed, as others
		 * may have pointers to it and would get confused, or crash, if
		 * it suddenly disappeared.
		 */
		.suppress_bind_attrs = true,
	}
};
