// SPDX-License-Identifier: GPL-2.0
/*
 * memory_group_manager.c
 *
 * C) COPYRIGHT 2019 ARM Limited. All rights reserved.
 * C) COPYRIGHT 2019-2021 Google LLC
 *
 */

#include <linux/atomic.h>
#ifdef CONFIG_MALI_MEMORY_GROUP_MANAGER_DEBUG_FS
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
#include <soc/google/meminfo.h>

#include <linux/memory_group_manager.h>

#include "pixel_slc.h"

#include <uapi/gpu/arm/midgard/platform/pixel/pixel_memory_group_manager.h>

#define NUM_PAGES_IN_2MB_LARGE_PAGE (SZ_2M / PAGE_SIZE)
#define ORDER_SMALL_PAGE 0
#define ORDER_LARGE_PAGE const_ilog2(NUM_PAGES_IN_2MB_LARGE_PAGE)

/**
 * enum mgm_group_id - Symbolic names for used memory groups
 */
enum mgm_group_id
{
	/**
	 * @MGM_RESERVED_GROUP_ID: The Mali driver requires that allocations made on one of the
	 *                         groups are not treated specially.
	 */
	MGM_RESERVED_GROUP_ID = 0,

	/**
	 * @MGM_SLC_GROUP_ID: Group for memory that should be cached in the system level cache.
	 */
	MGM_SLC_GROUP_ID = 1,

	/**
	 * @MGM_IMPORTED_MEMORY_GROUP_ID: Imported memory is handled by the allocator of the memory,
	 *                                and the Mali DDK will request a group_id for such memory
	 *                                via mgm_get_import_memory_id(). We specify which group we
	 *                                want to use for this here.
	 */
	MGM_IMPORTED_MEMORY_GROUP_ID = (MEMORY_GROUP_MANAGER_NR_GROUPS - 1),
};

static atomic64_t total_gpu_pages = ATOMIC64_INIT(0);

#define INVALID_GROUP_ID(group_id) \
	WARN_ON((group_id) >= MEMORY_GROUP_MANAGER_NR_GROUPS)

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
 * This structure allows page allocation information to be displayed via
 * debugfs. Display is organized per group with small and large sized pages.
 */
struct mgm_group {
	atomic_t size;
	atomic_t lp_size;
#ifdef CONFIG_MALI_MEMORY_GROUP_MANAGER_DEBUG_FS
	atomic_t insert_pfn;
	atomic_t update_gpu_pte;
#endif
};

/**
 * struct mgm_groups - Structure for groups of memory group manager
 *
 * @groups: To keep track of the number of allocated pages of all groups
 * @ngroups: Number of groups actually used
 * @npartitions: Number of partitions used by all groups combined
 * @dev: device attached
 * @kobj: &sruct kobject used for linking to pixel_stats_sysfs node
 * @mgm_debugfs_root: debugfs root directory of memory group manager
 * @slc_data: To track GPU SLC partitions.
 *
 * This structure allows page allocation information to be displayed via
 * debugfs. Display is organized per group with small and large sized pages.
 */
struct mgm_groups {
	struct mgm_group groups[MEMORY_GROUP_MANAGER_NR_GROUPS];
	struct device *dev;
	struct kobject kobj;
#ifdef CONFIG_MALI_MEMORY_GROUP_MANAGER_DEBUG_FS
	struct dentry *mgm_debugfs_root;
#endif
	struct slc_data slc_data;
};

/*
 * DebugFS
 */

#ifdef CONFIG_MALI_MEMORY_GROUP_MANAGER_DEBUG_FS

static int mgm_debugfs_size_get(void *data, u64 *val)
{
	struct mgm_group *group = data;
	*val = (u64)atomic_read(&group->size);
	return 0;
}

static int mgm_debugfs_lp_size_get(void *data, u64 *val)
{
	struct mgm_group *group = data;
	*val = (u64)atomic_read(&group->lp_size);
	return 0;
}

static int mgm_debugfs_insert_pfn_get(void *data, u64 *val)
{
	struct mgm_group *group = data;
	*val = (u64)atomic_read(&group->insert_pfn);
	return 0;
}

static int mgm_debugfs_update_gpu_pte_get(void *data, u64 *val)
{
	struct mgm_group *group = data;
	*val = (u64)atomic_read(&group->update_gpu_pte);
	return 0;
}

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

#endif /* CONFIG_MALI_MEMORY_GROUP_MANAGER_DEBUG_FS */

/*
 * Pixel Stats sysfs
 */
#ifdef CONFIG_MALI_PIXEL_STATS

extern struct kobject *pixel_stat_gpu_kobj;

#define MGM_ATTR_RO(_name) \
	static struct kobj_attribute _name##_attr = __ATTR_RO(_name)
#define MGM_ATTR_WO(_name) \
	static struct kobj_attribute _name##_attr = __ATTR_WO(_name)

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

static ssize_t slc_pin_partition_store(struct kobject* kobj,
                                       struct kobj_attribute* attr,
                                       const char* buf,
                                       size_t count)
{
	struct mgm_groups *data = container_of(kobj, struct mgm_groups, kobj);
	bool pin;

	if (!data)
		return -ENODEV;

	if (kstrtobool(buf, &pin))
		return -EINVAL;

	slc_pin(&data->slc_data, pin);

	return count;
}
MGM_ATTR_WO(slc_pin_partition);

static struct attribute *mgm_attrs[] = {
	&total_page_count_attr.attr,
	&small_page_count_attr.attr,
	&large_page_count_attr.attr,
	&slc_pin_partition_attr.attr,
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

static unsigned long gpu_meminfo_size(void *private)
{
	return atomic64_read(&total_gpu_pages) << (PAGE_SHIFT - 10);
}

static struct meminfo gpu_meminfo = {
	.name = "Gpu",
	.size_kb = gpu_meminfo_size,
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

	register_meminfo(&gpu_meminfo);
	return 0;
}

static void mgm_sysfs_term(struct mgm_groups *data)
{
	unregister_meminfo(&gpu_meminfo);
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

static atomic_t* get_size_counter(struct memory_group_manager_device* mgm_dev, unsigned int group_id, unsigned int order)
{
	static atomic_t err_atomic;
	struct mgm_groups *data = mgm_dev->data;

	switch (order) {
	case ORDER_SMALL_PAGE:
		return &data->groups[group_id].size;
	case ORDER_LARGE_PAGE:
		return &data->groups[group_id].lp_size;
	default:
		dev_err(data->dev, "Unknown order(%u)\n", order);
		return &err_atomic;
	}
}

static void update_size(struct memory_group_manager_device *mgm_dev, unsigned int
		group_id, unsigned int order, bool alloc)
{
	static DEFINE_RATELIMIT_STATE(gpu_alloc_rs, 10*HZ, 1);
	atomic_t* size = get_size_counter(mgm_dev, group_id, order);

	if (alloc) {
		atomic_inc(size);
		atomic64_add(1 << order, &total_gpu_pages);
	} else {
		WARN_ON(atomic_dec_return(size) < 0);
		atomic64_sub(1 << order, &total_gpu_pages);
	}

	if (atomic64_read(&total_gpu_pages) >= (4 << (30 - PAGE_SHIFT)) &&
			  __ratelimit(&gpu_alloc_rs))
		pr_warn("total_gpu_pages %lld\n", atomic64_read(&total_gpu_pages));
}

static struct page *mgm_alloc_page(
	struct memory_group_manager_device *mgm_dev, unsigned int group_id,
	gfp_t gfp_mask, unsigned int order)
{
	struct mgm_groups *const data = mgm_dev->data;
	struct page *p;

	dev_dbg(data->dev,
		"%s(mgm_dev=%p, group_id=%u gfp_mask=0x%x order=%u\n",
		__func__, (void *)mgm_dev, group_id, gfp_mask, order);

	if (INVALID_GROUP_ID(group_id))
		return NULL;

	/* We don't expect to be allocting pages into the group used for
	 * external or imported memory
	 */
	if (WARN_ON(group_id == MGM_IMPORTED_MEMORY_GROUP_ID))
		return NULL;

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
	struct memory_group_manager_device *mgm_dev, unsigned int group_id,
	struct page *page, unsigned int order)
{
	struct mgm_groups *const data = mgm_dev->data;

	dev_dbg(data->dev, "%s(mgm_dev=%p, group_id=%u page=%p order=%u\n",
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
	struct memory_group_manager_device *const mgm_dev, unsigned int const group_id,
	unsigned int pbha_id, unsigned int pte_flags, int const mmu_level, u64 pte)
{
	struct mgm_groups *const data = mgm_dev->data;
	u64 const old_pte = pte;

	dev_dbg(data->dev,
		"%s(mgm_dev=%p, group_id=%u, pbha_id=%u, pte_flags=%u, mmu_level=%d, pte=0x%llx)\n",
		__func__, (void *)mgm_dev, group_id, pbha_id, pte_flags, mmu_level, pte);

	if (INVALID_GROUP_ID(group_id))
		return pte;

	switch (group_id) {
	case MGM_RESERVED_GROUP_ID:
	case MGM_IMPORTED_MEMORY_GROUP_ID:
		/* The reserved group doesn't set PBHA bits */
		pte = slc_wipe_pbha(pte);
		break;
	case MGM_SLC_GROUP_ID:
		/* Map requests for SLC memory groups to SLC */
		pte = slc_set_pbha(&data->slc_data, pte);
		break;
	default:
		break;
	}

	dev_dbg(data->dev, "%s: group_id=%u pte=0x%llx -> 0x%llx\n",
		__func__, group_id, old_pte, pte);

#ifdef CONFIG_MALI_MEMORY_GROUP_MANAGER_DEBUG_FS
	atomic_inc(&data->groups[group_id].update_gpu_pte);
#endif

	return pte;
}

static u64 mgm_pte_to_original_pte(struct memory_group_manager_device *mgm_dev, unsigned int group_id,
				int mmu_level, u64 pte)
{
	if (INVALID_GROUP_ID(group_id))
		return pte;

	return slc_wipe_pbha(pte);
}

static vm_fault_t mgm_vmf_insert_pfn_prot(
	struct memory_group_manager_device *const mgm_dev, unsigned int const group_id,
	struct vm_area_struct *const vma, unsigned long const addr,
	unsigned long const pfn, pgprot_t const prot)
{
	struct mgm_groups *const data = mgm_dev->data;
	vm_fault_t fault;

	dev_dbg(data->dev,
		"%s(mgm_dev=%p, group_id=%u, vma=%p, addr=0x%lx, pfn=0x%lx,"
		" prot=0x%llx)\n",
		__func__, (void *)mgm_dev, group_id, (void *)vma, addr, pfn,
		pgprot_val(prot));

	if (INVALID_GROUP_ID(group_id))
		return VM_FAULT_SIGBUS;

	fault = vmf_insert_pfn_prot(vma, addr, pfn, prot);

	if (fault != VM_FAULT_NOPAGE)
		dev_err(data->dev, "vmf_insert_pfn_prot failed\n");
#ifdef CONFIG_MALI_MEMORY_GROUP_MANAGER_DEBUG_FS
	else
		atomic_inc(&data->groups[group_id].insert_pfn);
#endif

	return fault;
}

void pixel_mgm_slc_update_signal(struct memory_group_manager_device* mgm_dev, u64 signal)
{
	struct mgm_groups *const data = mgm_dev->data;

	slc_update_signal(&data->slc_data, signal);
}
EXPORT_SYMBOL_GPL(pixel_mgm_slc_update_signal);

void pixel_mgm_slc_inc_refcount(struct memory_group_manager_device* mgm_dev)
{
	struct mgm_groups *const data = mgm_dev->data;

	slc_inc_refcount(&data->slc_data);
}
EXPORT_SYMBOL_GPL(pixel_mgm_slc_inc_refcount);

void pixel_mgm_slc_dec_refcount(struct memory_group_manager_device* mgm_dev)
{
	struct mgm_groups *const data = mgm_dev->data;

	slc_dec_refcount(&data->slc_data);
}
EXPORT_SYMBOL_GPL(pixel_mgm_slc_dec_refcount);

static int mgm_initialize_data(struct mgm_groups *mgm_data)
{
	int i, ret;

	if ((ret = slc_init_data(&mgm_data->slc_data, mgm_data->dev)))
		goto out_err;

	for (i = 0; i < MEMORY_GROUP_MANAGER_NR_GROUPS; i++) {
		atomic_set(&mgm_data->groups[i].size, 0);
		atomic_set(&mgm_data->groups[i].lp_size, 0);
#ifdef CONFIG_MALI_MEMORY_GROUP_MANAGER_DEBUG_FS
		atomic_set(&mgm_data->groups[i].insert_pfn, 0);
		atomic_set(&mgm_data->groups[i].update_gpu_pte, 0);
#endif
	}


	if ((ret = mgm_debugfs_init(mgm_data)))
		goto out_err;

	if ((ret = mgm_sysfs_init(mgm_data)))
		goto out_err;

	return ret;

out_err:
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
	}

	slc_term_data(&data->slc_data);

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
