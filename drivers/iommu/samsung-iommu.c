// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 */

#define pr_fmt(fmt) "sysmmu: " fmt

#include <linux/dma-iommu.h>
#include <linux/iommu.h>
#include <linux/module.h>
#include <linux/of_iommu.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

typedef u32 sysmmu_iova_t;
typedef u32 sysmmu_pte_t;

#define SECT_ORDER 20
#define LPAGE_ORDER 16
#define SPAGE_ORDER 12

#define SECT_SIZE  (1UL << SECT_ORDER)
#define LPAGE_SIZE (1UL << LPAGE_ORDER)
#define SPAGE_SIZE (1UL << SPAGE_ORDER)

#define NUM_LV1ENTRIES	4096
#define NUM_LV2ENTRIES (SECT_SIZE / SPAGE_SIZE)
#define LV1TABLE_SIZE (NUM_LV1ENTRIES * sizeof(sysmmu_pte_t))
#define LV2TABLE_SIZE (NUM_LV2ENTRIES * sizeof(sysmmu_pte_t))

struct sysmmu_drvdata {
	struct iommu_device iommu;
	struct device *dev;
};

static struct iommu_ops samsung_sysmmu_ops;
static struct platform_driver samsung_sysmmu_driver;

struct samsung_sysmmu_domain {
	struct iommu_domain domain;
	sysmmu_pte_t *page_table;
	atomic_t *lv2entcnt;
	spinlock_t pgtablelock; /* serialize races to page table updates */
};

static bool sysmmu_global_init_done;
static struct device sync_dev;
static struct kmem_cache *flpt_cache, *slpt_cache;

static struct samsung_sysmmu_domain *to_sysmmu_domain(struct iommu_domain *dom)
{
	return container_of(dom, struct samsung_sysmmu_domain, domain);
}

static inline void pgtable_flush(void *vastart, void *vaend)
{
	dma_sync_single_for_device(&sync_dev, virt_to_phys(vastart),
				   vaend - vastart, DMA_TO_DEVICE);
}

static bool samsung_sysmmu_capable(enum iommu_cap cap)
{
	return cap == IOMMU_CAP_CACHE_COHERENCY;
}

static struct iommu_domain *samsung_sysmmu_domain_alloc(unsigned int type)
{
	struct samsung_sysmmu_domain *domain;

	if (type != IOMMU_DOMAIN_UNMANAGED &&
	    type != IOMMU_DOMAIN_DMA &&
	    type != IOMMU_DOMAIN_IDENTITY) {
		pr_err("invalid domain type %u\n", type);
		return NULL;
	}

	domain = kzalloc(sizeof(*domain), GFP_KERNEL);
	if (!domain)
		return NULL;

	domain->page_table =
		(sysmmu_pte_t *)kmem_cache_alloc(flpt_cache,
						 GFP_KERNEL | __GFP_ZERO);
	if (!domain->page_table)
		goto err_pgtable;

	domain->lv2entcnt = kcalloc(NUM_LV1ENTRIES, sizeof(*domain->lv2entcnt),
				    GFP_KERNEL);
	if (!domain->lv2entcnt)
		goto err_counter;

	if (type == IOMMU_DOMAIN_DMA) {
		int ret = iommu_get_dma_cookie(&domain->domain);

		if (ret) {
			pr_err("failed to get dma cookie (%d)\n", ret);
			goto err_get_dma_cookie;
		}
	}

	pgtable_flush(domain->page_table, domain->page_table + NUM_LV1ENTRIES);

	spin_lock_init(&domain->pgtablelock);

	return &domain->domain;

err_get_dma_cookie:
	kfree(domain->lv2entcnt);
err_counter:
	kmem_cache_free(flpt_cache, domain->page_table);
err_pgtable:
	kfree(domain);
	return NULL;
}

static void samsung_sysmmu_domain_free(struct iommu_domain *dom)
{
	struct samsung_sysmmu_domain *domain = to_sysmmu_domain(dom);

	iommu_put_dma_cookie(dom);
	kmem_cache_free(flpt_cache, domain->page_table);
	kfree(domain->lv2entcnt);
	kfree(domain);
}

static int samsung_sysmmu_attach_dev(struct iommu_domain *dom,
				     struct device *dev)
{
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);

	if (!fwspec || fwspec->ops != &samsung_sysmmu_ops) {
		dev_err(dev, "failed to attach, IOMMU instance data %s.\n",
			!fwspec ? "is not initialized" : "has different ops");
		return -ENXIO;
	}

	if (!fwspec->iommu_priv) {
		dev_err(dev, "has no IOMMU\n");
		return -ENODEV;
	}

	return 0;
}

static int samsung_sysmmu_map(struct iommu_domain *dom, unsigned long iova,
			      phys_addr_t paddr, size_t size, int prot)
{
	return 0;
}

static size_t samsung_sysmmu_unmap(struct iommu_domain *dom,
				   unsigned long iova, size_t size,
				   struct iommu_iotlb_gather *gather)
{
	return 0;
}

static void samsung_sysmmu_flush_iotlb_all(struct iommu_domain *dom)
{
}

static void samsung_sysmmu_iotlb_sync(struct iommu_domain *dom,
				      struct iommu_iotlb_gather *gather)
{
}

static phys_addr_t samsung_sysmmu_iova_to_phys(struct iommu_domain *dom,
					       dma_addr_t iova)
{
	return 0;
}

void samsung_sysmmu_dump_pagetable(struct device *dev, dma_addr_t iova)
{
}

static int samsung_sysmmu_add_device(struct device *dev)
{
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);
	struct iommu_group *group;

	if (!fwspec) {
		dev_dbg(dev, "IOMMU instance data is not initialized\n");
		return -ENODEV;
	}

	if (fwspec->ops != &samsung_sysmmu_ops) {
		dev_err(dev, "has different IOMMU ops\n");
		return -ENODEV;
	}

	group = iommu_group_get_for_dev(dev);
	if (IS_ERR(group)) {
		dev_err(dev, "has no suitable IOMMU group. (err:%ld)\n",
			PTR_ERR(group));
		return PTR_ERR(group);
	}

	iommu_group_put(group);

	return 0;
}

static void samsung_sysmmu_remove_device(struct device *dev)
{
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);

	if (!fwspec || fwspec->ops != &samsung_sysmmu_ops)
		return;

	iommu_group_remove_device(dev);
	iommu_fwspec_free(dev);
}

static struct iommu_group *samsung_sysmmu_device_group(struct device *dev)
{
	if (device_iommu_mapped(dev))
		return iommu_group_get(dev);

	return generic_device_group(dev);
}

static int samsung_sysmmu_of_xlate(struct device *dev,
				   struct of_phandle_args *args)
{
	struct platform_device *sysmmu = of_find_device_by_node(args->np);
	struct sysmmu_drvdata *data = platform_get_drvdata(sysmmu);
	struct iommu_fwspec *fwspec;
	unsigned int fwid = 0;
	int ret;

	ret = iommu_device_link(&data->iommu, dev);
	if (ret) {
		dev_err(dev, "failed to iommu device link. (err:%d)\n", ret);
		return ret;
	}

	ret = iommu_fwspec_add_ids(dev, &fwid, 1);
	if (ret) {
		dev_err(dev, "failed to add fwspec. (err:%d)\n", ret);
		iommu_device_unlink(&data->iommu, dev);
		return ret;
	}

	fwspec = dev_iommu_fwspec_get(dev);
	fwspec->iommu_priv = data;
	return ret;
}

static struct iommu_ops samsung_sysmmu_ops = {
	.capable		= samsung_sysmmu_capable,
	.domain_alloc		= samsung_sysmmu_domain_alloc,
	.domain_free		= samsung_sysmmu_domain_free,
	.attach_dev		= samsung_sysmmu_attach_dev,
	.map			= samsung_sysmmu_map,
	.unmap			= samsung_sysmmu_unmap,
	.flush_iotlb_all	= samsung_sysmmu_flush_iotlb_all,
	.iotlb_sync		= samsung_sysmmu_iotlb_sync,
	.iova_to_phys		= samsung_sysmmu_iova_to_phys,
	.add_device		= samsung_sysmmu_add_device,
	.remove_device		= samsung_sysmmu_remove_device,
	.device_group		= samsung_sysmmu_device_group,
	.of_xlate		= samsung_sysmmu_of_xlate,
	.pgsize_bitmap		= SECT_SIZE | LPAGE_SIZE | SPAGE_SIZE,
};

static int samsung_sysmmu_init_global(void)
{
	int ret = 0;

	flpt_cache = kmem_cache_create("samsung-iommu-lv1table",
				       LV1TABLE_SIZE, LV1TABLE_SIZE,
				       0, NULL);
	if (!flpt_cache)
		return -ENOMEM;

	slpt_cache = kmem_cache_create("samsung-iommu-lv2table",
				       LV2TABLE_SIZE, LV2TABLE_SIZE,
				       0, NULL);
	if (!slpt_cache) {
		ret = -ENOMEM;
		goto err_init_slpt_fail;
	}

	bus_set_iommu(&platform_bus_type, &samsung_sysmmu_ops);

	device_initialize(&sync_dev);
	sysmmu_global_init_done = true;

	return 0;

err_init_slpt_fail:
	kmem_cache_destroy(flpt_cache);

	return ret;
}

static int samsung_sysmmu_device_probe(struct platform_device *pdev)
{
	struct sysmmu_drvdata *data;
	struct device *dev = &pdev->dev;
	int err = 0;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	data->dev = dev;

	err = iommu_device_sysfs_add(&data->iommu, data->dev,
				     NULL, dev_name(dev));
	if (err) {
		dev_err(dev, "failed to register iommu in sysfs\n");
		return err;
	}

	iommu_device_set_ops(&data->iommu, &samsung_sysmmu_ops);
	iommu_device_set_fwnode(&data->iommu, dev->fwnode);

	err = iommu_device_register(&data->iommu);
	if (err) {
		dev_err(dev, "failed to register iommu\n");
		goto err_iommu_register;
	}

	if (!sysmmu_global_init_done) {
		err = samsung_sysmmu_init_global();
		if (err) {
			dev_err(dev, "failed to initialize global data\n");
			goto err_global_init;
		}
	}

	platform_set_drvdata(pdev, data);

	dev_info(dev, "initialized IOMMU\n");

	return 0;

err_global_init:
	iommu_device_unregister(&data->iommu);
err_iommu_register:
	iommu_device_sysfs_remove(&data->iommu);
	return err;
}

static void samsung_sysmmu_device_shutdown(struct platform_device *pdev)
{
}

static const struct of_device_id sysmmu_of_match[] = {
	{ .compatible = "samsung,sysmmu-v8" },
	{ }
};

static struct platform_driver samsung_sysmmu_driver = {
	.driver	= {
		.name			= "samsung-sysmmu",
		.of_match_table		= of_match_ptr(sysmmu_of_match),
		.suppress_bind_attrs	= true,
	},
	.probe	= samsung_sysmmu_device_probe,
	.shutdown = samsung_sysmmu_device_shutdown,
};
module_platform_driver(samsung_sysmmu_driver);
MODULE_LICENSE("GPL v2");
