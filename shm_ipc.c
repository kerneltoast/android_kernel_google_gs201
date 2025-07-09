// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2014-2019, Samsung Electronics.
 *
 */

#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/of_reserved_mem.h>
#include <linux/of_fdt.h>
#include <linux/shm_ipc.h>

#include "modem_utils.h"

/*
 * Reserved memory
 */
#define MAX_CP_RMEM	10

struct cp_reserved_mem {
	char *name;
	u32 index;
	unsigned long p_base;
	u32 size;
};

static int _rmem_count;
static struct cp_reserved_mem _cp_rmem[MAX_CP_RMEM];

#if defined(MODULE)
static int cp_rmem_setup_latecall(struct platform_device *pdev)
{
	struct device_node *np;
	struct reserved_mem *rmem;
	u32 rmem_index = 0;
	int i;

	for (i = 0; i < MAX_CP_RMEM; i++) {
		np = of_parse_phandle(pdev->dev.of_node, "memory-region", i);
		if (!np)
			break;

		mif_dt_read_u32(np, "rmem_index", rmem_index);

		rmem = of_reserved_mem_lookup(np);
		if (!rmem) {
			mif_err("of_reserved_mem_lookup() failed\n");
			break;
		}

		_cp_rmem[i].index = rmem_index;
		_cp_rmem[i].name = (char *)rmem->name;
		_cp_rmem[i].p_base = rmem->base;
		_cp_rmem[i].size = rmem->size;

		mif_info("rmem %d %s 0x%08lx 0x%08x\n",
				_cp_rmem[i].index, _cp_rmem[i].name,
				_cp_rmem[i].p_base, _cp_rmem[i].size);
	}

	return 0;
}
#else
static int __init cp_rmem_setup(struct reserved_mem *rmem)
{
	const __be32 *prop;
	int len;

	if (_rmem_count >= MAX_CP_RMEM) {
		mif_err("_cp_rmem is full for %s\n", rmem->name);
		return -ENOMEM;
	}

	prop = of_get_flat_dt_prop(rmem->fdt_node, "rmem_index", &len);
	if (!prop) {
		mif_err("rmem_name is not defined for %s\n", rmem->name);
		return -ENOENT;
	}
	_cp_rmem[be32_to_cpu(prop[0])].index = be32_to_cpu(prop[0]);
	_cp_rmem[be32_to_cpu(prop[0])].name = (char *)rmem->name;
	_cp_rmem[be32_to_cpu(prop[0])].p_base = rmem->base;
	_cp_rmem[be32_to_cpu(prop[0])].size = rmem->size;
	_rmem_count++;

	mif_info("rmem %d %s 0x%08lx 0x%08x\n",
			_cp_rmem[be32_to_cpu(prop[0])].index, _cp_rmem[be32_to_cpu(prop[0])].name,
			_cp_rmem[be32_to_cpu(prop[0])].p_base, _cp_rmem[be32_to_cpu(prop[0])].size);

	return 0;
}
RESERVEDMEM_OF_DECLARE(modem_if, "exynos,modem_if", cp_rmem_setup);

#endif

/*
 * Shared memory
 */
struct cp_shared_mem {
	char *name;
	u32 index;
	u32 rmem;
	u32 cp_num;
	unsigned long p_base;
	u32 size;
	bool cached;
	void __iomem *v_base;
};

static struct cp_shared_mem _cp_shmem[MAX_CP_NUM][MAX_CP_SHMEM];

static int cp_shmem_setup(struct device *dev)
{
	struct device_node *regions = NULL;
	struct device_node *child = NULL;
	u32 cp_num;
	u32 shmem_index, rmem_index;
	u32 offset;
	u32 count = 0;

	mif_dt_read_u32(dev->of_node, "cp_num", cp_num);

	regions = of_get_child_by_name(dev->of_node, "regions");
	if (!regions) {
		mif_err("of_get_child_by_name() error:regions\n");
		return -EINVAL;
	}

	for_each_child_of_node(regions, child) {
		if (count >= MAX_CP_SHMEM) {
			mif_err("_cp_shmem is full for %d\n", count);
			return -ENOMEM;
		}
		mif_dt_read_u32(child, "region,index", shmem_index);
		_cp_shmem[cp_num][shmem_index].index = shmem_index;
		_cp_shmem[cp_num][shmem_index].cp_num = cp_num;
		mif_dt_read_string(child, "region,name", _cp_shmem[cp_num][shmem_index].name);

		mif_dt_read_u32(child, "region,rmem", _cp_shmem[cp_num][shmem_index].rmem);
		rmem_index = _cp_shmem[cp_num][shmem_index].rmem;
		if (!_cp_rmem[rmem_index].p_base) {
			mif_err("_cp_rmem[%d].p_base is null\n", rmem_index);
			return -ENOMEM;
		}
		mif_dt_read_u32(child, "region,offset", offset);

		_cp_shmem[cp_num][shmem_index].p_base = _cp_rmem[rmem_index].p_base + offset;
		mif_dt_read_u32(child, "region,size", _cp_shmem[cp_num][shmem_index].size);
		if ((_cp_shmem[cp_num][shmem_index].p_base + _cp_shmem[cp_num][shmem_index].size) >
			(_cp_rmem[rmem_index].p_base + _cp_rmem[rmem_index].size)) {
			mif_err("%d %d size error 0x%08lx 0x%08x 0x%08lx 0x%08x\n",
				rmem_index, shmem_index,
				_cp_shmem[cp_num][shmem_index].p_base,
				_cp_shmem[cp_num][shmem_index].size,
				_cp_rmem[rmem_index].p_base, _cp_rmem[rmem_index].size);
			return -ENOMEM;
		}

		mif_dt_read_u32_noerr(child, "region,cached",
				_cp_shmem[cp_num][shmem_index].cached);
		count++;
	}

	return 0;
}

/*
 * Memory map on CP binary
 */
#define MAX_MAP_ON_CP		10
#define MAP_ON_CP_OFFSET	0xA0

struct ns_map_info {	/* Non secure map */
	u32 name;
	u32 offset;
	u32 size;
};

struct cp_mem_map {	/* Memory map on CP */
	u32 version;
	u32 secure_size;
	u32 ns_size;
	u32 ns_map_count;
	struct ns_map_info ns_map[MAX_MAP_ON_CP];
	u32 mem_guard;
};

struct cp_toc {		/* CP TOC */
	char name[12];
	u32 img_offset;
	u32 mem_offset;
	u32 size;
	u32 crc;
	u32 reserved;
} __packed;

static int _mem_map_on_cp[MAX_CP_NUM] = {};

static int cp_shmem_check_mem_map_on_cp(struct device *dev)
{
	u32 cp_num = 0;
	void __iomem *base = NULL;
	struct cp_toc *toc = NULL;
	struct cp_mem_map map = {};
	u32 name;
	int i;
	u32 rmem_index = 0;
	u32 shmem_index = 0;
	long long base_diff = 0;

	mif_dt_read_u32(dev->of_node, "cp_num", cp_num);
	base = phys_to_virt(cp_shmem_get_base(cp_num, SHMEM_CP));
	if (!base) {
		mif_err("base is null\n");
		return -ENOMEM;
	}

	toc = (struct cp_toc *)(base + sizeof(struct cp_toc));
	if (!toc) {
		mif_err("toc is null\n");
		return -ENOMEM;
	}
	mif_info("offset:0x%08x\n", toc->img_offset);

	if (toc->img_offset > (cp_shmem_get_size(cp_num, SHMEM_CP) - MAP_ON_CP_OFFSET)) {
		mif_info("Invalid img_offset:0x%08x. Use dt information\n", toc->img_offset);
		return 0;
	}

	memcpy(&map, base + toc->img_offset + MAP_ON_CP_OFFSET, sizeof(struct cp_mem_map));
	name = ntohl(map.version);
	if (strncmp((const char *)&name, "MEM\0", sizeof(name))) {
		mif_err("map.version error:0x%08x. Use dt information\n", map.version);
		return 0;
	}

	_mem_map_on_cp[cp_num] = 1;

	mif_info("secure_size:0x%08x ns_size:0x%08x count:%d\n",
			map.secure_size, map.ns_size, map.ns_map_count);
	_cp_shmem[cp_num][SHMEM_CP].size = map.secure_size;

	for (i = 0; i < map.ns_map_count; i++) {
		name = map.ns_map[i].name;
		if (!strncmp((const char *)&name, "CPI\0", sizeof(name)))
			shmem_index = SHMEM_IPC;
		else if (!strncmp((const char *)&name, "SSV\0", sizeof(name)))
			shmem_index = SHMEM_VSS;
		else if (!strncmp((const char *)&name, "GOL\0", sizeof(name)))
			shmem_index = SHMEM_BTL;
		else if (!strncmp((const char *)&name, "EGOL\0", sizeof(name)))
			shmem_index = SHMEM_BTL_EXT;
		else if (!strncmp((const char *)&name, "B2L\0", sizeof(name)))
			shmem_index = SHMEM_L2B;
		else if (!strncmp((const char *)&name, "PKP\0", sizeof(name)))
			shmem_index = SHMEM_PKTPROC;
		else if (!strncmp((const char *)&name, "UKP\0", sizeof(name)))
			shmem_index = SHMEM_PKTPROC_UL;
		else if (!strncmp((const char *)&name, "MDD\0", sizeof(name)))
			shmem_index = SHMEM_DDM;
		else
			continue;

		rmem_index = _cp_shmem[cp_num][shmem_index].rmem;
		if (!_cp_rmem[rmem_index].p_base) {
			mif_err("_cp_rmem[%d].p_base is null\n", rmem_index);
			return -ENOMEM;
		}

		base_diff = _cp_rmem[rmem_index].p_base - _cp_rmem[0].p_base;
		_cp_shmem[cp_num][shmem_index].p_base =
			_cp_rmem[rmem_index].p_base + map.ns_map[i].offset - base_diff;
		_cp_shmem[cp_num][shmem_index].size = map.ns_map[i].size;

		if ((_cp_shmem[cp_num][shmem_index].p_base + _cp_shmem[cp_num][shmem_index].size) >
			(_cp_rmem[rmem_index].p_base + _cp_rmem[rmem_index].size)) {
			mif_err("rmem:%d shmem_index:%d size error 0x%08lx 0x%08x 0x%08lx 0x%08x\n",
				rmem_index, shmem_index,
				_cp_shmem[cp_num][shmem_index].p_base,
				_cp_shmem[cp_num][shmem_index].size,
				_cp_rmem[rmem_index].p_base, _cp_rmem[rmem_index].size);
			return -ENOMEM;
		}

		mif_info("rmem:%d shmem_index:%d base:0x%08lx offset:0x%08x size:0x%08x\n",
				rmem_index, shmem_index, _cp_rmem[rmem_index].p_base,
				map.ns_map[i].offset, map.ns_map[i].size);
	}

	return 0;
}

/*
 * Export functions - legacy
 */
unsigned long shm_get_msi_base(void)
{
	return cp_shmem_get_base(0, SHMEM_MSI);
}
EXPORT_SYMBOL(shm_get_msi_base);

void __iomem *shm_get_vss_region(void)
{
	return cp_shmem_get_region(0, SHMEM_VSS);
}
EXPORT_SYMBOL(shm_get_vss_region);

unsigned long shm_get_vss_base(void)
{
	return cp_shmem_get_base(0, SHMEM_VSS);
}
EXPORT_SYMBOL(shm_get_vss_base);

u32 shm_get_vss_size(void)
{
	return cp_shmem_get_size(0, SHMEM_VSS);
}
EXPORT_SYMBOL(shm_get_vss_size);

void __iomem *shm_get_vparam_region(void)
{
	return cp_shmem_get_region(0, SHMEM_VPA);
}
EXPORT_SYMBOL(shm_get_vparam_region);

unsigned long shm_get_vparam_base(void)
{
	return cp_shmem_get_base(0, SHMEM_VPA);
}
EXPORT_SYMBOL(shm_get_vparam_base);

u32 shm_get_vparam_size(void)
{
	return cp_shmem_get_size(0, SHMEM_VPA);
}
EXPORT_SYMBOL(shm_get_vparam_size);

/*
 * Export functions
 */
int cp_shmem_get_mem_map_on_cp_flag(u32 cp_num)
{
	mif_info("cp:%d flag:%d\n", cp_num, _mem_map_on_cp[cp_num]);

	return _mem_map_on_cp[cp_num];
}
EXPORT_SYMBOL(cp_shmem_get_mem_map_on_cp_flag);

void __iomem *cp_shmem_get_nc_region(unsigned long base, u32 size)
{
	unsigned int num_pages = (unsigned int)DIV_ROUND_UP(size, PAGE_SIZE);
	pgprot_t prot = pgprot_writecombine(PAGE_KERNEL);
	struct page **pages;
	void *v_addr;
	unsigned int i;

	if (!base)
		return NULL;

	pages = kvcalloc(num_pages, sizeof(struct page *), GFP_KERNEL);
	if (!pages)
		return NULL;

	for (i = 0; i < num_pages; i++) {
		pages[i] = phys_to_page(base);
		base += PAGE_SIZE;
	}

	v_addr = vmap(pages, num_pages, VM_MAP, prot);
	if (!v_addr)
		mif_err("Failed to vmap pages\n");

	kvfree(pages);

	return (void __iomem *)v_addr;
}
EXPORT_SYMBOL(cp_shmem_get_nc_region);

void __iomem *cp_shmem_get_region(u32 cp, u32 idx)
{
	if (_cp_shmem[cp][idx].v_base)
		return _cp_shmem[cp][idx].v_base;

	if (_cp_shmem[cp][idx].cached)
		_cp_shmem[cp][idx].v_base = phys_to_virt(_cp_shmem[cp][idx].p_base);
	else
		_cp_shmem[cp][idx].v_base = cp_shmem_get_nc_region(_cp_shmem[cp][idx].p_base,
				_cp_shmem[cp][idx].size);

	return _cp_shmem[cp][idx].v_base;
}
EXPORT_SYMBOL(cp_shmem_get_region);

void cp_shmem_release_region(u32 cp, u32 idx)
{
	if (_cp_shmem[cp][idx].v_base)
		vunmap(_cp_shmem[cp][idx].v_base);
}
EXPORT_SYMBOL(cp_shmem_release_region);

void cp_shmem_release_rmem(u32 cp, u32 idx, u32 headroom)
{
	int i;
	unsigned long base, offset = 0;
	u32 size;
	struct page *page;

	base = cp_shmem_get_base(cp, idx);
	size = cp_shmem_get_size(cp, idx);
	mif_info("Release rmem base:0x%08lx size:0x%08x headroom:0x%08x\n", base, size, headroom);

	for (i = 0; i < (size >> PAGE_SHIFT); i++) {
		if (offset >= headroom) {
			page = phys_to_page(base + offset);
			free_reserved_page(page);
		}
		offset += PAGE_SIZE;
	}
}
EXPORT_SYMBOL(cp_shmem_release_rmem);

unsigned long cp_shmem_get_base(u32 cp, u32 idx)
{
	return _cp_shmem[cp][idx].p_base;
}
EXPORT_SYMBOL(cp_shmem_get_base);

u32 cp_shmem_get_size(u32 cp, u32 idx)
{
	return _cp_shmem[cp][idx].size;
}
EXPORT_SYMBOL(cp_shmem_get_size);

/*
 * Platform driver
 */
static int cp_shmem_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret = 0;
	u32 use_map_on_cp = 0;
	int i, j;

	mif_info("+++\n");

#if defined(MODULE)
	if (!_rmem_count) {
		ret = cp_rmem_setup_latecall(pdev);
		if (ret) {
			mif_err("cp_rmem_setup_latecall() error:%d\n", ret);
			goto fail;
		}
	}
#endif

	ret = cp_shmem_setup(dev);
	if (ret) {
		mif_err("cp_shmem_setup() error:%d\n", ret);
		goto fail;
	}

	mif_dt_read_u32(dev->of_node, "use_mem_map_on_cp", use_map_on_cp);
	if (use_map_on_cp) {
		ret = cp_shmem_check_mem_map_on_cp(dev);
		if (ret) {
			mif_err("cp_shmem_check_mem_map_on_cp() error:%d\n", ret);
			goto fail;
		}
	} else {
		mif_info("use_mem_map_on_cp is disabled. Use dt information\n");
	}

	for (i = 0; i < MAX_CP_NUM; i++) {
		for (j = 0; j < MAX_CP_SHMEM; j++) {
			if (!_cp_shmem[i][j].name)
				continue;

			mif_info("cp_num:%d rmem:%d index:%d %s 0x%08lx 0x%08x %d\n",
					_cp_shmem[i][j].cp_num, _cp_shmem[i][j].rmem,
					_cp_shmem[i][j].index, _cp_shmem[i][j].name,
					_cp_shmem[i][j].p_base, _cp_shmem[i][j].size,
					_cp_shmem[i][j].cached);
		}
	}

	mif_info("---\n");

	return 0;

fail:
	panic("CP shmem probe failed\n");
	return ret;
}

static int cp_shmem_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id cp_shmem_dt_match[] = {
	{ .compatible = "samsung,exynos-cp-shmem", },
	{},
};
MODULE_DEVICE_TABLE(of, cp_shmem_dt_match);

static struct platform_driver cp_shmem_driver = {
	.probe = cp_shmem_probe,
	.remove = cp_shmem_remove,
	.driver = {
		.name = "cp_shmem",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(cp_shmem_dt_match),
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(cp_shmem_driver);

MODULE_SOFTDEP("pre: spi-s3c64xx");
MODULE_DESCRIPTION("Exynos CP shared memory driver");
MODULE_LICENSE("GPL");
