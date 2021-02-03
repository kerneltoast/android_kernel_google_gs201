// SPDX-License-Identifier: GPL-2.0
/*
 * ION Buffer Protection for exynos
 *
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 */

#include <linux/slab.h>
#include <linux/genalloc.h>
#include <linux/smc.h>
#include <linux/kmemleak.h>
#include <linux/dma-mapping.h>
#include <linux/arm-smccc.h>
#include <linux/dma-direct.h>
#include <linux/samsung-secure-iova.h>
#include <soc/google/exynos-itmon.h>

#include <linux/ion.h>

#include "ion_exynos.h"

#define SMC_DRM_PPMP_PROT		(0x82002110)
#define SMC_DRM_PPMP_UNPROT		(0x82002111)

static inline unsigned long ppmp_smc(unsigned long cmd, unsigned long arg0,
				     unsigned long arg1, unsigned long arg2)
{
	struct arm_smccc_res res;

	arm_smccc_smc(cmd, arg0, arg1, arg2, 0, 0, 0, 0, &res);
	return (unsigned long)res.a0;
}

static int ion_secure_protect(struct device *dev,
			      struct ion_buffer_prot_info *protdesc,
			      unsigned int protalign)
{
	unsigned long size = protdesc->chunk_count * protdesc->chunk_size;
	unsigned long drmret = 0, dma_addr = 0;
	phys_addr_t protdesc_phys = virt_to_phys(protdesc);
	int ret;

	dma_addr = secure_iova_alloc(size, max_t(u32, protalign, PAGE_SIZE));
	if (!dma_addr)
		return -ENOMEM;

	protdesc->dma_addr = (unsigned int)dma_addr;

	dma_sync_single_for_device(dev, phys_to_dma(dev, protdesc_phys),
				   sizeof(*protdesc), DMA_TO_DEVICE);

	drmret = ppmp_smc(SMC_DRM_PPMP_PROT, protdesc_phys, 0, 0);
	if (drmret) {
		ret = -EACCES;
		goto err_smc;
	}

	return 0;
err_smc:
	secure_iova_free(dma_addr, size);
	perr("CMD %#x (err=%#lx,va=%#lx,len=%#lx,cnt=%u,flg=%u)",
	     SMC_DRM_PPMP_PROT, drmret, dma_addr, size,
	     protdesc->chunk_count, protdesc->flags);

	return ret;
}

static int ion_secure_unprotect(struct ion_buffer_prot_info *protdesc)
{
	unsigned long size = protdesc->chunk_count * protdesc->chunk_size;
	unsigned long ret;

	ret = ppmp_smc(SMC_DRM_PPMP_UNPROT, virt_to_phys(protdesc), 0, 0);

	if (ret) {
		perr("CMD %#x (err=%#lx,va=%#lx,len=%#lx,cnt=%u,flg=%u)",
		     SMC_DRM_PPMP_UNPROT, ret, protdesc->dma_addr,
		     size, protdesc->chunk_count, protdesc->flags);
		return -EACCES;
	}
	/*
	 * retain the secure device address if unprotection to its area fails.
	 * It might be unusable forever since we do not know the state o ft he
	 * secure world before returning error from ppmp_smc() above.
	 */
	secure_iova_free(protdesc->dma_addr, size);

	return 0;
}

void *ion_buffer_protect(struct device *dev, unsigned int protection_id,
			 unsigned int size, unsigned long phys,
			 unsigned int protalign)
{
	struct ion_buffer_prot_info *protdesc;
	int ret;

	protdesc = kmalloc(sizeof(*protdesc), GFP_KERNEL);
	if (!protdesc)
		return ERR_PTR(-ENOMEM);

	protdesc->chunk_count = 1,
	protdesc->flags = protection_id;
	protdesc->chunk_size = ALIGN(size, protalign);
	protdesc->bus_address = phys;

	ret = ion_secure_protect(dev, protdesc, protalign);
	if (ret) {
		perr("protection failure (id%u,len%u,base%#lx,align%#x",
		     protection_id, size, phys, protalign);
		kfree(protdesc);
		return ERR_PTR(ret);
	}

	return protdesc;
}

int ion_buffer_unprotect(void *priv)
{
	struct ion_buffer_prot_info *protdesc = priv;
	int ret = 0;

	if (priv) {
		ret = ion_secure_unprotect(protdesc);
		kfree(protdesc);
	}

	return ret;
}

#if IS_ENABLED(CONFIG_EXYNOS_ITMON)

#define SMC_DRM_PPMPU_INFO	0x820020D1

#define is_secure_info_fail(x)		((((x) >> 16) & 0xffff) == 0xdead)
static inline u32 read_sec_info(unsigned int addr)
{
	unsigned long ret;

	ret = ppmp_smc(SMC_DRM_PPMPU_INFO, addr, 0, 0);
	if (is_secure_info_fail(ret))
		perr("Invalid value returned, %#lx\n", ret);

	return (u32)ret;
}

#define PPMPU0_SFR_BASE         0x228C0000
#define PPMPU1_SFR_BASE         0x229C0000
#define PPMPU2_SFR_BASE         0x22AC0000
#define PPMPU3_SFR_BASE         0x22BC0000

static phys_addr_t ppmpus[] = {
	PPMPU0_SFR_BASE,
	PPMPU1_SFR_BASE,
	PPMPU2_SFR_BASE,
	PPMPU3_SFR_BASE,
};

#define PPMPU_CTRL              0x0

#define PPMPU_ILLEGAL_READ_ADDR_LOW     (0x10)
#define PPMPU_ILLEGAL_READ_ADDR_HIGH    (0x14)
#define PPMPU_ILLEGAL_READ_FIELD        (0x18)

#define PPMPU_ILLEGAL_WRITE_ADDR_LOW    (0x20)
#define PPMPU_ILLEGAL_WRITE_ADDR_HIGH   (0x24)
#define PPMPU_ILLEGAL_WRITE_FIELD       (0x28)

static int __ion_itmon_notifier(struct notifier_block *nb, unsigned long action,
				void *nb_data)
{
	struct itmon_notifier *itmon_info = nb_data;
	unsigned int i;
	int ret = NOTIFY_OK;

	if (IS_ERR_OR_NULL(itmon_info))
		return ret;

	if (!pfn_valid(PHYS_PFN(itmon_info->target_addr)))
		return ret;

#define ERRCODE_DECERR			(1)
	if (itmon_info->errcode != ERRCODE_DECERR)
		return ret;

	perr("PPMPU register dump:\n");
	for (i = 0; i < ARRAY_SIZE(ppmpus); i++) {
		phys_addr_t base = ppmpus[i];
		phys_addr_t read_addr;
		u32 read_field;
		phys_addr_t write_addr;
		u32 write_field;
		u32 ctrl;

		ctrl = read_sec_info(base + PPMPU_CTRL);
		read_addr = (phys_addr_t)read_sec_info(base + PPMPU_ILLEGAL_READ_ADDR_HIGH) << 32;
		read_addr |= (phys_addr_t)read_sec_info(base + PPMPU_ILLEGAL_READ_ADDR_LOW);
		read_field = read_sec_info(base + PPMPU_ILLEGAL_READ_FIELD);
		write_addr = (phys_addr_t)read_sec_info(base + PPMPU_ILLEGAL_WRITE_ADDR_HIGH) << 32;
		write_addr |= (phys_addr_t)read_sec_info(base + PPMPU_ILLEGAL_WRITE_ADDR_LOW);
		write_field = read_sec_info(base + PPMPU_ILLEGAL_WRITE_FIELD);

		perr("PPMPU%u CTRL                %#010x", i, ctrl);
		perr("PPMPU%u ILLEGAL_READ_ADDR   %pap", i, &read_addr);
		perr("PPMPU%u ILLEGAL_READ_FIELD  %#010x", i, read_field);
		perr("PPMPU%u ILLEGAL_WRITE_ADDR  %pap", i, &write_addr);
		perr("PPMPU%u ILLEGAL_WRITE_FIELD %#010x", i, write_field);
		perr("");
	}

	return ret;
}

static struct notifier_block itmon_nb;

int ion_secure_itmon_init(void)
{
	itmon_nb.notifier_call = __ion_itmon_notifier;
	itmon_notifier_chain_register(&itmon_nb);
	return 0;
}

int ion_secure_itmon_exit(void)
{
	itmon_notifier_chain_unregister(&itmon_nb);
	return 0;
}
#endif
