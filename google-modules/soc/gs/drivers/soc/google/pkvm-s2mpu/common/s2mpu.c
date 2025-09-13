// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2021 - Google LLC
 * Author: David Brazdil <dbrazdil@google.com>
 */

#include <linux/kvm_host.h>
#include "kvm_s2mpu.h"
#include <asm/kvm_host.h>
#include <asm/kvm_asm.h>

/* Token of S2MPU driver, token is the load address of the module. */
static u64 token;
/* For an nvhe symbol loaded as a module, get the hyp address of it. */
#define ksym_ref_addr_nvhe(x)	((u64)(&kvm_nvhe_sym(x)) - \
				 (u64)THIS_MODULE->arch.hyp.text.start + token)


extern struct pkvm_iommu_driver kvm_nvhe_sym(pkvm_sysmmu_sync_driver);
extern struct pkvm_iommu_driver kvm_nvhe_sym(pkvm_s2mpu_driver);
extern char __kvm_nvhe___hypmod_text_start[];

static int init_s2mpu_driver(u64 tok)
{
	static DEFINE_MUTEX(lock);
	static bool init_done;
	struct mpt *mpt;
	unsigned int gb;
	unsigned long addr;
	u64 pfn;
	int ret = 0;
	const int smpt_order = SMPT_ORDER(MPT_PROT_BITS);

	mutex_lock(&lock);
	if (init_done)
		goto out;

	token = tok;
	/* Allocate a page for driver data. Must fit MPT descriptor. */
	BUILD_BUG_ON(sizeof(*mpt) > PAGE_SIZE);
	addr = __get_free_page(GFP_KERNEL);
	if (!addr) {
		ret = -ENOMEM;
		goto out;
	}

	mpt = (struct mpt *)addr;

	/* Allocate SMPT buffers. */
	for_each_gb(gb) {
		addr = __get_free_pages(GFP_KERNEL, smpt_order);
		if (!addr) {
			ret = -ENOMEM;
			goto out_free;
		}
		mpt->fmpt[gb].smpt = (u32 *)addr;
	}

	/* Share MPT descriptor with hyp. */
	pfn = __pa(mpt) >> PAGE_SHIFT;
	ret = kvm_call_hyp_nvhe(__pkvm_host_share_hyp, pfn);
	if (ret)
		goto out_free;

	/* Hypercall to initialize EL2 driver. */
	ret = pkvm_iommu_driver_init(ksym_ref_addr_nvhe(pkvm_s2mpu_driver),
				     mpt, sizeof(*mpt));
	if (ret)
		goto out_unshare;

	init_done = true;

out_unshare:
	WARN_ON(kvm_call_hyp_nvhe(__pkvm_host_unshare_hyp, pfn));
out_free:
	/* TODO - will driver return the memory? */
	if (ret) {
		for_each_gb(gb)
			free_pages((unsigned long)mpt->fmpt[gb].smpt, smpt_order);
		free_page((unsigned long)mpt);
	}
out:
	mutex_unlock(&lock);
	return ret;
}

int pkvm_iommu_s2mpu_init(u64 token)
{
	if (!is_protected_kvm_enabled())
		return -ENODEV;

	return init_s2mpu_driver(token);
}
EXPORT_SYMBOL_GPL(pkvm_iommu_s2mpu_init);

int pkvm_iommu_s2mpu_register(struct device *dev, phys_addr_t addr, u8 flags)
{
	if (!is_protected_kvm_enabled())
		return -ENODEV;

	return pkvm_iommu_register(dev, ksym_ref_addr_nvhe(pkvm_s2mpu_driver),
				   addr, S2MPU_MMIO_SIZE, NULL, flags);
}
EXPORT_SYMBOL_GPL(pkvm_iommu_s2mpu_register);

static int init_sysmmu_sync_driver(void)
{
	static DEFINE_MUTEX(lock);
	static bool init_done;

	int ret = 0;

	mutex_lock(&lock);
	if (!init_done) {
		ret = pkvm_iommu_driver_init(ksym_ref_addr_nvhe(pkvm_sysmmu_sync_driver),
					     NULL, 0);
		init_done = !ret;
	}
	mutex_unlock(&lock);
	return ret;
}

int pkvm_iommu_sysmmu_sync_register(struct device *dev, phys_addr_t addr,
				    struct device *parent)
{
	int ret;

	if (!is_protected_kvm_enabled())
		return -ENODEV;

	ret = init_sysmmu_sync_driver();
	if (ret)
		return ret;

	return pkvm_iommu_register(dev, ksym_ref_addr_nvhe(pkvm_sysmmu_sync_driver),
				   addr + SYSMMU_SYNC_S2_OFFSET,
				   SYSMMU_SYNC_S2_MMIO_SIZE, parent, 0);
}
EXPORT_SYMBOL_GPL(pkvm_iommu_sysmmu_sync_register);
