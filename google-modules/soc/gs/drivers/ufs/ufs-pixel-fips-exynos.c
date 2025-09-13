// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2021 Google LLC
 */

#include "ufs-pixel-fips.h"
#include "ufs-exynos-gs.h"

#define ISE_VERSION_MAJOR(x)		(((x) >> 16) & 0xFF)
#define ISE_VERSION_MINOR(x)		(((x) >> 8) & 0xFF)
#define ISE_VERSION_REVISION(x)		((x) & 0xFF)

void ufs_report_ise_version_once(struct ufs_hba *hba)
{
	struct exynos_ufs *ufs = to_exynos_ufs(hba);
	struct ufs_vs_handle *handle = &ufs->handle;
	enum { ISE_VERSION_REG_OFFSET = 0x1C };
	static u32 ise_version;

	if (ise_version)
		return;

	ise_version = readl(handle->ufsp + ISE_VERSION_REG_OFFSET);
	pr_info("ISE HW version  %u.%u.%u\n", ISE_VERSION_MAJOR(ise_version),
		ISE_VERSION_MINOR(ise_version),
		ISE_VERSION_REVISION(ise_version));
}

#undef ISE_VERSION_MAJOR
#undef ISE_VERSION_MINOR
#undef ISE_VERSION_REVISION
