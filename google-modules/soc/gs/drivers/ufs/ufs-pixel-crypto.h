/* SPDX-License-Identifier: GPL-2.0-only */
//
// Copyright 2020 Google LLC

#ifndef _UFS_PIXEL_CRYPTO_H_
#define _UFS_PIXEL_CRYPTO_H_

#define CRYPTO_DATA_UNIT_SIZE 4096

#ifdef CONFIG_SCSI_UFS_CRYPTO
int pixel_ufs_crypto_init(struct ufs_hba *hba);
void pixel_ufs_crypto_resume(struct ufs_hba *hba);
#ifdef CONFIG_SCSI_UFS_CRYPTO_SW_KEYS_MODE
int exynos_ufs_crypto_init_sw_keys_mode(struct ufs_hba *hba);
#endif /* CONFIG_SCSI_UFS_CRYPTO_SW_KEYS_MODE */
#else
static inline int pixel_ufs_crypto_init(struct ufs_hba *hba)
{
	return 0;
}
static inline void pixel_ufs_crypto_resume(struct ufs_hba *hba)
{
}
#endif /* !CONFIG_SCSI_UFS_CRYPTO */

#endif /* _UFS_PIXEL_CRYPTO_H_ */
