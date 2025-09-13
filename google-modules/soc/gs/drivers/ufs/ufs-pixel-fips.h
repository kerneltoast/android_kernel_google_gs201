/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Pixel-Specific UFS feature support
 *
 * Copyright 2021 Google LLC
 *
 * Authors: Konstantin Vyshetsky <vkon@google.com>
 */

#ifndef _UFS_PIXEL_FIPS_H_
#define _UFS_PIXEL_FIPS_H_

#include <ufs/ufshcd.h>

enum key_delivery_mode {
	KEY_DELIVERY_SW = 1,
	KEY_DELIVERY_HW = 2
};

struct ufs_pixel_fips_info {
	u32 hmac_self_test_attempted;
	u32 hmac_self_test_passed;
	u32 self_integrity_test_attempted;
	u32 self_integrity_test_passed;
	u32 encryption_test_attempted;
	u32 encryption_test_passed;
	u32 decryption_test_attempted;
	u32 decryption_test_passed;
	u8 ise_version_major;
	u8 ise_version_minor;
	u8 ise_version_revision;
	u8 key_delivery_mode;
};

/*
 * ufs_pixel_fips_verify - Performs FIPS compliant self test
 * @hba: adapter instance to execute test on
 *
 * Returns 0 on success, -EIO on UFS I/O error, -EINVAL on
 * encryption/decryption error
 */
int ufs_pixel_fips_verify(struct ufs_hba *hba);

const struct ufs_pixel_fips_info *ufs_pixel_fips_get_info(struct ufs_hba *hba);

#endif
