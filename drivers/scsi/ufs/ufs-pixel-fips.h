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

#include "ufshcd.h"

/*
 * ufs_pixel_fips_verify - Performs FIPS compliant self test
 * @hba: adapter instance to execute test on
 *
 * Returns 0 on success, -EIO on UFS I/O error, -EINVAL on
 * encryption/decryption error
 */
int ufs_pixel_fips_verify(struct ufs_hba *hba);

#endif
