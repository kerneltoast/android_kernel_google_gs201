/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Pixel-Specific UFS feature support
 *
 * Copyright 2021 Google LLC
 *
 * Authors: Konstantin Vyshetsky <vkon@google.com>
 */

#include <linux/ioctl.h>
#include <linux/types.h>

enum ufs_pixel_acvp_aes_direction {
	ACVP_ENCRYPT,
	ACVP_DECRYPT,
	ACVP_AES_DIRECTION_MAX
};

enum ufs_pixel_acvp_aes_algorithm {
	AES_XTS,
	AES_CBC,
	AES_CBC_MCT
};

struct ufs_pixel_acvp_aes_req {
	__u64 src_ptr;
	__u64 dst_ptr;
	__u64 key_ptr;
	__u64 iv_ptr;
	__u32 src_len;
	__u32 dst_len;
	__u32 key_len;
	__u32 iv_len;
	__u32 aes_direction;
	__u32 aes_algorithm;
};

enum ufs_pixel_acvp_hash_type {
	HMAC_SHA_256,
	SHA_256,
	SHA_256_MCT
};

struct ufs_pixel_acvp_hash_req {
	__u64 src_ptr;
	__u64 dst_ptr;
	__u64 key_ptr;
	__u32 src_len;
	__u32 key_len;
	__u32 hash_type;
};

#define UFS_PIXEL_ACVP_AES_REQ	_IOWR('a', 1, struct ufs_pixel_acvp_aes_req)
#define UFS_PIXEL_ACVP_HASH_REQ	_IOWR('h', 2, struct ufs_pixel_acvp_hash_req)
