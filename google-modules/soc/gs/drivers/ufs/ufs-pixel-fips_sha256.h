/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Common values for SHA algorithms
 */

#ifndef _UFS_PIXEL_FIPS_SHA_H
#define _UFS_PIXEL_FIPS_SHA_H

#include <linux/types.h>

#define UFS_PIXEL_FIPS_SHA256_DIGEST_SIZE	32
#define UFS_PIXEL_FIPS_SHA256_BLOCK_SIZE	64

#define UFS_PIXEL_FIPS_SHA256_H0	0x6a09e667UL
#define UFS_PIXEL_FIPS_SHA256_H1	0xbb67ae85UL
#define UFS_PIXEL_FIPS_SHA256_H2	0x3c6ef372UL
#define UFS_PIXEL_FIPS_SHA256_H3	0xa54ff53aUL
#define UFS_PIXEL_FIPS_SHA256_H4	0x510e527fUL
#define UFS_PIXEL_FIPS_SHA256_H5	0x9b05688cUL
#define UFS_PIXEL_FIPS_SHA256_H6	0x1f83d9abUL
#define UFS_PIXEL_FIPS_SHA256_H7	0x5be0cd19UL

#define UFS_PIXEL_FIPS_FMP_FIPS_HMAC_INNER_PAD	0x36
#define UFS_PIXEL_FIPS_FMP_FIPS_HMAC_OUTER_PAD	0x5c

struct ufs_pixel_fips_sha256_state {
	u32 state[UFS_PIXEL_FIPS_SHA256_DIGEST_SIZE / 4];
	u64 count;
	u8 buf[UFS_PIXEL_FIPS_SHA256_BLOCK_SIZE];
};

static inline void
ufs_pixel_fips_sha256_init(struct ufs_pixel_fips_sha256_state *sctx)
{
	sctx->state[0] = UFS_PIXEL_FIPS_SHA256_H0;
	sctx->state[1] = UFS_PIXEL_FIPS_SHA256_H1;
	sctx->state[2] = UFS_PIXEL_FIPS_SHA256_H2;
	sctx->state[3] = UFS_PIXEL_FIPS_SHA256_H3;
	sctx->state[4] = UFS_PIXEL_FIPS_SHA256_H4;
	sctx->state[5] = UFS_PIXEL_FIPS_SHA256_H5;
	sctx->state[6] = UFS_PIXEL_FIPS_SHA256_H6;
	sctx->state[7] = UFS_PIXEL_FIPS_SHA256_H7;
	sctx->count = 0;
}

void ufs_pixel_fips_sha256_update(struct ufs_pixel_fips_sha256_state *sctx,
				  const u8 *data, unsigned int len);
void ufs_pixel_fips_sha256_final(struct ufs_pixel_fips_sha256_state *sctx,
				 u8 *out);
void ufs_pixel_fips_sha256(const u8 *data, unsigned int len, u8 *out);
void ufs_pixel_fips_hmac_sha256(const u8 *data, unsigned int len, const u8 *key,
				unsigned int key_len, u8 *out);

#endif
