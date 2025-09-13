// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Exynos FMP (Flash Memory Protector) UFS crypto support
 *
 * Copyright (C) 2020 Samsung Electronics Co., Ltd.
 * Copyright 2020 Google LLC
 *
 * Authors: Boojin Kim <boojin.kim@samsung.com>
 *	    Eric Biggers <ebiggers@google.com>
 *
 * This is the support for using FMP in the software keys mode
 * (also called the traditional FMP mode or legacy FMP mode).
 */

#include <asm/unaligned.h>
#include <crypto/aes.h>
#include <crypto/algapi.h>
#include <linux/soc/samsung/exynos-smc.h>
#include <core/ufshcd-crypto.h>
#include "ufs-exynos-gs.h"
#include "ufs-pixel-crypto.h"
#include "ufs-pixel-fips.h"

#undef CREATE_TRACE_POINTS
#include <trace/hooks/ufshcd.h>

static int exynos_ufs_swkeys_register_fips_self_test(void);

enum fmp_crypto_algo_mode {
	FMP_BYPASS_MODE = 0,
	FMP_ALGO_MODE_AES_CBC = 1,
	FMP_ALGO_MODE_AES_XTS = 2,
};

#define FMP_DATA_UNIT_SIZE 4096

struct fmp_sg_entry {
	/* The first four fields correspond to those of ufshcd_sg_entry. */
	__le32 des0;
	__le32 des1;
	__le32 des2;
	/*
	 * The algorithm and key length are configured in the high bits of des3,
	 * whose low bits already contain ufshcd_sg_entry::size.
	 */
	__le32 des3;
#define FKL			(1 << 26)
#define SET_KEYLEN(ent, v)	((ent)->des3 |= cpu_to_le32(v))
#define SET_FAS(ent, v)		((ent)->des3 |= cpu_to_le32((v) << 28))

	/* The IV with all bytes reversed */
	__be32 file_iv[4];

	/*
	 * The key with all bytes reversed.  For XTS, the two halves of the key
	 * are given separately and are byte-reversed separately.
	 */
	__be32 file_enckey[8];
	__be32 file_twkey[8];

	/* Not used */
	__be32 disk_iv[4];
	__le32 reserved[4];
};

int exynos_ufs_crypto_init_sw_keys_mode(struct ufs_hba *hba)
{
	long ret;

	ret = exynos_smc(SMC_CMD_FMP_SECURITY, 0, SMU_EMBEDDED, CFG_DESCTYPE_3);
	if (ret) {
		dev_err(hba->dev,
			"SMC_CMD_FMP_SECURITY failed on init: %ld.  Disabling FMP support.\n",
			ret);
		return 0;
	}

	ret = exynos_smc(SMC_CMD_SMU, SMU_INIT, SMU_EMBEDDED, 0);
	if (ret) {
		dev_err(hba->dev,
			"SMC_CMD_SMU(SMU_INIT) failed: %ld.  Disabling FMP support.\n",
			ret);
		return 0;
	}

	ret = exynos_ufs_swkeys_register_fips_self_test();
	if (ret)
		return ret;

	/* Advertise crypto support to ufshcd-core. */
	hba->caps |= UFSHCD_CAP_CRYPTO;

	/* Advertise crypto quirks to ufshcd-core. */
	hba->android_quirks |= UFSHCD_ANDROID_QUIRK_CUSTOM_CRYPTO_PROFILE |
		               UFSHCD_ANDROID_QUIRK_BROKEN_CRYPTO_ENABLE |
		               UFSHCD_ANDROID_QUIRK_KEYS_IN_PRDT;
	hba->sg_entry_size = sizeof(struct fmp_sg_entry);

	/* Advertise crypto capabilities to the block layer. */
	devm_blk_crypto_profile_init(hba->dev, &hba->crypto_profile, 0);
	hba->crypto_profile.max_dun_bytes_supported = AES_BLOCK_SIZE;
	hba->crypto_profile.key_types_supported = BLK_CRYPTO_KEY_TYPE_STANDARD;
	hba->crypto_profile.dev = hba->dev;
	hba->crypto_profile.modes_supported[BLK_ENCRYPTION_MODE_AES_256_XTS] =
		FMP_DATA_UNIT_SIZE;
	return 0;
}

#if IS_ENABLED(CONFIG_SCSI_UFS_PIXEL_FIPS140)
static void exynos_ufs_swkeys_fips_self_test(void *data, struct ufs_hba *hba)
{
	if (ufs_pixel_fips_verify(hba))
		panic("FMP self test failed");
}

static int exynos_ufs_swkeys_register_fips_self_test(void)
{
	return register_trace_android_rvh_ufs_complete_init(
		exynos_ufs_swkeys_fips_self_test, NULL);
}
#else
static inline __be32 fmp_key_word(const u8 *key, int j)
{
	return cpu_to_be32(get_unaligned_le32(
			key + AES_KEYSIZE_256 - (j + 1) * sizeof(__le32)));
}

/* Configure inline encryption (or decryption) on requests that require it. */
static void exynos_ufs_swkeys_fill_prdt(void *unused, struct ufs_hba *hba,
					struct ufshcd_lrb *lrbp,
					unsigned int segments, int *err)
{
	const struct bio_crypt_ctx *bc;
	const u8 *enckey, *twkey;
	u64 dun_lo, dun_hi;
	struct fmp_sg_entry *table;
	unsigned int i;

	/*
	 * There's nothing to do for unencrypted requests, since the mode field
	 * ("FAS") is already 0 (FMP_BYPASS_MODE) by default, as it's in the
	 * same word as ufshcd_sg_entry::size which was already initialized.
	 */
	bc = scsi_cmd_to_rq(lrbp->cmd)->crypt_ctx;
	BUILD_BUG_ON(FMP_BYPASS_MODE != 0);
	if (!bc)
		return;

	enckey = bc->bc_key->raw;
	twkey = enckey + AES_KEYSIZE_256;
	dun_lo = bc->bc_dun[0];
	dun_hi = bc->bc_dun[1];

	/* Reject weak AES-XTS keys. */
	if (!crypto_memneq(enckey, twkey, AES_KEYSIZE_256)) {
		dev_err(hba->dev, "Can't use weak AES-XTS key\n");
		*err = -EIO;
		return;
	}

	/* Configure FMP on each segment of the request. */
	table = (struct fmp_sg_entry *)lrbp->ucd_prdt_ptr;
	for (i = 0; i < segments; i++) {
		struct fmp_sg_entry *ent = &table[i];
		struct ufshcd_sg_entry *prd = (struct ufshcd_sg_entry *)ent;
		int j;

		/* Each segment must be exactly one data unit. */
		if (le32_to_cpu(prd->size) + 1 != FMP_DATA_UNIT_SIZE) {
			dev_err(hba->dev,
				"scatterlist segment is misaligned for FMP\n");
			*err = -EIO;
			return;
		}

		/* Set the algorithm and key length. */
		SET_FAS(ent, FMP_ALGO_MODE_AES_XTS);
		SET_KEYLEN(ent, FKL);

		/* Set the key. */
		for (j = 0; j < AES_KEYSIZE_256 / sizeof(u32); j++) {
			ent->file_enckey[j] = fmp_key_word(enckey, j);
			ent->file_twkey[j] = fmp_key_word(twkey, j);
		}

		/* Set the IV. */
		ent->file_iv[0] = cpu_to_be32(upper_32_bits(dun_hi));
		ent->file_iv[1] = cpu_to_be32(lower_32_bits(dun_hi));
		ent->file_iv[2] = cpu_to_be32(upper_32_bits(dun_lo));
		ent->file_iv[3] = cpu_to_be32(lower_32_bits(dun_lo));

		/* Increment the data unit number. */
		dun_lo++;
		if (dun_lo == 0)
			dun_hi++;
	}
}

static int exynos_ufs_swkeys_register_fips_self_test(void)
{
	/*
	 * The fips module will register internal fill_prdt implementation.
	 * If the module is not enabled, then register fill_rdt here.
	 */
	return register_trace_android_vh_ufs_fill_prdt(
		exynos_ufs_swkeys_fill_prdt, NULL);
}
#endif
