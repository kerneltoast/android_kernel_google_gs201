// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Exynos FMP (Flash Memory Protector) UFS crypto support
 *
 * Copyright (C) 2020 Samsung Electronics Co., Ltd.
 * Copyright 2020 Google LLC
 *
 * Authors: Boojin Kim <boojin.kim@samsung.com>
 *	    Eric Biggers <ebiggers@google.com>
 */

#include <asm/unaligned.h>
#include <crypto/aes.h>
#include <crypto/algapi.h>
#include <linux/soc/samsung/exynos-smc.h>

#include "ufshcd.h"
#include "ufshcd-crypto.h"
#include "ufs-exynos.h"

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

void exynos_ufs_fmp_init(struct ufs_hba *hba)
{
	long ret;

	ret = exynos_smc(SMC_CMD_FMP_SECURITY, 0, SMU_EMBEDDED, CFG_DESCTYPE_3);
	if (ret) {
		dev_err(hba->dev,
			"SMC_CMD_FMP_SECURITY failed on init: %ld.  Disabling FMP support.\n",
			ret);
		return;
	}

	ret = exynos_smc(SMC_CMD_SMU, SMU_INIT, SMU_EMBEDDED, 0);
	if (ret) {
		dev_err(hba->dev,
			"SMC_CMD_SMU(SMU_INIT) failed: %ld.  Disabling FMP support.\n",
			ret);
		return;
	}

	/* Advertise crypto support to ufshcd-core. */
	hba->caps |= UFSHCD_CAP_CRYPTO;

	/* Advertise crypto quirks to ufshcd-core. */
	hba->quirks |= UFSHCD_QUIRK_NO_KEYSLOTS |
		       UFSHCD_QUIRK_KEYS_IN_PRDT;
	hba->sg_entry_size = sizeof(struct fmp_sg_entry);

	/* Advertise crypto capabilities to the block layer. */
	blk_ksm_init_passthrough(&hba->ksm);
	hba->ksm.max_dun_bytes_supported = AES_BLOCK_SIZE;
	hba->ksm.features = BLK_CRYPTO_FEATURE_STANDARD_KEYS;
	hba->ksm.dev = hba->dev;
	hba->ksm.crypto_modes_supported[BLK_ENCRYPTION_MODE_AES_256_XTS] =
		FMP_DATA_UNIT_SIZE;
}

void exynos_ufs_fmp_resume(struct ufs_hba *hba)
{
	long ret;

	ret = exynos_smc(SMC_CMD_FMP_SECURITY, 0, SMU_EMBEDDED, CFG_DESCTYPE_3);
	if (ret)
		dev_err(hba->dev,
			"SMC_CMD_FMP_SECURITY failed on resume: %ld\n", ret);

	ret = exynos_smc(SMC_CMD_FMP_SMU_RESUME, 0, SMU_EMBEDDED, 0);
	if (ret)
		dev_err(hba->dev, "SMC_CMD_FMP_SMU_RESUME failed: %ld\n", ret);
}

static inline __be32 fmp_key_word(const u8 *key, int j)
{
	return cpu_to_be32(get_unaligned_le32(
			key + AES_KEYSIZE_256 - (j + 1) * sizeof(__le32)));
}

/* Configure FMP on requests that have an encryption context. */
int exynos_ufs_fmp_fill_prdt(struct ufs_hba *hba, struct ufshcd_lrb *lrbp,
			     unsigned int segments)
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
	bc = lrbp->cmd->request->crypt_ctx;
	BUILD_BUG_ON(FMP_BYPASS_MODE != 0);
	if (!bc)
		return 0;

	/* If FMP wasn't enabled, we shouldn't get any encrypted requests. */
	if (WARN_ON_ONCE(!(hba->caps & UFSHCD_CAP_CRYPTO)))
		return -EIO;

	enckey = bc->bc_key->raw;
	twkey = enckey + AES_KEYSIZE_256;
	dun_lo = bc->bc_dun[0];
	dun_hi = bc->bc_dun[1];

	/* Reject weak AES-XTS keys. */
	if (!crypto_memneq(enckey, twkey, AES_KEYSIZE_256)) {
		dev_err(hba->dev, "Can't use weak AES-XTS key\n");
		return -EIO;
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
			return -EIO;
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
	return 0;
}
