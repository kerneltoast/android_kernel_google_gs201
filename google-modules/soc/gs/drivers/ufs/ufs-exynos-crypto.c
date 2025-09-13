// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Pixel-specific UFS inline encryption support using FMP (Flash Memory
 * Protector) and the KDN (Key Distribution Network)
 *
 * Copyright 2020 Google LLC
 */
#include <linux/gsa/gsa_kdn.h>
#include <linux/regmap.h>
#include <linux/soc/samsung/exynos-smc.h>
#include <trace/hooks/ufshcd.h>
#include "ufs-exynos-gs.h"
#include "ufs-pixel-crypto.h"
#include "ufs-pixel-fips.h"

#define HSI2_KDN_CONTROL_MONITOR	0x400	/* offset from HSI2 base */
#define MKE_MONITOR			BIT(0)	/* Master Key Enable */
#define DT_MONITOR			BIT(1)	/* Descriptor Type */
#define RDY_MONITOR			BIT(2)	/* KDN ready? */

/*
 * Format of UFS PRDT entries when the KDN is enabled and the PRDT-based
 * descriptor mode is enabled.  In this mode, when the data in a UFS request
 * should be encrypted (or decrypted), the keyslot and IV for each 4KB of data
 * is specified in the corresponding PRDT entry.  This uses extra fields beyond
 * the ones specified by the UFSHCI standard.
 */
struct pixel_ufs_prdt_entry {
	/* The first four fields correspond to those of ufshcd_sg_entry. */
	__le32 des0;
	__le32 des1;
	__le32 des2;
	/*
	 * The crypto enable bit and keyslot are configured in the high bits of
	 * des3, whose low bits already contain ufshcd_sg_entry::size.
	 */
#define CRYPTO_ENABLE		(1U << 31)
#define CRYPTO_KEYSLOT(keyslot)	((keyslot) << 18)
	__le32 des3;

	/* The IV with all bytes reversed */
	__be64 iv[2];

	/* Unused (when KE=0) */
	__le32 nonce[4];

	/* Unused */
	__le32 reserved[20];
};

/*
 * Read the HSI2_KDN_CONTROL_MONITOR register to verify that the KDN is
 * configured correctly.
 *
 * Note that the KE (KDF Enable) bit isn't shown by the register, as it is
 * actually a per-keyslot thing.  So we can't verify KE=0 here.
 */
static void exynos_check_crypto_hw(struct ufs_hba *hba)
{
	struct exynos_ufs *ufs = to_exynos_ufs(hba);
	unsigned int val = 0;
	int err;

	err = regmap_read(ufs->regmap_sys, HSI2_KDN_CONTROL_MONITOR, &val);
	if (err) {
		dev_err(ufs->dev,
			"failed to read HSI2_KDN_CONTROL_MONITOR; err=%d\n",
			err);
		return;
	}
	WARN((val & (MKE_MONITOR | DT_MONITOR)) != MKE_MONITOR,
	     "unexpected KDN status in HSI2_KDN_CONTROL_MONITOR: 0x%08x\n",
	     val);
}

/*
 * Configure the UFS inline encryption hardware in the way we'd like to use it:
 *
 * - MKE=1: KDN / master keys enabled.  I.e. keys are "wrapped keys" and
 *   provided to the UFS controller / FMP indirectly via the KDN, as opposed to
 *   passing raw keys directly to the UFS controller / FMP.
 *
 * - DT=0: Keyslot and IV are specified in PRDT entries using extra fields.
 *   Don't use the UTRD option, as it uses the wrong endianness and increment
 *   amount for IVs, and its UTRD struct still differs from the UFSHCI standard.
 *
 * - KE=0: KDF disabled, so the nonce field is unused.  The KDF can't be used
 *   yet because the Linux storage stack doesn't yet support hardware derivation
 *   of per-file keys, but rather uses the IV to distinguish different files.
 */
static int pixel_ufs_crypto_configure_hw(struct ufs_hba *hba)
{
	struct pixel_ufs *ufs = to_pixel_ufs(hba);
	unsigned long ret;
	int err;

	/*
	 * Call into GSA to set the desired KDN configuration bits: MKE=1, DT=0,
	 * KE=0.  (See above for explanation.)  Note: the UFS controller needs
	 * to be reset for it to recognize these new settings.  This is done
	 * later when ufshcd-core resets the controller before enabling it.
	 */
	err = gsa_kdn_set_operating_mode(ufs->gsa_dev,
					 KDN_SW_KDF_MODE,
					 KDN_UFS_DESCR_TYPE_PRDT);
	if (err) {
		dev_err(ufs->dev, "failed to configure KDN; err=%d\n", err);
		return -ENODEV;
	}
	exynos_check_crypto_hw(ufs->hba);
	dev_info(ufs->dev, "configured KDN with MKE=1, DT=0, KE=0\n");

	/*
	 * This call (which sets DESCTYPE to 0x3 in the FMPSECURITY0 register)
	 * is needed to make the hardware use the larger PRDT entry size.
	 */
	ret = exynos_smc(SMC_CMD_FMP_SECURITY, 0, SMU_EMBEDDED, CFG_DESCTYPE_3);
	if (ret) {
		dev_err(ufs->dev,
			"SMC_CMD_FMP_SECURITY failed on init; ret=%lu\n", ret);
		return -EINVAL;
	}

	/*
	 * This SMC call to initialize FMP was in the original FMP code.  It
	 * seems to still be necessary; if it's omitted, errors occur when
	 * inline encryption is used.
	 */
	ret = exynos_smc(SMC_CMD_SMU, SMU_INIT, SMU_EMBEDDED, 0);
	if (ret) {
		dev_err(ufs->dev, "SMC_CMD_SMU(SMU_INIT) failed; ret=%lu\n",
			ret);
		return -EINVAL;
	}

	return 0;
}

void pixel_ufs_crypto_resume(struct ufs_hba *hba)
{
	struct pixel_ufs *ufs = to_pixel_ufs(hba);
	unsigned long ret;

	if (!(hba->caps & UFSHCD_CAP_CRYPTO))
		return;

	ret = exynos_smc(SMC_CMD_FMP_SECURITY, 0, SMU_EMBEDDED, CFG_DESCTYPE_3);
	if (ret)
		dev_err(ufs->dev,
			"SMC_CMD_FMP_SECURITY failed on resume; ret=%lu\n",
			ret);

	ret = exynos_smc(SMC_CMD_FMP_SMU_RESUME, 0, SMU_EMBEDDED, 0);
	if (ret)
		dev_err(ufs->dev, "SMC_CMD_FMP_SMU_RESUME failed; ret=%lu\n",
			ret);
}

/* Configure inline encryption (or decryption) on requests that require it. */
static void pixel_ufs_crypto_fill_prdt(void *unused, struct ufs_hba *hba,
				       struct ufshcd_lrb *lrbp,
				       unsigned int segments, int *err)
{
	struct pixel_ufs_prdt_entry *prdt =
		(struct pixel_ufs_prdt_entry *)lrbp->ucd_prdt_ptr;
	unsigned int i;

	/*
	 * There's nothing to do for unencrypted requests, since the "crypto
	 * enable" bit is already 0 by default, as it's in the same word as
	 * ufshcd_sg_entry::size which was already initialized.
	 */
	if (lrbp->crypto_key_slot < 0)
		return;

	/* Configure encryption on each segment of the request. */
	for (i = 0; i < segments; i++) {
		struct pixel_ufs_prdt_entry *ent = &prdt[i];
		struct ufshcd_sg_entry *prd = (struct ufshcd_sg_entry *)ent;

		/* Each segment must be exactly one data unit. */
		if (le32_to_cpu(prd->size) + 1 != CRYPTO_DATA_UNIT_SIZE) {
			dev_err(hba->dev,
				"scatterlist segment is misaligned for crypto\n");
			*err = -EIO;
			return;
		}

		/* Enable crypto and set the keyslot. */
		ent->des3 |= cpu_to_le32(CRYPTO_ENABLE |
					 CRYPTO_KEYSLOT(lrbp->crypto_key_slot));

		/*
		 * Set the IV.  The DUN is *supposed* to be formatted as a
		 * little endian integer to produce the 16-byte AES-XTS IV, like
		 * it is in the UFS standard.  But this hardware interprets the
		 * IV bytes backwards.  Therefore, we actually need to format
		 * the DUN as big endian to get the right ciphertext at the end.
		 */
		ent->iv[0] = 0;
		ent->iv[1] = cpu_to_be64(lrbp->data_unit_num + i);
	}

	/*
	 * Unset the keyslot in the ufshcd_lrb so that the keyslot and DUN don't
	 * get filled into the UTRD according to the UFSHCI standard.
	 */
	lrbp->crypto_key_slot = -1;
}

static int pixel_ufs_register_fill_prdt(void)
{
	return register_trace_android_vh_ufs_fill_prdt(
				pixel_ufs_crypto_fill_prdt, NULL);
}

static void pixel_ufs_ise_self_test(void *data, struct ufs_hba *hba)
{
	/*
	 * This SMC call sets USEOTPKEY bit to 1 in FMPSECURITY0 register. This
	 * causes incoming encryption keys to be XOR'ed with EFUSE key per
	 * section 1.4.3.3 of UFS Link Manual, a functionality needed by the UFS
	 * CMVP self test.
	 */
	if (exynos_smc(SMC_CMD_FMP_USE_OTP_KEY, 0, SMU_EMBEDDED, 1))
		panic("SMC_CMD_FMP_USE_OTP_KEY(0) failed");

	if (ufs_pixel_fips_verify(hba))
		panic("FMP self test failed");

	/*
	 * This SMC call sets USEOTPKEY bit back to 0 in FMPSECURITY0 register.
	 */
	if (exynos_smc(SMC_CMD_FMP_USE_OTP_KEY, 0, SMU_EMBEDDED, 0))
		panic("SMC_CMD_FMP_USE_OTP_KEY(1) failed");
}

static int pixel_ufs_register_fips_self_test(void)
{
	return register_trace_android_rvh_ufs_complete_init(
		pixel_ufs_ise_self_test, NULL);
}

static int exynos_crypto_init(struct ufs_hba *hba)
{
	int err;
	bool register_fips_module;

	/* Override the PRDT entry size to include the extra crypto fields. */
	hba->sg_entry_size = sizeof(struct pixel_ufs_prdt_entry);

	err = pixel_ufs_crypto_configure_hw(hba);
	if (err)
		return err;

	/*
	 * GS101 FIPS 140 module does not support HW delivered keys. Do not
	 * register the module on GS101.
	 */
#if IS_ENABLED(CONFIG_SCSI_UFS_PIXEL_FIPS140)
	register_fips_module = true;
#else
	register_fips_module = false;
#endif

	/* The FIPS 140 module will register internal fill prdt function. */
	if (register_fips_module) {
		err = pixel_ufs_register_fips_self_test();
	} else {
		err = pixel_ufs_register_fill_prdt();
	}

	return err;
}

const struct pixel_crypto_ops exynos_crypto_ops = {
	.crypto_init = exynos_crypto_init,
};
