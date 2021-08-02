// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Pixel-specific UFS inline encryption support using FMP (Flash Memory
 * Protector) and the KDN (Key Distribution Network)
 *
 * Copyright 2020 Google LLC
 */

#include <linux/gsa/gsa_kdn.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/soc/samsung/exynos-smc.h>

#include "ufshcd.h"
#include "ufshcd-crypto.h"
#include "ufs-exynos.h"

#undef CREATE_TRACE_POINTS
#include <trace/hooks/ufshcd.h>

static void pixel_ufs_crypto_restore_keys(void *unused, struct ufs_hba *hba,
					  int *err);

static void pixel_ufs_crypto_fill_prdt(void *unused, struct ufs_hba *hba,
				       struct ufshcd_lrb *lrbp,
				       unsigned int segments, int *err);

#define CRYPTO_DATA_UNIT_SIZE 4096

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
 * Block new UFS requests from being issued, and wait for any outstanding UFS
 * requests to complete.   Modified from ufshcd_clock_scaling_prepare().
 * Must be paired with ufshcd_put_exclusive_access().
 */
static void ufshcd_get_exclusive_access(struct ufs_hba *hba)
{
	#define DOORBELL_CLR_WARN_US		(5 * 1000 * 1000) /* 5 secs */
	#define	DEFAULT_IO_TIMEOUT		(msecs_to_jiffies(20))
	u32 tm_doorbell;
	u32 tr_doorbell;
	ktime_t start;
	unsigned long flags;

	if (atomic_inc_return(&hba->scsi_block_reqs_cnt) == 1)
		scsi_block_requests(hba->host);

	down_write(&hba->clk_scaling_lock);

	ufshcd_hold(hba, false);
	spin_lock_irqsave(hba->host->host_lock, flags);
	start = ktime_get();
	do {
		tm_doorbell = ufshcd_readl(hba, REG_UTP_TASK_REQ_DOOR_BELL);
		tr_doorbell = ufshcd_readl(hba, REG_UTP_TRANSFER_REQ_DOOR_BELL);
		if (!tm_doorbell && !tr_doorbell)
			break;

		spin_unlock_irqrestore(hba->host->host_lock, flags);
		io_schedule_timeout(DEFAULT_IO_TIMEOUT);
		if (ktime_to_us(ktime_sub(ktime_get(), start)) >
					DOORBELL_CLR_WARN_US) {
			start = ktime_get();
			dev_err(hba->dev,
				"%s: warning: waiting too much for doorbell to clear (tm=0x%x, tr=0x%x)\n",
				__func__, tm_doorbell, tr_doorbell);
		}
		spin_lock_irqsave(hba->host->host_lock, flags);
	} while (tm_doorbell || tr_doorbell);

	spin_unlock_irqrestore(hba->host->host_lock, flags);
	ufshcd_release(hba);
}

static void ufshcd_put_exclusive_access(struct ufs_hba *hba)
{
	up_write(&hba->clk_scaling_lock);
	if (atomic_dec_and_test(&hba->scsi_block_reqs_cnt))
		scsi_unblock_requests(hba->host);
}

static int pixel_ufs_keyslot_program(struct blk_keyslot_manager *ksm,
				     const struct blk_crypto_key *key,
				     unsigned int slot)
{
	struct ufs_hba *hba = container_of(ksm, struct ufs_hba, ksm);
	struct exynos_ufs *ufs = to_exynos_ufs(hba);
	int err;

	dev_info(ufs->dev,
		 "kdn: programming keyslot %u with %u-byte wrapped key\n",
		 slot, key->size);

	/*
	 * This hardware doesn't allow any encrypted I/O at all while a keyslot
	 * is being modified.
	 */
	ufshcd_get_exclusive_access(hba);

	err = gsa_kdn_program_key(ufs->gsa_dev, slot, key->raw, key->size);
	if (err)
		dev_err(ufs->dev, "kdn: failed to program key; err=%d\n", err);

	ufshcd_put_exclusive_access(hba);

	return err;
}

static int pixel_ufs_keyslot_evict(struct blk_keyslot_manager *ksm,
				   const struct blk_crypto_key *key,
				   unsigned int slot)
{
	struct ufs_hba *hba = container_of(ksm, struct ufs_hba, ksm);
	struct exynos_ufs *ufs = to_exynos_ufs(hba);
	int err;

	dev_info(ufs->dev, "kdn: evicting keyslot %u\n", slot);

	/*
	 * This hardware doesn't allow any encrypted I/O at all while a keyslot
	 * is being modified.
	 */
	ufshcd_get_exclusive_access(hba);

	err = gsa_kdn_program_key(ufs->gsa_dev, slot, NULL, 0);
	if (err)
		dev_err(ufs->dev, "kdn: failed to evict key; err=%d\n", err);

	ufshcd_put_exclusive_access(hba);

	return err;
}

static int pixel_ufs_derive_raw_secret(struct blk_keyslot_manager *ksm,
				       const u8 *wrapped_key,
				       unsigned int wrapped_key_size,
				       u8 *secret, unsigned int secret_size)
{
	struct ufs_hba *hba = container_of(ksm, struct ufs_hba, ksm);
	struct exynos_ufs *ufs = to_exynos_ufs(hba);
	int ret;

	dev_info(ufs->dev,
		 "kdn: deriving %u-byte raw secret from %u-byte wrapped key\n",
		 secret_size, wrapped_key_size);

	ret = gsa_kdn_derive_raw_secret(ufs->gsa_dev, secret, secret_size,
					wrapped_key, wrapped_key_size);
	if (ret != secret_size) {
		dev_err(ufs->dev, "kdn: failed to derive raw secret; ret=%d\n",
			ret);
		/*
		 * gsa_kdn_derive_raw_secret() returns -EIO on "bad key" but
		 * upper layers expect -EINVAL.  Just always return -EINVAL.
		 */
		return -EINVAL;
	}
	return 0;
}

static const struct blk_ksm_ll_ops pixel_ufs_ksm_ops = {
	.keyslot_program	= pixel_ufs_keyslot_program,
	.keyslot_evict		= pixel_ufs_keyslot_evict,
	.derive_raw_secret	= pixel_ufs_derive_raw_secret,
};

static void pixel_ufs_release_gsa_device(void *_ufs)
{
	struct exynos_ufs *ufs = _ufs;

	put_device(ufs->gsa_dev);
}

/*
 * Get the GSA device from the device tree and save a pointer to it in the UFS
 * host struct.
 */
static int pixel_ufs_find_gsa_device(struct exynos_ufs *ufs)
{
	struct device_node *np;
	struct platform_device *gsa_pdev;

	np = of_parse_phandle(ufs->dev->of_node, "gsa-device", 0);
	if (!np) {
		dev_warn(ufs->dev,
			 "gsa-device phandle not found in UFS device tree node\n");
		return -ENODEV;
	}
	gsa_pdev = of_find_device_by_node(np);
	of_node_put(np);

	if (!gsa_pdev) {
		dev_err(ufs->dev,
			"gsa-device phandle doesn't refer to a device\n");
		return -ENODEV;
	}
	ufs->gsa_dev = &gsa_pdev->dev;
	return devm_add_action_or_reset(ufs->dev, pixel_ufs_release_gsa_device,
					ufs);
}

#define HSI2_KDN_CONTROL_MONITOR	0x400	/* offset from HSI2 base */
#define MKE_MONITOR			BIT(0)	/* Master Key Enable */
#define DT_MONITOR			BIT(1)	/* Descriptor Type */
#define RDY_MONITOR			BIT(2)	/* KDN ready? */

/*
 * Read the HSI2_KDN_CONTROL_MONITOR register to verify that the KDN is
 * configured correctly.
 *
 * Note that the KE (KDF Enable) bit isn't shown by the register, as it is
 * actually a per-keyslot thing.  So we can't verify KE=0 here.
 */
static void pixel_ufs_crypto_check_hw(struct exynos_ufs *ufs)
{
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
static int pixel_ufs_crypto_configure_hw(struct exynos_ufs *ufs)
{
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
	pixel_ufs_crypto_check_hw(ufs);
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

/* Initialize UFS inline encryption support. */
int pixel_ufs_crypto_init(struct ufs_hba *hba)
{
	struct exynos_ufs *ufs = to_exynos_ufs(hba);
	int err;

	err = pixel_ufs_find_gsa_device(ufs);
	if (err == -ENODEV)
		goto disable;
	if (err)
		return err;

	err = pixel_ufs_crypto_configure_hw(ufs);
	if (err == -ENODEV)
		goto disable;
	if (err)
		return err;

	err = register_trace_android_vh_ufs_fill_prdt(
				pixel_ufs_crypto_fill_prdt, NULL);
	if (err)
		return err;

	err = register_trace_android_rvh_ufs_reprogram_all_keys(
				pixel_ufs_crypto_restore_keys, NULL);
	if (err)
		return err;

	/* Advertise crypto support to ufshcd-core. */
	hba->caps |= UFSHCD_CAP_CRYPTO;

	/* Advertise crypto quirks to ufshcd-core. */

	/*
	 * We need to override the blk_keyslot_manager, firstly in order to
	 * override the UFSHCI standand blk_ksm_ll_ops with operations that
	 * program/evict wrapped keys via the KDN, and secondly in order to
	 * declare wrapped key support rather than standard key support.
	 */
	hba->quirks |= UFSHCD_QUIRK_CUSTOM_KEYSLOT_MANAGER;

	/*
	 * This host controller doesn't support the standard
	 * CRYPTO_GENERAL_ENABLE bit in REG_CONTROLLER_ENABLE.  Instead it just
	 * always has crypto support enabled.
	 */
	hba->quirks |= UFSHCD_QUIRK_BROKEN_CRYPTO_ENABLE;

	/* Override the PRDT entry size to include the extra crypto fields. */
	hba->sg_entry_size = sizeof(struct pixel_ufs_prdt_entry);

	/* Advertise crypto capabilities to the block layer. */
	err = devm_blk_ksm_init(hba->dev, &hba->ksm, KDN_SLOT_NUM);
	if (err)
		return err;
	hba->ksm.ksm_ll_ops = pixel_ufs_ksm_ops;
	/*
	 * The PRDT entries accept 16-byte IVs, but currently the driver passes
	 * the DUN through ufshcd_lrb::data_unit_num which is 8-byte.  8 bytes
	 * is enough for upper layers, so for now just use that as the limit.
	 */
	hba->ksm.max_dun_bytes_supported = 8;
	hba->ksm.features = BLK_CRYPTO_FEATURE_WRAPPED_KEYS;
	hba->ksm.dev = ufs->dev;
	hba->ksm.crypto_modes_supported[BLK_ENCRYPTION_MODE_AES_256_XTS] =
		CRYPTO_DATA_UNIT_SIZE;

	dev_info(ufs->dev,
		 "enabled inline encryption support with wrapped keys\n");
	return 0;

disable:
	/*
	 * If the GSA support for wrapped keys seems to be missing, then fall
	 * back to disabling crypto support and continuing with driver probe.
	 * Attempts to use wrapped keys will fail, but any other use of UFS will
	 * continue to work.
	 */
	dev_warn(hba->dev, "disabling inline encryption support\n");
	hba->caps &= ~UFSHCD_CAP_CRYPTO;
	return 0;
}

static void pixel_ufs_crypto_restore_keys(void *unused, struct ufs_hba *hba,
					  int *err)
{
	struct exynos_ufs *ufs = to_exynos_ufs(hba);

	/*
	 * GSA provides a function to restore all keys which is faster than
	 * programming all keys individually, so use it in order to avoid
	 * unnecessary resume latency.
	 *
	 * GSA also relies on this function being called in order to configure
	 * some hardening against power analysis attacks.
	 */
	dev_info(ufs->dev, "kdn: restoring keys\n");
	*err = gsa_kdn_restore_keys(ufs->gsa_dev);
	if (*err)
		dev_err(ufs->dev, "kdn: failed to restore keys; err=%d\n",
			*err);
}

void pixel_ufs_crypto_resume(struct ufs_hba *hba)
{
	struct exynos_ufs *ufs = to_exynos_ufs(hba);
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
