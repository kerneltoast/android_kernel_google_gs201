// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Pixel UFS inline encryption support
 *
 * Copyright 2020 Google LLC
 *
 * UFS inline encryption support using FMP (Flash Memory Protector).
 * Two operating modes are supported:
 *
 * - Hardware keys mode, also called KDN mode.  In this mode, there are a
 *   certain number of keyslots, like there are in the UFS standard crypto.
 *   However, unlike the UFS standard crypto, all keys are hardware-wrapped keys
 *   rather than raw keys.  The keys are delivered to FMP indirectly via the KDN
 *   (Key Distribution Network) and GSA (Google Security Anchor) rather than via
 *   writes to UFS registers.  The way the keyslot and IV of each request are
 *   passed to the UFS controller also differs from the UFS standard.
 *
 * - Software keys mode, also called the traditional FMP mode or legacy FMP
 *   mode.  In this mode, software specifies the raw keys to use, similar to the
 *   UFS standard crypto.  However, the way the keys and IVs are passed to the
 *   UFS controller still differs from the UFS standard. This mode must be enabled
 *   via a Kconfig option.
 *
 * These two modes are not compatible with each other, and the mode to use is
 * set at module load time by the "use_kdn" module parameter.  Upper layers in
 * the storage stack must be configured to use the appropriate type of keys when
 * the mode is changed; otherwise inline encryption won't be able to be used.
 */

#include <linux/gsa/gsa_kdn.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <ufs/ufshcd.h>
#include <core/ufshcd-crypto.h>

#include "ufs-pixel.h"
#include "ufs-pixel-fips.h"
#include "ufs-pixel-crypto.h"

#undef CREATE_TRACE_POINTS
#include <trace/hooks/ufshcd.h>

#if IS_ENABLED(CONFIG_SCSI_UFS_CRYPTO_SW_KEYS_MODE)
static bool use_kdn = true;
module_param(use_kdn, bool, 0444);
MODULE_PARM_DESC(use_kdn, "Use hardware keys mode (KDN mode) for inline crypto");
#endif

static void pixel_ufs_crypto_restore_keys(void *unused, struct ufs_hba *hba,
					  int *err);

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
 * Determine the number of pending commands by counting the bits in the SCSI
 * device budget maps.
 */
static u32 ufshcd_pending_cmds(struct ufs_hba *hba)
{
	struct scsi_device *sdev;
	u32 pending = 0;

	shost_for_each_device(sdev, hba->host)
		pending += sbitmap_weight(&sdev->budget_map);

	return pending;
}

/*
 * Block new UFS requests from being issued, and wait for any outstanding UFS
 * requests to complete. Must be paired with ufshcd_resume_io().
 */
static void ufshcd_block_io(struct ufs_hba *hba)
{
	ktime_t deadline = ktime_add_ms(ktime_get(), 5 * 1000);
	struct scsi_device *sdev;

	/*
	 * If ufshcd_block_io() is called before the tag set has been
	 * initialized then we know that no I/O is ongoing.
	 */
	if (!hba->host->tag_set.tags)
		return;

	shost_for_each_device(sdev, hba->host)
		blk_mq_quiesce_queue(sdev->request_queue);

	while (ufshcd_pending_cmds(hba)) {
		if (ktime_after(ktime_get(), deadline)) {
			dev_err(hba->dev, "%s waited too long\n", __func__);
			return;
		}
		io_schedule_timeout(msecs_to_jiffies(20));
	}
}

static void ufshcd_resume_io(struct ufs_hba *hba)
{
	struct scsi_device *sdev;

	if (!hba->host->tag_set.tags)
		return;

	shost_for_each_device(sdev, hba->host)
		blk_mq_unquiesce_queue(sdev->request_queue);
}

static int pixel_ufs_keyslot_program(struct blk_crypto_profile *profile,
				     const struct blk_crypto_key *key,
				     unsigned int slot)
{
	struct ufs_hba *hba = container_of(profile,
					struct ufs_hba, crypto_profile);
	struct pixel_ufs *ufs = to_pixel_ufs(hba);
	int err;

	dev_info(ufs->dev,
		 "kdn: programming keyslot %u with %u-byte wrapped key\n",
		 slot, key->size);

	/*
	 * This hardware doesn't allow any encrypted I/O at all while a keyslot
	 * is being modified.
	 */
	ufshcd_block_io(hba);

	err = gsa_kdn_program_key(ufs->gsa_dev, slot, key->raw, key->size);
	if (err)
		dev_err(ufs->dev, "kdn: failed to program key; err=%d\n", err);

	ufshcd_resume_io(hba);

	return err;
}

static int pixel_ufs_keyslot_evict(struct blk_crypto_profile *profile,
				   const struct blk_crypto_key *key,
				   unsigned int slot)
{
	struct ufs_hba *hba = container_of(profile,
					struct ufs_hba, crypto_profile);
	struct pixel_ufs *ufs = to_pixel_ufs(hba);
	int err;

	dev_info(ufs->dev, "kdn: evicting keyslot %u\n", slot);

	/*
	 * This hardware doesn't allow any encrypted I/O at all while a keyslot
	 * is being modified.
	 */
	ufshcd_block_io(hba);

	err = gsa_kdn_program_key(ufs->gsa_dev, slot, NULL, 0);
	if (err)
		dev_err(ufs->dev, "kdn: failed to evict key; err=%d\n", err);

	ufshcd_resume_io(hba);

	return err;
}

static int pixel_ufs_derive_sw_secret(struct blk_crypto_profile *profile,
				       const u8 *eph_key, size_t eph_key_size,
				       u8 sw_secret[BLK_CRYPTO_SW_SECRET_SIZE])
{
	struct ufs_hba *hba = container_of(profile,
					struct ufs_hba, crypto_profile);
	struct pixel_ufs *ufs = to_pixel_ufs(hba);
	int ret;

	dev_info(ufs->dev,
		 "kdn: deriving %u-byte raw secret from %zu-byte wrapped key\n",
		 BLK_CRYPTO_SW_SECRET_SIZE, eph_key_size);

	ret = gsa_kdn_derive_raw_secret(ufs->gsa_dev, sw_secret,
					BLK_CRYPTO_SW_SECRET_SIZE,
					eph_key, eph_key_size);
	if (ret != BLK_CRYPTO_SW_SECRET_SIZE) {
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

static const struct blk_crypto_ll_ops pixel_ufs_crypto_ops = {
	.keyslot_program	= pixel_ufs_keyslot_program,
	.keyslot_evict		= pixel_ufs_keyslot_evict,
	.derive_sw_secret	= pixel_ufs_derive_sw_secret,
};

static void pixel_ufs_release_gsa_device(void *_ufs)
{
	struct pixel_ufs *ufs = _ufs;

	put_device(ufs->gsa_dev);
}

/*
 * Get the GSA device from the device tree and save a pointer to it in the UFS
 * host struct.
 */
static int pixel_ufs_find_gsa_device(struct pixel_ufs *ufs)
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

static int pixel_ufs_crypto_init_hw_keys_mode(struct ufs_hba *hba)
{
	struct pixel_ufs *ufs = to_pixel_ufs(hba);
	int err;

	err = pixel_ufs_find_gsa_device(ufs);
	if (err == -ENODEV)
		goto disable;
	if (err)
		return err;

	if (ufs->crypto_ops->crypto_init)
		err = ufs->crypto_ops->crypto_init(hba);
	if (err == -ENODEV)
		goto disable;
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
	 * override the UFSHCI standand blk_crypto_ll_ops with operations that
	 * program/evict wrapped keys via the KDN, and secondly in order to
	 * declare wrapped key support rather than standard key support.
	 */
	hba->android_quirks |= UFSHCD_ANDROID_QUIRK_CUSTOM_CRYPTO_PROFILE;

	/*
	 * This host controller doesn't support the standard
	 * CRYPTO_GENERAL_ENABLE bit in REG_CONTROLLER_ENABLE.  Instead it just
	 * always has crypto support enabled.
	 */
	hba->android_quirks |= UFSHCD_ANDROID_QUIRK_BROKEN_CRYPTO_ENABLE;

	/* Advertise crypto capabilities to the block layer. */
	err = devm_blk_crypto_profile_init(hba->dev, &hba->crypto_profile,
								KDN_SLOT_NUM);
	if (err)
		return err;
	hba->crypto_profile.ll_ops = pixel_ufs_crypto_ops;
	/*
	 * The PRDT entries accept 16-byte IVs, but currently the driver passes
	 * the DUN through ufshcd_lrb::data_unit_num which is 8-byte.  8 bytes
	 * is enough for upper layers, so for now just use that as the limit.
	 */
	hba->crypto_profile.max_dun_bytes_supported = 8;
	hba->crypto_profile.key_types_supported = BLK_CRYPTO_KEY_TYPE_HW_WRAPPED;
	hba->crypto_profile.dev = ufs->dev;
	hba->crypto_profile.modes_supported[BLK_ENCRYPTION_MODE_AES_256_XTS] =
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

/* Initialize UFS inline encryption support. */
int pixel_ufs_crypto_init(struct ufs_hba *hba)
{
#if IS_ENABLED(CONFIG_SCSI_UFS_CRYPTO_SW_KEYS_MODE)
	if (!use_kdn)
		return exynos_ufs_crypto_init_sw_keys_mode(hba);
#endif
	return pixel_ufs_crypto_init_hw_keys_mode(hba);
}

static void pixel_ufs_crypto_restore_keys(void *unused, struct ufs_hba *hba,
					  int *err)
{
	struct pixel_ufs *ufs = to_pixel_ufs(hba);

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
