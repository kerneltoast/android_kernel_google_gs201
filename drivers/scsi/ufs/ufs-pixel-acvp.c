// SPDX-License-Identifier: GPL-2.0-only
/*
 * Pixel-Specific UFS feature support
 *
 * Copyright 2021 Google LLC
 *
 * Authors: Konstantin Vyshetsky <vkon@google.com>
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/bio.h>
#include <linux/blkdev.h>
#include <linux/blk-crypto.h>
#include <linux/miscdevice.h>
#include <linux/vmalloc.h>
#include "ufs-pixel-acvp.h"
#include "ufs-pixel-fips_sha256.h"

#undef pr_fmt
#define pr_fmt(fmt) "ufs-pixel-acvp: " fmt

#define UFS_PIXEL_ACVP_PARTITION		"/dev/block/by-name/fips"
#define UFS_PIXEL_ACVP_LBA			0
#define UFS_PIXEL_ACVP_NAME			"fmp_acvp"
#define UFS_PIXEL_ACVP_AES_256_XTS_KEY_LEN	64
#define UFS_PIXEL_ACVP_AES_256_XTS_IV_LEN	16
#define UFS_PIXEL_ACVP_AES_MAX_INPUT_LEN	4096
#define UFS_PIXEL_ACVP_AES_MIN_ALIGN		16
#define UFS_PIXEL_ACVP_HASH_MAX_INPUT_LEN	65536
#define UFS_PIXEL_ACVP_HASH_MAX_KEY_LEN		4096

struct acvp_crypto_ctx {
	struct blk_crypto_key bc_key;
	u8 raw_key[UFS_PIXEL_ACVP_AES_256_XTS_KEY_LEN];
	u64 bc_dun[BLK_CRYPTO_DUN_ARRAY_SIZE];
};

static struct {
	struct miscdevice miscdev;
	struct block_device *bdev;
	struct page *io_page;
	struct page *crypto_page;
	void *io_addr;
	struct acvp_crypto_ctx *acvp_crypto_ctx;
} acvp_ctx;

struct aes_req_int {
	void __user *src_ptr;
	void __user *dst_ptr;
	void __user *key_ptr;
	void __user *iv_ptr;
	__u32 src_len;
	__u32 dst_len;
	__u32 key_len;
	__u32 iv_len;
	__u32 aes_direction;
	__u32 aes_algorithm;
};

struct hash_req_int {
	void __user *src_ptr;
	void __user *dst_ptr;
	void __user *key_ptr;
	__u32 src_len;
	__u32 key_len;
	__u32 hash_type;
};

static int ufs_pixel_acvp_get_aes_req(unsigned long src,
				       struct aes_req_int *aes_req_int)
{
	struct ufs_pixel_acvp_aes_req aes_req;

	if (unlikely(copy_from_user(&aes_req, (void __user *)src,
				    sizeof(aes_req))))
		return -EFAULT;

	aes_req_int->src_ptr = u64_to_user_ptr(aes_req.src_ptr);
	aes_req_int->dst_ptr = u64_to_user_ptr(aes_req.dst_ptr);
	aes_req_int->key_ptr = u64_to_user_ptr(aes_req.key_ptr);
	aes_req_int->iv_ptr = u64_to_user_ptr(aes_req.iv_ptr);
	aes_req_int->src_len = aes_req.src_len;
	aes_req_int->dst_len = aes_req.dst_len;
	aes_req_int->key_len = aes_req.key_len;
	aes_req_int->iv_len = aes_req.iv_len;
	aes_req_int->aes_direction = aes_req.aes_direction;
	aes_req_int->aes_algorithm = aes_req.aes_algorithm;

	return 0;
}

static int ufs_pixel_acvp_get_hash_req(unsigned long src,
				       struct hash_req_int *hash_req_int)
{
	struct ufs_pixel_acvp_hash_req hash_req;

	if (unlikely(copy_from_user(&hash_req, (void __user *)src,
				    sizeof(hash_req))))
		return -EFAULT;

	hash_req_int->src_ptr = u64_to_user_ptr(hash_req.src_ptr);
	hash_req_int->dst_ptr = u64_to_user_ptr(hash_req.dst_ptr);
	hash_req_int->key_ptr = u64_to_user_ptr(hash_req.key_ptr);
	hash_req_int->src_len = hash_req.src_len;
	hash_req_int->key_len = hash_req.key_len;
	hash_req_int->hash_type = hash_req.hash_type;

	return 0;
}

static int ufs_pixel_acvp_send_io(struct page *page, unsigned int bi_opf,
				  struct acvp_crypto_ctx *crypto_ctx)
{
	struct bio *bio;
	int ret;

	bio = bio_alloc(GFP_KERNEL, 1);

	bio_set_dev(bio, acvp_ctx.bdev);
	bio->bi_iter.bi_sector = UFS_PIXEL_ACVP_LBA;
	bio->bi_opf = bi_opf;
	bio_add_page(bio, page, PAGE_SIZE, 0);

	if (crypto_ctx)
		bio_crypt_set_ctx(bio, &crypto_ctx->bc_key, crypto_ctx->bc_dun,
				  GFP_KERNEL);

	ret = submit_bio_wait(bio);
	bio_put(bio);

	return ret;
}

static int ufs_pixel_acvp_write(struct page *page,
				struct acvp_crypto_ctx *crypto_ctx)
{
	return ufs_pixel_acvp_send_io(page, REQ_OP_WRITE | REQ_FUA, crypto_ctx);
}

static int ufs_pixel_acvp_read(struct page *page,
			       struct acvp_crypto_ctx *crypto_ctx)
{
	return ufs_pixel_acvp_send_io(page, REQ_OP_READ, crypto_ctx);
}

static int ufs_pixel_acvp_handle_aes_req(struct aes_req_int *aes_req)
{
	int ret;

	/*
	* To align with current kernel capabilities we only support
	* AES-256-XTS
	*/
	if (aes_req->aes_algorithm != AES_XTS) {
		pr_err("Invalid AES algorithm\n");
		return -EINVAL;
	}

	if (aes_req->key_len != UFS_PIXEL_ACVP_AES_256_XTS_KEY_LEN) {
		pr_err("Invalid AES-256-XTS key len (%u)\n", aes_req->key_len);
		return -EINVAL;
	}
	BUILD_BUG_ON(UFS_PIXEL_ACVP_AES_256_XTS_KEY_LEN > BLK_CRYPTO_MAX_KEY_SIZE);

	if (aes_req->iv_len != UFS_PIXEL_ACVP_AES_256_XTS_IV_LEN) {
		pr_err("Invalid AES-256-XTS iv len (%u)\n", aes_req->iv_len);
		return -EINVAL;
	}
	BUILD_BUG_ON(UFS_PIXEL_ACVP_AES_256_XTS_IV_LEN > sizeof(acvp_ctx.acvp_crypto_ctx->bc_dun));

	if (aes_req->src_len > UFS_PIXEL_ACVP_AES_MAX_INPUT_LEN) {
		pr_err("Invalid AES-256-XTS input len (%u)\n",
		       aes_req->src_len);
		return -EINVAL;
	}

	if (aes_req->src_len % UFS_PIXEL_ACVP_AES_MIN_ALIGN) {
		pr_err("Invalid AES-256-XTS input len alignment (%u)\n",
		       aes_req->src_len);
		return -EINVAL;
	}

	if (aes_req->dst_len != aes_req->src_len) {
		pr_err("Invalid AES-256-XTS output len (%u)\n",
		       aes_req->dst_len);
		return -EINVAL;
	}

	if (aes_req->aes_direction >= ACVP_AES_DIRECTION_MAX) {
		pr_err("Invalid AES direction (%d)\n", aes_req->aes_direction);
		return -EINVAL;
	}

	memset(acvp_ctx.acvp_crypto_ctx->bc_dun, 0,
	       sizeof(acvp_ctx.acvp_crypto_ctx->bc_dun));

	if (copy_from_user(acvp_ctx.io_addr, aes_req->src_ptr,
			   aes_req->src_len) ||
	    copy_from_user(acvp_ctx.acvp_crypto_ctx->raw_key,
			   aes_req->key_ptr, aes_req->key_len) ||
	    copy_from_user(acvp_ctx.acvp_crypto_ctx->bc_dun, aes_req->iv_ptr,
			   aes_req->iv_len))
		return -EFAULT;

	ret = blk_crypto_init_key(&acvp_ctx.acvp_crypto_ctx->bc_key,
				  acvp_ctx.acvp_crypto_ctx->raw_key,
				  aes_req->key_len,
				  false,
				  BLK_ENCRYPTION_MODE_AES_256_XTS,
				  UFS_PIXEL_ACVP_AES_256_XTS_IV_LEN,
				  UFS_PIXEL_ACVP_AES_MAX_INPUT_LEN);
	if (ret) {
		pr_err("Failed blk_crypto_init_key (%d)\n", ret);
		return ret;
	}

	if (aes_req->aes_direction == ACVP_ENCRYPT) {
		if (ufs_pixel_acvp_write(acvp_ctx.io_page,
					 acvp_ctx.acvp_crypto_ctx))
			return -EIO;
		if (ufs_pixel_acvp_read(acvp_ctx.io_page, NULL))
			return -EIO;
	} else {
		if (ufs_pixel_acvp_write(acvp_ctx.io_page, NULL))
			return -EIO;
		if (ufs_pixel_acvp_read(acvp_ctx.io_page,
					acvp_ctx.acvp_crypto_ctx))
			return -EIO;
	}

	if (copy_to_user(aes_req->dst_ptr, acvp_ctx.io_addr,
			 aes_req->src_len))
		return -EFAULT;

	return 0;
}

static int ufs_pixel_acvp_handle_hash_req(struct hash_req_int *hash_req)
{
	u8 digest[UFS_PIXEL_FIPS_SHA256_DIGEST_SIZE];
	void *msg = NULL;
	void *key = NULL;
	int ret = 0;

	if (hash_req->src_len > UFS_PIXEL_ACVP_HASH_MAX_INPUT_LEN) {
		pr_err("Message length must be at most %u\n",
			UFS_PIXEL_ACVP_HASH_MAX_INPUT_LEN);
		return -EINVAL;
	}

	if (hash_req->src_len) {
		msg = vmemdup_user(hash_req->src_ptr, hash_req->src_len);
		if (IS_ERR(msg)) {
			ret = PTR_ERR(msg);
			msg = NULL;
			goto out;
		}
	}

	if (hash_req->hash_type == HMAC_SHA_256) {
		if (hash_req->key_len > UFS_PIXEL_ACVP_HASH_MAX_KEY_LEN) {
			pr_err("Key length must be at most %u\n",
				UFS_PIXEL_ACVP_HASH_MAX_KEY_LEN);
			ret = -EINVAL;
			goto out;
		}

		key = vmemdup_user(hash_req->key_ptr, hash_req->key_len);
		if (IS_ERR(key)) {
			ret = PTR_ERR(key);
			key = NULL;
			goto out;
		}

		ufs_pixel_fips_hmac_sha256(msg, hash_req->src_len, key,
				      hash_req->key_len, digest);
	} else if (hash_req->hash_type == SHA_256) {
		ufs_pixel_fips_sha256(msg, hash_req->src_len, digest);
	} else if (hash_req->hash_type == SHA_256_MCT) {
		u8 mc_digest[UFS_PIXEL_FIPS_SHA256_DIGEST_SIZE * 3];
		u8 *md_a = mc_digest;
		u8 *md_b = mc_digest + UFS_PIXEL_FIPS_SHA256_DIGEST_SIZE;
		u8 *md_c = mc_digest + UFS_PIXEL_FIPS_SHA256_DIGEST_SIZE * 2;
		u32 i;

		if (hash_req->src_len < UFS_PIXEL_FIPS_SHA256_DIGEST_SIZE) {
			pr_err("Message length must be at least %u\n",
			       UFS_PIXEL_FIPS_SHA256_DIGEST_SIZE);
			ret = -EINVAL;
			goto out;
		}

		memcpy(md_a, msg, UFS_PIXEL_FIPS_SHA256_DIGEST_SIZE);
		memcpy(md_b, msg, UFS_PIXEL_FIPS_SHA256_DIGEST_SIZE);
		memcpy(md_c, msg, UFS_PIXEL_FIPS_SHA256_DIGEST_SIZE);

		for (i = 0; i < 1000; i++) {
			ufs_pixel_fips_sha256(mc_digest,
					UFS_PIXEL_FIPS_SHA256_DIGEST_SIZE * 3,
					digest);
			memcpy(md_a, md_b, UFS_PIXEL_FIPS_SHA256_DIGEST_SIZE);
			memcpy(md_b, md_c, UFS_PIXEL_FIPS_SHA256_DIGEST_SIZE);
			memcpy(md_c, digest, UFS_PIXEL_FIPS_SHA256_DIGEST_SIZE);

			if (i && !(i % 64))
				cond_resched();
		}
	} else {
		pr_err("Unsupported hash type 0x%x\n", hash_req->hash_type);
		ret = -EINVAL;
		goto out;
	}

	if (copy_to_user(hash_req->dst_ptr, digest,
			 UFS_PIXEL_FIPS_SHA256_DIGEST_SIZE))
		ret = -EFAULT;

out:
	kvfree(msg);
	kvfree(key);

	return ret;
}

static long ufs_pixel_acvp_ioctl(struct file *file, unsigned int cmd,
				 unsigned long arg)
{
	struct aes_req_int aes_req;
	struct hash_req_int hash_req;
	int ret;

	switch (cmd) {
	case UFS_PIXEL_ACVP_AES_REQ:
		ret = ufs_pixel_acvp_get_aes_req(arg, &aes_req);
		if (ret)
			return ret;
		ret = ufs_pixel_acvp_handle_aes_req(&aes_req);
		break;
	case UFS_PIXEL_ACVP_HASH_REQ:
		ret = ufs_pixel_acvp_get_hash_req(arg, &hash_req);
		if (ret)
			return ret;
		ret = ufs_pixel_acvp_handle_hash_req(&hash_req);
		break;
	default:
		ret = -ENOTTY;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long ufs_pixel_acvp_compat_ioctl(struct file *file, unsigned int cmd,
					unsigned long arg)
{
	return ufs_pixel_acvp_ioctl(file, cmd, (unsigned long)compat_ptr(arg));
}
#endif

static const struct file_operations ufs_pixel_acvp_fops = {
	.owner			= THIS_MODULE,
	.unlocked_ioctl	= ufs_pixel_acvp_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl		= ufs_pixel_acvp_compat_ioctl,
#endif
};

static int ufs_pixel_acvp_init(void)
{
	int ret = 0;

	acvp_ctx.miscdev.minor = MISC_DYNAMIC_MINOR;
	acvp_ctx.miscdev.name = UFS_PIXEL_ACVP_NAME;
	acvp_ctx.miscdev.fops = &ufs_pixel_acvp_fops;

	ret = misc_register(&acvp_ctx.miscdev);
	if (ret) {
		pr_err("Failed to register misc device ret=%d\n", ret);
		goto out;
	}

	acvp_ctx.bdev = blkdev_get_by_path(UFS_PIXEL_ACVP_PARTITION,
				  FMODE_WRITE | FMODE_READ, NULL);
	if (IS_ERR(acvp_ctx.bdev)) {
		pr_err("Failed to open %s\n", UFS_PIXEL_ACVP_PARTITION);
		ret = -ENODEV;
		goto misc_deregister;
	}

	acvp_ctx.io_page = alloc_page(GFP_KERNEL);
	if (!acvp_ctx.io_page) {
		ret = -ENOMEM;
		goto blkdev_put;
	}

	acvp_ctx.io_addr = page_address(acvp_ctx.io_page);

	acvp_ctx.crypto_page = alloc_page(GFP_KERNEL);
	if (!acvp_ctx.crypto_page) {
		ret = -ENOMEM;
		goto free_page;
	}

	acvp_ctx.acvp_crypto_ctx = page_address(acvp_ctx.crypto_page);

	return ret;

free_page:
	__free_page(acvp_ctx.io_page);
blkdev_put:
	blkdev_put(acvp_ctx.bdev, FMODE_WRITE | FMODE_READ);
misc_deregister:
	misc_deregister(&acvp_ctx.miscdev);
out:
	return ret;
}

static void ufs_pixel_acvp_exit(void)
{
	__free_page(acvp_ctx.crypto_page);
	__free_page(acvp_ctx.io_page);
	blkdev_put(acvp_ctx.bdev, FMODE_WRITE | FMODE_READ);
	misc_deregister(&acvp_ctx.miscdev);
}

module_init(ufs_pixel_acvp_init);
module_exit(ufs_pixel_acvp_exit);

MODULE_DESCRIPTION("UFS FIPS140 ACVP Certification Module");
MODULE_AUTHOR("Konstantin Vyshetsky");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");
