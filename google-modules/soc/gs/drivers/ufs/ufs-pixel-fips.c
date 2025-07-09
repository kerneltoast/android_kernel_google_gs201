// SPDX-License-Identifier: GPL-2.0-only
/*
 * Pixel-Specific UFS feature support
 *
 * Copyright 2021 Google LLC
 *
 * Authors: Konstantin Vyshetsky <vkon@google.com>
 *
 */

#include <asm/unaligned.h>
#include <crypto/aes.h>
#include <crypto/algapi.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <scsi/scsi_proto.h>
#include "ufs-exynos-gs.h"
#include "ufs-pixel-fips.h"
#include "ufs-pixel-fips_sha256.h"

#undef CREATE_TRACE_POINTS
#include <trace/hooks/ufshcd.h>

#undef pr_fmt
#define pr_fmt(fmt) "ufs-pixel-fips140: " fmt

/*
 * FIPS140-3 requires module name and version information to be available. These
 * values will be logged to kernel log upon loading.
 */
#define UFS_PIXEL_FIPS140_MODULE_NAME "UFS Pixel FIPS CMVP Module"
#define UFS_PIXEL_FIPS140_MODULE_VERSION "2.0.0"

/*
 * As the verification logic will run before GPT data is available, module
 * params are passed to cmdline from bootloader
 */
static u32 fips_first_lba;
module_param(fips_first_lba, uint, 0444);
MODULE_PARM_DESC(fips_first_lba, "First LBA of FIPS partition");
static u32 fips_last_lba;
module_param(fips_last_lba, uint, 0444);
MODULE_PARM_DESC(fips_last_lba, "Last LBA of FIPS partition");
static u32 fips_lu;
module_param(fips_lu, uint, 0444);
MODULE_PARM_DESC(fips_lu, "FIPS partition LUN");
static bool use_hw_keys = true;
module_param(use_hw_keys, bool, 0444);
MODULE_PARM_DESC(
	use_hw_keys,
	"Sets operating mode to be either hardware or software keys mode");

#define UFS_PIXEL_UCD_SIZE		(4096)
#define UFS_PIXEL_BUFFER_SIZE		(4096)
#define UFS_PIXEL_CRYPTO_DATA_UNIT_SIZE (4096)
#define UFS_PIXEL_MASTER_KEY_INDEX	(15)
#define UTRD_CMD_TYPE_UFS_STORAGE	(1 << 28)
#define UTRD_DD_SYSTEM_TO_DEVICE	(1 << 25) /* Write */
#define UTRD_DD_DEVICE_TO_SYSTEM	(1 << 26) /* Read */
#define UTRD_CRYPTO_DISABLE		(0)
#define UTRD_CRYPTO_ENABLE		(1 << 23)
#define PRDT_FAS_XTS			(2 << 28) /* File Algorithm Selector */
#define PRDT_FKL_256			(1 << 26) /* File Key Length */
#define SENSE_DATA_ALLOC_LEN		(18)
#define UPIU_TT_COMMAND			(1)
#define IO_COMPLETION_TIMEOUT_MS	(200)
#define IO_RETRY_COUNT			(25)
#define ENCKEY_NUM_WORDS		(8)
#define TWKEY_NUM_WORDS			(8)
#define MESSAGE_LENGTH			(32)
#define ISE_VERSION_REG_OFFSET		(0x1C)
#define ISE_VERSION_MAJOR(x)		(((x) >> 16) & 0xFF)
#define ISE_VERSION_MINOR(x)		(((x) >> 8) & 0xFF)
#define ISE_VERSION_REVISION(x)		((x) & 0xFF)

struct fips_buffer_info {
	void *io_buffer;
	struct utp_transfer_cmd_desc *ucd_addr;
	dma_addr_t io_buffer_dma_addr;
	dma_addr_t ucd_dma_addr;
	union {
		const u8 *key;
		u32 mki;
	};
	const u8 *iv;
};

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

	/*
	* The key with all bytes reversed.  For XTS, the two
	* halves of the key are given separately and are
	* byte-reversed separately.
	*/
	__be32 file_enckey[ENCKEY_NUM_WORDS];
	__be32 file_twkey[TWKEY_NUM_WORDS];

	/* Unused */
	__le32 reserved[8];
};

struct sense_data {
	u8 data[SENSE_DATA_ALLOC_LEN];
};

struct scsi_cdb {
	u8 op_code;
	u8 dword1;
	__be32 lba;
	u8 dword6;
	__be16 transfer_len;
	u8 dword9;
} __packed;

struct upiu_header {
	/* DWORD 0 */
	u8 transaction_type;
	u8 flags;
	u8 lun;
	u8 task_tag;
	/* DWORD 1 */
	u8 rsvd; // 4 bit initiator_id + 4 bit cmd_set_type
	u8 function;
	u8 response;
	u8 status;
	/* DWORD 2 */
	u8 total_ehs_len;
	u8 device_info;
	__be16 data_segment_len;
	/* DWORD 3 - 7 */
	struct {
		__be32 edtl; // Expected Data Transfer Length
		struct scsi_cdb cdb; // SCSI Command Descriptor Block
	};
} __packed;

struct upiu {
	struct upiu_header header;
	__be16 sense_data_length;
	struct sense_data sense_data;
} __packed;

static int ise_available;

/* Configure inline encryption (or decryption) on requests that require it. */
static void ufs_pixel_fips_crypto_fill_prdt_hw_key_mode(void *unused,
							struct ufs_hba *hba,
							struct ufshcd_lrb *lrbp,
							unsigned int segments,
							int *err)
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

	/*
	 * FIPS140-3 prohibits access to encryption services if they did not
	 * pass the algorithm self test. Trigger a panic in such an event.
	 */
	if (!ise_available)
		panic("ISE encryption services are disabled\n");

	/* Configure encryption on each segment of the request. */
	for (i = 0; i < segments; i++) {
		struct pixel_ufs_prdt_entry *ent = &prdt[i];
		struct ufshcd_sg_entry *prd = (struct ufshcd_sg_entry *)ent;

		/* Each segment must be exactly one data unit. */
		if (le32_to_cpu(prd->size) + 1 != UFS_PIXEL_CRYPTO_DATA_UNIT_SIZE) {
			pr_err("scatterlist segment is misaligned for crypto\n");
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

/* Configure inline encryption (or decryption) on requests that require it. */
static void ufs_pixel_fips_crypto_fill_prdt_sw_key_mode(void *unused,
							struct ufs_hba *hba,
							struct ufshcd_lrb *lrbp,
							unsigned int segments,
							int *err)
{
	const struct bio_crypt_ctx *bc;
	const u8 *key, *tweak_key;
	u64 dun_lo, dun_hi;
	struct pixel_ufs_prdt_entry *prdt;
	unsigned int i;

	/*
	 * There's nothing to do for unencrypted requests, since the mode field
	 * ("FAS") is already 0 (FMP_BYPASS_MODE) by default, as it's in the
	 * same word as ufshcd_sg_entry::size which was already initialized.
	 */
	bc = scsi_cmd_to_rq(lrbp->cmd)->crypt_ctx;
	if (!bc)
		return;

	key = bc->bc_key->raw;
	tweak_key = key + AES_KEYSIZE_256;
	dun_lo = bc->bc_dun[0];
	dun_hi = bc->bc_dun[1];

	/* Reject weak AES-XTS keys. */
	if (!crypto_memneq(key, tweak_key, AES_KEYSIZE_256)) {
		dev_err(hba->dev, "Can't use weak AES-XTS key\n");
		*err = -EIO;
		return;
	}

	/* Configure FMP on each segment of the request. */
	prdt = (struct pixel_ufs_prdt_entry *)lrbp->ucd_prdt_ptr;
	for (i = 0; i < segments; i++) {
		struct pixel_ufs_prdt_entry *ent = &prdt[i];
		struct ufshcd_sg_entry *prd = (struct ufshcd_sg_entry *)ent;
		int j;

		/* Each segment must be exactly one data unit. */
		if (le32_to_cpu(prd->size) + 1 !=
		    UFS_PIXEL_CRYPTO_DATA_UNIT_SIZE) {
			pr_err("scatterlist segment is misaligned for crypto\n");
			*err = -EIO;
			return;
		}

		/* Set the algorithm and key length. */
		ent->des3 |= cpu_to_le32(PRDT_FAS_XTS | PRDT_FKL_256);

		/* Set the key. */
		for (j = 0; j < ENCKEY_NUM_WORDS; j++) {
			ent->file_enckey[ENCKEY_NUM_WORDS - 1 - j] =
				get_unaligned_be32(&key[j * 4]);
			ent->file_twkey[TWKEY_NUM_WORDS - 1 - j] =
				get_unaligned_be32(&tweak_key[j * 4]);
		}

		/* Set the IV. */
		ent->iv[0] = cpu_to_be64(dun_hi);
		ent->iv[1] = cpu_to_be64(dun_lo);

		/* Increment the data unit number. */
		dun_lo++;
		if (dun_lo == 0)
			dun_hi++;
	}
}

static void ufs_pixel_fips_build_utrd(struct ufs_hba *hba,
				      struct utp_transfer_req_desc *utrd,
				      struct fips_buffer_info *bi,
				      u32 data_direction)
{
	dma_addr_t ucd_dma_addr = bi->ucd_dma_addr;
	u16 response_offset =
		offsetof(struct utp_transfer_cmd_desc, response_upiu);
	u16 prdt_offset = offsetof(struct utp_transfer_cmd_desc, prd_table);
	u16 prdt_length = sizeof(struct pixel_ufs_prdt_entry);
	u32 crypto = bi->iv ? UTRD_CRYPTO_ENABLE : UTRD_CRYPTO_DISABLE;

	memset(utrd, 0, sizeof(struct utp_transfer_req_desc));

	utrd->header.dword_0 = cpu_to_le32(UTRD_CMD_TYPE_UFS_STORAGE |
					   data_direction | crypto);
	utrd->header.dword_1 = 0;
	utrd->header.dword_2 = cpu_to_le32(OCS_INVALID_COMMAND_STATUS);
	utrd->header.dword_3 = 0;

	utrd->command_desc_base_addr = cpu_to_le64(ucd_dma_addr);

	if (hba->quirks & UFSHCD_QUIRK_PRDT_BYTE_GRAN) {
		utrd->response_upiu_length = cpu_to_le16(ALIGNED_UPIU_SIZE);
		utrd->response_upiu_offset = cpu_to_le16(response_offset);
		utrd->prd_table_offset = cpu_to_le16(prdt_offset);
		utrd->prd_table_length = cpu_to_le16(prdt_length);
	} else {
		utrd->response_upiu_length =
			cpu_to_le16(ALIGNED_UPIU_SIZE >> 2);
		utrd->response_upiu_offset = cpu_to_le16(response_offset >> 2);
		utrd->prd_table_offset = cpu_to_le16(prdt_offset >> 2);
		utrd->prd_table_length = cpu_to_le16(1);
	}
}

static void ufs_pixel_fips_build_prdt(struct ufs_hba *hba,
				      struct fips_buffer_info *bi,
				      u32 buffer_len)
{
	struct utp_transfer_cmd_desc *ucd_addr = bi->ucd_addr;
	dma_addr_t buffer_dma_addr = bi->io_buffer_dma_addr;
	struct pixel_ufs_prdt_entry *sg_entry =
		(struct pixel_ufs_prdt_entry *)ucd_addr->prd_table;

	sg_entry->des0 = cpu_to_le32(lower_32_bits(buffer_dma_addr));
	sg_entry->des1 = cpu_to_le32(upper_32_bits(buffer_dma_addr));
	sg_entry->des2 = 0;

	if (!bi->iv) {
		sg_entry->des3 = cpu_to_le32(buffer_len - 1);
		return;
	}

	/*
	* The hardware interprets the IV in backwards order. Hence we
	* reverse each byte of the 16 byte IV.
	*/
	sg_entry->iv[0] = get_unaligned_be64(bi->iv + 8);
	sg_entry->iv[1] = get_unaligned_be64(bi->iv);

	if (use_hw_keys) {
		sg_entry->des3 =
			cpu_to_le32(CRYPTO_ENABLE | CRYPTO_KEYSLOT(bi->mki) |
				    (buffer_len - 1));
	} else {
		const u8 *key = bi->key;
		const u8 *tweak_key = key + (ENCKEY_NUM_WORDS * 4);
		u32 i;

		/*
		* The verification only needs to test AES-256-XTS algorithm as
		* it's the only one being used by the system and the only one
		* being certified. The actual gs101 hardware is capable of
		* supporting AES-128-CBC, AES-256-CBC, and AES-128-XTS as well.
		*/
		sg_entry->des3 = cpu_to_le32(PRDT_FAS_XTS | PRDT_FKL_256 |
					     (buffer_len - 1));

		for (i = 0; i < ENCKEY_NUM_WORDS; i++) {
			sg_entry->file_enckey[ENCKEY_NUM_WORDS - 1 - i] =
				get_unaligned_be32(&key[i * 4]);
			sg_entry->file_twkey[TWKEY_NUM_WORDS - 1 - i] =
				get_unaligned_be32(&tweak_key[i * 4]);
		}
	}
}

static void ufs_pixel_fips_build_upiu(struct ufs_hba *hba,
				      struct utp_transfer_cmd_desc *ucd_addr,
				      struct scsi_cdb *cdb, u16 flags, u32 lun,
				      u32 buffer_len, u8 task_tag)
{
	struct utp_upiu_req *ucd_req_ptr =
		(struct utp_upiu_req *)ucd_addr->command_upiu;

	ucd_req_ptr->header.dword_0 = UPIU_HEADER_DWORD(
		UPIU_TRANSACTION_COMMAND, flags, lun, task_tag);
	ucd_req_ptr->header.dword_1 = 0;
	ucd_req_ptr->header.dword_2 = 0;
	ucd_req_ptr->sc.exp_data_transfer_len = cpu_to_be32(buffer_len);
	memcpy(ucd_req_ptr->sc.cdb, cdb, sizeof(struct scsi_cdb));
}

static int ufs_pixel_fips_send_utrd(struct ufs_hba *hba,
				    struct utp_transfer_req_desc *utrd,
				    u8 task_tag)
{
	struct utp_transfer_req_desc utrd_temp;
	unsigned long timeout;
	u32 tr_doorbell;

	memcpy(&utrd_temp, hba->utrdl_base_addr + task_tag,
	       sizeof(struct utp_transfer_req_desc));
	memcpy(hba->utrdl_base_addr + task_tag, utrd,
	       sizeof(struct utp_transfer_req_desc));

	if (hba->vops && hba->vops->setup_xfer_req) {
		spin_lock_irq(&hba->outstanding_lock);
		hba->vops->setup_xfer_req(hba, task_tag, true);
		spin_unlock_irq(&hba->outstanding_lock);
	}

	ufshcd_writel(hba, 1 << task_tag, REG_UTP_TRANSFER_REQ_DOOR_BELL);

	/* Make sure that doorbell is committed immediately */
	wmb();

	/* Wait for completion */
	timeout = jiffies + msecs_to_jiffies(IO_COMPLETION_TIMEOUT_MS);
	while (time_before(jiffies, timeout)) {
		tr_doorbell = ufshcd_readl(hba, REG_UTP_TRANSFER_REQ_DOOR_BELL);
		if (!tr_doorbell)
			break;

		usleep_range(50, 100);
	}

	memcpy(utrd, hba->utrdl_base_addr + task_tag,
	       sizeof(struct utp_transfer_req_desc));
	memcpy(hba->utrdl_base_addr + task_tag, &utrd_temp,
	       sizeof(struct utp_transfer_req_desc));

	if (tr_doorbell) {
		pr_err("Request timed out\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static int ufs_pixel_fips_check_response(struct utp_upiu_rsp *resp, u8 ocs)
{
	u8 status = be32_to_cpu(resp->header.dword_1);
	u8 response = be32_to_cpu(resp->header.dword_1) >> 8;
	if (ocs || status || response) {
		uint8_t response_code = resp->sr.sense_data[0] & 0x7F;
		if (response_code == 0x70) {
			uint8_t key = resp->sr.sense_data[2] & 0xF;
			uint8_t asc = resp->sr.sense_data[12];
			uint8_t ascq = resp->sr.sense_data[13];
			if (!ocs && status == 2 && response == 1 && key == 6 && asc == 0x29)
				pr_info("UA Reported\n");
			else
				pr_warn("I/O Result: OCS=%x status=%X key=%X asc=%X ascq=%X\n",
					ocs, status, key, asc, ascq);
		} else {
			pr_warn("I/O Result: OCS=%x status=%X\n", ocs, status);
		}
		return -EIO;
	}

	return 0;
}

int ufs_pixel_fips_send_request(struct ufs_hba *hba, struct scsi_cdb *cdb,
				struct fips_buffer_info *bi, u32 buffer_len,
				u32 lu)
{
	struct utp_transfer_req_desc utrd;
	struct utp_upiu_rsp *resp_upiu =
		(struct utp_upiu_rsp *)bi->ucd_addr->response_upiu;
	u8 task_tag = 0x7;
	u8 ocs;
	u32 data_direction;
	u32 flags;
	int ret;

	memset(bi->ucd_addr, 0, UFS_PIXEL_UCD_SIZE);

	/* Build CDB */
	switch (cdb->op_code) {
	case REQUEST_SENSE:
		data_direction = UTRD_DD_DEVICE_TO_SYSTEM;
		flags = UPIU_CMD_FLAGS_READ;
		break;
	case READ_10:
		data_direction = UTRD_DD_DEVICE_TO_SYSTEM;
		flags = UPIU_CMD_FLAGS_READ;
		break;
	case WRITE_10:
		data_direction = UTRD_DD_SYSTEM_TO_DEVICE;
		flags = UPIU_CMD_FLAGS_WRITE;
		break;
	default:
		pr_err("unsupported scsi op code 0x%02x\n", cdb->op_code);
		return -EBADRQC;
	}

	/* Build UTRD */
	ufs_pixel_fips_build_utrd(hba, &utrd, bi, data_direction);

	/* Build PRDT */
	ufs_pixel_fips_build_prdt(hba, bi, buffer_len);

	/* Build UPIU */
	ufs_pixel_fips_build_upiu(hba, bi->ucd_addr, cdb, flags, lu, buffer_len,
				  task_tag);

	/* Make sure descriptors are ready before ringing the task doorbell */
	wmb();

	/* Send */
	ret = ufs_pixel_fips_send_utrd(hba, &utrd, task_tag);
	if (ret)
		return ret;

	ocs = le32_to_cpu(utrd.header.dword_2) & 0xFF;

	return ufs_pixel_fips_check_response(resp_upiu, ocs);
}

static int ufs_pixel_fips_request_sense(struct ufs_hba *hba,
					struct fips_buffer_info *bi)
{
	struct scsi_cdb cdb = {};
	int ret;

	cdb.op_code = REQUEST_SENSE;
	cdb.transfer_len = cpu_to_be16(SENSE_DATA_ALLOC_LEN);

	ret = ufs_pixel_fips_send_request(hba, &cdb, bi, SENSE_DATA_ALLOC_LEN,
					  fips_lu);

	if (ret)
		return -EIO;

	return 0;
}

static int ufs_pixel_fips_send_io(struct ufs_hba *hba,
				  struct fips_buffer_info *bi, u8 op_code)
{
	struct scsi_cdb cdb = {};
	int ret;
	int retry = IO_RETRY_COUNT;

	cdb.op_code = op_code;
	cdb.lba = cpu_to_be32(fips_first_lba);
	cdb.transfer_len = cpu_to_be16(1);

	do {
		ret = ufs_pixel_fips_send_request(
			hba, &cdb, bi, UFS_PIXEL_BUFFER_SIZE, fips_lu);
	} while (ret && retry-- > 0);

	if (ret)
		return -EIO;

	return 0;
}

static int ufs_pixel_fips_read(struct ufs_hba *hba, struct fips_buffer_info *bi)
{
	return ufs_pixel_fips_send_io(hba, bi, READ_10);
}

static int ufs_pixel_fips_write(struct ufs_hba *hba,
				struct fips_buffer_info *bi)
{
	return ufs_pixel_fips_send_io(hba, bi, WRITE_10);
}

static const u8 pixel_fips_encryption_pt[] = {
	0x54, 0x68, 0x69, 0x73, 0x20, 0x69, 0x73, 0x20, /* "This is " */
	0x61, 0x20, 0x33, 0x32, 0x42, 0x20, 0x65, 0x6E, /* "a 32B en" */
	0x63, 0x72, 0x79, 0x70, 0x74, 0x20, 0x70, 0x74, /* "crypt pt" */
	0x20, 0x6D, 0x65, 0x73, 0x73, 0x61, 0x67, 0x65, /* " message" */
};

/* Ciphertext based on the fips_encryption key below used in sw key mode. */
static const u8 pixel_fips_encryption_ct[] = {
	0x57, 0x4F, 0x6F, 0xD1, 0x21, 0x33, 0xE5, 0xF4,
	0x6F, 0x72, 0x30, 0x63, 0x8B, 0xCE, 0xA7, 0xD4,
	0x84, 0x84, 0xF9, 0xE2, 0xC5, 0xB9, 0xE4, 0x48,
	0x8D, 0x49, 0x02, 0x88, 0x80, 0xB3, 0x07, 0xE2,
};

/* Ciphertext based on the EFUSE encryption key used in hw key mode. */
static const u8 pixel_fips_encryption_efuse_ct[] = {
	0xEE, 0xD2, 0xD3, 0x69, 0xE9, 0x60, 0x48, 0xF1,
	0x26, 0xE8, 0xC6, 0xD7, 0x2E, 0xFB, 0x0C, 0x69,
	0x2A, 0xC4, 0xF4, 0x32, 0x58, 0xD0, 0x7B, 0xC2,
	0x75, 0xA9, 0xB0, 0x4B, 0x4E, 0x39, 0x31, 0x98,
};

static const u8 pixel_fips_encryption_key[] = {
	0x54, 0x68, 0x69, 0x73, 0x20, 0x69, 0x73, 0x20, /* "This is " */
	0x61, 0x20, 0x33, 0x32, 0x42, 0x20, 0x65, 0x6E, /* "a 32B en" */
	0x63, 0x72, 0x79, 0x70, 0x74, 0x69, 0x6F, 0x6E, /* "cryption" */
	0x20, 0x6B, 0x65, 0x79, 0x20, 0x77, 0x69, 0x74, /* " key wit" */
	0x68, 0x20, 0x61, 0x20, 0x33, 0x32, 0x42, 0x20, /* "h a 32B " */
	0x65, 0x6E, 0x63, 0x72, 0x79, 0x70, 0x74, 0x69, /* "encrypti" */
	0x6F, 0x6E, 0x20, 0x74, 0x77, 0x65, 0x61, 0x6B, /* "on tweak" */
	0x20, 0x6B, 0x65, 0x79, 0x20, 0x20, 0x20, 0x20, /* " key    " */
};

static const u8 pixel_fips_encryption_iv[] = {
	0x41, 0x20, 0x31, 0x36, 0x42, 0x20, 0x65, 0x6E, /* "A 16B en" */
	0x63, 0x72, 0x79, 0x70, 0x74, 0x20, 0x49, 0x56, /* "crypt IV" */
};

static u32 ufs_pixel_fips_get_ise_version(struct ufs_hba *hba)
{
	struct exynos_ufs *ufs = to_exynos_ufs(hba);
	struct ufs_vs_handle *handle = &ufs->handle;

	return readl(handle->ufsp + ISE_VERSION_REG_OFFSET);
}

static struct ufs_pixel_fips_info fips_info;
const struct ufs_pixel_fips_info *ufs_pixel_fips_get_info(struct ufs_hba *hba)
{
	u32 ise_version = ufs_pixel_fips_get_ise_version(hba);

	fips_info.ise_version_major = ISE_VERSION_MAJOR(ise_version);
	fips_info.ise_version_minor = ISE_VERSION_MINOR(ise_version);
	fips_info.ise_version_revision = ISE_VERSION_REVISION(ise_version);
	fips_info.key_delivery_mode =
		use_hw_keys ? KEY_DELIVERY_HW : KEY_DELIVERY_SW;

	return &fips_info;
}
EXPORT_SYMBOL_GPL(ufs_pixel_fips_get_info);

int ufs_pixel_fips_verify(struct ufs_hba *hba)
{
	int ret;
	u32 interrupts;
	struct fips_buffer_info bi;
	static bool print_ise_version = true;
	const u8 *encryption_ct = use_hw_keys ? pixel_fips_encryption_efuse_ct :
						pixel_fips_encryption_ct;

	if (print_ise_version) {
		u32 ise_version = ufs_pixel_fips_get_ise_version(hba);
		pr_info("ISE HW version  %u.%u.%u\n",
			 ISE_VERSION_MAJOR(ise_version),
			 ISE_VERSION_MINOR(ise_version),
			 ISE_VERSION_REVISION(ise_version));
		print_ise_version = false;
	}

	ise_available = 0;

	if (!fips_first_lba || !fips_last_lba ||
	    fips_last_lba < fips_first_lba) {
		pr_err("Invalid module params: first_lba=%u last_lba=%u\n",
		       fips_first_lba, fips_last_lba);
		return -EINVAL;
	}

	bi.io_buffer = dma_alloc_coherent(hba->dev, UFS_PIXEL_BUFFER_SIZE,
					  &bi.io_buffer_dma_addr,
					  GFP_NOIO | __GFP_NOFAIL);
	if (!bi.io_buffer)
		return -ENOMEM;

	bi.ucd_addr = dma_alloc_coherent(hba->dev, UFS_PIXEL_UCD_SIZE,
					 &bi.ucd_dma_addr,
					 GFP_NOIO | __GFP_NOFAIL);
	if (!bi.ucd_addr) {
		dma_free_coherent(hba->dev, UFS_PIXEL_BUFFER_SIZE, bi.io_buffer,
				  bi.io_buffer_dma_addr);
		return -ENOMEM;
	}
	bi.iv = NULL;
	bi.key = NULL;

	/*
	 * Enable clocks, exit hibern8, set link as active
	 * Will release on function exit
	 */
	ufshcd_hold(hba, false);

	/*
	 * Disable all interrupts except UTP Transfer Request Completion
	 * Controller will not complete requests without this enabled
	 * Restore on function exit
	 */
	interrupts = ufshcd_readl(hba, REG_INTERRUPT_ENABLE);
	ufshcd_writel(hba, UTP_TRANSFER_REQ_COMPL, REG_INTERRUPT_ENABLE);

	ufs_pixel_fips_request_sense(hba, &bi);

	/*
	 * Verify Encryption:
	 * Write plaintext with specified crypto parameters, then read raw.
	 * Compare vs expected ciphertext.
	 */
	fips_info.encryption_test_attempted++;
	memset(bi.io_buffer, 0, UFS_PIXEL_BUFFER_SIZE);
	memcpy(bi.io_buffer, pixel_fips_encryption_pt,
	       sizeof(pixel_fips_encryption_pt));
	if (use_hw_keys)
		bi.mki = UFS_PIXEL_MASTER_KEY_INDEX;
	else
		bi.key = pixel_fips_encryption_key;
	bi.iv = pixel_fips_encryption_iv;

	ret = ufs_pixel_fips_write(hba, &bi);
	if (ret)
		goto out;

	memset(bi.io_buffer, 0, UFS_PIXEL_BUFFER_SIZE);
	bi.key = NULL;
	bi.iv = NULL;

	ret = ufs_pixel_fips_read(hba, &bi);
	if (ret)
		goto out;

	if (memcmp(bi.io_buffer, encryption_ct, MESSAGE_LENGTH)) {
		pr_err("Encryption verification failed\n");
		ret = -EINVAL;
		goto out;
	}

	fips_info.encryption_test_passed++;
	pr_info("Encryption verification passed\n");

	/*
	 * Verify Decryption:
	 * Since the ciphertext is already stored we just read back with the
	 * specified crypto parameters.
	 * Compare vs expected plaintext.
	 */
	fips_info.decryption_test_attempted++;
	memset(bi.io_buffer, 0, UFS_PIXEL_BUFFER_SIZE);
	if (use_hw_keys)
		bi.mki = UFS_PIXEL_MASTER_KEY_INDEX;
	else
		bi.key = pixel_fips_encryption_key;
	bi.iv = pixel_fips_encryption_iv;

	ret = ufs_pixel_fips_read(hba, &bi);
	if (ret)
		goto out;

	if (memcmp(bi.io_buffer, pixel_fips_encryption_pt,
		   sizeof(pixel_fips_encryption_pt))) {
		pr_err("Decryption verification failed\n");
		ret = -EINVAL;
		goto out;
	}

	fips_info.decryption_test_passed++;
	pr_info("Decryption verification passed\n");

out:
	ufshcd_writel(hba, interrupts, REG_INTERRUPT_ENABLE);
	ufshcd_release(hba);
	memzero_explicit(bi.ucd_addr, UFS_PIXEL_UCD_SIZE);
	memzero_explicit(bi.io_buffer, UFS_PIXEL_BUFFER_SIZE);
	dma_free_coherent(hba->dev, UFS_PIXEL_UCD_SIZE, bi.ucd_addr,
			  bi.ucd_dma_addr);
	dma_free_coherent(hba->dev, UFS_PIXEL_BUFFER_SIZE, bi.io_buffer,
			  bi.io_buffer_dma_addr);

	ise_available = !ret;

	return ret;
}
EXPORT_SYMBOL_GPL(ufs_pixel_fips_verify);

static int __init unapply_text_relocations(void *section, int section_size,
					   const Elf64_Rela *rela, int numrels)
{
	while (numrels--) {
		u32 *place = (u32 *)(section + rela->r_offset);

		if (rela->r_offset >= section_size) {
			pr_err("rela->r_offset(%llu) >= section_size(%u)",
			       rela->r_offset, section_size);
			return -EINVAL;
		}

		switch (ELF64_R_TYPE(rela->r_info)) {
#ifdef CONFIG_ARM64
		case R_AARCH64_JUMP26:
		case R_AARCH64_CALL26:
			*place &= ~GENMASK(25, 0);
			break;

		case R_AARCH64_ADR_PREL_LO21:
		case R_AARCH64_ADR_PREL_PG_HI21:
		case R_AARCH64_ADR_PREL_PG_HI21_NC:
			*place &= ~(GENMASK(30, 29) | GENMASK(23, 5));
			break;

		case R_AARCH64_ADD_ABS_LO12_NC:
		case R_AARCH64_LDST8_ABS_LO12_NC:
		case R_AARCH64_LDST16_ABS_LO12_NC:
		case R_AARCH64_LDST32_ABS_LO12_NC:
		case R_AARCH64_LDST64_ABS_LO12_NC:
		case R_AARCH64_LDST128_ABS_LO12_NC:
			*place &= ~GENMASK(21, 10);
			break;
		default:
			pr_err("unhandled relocation type %llu\n",
			       ELF64_R_TYPE(rela->r_info));
			return -EINVAL;
#else
#error
#endif
		}
		rela++;
	}

	return 0;
}

enum {
       PACIASP         = 0xd503233f,
       AUTIASP         = 0xd50323bf,
       SCS_PUSH        = 0xf800865e,
       SCS_POP         = 0xf85f8e5e,
};

/*
 * To make the integrity check work with dynamic Shadow Call Stack (SCS),
 * replace all instructions that push or pop from the SCS with the Pointer
 * Authentication Code (PAC) instructions that were present originally.
 */
static void __init unapply_scs_patch(void *section, int section_size)
{
#if defined(CONFIG_ARM64) && defined(CONFIG_UNWIND_PATCH_PAC_INTO_SCS)
       u32 *insns = section;
       int i;

       for (i = 0; i < section_size / sizeof(insns[0]); i++) {
               if (insns[i] == SCS_PUSH)
                       insns[i] = PACIASP;
               else if (insns[i] == SCS_POP)
                       insns[i] = AUTIASP;
       }
#endif
}

static const u8 ufs_pixel_fips_hmac_message[] = {
	0x54, 0x68, 0x69, 0x73, 0x20, 0x69, 0x73, 0x20, /* "This is " */
	0x61, 0x20, 0x35, 0x38, 0x42, 0x20, 0x6D, 0x65, /* "a 58B me" */
	0x73, 0x73, 0x61, 0x67, 0x65, 0x20, 0x66, 0x6F, /* "ssage fo" */
	0x72, 0x20, 0x48, 0x4D, 0x41, 0x43, 0x20, 0x76, /* "r HMAC v" */
	0x65, 0x72, 0x69, 0x66, 0x69, 0x63, 0x61, 0x74, /* "erificat" */
	0x69, 0x6F, 0x6E, 0x20, 0x69, 0x6E, 0x20, 0x46, /* "ion in F" */
	0x49, 0x50, 0x53, 0x20, 0x6D, 0x6F, 0x64, 0x75, /* "IPS modu" */
	0x6C, 0x65,					/* "le"       */
};

static const u8 ufs_pixel_fips_hmac_key[] = {
	0x54, 0x68, 0x69, 0x73, 0x20, 0x69, 0x73, 0x20, /* "This is " */
	0x61, 0x20, 0x35, 0x34, 0x42, 0x20, 0x6B, 0x65, /* "a 54B ke" */
	0x79, 0x20, 0x66, 0x6F, 0x72, 0x20, 0x48, 0x4D, /* "y for HM" */
	0x41, 0x43, 0x20, 0x76, 0x65, 0x72, 0x69, 0x66, /* "AC verif" */
	0x69, 0x63, 0x61, 0x74, 0x69, 0x6F, 0x6E, 0x20, /* "ication " */
	0x69, 0x6E, 0x20, 0x46, 0x49, 0x50, 0x53, 0x20, /* "in FIPS " */
	0x6D, 0x6F, 0x64, 0x75, 0x6C, 0x65,		/* "module"   */
};

static const u8 ufs_pixel_fips_hmac_expected[] = {
	0x35, 0x3E, 0xA3, 0xB1, 0xEF, 0x1A, 0x79, 0x46,
	0xDA, 0x21, 0x27, 0x64, 0x8F, 0x37, 0x1D, 0xD2,
	0x5B, 0x5B, 0x84, 0xF3, 0x60, 0xB6, 0x95, 0x61,
	0xF9, 0x06, 0x07, 0x73, 0x18, 0x77, 0xB7, 0x1D,
};

u8 __initdata fips140_integ_hmac_key[] = {
	0x54, 0x68, 0x65, 0x20, 0x71, 0x75, 0x69, 0x63, /* "The quic" */
	0x6B, 0x20, 0x62, 0x72, 0x6F, 0x77, 0x6E, 0x20, /* "k brown " */
	0x66, 0x6F, 0x78, 0x20, 0x6A, 0x75, 0x6D, 0x70, /* "fox jump" */
	0x73, 0x20, 0x6F, 0x76, 0x65, 0x72, 0x20, 0x74, /* "s over t" */
	0x68, 0x65, 0x20, 0x6C, 0x61, 0x7A, 0x79, 0x20, /* "he lazy " */
	0x64, 0x6F, 0x67, 0x00				/* "dog"      */
};

u8 __initdata fips140_integ_hmac_digest[UFS_PIXEL_FIPS_SHA256_DIGEST_SIZE];
const u8 __fips140_text_start __section(".text.._start");
const u8 __fips140_text_end __section(".text.._end");
const u8 __fips140_rodata_start __section(".rodata.._start");
const u8 __fips140_rodata_end __section(".rodata.._end");
const u8 *__ufs_pixel_text_start = &__fips140_text_start;
const u8 *__ufs_pixel_rodata_start = &__fips140_rodata_start;

static int __init ufs_pixel_hmac_self_test(void)
{
	u8 hmac_digest[UFS_PIXEL_FIPS_SHA256_DIGEST_SIZE];
	int ret;

	fips_info.hmac_self_test_attempted++;
	ufs_pixel_fips_hmac_sha256(ufs_pixel_fips_hmac_message,
				   sizeof(ufs_pixel_fips_hmac_message),
				   ufs_pixel_fips_hmac_key,
				   sizeof(ufs_pixel_fips_hmac_key),
				   hmac_digest);

	ret = memcmp(hmac_digest, ufs_pixel_fips_hmac_expected,
		      UFS_PIXEL_FIPS_SHA256_DIGEST_SIZE);
	memzero_explicit(hmac_digest, sizeof(hmac_digest));
	if (!ret)
		fips_info.hmac_self_test_passed++;

	return ret;
}
extern struct {
	u32	offset;
	u32	count;
} fips140_rela_text;

static int __init ufs_pixel_self_integrity_test(void)
{
	u8 hmac_digest[UFS_PIXEL_FIPS_SHA256_DIGEST_SIZE];
	size_t text_len;
	size_t rodata_len;
	void *hmac_buffer;
	int ret;

	fips_info.self_integrity_test_attempted++;
	text_len = &__fips140_text_end - &__fips140_text_start;
	rodata_len = &__fips140_rodata_end - &__fips140_rodata_start;
	hmac_buffer = kmalloc(text_len + rodata_len, GFP_KERNEL);
	if (!hmac_buffer)
		return -ENOMEM;

	memcpy(hmac_buffer, __ufs_pixel_text_start, text_len);
	memcpy(hmac_buffer + text_len, __ufs_pixel_rodata_start, rodata_len);

	ret = unapply_text_relocations(hmac_buffer, text_len,
				       offset_to_ptr(&fips140_rela_text.offset),
				       fips140_rela_text.count);
	if (ret) {
		kfree_sensitive(hmac_buffer);
		return ret;
	}

	unapply_scs_patch(hmac_buffer, text_len);

	ufs_pixel_fips_hmac_sha256(hmac_buffer, text_len + rodata_len,
				   fips140_integ_hmac_key,
				   strlen(fips140_integ_hmac_key), hmac_digest);

	kfree_sensitive(hmac_buffer);

	ret = memcmp(hmac_digest, fips140_integ_hmac_digest,
		      UFS_PIXEL_FIPS_SHA256_DIGEST_SIZE);
	memzero_explicit(hmac_digest, sizeof(hmac_digest));
	if (!ret)
		fips_info.self_integrity_test_passed++;

	return ret;
}

static int __init ufs_pixel_fips_init(void)
{
	int ret;

	pr_info ("%s version %s loading...\n",
		 UFS_PIXEL_FIPS140_MODULE_NAME,
		 UFS_PIXEL_FIPS140_MODULE_VERSION);

	/* Verify internal HMAC functionality */
	if (ufs_pixel_hmac_self_test()) {
		pr_err("HMAC self test failed\n");
		return -EINVAL;
	}
	pr_info("HMAC self test passed\n");

	/* Perform module self integrity check */
	if (ufs_pixel_self_integrity_test()) {
		pr_err("Verify self HMAC failed\n");
		return -EINVAL;
	}
	pr_info("Verify self HMAC passed\n");

	ret = register_trace_android_vh_ufs_fill_prdt(
		use_hw_keys ? ufs_pixel_fips_crypto_fill_prdt_hw_key_mode :
			      ufs_pixel_fips_crypto_fill_prdt_sw_key_mode,
		NULL);
	if (ret)
		pr_err("Failed to register ufs_pixel_fips_crypto_fill_prdt\n");

	return ret;
}

static void ufs_pixel_fips_exit(void)
{
}

module_init(ufs_pixel_fips_init);
module_exit(ufs_pixel_fips_exit);

MODULE_DESCRIPTION(
	"FIPS140-3 Compliant SW Driven UFS Inline Encryption Self Test Module");
MODULE_AUTHOR("Konstantin Vyshetsky");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(UFS_PIXEL_FIPS140_MODULE_VERSION);
