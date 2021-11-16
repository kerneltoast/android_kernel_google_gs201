// SPDX-License-Identifier: GPL-2.0-only
/*
 * Pixel-Specific UFS feature support
 *
 * Copyright 2021 Google LLC
 *
 * Authors: Konstantin Vyshetsky <vkon@google.com>
 * Version: 1.0.0
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <asm/unaligned.h>
#include <scsi/scsi_proto.h>
#include "ufs-pixel-fips.h"
#include "ufs-pixel-fips_sha256.h"

#undef pr_fmt
#define pr_fmt(fmt) "ufs-pixel-fips140: " fmt

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

#define UFS_PIXEL_UCD_SIZE		(4096)
#define UFS_PIXEL_BUFFER_SIZE		(4096)
#define UTRD_CMD_TYPE_UFS_STORAGE	(1 << 28)
#define UTRD_DD_SYSTEM_TO_DEVICE	(1 << 25) /* Write */
#define UTRD_DD_DEVICE_TO_SYSTEM	(1 << 26) /* Read */
#define UTRD_CRYPTO_DISABLE		(0)
#define UTRD_CRYPTO_ENABLE		(1 << 23)
#define PRDT_FAS_XTS			(2 << 28) /* File Algorithm Selector */
#define PRDT_FKL_256			(1 << 26) /* File Key Length */
#define SENSE_DATA_ALLOC_LEN		(18)
#define UPIU_TT_COMMAND			(1)
#define IO_COMPLETION_TIMEOUT_MS	(50)
#define IO_RETRY_COUNT			(3)
#define SG_ENTRY_IV_NUM_WORDS		(4)
#define SG_ENTRY_ENCKEY_NUM_WORDS	(8)
#define SG_ENTRY_TWKEY_NUM_WORDS	(8)

struct fips_buffer_info {
	void *io_buffer;
	struct utp_transfer_cmd_desc *ucd_addr;
	dma_addr_t io_buffer_dma_addr;
	dma_addr_t ucd_dma_addr;
};

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

	/* The IV with all bytes reversed */
	__be32 file_iv[SG_ENTRY_IV_NUM_WORDS];

	/*
	 * The key with all bytes reversed.  For XTS, the two halves of the key
	 * are given separately and are byte-reversed separately.
	 */
	__be32 file_enckey[SG_ENTRY_ENCKEY_NUM_WORDS];
	__be32 file_twkey[SG_ENTRY_TWKEY_NUM_WORDS];

	/* Not used */
	__be32 disk_iv[4];
	__le32 reserved[4];
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

static void ufs_pixel_fips_build_utrd(struct ufs_hba *hba,
				      struct utp_transfer_req_desc *utrd,
				      dma_addr_t ucd_dma_addr,
				      u32 data_direction, u32 crypto)
{
	u16 response_offset =
		offsetof(struct utp_transfer_cmd_desc, response_upiu);
	u16 prdt_offset = offsetof(struct utp_transfer_cmd_desc, prd_table);
	u16 prdt_length = sizeof(struct fmp_sg_entry);

	memset(utrd, 0, sizeof(struct utp_transfer_req_desc));

	utrd->header.dword_0 = cpu_to_le32(UTRD_CMD_TYPE_UFS_STORAGE |
					   data_direction | crypto);
	utrd->header.dword_1 = 0;
	utrd->header.dword_2 = cpu_to_le32(OCS_INVALID_COMMAND_STATUS);
	utrd->header.dword_3 = 0;

	utrd->command_desc_base_addr_lo =
		cpu_to_le32(lower_32_bits(ucd_dma_addr));
	utrd->command_desc_base_addr_hi =
		cpu_to_le32(upper_32_bits(ucd_dma_addr));

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
				      struct utp_transfer_cmd_desc *ucd_addr,
				      dma_addr_t buffer_dma_addr,
				      u32 buffer_len, const u8 *key,
				      const u8 *iv)
{
	struct fmp_sg_entry *sg_entry =
		(struct fmp_sg_entry *)ucd_addr->prd_table;

	sg_entry->des0 = cpu_to_le32(lower_32_bits(buffer_dma_addr));
	sg_entry->des1 = cpu_to_le32(upper_32_bits(buffer_dma_addr));
	sg_entry->des2 = 0;
	if (!(key && iv)) {
		sg_entry->des3 = cpu_to_le32(buffer_len - 1);
	} else {
		u32 i;
		const u8 *tweak_key = key + (SG_ENTRY_ENCKEY_NUM_WORDS * 4);

		/*
		* The verification only needs to test AES-256-XTS algorithm as
		* it's the only one being used by the system and the only one
		* being certified. The actual gs101 hardware is capable of
		* supporting AES-128-CBC, AES-256-CBC, and AES-128-XTS as well.
		*/
		sg_entry->des3 = cpu_to_le32(PRDT_FAS_XTS | PRDT_FKL_256 |
					     (buffer_len - 1));

		for (i = 0; i < SG_ENTRY_IV_NUM_WORDS; i++) {
			sg_entry->file_iv[SG_ENTRY_IV_NUM_WORDS - 1 - i] =
				get_unaligned_be32(&iv[i * 4]);
		}

		for (i = 0; i < SG_ENTRY_ENCKEY_NUM_WORDS; i++) {
			sg_entry->file_enckey[SG_ENTRY_ENCKEY_NUM_WORDS - 1 - i] =
				get_unaligned_be32(&key[i * 4]);
			sg_entry->file_twkey[SG_ENTRY_TWKEY_NUM_WORDS - 1 - i] =
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

	if (hba->vops && hba->vops->setup_xfer_req)
		hba->vops->setup_xfer_req(hba, task_tag, true);

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

int ufs_pixel_fips_send_request(struct ufs_hba *hba, struct scsi_cdb *cdb,
				struct fips_buffer_info *bi, u32 buffer_len,
				u32 lu, const u8 *key, const u8 *iv)
{
	struct utp_transfer_req_desc utrd;
	struct utp_upiu_rsp *resp_upiu =
		(struct utp_upiu_rsp *)bi->ucd_addr->response_upiu;
	u8 task_tag = 0x7;
	u32 data_direction;
	u32 flags;
	u32 crypto;
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
	crypto = key && iv ? UTRD_CRYPTO_ENABLE : UTRD_CRYPTO_DISABLE;
	ufs_pixel_fips_build_utrd(hba, &utrd, bi->ucd_dma_addr, data_direction,
				  crypto);
	/* Build PRDT */
	ufs_pixel_fips_build_prdt(hba, bi->ucd_addr, bi->io_buffer_dma_addr,
				  buffer_len, key, iv);

	/* Build UPIU */
	ufs_pixel_fips_build_upiu(hba, bi->ucd_addr, cdb, flags, lu, buffer_len,
				  task_tag);

	/* Make sure descriptors are ready before ringing the task doorbell */
	wmb();

	/* Send */
	ret = ufs_pixel_fips_send_utrd(hba, &utrd, task_tag);
	if (!ret) {
		u8 ocs = le32_to_cpu(utrd.header.dword_2) & 0xFF;
		u8 upiu_status = be32_to_cpu(resp_upiu->header.dword_1) & 0xFF;
		u8 upiu_response =
			(be32_to_cpu(resp_upiu->header.dword_1) >> 8) & 0xFF;
		if (ocs || upiu_status || upiu_response) {
			pr_err("Request failed OCS=%02X status=%02X resp=%02X\n",
			       ocs, upiu_status, upiu_response);
			ret = -EIO;
		}
	}

	return ret;
}

static int ufs_pixel_fips_request_sense(struct ufs_hba *hba,
					struct fips_buffer_info *bi)
{
	struct scsi_cdb cdb = {};
	int ret;

	cdb.op_code = REQUEST_SENSE;
	cdb.transfer_len = cpu_to_be16(SENSE_DATA_ALLOC_LEN);

	ret = ufs_pixel_fips_send_request(hba, &cdb, bi, SENSE_DATA_ALLOC_LEN,
					  fips_lu, NULL, NULL);

	if (ret)
		return -EIO;

	return 0;
}

static int ufs_pixel_fips_send_io(struct ufs_hba *hba,
				  struct fips_buffer_info *bi, const u8 *key,
				  const u8 *iv, u8 op_code)
{
	struct scsi_cdb cdb = {};
	int ret;
	int retry = IO_RETRY_COUNT;

	cdb.op_code = op_code;
	cdb.lba = cpu_to_be32(fips_first_lba);
	cdb.transfer_len = cpu_to_be16(1);

	do {
		ret = ufs_pixel_fips_send_request(
			hba, &cdb, bi, UFS_PIXEL_BUFFER_SIZE, fips_lu, key, iv);
	} while (ret && retry-- > 0);

	if (ret)
		return -EIO;

	return 0;
}

static int ufs_pixel_fips_read(struct ufs_hba *hba, struct fips_buffer_info *bi,
			       const u8 *key, const u8 *iv)
{
	return ufs_pixel_fips_send_io(hba, bi, key, iv, READ_10);
}

static int ufs_pixel_fips_write(struct ufs_hba *hba,
				struct fips_buffer_info *bi, const u8 *key,
				const u8 *iv)
{
	return ufs_pixel_fips_send_io(hba, bi, key, iv, WRITE_10);
}

static const u8 pixel_fips_encryption_pt[] = {
	0x54, 0x68, 0x69, 0x73, 0x20, 0x69, 0x73, 0x20, /* "This is " */
	0x61, 0x20, 0x33, 0x32, 0x42, 0x20, 0x65, 0x6E, /* "a 32B en" */
	0x63, 0x72, 0x79, 0x70, 0x74, 0x20, 0x70, 0x74, /* "crypt pt" */
	0x20, 0x6D, 0x65, 0x73, 0x73, 0x61, 0x67, 0x65, /* " message" */
};

static const u8 pixel_fips_encryption_ct[] = {
	0x57, 0x4F, 0x6F, 0xD1, 0x21, 0x33, 0xE5, 0xF4,
	0x6F, 0x72, 0x30, 0x63, 0x8B, 0xCE, 0xA7, 0xD4,
	0x84, 0x84, 0xF9, 0xE2, 0xC5, 0xB9, 0xE4, 0x48,
	0x8D, 0x49, 0x02, 0x88, 0x80, 0xB3, 0x07, 0xE2,
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

int ufs_pixel_fips_verify(struct ufs_hba *hba)
{
	int ret;
	u32 interrupts;
	struct fips_buffer_info bi;

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
	memset(bi.io_buffer, 0, UFS_PIXEL_BUFFER_SIZE);
	memcpy(bi.io_buffer, pixel_fips_encryption_pt,
	       sizeof(pixel_fips_encryption_pt));

	ret = ufs_pixel_fips_write(hba, &bi, pixel_fips_encryption_key,
				   pixel_fips_encryption_iv);
	if (ret)
		goto out;

	memset(bi.io_buffer, 0, UFS_PIXEL_BUFFER_SIZE);

	ret = ufs_pixel_fips_read(hba, &bi, NULL, NULL);
	if (ret)
		goto out;

	if (memcmp(bi.io_buffer, pixel_fips_encryption_ct,
		   sizeof(pixel_fips_encryption_ct))) {
		pr_err("Encryption verification failed\n");
		ret = -EINVAL;
		goto out;
	}

	pr_info("Encryption verification passed\n");

	/*
	 * Verify Decryption:
	 * Since the ciphertext is already stored we just read back with the
	 * specified crypto parameters.
	 * Compare vs expected plaintext.
	 */
	memset(bi.io_buffer, 0, UFS_PIXEL_BUFFER_SIZE);

	ret = ufs_pixel_fips_read(hba, &bi, pixel_fips_encryption_key,
				  pixel_fips_encryption_iv);
	if (ret)
		goto out;

	if (memcmp(bi.io_buffer, pixel_fips_encryption_pt,
		   sizeof(pixel_fips_encryption_pt))) {
		pr_err("Decryption verification failed\n");
		ret = -EINVAL;
		goto out;
	}
	pr_info("Decryption verification passed\n");

out:
	ufshcd_writel(hba, interrupts, REG_INTERRUPT_ENABLE);
	ufshcd_release(hba);
	dma_free_coherent(hba->dev, UFS_PIXEL_UCD_SIZE, bi.ucd_addr,
			  bi.ucd_dma_addr);
	dma_free_coherent(hba->dev, UFS_PIXEL_BUFFER_SIZE, bi.io_buffer,
			  bi.io_buffer_dma_addr);

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

	ufs_pixel_fips_hmac_sha256(ufs_pixel_fips_hmac_message,
				   sizeof(ufs_pixel_fips_hmac_message),
				   ufs_pixel_fips_hmac_key,
				   sizeof(ufs_pixel_fips_hmac_key),
				   hmac_digest);

	return memcmp(hmac_digest, ufs_pixel_fips_hmac_expected,
		      UFS_PIXEL_FIPS_SHA256_DIGEST_SIZE);
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
		kfree(hmac_buffer);
		return ret;
	}

	ufs_pixel_fips_hmac_sha256(hmac_buffer, text_len + rodata_len,
				   fips140_integ_hmac_key,
				   strlen(fips140_integ_hmac_key), hmac_digest);

	kfree(hmac_buffer);

	return memcmp(hmac_digest, fips140_integ_hmac_digest,
		      UFS_PIXEL_FIPS_SHA256_DIGEST_SIZE);
}

static int __init ufs_pixel_fips_init(void)
{
	/*
	 * If the first LBA of FIPS partition is 0, SW key mode is disabled
	 * and the module doesn't have to do anything. Additional check will
	 * be performed in ufs_pixel_fips_verify().
	 */
	if (!fips_first_lba)
		return 0;

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

	return 0;
}

static void ufs_pixel_fips_exit(void)
{
}

module_init(ufs_pixel_fips_init);
module_exit(ufs_pixel_fips_exit);

MODULE_DESCRIPTION(
	"FIPS140-2 Compliant SW Driven UFS Inline Encryption Self Test Module");
MODULE_AUTHOR("Konstantin Vyshetsky");
MODULE_LICENSE("GPL v2");
