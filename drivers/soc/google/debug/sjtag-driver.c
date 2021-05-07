// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021 Google LLC
 *    Author: Peter Csaszar <pcsaszar@google.com>
 */

#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/gsa/gsa_sjtag.h>

#include <soc/google/debug-snapshot.h>
#include <soc/google/exynos-adv-tracer.h>

#define SJTAG_WORKBUFF_SIZE		512
#define SJTAG_FILEOP_STR_SIZE		40

#define SJTAG_PRODUCT_ID_SIZE		4
#define SJTAG_PKHASH_SIZE		64
#define SJTAG_STATUS_SIZE		4
#define SJTAG_CHIP_ID_SIZE		8
#define SJTAG_DBG_DOMAIN_SIZE		4
#define SJTAG_ACCESS_LVL_SIZE		8
#define SJTAG_DBG_ITVL_SIZE		4
#define SJTAG_DBG_TIME_SIZE		4
#define SJTAG_PUBKEY_SIZE		136
#define SJTAG_CHALLENGE_SIZE		64
#define SJTAG_SIGNATURE_SIZE		136

#define SJTAG_MAX_SIGFILE_SIZE		139
#define SJTAG_SIG_ASN1_TAG_SEQUENCE	0x30
#define SJTAG_SIG_ASN1_TAG_INTEGER	0x2
#define SJTAG_SIG_MAX_SECTION_SIZE	66
#define SJTAG_SIG_PADDED_SECTION_SIZE	68

enum sjtag_status_field_pos {
	SJTAG_STATUS_PROGRESS_POS = 6,
	SJTAG_STATUS_STATE_POS = 16,
	SJTAG_STATUS_MSG_POS = 20,
	SJTAG_STATUS_ERRCODE_POS = 28
};

enum sjtag_status_field_mask {
	SJTAG_STATUS_PROGRESS_MASK = 0x7,
	SJTAG_STATUS_STATE_MASK = 0x7,
	SJTAG_STATUS_MSG_MASK = 0x7,
	SJTAG_STATUS_ERRCODE_MASK = 0x7
};

enum sjtag_driver_error_code {
	SJTAG_DRV_ERROR_NONE = 0,
	SJTAG_DRV_ERROR_JTAG_ERROR,
	SJTAG_DRV_ERROR_WRONG_STATE,
};

enum sjtag_hw_instance {
	SJTAG_HW_AP = 0,
	SJTAG_HW_GSA,
	SJTAG_HW_NUM
};

static const char *sjtag_hw_progress[8] = {
	"None",
	"Ready for request",
	"Ready for response",
	"<3>",
	"Auth passed",
	"<5>",
	"<6>",
	"<7>"
};

static const char *sjtag_hw_state[8] = {
	"Idle",
	"Receiving request",
	"Generating challenge",
	"Receiving response",
	"Computing verification",
	"Finished",
	"End",
	"<7>"
};

static const char *sjtag_hw_msg[8] = {
	"None",
	"Bad PK",
	"TRNG error",
	"ECDSA fail",
	"<4>",
	"<5>",
	"Session expired",
	"Session terminated"
};

static const char *sjtag_driver_error[8] = {
	"None",
	"SJTAG error",
	"Wrong state",
	"SJTAG timeout",
	"<4>",
	"<5>",
	"<6>",
	"<7>"
};

enum sjtag_ipc_cmd_code {
	SJTAG_GET_PRODUCT_ID = 0,
	SJTAG_GET_PKHASH,
	SJTAG_BEGIN,
	SJTAG_GET_CHALLENGE,
	SJTAG_RUN_AUTH,
	SJTAG_END,
	SJTAG_GET_STATUS,
	SJTAG_GET_DBG_TIME,
	SJTAG_GET_PUBKEY
};

struct sjtag_ipc_cmd_entry {
	const char *str;
	u32 ap_id;
};

#define SJTAG_IPC_GEN(x)	{ #x, EAT_IPC_CMD_##x }

static const struct sjtag_ipc_cmd_entry sjtag_ipc_cmd_lut[] = {
	SJTAG_IPC_GEN(SJTAG_GET_PRODUCT_ID),
	SJTAG_IPC_GEN(SJTAG_GET_PKHASH),
	SJTAG_IPC_GEN(SJTAG_BEGIN),
	SJTAG_IPC_GEN(SJTAG_GET_CHALLENGE),
	SJTAG_IPC_GEN(SJTAG_RUN_AUTH),
	SJTAG_IPC_GEN(SJTAG_END),
	SJTAG_IPC_GEN(SJTAG_GET_STATUS),
	SJTAG_IPC_GEN(SJTAG_GET_DBG_TIME),
	SJTAG_IPC_GEN(SJTAG_GET_PUBKEY)
};

struct auth_tok_from_hw_t {
	char chip_id[SJTAG_CHIP_ID_SIZE];
	char challenge[SJTAG_CHALLENGE_SIZE];
};

struct auth_tok_t {
	char dbg_itvl[SJTAG_DBG_ITVL_SIZE];
	char access_lvl_1[SJTAG_ACCESS_LVL_SIZE / 2];
	char access_lvl_0[SJTAG_ACCESS_LVL_SIZE / 2];
	char dbg_domain[SJTAG_DBG_DOMAIN_SIZE];
	struct auth_tok_from_hw_t at_hw;
};

struct sig_auth_tok_t {
	char dbg_domain[SJTAG_DBG_DOMAIN_SIZE];
	char access_lvl[SJTAG_ACCESS_LVL_SIZE];
	char dbg_itvl[SJTAG_DBG_ITVL_SIZE];
	char sig[SJTAG_SIGNATURE_SIZE];
};

struct sjtag_dev_state {
	struct device *dev;
	struct device *gsa_dev;
	struct dentry *top_dir_entry;
	char *workbuff;
	const char *hw_instance;
	u32 ms_per_tick;
	u32 ipc_timeout_ms;
	char pubkey[SJTAG_PUBKEY_SIZE];
	char dbg_domain[SJTAG_DBG_DOMAIN_SIZE];
	char access_lvl[SJTAG_ACCESS_LVL_SIZE];
	char dbg_itvl[SJTAG_DBG_ITVL_SIZE];
};

static void sjtag_reverse_bytes(char *startptr, size_t length)
{
	int temp;
	char *endptr = startptr + length - 1;

	while (startptr < endptr) {
		temp = *startptr;
		*startptr++ = *endptr;
		*endptr-- = temp;
	}
}

static int sjtag_get_dbgc_hw_progress(u32 status)
{
	return (status >> SJTAG_STATUS_PROGRESS_POS) & SJTAG_STATUS_PROGRESS_MASK;
}

static int sjtag_get_dbgc_hw_state(u32 status)
{
	return (status >> SJTAG_STATUS_STATE_POS) & SJTAG_STATUS_STATE_MASK;
}

static int sjtag_get_dbgc_hw_message(u32 status)
{
	return (status >> SJTAG_STATUS_MSG_POS) & SJTAG_STATUS_MSG_MASK;
}

static int sjtag_get_dbgc_drv_errcode(u32 status)
{
	return (status >> SJTAG_STATUS_ERRCODE_POS) & SJTAG_STATUS_ERRCODE_MASK;
}

static int sjtag_ipc_cmd(struct sjtag_dev_state *st, int cmd_code, char *data_buff, int data_len,
		const char *file_op)
{
	int ipc_status;
	u32 cmd_status;

	const char *cmd_str = sjtag_ipc_cmd_lut[cmd_code].str;
	u32 ap_cmd_id = sjtag_ipc_cmd_lut[cmd_code].ap_id;

	if (!st->gsa_dev) {
		struct adv_tracer_ipc_cmd cmd = {
			.cmd_raw = {
				.cmd = ap_cmd_id,
				.size = 1,
			},
		};
		ipc_status = adv_tracer_ipc_send_data_polling_timeout(EAT_FRM_CHANNEL, &cmd,
				st->ipc_timeout_ms * NSEC_PER_MSEC);
		cmd_status = cmd.buffer[1];
	} else {
		switch (cmd_code) {
		case SJTAG_GET_PKHASH:
			ipc_status = gsa_sjtag_get_pub_key_hash(st->gsa_dev, data_buff, data_len,
					&cmd_status);
			break;
		case SJTAG_BEGIN:
			ipc_status = gsa_sjtag_set_pub_key(st->gsa_dev, data_buff, data_len,
					&cmd_status);
			break;
		case SJTAG_GET_CHALLENGE:
			ipc_status = gsa_sjtag_get_challenge(st->gsa_dev, data_buff, data_len,
					&cmd_status);
			break;
		case SJTAG_RUN_AUTH:
			ipc_status = gsa_sjtag_send_srv_response(st->gsa_dev, data_buff, data_len,
					&cmd_status);
			break;
		case SJTAG_END:
			ipc_status = gsa_sjtag_end_session(st->gsa_dev, &cmd_status);
			break;
		case SJTAG_GET_STATUS:
			ipc_status = gsa_sjtag_get_status(st->gsa_dev, NULL, &cmd_status, NULL);
			*(u32 *)data_buff = cmd_status;
			break;
		case SJTAG_GET_DBG_TIME:
			ipc_status = gsa_sjtag_get_status(st->gsa_dev, NULL, NULL, &cmd_status);
			*(u32 *)data_buff = cmd_status;	/* Used for Debug Time in this case */
			break;
		default:
			ipc_status = -EFAULT;
			break;
		}
	}

	if (ipc_status < 0) {
		pr_err("%s: %s failed - IPC status: %d\n", file_op, cmd_str, ipc_status);
		return -EIO;
	}

	if (sjtag_get_dbgc_drv_errcode(cmd_status) != SJTAG_DRV_ERROR_NONE) {
		pr_err("%s: %s: ERROR - Cmd status: 0x%08X\n", file_op, cmd_str, cmd_status);
		return -EPERM;
	}

	pr_debug("%s: %s: Success - Cmd status: 0x%08X\n", file_op, cmd_str, cmd_status);

	return 0;
}

static ssize_t sjtag_simple_read_from_hw(struct file *file, char __user *ubuf, size_t count,
		loff_t *ppos, int cmd_code, int byte_size)
{
	struct sjtag_dev_state *st = (struct sjtag_dev_state *)file->private_data;
	char *wbuff = st->workbuff;
	char file_op[SJTAG_FILEOP_STR_SIZE];
	uintptr_t dss_header_base = dbg_snapshot_get_item_vaddr("header");
	char *exchange_buff;
	int ipc_rc;
	int len;

	if (*ppos != 0)
		return 0;

	if (dss_header_base == 0)
		return -ENODEV;

	if (2 * byte_size > SJTAG_WORKBUFF_SIZE)
		return -EFAULT;

	scnprintf(file_op, SJTAG_FILEOP_STR_SIZE, "sjtag/%s/%s read", st->hw_instance,
			file->f_path.dentry->d_name.name);
	exchange_buff = (char *)dss_header_base + DSS_HDR_DBGC_EXCHG_BUFF_OFFS;

	ipc_rc = sjtag_ipc_cmd(st, cmd_code, exchange_buff, byte_size, file_op);
	if (ipc_rc < 0)
		return ipc_rc;

	bin2hex(wbuff, exchange_buff, byte_size);

	len = simple_read_from_buffer(ubuf, count, ppos, wbuff, 2 * byte_size);
	if (len < 0)
		pr_err("%s: Copy to ubuf failed\n", file_op);

	return len;
}

/* Device query */

static ssize_t sjtag_product_id_read(struct file *file, char __user *ubuf,
		size_t count, loff_t *ppos)
{
	return sjtag_simple_read_from_hw(file, ubuf, count, ppos, SJTAG_GET_PRODUCT_ID,
			SJTAG_PRODUCT_ID_SIZE);
}

static ssize_t sjtag_pkhash_read(struct file *file, char __user *ubuf,
		size_t count, loff_t *ppos)
{
	return sjtag_simple_read_from_hw(file, ubuf, count, ppos, SJTAG_GET_PKHASH,
			SJTAG_PKHASH_SIZE);
}

/* Auth flow */

static ssize_t sjtag_begin_write(struct file *file, const char __user *ubuf,
		size_t count, loff_t *ppos)
{
	struct sjtag_dev_state *st = (struct sjtag_dev_state *)file->private_data;
	char file_op[SJTAG_FILEOP_STR_SIZE];
	uintptr_t dss_header_base = dbg_snapshot_get_item_vaddr("header");
	char *exchange_buff;
	int ipc_rc;

	if (*ppos != 0)
		return -EINVAL;

	if (dss_header_base == 0)
		return -ENODEV;

	scnprintf(file_op, SJTAG_FILEOP_STR_SIZE, "sjtag/%s/%s write", st->hw_instance,
			file->f_path.dentry->d_name.name);
	exchange_buff = (char *)dss_header_base + DSS_HDR_DBGC_EXCHG_BUFF_OFFS;

	memcpy(exchange_buff, st->pubkey, SJTAG_PUBKEY_SIZE);

	ipc_rc = sjtag_ipc_cmd(st, SJTAG_BEGIN, exchange_buff, SJTAG_PUBKEY_SIZE, file_op);
	if (ipc_rc < 0)
		return ipc_rc;

	*ppos = count;
	return count;
}

static ssize_t sjtag_preauth_read(struct file *file, char __user *ubuf,
		size_t count, loff_t *ppos)
{
	struct sjtag_dev_state *st = (struct sjtag_dev_state *)file->private_data;
	char *wbuff = st->workbuff;
	char file_op[SJTAG_FILEOP_STR_SIZE];
	uintptr_t dss_header_base = dbg_snapshot_get_item_vaddr("header");
	char *exchange_buff;
	struct auth_tok_from_hw_t *auth_tok_from_hw;
	struct auth_tok_t *auth_tok;
	int ipc_rc;
	int len;

	if (*ppos != 0)
		return 0;

	if (dss_header_base == 0)
		return -ENODEV;

	scnprintf(file_op, SJTAG_FILEOP_STR_SIZE, "sjtag/%s/%s read", st->hw_instance,
			file->f_path.dentry->d_name.name);
	exchange_buff = (char *)dss_header_base + DSS_HDR_DBGC_EXCHG_BUFF_OFFS;

	ipc_rc = sjtag_ipc_cmd(st, SJTAG_GET_CHALLENGE, exchange_buff, sizeof(*auth_tok_from_hw),
			file_op);
	if (ipc_rc < 0)
		return ipc_rc;

	/*
	 * Assemble the auth token from the chip ID / challenge from the subsystem driver & the
	 * access parameters maintained by the kernel, and format it to be signed
	 */

	auth_tok_from_hw = (struct auth_tok_from_hw_t *)exchange_buff;
	auth_tok = (struct auth_tok_t *)exchange_buff;

	/* Bump up the section received from the subsystem driver */
	memmove(&auth_tok->at_hw, auth_tok_from_hw, sizeof(*auth_tok_from_hw));

	/* Copy the access parameters into the void (Access Level also needs longword-swap) */
	memcpy(auth_tok->dbg_itvl, st->dbg_itvl, SJTAG_DBG_ITVL_SIZE);
	memcpy(auth_tok->access_lvl_1, st->access_lvl + SJTAG_ACCESS_LVL_SIZE / 2,
			SJTAG_ACCESS_LVL_SIZE / 2);
	memcpy(auth_tok->access_lvl_0, st->access_lvl, SJTAG_ACCESS_LVL_SIZE / 2);
	memcpy(auth_tok->dbg_domain, st->dbg_domain, SJTAG_DBG_DOMAIN_SIZE);

	/* Reverse the entire blob */
	sjtag_reverse_bytes((char *)auth_tok, sizeof(*auth_tok));

	/* Convert raw binary to hexdump */
	bin2hex(wbuff, auth_tok, sizeof(*auth_tok));

	len = simple_read_from_buffer(ubuf, count, ppos, wbuff, 2 * sizeof(*auth_tok));
	if (len < 0)
		pr_err("%s: Copy to ubuf failed\n", file_op);

	return len;
}

static u32 parse_asn1_length(char **pptr)
{
	u32 short_len;
	u32 long_len = 0;

	short_len = *(*pptr)++;
	if (!(short_len & 0x80))
		return short_len;

	short_len &= 0x7f;
	while (short_len--) {
		long_len <<= 8;
		long_len |= *(*pptr)++;
	}

	return long_len;
}

static ssize_t sjtag_auth_write(struct file *file, const char __user *ubuf,
		size_t count, loff_t *ppos)
{
	struct sjtag_dev_state *st = (struct sjtag_dev_state *)file->private_data;
	char *wbuff = st->workbuff;
	char file_op[SJTAG_FILEOP_STR_SIZE];
	uintptr_t dss_header_base = dbg_snapshot_get_item_vaddr("header");
	char *exchange_buff;
	struct sig_auth_tok_t *sig_auth_tok;
	int ipc_rc;
	int len;
	char *sigptr;
	char *destptr;
	int section, byte;
	u32 block_len;

	if (*ppos != 0)
		return -EINVAL;

	if (dss_header_base == 0)
		return -ENODEV;

	scnprintf(file_op, SJTAG_FILEOP_STR_SIZE, "sjtag/%s/%s write", st->hw_instance,
			file->f_path.dentry->d_name.name);
	exchange_buff = (char *)dss_header_base + DSS_HDR_DBGC_EXCHG_BUFF_OFFS;

	/*
	 * Assemble the signed auth token from the access parameters maintained by the kernel
	 * & the signature of the auth token (The sig is in a format that requires  parsing &
	 * processing. No length or character check - using metadata in payload for validation)
	 */

	sig_auth_tok = (struct sig_auth_tok_t *)exchange_buff;

	/* Get the sig payload from userspace */
	len = simple_write_to_buffer(wbuff, 2 * SJTAG_MAX_SIGFILE_SIZE, ppos, ubuf, count);
	if (len < 0) {
		pr_err("%s: Copy from ubuf failed\n", file_op);
		return len;
	}
	len /= 2;

	/* Convert hexdump to raw binary */
	if (hex2bin(wbuff, wbuff, len)) {
		pr_err("%s: Invalid payload\n", file_op);
		return -EINVAL;
	}

	/* Start building the payload with the access parameters */
	memcpy(sig_auth_tok->dbg_domain, st->dbg_domain, SJTAG_DBG_DOMAIN_SIZE);
	memcpy(sig_auth_tok->access_lvl, st->access_lvl, SJTAG_ACCESS_LVL_SIZE);
	memcpy(sig_auth_tok->dbg_itvl, st->dbg_itvl, SJTAG_DBG_ITVL_SIZE);

	/* Parse the sig file header */
	sigptr = wbuff;
	destptr = sig_auth_tok->sig;

	if (*sigptr++ != SJTAG_SIG_ASN1_TAG_SEQUENCE) {
		pr_err("%s: Invalid sig payload type tag\n", file_op);
		return -EINVAL;
	}
	block_len = parse_asn1_length(&sigptr);
	if (block_len > len - (sigptr - wbuff)) {
		pr_err("%s: Inconsistent sig payload size\n", file_op);
		return -EINVAL;
	}

	for (section = 0; section < 2; section++) {
		/* Parse the section header */
		if (*sigptr++ != SJTAG_SIG_ASN1_TAG_INTEGER) {
			pr_err("%s: Invalid sig section type tag\n", file_op);
			return -EINVAL;
		}
		block_len = parse_asn1_length(&sigptr);
		if (block_len > SJTAG_SIG_MAX_SECTION_SIZE) {
			pr_err("%s: Invalid sig section size\n", file_op);
			return -EINVAL;
		}
		if (block_len > len - (sigptr - wbuff)) {
			pr_err("%s: Inconsistent sig section size\n", file_op);
			return -EINVAL;
		}

		sigptr += block_len;
		for (byte = 0; byte < block_len; byte++)
			*destptr++ = *--sigptr;
		for ( ; byte < SJTAG_SIG_PADDED_SECTION_SIZE; byte++)
			*destptr++ = 0;
		sigptr += block_len;
	}

	ipc_rc = sjtag_ipc_cmd(st, SJTAG_RUN_AUTH, exchange_buff, sizeof(*sig_auth_tok), file_op);
	if (ipc_rc < 0)
		return ipc_rc;

	pr_info("SJTAG::%s session started\n", st->hw_instance);

	return count;
}

static ssize_t sjtag_end_write(struct file *file, const char __user *ubuf,
		size_t count, loff_t *ppos)
{
	struct sjtag_dev_state *st = (struct sjtag_dev_state *)file->private_data;
	char file_op[SJTAG_FILEOP_STR_SIZE];
	int ipc_rc;

	if (*ppos != 0)
		return -EINVAL;

	scnprintf(file_op, SJTAG_FILEOP_STR_SIZE, "sjtag/%s/%s write", st->hw_instance,
			file->f_path.dentry->d_name.name);

	ipc_rc = sjtag_ipc_cmd(st, SJTAG_END, NULL, 0, file_op);
	if (ipc_rc < 0)
		return ipc_rc;

	pr_info("SJTAG::%s session ended\n", st->hw_instance);

	*ppos = count;
	return count;
}

/* Status query */

static ssize_t sjtag_status_read(struct file *file, char __user *ubuf,
		size_t count, loff_t *ppos)
{
	return sjtag_simple_read_from_hw(file, ubuf, count, ppos, SJTAG_GET_STATUS,
			SJTAG_STATUS_SIZE);
}

static ssize_t sjtag_status_h_read(struct file *file, char __user *ubuf,
		size_t count, loff_t *ppos)
{
	struct sjtag_dev_state *st = (struct sjtag_dev_state *)file->private_data;
	char *wbuff = st->workbuff;
	char file_op[SJTAG_FILEOP_STR_SIZE];
	uintptr_t dss_header_base = dbg_snapshot_get_item_vaddr("header");
	char *exchange_buff;
	int ipc_rc;
	int len;
	u32 status;

	if (*ppos != 0)
		return 0;

	if (dss_header_base == 0)
		return -ENODEV;

	scnprintf(file_op, SJTAG_FILEOP_STR_SIZE, "sjtag/%s/%s read", st->hw_instance,
			file->f_path.dentry->d_name.name);
	exchange_buff = (char *)dss_header_base + DSS_HDR_DBGC_EXCHG_BUFF_OFFS;

	ipc_rc = sjtag_ipc_cmd(st, SJTAG_GET_STATUS, exchange_buff, SJTAG_STATUS_SIZE, file_op);
	if (ipc_rc < 0)
		return ipc_rc;

	status = *(u32 *)exchange_buff;

	len = scnprintf(wbuff, SJTAG_WORKBUFF_SIZE,
			"Driver status: 0x%08X  -->  ", status);
	len += scnprintf(wbuff + len, SJTAG_WORKBUFF_SIZE - len,
			"Failure: %s,  Progress: %s,  State: %s,  Message: %s\n",
			sjtag_driver_error[sjtag_get_dbgc_drv_errcode(status)],
			sjtag_hw_progress[sjtag_get_dbgc_hw_progress(status)],
			sjtag_hw_state[sjtag_get_dbgc_hw_state(status)],
			sjtag_hw_msg[sjtag_get_dbgc_hw_message(status)]);

	len = simple_read_from_buffer(ubuf, count, ppos, wbuff, len);
	if (len < 0)
		pr_err("%s: Copy to ubuf failed\n", file_op);

	return len;
}

static ssize_t sjtag_dbg_time_read(struct file *file, char __user *ubuf,
		size_t count, loff_t *ppos)
{
	return sjtag_simple_read_from_hw(file, ubuf, count, ppos, SJTAG_GET_DBG_TIME,
			SJTAG_DBG_TIME_SIZE);
}

static ssize_t sjtag_dbg_time_h_read(struct file *file, char __user *ubuf,
		size_t count, loff_t *ppos)
{
	struct sjtag_dev_state *st = (struct sjtag_dev_state *)file->private_data;
	char *wbuff = st->workbuff;
	char file_op[SJTAG_FILEOP_STR_SIZE];
	uintptr_t dss_header_base = dbg_snapshot_get_item_vaddr("header");
	char *exchange_buff;
	int ipc_rc;
	int len;
	u32 dbg_time;

	if (*ppos != 0)
		return 0;

	if (dss_header_base == 0)
		return -ENODEV;

	scnprintf(file_op, SJTAG_FILEOP_STR_SIZE, "sjtag/%s/%s read", st->hw_instance,
			file->f_path.dentry->d_name.name);
	exchange_buff = (char *)dss_header_base + DSS_HDR_DBGC_EXCHG_BUFF_OFFS;

	ipc_rc = sjtag_ipc_cmd(st, SJTAG_GET_DBG_TIME, exchange_buff, SJTAG_DBG_TIME_SIZE, file_op);
	if (ipc_rc < 0)
		return ipc_rc;

	dbg_time = *(u32 *)exchange_buff;

	len = scnprintf(wbuff, SJTAG_WORKBUFF_SIZE, "Elapsed debug time: %d sec\n",
			dbg_time * st->ms_per_tick / 1000);

	len = simple_read_from_buffer(ubuf, count, ppos, wbuff, len);
	if (len < 0)
		pr_err("%s: Copy to ubuf failed\n", file_op);

	return len;
}

/* Auth debug */

static ssize_t sjtag_hw_pubkey_read(struct file *file, char __user *ubuf,
		size_t count, loff_t *ppos)
{
	return sjtag_simple_read_from_hw(file, ubuf, count, ppos, SJTAG_GET_PUBKEY,
			SJTAG_PUBKEY_SIZE);
}

/* Configurable param handling */

static ssize_t sjtag_read_configurable_param(struct file *file, char __user *ubuf,
		size_t count, loff_t *ppos, char *param, int byte_size)
{
	struct sjtag_dev_state *st = (struct sjtag_dev_state *)file->private_data;
	char *wbuff = st->workbuff;
	char file_op[SJTAG_FILEOP_STR_SIZE];
	int len;

	if (*ppos != 0)
		return 0;

	if (2 * byte_size > SJTAG_WORKBUFF_SIZE)
		return -EFAULT;

	scnprintf(file_op, SJTAG_FILEOP_STR_SIZE, "sjtag/%s/%s write", st->hw_instance,
			file->f_path.dentry->d_name.name);

	bin2hex(wbuff, param, byte_size);

	len = simple_read_from_buffer(ubuf, count, ppos, wbuff, 2 * byte_size);
	if (len < 0)
		pr_err("%s: Copy to ubuf failed\n", file_op);

	return len;
}

static ssize_t sjtag_write_configurable_param(struct file *file, const char __user *ubuf,
		size_t count, loff_t *ppos, char *param, int byte_size)
{
	struct sjtag_dev_state *st = (struct sjtag_dev_state *)file->private_data;
	char *wbuff = st->workbuff;
	char file_op[SJTAG_FILEOP_STR_SIZE];
	int len;

	if (*ppos != 0)
		return -EINVAL;

	if (2 * byte_size > SJTAG_WORKBUFF_SIZE)
		return -EFAULT;

	scnprintf(file_op, SJTAG_FILEOP_STR_SIZE, "sjtag/%s/%s write", st->hw_instance,
			file->f_path.dentry->d_name.name);

	if (count < 2 * byte_size) {
		pr_err("%s: Insufficient payload size %d\n", file_op, count);
		return -EINVAL;
	}

	len = simple_write_to_buffer(wbuff, 2 * byte_size, ppos, ubuf, count);
	if (len < 0) {
		pr_err("%s: Copy from ubuf failed\n", file_op);
		return len;
	}

	if (hex2bin(wbuff, wbuff, byte_size)) {
		pr_err("%s: Invalid payload\n", file_op);
		return -EINVAL;
	}

	memcpy(param, wbuff, byte_size);

	return count;
}

static ssize_t sjtag_pubkey_read(struct file *file, char __user *ubuf,
		size_t count, loff_t *ppos)
{
	struct sjtag_dev_state *st = (struct sjtag_dev_state *)file->private_data;

	return sjtag_read_configurable_param(file, ubuf, count, ppos, st->pubkey,
			SJTAG_PUBKEY_SIZE);
}

static ssize_t sjtag_pubkey_write(struct file *file, const char __user *ubuf,
		size_t count, loff_t *ppos)
{
	struct sjtag_dev_state *st = (struct sjtag_dev_state *)file->private_data;

	return sjtag_write_configurable_param(file, ubuf, count, ppos, st->pubkey,
			SJTAG_PUBKEY_SIZE);
}

static ssize_t sjtag_dbg_domain_read(struct file *file, char __user *ubuf,
		size_t count, loff_t *ppos)
{
	struct sjtag_dev_state *st = (struct sjtag_dev_state *)file->private_data;

	return sjtag_read_configurable_param(file, ubuf, count, ppos, st->dbg_domain,
			SJTAG_DBG_DOMAIN_SIZE);
}

static ssize_t sjtag_dbg_domain_write(struct file *file, const char __user *ubuf,
		size_t count, loff_t *ppos)
{
	struct sjtag_dev_state *st = (struct sjtag_dev_state *)file->private_data;

	return sjtag_write_configurable_param(file, ubuf, count, ppos, st->dbg_domain,
			SJTAG_DBG_DOMAIN_SIZE);
}

static ssize_t sjtag_access_lvl_read(struct file *file, char __user *ubuf,
		size_t count, loff_t *ppos)
{
	struct sjtag_dev_state *st = (struct sjtag_dev_state *)file->private_data;

	return sjtag_read_configurable_param(file, ubuf, count, ppos, st->access_lvl,
			SJTAG_ACCESS_LVL_SIZE);
}

static ssize_t sjtag_access_lvl_write(struct file *file, const char __user *ubuf,
		size_t count, loff_t *ppos)
{
	struct sjtag_dev_state *st = (struct sjtag_dev_state *)file->private_data;

	return sjtag_write_configurable_param(file, ubuf, count, ppos, st->access_lvl,
			SJTAG_ACCESS_LVL_SIZE);
}

static ssize_t sjtag_dbg_itvl_read(struct file *file, char __user *ubuf,
		size_t count, loff_t *ppos)
{
	struct sjtag_dev_state *st = (struct sjtag_dev_state *)file->private_data;

	return sjtag_read_configurable_param(file, ubuf, count, ppos, st->dbg_itvl,
			SJTAG_DBG_ITVL_SIZE);
}

static ssize_t sjtag_dbg_itvl_write(struct file *file, const char __user *ubuf,
		size_t count, loff_t *ppos)
{
	struct sjtag_dev_state *st = (struct sjtag_dev_state *)file->private_data;

	return sjtag_write_configurable_param(file, ubuf, count, ppos, st->dbg_itvl,
			SJTAG_DBG_ITVL_SIZE);
}

static const struct file_operations sjtag_product_id_fops = {
	.open	= simple_open,
	.read	= sjtag_product_id_read,
	.llseek	= default_llseek,
};

static const struct file_operations sjtag_pkhash_fops = {
	.open	= simple_open,
	.read	= sjtag_pkhash_read,
	.llseek	= default_llseek,
};

static const struct file_operations sjtag_begin_fops = {
	.open	= simple_open,
	.write	= sjtag_begin_write,
	.llseek	= default_llseek,
};

static const struct file_operations sjtag_preauth_fops = {
	.open	= simple_open,
	.read	= sjtag_preauth_read,
	.llseek	= default_llseek,
};

static const struct file_operations sjtag_auth_fops = {
	.open	= simple_open,
	.write	= sjtag_auth_write,
	.llseek	= default_llseek,
};

static const struct file_operations sjtag_end_fops = {
	.open	= simple_open,
	.write	= sjtag_end_write,
	.llseek	= default_llseek,
};

static const struct file_operations sjtag_status_fops = {
	.open	= simple_open,
	.read	= sjtag_status_read,
	.llseek	= default_llseek,
};

static const struct file_operations sjtag_status_h_fops = {
	.open	= simple_open,
	.read	= sjtag_status_h_read,
	.llseek	= default_llseek,
};

static const struct file_operations sjtag_dbg_time_fops = {
	.open	= simple_open,
	.read	= sjtag_dbg_time_read,
	.llseek	= default_llseek,
};

static const struct file_operations sjtag_dbg_time_h_fops = {
	.open	= simple_open,
	.read	= sjtag_dbg_time_h_read,
	.llseek	= default_llseek,
};

static const struct file_operations sjtag_hw_pubkey_fops = {
	.open	= simple_open,
	.read	= sjtag_hw_pubkey_read,
	.llseek	= default_llseek,
};

static const struct file_operations sjtag_pubkey_fops = {
	.open	= simple_open,
	.read	= sjtag_pubkey_read,
	.write	= sjtag_pubkey_write,
	.llseek	= default_llseek,
};

static const struct file_operations sjtag_dbg_domain_fops = {
	.open	= simple_open,
	.read	= sjtag_dbg_domain_read,
	.write	= sjtag_dbg_domain_write,
	.llseek	= default_llseek,
};

static const struct file_operations sjtag_access_lvl_fops = {
	.open	= simple_open,
	.read	= sjtag_access_lvl_read,
	.write	= sjtag_access_lvl_write,
	.llseek	= default_llseek,
};

static const struct file_operations sjtag_dbg_itvl_fops = {
	.open	= simple_open,
	.read	= sjtag_dbg_itvl_read,
	.write	= sjtag_dbg_itvl_write,
	.llseek	= default_llseek,
};

struct sjtag_file_info {
	char *fname;
	umode_t mode;
	const struct file_operations *fops;
	bool in_ap;
	bool in_gsa;
};

static const struct sjtag_file_info sjtag_files[] = {
	{ "product_id", 0444, &sjtag_product_id_fops, true, false },
	{ "pkhash", 0444, &sjtag_pkhash_fops, true, true },
	{ "begin", 0222, &sjtag_begin_fops, true, true },
	{ "preauth", 0444, &sjtag_preauth_fops, true, true },
	{ "auth", 0222, &sjtag_auth_fops, true, true },
	{ "end", 0222, &sjtag_end_fops, true, true },
	{ "status", 0444, &sjtag_status_fops, true, true },
	{ "status_h", 0444, &sjtag_status_h_fops, true, true },
	{ "dbg_time", 0444, &sjtag_dbg_time_fops, true, true },
	{ "dbg_time_h", 0444, &sjtag_dbg_time_h_fops, true, true },
	{ "hw_pubkey", 0444, &sjtag_hw_pubkey_fops, true, false },
	{ "pubkey", 0666, &sjtag_pubkey_fops, true, true },
	{ "dbg_domain", 0666, &sjtag_dbg_domain_fops, true, true },
	{ "access_lvl", 0666, &sjtag_access_lvl_fops, true, true },
	{ "dbg_itvl", 0666, &sjtag_dbg_itvl_fops, true, true }
};

static int sjtag_read_dt(struct platform_device *pdev)
{
	int rc;
	struct device_node *node = pdev->dev.of_node;
	struct sjtag_dev_state *st = pdev->dev.driver_data;

	rc = of_property_read_string(node, "hw-instance", &st->hw_instance);
	if (rc) {
		dev_err(&pdev->dev, "Missing or invalid instance name\n");
		return rc;
	}

	of_property_read_u32(node, "ms-per-tick", &st->ms_per_tick);
	of_property_read_u32(node, "ipc-timeout-ms", &st->ipc_timeout_ms);

	of_property_read_u8_array(node, "pubkey", st->pubkey, SJTAG_PUBKEY_SIZE);
	of_property_read_u32(node, "dbg-domain", (u32 *)st->dbg_domain);
	of_property_read_u64(node, "access-lvl", (u64 *)st->access_lvl);
	of_property_read_u32(node, "dbg-itvl", (u32 *)st->dbg_itvl);

	return 0;
}

static int sjtag_create_files(struct platform_device *pdev)
{
	int i;
	int rc;
	struct dentry *d_dent;
	struct dentry *f_dent;
	struct sjtag_dev_state *st = pdev->dev.driver_data;

	d_dent = debugfs_lookup("sjtag", NULL);
	if (!d_dent) {
		d_dent = debugfs_create_dir("sjtag", NULL);
		if (IS_ERR(d_dent)) {
			rc = PTR_ERR(d_dent);
			goto quit;
		}
	}
	st->top_dir_entry = d_dent;

	d_dent = debugfs_create_dir(st->hw_instance, d_dent);
	if (IS_ERR(d_dent)) {
		rc = PTR_ERR(d_dent);
		goto quit;
	}

	for (i = 0; i < ARRAY_SIZE(sjtag_files); i++) {
		if (!strcmp(st->hw_instance, "ap") ?
				sjtag_files[i].in_ap : sjtag_files[i].in_gsa) {
			f_dent = debugfs_create_file(sjtag_files[i].fname, sjtag_files[i].mode,
					d_dent, st, sjtag_files[i].fops);
			if (IS_ERR(f_dent)) {
				rc = PTR_ERR(f_dent);
				goto out;
			}
		}
	}

	return 0;

out:
	debugfs_remove_recursive(d_dent);
quit:
	dev_err(&pdev->dev, "Subdir init failed\n");

	return rc;
}

static void sjtag_release_gsa_device(void *rock)
{
	struct sjtag_dev_state *st = (struct sjtag_dev_state *)rock;

	put_device(st->gsa_dev);
}

static int sjtag_find_gsa_device(struct sjtag_dev_state *st)
{
	struct device_node *np;
	struct platform_device *gsa_pdev;

	if (!strcmp(st->hw_instance, "ap")) {
		st->gsa_dev = NULL;
		return 0;
	}

	np = of_parse_phandle(st->dev->of_node, "gsa-device", 0);
	if (!np) {
		dev_err(st->dev, "gsa-device phandle not found in SJTAG device tree node\n");
		return -ENODEV;
	}
	gsa_pdev = of_find_device_by_node(np);
	of_node_put(np);

	if (!gsa_pdev) {
		dev_err(st->dev, "gsa-device phandle doesn't refer to a device\n");
		return -ENODEV;
	}
	st->gsa_dev = &gsa_pdev->dev;

	return devm_add_action_or_reset(st->dev, sjtag_release_gsa_device, st);
}

/********************************************************************/

static int sjtag_probe(struct platform_device *pdev)
{
	int rc;
	struct device *dev = &pdev->dev;
	struct sjtag_dev_state *st;

	st = devm_kzalloc(dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->workbuff = devm_kmalloc(dev, SJTAG_WORKBUFF_SIZE, GFP_KERNEL);
	if (!st->workbuff)
		return -ENOMEM;

	st->dev = dev;
	platform_set_drvdata(pdev, st);

	rc = sjtag_read_dt(pdev);
	if (rc)
		return rc;

	rc = sjtag_create_files(pdev);
	if (rc)
		return rc;

	rc = sjtag_find_gsa_device(st);
	if (rc)
		return rc;

	dev_dbg(dev, "Initialized\n");

	return 0;
}

static int sjtag_remove(struct platform_device *pdev)
{
	struct sjtag_dev_state *st = platform_get_drvdata(pdev);

	debugfs_remove_recursive(st->top_dir_entry);
	st->top_dir_entry = NULL;

	return 0;
}

static const struct of_device_id sjtag_of_match[] = {
	{ .compatible = "google,sjtag", },
	{},
};
MODULE_DEVICE_TABLE(of, sjtag_of_match);

static struct platform_driver sjtag_driver = {
	.probe = sjtag_probe,
	.remove = sjtag_remove,
	.driver	= {
		.name = "sjtag",
		.of_match_table = sjtag_of_match,
	},
};

static int __init sjtag_driver_init(void)
{
	return platform_driver_register(&sjtag_driver);
}

static void __exit sjtag_driver_exit(void)
{
	platform_driver_unregister(&sjtag_driver);
}

MODULE_DESCRIPTION("Google SJTAG platform driver");
MODULE_LICENSE("GPL v2");
module_init(sjtag_driver_init);
module_exit(sjtag_driver_exit);
