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
#include <soc/google/sjtag-driver.h>

#define SJTAG_WORKBUFF_SIZE		512
#define SJTAG_FILEOP_STR_SIZE		20

#define SJTAG_PRODUCT_ID_SIZE		2
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

#define SJTAG_BEGIN_FORCE_STR		"force"

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

enum sjtag_status_bit {
	SJTAG_STATUS_SOFT_LOCK = 4,
	SJTAG_STATUS_AUTH_PASS = 8
};

enum sjtag_driver_error_code {
	SJTAG_DRV_ERROR_NONE = 0,
	SJTAG_DRV_ERROR_JTAG_ERROR,
	SJTAG_DRV_ERROR_WRONG_STATE,
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

enum sjtag_ipc_cmd_code_for_sysfs {
	sjtag_ipc_cmd_code_for_product_id = SJTAG_GET_PRODUCT_ID,
	sjtag_ipc_cmd_code_for_pkhash = SJTAG_GET_PKHASH,
	sjtag_ipc_cmd_code_for_status = SJTAG_GET_STATUS,
	sjtag_ipc_cmd_code_for_dbg_time = SJTAG_GET_DBG_TIME,
	sjtag_ipc_cmd_code_for_dbg_time_s = SJTAG_GET_DBG_TIME,
	sjtag_ipc_cmd_code_for_hw_pubkey = SJTAG_GET_PUBKEY
};

enum sjtag_ipc_payload_len_for_sysfs {
	sjtag_ipc_payload_len_for_product_id = SJTAG_PRODUCT_ID_SIZE,
	sjtag_ipc_payload_len_for_pkhash = SJTAG_PKHASH_SIZE,
	sjtag_ipc_payload_len_for_status = SJTAG_STATUS_SIZE,
	sjtag_ipc_payload_len_for_dbg_time = SJTAG_DBG_TIME_SIZE,
	sjtag_ipc_payload_len_for_dbg_time_s = SJTAG_DBG_TIME_SIZE,
	sjtag_ipc_payload_len_for_hw_pubkey = SJTAG_PUBKEY_SIZE
};

enum sjtag_cfg_param_id {
	sjtag_cfg_param_id_pubkey = 0,
	sjtag_cfg_param_id_dbg_domain,
	sjtag_cfg_param_id_access_lvl,
	sjtag_cfg_param_id_dbg_itvl,
	SJTAG_CFG_PARAM_NUM
};

enum sjtag_cfg_param_len {
	sjtag_cfg_param_len_pubkey = SJTAG_PUBKEY_SIZE,
	sjtag_cfg_param_len_dbg_domain = SJTAG_DBG_DOMAIN_SIZE,
	sjtag_cfg_param_len_access_lvl = SJTAG_ACCESS_LVL_SIZE,
	sjtag_cfg_param_len_dbg_itvl = SJTAG_DBG_ITVL_SIZE
};

enum sjtag_hw_instance {
	SJTAG_HW_AP = 0,
	SJTAG_HW_GSA,
	SJTAG_HW_NUM
};

enum sjtag_auth_status {
	SJTAG_AUTH_UNKNOWN = 0,
	SJTAG_AUTH_UNAUTHD,
	SJTAG_AUTH_AUTHD,
	SJTAG_AUTH_OPEN
};

enum sjtag_consent_status {
	SJTAG_CONSENT_UNKNOWN = 0,
	SJTAG_CONSENT_UNGRANTED,
	SJTAG_CONSENT_GRANTED
};

struct sjtag_ipc_cmd_entry {
	const char *str;
	u32 ap_id;
};

static int sjtag_consent = SJTAG_CONSENT_UNKNOWN;
static int sjtag_auth_cache[SJTAG_HW_NUM] = { SJTAG_AUTH_UNKNOWN, SJTAG_AUTH_UNKNOWN };

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
	int hw_id;
	char *workbuff;
	u32 ms_per_tick;
	u32 ipc_timeout_ms;
	char *cfg_params[SJTAG_CFG_PARAM_NUM];
};

static void reverse_bytes(char *startptr, size_t length)
{
	int temp;
	char *endptr = startptr + length - 1;

	while (startptr < endptr) {
		temp = *startptr;
		*startptr++ = *endptr;
		*endptr-- = temp;
	}
}

static void update_auth_cache(int hw_id, u32 cmd_sts, u32 consent_sts, int ipc_sts)
{
	/*
	 * Update the auth status cache. Note that user consent & SJTAG enforcement (SOFT_LOCK) are
	 * constant, therefore are not invalidated upon an IPC error (which also includes
	 * unsuccessful register reads on Debug Core) - however, the state of being authenticated
	 * is invalidated.
	 */

	if (ipc_sts >= 0) {
		if (consent_sts != ~0)
			sjtag_consent = consent_sts == 0 ?
					SJTAG_CONSENT_UNGRANTED : SJTAG_CONSENT_GRANTED;

		if (cmd_sts & BIT(SJTAG_STATUS_SOFT_LOCK)) {
			if (cmd_sts & BIT(SJTAG_STATUS_AUTH_PASS))
				sjtag_auth_cache[hw_id] = SJTAG_AUTH_AUTHD;
			else
				sjtag_auth_cache[hw_id] = SJTAG_AUTH_UNAUTHD;
		} else {
			sjtag_auth_cache[hw_id] = SJTAG_AUTH_OPEN;
		}
	} else {
		if (sjtag_auth_cache[hw_id] < SJTAG_AUTH_OPEN)
			sjtag_auth_cache[hw_id] = SJTAG_AUTH_UNKNOWN;
	}
}

static int get_dbgc_drv_errcode(u32 status)
{
	return (status >> SJTAG_STATUS_ERRCODE_POS) & SJTAG_STATUS_ERRCODE_MASK;
}

static int send_ipc_cmd(struct sjtag_dev_state *st, int cmd_code, char *data_buff, int data_len,
		const char *file_op)
{
	int ipc_status;
	u32 cmd_status = ~0;
	u32 consent_status = ~0;
	u32 dbg_time;
	char consent_status_str[10] = "";

	const char *cmd_str = sjtag_ipc_cmd_lut[cmd_code].str;

	if (st->hw_id == SJTAG_HW_AP || cmd_code == SJTAG_GET_PRODUCT_ID) {
		u32 ap_cmd_id = sjtag_ipc_cmd_lut[cmd_code].ap_id;
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
			ipc_status = gsa_sjtag_get_status(st->gsa_dev, &consent_status, &cmd_status,
					NULL);
			if (data_buff)
				*(u32 *)data_buff = cmd_status;
			break;
		case SJTAG_GET_DBG_TIME:
			ipc_status = gsa_sjtag_get_status(st->gsa_dev, NULL, &cmd_status,
					&dbg_time);
			*(u32 *)data_buff = dbg_time;
			break;
		default:
			ipc_status = -EFAULT;
			break;
		}
	}

	update_auth_cache(st->hw_id, cmd_status, consent_status, ipc_status);

	if (ipc_status < 0) {
		dev_err(st->dev, "%s: %s failed - IPC status: %d\n", file_op, cmd_str, ipc_status);
		return -EIO;
	}

	if (consent_status != ~0)
		scnprintf(consent_status_str, sizeof(consent_status_str), "-%X", consent_status);


	if (get_dbgc_drv_errcode(cmd_status) != SJTAG_DRV_ERROR_NONE) {
		dev_err(st->dev, "%s: %s: ERROR - Cmd status: 0x%08X%s\n", file_op,
				cmd_str, cmd_status, consent_status_str);
		return -EPERM;
	}

	dev_info(st->dev, "%s: %s: Success - Cmd status: 0x%08X%s\n", file_op, cmd_str, cmd_status,
			consent_status_str);

	return 0;
}

static ssize_t simple_read_from_hw(struct device *dev, struct device_attribute *dev_attr,
		char *buf, int ipc_cmd_code, int byte_size)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sjtag_dev_state *st = platform_get_drvdata(pdev);
	char file_op[SJTAG_FILEOP_STR_SIZE];
	uintptr_t dss_header_base = dbg_snapshot_get_item_vaddr("header");
	char *exchange_buff;
	int ipc_rc;

	if (dss_header_base == 0)
		return -ENODEV;

	if (2 * byte_size > PAGE_SIZE)
		return -EFAULT;

	scnprintf(file_op, SJTAG_FILEOP_STR_SIZE, "\"%s\" read", dev_attr->attr.name);
	exchange_buff = (char *)dss_header_base + DSS_HDR_DBGC_EXCHG_BUFF_OFFS;

	ipc_rc = send_ipc_cmd(st, ipc_cmd_code, exchange_buff, byte_size, file_op);
	if (ipc_rc < 0)
		return ipc_rc;

	/* Handle the exceptions */
	if (ipc_cmd_code == SJTAG_GET_PRODUCT_ID) {	/* Only bits 27:12 are kept */
		u32 *bare_product_id = (u32 *)exchange_buff;
		*bare_product_id = (*bare_product_id >> 12) & 0xffff;
	}

	/* Reverse byte order (exception: public key) */
	if (ipc_cmd_code != SJTAG_GET_PUBKEY)
		reverse_bytes(exchange_buff, byte_size);

	bin2hex(buf, exchange_buff, byte_size);

	return 2 * byte_size;
}

#define GEN_SIMPLE_READ_FROM_HW_HANDLER(_name) \
static ssize_t _name##_show(struct device *dev, struct device_attribute *dev_attr, char *buf) \
{ \
	return simple_read_from_hw(dev, dev_attr, buf, sjtag_ipc_cmd_code_for_##_name, \
			sjtag_ipc_payload_len_for_##_name); \
} \
static DEVICE_ATTR_RO(_name)

static ssize_t pretty_read_from_hw(struct device *dev, struct device_attribute *dev_attr,
		char *buf, int ipc_cmd_code, int byte_size)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sjtag_dev_state *st = platform_get_drvdata(pdev);
	char file_op[SJTAG_FILEOP_STR_SIZE];
	uintptr_t dss_header_base = dbg_snapshot_get_item_vaddr("header");
	char *exchange_buff;
	int ipc_rc;
	int len;

	if (dss_header_base == 0)
		return -ENODEV;

	scnprintf(file_op, SJTAG_FILEOP_STR_SIZE, "\"%s\" read", dev_attr->attr.name);
	exchange_buff = (char *)dss_header_base + DSS_HDR_DBGC_EXCHG_BUFF_OFFS;

	ipc_rc = send_ipc_cmd(st, ipc_cmd_code, exchange_buff, byte_size, file_op);
	if (ipc_rc < 0)
		return ipc_rc;

	if (ipc_cmd_code == SJTAG_GET_DBG_TIME) {
		u32 dbg_time = *(u32 *)exchange_buff;

		len = sysfs_emit(buf, "%d\n", dbg_time * st->ms_per_tick / 1000);
	} else {
		len = 0;
	}

	return len;
}

#define GEN_PRETTY_READ_FROM_HW_HANDLER(_name) \
static ssize_t _name##_show(struct device *dev, struct device_attribute *dev_attr, char *buf) \
{ \
	return pretty_read_from_hw(dev, dev_attr, buf, sjtag_ipc_cmd_code_for_##_name, \
			sjtag_ipc_payload_len_for_##_name); \
} \
static DEVICE_ATTR_RO(_name)

/* Device query */

GEN_SIMPLE_READ_FROM_HW_HANDLER(product_id);

GEN_SIMPLE_READ_FROM_HW_HANDLER(pkhash);

/* Auth flow */

static ssize_t begin_store(struct device *dev, struct device_attribute *dev_attr,
		const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sjtag_dev_state *st = platform_get_drvdata(pdev);
	char file_op[SJTAG_FILEOP_STR_SIZE];
	uintptr_t dss_header_base = dbg_snapshot_get_item_vaddr("header");
	char *exchange_buff;
	int ipc_rc;

	if (dss_header_base == 0)
		return -ENODEV;

	/*
	 * Don't start auth 1) if SJTAG not enforced or already auth'd (not necessary) OTHERWISE
	 *                  2) if consent not granted (not allowed)
	 */
	if (strncmp(buf, SJTAG_BEGIN_FORCE_STR, strlen(SJTAG_BEGIN_FORCE_STR))) {
		if (sjtag_auth_cache[st->hw_id] >= SJTAG_AUTH_AUTHD)
			return -EEXIST;
		if (sjtag_consent <= SJTAG_CONSENT_UNGRANTED)
			return -EINVAL;
	}

	scnprintf(file_op, SJTAG_FILEOP_STR_SIZE, "\"%s\" write", dev_attr->attr.name);
	exchange_buff = (char *)dss_header_base + DSS_HDR_DBGC_EXCHG_BUFF_OFFS;

	memcpy(exchange_buff, st->cfg_params[sjtag_cfg_param_id_pubkey], SJTAG_PUBKEY_SIZE);

	ipc_rc = send_ipc_cmd(st, SJTAG_BEGIN, exchange_buff, SJTAG_PUBKEY_SIZE, file_op);
	if (ipc_rc < 0)
		return ipc_rc;

	return count;
}
static DEVICE_ATTR_WO(begin);

static ssize_t preauth_show(struct device *dev, struct device_attribute *dev_attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sjtag_dev_state *st = platform_get_drvdata(pdev);
	char file_op[SJTAG_FILEOP_STR_SIZE];
	uintptr_t dss_header_base = dbg_snapshot_get_item_vaddr("header");
	char *exchange_buff;
	struct auth_tok_from_hw_t *auth_tok_from_hw;
	struct auth_tok_t *auth_tok;
	int ipc_rc;

	if (dss_header_base == 0)
		return -ENODEV;

	scnprintf(file_op, SJTAG_FILEOP_STR_SIZE, "\"%s\" read", dev_attr->attr.name);
	exchange_buff = (char *)dss_header_base + DSS_HDR_DBGC_EXCHG_BUFF_OFFS;

	ipc_rc = send_ipc_cmd(st, SJTAG_GET_CHALLENGE, exchange_buff, sizeof(*auth_tok_from_hw),
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
	memcpy(auth_tok->dbg_itvl,
			st->cfg_params[sjtag_cfg_param_id_dbg_itvl],
			SJTAG_DBG_ITVL_SIZE);
	memcpy(auth_tok->access_lvl_1,
			st->cfg_params[sjtag_cfg_param_id_access_lvl] + SJTAG_ACCESS_LVL_SIZE / 2,
			SJTAG_ACCESS_LVL_SIZE / 2);
	memcpy(auth_tok->access_lvl_0,
			st->cfg_params[sjtag_cfg_param_id_access_lvl],
			SJTAG_ACCESS_LVL_SIZE / 2);
	memcpy(auth_tok->dbg_domain,
			st->cfg_params[sjtag_cfg_param_id_dbg_domain],
			SJTAG_DBG_DOMAIN_SIZE);

	/* Reverse the entire blob */
	reverse_bytes((char *)auth_tok, sizeof(*auth_tok));

	/* Convert raw binary to hexdump */
	bin2hex(buf, auth_tok, sizeof(*auth_tok));

	return 2 * sizeof(*auth_tok);
}
static DEVICE_ATTR_RO(preauth);

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

static ssize_t auth_store(struct device *dev, struct device_attribute *dev_attr,
		const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sjtag_dev_state *st = platform_get_drvdata(pdev);
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

	if (dss_header_base == 0)
		return -ENODEV;

	scnprintf(file_op, SJTAG_FILEOP_STR_SIZE, "\"%s\" write", dev_attr->attr.name);
	exchange_buff = (char *)dss_header_base + DSS_HDR_DBGC_EXCHG_BUFF_OFFS;

	/*
	 * Assemble the signed auth token from the access parameters maintained by the kernel
	 * & the signature of the auth token. (The sig is in a format that requires  parsing &
	 * processing.)
	 */

	sig_auth_tok = (struct sig_auth_tok_t *)exchange_buff;
	if (count > 2 * SJTAG_MAX_SIGFILE_SIZE + 1)	/* +1: Newline character */
		dev_warn(dev, "%s: Larger-than-expected sig payload size\n", file_op);
	len = min((int)count / 2, SJTAG_MAX_SIGFILE_SIZE);

	/* Convert hexdump to raw binary */
	if (hex2bin(wbuff, buf, len)) {
		dev_err(dev, "%s: Invalid sig payload\n", file_op);
		return -EINVAL;
	}

	/* Start building the payload with the access parameters */
	memcpy(sig_auth_tok->dbg_domain, st->cfg_params[sjtag_cfg_param_id_dbg_domain],
			SJTAG_DBG_DOMAIN_SIZE);
	memcpy(sig_auth_tok->access_lvl, st->cfg_params[sjtag_cfg_param_id_access_lvl],
			SJTAG_ACCESS_LVL_SIZE);
	memcpy(sig_auth_tok->dbg_itvl, st->cfg_params[sjtag_cfg_param_id_dbg_itvl],
			SJTAG_DBG_ITVL_SIZE);

	/* Parse the sig file header */
	sigptr = wbuff;
	destptr = sig_auth_tok->sig;

	if (*sigptr++ != SJTAG_SIG_ASN1_TAG_SEQUENCE) {
		dev_err(dev, "%s: Invalid sig payload type tag\n", file_op);
		return -EINVAL;
	}
	block_len = parse_asn1_length(&sigptr);
	if (block_len > len - (sigptr - wbuff)) {
		dev_err(dev, "%s: Inconsistent sig payload size\n", file_op);
		return -EINVAL;
	}

	for (section = 0; section < 2; section++) {
		/* Parse the section header */
		if (*sigptr++ != SJTAG_SIG_ASN1_TAG_INTEGER) {
			dev_err(dev, "%s: Invalid sig section type tag\n", file_op);
			return -EINVAL;
		}
		block_len = parse_asn1_length(&sigptr);
		if (block_len > SJTAG_SIG_MAX_SECTION_SIZE) {
			dev_err(dev, "%s: Invalid sig section size\n", file_op);
			return -EINVAL;
		}
		if (block_len > len - (sigptr - wbuff)) {
			dev_err(dev, "%s: Inconsistent sig section size\n", file_op);
			return -EINVAL;
		}

		sigptr += block_len;
		for (byte = 0; byte < block_len; byte++)
			*destptr++ = *--sigptr;
		for ( ; byte < SJTAG_SIG_PADDED_SECTION_SIZE; byte++)
			*destptr++ = 0;
		sigptr += block_len;
	}

	ipc_rc = send_ipc_cmd(st, SJTAG_RUN_AUTH, exchange_buff, sizeof(*sig_auth_tok), file_op);
	if (ipc_rc < 0)
		return ipc_rc;

	dev_info(dev, "Authenticated session started\n");

	return count;
}
static DEVICE_ATTR_WO(auth);

static ssize_t end_store(struct device *dev, struct device_attribute *dev_attr,
		const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sjtag_dev_state *st = platform_get_drvdata(pdev);
	char file_op[SJTAG_FILEOP_STR_SIZE];
	int ipc_rc;

	scnprintf(file_op, SJTAG_FILEOP_STR_SIZE, "\"%s\" write", dev_attr->attr.name);

	ipc_rc = send_ipc_cmd(st, SJTAG_END, NULL, 0, file_op);
	if (ipc_rc < 0)
		return ipc_rc;

	dev_info(dev, "Authenticated session ended\n");

	return count;
}
static DEVICE_ATTR_WO(end);

/* Status query */

GEN_SIMPLE_READ_FROM_HW_HANDLER(status);

GEN_SIMPLE_READ_FROM_HW_HANDLER(dbg_time);
GEN_PRETTY_READ_FROM_HW_HANDLER(dbg_time_s);

static ssize_t consent_show(struct device *dev, struct device_attribute *dev_attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d", sjtag_consent);
}
static DEVICE_ATTR_RO(consent);

/* Auth debug */

GEN_SIMPLE_READ_FROM_HW_HANDLER(hw_pubkey);

/* Configurable param handling */

static ssize_t read_configurable_param(struct device *dev, struct device_attribute *dev_attr,
		char *buf, int param_id, int byte_size)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sjtag_dev_state *st = platform_get_drvdata(pdev);
	char *wbuff = st->workbuff;

	if (2 * byte_size > PAGE_SIZE)
		return -EFAULT;

	memcpy(wbuff, st->cfg_params[param_id], byte_size);

	/* Reverse byte order from little-endian to human-readable (exception: public key) */
	if (param_id != sjtag_cfg_param_id_pubkey)
		reverse_bytes(wbuff, byte_size);

	bin2hex(buf, wbuff, byte_size);

	return 2 * byte_size;
}

static ssize_t write_configurable_param(struct device *dev, struct device_attribute *dev_attr,
		const char *buf, size_t count, int param_id, int byte_size)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sjtag_dev_state *st = platform_get_drvdata(pdev);
	char *wbuff = st->workbuff;
	char file_op[SJTAG_FILEOP_STR_SIZE];

	if (2 * byte_size > SJTAG_WORKBUFF_SIZE)
		return -EFAULT;

	scnprintf(file_op, SJTAG_FILEOP_STR_SIZE, "\"%s\" write", dev_attr->attr.name);

	if (count < 2 * byte_size) {
		dev_err(dev, "%s: Insufficient payload size %d\n", file_op, count);
		return -EINVAL;
	}

	/* Convert into wbuff instead of param directly: preserve original content upon bad input */
	if (hex2bin(wbuff, buf, byte_size)) {
		dev_err(dev, "%s: Invalid payload\n", file_op);
		return -EINVAL;
	}

	/* Reverse byte order from human-readable to little-endian (exception: public key) */
	if (param_id != sjtag_cfg_param_id_pubkey)
		reverse_bytes(wbuff, byte_size);

	memcpy(st->cfg_params[param_id], wbuff, byte_size);

	return count;
}

#define GEN_CONFIGURABLE_PARAM_HANDLERS(_name) \
static ssize_t _name##_show(struct device *dev, struct device_attribute *dev_attr, char *buf) \
{ \
	return read_configurable_param(dev, dev_attr, buf, sjtag_cfg_param_id_##_name, \
			sjtag_cfg_param_len_##_name); \
} \
\
static ssize_t _name##_store(struct device *dev, struct device_attribute *dev_attr, \
		const char *buf, size_t count) \
{ \
	return write_configurable_param(dev, dev_attr, buf, count, sjtag_cfg_param_id_##_name, \
			sjtag_cfg_param_len_##_name); \
} \
static DEVICE_ATTR_RW(_name)

GEN_CONFIGURABLE_PARAM_HANDLERS(pubkey);
GEN_CONFIGURABLE_PARAM_HANDLERS(dbg_domain);
GEN_CONFIGURABLE_PARAM_HANDLERS(access_lvl);
GEN_CONFIGURABLE_PARAM_HANDLERS(dbg_itvl);

static struct attribute *sjtag_attrs[] = {
	&dev_attr_product_id.attr,
	&dev_attr_pkhash.attr,
	&dev_attr_begin.attr,
	&dev_attr_preauth.attr,
	&dev_attr_auth.attr,
	&dev_attr_end.attr,
	&dev_attr_status.attr,
	&dev_attr_dbg_time.attr,
	&dev_attr_dbg_time_s.attr,
	&dev_attr_consent.attr,
	&dev_attr_hw_pubkey.attr,
	&dev_attr_pubkey.attr,
	&dev_attr_dbg_domain.attr,
	&dev_attr_access_lvl.attr,
	&dev_attr_dbg_itvl.attr,
	NULL
};

static const struct attribute_group sjtag_group = {
	.name = "interface",
	.attrs = sjtag_attrs,
};

static const struct attribute_group *sjtag_groups[] = {
	&sjtag_group,
	NULL,
};

static int sjtag_read_dt(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct sjtag_dev_state *st = platform_get_drvdata(pdev);

	st->hw_id = strcmp(pdev->name, "sjtag_ap") ? SJTAG_HW_GSA : SJTAG_HW_AP;

	of_property_read_u32(node, "ms-per-tick", &st->ms_per_tick);
	of_property_read_u32(node, "ipc-timeout-ms", &st->ipc_timeout_ms);

	of_property_read_u8_array(node, "pubkey",
			st->cfg_params[sjtag_cfg_param_id_pubkey], SJTAG_PUBKEY_SIZE);
	of_property_read_u32(node, "dbg-domain",
			(u32 *)st->cfg_params[sjtag_cfg_param_id_dbg_domain]);
	of_property_read_u64(node, "access-lvl",
			(u64 *)st->cfg_params[sjtag_cfg_param_id_access_lvl]);
	of_property_read_u32(node, "dbg-itvl",
			(u32 *)st->cfg_params[sjtag_cfg_param_id_dbg_itvl]);

	return 0;
}

static void sjtag_release_gsa_device(void *rock)
{
	struct sjtag_dev_state *st = (struct sjtag_dev_state *)rock;

	put_device(st->gsa_dev);
}

static int sjtag_find_gsa_device(struct platform_device *pdev)
{
	struct sjtag_dev_state *st = platform_get_drvdata(pdev);
	struct device_node *np;
	struct platform_device *gsa_pdev;

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
	int i;
	int rc;
	struct device *dev = &pdev->dev;
	struct sjtag_dev_state *st;
	int cfg_param_sizes[SJTAG_CFG_PARAM_NUM] = { SJTAG_PUBKEY_SIZE, SJTAG_DBG_DOMAIN_SIZE,
			SJTAG_ACCESS_LVL_SIZE, SJTAG_DBG_ITVL_SIZE };

	st = devm_kzalloc(dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->workbuff = devm_kmalloc(dev, SJTAG_WORKBUFF_SIZE, GFP_KERNEL);
	if (!st->workbuff)
		return -ENOMEM;

	for (i = 0; i < SJTAG_CFG_PARAM_NUM; i++) {
		st->cfg_params[i] = devm_kzalloc(dev, cfg_param_sizes[i], GFP_KERNEL);
		if (!st->cfg_params[i])
			return -ENOMEM;
	}

	st->dev = dev;
	platform_set_drvdata(pdev, st);

	rc = sjtag_read_dt(pdev);
	if (rc)
		return rc;

	rc = sjtag_find_gsa_device(pdev);
	if (rc)
		return rc;

	send_ipc_cmd(st, SJTAG_GET_STATUS, NULL, SJTAG_STATUS_SIZE, "[probe sts read]");

	dev_dbg(dev, "Initialized\n");

	return 0;
}

static int sjtag_remove(struct platform_device *pdev)
{
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
		.dev_groups = sjtag_groups
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

int sjtag_is_locked(void)
{
	/* Locked: consent granted but SJTAG not auth'd OR consent not granted and SJTAG enforced */
	return sjtag_auth_cache[SJTAG_HW_AP] <=
			(sjtag_consent == SJTAG_CONSENT_GRANTED ?
					SJTAG_AUTH_UNAUTHD : SJTAG_AUTH_AUTHD);
}
EXPORT_SYMBOL_GPL(sjtag_is_locked);

MODULE_DESCRIPTION("Google SJTAG platform driver");
MODULE_LICENSE("GPL v2");
module_init(sjtag_driver_init);
module_exit(sjtag_driver_exit);
