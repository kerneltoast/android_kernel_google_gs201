// SPDX-License-Identifier: GPL-2.0-only
/*
 * Platform device driver for the Google GSA core.
 *
 * Copyright (C) 2020 Google LLC
 */
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <linux/gsa/gsa_aoc.h>
#include <linux/gsa/gsa_kdn.h>
#include <linux/gsa/gsa_sjtag.h>
#include <linux/gsa/gsa_tpu.h>
#include "gsa_mbox.h"
#include "gsa_priv.h"
#include "gsa_tz.h"
#include "hwmgr-ipc.h"

struct gsa_dev_state {
	struct device *dev;
	struct gsa_mbox *mb;
	dma_addr_t bb_da;
	void *bb_va;
	size_t bb_sz;
	struct mutex bb_lock; /* protects access to bounce buffer */
	struct gsa_tz_chan_ctx aoc_srv;
	struct gsa_tz_chan_ctx tpu_srv;
};

/*
 *  Internal command interface
 */
int gsa_send_cmd(struct device *dev, u32 cmd, u32 *req, u32 req_argc,
		 u32 *rsp, u32 rsp_argc)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gsa_dev_state *s = platform_get_drvdata(pdev);

	return gsa_send_mbox_cmd(s->mb, cmd, req, req_argc, rsp, rsp_argc);
};
EXPORT_SYMBOL_GPL(gsa_send_cmd);

int gsa_send_simple_cmd(struct device *dev, u32 cmd)
{
	return gsa_send_cmd(dev, cmd, NULL, 0, NULL, 0);
}
EXPORT_SYMBOL_GPL(gsa_send_simple_cmd);

int gsa_send_one_arg_cmd(struct device *dev, u32 cmd, u32 arg)
{
	return gsa_send_cmd(dev, cmd, &arg, 1, NULL, 0);
}
EXPORT_SYMBOL_GPL(gsa_send_one_arg_cmd);

static int gsa_send_load_img_cmd(struct device *dev, uint32_t cmd,
				 dma_addr_t hdr_da, phys_addr_t body_pa)
{
	u32 req[4];
	struct platform_device *pdev = to_platform_device(dev);
	struct gsa_dev_state *s = platform_get_drvdata(pdev);

	req[IMG_LOADER_HEADER_ADDR_LO_IDX] = (u32)hdr_da;
	req[IMG_LOADER_HEADER_ADDR_HI_IDX] = (u32)(hdr_da >> 32);
	req[IMG_LOADER_BODY_ADDR_LO_IDX] = (u32)body_pa;
	req[IMG_LOADER_BODY_ADDR_HI_IDX] = (u32)(body_pa >> 32);

	return gsa_send_mbox_cmd(s->mb, cmd, req, 4, NULL, 0);
}

static int gsa_tz_send_hwmgr_state_cmd(struct gsa_tz_chan_ctx *ctx, u32 cmd)
{
	int rc;
	struct {
		struct hwmgr_rsp_hdr hdr;
		struct hwmgr_state_cmd_rsp rsp;
	} rsp_msg;

	struct {
		struct hwmgr_req_hdr hdr;
		struct hwmgr_state_cmd_req req;
	} req_msg;


	req_msg.hdr.cmd = HWMGR_CMD_STATE_CMD;
	req_msg.req.cmd = cmd;

	rc = gsa_tz_chan_msg_xchg(ctx,
				  &req_msg, sizeof(req_msg),
				  &rsp_msg, sizeof(rsp_msg));

	if (rc != sizeof(rsp_msg)) {
		return -EIO;
	}

	if (rsp_msg.hdr.cmd != (req_msg.hdr.cmd | HWMGR_CMD_RESP)) {
		return -EIO;
	}

	if (rsp_msg.hdr.err) {
		return -EIO;
	}

	return rsp_msg.rsp.state;
}

static int gsa_tz_send_hwmgr_unload_fw_image_cmd(struct gsa_tz_chan_ctx *ctx)
{
	int rc;
	struct {
		struct hwmgr_rsp_hdr hdr;
	} rsp_msg;

	struct {
		struct hwmgr_req_hdr hdr;
	} req_msg;


	req_msg.hdr.cmd = HWMGR_CMD_UNLOAD_IMG;

	rc = gsa_tz_chan_msg_xchg(ctx,
				  &req_msg, sizeof(req_msg),
				  &rsp_msg, sizeof(rsp_msg));

	if (rc != sizeof(rsp_msg)) {
		return -EIO;
	}

	if (rsp_msg.hdr.cmd != (req_msg.hdr.cmd | HWMGR_CMD_RESP)) {
		return -EIO;
	}

	if (rsp_msg.hdr.err) {
		return -EIO;
	}

	return 0;
}

/*
 *  External AOC interface
 */
int gsa_load_aoc_fw_image(struct device *gsa,
			  dma_addr_t img_meta,
			  phys_addr_t img_body)
{
	return gsa_send_load_img_cmd(gsa, GSA_MB_CMD_LOAD_AOC_FW_IMG,
				     img_meta, img_body);
}
EXPORT_SYMBOL_GPL(gsa_load_aoc_fw_image);

int gsa_unload_aoc_fw_image(struct device *gsa)
{
	struct platform_device *pdev = to_platform_device(gsa);
	struct gsa_dev_state *s = platform_get_drvdata(pdev);

	return gsa_tz_send_hwmgr_unload_fw_image_cmd(&s->aoc_srv);
}
EXPORT_SYMBOL_GPL(gsa_unload_aoc_fw_image);

int gsa_send_aoc_cmd(struct device *gsa, enum gsa_aoc_cmd arg)
{
	struct platform_device *pdev = to_platform_device(gsa);
	struct gsa_dev_state *s = platform_get_drvdata(pdev);

	return gsa_tz_send_hwmgr_state_cmd(&s->aoc_srv, arg);
}
EXPORT_SYMBOL_GPL(gsa_send_aoc_cmd);

/*
 *  External TPU interface
 */
int gsa_load_tpu_fw_image(struct device *gsa,
			  dma_addr_t img_meta,
			  phys_addr_t img_body)
{
	return gsa_send_load_img_cmd(gsa, GSA_MB_CMD_LOAD_TPU_FW_IMG,
				     img_meta, img_body);
}
EXPORT_SYMBOL_GPL(gsa_load_tpu_fw_image);

int gsa_unload_tpu_fw_image(struct device *gsa)
{
	struct platform_device *pdev = to_platform_device(gsa);
	struct gsa_dev_state *s = platform_get_drvdata(pdev);

	return gsa_tz_send_hwmgr_unload_fw_image_cmd(&s->tpu_srv);
}
EXPORT_SYMBOL_GPL(gsa_unload_tpu_fw_image);

int gsa_send_tpu_cmd(struct device *gsa, enum gsa_tpu_cmd arg)
{
	struct platform_device *pdev = to_platform_device(gsa);
	struct gsa_dev_state *s = platform_get_drvdata(pdev);

	return gsa_tz_send_hwmgr_state_cmd(&s->tpu_srv, arg);
}
EXPORT_SYMBOL_GPL(gsa_send_tpu_cmd);


/*
 *  External KDN interface
 */
static int send_kdn_cmd(struct gsa_dev_state *s, u32 cmd,
			void *dst_buf, size_t dst_buf_sz, u32 opts,
			const void *src_data, size_t src_data_len)
{
	int ret;
	size_t cb;
	u32 req[KDN_REQ_ARGC];
	u32 rsp[KDN_RSP_ARGC];

	if (dst_buf_sz) {
		if (!dst_buf) {
			/* invalid args */
			return -EINVAL;
		}
		if (dst_buf_sz > s->bb_sz) {
			/* too much data */
			return -EINVAL;
		}
	}

	/* copy in data */
	if (src_data_len) {
		if (!src_data) {
			/* invalid args */
			return -EINVAL;
		}

		if (src_data_len > s->bb_sz) {
			/* too much data */
			return -EINVAL;
		}

		memcpy(s->bb_va, src_data, src_data_len);
	}

	/* Invoke KDN command */
	req[KDN_DATA_BUF_ADDR_LO_IDX] = (u32)s->bb_da;
	req[KDN_DATA_BUF_ADDR_HI_IDX] = (u32)(s->bb_da >> 32);
	req[KDN_DATA_BUF_SIZE_IDX] = max_t(u32, dst_buf_sz, src_data_len);
	req[KDN_DATA_LEN_IDX] = (u32)src_data_len;
	req[KDN_OPTION_IDX] = opts;

	ret = gsa_send_mbox_cmd(s->mb, cmd, req, ARRAY_SIZE(req),
				rsp, ARRAY_SIZE(rsp));
	if (ret < 0) {
		/* mailbox command failed */
		return ret;
	}

	if (ret != KDN_RSP_ARGC) {
		/* unexpected reply */
		return -EINVAL;
	}

	/* copy data out */
	cb = rsp[KDN_RSP_DATA_LEN_IDX];

	if (cb > dst_buf_sz) {
		/* buffer too short */
		return -EINVAL;
	}

	if (cb) {
		/* copy data to destination buffer */
		memcpy(dst_buf, s->bb_va, cb);
	}

	return cb;
}

int gsa_kdn_derive_raw_secret(struct device *gsa, void *buf, size_t buf_sz,
			      const void *key_blob, size_t key_blob_len)
{
	int ret;
	struct platform_device *pdev = to_platform_device(gsa);
	struct gsa_dev_state *s = platform_get_drvdata(pdev);

	mutex_lock(&s->bb_lock);
	ret = send_kdn_cmd(s, GSA_MB_CMD_KDN_DERIVE_RAW_SECRET,
			   buf, buf_sz, 0, key_blob, key_blob_len);
	mutex_unlock(&s->bb_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(gsa_kdn_derive_raw_secret);

int gsa_kdn_program_key(struct device *gsa, u32 slot, const void *key_blob,
			size_t key_blob_len)
{
	int ret;
	struct platform_device *pdev = to_platform_device(gsa);
	struct gsa_dev_state *s = platform_get_drvdata(pdev);

	mutex_lock(&s->bb_lock);
	ret = send_kdn_cmd(s, GSA_MB_CMD_KDN_PROGRAM_KEY,
			   NULL, 0, slot, key_blob, key_blob_len);
	mutex_unlock(&s->bb_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(gsa_kdn_program_key);

int gsa_kdn_restore_keys(struct device *gsa)
{
	int ret;

	/* Restore keys is a special no argument command */
	ret = gsa_send_cmd(gsa, GSA_MB_CMD_KDN_RESTORE_KEYS, NULL, 0, NULL, 0);
	if (ret < 0)
		return ret;
	else
		return 0;
}
EXPORT_SYMBOL_GPL(gsa_kdn_restore_keys);


int gsa_kdn_set_operating_mode(struct device *gsa,
			       enum kdn_op_mode mode,
			       enum kdn_ufs_descr_type descr)
{
	int ret;
	u32 req[KDN_SET_OP_MODE_ARGC];

	req[KDN_SET_OP_MODE_MODE_IDX] = mode;
	req[KDN_SET_OP_MODE_UFS_DESCR_IDX] = descr;

	ret = gsa_send_cmd(gsa, GSA_MB_CMD_KDN_SET_OP_MODE,
			   req, ARRAY_SIZE(req), NULL, 0);
	if (ret < 0)
		return ret;
	else
		return 0;
}
EXPORT_SYMBOL_GPL(gsa_kdn_set_operating_mode);

/*
 *   External SJTAG management interface
 */
static int send_sjtag_data_cmd(struct gsa_dev_state *s, u32 cmd,
			       void *dst_buf, size_t dst_buf_sz,
			       const void *src_data, size_t src_data_len,
			       u32 *status)
{
	int ret;
	size_t cb;
	u32 req[SJTAG_DATA_REQ_ARGC];
	u32 rsp[SJTAG_DATA_RSP_ARGC];

	if (dst_buf_sz) {
		if (!dst_buf) {
			/* invalid args */
			return -EINVAL;
		}
		if (dst_buf_sz > s->bb_sz) {
			/* too much data */
			return -EINVAL;
		}
	}

	/* copy in data */
	if (src_data_len) {
		if (!src_data) {
			/* invalid args */
			return -EINVAL;
		}

		if (src_data_len > s->bb_sz) {
			/* too much data */
			return -EINVAL;
		}

		memcpy(s->bb_va, src_data, src_data_len);
	}

	/* Invoke SJTAG command */
	req[SJTAG_DATA_BUF_ADDR_LO_IDX] = (u32)s->bb_da;
	req[SJTAG_DATA_BUF_ADDR_HI_IDX] = (u32)(s->bb_da >> 32);
	req[SJTAG_DATA_BUF_SIZE_IDX] = max_t(u32, dst_buf_sz, src_data_len);
	req[SJTAG_DATA_LEN_IDX] = (u32)src_data_len;

	ret = gsa_send_mbox_cmd(s->mb, cmd, req, ARRAY_SIZE(req),
				rsp, ARRAY_SIZE(rsp));
	if (ret < 0) {
		/* mailbox command failed */
		return ret;
	}

	if (ret != SJTAG_DATA_RSP_ARGC) {
		/* unexpected reply */
		return -EINVAL;
	}

	/* return command status */
	if (status)
		*status = rsp[SJTAG_DATA_RSP_STATUS_IDX];

	/* copy data out */
	cb = rsp[SJTAG_DATA_RSP_DATA_LEN_IDX];

	if (cb > dst_buf_sz) {
		/* buffer too short */
		return -EINVAL;
	}

	if (cb) {
		/* copy data to destination buffer */
		memcpy(dst_buf, s->bb_va, cb);
	}

	return cb;
}

int gsa_sjtag_get_status(struct device *gsa, u32 *debug_allowed, u32 *hw_state,
			 u32 *debug_time)
{
	int ret;
	u32 rsp[SJTAG_STATUS_RSP_ARGC];
	struct gsa_dev_state *s;
	struct platform_device *pdev;

	pdev = to_platform_device(gsa);
	s = platform_get_drvdata(pdev);

	ret = gsa_send_cmd(gsa, GSA_MB_CMD_SJTAG_GET_STATUS, NULL, 0,
			   rsp, ARRAY_SIZE(rsp));
	if (ret < 0)
		return ret;

	/* we expect exactly 2 parameters */
	if (ret != SJTAG_STATUS_RSP_ARGC)
		return -EIO;

	if (debug_allowed)
		*debug_allowed= rsp[SJTAG_STATUS_RSP_DEBUG_ALLOWED_IDX];

	if (hw_state)
		*hw_state = rsp[SJTAG_STATUS_RSP_HW_STATUS_IDX];

	if (debug_time)
		*debug_time = rsp[SJTAG_STATUS_RSP_DEBUG_TIME_IDX];

	return 0;
}
EXPORT_SYMBOL_GPL(gsa_sjtag_get_status);

int gsa_sjtag_get_chip_id(struct device *gsa, u32 id[2])
{
	int ret;

	ret = gsa_send_cmd(gsa, GSA_MB_CMD_SJTAG_GET_CHIP_ID, NULL, 0, id, 2);
	if (ret < 0)
		return ret;

	/* we expect exactly 2 parameters */
	if (ret != 2)
		return -EIO;

	return 0;
}
EXPORT_SYMBOL_GPL(gsa_sjtag_get_chip_id);

int gsa_sjtag_get_pub_key_hash(struct device *gsa, void *hash, size_t size,
			       u32 *status)
{
	int ret;
	struct gsa_dev_state *s;
	struct platform_device *pdev;

	pdev = to_platform_device(gsa);
	s = platform_get_drvdata(pdev);

	mutex_lock(&s->bb_lock);
	ret = send_sjtag_data_cmd(s, GSA_MB_CMD_SJTAG_GET_PUB_KEY_HASH,
				  hash, size, NULL, 0, status);
	mutex_unlock(&s->bb_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(gsa_sjtag_get_pub_key_hash);

int gsa_sjtag_set_pub_key(struct device *gsa, const void *key, size_t size,
			  u32 *status)
{
	int ret;
	struct gsa_dev_state *s;
	struct platform_device *pdev;

	pdev = to_platform_device(gsa);
	s = platform_get_drvdata(pdev);

	mutex_lock(&s->bb_lock);
	ret = send_sjtag_data_cmd(s, GSA_MB_CMD_SJTAG_SET_PUB_KEY,
				  NULL, 0, key, size, status);
	mutex_unlock(&s->bb_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(gsa_sjtag_set_pub_key);

int gsa_sjtag_get_challenge(struct device *gsa, void *challenge, size_t size,
			    u32 *status)
{
	int ret;
	struct gsa_dev_state *s;
	struct platform_device *pdev;

	pdev = to_platform_device(gsa);
	s = platform_get_drvdata(pdev);

	mutex_lock(&s->bb_lock);
	ret = send_sjtag_data_cmd(s, GSA_MB_CMD_SJTAG_GET_CHALLENGE,
				  challenge, size, NULL, 0, status);
	mutex_unlock(&s->bb_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(gsa_sjtag_get_challenge);

int gsa_sjtag_send_srv_response(struct device *gsa,
				const void *rsp, size_t size,
				u32 *status)
{
	int ret;
	struct gsa_dev_state *s;
	struct platform_device *pdev;

	pdev = to_platform_device(gsa);
	s = platform_get_drvdata(pdev);

	mutex_lock(&s->bb_lock);
	ret = send_sjtag_data_cmd(s, GSA_MB_CMD_SJTAG_ENABLE,
				  NULL, 0, rsp, size, status);
	mutex_unlock(&s->bb_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(gsa_sjtag_send_srv_response);

int gsa_sjtag_end_session(struct device *gsa, u32 *status)
{
	int rc;
	struct gsa_dev_state *s;
	struct platform_device *pdev;

	pdev = to_platform_device(gsa);
	s = platform_get_drvdata(pdev);

	rc = gsa_send_cmd(gsa, GSA_MB_CMD_SJTAG_FINISH, NULL, 0, status, 1);
	if (rc < 0)
		return rc;

	/* exactly 1 argument is expected  */
	if (rc != 1)
		return -EIO;

	return 0;
}
EXPORT_SYMBOL_GPL(gsa_sjtag_end_session);

/********************************************************************/

static int gsa_probe(struct platform_device *pdev)
{
	int err;
	struct gsa_dev_state *s;
	struct device *dev = &pdev->dev;

	s = devm_kzalloc(dev, sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;

	s->dev = dev;
	mutex_init(&s->bb_lock);
	platform_set_drvdata(pdev, s);

	/*
	 * Set DMA mask and coherent to 32-bit as ATM we cannot use sysmmu.
	 * The main problem is that GSA can sleep independently from Main AP
	 * and if GSA is in sleep sysmmu block is powered off.
	 */
	err = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (err) {
		dev_err(dev, "failed (%d) to setup dma mask\n", err);
		return err;
	}

	/* initialize mailbox */
	s->mb = gsa_mbox_init(pdev);
	if (IS_ERR(s->mb))
		return -ENOMEM;

	/* add children */
	err = devm_of_platform_populate(dev);
	if (err < 0) {
		dev_err(dev, "populate children failed (%d)\n", err);
		return err;
	}

	/* alloc bounce buffer */
	s->bb_va = dmam_alloc_coherent(dev, PAGE_SIZE, &s->bb_da, GFP_KERNEL);
	if (!s->bb_va)
		return -ENOMEM;
	s->bb_sz = PAGE_SIZE;

	/* Initialize TZ serice link to HWMGR */
	gsa_tz_chan_ctx_init(&s->aoc_srv, HWMGR_AOC_PORT, dev);
	gsa_tz_chan_ctx_init(&s->tpu_srv, HWMGR_TPU_PORT, dev);

	dev_info(dev, "Initialized\n");

	return 0;
}

static int gsa_remove(struct platform_device *pdev)
{
	struct gsa_dev_state *s = platform_get_drvdata(pdev);

	/* close connection to tz services */
	gsa_tz_chan_close(&s->aoc_srv);
	gsa_tz_chan_close(&s->tpu_srv);

	return 0;
}

static const struct of_device_id gsa_of_match[] = {
	{ .compatible = "google,gs101-gsa-v1", },
	{},
};
MODULE_DEVICE_TABLE(of, gsa_of_match);

static struct platform_driver gsa_driver = {
	.probe = gsa_probe,
	.remove = gsa_remove,
	.driver	= {
		.name = "gsa",
		.of_match_table = gsa_of_match,
	},
};

static int __init gsa_driver_init(void)
{
	return platform_driver_register(&gsa_driver);
}

static void __exit gsa_driver_exit(void)
{
	platform_driver_unregister(&gsa_driver);
}

MODULE_DESCRIPTION("Google GSA core platform driver");
MODULE_LICENSE("GPL v2");
module_init(gsa_driver_init);
module_exit(gsa_driver_exit);
