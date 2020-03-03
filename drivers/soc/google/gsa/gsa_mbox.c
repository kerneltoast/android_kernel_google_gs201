// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2019 Google LLC.
 */
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "gsa_mbox.h"

/* Mailbox control Register */
#define MBOX_MCUCTLR_REG 0x0000

/* Interrupt Generation Register */
#define MBOX_INTGR0_REG 0x0020

/* Interrupt Clear Register 0 */
#define MBOX_INTCR0_REG 0x0024

/* Interrupt Mask Register 0 */
#define MBOX_INTMR0_REG 0x0028

/* Interrupt Raw Status Register 0 */
#define MBOX_INTSR0_REG 0x002C

/* Interrupt Masked status register */
#define MBOX_INTMSR0_REG 0x0030

/* Interrupt Generation Register 1 */
#define MBOX_INTGR1_REG 0x0040

/* Interrupt Mask register  */
#define MBOX_INTMR1_REG 0x0048

/* Interrupt Raw status register */
#define MBOX_INTSR1_REG 0x004C

/* Interrupt Masked status register  */
#define MBOX_INTMSR1_REG 0x0050

/* Shared registers */
#define MBOX_SR_BASE_REG 0x0080
#define MBOX_SR_REG(n) (MBOX_SR_BASE_REG + (n) * 4)

/* Number of shared registers  */
#define MBOX_SR_NUM 16

enum mbox_host_irq {
	MBOX_HOST_REQ_IRQ = 0,
	MBOX_HOST_IRQ_NUM,
};

enum mbox_client_irq {
	MBOX_CLIENT_RSP_IRQ = 0,
	MBOX_CLIENT_IRQ_NUM,
};

struct gsa_mbox_req {
	u32 cmd;
	u32 argc;
	u32 *args;
};

struct gsa_mbox_rsp {
	u32 cmd;
	u32 err;
	u32 argc;
	u32 *args;
};

struct gsa_mbox {
	struct device *dev;
	void __iomem *base;
	int irq;
	spinlock_t slock; /* protects RMW like access to some registers */
	struct mutex mbox_lock; /* protects access to SRs */
	struct completion mbox_cmd_completion;
	u32 exp_intmr0;
};

static void gsa_mbox_mask_irq0(struct gsa_mbox *mb, u32 mask)
{
	u32 v;
	unsigned long irq_flags;

	spin_lock_irqsave(&mb->slock, irq_flags);
	v = readl(mb->base + MBOX_INTMR0_REG);
	v |= mask;
	writel(v, mb->base + MBOX_INTMR0_REG);
	mb->exp_intmr0 = v;
	spin_unlock_irqrestore(&mb->slock, irq_flags);
}

static void gsa_mbox_unmask_irq0(struct gsa_mbox *mb, u32 mask)
{
	u32 v;
	unsigned long irq_flags;

	spin_lock_irqsave(&mb->slock, irq_flags);
	v = readl(mb->base + MBOX_INTMR0_REG);
	v &= ~mask;
	writel(v, mb->base + MBOX_INTMR0_REG);
	mb->exp_intmr0 = v;
	spin_unlock_irqrestore(&mb->slock, irq_flags);
}

static void gsa_mbox_sync_irq0(struct gsa_mbox *mb)
{
	u32 v;
	unsigned long irq_flags;

	spin_lock_irqsave(&mb->slock, irq_flags);
	v = readl(mb->base + MBOX_INTMR0_REG);
	if (v != mb->exp_intmr0)
		writel(mb->exp_intmr0, mb->base + MBOX_INTMR0_REG);
	spin_unlock_irqrestore(&mb->slock, irq_flags);
}

static void gsa_mbox_clr_irq0(struct gsa_mbox *mb, u32 mask)
{
	writel(mask, mb->base + MBOX_INTCR0_REG);
}

static irqreturn_t gsa_mb_irq_handler(int irq, void *data)
{
	u32 v;
	struct gsa_mbox *mb = data;

	dev_dbg(mb->dev, "%s: got irq %d\n", __func__, irq);

	/*
	 * check if somehow we have lost state, like a host resets mailbox
	 * under us
	 */
	gsa_mbox_sync_irq0(mb);

	v = readl(mb->base + MBOX_INTMSR0_REG);
	if (v & (0x1u << MBOX_CLIENT_RSP_IRQ)) {
		/* response */
		gsa_mbox_mask_irq0(mb, (0x1u << MBOX_CLIENT_RSP_IRQ));
		complete(&mb->mbox_cmd_completion);
	}

	return IRQ_HANDLED;
}

struct gsa_mbox *gsa_mbox_init(struct platform_device *pdev)
{
	int err;
	struct gsa_mbox *mb;
	struct resource *res;
	struct device *dev = &pdev->dev;

	mb = devm_kzalloc(dev, sizeof(*mb), GFP_KERNEL);
	if (!mb)
		return ERR_PTR(-ENOMEM);

	mb->dev = dev;
	spin_lock_init(&mb->slock);
	mutex_init(&mb->mbox_lock);
	init_completion(&mb->mbox_cmd_completion);

	/* map mbox registers */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mb->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(mb->base)) {
		dev_err(dev, "ioremap failed (%d)\n", (int)PTR_ERR(mb->base));
		return mb->base;
	}

	/* place hardware into known state */
	mb->exp_intmr0 = 0xFFFF;
	writel(mb->exp_intmr0, mb->base + MBOX_INTMR0_REG);

	/* register mbox interrupt */
	mb->irq = platform_get_irq(pdev, 0);
	if (mb->irq <= 0) {
		dev_err(dev, "get_irq failed (%d)\n", mb->irq);
		return ERR_PTR(mb->irq);
	}

	err = devm_request_irq(dev, mb->irq, gsa_mb_irq_handler, 0,
			       "gsa-mb-irq", mb);
	if (err) {
		dev_err(dev, "request_irq failed %d\n", err);
		return ERR_PTR(err);
	}

	return mb;
}

static int exec_mbox_cmd_sync(struct gsa_mbox *mb,
			      struct gsa_mbox_req *req,
			      struct gsa_mbox_rsp *rsp)
{
	u32 i;
	int ret;
	u32 max_rsp_argc;

	if (!mb)
		return -ENODEV;

	if (!req || !rsp)
		return -EINVAL;

	if (req->argc && !req->args) {
		/* request args required */
		return -EINVAL;
	}

	if (rsp->argc && !rsp->args) {
		/* response args required */
		return -EINVAL;
	}

	if (req->argc + 2 > MBOX_SR_NUM) {
		/* too many request arguments */
		return -EINVAL;
	}

	/* save max response arg count */
	max_rsp_argc = rsp->argc;

	mutex_lock(&mb->mbox_lock);

	/* write command */
	writel(req->cmd, mb->base + MBOX_SR_REG(0));
	writel(req->argc, mb->base + MBOX_SR_REG(1));
	for (i = 0; i < req->argc; i++)
		writel(req->args[i], mb->base + MBOX_SR_REG(i + 2));

	/* initiate request */
	reinit_completion(&mb->mbox_cmd_completion);

	/* unmask response interrupt */
	gsa_mbox_clr_irq0(mb, (0x1u << MBOX_CLIENT_RSP_IRQ));
	gsa_mbox_unmask_irq0(mb, (0x1u << MBOX_CLIENT_RSP_IRQ));

	/* raise request interrupt */
	writel((0x1u << MBOX_HOST_REQ_IRQ), mb->base + MBOX_INTGR1_REG);

	/* wait for response */
	wait_for_completion(&mb->mbox_cmd_completion);

	/*  read response */
	rsp->cmd = readl(mb->base + MBOX_SR_REG(0));
	rsp->err = readl(mb->base + MBOX_SR_REG(1));
	rsp->argc = readl(mb->base + MBOX_SR_REG(2));

	/* check argc: first 3 registers are for cmd, err and argc */
	if (WARN_ON(rsp->argc + 3 > MBOX_SR_NUM)) {
		/* malformed response */
		ret = -EIO;
		goto err_bad_rsp;
	}

	if (WARN_ON(rsp->argc > max_rsp_argc)) {
		/* not enough space to save all returned arguments */
		rsp->argc = max_rsp_argc;
	}

	/* copy response */
	for (i = 0; i < rsp->argc; i++)
		rsp->args[i] = readl(mb->base + MBOX_SR_REG(i + 3));

	ret = 0;

err_bad_rsp:
	/* clear and mask response interrupts */
	gsa_mbox_clr_irq0(mb, (0x1u << MBOX_CLIENT_RSP_IRQ));
	gsa_mbox_mask_irq0(mb, (0x1u << MBOX_CLIENT_RSP_IRQ));

	mutex_unlock(&mb->mbox_lock);
	return ret;
}

/*
 * GSA specific mailbox protocol support
 */

/* Command response bit */
#define GSA_MB_CMD_RSP	(0x1U << 31)

/* Mailbox error codes */
enum gsa_mb_error {
	/* Defined by GSA ROM */
	GSA_MB_OK = 0U,
	GSA_MB_ERR_INVALID_ARGS = 1U,
	GSA_MB_ERR_AUTH_FAILED = 2U,
	GSA_MB_ERR_BUSY = 3U,
	GSA_MB_ERR_ALREADY_RUNNING = 4U,
	GSA_MB_ERR_OUT_OF_RESOURCES = 5U,
	GSA_MB_ERR_BAD_HANDLE = 6U,

	/* Extended by GSA firmware */
	GSA_MB_ERR_GENERIC = 128U,
	GSA_MB_ERR_INTERNAL = 129U,
	GSA_MB_ERR_TIMED_OUT = 130U,
	GSA_MB_ERR_BAD_STATE = 131U,
};

static int check_mbox_cmd_rsp(struct device *dev,
			      struct gsa_mbox_rsp *rsp, u32 cmd)
{
	if (rsp->cmd != (cmd | GSA_MB_CMD_RSP)) {
		/* bad response */
		dev_err(dev, "cmd=0x%x\n", rsp->cmd);
		return -EIO;
	}

	if (rsp->err == GSA_MB_OK)
		return 0;

	dev_err(dev, "mbox cmd=0x%x returned err=%u\n", cmd, rsp->err);

	switch (rsp->err) {
	case GSA_MB_ERR_BAD_HANDLE:
	case GSA_MB_ERR_INVALID_ARGS:
		return -EINVAL;

	case GSA_MB_ERR_BUSY:
		return -EBUSY;

	case GSA_MB_ERR_AUTH_FAILED:
		return -EACCES;

	case GSA_MB_ERR_OUT_OF_RESOURCES:
		return -ENOMEM;

	case GSA_MB_ERR_ALREADY_RUNNING:
		return -EEXIST;

	case GSA_MB_ERR_TIMED_OUT:
		return -ETIMEDOUT;

	default:
		return -EIO;
	}

	return 0;
}

int gsa_send_mbox_cmd(struct gsa_mbox *mb, u32 cmd,
		      u32 *req_args, u32 req_argc,
		      u32 *rsp_args, u32 rsp_max_argc)
{
	int ret;
	struct gsa_mbox_req mb_req;
	struct gsa_mbox_rsp mb_rsp;

	/* prepare request */
	mb_req.cmd = cmd;
	mb_req.argc = req_argc;
	mb_req.args = req_args;

	/* prepare response */
	mb_rsp.cmd = 0;
	mb_rsp.err = 0;
	mb_rsp.argc = rsp_max_argc;
	mb_rsp.args = rsp_args;

	/* execute command */
	ret = exec_mbox_cmd_sync(mb, &mb_req, &mb_rsp);
	if (ret < 0)
		return ret;

	/* check result */
	ret = check_mbox_cmd_rsp(mb->dev, &mb_rsp, cmd);
	if (ret < 0)
		return ret;

	return mb_rsp.argc;
}

MODULE_LICENSE("GPL v2");
