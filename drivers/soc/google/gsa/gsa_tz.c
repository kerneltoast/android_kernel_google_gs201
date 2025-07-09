// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2021 Google LLC
 */
#include "gsa_tz.h"

#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#define TZ_CON_TIMEOUT  5000
#define TZ_MSG_TIMEOUT 10000
#define TZ_BUF_TIMEOUT 10000

static struct tipc_msg_buf *tz_srv_handle_msg(void *data,
					      struct tipc_msg_buf *rxbuf)
{
	struct gsa_tz_chan_ctx *ctx = data;

	/* copy out reply then signal completion */
	mutex_lock(&ctx->rsp_lock);
	if (ctx->rsp_buf) {
		size_t len = mb_avail_data(rxbuf);

		if (len <= ctx->rsp_buf_size) {
			memcpy(ctx->rsp_buf, mb_get_data(rxbuf, len), len);
			ctx->rsp_res = len;
		} else {
			dev_err(ctx->dev,
				"TZ: RSP buffer is too small (%zd vs. %zd)\n",
				ctx->rsp_buf_size, len);
			ctx->rsp_res = -EMSGSIZE;
		}
	} else {
		dev_err(ctx->dev, "TZ: no RSP buffer: drop message\n");
	}
	mutex_unlock(&ctx->rsp_lock);
	complete(&ctx->reply_comp);

	return rxbuf;
}

static void tz_srv_handle_event(void *data, int event)
{
	struct gsa_tz_chan_ctx *ctx = data;

	complete(&ctx->reply_comp);
}

static const struct tipc_chan_ops tz_srv_ops = {
	.handle_msg = tz_srv_handle_msg,
	.handle_event = tz_srv_handle_event,
};

static int tz_srv_connect_locked(struct gsa_tz_chan_ctx *ctx)
{
	int rc;
	unsigned long timeout;
	struct tipc_chan *chan;

	chan = tipc_create_channel(NULL, &tz_srv_ops, ctx);
	if (IS_ERR(chan)) {
		dev_err(ctx->dev, "TZ: failed (%ld) to create chan\n",
			PTR_ERR(chan));
		return PTR_ERR(chan);
	}

	reinit_completion(&ctx->reply_comp);

	rc = tipc_chan_connect(chan, ctx->port);
	if (rc < 0) {
		dev_err(ctx->dev, "TZ: failed (%d) to connect\n", rc);
		goto err_connect;
	}

	/* wait for connection */
	timeout = msecs_to_jiffies(TZ_CON_TIMEOUT);
	rc = wait_for_completion_timeout(&ctx->reply_comp, timeout);
	if (rc <= 0) {
		rc = (!rc) ? -ETIMEDOUT : rc;
		dev_err(ctx->dev, "TZ: failed (%d) to wait for connect\n", rc);
		goto err_reply;
	}

	/* Note: we will check connection state while sending message */

	ctx->chan = chan;
	return 0;

err_reply:
	tipc_chan_shutdown(chan);
err_connect:
	tipc_chan_destroy(chan);

	return rc;
};

static void gsa_tz_chan_close_locked(struct gsa_tz_chan_ctx *ctx)
{
	if (ctx->chan) {
		tipc_chan_shutdown(ctx->chan);
		tipc_chan_destroy(ctx->chan);
		ctx->chan = NULL;
	}
}

int gsa_tz_chan_msg_xchg(struct gsa_tz_chan_ctx *ctx,
			 const void *req, size_t req_len,
			 void *rsp, size_t rsp_size)
{

	int rc;
	unsigned long timeout;
	struct tipc_msg_buf *txbuf;
	unsigned int retry_connect = 1;

	/* we need both request and response */
	if (!req || !rsp) {
		return -EINVAL;
	}

	mutex_lock(&ctx->req_lock);

reconnect:
	if (!ctx->chan) {
		/* connect to TZ service */
		rc = tz_srv_connect_locked(ctx);
		if (rc < 0) {
			dev_err(ctx->dev, "TZ: failed (%d) to connect\n", rc);
			goto err_connect;
		}
		dev_info(ctx->dev, "TZ: %s connected\n", ctx->port);
	}

	/* get tx buffer */
	txbuf = tipc_chan_get_txbuf_timeout(ctx->chan, TZ_BUF_TIMEOUT);
	if (IS_ERR(txbuf)) {
		dev_err(ctx->dev,
			"TZ: failed (%ld) to get txbuf\n", PTR_ERR(txbuf));
		rc = PTR_ERR(txbuf);
		goto err_get_buf;
	}

	/* copy in request */
	memcpy(mb_put_data(txbuf, req_len), req, req_len);

	/* attach buffer to store response */
	mutex_lock(&ctx->rsp_lock);
	ctx->rsp_buf = rsp;
	ctx->rsp_buf_size = rsp_size;
	ctx->rsp_res = -ENOMSG;
	mutex_unlock(&ctx->rsp_lock);

	/* init completion */
	reinit_completion(&ctx->reply_comp);

	/* queue message */
	rc = tipc_chan_queue_msg(ctx->chan, txbuf);
	if (rc < 0) {
		dev_err(ctx->dev, "TZ: failed (%d) to queue msg\n", rc);

		/* drop buffer */
		tipc_chan_put_txbuf(ctx->chan, txbuf);

		/* reset connection */
		gsa_tz_chan_close_locked(ctx);

		/*
		 * It is possible that server has closed connection
		 * and we might be able to recover
		 */
		if (retry_connect--) {
			dev_info(ctx->dev, "TZ: reconnect\n");
			goto reconnect;
		}

		goto err_queue;
	}

	/* wait for response */
	timeout = msecs_to_jiffies(TZ_MSG_TIMEOUT);
	rc = wait_for_completion_timeout(&ctx->reply_comp, timeout);
	if (rc <= 0) {
		rc = (!rc) ? -ETIMEDOUT : rc;
		dev_err(ctx->dev, "TZ: failed (%d) to wait for reply\n", rc);

		/* reset connection */
		gsa_tz_chan_close_locked(ctx);

		/* bail, it is unlikely that reconnect would solve it */
	}

err_queue:
err_get_buf:
err_connect:

	/* detach response buffer and return response result */
	mutex_lock(&ctx->rsp_lock);
	ctx->rsp_buf = NULL;
	ctx->rsp_buf_size = 0;

	if (rc > 0) {
		/* return response result */
		rc = ctx->rsp_res;
	}
	mutex_unlock(&ctx->rsp_lock);

	/* release request lock */
	mutex_unlock(&ctx->req_lock);
	return rc;
}
EXPORT_SYMBOL_GPL(gsa_tz_chan_msg_xchg);

void gsa_tz_chan_close(struct gsa_tz_chan_ctx *ctx)
{
	mutex_lock(&ctx->req_lock);
	gsa_tz_chan_close_locked(ctx);
	mutex_unlock(&ctx->req_lock);
}
EXPORT_SYMBOL_GPL(gsa_tz_chan_close);

void gsa_tz_chan_ctx_init(struct gsa_tz_chan_ctx *ctx, const char *port,
			  struct device *dev)
{
	ctx->dev = dev;
	strlcpy(ctx->port, port, sizeof(ctx->port));
	mutex_init(&ctx->req_lock);
	mutex_init(&ctx->rsp_lock);
	init_completion(&ctx->reply_comp);
	ctx->rsp_buf = NULL;
	ctx->rsp_buf_size = 0;
	ctx->rsp_res = 0;
}
EXPORT_SYMBOL_GPL(gsa_tz_chan_ctx_init);
