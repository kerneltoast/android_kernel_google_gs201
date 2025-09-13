// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2023 Google LLC.
 *
 * Author: Varad Gautam <varadgautam@google.com>
 */

#include "pixel_gpu_uevent.h"
#include "mali_kbase_config_platform.h"

#define GPU_UEVENT_RATELIMIT_MS (1200000U) /* 20min */

static bool gpu_uevent_check_valid(const struct gpu_uevent *evt)
{
    switch (evt->type) {
    case GPU_UEVENT_TYPE_KMD_ERROR:
        switch (evt->info) {
        case GPU_UEVENT_INFO_CSG_REQ_STATUS_UPDATE:
        case GPU_UEVENT_INFO_CSG_SUSPEND:
        case GPU_UEVENT_INFO_CSG_SLOTS_SUSPEND:
        case GPU_UEVENT_INFO_CSG_GROUP_SUSPEND:
        case GPU_UEVENT_INFO_CSG_EP_CFG:
        case GPU_UEVENT_INFO_CSG_SLOTS_START:
        case GPU_UEVENT_INFO_GROUP_TERM:
        case GPU_UEVENT_INFO_QUEUE_START:
        case GPU_UEVENT_INFO_QUEUE_STOP:
        case GPU_UEVENT_INFO_QUEUE_STOP_ACK:
        case GPU_UEVENT_INFO_CSG_SLOT_READY:
        case GPU_UEVENT_INFO_L2_PM_TIMEOUT:
        case GPU_UEVENT_INFO_PM_TIMEOUT:
        case GPU_UEVENT_INFO_TILER_OOM:
        case GPU_UEVENT_INFO_PROGRESS_TIMER:
        case GPU_UEVENT_INFO_CS_ERROR:
        case GPU_UEVENT_INFO_FW_ERROR:
        case GPU_UEVENT_INFO_PMODE_EXIT_TIMEOUT:
        case GPU_UEVENT_INFO_PMODE_ENTRY_FAILURE:
        case GPU_UEVENT_INFO_GPU_PAGE_FAULT:
        case GPU_UEVENT_INFO_MMU_AS_ACTIVE_STUCK:
        case GPU_UEVENT_INFO_TRACE_BUF_INVALID_SLOT:
            return true;
        default:
            return false;
        }
        break;
    case GPU_UEVENT_TYPE_GPU_RESET:
        switch (evt->info) {
        case GPU_UEVENT_INFO_CSF_RESET_OK:
        case GPU_UEVENT_INFO_CSF_RESET_FAILED:
            return true;
        default:
            return false;
        }
        break;
    default:
        break;
    }

    return false;
}

static void pixel_gpu_uevent_send_worker(struct work_struct *data)
{
    struct pixel_context *pc = container_of(data, struct pixel_context,
        gpu_uevent_ctx.gpu_uevent_work);
    struct kbase_device *kbdev = pc->kbdev;
    struct gpu_uevent_ctx *gpu_uevent_ctx = &pc->gpu_uevent_ctx;
    enum uevent_env_idx {
        ENV_IDX_TYPE,
        ENV_IDX_INFO,
        ENV_IDX_NULL,
        ENV_IDX_MAX
    };
    char *env[ENV_IDX_MAX] = {0};
    unsigned long flags, current_ts = jiffies;
    bool suppress_uevent = false;
    struct gpu_uevent evt = {0};

    if (!kfifo_initialized(&gpu_uevent_ctx->evts_fifo))
        return;

    spin_lock_irqsave(&gpu_uevent_ctx->lock, flags);

    if (kfifo_out(&gpu_uevent_ctx->evts_fifo, &evt, 1 /* nelems */) &&
        gpu_uevent_check_valid(&evt) &&
        time_after(current_ts, gpu_uevent_ctx->last_uevent_ts[evt.type]
            + msecs_to_jiffies(GPU_UEVENT_RATELIMIT_MS))) {
        gpu_uevent_ctx->last_uevent_ts[evt.type] = current_ts;
    } else {
        suppress_uevent = true;
    }

    spin_unlock_irqrestore(&gpu_uevent_ctx->lock, flags);

    if (suppress_uevent)
        return;

    env[ENV_IDX_TYPE] = (char *) gpu_uevent_type_str(evt.type);
    env[ENV_IDX_INFO] = (char *) gpu_uevent_info_str(evt.info);
    env[ENV_IDX_NULL] = NULL;

    kobject_uevent_env(&kbdev->dev->kobj, KOBJ_CHANGE, env);
}

void pixel_gpu_uevent_send(struct kbase_device *kbdev, const struct gpu_uevent *evt)
{
    struct pixel_context *pc = kbdev->platform_context;
    struct gpu_uevent_ctx *gpu_uevent_ctx = &pc->gpu_uevent_ctx;
    unsigned long flags, current_ts = jiffies;
    bool suppress_uevent = false;

    if (!gpu_uevent_check_valid(evt)) {
        dev_err(kbdev->dev, "unrecognized uevent type=%u info=%u", evt->type, evt->info);
        return;
    }

    if (!kfifo_initialized(&gpu_uevent_ctx->evts_fifo))
        return;

    spin_lock_irqsave(&gpu_uevent_ctx->lock, flags);

    if (time_after(current_ts, gpu_uevent_ctx->last_uevent_ts[evt->type]
            + msecs_to_jiffies(GPU_UEVENT_RATELIMIT_MS))) {
        if (!kfifo_avail(&gpu_uevent_ctx->evts_fifo)) {
            /* Drop the oldest unprocessed uevent if the queue is full. */
            kfifo_skip(&gpu_uevent_ctx->evts_fifo);
        }

        kfifo_in(&gpu_uevent_ctx->evts_fifo, evt, 1 /* nelems */);
    } else {
        suppress_uevent = true;
    }

    spin_unlock_irqrestore(&gpu_uevent_ctx->lock, flags);

    if (suppress_uevent)
        return;

    schedule_work(&gpu_uevent_ctx->gpu_uevent_work);
}

void pixel_gpu_uevent_kmd_error_send(struct kbase_device *kbdev, const enum gpu_uevent_info info)
{
    const struct gpu_uevent evt = {
        .type = GPU_UEVENT_TYPE_KMD_ERROR,
        .info = info
    };

    pixel_gpu_uevent_send(kbdev, &evt);
}

void gpu_uevent_term(struct kbase_device *kbdev)
{
    struct pixel_context *pc = kbdev->platform_context;
    struct gpu_uevent_ctx *gpu_uevent_ctx = &pc->gpu_uevent_ctx;

    cancel_work_sync(&gpu_uevent_ctx->gpu_uevent_work);
    if (kfifo_initialized(&gpu_uevent_ctx->evts_fifo))
        kfifo_free(&gpu_uevent_ctx->evts_fifo);
}

int gpu_uevent_init(struct kbase_device *kbdev)
{
    struct pixel_context *pc = kbdev->platform_context;
    struct gpu_uevent_ctx *gpu_uevent_ctx = &pc->gpu_uevent_ctx;

    memset(&pc->gpu_uevent_ctx, 0, sizeof(struct gpu_uevent_ctx));
    spin_lock_init(&gpu_uevent_ctx->lock);
    INIT_WORK(&gpu_uevent_ctx->gpu_uevent_work, pixel_gpu_uevent_send_worker);

    if (kfifo_alloc(&gpu_uevent_ctx->evts_fifo, 4 /* nelems */, GFP_KERNEL))
        return -ENOMEM;

    return 0;
}
