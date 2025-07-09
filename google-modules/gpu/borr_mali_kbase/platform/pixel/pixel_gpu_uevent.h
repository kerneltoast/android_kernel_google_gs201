// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2023 Google LLC.
 *
 * Author: Varad Gautam <varadgautam@google.com>
 */

#ifndef _PIXEL_GPU_UEVENT_H_
#define _PIXEL_GPU_UEVENT_H_

#include <mali_kbase.h>
#include <linux/kfifo.h>

#define GPU_UEVENT_TYPE_LIST                    \
    GPU_UEVENT_TYPE(NONE)                       \
    GPU_UEVENT_TYPE(KMD_ERROR)                  \
    GPU_UEVENT_TYPE(GPU_RESET)                  \
    GPU_UEVENT_TYPE(MAX)

#define GPU_UEVENT_TYPE(type) GPU_UEVENT_TYPE_##type,
enum gpu_uevent_type {
    GPU_UEVENT_TYPE_LIST
};

#undef GPU_UEVENT_TYPE
#define GPU_UEVENT_TYPE(type) "GPU_UEVENT_TYPE="#type,
static inline const char *gpu_uevent_type_str(enum gpu_uevent_type type) {
    static const char * const gpu_uevent_types[] = {
        GPU_UEVENT_TYPE_LIST
    };
    return gpu_uevent_types[type];
}
#undef GPU_UEVENT_TYPE

#define GPU_UEVENT_INFO_LIST                    \
    GPU_UEVENT_INFO(NONE)                       \
    GPU_UEVENT_INFO(CSG_REQ_STATUS_UPDATE)      \
    GPU_UEVENT_INFO(CSG_SUSPEND)                \
    GPU_UEVENT_INFO(CSG_SLOTS_SUSPEND)          \
    GPU_UEVENT_INFO(CSG_GROUP_SUSPEND)          \
    GPU_UEVENT_INFO(CSG_EP_CFG)                 \
    GPU_UEVENT_INFO(CSG_SLOTS_START)            \
    GPU_UEVENT_INFO(GROUP_TERM)                 \
    GPU_UEVENT_INFO(QUEUE_START)                \
    GPU_UEVENT_INFO(QUEUE_STOP)                 \
    GPU_UEVENT_INFO(QUEUE_STOP_ACK)             \
    GPU_UEVENT_INFO(CSG_SLOT_READY)             \
    GPU_UEVENT_INFO(L2_PM_TIMEOUT)              \
    GPU_UEVENT_INFO(PM_TIMEOUT)                 \
    GPU_UEVENT_INFO(CSF_RESET_OK)               \
    GPU_UEVENT_INFO(CSF_RESET_FAILED)           \
    GPU_UEVENT_INFO(TILER_OOM)                  \
    GPU_UEVENT_INFO(PROGRESS_TIMER)             \
    GPU_UEVENT_INFO(CS_ERROR)                   \
    GPU_UEVENT_INFO(FW_ERROR)                   \
    GPU_UEVENT_INFO(PMODE_EXIT_TIMEOUT)         \
    GPU_UEVENT_INFO(PMODE_ENTRY_FAILURE)        \
    GPU_UEVENT_INFO(GPU_PAGE_FAULT)             \
    GPU_UEVENT_INFO(MMU_AS_ACTIVE_STUCK)        \
    GPU_UEVENT_INFO(TRACE_BUF_INVALID_SLOT)     \
    GPU_UEVENT_INFO(MAX)

#define GPU_UEVENT_INFO(info) GPU_UEVENT_INFO_##info,
enum gpu_uevent_info {
    GPU_UEVENT_INFO_LIST
};
#undef GPU_UEVENT_INFO
#define GPU_UEVENT_INFO(info) "GPU_UEVENT_INFO="#info,
static inline const char *gpu_uevent_info_str(enum gpu_uevent_info info) {
    static const char * const gpu_uevent_infos[] = {
        GPU_UEVENT_INFO_LIST
    };
    return gpu_uevent_infos[info];
}
#undef GPU_UEVENT_INFO

struct gpu_uevent {
    enum gpu_uevent_type type;
    enum gpu_uevent_info info;
};

struct gpu_uevent_ctx {
    unsigned long last_uevent_ts[GPU_UEVENT_TYPE_MAX];
    DECLARE_KFIFO_PTR(evts_fifo, struct gpu_uevent);
    spinlock_t lock;
    struct work_struct gpu_uevent_work;
};

void pixel_gpu_uevent_send(struct kbase_device *kbdev, const struct gpu_uevent *evt);

void pixel_gpu_uevent_kmd_error_send(struct kbase_device *kbdev, const enum gpu_uevent_info info);

void gpu_uevent_term(struct kbase_device *kbdev);
int gpu_uevent_init(struct kbase_device *kbdev);

#endif /* _PIXEL_GPU_UEVENT_H_ */
