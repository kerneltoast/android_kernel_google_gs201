// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2023 Google LLC.
 *
 * Author: Varad Gautam <varadgautam@google.com>
 */

#ifndef _PIXEL_GPU_UEVENT_H_
#define _PIXEL_GPU_UEVENT_H_

#include <mali_kbase.h>

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

void pixel_gpu_uevent_send(struct kbase_device *kbdev, const struct gpu_uevent *evt);

#endif /* _PIXEL_GPU_UEVENT_H_ */
