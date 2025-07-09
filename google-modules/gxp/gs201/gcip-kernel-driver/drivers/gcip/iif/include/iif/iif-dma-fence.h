/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Bridges DMA fence with inter-IP fence. Creates an AP-signaled IIF which will be signaled by the
 * IIF driver once the bound DMA fence is signaled. The DMA fence status will be propagated to the
 * IIF.
 *
 * The purpose of this interface is to hold a command which is blocked by DMA fences in the firmware
 * side, not the kernel driver side. The IP driver is expected to create an IIF using this interface
 * if there are DMA fences blocking the command and add the created IIF to in-fences of the command
 * and send the command to the firmware directly.
 *
 * Copyright (C) 2024 Google LLC
 */

#ifndef __IIF_IIF_DMA_FENCE_H__
#define __IIF_IIF_DMA_FENCE_H__

#include <linux/dma-fence.h>
#include <linux/sched.h>

#include <iif/iif-fence.h>
#include <iif/iif-manager.h>

#define to_iif_dma_fence(iif) container_of(iif, struct iif_dma_fence, iif_fence)

/* Wrapper structure to bridge inter-IP fence and DMA fence. */
struct iif_dma_fence {
	/* The AP-signaled IIF which will be signaled when @dma_fence is signaled. */
	struct iif_fence iif_fence;
	/* The DMA fence that @iif_fence will wait on. */
	struct dma_fence *dma_fence;
	/* The thread which will wait on @dma_fence to be signaled and signal @iif_fence. */
	struct task_struct *task;
	/* The timeout of waiting on @dma_fence. */
	signed long timeout_jiffies;
};

/**
 * iif_dma_fence_wait_timeout() - Creates an inter-IP fence which will be signaled once @dma_fence
 *                                is signaled.
 * @mgr: IIF manager to create an inter-IP fence.
 * @dma_fence: The DMA fence to wait on.
 * @timeout_jiffies: The timeout in jiffies, or MAX_SCHEDULE_TIMEOUT to wait until @dma_fence gets
 *                   signaled.
 *
 * Internally, it will create a thread waiting on @dma_fence to be signaled and the thread will
 * signal the IIF once @dma_fence is signaled. As the subject who signal the fence is the IIF
 * driver, the signaler IP type of the returned IIF is AP.
 *
 * Note that the caller should release the refcount of the returned IIF (i.e., iif_fence_put())
 * if they don't need to access it anymore.
 *
 * This function cannot be called in the IRQ context.
 *
 * Returns an inter-IP fence on success. Otherwise, returns an error.
 */
struct iif_fence *iif_dma_fence_wait_timeout(struct iif_manager *mgr, struct dma_fence *dma_fence,
					     signed long timeout_jiffies);

/**
 * iif_dma_fence_wait() - The same with `iif_dma_fence_wait()`, but without timeout.
 * @mgr: IIF manager to create an inter-IP fence.
 * @dma_fence: The DMA fence to wait on.
 *
 * See `iif_dma_fence_wait_timeout()` for details.
 */
static inline struct iif_fence *iif_dma_fence_wait(struct iif_manager *mgr,
						   struct dma_fence *dma_fence)
{
	return iif_dma_fence_wait_timeout(mgr, dma_fence, MAX_SCHEDULE_TIMEOUT);
}

/**
 * iif_dma_fence_stop() - Stops @iif_fence waiting on the DMA fence.
 * @iif_fence: The inter-IP fence to stop waiting on the waitd DMA fence.
 *
 * The thread waiting on the DMA fence will be interrupted. If the thread was interrupted before
 * the DMA fence is signaled, @iif_fence will be signaled with -ERESTARTSYS error.
 *
 * This function must be called with an inter-IP fence which was created by `iif_dma_fence_wait()`
 * only. Also, it cannot be called in the IRQ context.
 *
 * This function can be called once at any time even after @iif_fence has been signaled and the
 * thread waiting on the DMA fence already terminated.
 */
void iif_dma_fence_stop(struct iif_fence *iif_fence);

#endif /* __IIF_IIF_DMA_FENCE_H__ */
