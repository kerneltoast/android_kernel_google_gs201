// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2024 Google LLC.
 *
 * Author: Jack Diver <diverj@google.com>
 */

#include <linux/atomic.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dev_printk.h>
/* Pixel integration includes */
#include <soc/google/acpm_ipc_ctrl.h>
#include "pixel_slc.h"


/**
 * DOC: PBHA
 *
 * Borr does not have "real" PBHA support. However, since we only use a 36-bit PA on the bus,
 * AxADDR[39:36] is wired up to the GPU AxUSER[PBHA] field seen by the rest of the system.
 * Those AxADDR bits come from [39:36] in the page descriptor.
 *
 * Odin and Turse have "real" PBHA support using a dedicated output signal and page descriptor field.
 * The AxUSER[PBHA] field is driven by the GPU's PBHA signal, and AxADDR[39:36] is dropped.
 * The page descriptor PBHA field is [62:59].
 *
 * We could write to both of these locations, as each SoC only reads from its respective PBHA
 * location with the other being ignored or dropped.
 *
 * b/148988078 contains confirmation of the above description.
 */
#if IS_ENABLED(CONFIG_SOC_GS101)
#define PBHA_BIT_POS  (36)
#else
#define PBHA_BIT_POS  (59)
#endif
#define PBHA_BIT_MASK (0xf)

#define PARTITION_DISABLE_HYSTERESIS (msecs_to_jiffies(100))
#define PARTITION_ENABLE_THRESHOLD   (7)


/**
 * partition_required() - Determine whether we require a partition to be enabled
 *
 * @pt: The partition to check.
 *
 * Check whether a partition meets the requirements for being enabled.
 *
 * Return: True, if the partition is required to be enabled, otherwise false.
 */
static bool partition_required(struct slc_partition *pt)
{
	lockdep_assert_held(&pt->lock);

	return (atomic_read(&pt->refcount) && (pt->signal >= PARTITION_ENABLE_THRESHOLD)) ||
	       pt->pinned;
}

/**
 * pixel_atomic_dec_and_lock_irqsave - lock on reaching reference count zero
 *
 * @val:   The atomic counter
 * @lock:  The spinlock in question
 * @flags: Storage for the current interrupt enable state
 *
 * Decrements @val by 1, if the result is 0, locks @lock.
 *
 * Return: True if the lock was taken, false for all other cases.
 */
static int pixel_atomic_dec_and_lock_irqsave(atomic_t* val, spinlock_t* lock, unsigned long* flags)
{
	/* Subtract 1 from counter unless that drops it to 0 (ie. it was 1) */
	if (atomic_add_unless(val, -1, 1))
		return 0;

	/* Otherwise do it the slow way */
	spin_lock_irqsave(lock, *flags);
	if (atomic_dec_and_test(val))
		return 1;
	spin_unlock_irqrestore(lock, *flags);

	return 0;
}

/**
 * slc_wipe_pbha - Clear any set PBHA bits from the pte.
 *
 * @pte: The pte to strip of PBHA.
 *
 * Return: The PTE with all PBHA stripped.
 */
u64 slc_wipe_pbha(u64 pte)
{
	return pte & ~((u64)PBHA_BIT_MASK << PBHA_BIT_POS);
}

/**
 * slc_set_pbha - Apply the PBHA to @pte.
 *
 * @data: The &struct slc_data tracking partition information.
 * @pte:  The pte to modify.
 *
 * Return: On success, returns a modified PTE. On failure the original PTE is returned.
 */
u64 slc_set_pbha(struct slc_data const *data, u64 pte)
{
	/* Clear any bits set in the PBHA range */
	pte = slc_wipe_pbha(pte);

	/* Apply the PBHA for the given virtual partition */
	return pte | (((u64)data->partition.pbha) & PBHA_BIT_MASK) << PBHA_BIT_POS;
}

/**
 * enable_partition - Enable @pt
 *
 * @data: The &struct slc_data tracking partition information.
 * @pt:   The &struct slc_partition representing the partition to enable.
 */
static void enable_partition(struct slc_data *data, struct slc_partition *pt)
{
	/* Skip if already enabled */
	if (pt->enabled)
		return;

	(void)pt_client_enable(data->pt_handle, pt->index);
	pt->enabled = true;

	dev_dbg(data->dev, "enabled partition %d", pt->index);
}

/**
 * disable_partition - Disable @pt
 *
 * @data: The &struct slc_data tracking partition information.
 * @pt:   The &struct slc_partition representing the partition to disable.
 */
static void disable_partition(struct slc_data *data, struct slc_partition *pt)
{
	/* Skip if not enabled */
	if (!pt->enabled)
		return;

	pt_client_disable_no_free(data->pt_handle, pt->index);
	pt->enabled = false;

	dev_dbg(data->dev, "disabled partition %d", pt->index);
}

/**
 * queue_disable_worker - Queue a delayed partition disable op
 *
 * @data: The &struct slc_data tracking partition information.
 */
static void queue_disable_worker(struct slc_data *data)
{
	queue_delayed_work(system_highpri_wq, &data->disable_work, PARTITION_DISABLE_HYSTERESIS);
}

/**
 * partition_disable_worker - Callback to lazily disable a partition
 *
 * @work: The &struct work_struct dequeued
 */
static void partition_disable_worker(struct work_struct *work)
{
	struct slc_data* data = container_of(work, struct slc_data, disable_work.work);
	struct slc_partition *pt = &data->partition;
	unsigned long flags;

	/* Complete any pending disable ops */
	spin_lock_irqsave(&pt->lock, flags);

	if (!partition_required(pt))
		disable_partition(data, pt);

	spin_unlock_irqrestore(&pt->lock, flags);
}

/**
 * slc_inc_refcount - Increase the partition reference count.
 *
 * @data: The &struct slc_data tracking partition information.
 *
 * If this is the first reference being taken, the partition will be enabled.
 */
void slc_inc_refcount(struct slc_data *data)
{
	struct slc_partition *pt = &data->partition;

	/* Try to re-enable the partition if this is the first reference */
	if (atomic_inc_return(&pt->refcount) == 1) {
		unsigned long flags;

		spin_lock_irqsave(&pt->lock, flags);

		/* Enable the partition immediately if it's required */
		if (partition_required(pt))
			enable_partition(data, pt);

		spin_unlock_irqrestore(&pt->lock, flags);
	}
}

/**
 * slc_dec_refcount - Decrease the partition reference count.
 *
 * @data: The &struct slc_data tracking partition information.
 *
 * If this is the last reference being released, the partition will be disabled.
 */
void slc_dec_refcount(struct slc_data *data)
{
	struct slc_partition *pt = &data->partition;
	unsigned long flags;

	/* Disable the partition if this was the last reference */
	if (pixel_atomic_dec_and_lock_irqsave(&pt->refcount, &pt->lock, &flags)) {

		/* Lazily disable the partition if it's no longer required */
		if (!partition_required(pt))
			queue_disable_worker(data);

		spin_unlock_irqrestore(&pt->lock, flags);
	}
}

void slc_update_signal(struct slc_data *data, u64 signal)
{
	struct slc_partition *pt = &data->partition;
	unsigned long flags;

	spin_lock_irqsave(&pt->lock, flags);

	/* Use ACPM signal when available */
	if (data->signal)
		pt->signal = ioread64((u64 __iomem*)data->signal);
	else
		pt->signal = signal;

	if (partition_required(pt))
		/* Enable the partition immediately if it's required */
		enable_partition(data, pt);
	else
		/* Lazily disable the partition if it's no longer required */
		queue_disable_worker(data);

	spin_unlock_irqrestore(&pt->lock, flags);
}

void slc_pin(struct slc_data *data, bool pin)
{
	struct slc_partition *pt = &data->partition;
	unsigned long flags;

	spin_lock_irqsave(&pt->lock, flags);

	pt->pinned = pin;
	if (pin)
		enable_partition(data, pt);
	else if (!partition_required(pt))
		queue_disable_worker(data);

	spin_unlock_irqrestore(&pt->lock, flags);
}

/**
 * init_partition - Register and initialize a partition with the SLC driver.
 *
 * @data:  The &struct slc_data tracking partition information.
 * @pt:    The &struct slc_partition to store the configured partition information.
 * @index: The index of the partition, relative to the DT node.
 *
 * Returns EINVAL on error, otherwise 0.
 */
static int init_partition(struct slc_data *data, struct slc_partition *pt, u32 index)
{
	ptid_t ptid;
	ptpbha_t pbha;
	int err = -EINVAL;

	ptid = pt_client_enable(data->pt_handle, index);
	if (ptid == PT_PTID_INVALID) {
		dev_err(data->dev, "failed to enable pt: %d\n", index);
		goto err_exit;
	}

	pbha = pt_pbha(data->dev->of_node, index);
	if (pbha == PT_PBHA_INVALID) {
		dev_err(data->dev, "failed to get PBHA for pt: %d\n", index);
		goto err_exit;
	}

	/* This retains the allocated ptid */
	pt_client_disable_no_free(data->pt_handle, index);

	/* Success */
	err = 0;

	*pt = (struct slc_partition) {
		.index = index,
		.ptid = ptid,
		.pbha = pbha,
		.enabled = false,
		.refcount = ATOMIC_INIT(0),
		.signal = 0,
		.pinned = false,
	};
	spin_lock_init(&pt->lock);

err_exit:
	return err;
}


/**
 * term_partition - Disable and free a partition, unregistering it.
 *
 * @data:  The &struct slc_data tracking partition information.
 * @pt:    The &struct slc_partition to terminate.
 *
 * Returns EINVAL on error, otherwise 0.
 */
static void term_partition(struct slc_data *data, struct slc_partition *pt)
{
	disable_partition(data, pt);
	pt_client_free(data->pt_handle, pt->index);
}

/**
 * slc_init_data - Read all SLC partition information, init the partitions, and track within @data.
 *
 * @data:  The &struct slc_data tracking partition information.
 * @dev:   The platform device associated with the parent node.
 *
 * Return: On success, returns 0. On failure an error code is returned.
 */
int slc_init_data(struct slc_data *data, struct device* dev)
{
	int ret = -EINVAL;

	if (data == NULL || dev == NULL)
		goto err_exit;

	/* Inherit the platform device */
	data->dev = dev;

	INIT_DELAYED_WORK(&data->disable_work, partition_disable_worker);

	/* Register our node with the SLC driver.
	 * This detects our partitions defined within the DT.
	 */
	data->pt_handle = pt_client_register(data->dev->of_node, NULL, NULL);
	if (IS_ERR(data->pt_handle)) {
		ret = PTR_ERR(data->pt_handle);
		dev_err(data->dev, "pt_client_register failed with: %d\n", ret);
		goto err_exit;
	}

	if (IS_ENABLED(PIXEL_GPU_SLC_ACPM_SIGNAL)) {
		u32 size;

		/* Obtain a handle to the ACPM provided GPU partition signal */
		if ((ret = acpm_ipc_get_buffer("GPU_SIGNAL", &data->signal, &size))) {
			dev_err(data->dev, "failed to retrieve SLC GPU signal: %d", ret);
			goto err_exit;
		}

		/* Validate the signal buffer size */
		if (size != sizeof(u64)) {
			dev_err(data->dev, "SLC GPU signal size incorrect: %d", size);
			goto err_exit;
		}
	}

	if ((ret = init_partition(data, &data->partition, 0)))
		goto pt_init_err_exit;

	return 0;

pt_init_err_exit:
	pt_client_unregister(data->pt_handle);

err_exit:
	return ret;
}

/**
 * slc_term_data - Tear down SLC partitions and free tracking data.
 *
 * @data:  The &struct slc_data tracking partition information.
 */
void slc_term_data(struct slc_data *data)
{
	/* Ensure all pending disable ops are complete */
	cancel_delayed_work_sync(&data->disable_work);

	term_partition(data, &data->partition);

	pt_client_unregister(data->pt_handle);
}
