// SPDX-License-Identifier: GPL-2.0-only
/*
 * GXP core telemetry support
 *
 * Copyright (C) 2021-2022 Google LLC
 */

#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/wait.h>

#include "gxp-config.h"
#include "gxp-core-telemetry.h"
#include "gxp-dma.h"
#include "gxp-firmware.h"
#include "gxp-firmware-data.h"
#include "gxp-host-device-structs.h"
#include "gxp-notification.h"
#include "gxp-vd.h"

static uint gxp_core_telemetry_buffer_size = CORE_TELEMETRY_DEFAULT_BUFFER_SIZE;
module_param_named(core_telemetry_buffer_size, gxp_core_telemetry_buffer_size, uint, 0660);

static inline bool is_telemetry_enabled(struct gxp_dev *gxp, uint core, u8 type)
{
	u32 device_status =
		gxp_fw_data_get_core_telemetry_device_status(gxp, core, type);

	return device_status & GXP_CORE_TELEMETRY_DEVICE_STATUS_ENABLED;
}

void gxp_core_telemetry_status_notify(struct gxp_dev *gxp, uint core)
{
	struct gxp_core_telemetry_manager *mgr = gxp->core_telemetry_mgr;

	/* Wake any threads waiting on a core telemetry disable ACK */
	wake_up(&mgr->waitq);

	/* Signal the appropriate eventfd for any active core telemetry types */
	mutex_lock(&mgr->lock);

	if (is_telemetry_enabled(gxp, core, GXP_TELEMETRY_TYPE_LOGGING) &&
	    mgr->logging_efd)
		eventfd_signal(mgr->logging_efd, 1);

	if (is_telemetry_enabled(gxp, core, GXP_TELEMETRY_TYPE_TRACING) &&
	    mgr->tracing_efd)
		eventfd_signal(mgr->tracing_efd, 1);

	mutex_unlock(&mgr->lock);
}

static void telemetry_status_notification_work(struct work_struct *work)
{
	struct gxp_core_telemetry_work *telem_work =
		container_of(work, struct gxp_core_telemetry_work, work);
	struct gxp_dev *gxp = telem_work->gxp;
	uint core = telem_work->core;

	gxp_core_telemetry_status_notify(gxp, core);
}

static struct buffer_data *allocate_telemetry_buffers(struct gxp_dev *gxp,
						      size_t size);
static void free_telemetry_buffers(struct gxp_dev *gxp, struct buffer_data *data);

/**
 * enable_telemetry_buffers() - enable the telemetry buffers from host.
 *
 * @gxp: The GXP device the buffers were allocated for.
 * @data: The data describing a set of core telemetry buffers to be enabled.
 * @type: Either `GXP_TELEMETRY_TYPE_LOGGING` or `GXP_TELEMETRY_TYPE_TRACING`.
 *
 * Return:
 * * 0         - Success
 * * otherwise - Error returned by `gxp_fw_data_set_core_telemetry_descriptors()`
 */
static int enable_telemetry_buffers(struct gxp_dev *gxp,
				    struct buffer_data *data, u8 type)
{
	int i, ret;

	/* Initialize the per core telemetry buffers header with magic code. */
	for (i = 0; i < GXP_NUM_CORES; i++) {
		/*
		 * First 64 bytes of per core telemetry buffers are reserved
		 * for buffer metadata header.  We don't need to explicitly
		 * reset the header fields as during buffer allocation the
		 * entire buffer is zeroed out. First 4 bytes of buffer
		 * metadata header are reserved for valid_magic field.
		 */
		*((uint *)data->buffers[i].vaddr) =
			GXP_TELEMETRY_BUFFER_VALID_MAGIC_CODE;
	}

	data->host_status |= GXP_CORE_TELEMETRY_HOST_STATUS_ENABLED;
	ret = gxp_fw_data_set_core_telemetry_descriptors(
		gxp, type, data->host_status, data->buffers, data->size);

	if (ret) {
		dev_err(gxp->dev,
			"setting telemetry buffers in scratchpad region failed (ret=%d).",
			ret);
		return ret;
	}

	data->is_enabled = true;
	return 0;
}

int gxp_core_telemetry_init(struct gxp_dev *gxp)
{
	struct gxp_core_telemetry_manager *mgr;
	struct buffer_data *log_buff_data, *trace_buff_data;
	int i, ret;

	mgr = devm_kzalloc(gxp->dev, sizeof(*mgr), GFP_KERNEL);
	if (!mgr)
		return -ENOMEM;

	mutex_init(&mgr->lock);
	for (i = 0; i < GXP_NUM_CORES; i++) {
		INIT_WORK(&mgr->notification_works[i].work,
			  telemetry_status_notification_work);
		mgr->notification_works[i].gxp = gxp;
		mgr->notification_works[i].core = i;

	}
	init_waitqueue_head(&mgr->waitq);

	gxp->core_telemetry_mgr = mgr;
	gxp_core_telemetry_buffer_size = ALIGN(gxp_core_telemetry_buffer_size,
					       GXP_CORE_TELEMETRY_BUFFER_UNIT_SIZE);
	if ((gxp_core_telemetry_buffer_size < CORE_TELEMETRY_DEFAULT_BUFFER_SIZE) ||
	    (gxp_core_telemetry_buffer_size > CORE_TELEMETRY_MAX_BUFFER_SIZE)) {
		dev_warn(gxp->dev,
			 "Invalid core telemetry buffer size, enforcing to default %u bytes\n",
			 CORE_TELEMETRY_DEFAULT_BUFFER_SIZE);
		gxp_core_telemetry_buffer_size = CORE_TELEMETRY_DEFAULT_BUFFER_SIZE;
	}

	/* TODO(b/260959553): Remove mutex_lock/unlock during legacy telemetry removal */
	mutex_lock(&mgr->lock);
	log_buff_data = allocate_telemetry_buffers(gxp, gxp_core_telemetry_buffer_size);
	if (IS_ERR_OR_NULL(log_buff_data)) {
		dev_warn(gxp->dev,
			 "Failed to allocate per core log buffer of %u bytes\n",
			 gxp_core_telemetry_buffer_size);
		ret = -ENOMEM;
		goto err_free_buffers;
	}

	trace_buff_data = allocate_telemetry_buffers(gxp, gxp_core_telemetry_buffer_size);
	if (IS_ERR_OR_NULL(trace_buff_data)) {
		dev_warn(gxp->dev,
			 "Failed to allocate per core trace buffer of %u bytes\n",
			 gxp_core_telemetry_buffer_size);
		free_telemetry_buffers(gxp, log_buff_data);
		ret = -ENOMEM;
		goto err_free_buffers;
	}

	ret = enable_telemetry_buffers(gxp, log_buff_data,
				       GXP_TELEMETRY_TYPE_LOGGING);
	if (ret) {
		dev_warn(gxp->dev, "enable telemetry buffer failed (ret=%d)",
			 ret);
		goto err_free;
	}
	ret = enable_telemetry_buffers(gxp, trace_buff_data,
				       GXP_TELEMETRY_TYPE_TRACING);
	if (ret) {
		dev_warn(gxp->dev, "enable telemetry buffer failed (ret=%d)",
			 ret);
		goto err_free;
	}

	gxp->core_telemetry_mgr->logging_buff_data = log_buff_data;
	gxp->core_telemetry_mgr->tracing_buff_data = trace_buff_data;
	mutex_unlock(&mgr->lock);
	return 0;

err_free:
	free_telemetry_buffers(gxp, log_buff_data);
	free_telemetry_buffers(gxp, trace_buff_data);
err_free_buffers:
	mutex_unlock(&mgr->lock);
	mutex_destroy(&mgr->lock);
	devm_kfree(gxp->dev, mgr);
	gxp->core_telemetry_mgr = NULL;
	return ret;
}

/* Wrapper struct to be used by the core telemetry vma_ops. */
struct telemetry_vma_data {
	struct gxp_dev *gxp;
	struct buffer_data *buff_data;
	u8 type;
	refcount_t ref_count;
};

static void telemetry_vma_open(struct vm_area_struct *vma)
{
	struct gxp_dev *gxp;
	struct telemetry_vma_data *vma_data =
		(struct telemetry_vma_data *)vma->vm_private_data;
	/*
	 * vma_ops are required only for legacy telemetry flow
	 * to keep track of buffer allocation during mmap and
	 * buffer free during munmap.
	 */
	if (IS_ERR_OR_NULL(vma_data))
		return;

	gxp = vma_data->gxp;
	mutex_lock(&gxp->core_telemetry_mgr->lock);

	refcount_inc(&vma_data->ref_count);

	mutex_unlock(&gxp->core_telemetry_mgr->lock);
}

static void telemetry_vma_close(struct vm_area_struct *vma)
{
	struct gxp_dev *gxp;
	struct buffer_data *buff_data;
	u8 type;
	struct telemetry_vma_data *vma_data =
		(struct telemetry_vma_data *)vma->vm_private_data;
	/*
	 * vma_ops are required only for legacy telemetry flow
	 * to keep track of buffer allocation during mmap and
	 * buffer free during munmap.
	 */
	if (IS_ERR_OR_NULL(vma_data))
		return;

	gxp = vma_data->gxp;
	buff_data = vma_data->buff_data;
	type = vma_data->type;

	mutex_lock(&gxp->core_telemetry_mgr->lock);

	if (!refcount_dec_and_test(&vma_data->ref_count))
		goto out;

	/*
	 * Free the core telemetry buffers if they are no longer in use.
	 *
	 * If a client enabled core telemetry, then closed their VMA without
	 * disabling it, firmware will still be expecting those buffers to be
	 * mapped. If this is the case, core telemetry will be disabled, and the
	 * buffers freed, when the client is closed.
	 *
	 * We cannot disable core telemetry here, since attempting to lock the
	 * `vd_semaphore` while holding the mmap lock can lead to deadlocks.
	 */
	if (refcount_dec_and_test(&buff_data->ref_count)) {
		switch (type) {
		case GXP_TELEMETRY_TYPE_LOGGING:
			gxp->core_telemetry_mgr->logging_buff_data_legacy = NULL;
			break;
		case GXP_TELEMETRY_TYPE_TRACING:
			gxp->core_telemetry_mgr->tracing_buff_data_legacy = NULL;
			break;
		default:
			dev_warn(gxp->dev, "%s called with invalid type %u\n",
				 __func__, type);
		}
		free_telemetry_buffers(gxp, buff_data);
	}

	kfree(vma_data);

out:
	mutex_unlock(&gxp->core_telemetry_mgr->lock);
}

/* TODO(b/260959553): Remove vma ops during legacy telemetry removal */
static const struct vm_operations_struct telemetry_vma_ops = {
	.open = telemetry_vma_open,
	.close = telemetry_vma_close,
};

/**
 * check_telemetry_type_availability() - Checks if @type is valid and whether
 *                                       buffers of that type already exists.
 * @gxp: The GXP device to check availability for
 * @type: Either `GXP_TELEMETRY_TYPE_LOGGING` or `GXP_TELEMETRY_TYPE_TRACING`
 *
 * Caller must hold the core telemetry_manager's lock.
 *
 * Return:
 * * 0       - @type is valid and can have new buffers created
 * * -EBUSY  - Buffers already exist for @type
 * * -EINVAL - @type is not a valid core telemetry type
 */
static int check_telemetry_type_availability(struct gxp_dev *gxp, u8 type)
{
	lockdep_assert_held(&gxp->core_telemetry_mgr->lock);

	switch (type) {
	case GXP_TELEMETRY_TYPE_LOGGING:
		if (gxp->core_telemetry_mgr->logging_buff_data_legacy)
			return -EBUSY;
		break;
	case GXP_TELEMETRY_TYPE_TRACING:
		if (gxp->core_telemetry_mgr->tracing_buff_data_legacy)
			return -EBUSY;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/**
 * allocate_telemetry_buffers() - Allocate and populate a `struct buffer_data`,
 *                                including allocating and mapping one coherent
 *                                buffer of @size bytes per core.
 * @gxp: The GXP device to allocate the buffers for
 * @size: The size of buffer to allocate for each core
 *
 * Caller must hold the core telemetry_manager's lock.
 *
 * Return: A pointer to the `struct buffer_data` if successful, error otherwise
 */
static struct buffer_data *allocate_telemetry_buffers(struct gxp_dev *gxp,
						      size_t size)
{
	struct buffer_data *data;
	int i;
	int ret = 0;

	size = size < PAGE_SIZE ? PAGE_SIZE : size;

	/* TODO(b/260959553): Remove lockdep_assert_held during legacy telemetry removal */
	lockdep_assert_held(&gxp->core_telemetry_mgr->lock);

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return NULL;

	/* Allocate cache-coherent buffers for logging/tracing to */
	for (i = 0; i < GXP_NUM_CORES; i++) {
		/* Allocate a coherent buffer in the default domain */
		ret = gxp_dma_alloc_coherent_buf(gxp, NULL, size, GFP_KERNEL, 0,
						 &data->buffers[i]);
		if (ret) {
			dev_err(gxp->dev,
					"Failed to allocate coherent buffer\n");
			goto err_alloc;
		}
	}
	data->size = size;
	refcount_set(&data->ref_count, 1);
	data->is_enabled = false;

	return data;

err_alloc:
	while (i--)
		gxp_dma_free_coherent_buf(gxp, NULL, &data->buffers[i]);
	kfree(data);

	return ERR_PTR(ret);
}

/**
 * free_telemetry_buffers() - Unmap and free a `struct buffer_data`
 * @gxp: The GXP device the buffers were allocated for
 * @data: The descriptor of the buffers to unmap and free
 *
 * Caller must hold the core telemetry_manager's lock.
 */
static void free_telemetry_buffers(struct gxp_dev *gxp, struct buffer_data *data)
{
	int i;

	/* TODO(b/260959553): Remove lockdep_assert_held during legacy telemetry removal */
	lockdep_assert_held(&gxp->core_telemetry_mgr->lock);

	for (i = 0; i < GXP_NUM_CORES; i++)
		gxp_dma_free_coherent_buf(gxp, NULL, &data->buffers[i]);

	kfree(data);
}

/**
 * remap_telemetry_buffers() - Remaps a set of core telemetry buffers into a
 *                             user-space vm_area.
 * @gxp: The GXP device the buffers were allocated for
 * @vma: A vm area to remap the buffers into
 * @buff_data: The data describing a set of core telemetry buffers to remap
 *
 * Caller must hold the core telemetry_manager's lock.
 *
 * Return:
 * * 0         - Success
 * * otherwise - Error returned by `remap_pfn_range()`
 */
static int remap_telemetry_buffers(struct gxp_dev *gxp,
				   struct vm_area_struct *vma,
				   struct buffer_data *buff_data)
{
	unsigned long orig_pgoff = vma->vm_pgoff;
	int i;
	unsigned long offset;
	phys_addr_t phys;
	int ret = 0;

	lockdep_assert_held(&gxp->core_telemetry_mgr->lock);

	/* mmap the buffers */
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	vma->vm_flags |= VM_DONTCOPY | VM_DONTEXPAND | VM_DONTDUMP;
	vma->vm_pgoff = 0;

	for (i = 0; i < GXP_NUM_CORES; i++) {
		/*
		 * Remap each core's buffer a page at a time, in case it is not
		 * physically contiguous.
		 */
		for (offset = 0; offset < buff_data->size; offset += PAGE_SIZE) {
			/*
			 * `virt_to_phys()` does not work on memory allocated
			 * by `dma_alloc_coherent()`, so we have to use
			 * `iommu_iova_to_phys()` instead. Since all buffers
			 * are mapped to the default domain as well as any per-
			 * core domains, we can use it here to get the physical
			 * address of any valid IOVA, regardless of its core.
			 */
			phys = iommu_iova_to_phys(
				iommu_get_domain_for_dev(gxp->dev),
				buff_data->buffers[i].dma_addr + offset);
			ret = remap_pfn_range(
				vma,
				vma->vm_start + buff_data->size * i + offset,
				phys >> PAGE_SHIFT, PAGE_SIZE,
				vma->vm_page_prot);
			if (ret)
				goto out;
		}
	}

out:
	vma->vm_pgoff = orig_pgoff;
	/* TODO(b/260959553): Remove vma ops during legacy telemetry removal */
	vma->vm_ops = &telemetry_vma_ops;

	return ret;
}

int gxp_core_telemetry_mmap_buffers(struct gxp_dev *gxp, u8 type,
				    struct vm_area_struct *vma)
{
	int ret = 0;
	struct buffer_data *buff_data;
	size_t total_size = vma->vm_end - vma->vm_start;
	size_t size = total_size / GXP_NUM_CORES;

	if (!gxp->core_telemetry_mgr)
		return -ENODEV;

	if (type == GXP_TELEMETRY_TYPE_LOGGING)
		buff_data = gxp->core_telemetry_mgr->logging_buff_data;
	else if (type == GXP_TELEMETRY_TYPE_TRACING)
		buff_data = gxp->core_telemetry_mgr->tracing_buff_data;
	else
		return -EINVAL;
	/*
	 * Total size must divide evenly into a GXP_CORE_TELEMETRY_BUFFER_UNIT_SIZE
	 * aligned buffer per core.
	 */
	if (!total_size ||
	    total_size % (GXP_CORE_TELEMETRY_BUFFER_UNIT_SIZE * GXP_NUM_CORES)) {
		dev_warn(
			gxp->dev,
			"Invalid vma size(%lu bytes) requested for telemetry\n",
			total_size);
		return -EINVAL;
	}
	/*
	 * Per core log buffer size should be equal to pre allocated
	 * aligned buffer per core.
	 */
	if (size != buff_data->size) {
		dev_warn(
			gxp->dev,
			"Invalid per core requested telemetry buffer size(%lu bytes)\n",
			size);
		return -EINVAL;
	}
	mutex_lock(&gxp->core_telemetry_mgr->lock);
	ret = remap_telemetry_buffers(gxp, vma, buff_data);
	if (ret)
		goto err;
	vma->vm_private_data = NULL;
	mutex_unlock(&gxp->core_telemetry_mgr->lock);
	return 0;
err:
	mutex_unlock(&gxp->core_telemetry_mgr->lock);
	return ret;
}

int gxp_core_telemetry_mmap_buffers_legacy(struct gxp_dev *gxp, u8 type,
                                           struct vm_area_struct *vma)
{
	int ret = 0;
	struct telemetry_vma_data *vma_data;
	size_t total_size = vma->vm_end - vma->vm_start;
	size_t size = total_size / GXP_NUM_CORES;
	struct buffer_data *buff_data;
	int i;

	if (!gxp->core_telemetry_mgr)
		return -ENODEV;

	/* Total size must divide evenly into 1 page-aligned buffer per core */
	if (!total_size || total_size % (PAGE_SIZE * GXP_NUM_CORES))
		return -EINVAL;

	mutex_lock(&gxp->core_telemetry_mgr->lock);

	ret = check_telemetry_type_availability(gxp, type);
	if (ret)
		goto err;

	vma_data = kmalloc(sizeof(*vma_data), GFP_KERNEL);
	if (!vma_data) {
		ret = -ENOMEM;
		goto err;
	}

	buff_data = allocate_telemetry_buffers(gxp, size);
	if (IS_ERR(buff_data)) {
		ret = PTR_ERR(buff_data);
		goto err_free_vma_data;
	}

	ret = remap_telemetry_buffers(gxp, vma, buff_data);
	if (ret)
		goto err_free_buffers;

	vma_data->gxp = gxp;
	vma_data->buff_data = buff_data;
	vma_data->type = type;
	refcount_set(&vma_data->ref_count, 1);
	vma->vm_private_data = vma_data;

	/* Save book-keeping on the buffers in the core telemetry manager */
	if (type == GXP_TELEMETRY_TYPE_LOGGING)
		gxp->core_telemetry_mgr->logging_buff_data_legacy = buff_data;
	else /* type == GXP_TELEMETRY_TYPE_TRACING */
		gxp->core_telemetry_mgr->tracing_buff_data_legacy = buff_data;

	mutex_unlock(&gxp->core_telemetry_mgr->lock);

	return 0;

err_free_buffers:
	for (i = 0; i < GXP_NUM_CORES; i++)
		gxp_dma_free_coherent_buf(gxp, NULL, &buff_data->buffers[i]);
	kfree(buff_data);

err_free_vma_data:
	kfree(vma_data);

err:
	mutex_unlock(&gxp->core_telemetry_mgr->lock);
	return ret;
}

int gxp_core_telemetry_enable(struct gxp_dev *gxp, u8 type)
{
	struct buffer_data *data;
	int ret = 0;
	uint core;
	struct gxp_virtual_device *vd;

	/*
	 * `vd_semaphore` cannot be acquired while holding the core telemetry
	 * lock, so acquire it here before locking the core telemetry lock.
	 */
	down_read(&gxp->vd_semaphore);
	mutex_lock(&gxp->core_telemetry_mgr->lock);

	switch (type) {
	case GXP_TELEMETRY_TYPE_LOGGING:
		data = gxp->core_telemetry_mgr->logging_buff_data_legacy;
		break;
	case GXP_TELEMETRY_TYPE_TRACING:
		data = gxp->core_telemetry_mgr->tracing_buff_data_legacy;
		break;
	default:
		ret = -EINVAL;
		goto out;
	}

	if (!data) {
		ret = -ENXIO;
		goto out;
	}

	/* Map the buffers for any cores already running */
	for (core = 0; core < GXP_NUM_CORES; core++) {
		vd = gxp->core_to_vd[core];
		if (vd != NULL) {
			ret = gxp_dma_map_allocated_coherent_buffer(
				gxp, &data->buffers[core], vd->domain, 0);
			if (ret)
				goto err;
		}
	}

	/* Populate the buffer fields in firmware-data */
	data->host_status |= GXP_CORE_TELEMETRY_HOST_STATUS_ENABLED;
	gxp_fw_data_set_core_telemetry_descriptors(gxp, type, data->host_status,
						   data->buffers, data->size);

	/* Notify any running cores that firmware-data was updated */
	for (core = 0; core < GXP_NUM_CORES; core++) {
		if (gxp_is_fw_running(gxp, core))
			gxp_notification_send(gxp, core,
					      CORE_NOTIF_TELEMETRY_STATUS);
	}

	refcount_inc(&data->ref_count);
	data->is_enabled = true;

	goto out;
err:
	while (core--) {
		vd = gxp->core_to_vd[core];
		if (vd)
			gxp_dma_unmap_allocated_coherent_buffer(
				gxp, vd->domain, &data->buffers[core]);
	}

out:
	mutex_unlock(&gxp->core_telemetry_mgr->lock);
	up_read(&gxp->vd_semaphore);

	return ret;
}

/**
 * notify_core_and_wait_for_disable() - Notify a core that telemetry state has
 *                                      been changed by the host and wait for
 *                                      the core to stop using telemetry.
 * @gxp: The GXP device core telemetry is changing for
 * @core: The core in @gxp to notify of the telemetry state change
 * @type: Either `GXP_TELEMETRY_TYPE_LOGGING` or `GXP_TELEMETRY_TYPE_TRACING`
 *
 * Caller must hold `core_telemetry_mgr->lock`.
 * Caller must hold `gxp->vd_semaphore` for reading only.
 * It is not allowed to hold `gxp->vd_semaphore` for writing, since this
 * function needs to release `gxp->vd_semaphore` at different points to sleep.
 *
 * Return:
 * * 0      - Firmware on @core is no longer using telemetry of @type
 * * -ENXIO - Firmware on @core is unresponsive
 */
static int notify_core_and_wait_for_disable(struct gxp_dev *gxp, uint core,
					    u8 type)
{
	uint retries_left = 50;

	lockdep_assert_held(&gxp->core_telemetry_mgr->lock);
	lockdep_assert_held_read(&gxp->vd_semaphore);

	gxp_notification_send(gxp, core, CORE_NOTIF_TELEMETRY_STATUS);

	/* Wait for ACK from firmware */
	while (is_telemetry_enabled(gxp, core, type) &&
	       gxp_is_fw_running(gxp, core) && retries_left) {
		/* Release vd_semaphore while waiting */
		up_read(&gxp->vd_semaphore);

		/*
		 * The VD lock must be held to check if firmware is running, so
		 * the wait condition is only whether the firmware data has been
		 * updated to show the core disabling telemetry.
		 *
		 * If a core does stop running firmware while this function is
		 * asleep, it will be seen at the next timeout.
		 */
		wait_event_timeout(gxp->core_telemetry_mgr->waitq,
				   !is_telemetry_enabled(gxp, core, type),
				   msecs_to_jiffies(10));
		retries_left--;

		/*
		 * No function may attempt to acquire the `vd_semaphore` while
		 * holding the core telemetry lock, so it must be released, then
		 * re-acquired once the `vd_semaphore` is held.
		 */
		mutex_unlock(&gxp->core_telemetry_mgr->lock);
		down_read(&gxp->vd_semaphore);
		mutex_lock(&gxp->core_telemetry_mgr->lock);
	}

	/*
	 * If firmware has stopped running altogether, that is sufficient to be
	 * considered disabled. If firmware is started on this core again, it
	 * is responsible for clearing its status.
	 */
	if (unlikely(is_telemetry_enabled(gxp, core, type) &&
		     gxp_is_fw_running(gxp, core)))
		return -ENXIO;

	return 0;
}

/**
 * telemetry_disable_locked() - Helper function to break out the actual
 *                              process of disabling core telemetry so that it
 *                              can be invoked by internal functions that are
 *                              already holding the core telemetry lock.
 * @gxp: The GXP device to disable either logging or tracing for
 * @type: Either `GXP_TELEMETRY_TYPE_LOGGING` or `GXP_TELEMETRY_TYPE_TRACING`
 *
 * Caller must hold `core_telemetry_mgr->lock`.
 * Caller must hold `gxp->vd_semaphore` for reading only.
 * It is not allowed to hold `gxp->vd_semaphore` for writing, since this
 * function needs to release `gxp->vd_semaphore` at different points to sleep.
 *
 * Return:
 * * 0       - Success
 * * -EINVAL - The @type provided is not valid
 * * -ENXIO  - Buffers for @type have not been created/mapped yet
 */
static int telemetry_disable_locked(struct gxp_dev *gxp, u8 type)
{
	struct buffer_data *data;
	int ret = 0;
	uint core;
	struct gxp_virtual_device *vd;

	lockdep_assert_held(&gxp->core_telemetry_mgr->lock);
	lockdep_assert_held_read(&gxp->vd_semaphore);

	/* Cleanup core telemetry manager's book-keeping */
	switch (type) {
	case GXP_TELEMETRY_TYPE_LOGGING:
		data = gxp->core_telemetry_mgr->logging_buff_data_legacy;
		break;
	case GXP_TELEMETRY_TYPE_TRACING:
		data = gxp->core_telemetry_mgr->tracing_buff_data_legacy;
		break;
	default:
		return -EINVAL;
	}

	if (!data)
		return -ENXIO;

	if (!(data->host_status & GXP_CORE_TELEMETRY_HOST_STATUS_ENABLED))
		return 0;

	data->is_enabled = false;

	/* Clear the log buffer fields in firmware-data */
	data->host_status &= ~GXP_CORE_TELEMETRY_HOST_STATUS_ENABLED;
	gxp_fw_data_set_core_telemetry_descriptors(gxp, type, data->host_status, NULL, 0);

	/* Notify any running cores that firmware-data was updated */
	for (core = 0; core < GXP_NUM_CORES; core++) {
		if (gxp_is_fw_running(gxp, core)) {
			ret = notify_core_and_wait_for_disable(gxp, core, type);
			if (ret)
				dev_warn(
					gxp->dev,
					"%s: core%u failed to disable telemetry (type=%u, ret=%d)\n",
					__func__, core, type, ret);
		}
		vd = gxp->core_to_vd[core];
		if (vd)
			gxp_dma_unmap_allocated_coherent_buffer(
				gxp, vd->domain, &data->buffers[core]);
	}

	if (refcount_dec_and_test(&data->ref_count)) {
		switch (type) {
		case GXP_TELEMETRY_TYPE_LOGGING:
			gxp->core_telemetry_mgr->logging_buff_data_legacy = NULL;
			break;
		case GXP_TELEMETRY_TYPE_TRACING:
			gxp->core_telemetry_mgr->tracing_buff_data_legacy = NULL;
			break;
		default:
			/* NO-OP, we returned above if `type` was invalid */
			break;
		}
		free_telemetry_buffers(gxp, data);
	}

	return 0;
}

int gxp_core_telemetry_disable(struct gxp_dev *gxp, u8 type)
{
	int ret;

	/*
	 * `vd_semaphore` cannot be acquired while holding the core telemetry
	 * lock, so acquire it here before locking the core telemetry lock.
	 */
	down_read(&gxp->vd_semaphore);
	mutex_lock(&gxp->core_telemetry_mgr->lock);

	ret = telemetry_disable_locked(gxp, type);

	mutex_unlock(&gxp->core_telemetry_mgr->lock);
	up_read(&gxp->vd_semaphore);

	return ret;
}

int gxp_core_telemetry_register_eventfd(struct gxp_dev *gxp, u8 type, int fd)
{
	struct eventfd_ctx *new_ctx;
	struct eventfd_ctx **ctx_to_set = NULL;
	int ret = 0;

	new_ctx = eventfd_ctx_fdget(fd);
	if (IS_ERR(new_ctx))
		return PTR_ERR(new_ctx);

	mutex_lock(&gxp->core_telemetry_mgr->lock);

	switch (type) {
	case GXP_TELEMETRY_TYPE_LOGGING:
		ctx_to_set = &gxp->core_telemetry_mgr->logging_efd;
		break;
	case GXP_TELEMETRY_TYPE_TRACING:
		ctx_to_set = &gxp->core_telemetry_mgr->tracing_efd;
		break;
	default:
		ret = -EINVAL;
		eventfd_ctx_put(new_ctx);
		goto out;
	}

	if (*ctx_to_set) {
		dev_warn(
			gxp->dev,
			"Replacing existing core telemetry eventfd (type=%u)\n",
			type);
		eventfd_ctx_put(*ctx_to_set);
	}

	*ctx_to_set = new_ctx;

out:
	mutex_unlock(&gxp->core_telemetry_mgr->lock);
	return ret;
}

int gxp_core_telemetry_unregister_eventfd(struct gxp_dev *gxp, u8 type)
{
	int ret = 0;

	mutex_lock(&gxp->core_telemetry_mgr->lock);

	switch (type) {
	case GXP_TELEMETRY_TYPE_LOGGING:
		if (gxp->core_telemetry_mgr->logging_efd)
			eventfd_ctx_put(gxp->core_telemetry_mgr->logging_efd);
		gxp->core_telemetry_mgr->logging_efd = NULL;
		break;
	case GXP_TELEMETRY_TYPE_TRACING:
		if (gxp->core_telemetry_mgr->tracing_efd)
			eventfd_ctx_put(gxp->core_telemetry_mgr->tracing_efd);
		gxp->core_telemetry_mgr->tracing_efd = NULL;
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&gxp->core_telemetry_mgr->lock);

	return ret;
}

struct work_struct *
gxp_core_telemetry_get_notification_handler(struct gxp_dev *gxp, uint core)
{
	struct gxp_core_telemetry_manager *mgr = gxp->core_telemetry_mgr;

	if (!mgr || core >= GXP_NUM_CORES)
		return NULL;

	return &mgr->notification_works[core].work;
}

void gxp_core_telemetry_exit(struct gxp_dev *gxp)
{
	struct buffer_data *log_buff_data, *trace_buff_data;
	struct gxp_core_telemetry_manager *mgr = gxp->core_telemetry_mgr;

	if (!mgr) {
		dev_warn(gxp->dev, "Core telemetry manager was not allocated\n");
		return;
	}

	/* TODO(b/260959553): Remove mutex_lock/unlock during legacy telemetry removal */
	mutex_lock(&gxp->core_telemetry_mgr->lock);
	log_buff_data = mgr->logging_buff_data;
	trace_buff_data = mgr->tracing_buff_data;

	if (!IS_ERR_OR_NULL(log_buff_data))
		free_telemetry_buffers(gxp, log_buff_data);

	if (!IS_ERR_OR_NULL(trace_buff_data))
		free_telemetry_buffers(gxp, trace_buff_data);

	mutex_unlock(&gxp->core_telemetry_mgr->lock);

	if (!IS_ERR_OR_NULL(gxp->core_telemetry_mgr->logging_efd)) {
		dev_warn(gxp->dev, "logging_efd was not released\n");
		eventfd_ctx_put(gxp->core_telemetry_mgr->logging_efd);
		gxp->core_telemetry_mgr->logging_efd = NULL;
	}

	if (!IS_ERR_OR_NULL(gxp->core_telemetry_mgr->tracing_efd)) {
		dev_warn(gxp->dev, "tracing_efd was not released\n");
		eventfd_ctx_put(gxp->core_telemetry_mgr->tracing_efd);
		gxp->core_telemetry_mgr->tracing_efd = NULL;
	}

	mutex_destroy(&mgr->lock);
	devm_kfree(gxp->dev, mgr);
	gxp->core_telemetry_mgr = NULL;
}
