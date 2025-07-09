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

#define DEBUGFS_LOG_BUFF "log"
#define DEBUGFS_LOG_EVENTFD "log_eventfd"

static uint gxp_core_telemetry_buffer_size = CORE_TELEMETRY_DEFAULT_BUFFER_SIZE;
module_param_named(core_telemetry_buffer_size, gxp_core_telemetry_buffer_size, uint, 0660);

void gxp_core_telemetry_status_notify(struct gxp_dev *gxp, uint core)
{
	struct gxp_core_telemetry_manager *mgr = gxp->core_telemetry_mgr;

	/* Wake any threads waiting on a core telemetry disable ACK */
	wake_up(&mgr->waitq);

	/* Signal the appropriate eventfd for any active core telemetry types */
	mutex_lock(&mgr->lock);

	if (mgr->efd)
		eventfd_signal(mgr->efd, 1);

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
 *
 * Return:
 * * 0         - Success
 * * otherwise - Error returned by `gxp_fw_data_set_core_telemetry_descriptors()`
 */
static int enable_telemetry_buffers(struct gxp_dev *gxp, struct buffer_data *data)
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
	ret = gxp_fw_data_set_core_telemetry_descriptors(gxp, data->host_status, data->buffers,
							 data->size);

	if (ret) {
		dev_err(gxp->dev,
			"setting telemetry buffers in scratchpad region failed (ret=%d).",
			ret);
		return ret;
	}

	data->is_enabled = true;
	return 0;
}

int gxp_secure_core_telemetry_init(struct gxp_dev *gxp, phys_addr_t phys)
{
	int i;
	void *vaddr, *buffer_vaddr;
	size_t secure_carveout_buffer_size =
		GXP_NUM_CORES * SECURE_CORE_TELEMETRY_BUFFER_SIZE;

	if (!phys)
		return 0;

	buffer_vaddr = memremap(phys, secure_carveout_buffer_size, MEMREMAP_WC);
	if (!buffer_vaddr) {
		dev_err(gxp->dev, "memremap failed for secure buffer.");
		return -EINVAL;
	}

	for (i = 0; i < GXP_NUM_CORES; i++) {
		vaddr = buffer_vaddr + i * SECURE_CORE_TELEMETRY_BUFFER_SIZE;
		/*
		 * First 64 bytes of per core telemetry buffers are reserved
		 * for buffer metadata header. Zeroing it.
		 */
		memset(vaddr, 0, GXP_CORE_TELEMETRY_BUFFER_HEADER_SIZE);
		/* First 4 bytes of buffer header are reserved for valid_magic field. */
		*((uint *)vaddr) = GXP_TELEMETRY_SECURE_BUFFER_VALID_MAGIC_CODE;
	}

	gxp->core_telemetry_mgr->secure_buffer_carveout_addr = phys;
	memunmap(buffer_vaddr);
	return 0;
}

static int debugfs_log_buff_set(void *data, u64 val)
{
	struct gxp_dev *gxp = (struct gxp_dev *)data;
	int i;
	struct gxp_coherent_buf *buffers;
	u64 *ptr;

	mutex_lock(&gxp->core_telemetry_mgr->lock);

	buffers = gxp->core_telemetry_mgr->buff_data->buffers;
	for (i = 0; i < GXP_NUM_CORES; i++) {
		ptr = buffers[i].vaddr;
		*ptr = val;
	}

	mutex_unlock(&gxp->core_telemetry_mgr->lock);

	return 0;
}

static int debugfs_log_buff_get(void *data, u64 *val)
{
	struct gxp_dev *gxp = (struct gxp_dev *)data;
	struct gxp_coherent_buf *buffers;

	mutex_lock(&gxp->core_telemetry_mgr->lock);

	buffers = gxp->core_telemetry_mgr->buff_data->buffers;
	*val = *(u64 *)(buffers[0].vaddr);

	mutex_unlock(&gxp->core_telemetry_mgr->lock);

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(debugfs_log_buff_fops, debugfs_log_buff_get,
			 debugfs_log_buff_set, "%llu\n");

static int debugfs_log_eventfd_signal_set(void *data, u64 val)
{
	struct gxp_dev *gxp = (struct gxp_dev *)data;
	int ret = 0;

	mutex_lock(&gxp->core_telemetry_mgr->lock);

	if (!gxp->core_telemetry_mgr->efd) {
		ret = -ENODEV;
		goto out;
	}

	ret = eventfd_signal(gxp->core_telemetry_mgr->efd, 1);

out:
	mutex_unlock(&gxp->core_telemetry_mgr->lock);

	return ret;
}

DEFINE_DEBUGFS_ATTRIBUTE(debugfs_log_eventfd_signal_fops, NULL,
			 debugfs_log_eventfd_signal_set, "%llu\n");

int gxp_core_telemetry_init(struct gxp_dev *gxp)
{
	struct gxp_core_telemetry_manager *mgr;
	struct buffer_data *buff_data;
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

	buff_data = allocate_telemetry_buffers(gxp, gxp_core_telemetry_buffer_size);
	if (IS_ERR_OR_NULL(buff_data)) {
		dev_warn(gxp->dev, "Failed to allocate per core telemetry buffer of %u bytes\n",
			 gxp_core_telemetry_buffer_size);
		ret = -ENOMEM;
		goto err_free_buffers;
	}

	ret = enable_telemetry_buffers(gxp, buff_data);
	if (ret) {
		dev_warn(gxp->dev, "enable telemetry buffer failed (ret=%d)", ret);
		goto err_free;
	}

	gxp->core_telemetry_mgr->buff_data = buff_data;

	debugfs_create_file(DEBUGFS_LOG_BUFF, 0600, gxp->d_entry, gxp,
			    &debugfs_log_buff_fops);
	debugfs_create_file(DEBUGFS_LOG_EVENTFD, 0200, gxp->d_entry, gxp,
			    &debugfs_log_eventfd_signal_fops);

	return 0;

err_free:
	free_telemetry_buffers(gxp, buff_data);
err_free_buffers:
	mutex_destroy(&mgr->lock);
	devm_kfree(gxp->dev, mgr);
	gxp->core_telemetry_mgr = NULL;
	return ret;
}

/**
 * allocate_telemetry_buffers() - Allocate and populate a `struct buffer_data`,
 *                                including allocating and mapping one coherent
 *                                buffer of @size bytes per core.
 * @gxp: The GXP device to allocate the buffers for
 * @size: The size of buffer to allocate for each core
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
		data->buffers[i].dsp_addr = GXP_TELEMETRY_IOVA_BASE + i * size;
	}
	data->size = size;
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
 */
static void free_telemetry_buffers(struct gxp_dev *gxp, struct buffer_data *data)
{
	int i;

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
	vm_flags_set(vma, VM_DONTCOPY | VM_DONTEXPAND | VM_DONTDUMP);
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

	return ret;
}

int gxp_secure_core_telemetry_mmap_buffers(struct gxp_dev *gxp,
					   struct vm_area_struct *vma)
{
	int ret = 0;
	phys_addr_t phys = gxp->core_telemetry_mgr->secure_buffer_carveout_addr;
	size_t total_size = vma->vm_end - vma->vm_start;
	size_t secure_carveout_buffer_size =
		GXP_NUM_CORES * SECURE_CORE_TELEMETRY_BUFFER_SIZE;
	unsigned long orig_pgoff = vma->vm_pgoff;

	if (!phys) {
		dev_warn(gxp->dev, "Secure buffer is not mapped.\n");
		return -ENODATA;
	}

	/* Total size must be equal to reserved secure buffer carveout size. */
	if (total_size != secure_carveout_buffer_size) {
		dev_err(gxp->dev, "Invalid vma size requested (%lu != %lu).\n",
			total_size, secure_carveout_buffer_size);
		return -EINVAL;
	}

	mutex_lock(&gxp->core_telemetry_mgr->lock);

	/* mmap the buffers */
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	vm_flags_set(vma, VM_DONTCOPY | VM_DONTEXPAND | VM_DONTDUMP);
	vma->vm_pgoff = 0;
	/*
	 * Remapping the physically contiguous carveout area reserved
	 * for secure telemetry buffer.
	 */
	ret = remap_pfn_range(vma, vma->vm_start, phys >> PAGE_SHIFT,
			      secure_carveout_buffer_size, vma->vm_page_prot);
	if (ret)
		goto err;
	vma->vm_pgoff = orig_pgoff;
	vma->vm_private_data = NULL;
err:
	mutex_unlock(&gxp->core_telemetry_mgr->lock);
	return ret;
}

int gxp_core_telemetry_mmap_buffers(struct gxp_dev *gxp, struct vm_area_struct *vma)
{
	int ret = 0;
	struct buffer_data *buff_data;
	size_t total_size = vma->vm_end - vma->vm_start;
	size_t size = total_size / GXP_NUM_CORES;

	if (!gxp->core_telemetry_mgr)
		return -ENODEV;

	buff_data = gxp->core_telemetry_mgr->buff_data;
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

int gxp_core_telemetry_register_eventfd(struct gxp_dev *gxp, int fd)
{
	struct eventfd_ctx *new_ctx;
	struct eventfd_ctx **ctx_to_set = NULL;

	new_ctx = eventfd_ctx_fdget(fd);
	if (IS_ERR(new_ctx))
		return PTR_ERR(new_ctx);

	mutex_lock(&gxp->core_telemetry_mgr->lock);

	ctx_to_set = &gxp->core_telemetry_mgr->efd;

	if (*ctx_to_set) {
		dev_warn(gxp->dev, "Replacing existing core telemetry eventfd\n");
		eventfd_ctx_put(*ctx_to_set);
	}

	*ctx_to_set = new_ctx;

	mutex_unlock(&gxp->core_telemetry_mgr->lock);
	return 0;
}

void gxp_core_telemetry_unregister_eventfd(struct gxp_dev *gxp)
{
	mutex_lock(&gxp->core_telemetry_mgr->lock);

	if (gxp->core_telemetry_mgr->efd)
		eventfd_ctx_put(gxp->core_telemetry_mgr->efd);
	gxp->core_telemetry_mgr->efd = NULL;

	mutex_unlock(&gxp->core_telemetry_mgr->lock);
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
	struct buffer_data *buff_data;
	struct gxp_core_telemetry_manager *mgr = gxp->core_telemetry_mgr;

	if (!mgr) {
		dev_warn(gxp->dev, "Core telemetry manager was not allocated\n");
		return;
	}

	debugfs_remove(debugfs_lookup(DEBUGFS_LOG_BUFF, gxp->d_entry));
	debugfs_remove(debugfs_lookup(DEBUGFS_LOG_EVENTFD, gxp->d_entry));

	buff_data = mgr->buff_data;

	if (!IS_ERR_OR_NULL(buff_data))
		free_telemetry_buffers(gxp, buff_data);

	if (!IS_ERR_OR_NULL(gxp->core_telemetry_mgr->efd)) {
		dev_warn(gxp->dev, "efd was not released\n");
		eventfd_ctx_put(gxp->core_telemetry_mgr->efd);
		gxp->core_telemetry_mgr->efd = NULL;
	}

	mutex_destroy(&mgr->lock);
	devm_kfree(gxp->dev, mgr);
	gxp->core_telemetry_mgr = NULL;
}
