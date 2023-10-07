/* SPDX-License-Identifier: GPL-2.0 */
/*
 * GXP DMA interface.
 *
 * Copyright (C) 2021 Google LLC
 */
#ifndef __GXP_DMA_H__
#define __GXP_DMA_H__

#include <linux/dma-buf.h>
#include <linux/dma-direction.h>
#include <linux/dma-mapping.h>
#include <linux/iommu.h>
#include <linux/types.h>
#if (IS_ENABLED(CONFIG_GXP_TEST) || IS_ENABLED(CONFIG_ANDROID)) &&             \
	!IS_ENABLED(CONFIG_GXP_GEM5)
#include <soc/google/tpu-ext.h>
#endif

#include "gxp-internal.h"

struct gxp_iommu_domain {
	struct iommu_domain *domain;
	uint ctx_id;
};

struct gxp_coherent_buf {
	void *vaddr; /* kernel VA, no allocation if NULL */
	/* TODO(b/249030390): Use standard DMA-IOMMU APIs returned address */
	dma_addr_t dma_addr; /* DMA handle obtained from DMA-IOMMU APIs. */
	/*
	 * IOVA to be accessed by the device. Equal to @dma_addr when there is
	 * no self-managed IOMMU.
	 */
	dma_addr_t dsp_addr;
	u64 phys_addr; /* physical address, if available */
	size_t size;
};

struct gxp_dma_manager {
	struct rb_root mapping_tree;
};

/*
 * Error value to be returned in place of a dma_addr_t when a mapping fails.
 *
 * On newer kernels, this is defined in <linux/dma-mapping.h>. Redefined here
 * for older kernels, so clients can check for this value without worrying
 * which kernel version they're compiled for.
 */
#ifndef DMA_MAPPING_ERROR
#define DMA_MAPPING_ERROR (~(dma_addr_t)0)
#endif

/**
 * gxp_iommu_map() - Create mappings in iommu
 * @gxp: The GXP device
 * @gdomain: The IOMMU domain to create mappings in.
 *
 * Return: 0 on success or negative value indicating error
 */
int gxp_iommu_map(struct gxp_dev *gxp, struct gxp_iommu_domain *gdomain,
		  unsigned long iova, phys_addr_t paddr, size_t size, int prot);

/**
 * gxp_iommu_unmap() - Reverts mappings created by gxp_iommu_map()
 * @gxp: The GXP device
 * @gdomain: The IOMMU domain to revert mappings in.
 */
void gxp_iommu_unmap(struct gxp_dev *gxp, struct gxp_iommu_domain *gdomain,
		     unsigned long iova, size_t size);

/**
 * gxp_dma_init() - Initialize the GXP DMA subsystem
 * @gxp: The GXP device to initialize DMA for
 *
 * Return:
 * * 0       - DMA initialized successfully
 * * -EIO    - Failed to initialize underlying IOMMU hardware
 * * -ENODEV - The necessary hardware or device tree entries are missing
 * * -ENOMEM - Insufficient memory is available to initialize the interface
 */
int gxp_dma_init(struct gxp_dev *gxp);

/**
 * gxp_dma_exit() - Tear down the GXP DMA subsystem and release hardware
 * @gxp: The GXP device to tear down DMA for
 */
void gxp_dma_exit(struct gxp_dev *gxp);

/**
 * gxp_dma_domain_attach_device() - Attach the page table to the device and
 * perform necessary initialization.
 * @gxp: The GXP device to attach
 * @gdomain: The IOMMU domain to be attached.
 * @core_list: The physical cores to attach.
 *
 * Caller ensures a BLOCK wakelock is hold for the iommu attaching.
 */
int gxp_dma_domain_attach_device(struct gxp_dev *gxp,
				 struct gxp_iommu_domain *gdomain,
				 uint core_list);

/**
 * gxp_dma_domain_detach_device() - Detach the page table from the device.
 * @gxp: The GXP device to detach
 * @gdomain: The IOMMU domain to be detached
 *
 * Caller ensures a BLOCK wakelock is hold for the iommu detaching.
 */
void gxp_dma_domain_detach_device(struct gxp_dev *gxp,
				  struct gxp_iommu_domain *gdomain);

/**
 * gxp_dma_init_default_resources() - Set the various buffers/registers with
 * fixed IOVA
 * @gxp: The GXP device to set up the IOVAs for
 *
 * GXP firmware expects several buffers and registers to be mapped to fixed
 * locations in their IOVA space. This function sets up these fixed IOVAs.
 */
void gxp_dma_init_default_resources(struct gxp_dev *gxp);

/**
 * gxp_dma_map_core_resources() - Map the various buffers/registers with
 * fixed IOVAs on the IOMMU domain.
 * @gxp: The GXP device to set up the mappings for
 * @gdomain: The IOMMU domain to be mapped on
 * @core_list: The physical cores that may use the domain
 * @slice_index: The index of slice of shared buffer to be mapped
 *
 * GXP firmware expects several buffers and registers to be mapped to fixed
 * locations in their IOVA space. This function initializes all those mappings
 * for the core.
 *
 * This function must not be called until after all the `vaddr` and `size`
 * fields of every `struct gxp_mapped_resource` inside of @gxp have been
 * initialized.
 *
 * Return:
 * * 0    - Mappings created successfully
 * * -EIO - Failed to create one or more of the mappings
 */
int gxp_dma_map_core_resources(struct gxp_dev *gxp,
			       struct gxp_iommu_domain *gdomain, uint core_list,
			       u8 slice_index);

/**
 * gxp_dma_unmap_core_resources() - Unmap the IOVAs mapped by
 * gxp_dma_map_core_resources()
 * @gxp: The GXP device that was passed to gxp_dma_map_core_resources()
 * @gdomain: The IOMMU domain to be unmapped
 * @core_list: The physical cores the IOVAs were mapped for
 *
 * GXP firmware expects several buffers and registers to be mapped to fixed
 * locations in their IOVA space. This function releases all those mappings.
 */
void gxp_dma_unmap_core_resources(struct gxp_dev *gxp,
				  struct gxp_iommu_domain *gdomain,
				  uint core_list);

#if (IS_ENABLED(CONFIG_GXP_TEST) || IS_ENABLED(CONFIG_ANDROID)) &&             \
	!IS_ENABLED(CONFIG_GXP_GEM5)
/**
 * gxp_dma_map_tpu_buffer() - Map the tpu mbx queue buffers with fixed IOVAs
 * @gxp: The GXP device to set up the mappings for
 * @gdomain: The IOMMU domain to be mapped on
 * @core_list: A bitfield enumerating the physical cores the mapping is for
 * @mbx_info: Structure holding TPU-DSP mailbox queue buffer information
 *
 * Return:
 * * 0    - Mappings created successfully
 * * -EIO - Failed to create the mappings
 */
int gxp_dma_map_tpu_buffer(struct gxp_dev *gxp,
			   struct gxp_iommu_domain *gdomain, uint core_list,
			   struct edgetpu_ext_mailbox_info *mbx_info);

/**
 * gxp_dma_unmap_tpu_buffer() - Unmap IOVAs mapped by gxp_dma_map_tpu_buffer()
 * @gxp: The GXP device that was passed to gxp_dma_map_tpu_buffer()
 * @gdomain: The IOMMU domain the mappings were mapped on
 * @mbx_desc: Structure holding info for already mapped TPU-DSP mailboxes.
 */
void gxp_dma_unmap_tpu_buffer(struct gxp_dev *gxp,
			      struct gxp_iommu_domain *gdomain,
			      struct gxp_tpu_mbx_desc mbx_desc);
#endif // (CONFIG_GXP_TEST || CONFIG_ANDROID) && !CONFIG_GXP_GEM5

/**
 * gxp_dma_map_allocated_coherent_buffer() - Map a coherent buffer
 * @gxp: The GXP device to map the allocated buffer for
 * @buf: The coherent buffer
 * @gdomain: The IOMMU domain to be mapped on
 * @gxp_dma_flags: The type of mapping to create; currently unused
 *
 * Return: 0 on success else error code
 */
int gxp_dma_map_allocated_coherent_buffer(struct gxp_dev *gxp,
					  struct gxp_coherent_buf *buf,
					  struct gxp_iommu_domain *gdomain,
					  uint gxp_dma_flags);
/**
 * gxp_dma_unmap_allocated_coherent_buffer() - Unmap a coherent buffer
 * @gxp: The GXP device the buffer was allocated and mapped for
 * @gdomain: The IOMMU domain the mapping was mapped
 * @buf: The coherent buffer
 */
void gxp_dma_unmap_allocated_coherent_buffer(struct gxp_dev *gxp,
					     struct gxp_iommu_domain *gdomain,
					     struct gxp_coherent_buf *buf);
/**
 * gxp_dma_alloc_coherent() - Allocate and map a coherent buffer for a GXP core
 * @gxp: The GXP device to map the allocated buffer for
 * @gdomain: The IOMMU domain the mapping to be mapped on
 * @size: The size of the buffer to be allocated, in bytes
 * @flag: The type of memory to allocate (see kmalloc)
 * @gxp_dma_flags: The type of mapping to create; Currently unused
 * @buffer: The coherent buffer
 *
 * Return: 0 on success else error code
 *
 * If the passed @domain is a null pointer, this function will only allocate a
 * buffer but not map it to the domain.
 * Note: Allocated buffers size may be larger than the requested size.
 */
int gxp_dma_alloc_coherent_buf(struct gxp_dev *gxp,
			       struct gxp_iommu_domain *gdomain, size_t size,
			       gfp_t flag, uint gxp_dma_flags,
			       struct gxp_coherent_buf *buffer);
/**
 * gxp_dma_free_coherent() - Unmap and free a coherent buffer
 * @gxp: The GXP device the buffer was allocated and mapped for
 * @gdomain: The IOMMU domain the mapping was mapped to
 * @buf: The coherent buffer
 *
 * If the buffer is mapped via `gxp_dma_map_allocated_coherent_buffer`, the
 * caller must call `gxp_dma_unmap_allocated_coherent_buffer` to unmap before
 * freeing the buffer.
 *
 * If the passed @domain is a null pointer, this function will only free the
 * buffer but not do any unmapping.
 */
void gxp_dma_free_coherent_buf(struct gxp_dev *gxp,
			       struct gxp_iommu_domain *gdomain,
			       struct gxp_coherent_buf *buf);

/**
 * gxp_dma_map_sg() - Create a mapping for a scatter-gather list
 * @gxp: The GXP device to map the scatter-gather list for
 * @gdomain: The IOMMU domain to be mapped
 * @sg: The scatter-gather list of the buffer to be mapped
 * @nents: The number of entries in @sg
 * @direction: DMA direction
 * @attrs: The same set of flags used by the base DMA API
 * @gxp_dma_flags: The type of mapping to create
 *
 * Return: The number of scatter-gather entries mapped to
 */
int gxp_dma_map_sg(struct gxp_dev *gxp, struct gxp_iommu_domain *gdomain,
		   struct scatterlist *sg, int nents,
		   enum dma_data_direction direction, unsigned long attrs,
		   uint gxp_dma_flags);
/**
 * gxp_dma_unmap_sg() - Unmap a scatter-gather list
 * @gxp: The GXP device the scatter-gather list was mapped for
 * @gdomain: The IOMMU domain mapping was mapped on
 * @sg: The scatter-gather list to unmap; The same one passed to
 *      `gxp_dma_map_sg()`
 * @nents: The number of entries in @sg; Same value passed to `gxp_dma_map_sg()`
 * @direction: DMA direction; Same as passed to `gxp_dma_map_sg()`
 * @attrs: The same set of flags used by the base DMA API
 */
void gxp_dma_unmap_sg(struct gxp_dev *gxp, struct gxp_iommu_domain *gdomain,
		      struct scatterlist *sg, int nents,
		      enum dma_data_direction direction, unsigned long attrs);

/**
 * gxp_dma_map_iova_sgt() - Create a mapping for a scatter-gather list, with specific IOVA.
 * @gxp: The GXP device to map the scatter-gather list for
 * @gdomain: The IOMMU domain to be mapped
 * @iova: The IOVA to be mapped.
 * @sgt: The scatter-gather list table of the buffer to be mapped
 * @prot: The protection bits to be passed to IOMMU API
 *
 * Return: 0 on success. Negative errno otherwise.
 */
int gxp_dma_map_iova_sgt(struct gxp_dev *gxp, struct gxp_iommu_domain *gdomain,
			 dma_addr_t iova, struct sg_table *sgt, int prot);
/**
 * gxp_dma_unmap_iova_sgt() - Revert gxp_dma_map_iova_sgt()
 * @gxp: The GXP device the scatter-gather list was mapped for
 * @gdomain: The IOMMU domain mapping was mapped on
 * @iova: The IOVA to be un-mapped.
 * @sgt: The scatter-gather list to unmap; The same one passed to
 *      `gxp_dma_map_iova_sgt()`
 */
void gxp_dma_unmap_iova_sgt(struct gxp_dev *gxp,
			    struct gxp_iommu_domain *gdomain, dma_addr_t iova,
			    struct sg_table *sgt);

/**
 * gxp_dma_sync_sg_for_cpu() - Sync sg list for reading by the  CPU
 * @gxp: The GXP device the mapping was created for
 * @sg: The mapped scatter-gather list to be synced
 * @nents: The number of entries in @sg
 * @direction: DMA direction
 */
void gxp_dma_sync_sg_for_cpu(struct gxp_dev *gxp, struct scatterlist *sg,
			     int nents, enum dma_data_direction direction);
/**
 * gxp_dma_sync_sg_for_device() - Sync sg list for reading by the device
 * @gxp: The GXP device the mapping was created for
 * @sg: The mapped scatter-gather list to be synced
 * @nents: The number of entries in @sg
 * @direction: DMA direction
 */
void gxp_dma_sync_sg_for_device(struct gxp_dev *gxp, struct scatterlist *sg,
				int nents, enum dma_data_direction direction);

/**
 * gxp_dma_map_dmabuf_attachment() - Create a mapping for a dma-buf
 * @gxp: The GXP device to map the dma-buf for
 * @gdomain: The IOMMU domain the dma-buf to be mapped on
 * @attachment: An attachment, representing the dma-buf, obtained from
 *              `dma_buf_attach()`
 * @direction: DMA direction
 *
 * Return: A scatter-gather table describing the mapping of the dma-buf
 *         into the default IOMMU domain. Returns ERR_PTR on failure.
 */
struct sg_table *
gxp_dma_map_dmabuf_attachment(struct gxp_dev *gxp,
			      struct gxp_iommu_domain *gdomain,
			      struct dma_buf_attachment *attachment,
			      enum dma_data_direction direction);

/**
 * gxp_dma_unmap_dmabuf_attachment() - Unmap a dma-buf
 * @gxp: The GXP device the dma-buf was mapped for
 * @gdomain: The IOMMU domain the buffer was mapped on
 * @attachment: The attachment, representing the dma-buf, that was passed to
 *              `gxp_dma_map_dmabuf_attachment()` to create the mapping
 * @sgt: The scatter-gather table returned by `gxp_dma_map_dmabuf_attachment()`
 *       when mapping this dma-buf
 * @direction: DMA direction
 */
void gxp_dma_unmap_dmabuf_attachment(struct gxp_dev *gxp,
				     struct gxp_iommu_domain *gdomain,
				     struct dma_buf_attachment *attachment,
				     struct sg_table *sgt,
				     enum dma_data_direction direction);
/**
 * gxp_iommu_get_domain_for_dev() - Get default domain
 * @gxp: The GXP device to get the default domain for
 *
 * Return: Domain embedding default IOMMU domain information.
 */
struct gxp_iommu_domain *gxp_iommu_get_domain_for_dev(struct gxp_dev *gxp);

/**
 * gxp_iommu_aux_get_pasid() - Get PASID corresponding to gdomain
 * @gxp: The GXP device attached to IOMMU
 * @gdomain: The IOMMU domain to get the PASID for
 *
 * Return: PASID of the passed domain
 */
uint gxp_iommu_aux_get_pasid(struct gxp_dev *gxp,
			     struct gxp_iommu_domain *gdomain);

/**
 * gxp_iommu_setup_shareability() - Set shareability to enable IO-Coherency.
 * @gxp: The GXP device to set shareability for
 */
void gxp_iommu_setup_shareability(struct gxp_dev *gxp);

#endif /* __GXP_DMA_H__ */
