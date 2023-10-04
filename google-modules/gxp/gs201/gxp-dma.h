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
#include <linux/types.h>
#if (IS_ENABLED(CONFIG_GXP_TEST) || IS_ENABLED(CONFIG_ANDROID)) && !IS_ENABLED(CONFIG_GXP_GEM5)
#include <soc/google/tpu-ext.h>
#endif

#include "gxp-internal.h"

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
 * gxp_dma_domain_attach_device() - Attach the page table of a virtual core to
 * the device and perform any necessary initialization.
 * @gxp: The GXP device to attach
 * @vd: The virtual device including the virtual core
 * @virt_core: The virtual core the page table belongs to
 * @core: The physical core is bound with the virtual core
 *
 * The caller must make sure @vd will not be released for the duration of the
 * call.
 */
int gxp_dma_domain_attach_device(struct gxp_dev *gxp,
				 struct gxp_virtual_device *vd, uint virt_core,
				 uint core);

/**
 * gxp_dma_domain_detach_device() - Detach the page table of a virtual core from
 * the device.
 * @gxp: The GXP device to detach
 * @vd: The virtual device including the virtual core
 * @virt_core: The virtual core the page table belongs to
 *
 * The client the @vd belongs to must hold a BLOCK wakelock for the iommu
 * detaching
 *
 * The caller must make sure @vd will not be released for the duration of the
 * call.
 */
void gxp_dma_domain_detach_device(struct gxp_dev *gxp,
				  struct gxp_virtual_device *vd, uint virt_core);

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
 * fixed IOVAs on certain virtual core
 * @gxp: The GXP device to set up the mappings for
 * @vd: The virtual device including the virtual core the IOVA are mapped for
 * @virt_core: The virtual core the IOVAs are mapped for
 * @core: The corresponding physical core of the @virt_core
 *
 * GXP firmware expects several buffers and registers to be mapped to fixed
 * locations in their IOVA space. This function initializes all those mappings
 * for the core.
 *
 * This function must not be called until after all the `vaddr` and `size`
 * fields of every `struct gxp_mapped_resource` inside of @gxp have been
 * initialized.
 *
 * The caller must make sure @vd will not be released for the duration of the
 * call.
 *
 * Return:
 * * 0    - Mappings created successfully
 * * -EIO - Failed to create one or more of the mappings
 */
int gxp_dma_map_core_resources(struct gxp_dev *gxp,
			       struct gxp_virtual_device *vd, uint virt_core,
			       uint core);

/**
 * gxp_dma_unmap_core_resources() - Unmap the IOVAs mapped by
 * gxp_dma_map_resources
 * @gxp: The GXP device that was passed to gxp_dma_map_core_resources()
 * @vd: The virtual device including the virtual core the IOVAs were mapped for
 * @virt_core: The virtual core the IOVAs were mapped for
 * @core: The physical cores the IOVAs were mapped for
 *
 * GXP firmware expects several buffers and registers to be mapped to fixed
 * locations in their IOVA space. This function releases all those mappings.
 *
 * The caller must make sure @vd will not be released for the duration of the
 * call.
 */
void gxp_dma_unmap_core_resources(struct gxp_dev *gxp,
				  struct gxp_virtual_device *vd, uint virt_core,
				  uint core);

#if (IS_ENABLED(CONFIG_GXP_TEST) || IS_ENABLED(CONFIG_ANDROID)) && !IS_ENABLED(CONFIG_GXP_GEM5)
/**
 * gxp_dma_map_tpu_buffer() - Map the tpu mbx queue buffers with fixed IOVAs
 * @gxp: The GXP device to set up the mappings for
 * @vd: The virtual device including the virtual cores the mapping is for
 * @virt_core_list: A bitfield enumerating the virtual cores the mapping is for
 * @core_list: A bitfield enumerating the physical cores the mapping is for
 * @mbx_info: Structure holding TPU-DSP mailbox queue buffer information
 *
 * Return:
 * * 0    - Mappings created successfully
 * * -EIO - Failed to create the mappings
 *
 * The caller must make sure @vd will not be released for the duration of the
 * call.
 */
int gxp_dma_map_tpu_buffer(struct gxp_dev *gxp, struct gxp_virtual_device *vd,
			   uint virt_core_list, uint core_list,
			   struct edgetpu_ext_mailbox_info *mbx_info);

/**
 * gxp_dma_unmap_tpu_buffer() - Unmap IOVAs mapped by gxp_dma_map_tpu_buffer()
 * @gxp: The GXP device that was passed to gxp_dma_map_tpu_buffer()
 * @vd: The virtual device including the virtual cores the mapping was for
 * @mbx_desc: Structure holding info for already mapped TPU-DSP mailboxes. The
 * list of virtual cores to unmap is in this descriptor.
 *
 * The caller must make sure @vd will not be released for the duration of the
 * call.
 */
void gxp_dma_unmap_tpu_buffer(struct gxp_dev *gxp,
			      struct gxp_virtual_device *vd,
			      struct gxp_tpu_mbx_desc mbx_desc);
#endif  // (CONFIG_GXP_TEST || CONFIG_ANDROID) && !CONFIG_GXP_GEM5

/**
 * gxp_dma_map_allocated_coherent_buffer() - Map a coherent buffer
 * @gxp: The GXP device to map the allocated buffer for
 * @vd: The virtual device including the virtual cores the mapping is for
 * @virt_core_list: A bitfield enumerating the virtual cores the mapping is for
 * @size: The size of the allocated buffer, in bytes
 * @dma_handle: The allocated device IOVA
 * @gxp_dma_flags: The type of mapping to create; currently unused
 *
 * Return: Kernel virtual address of the mapped buffer
 *
 * The caller must make sure @vd will not be released for the duration of the
 * call.
 */
int gxp_dma_map_allocated_coherent_buffer(struct gxp_dev *gxp, void *buf,
					  struct gxp_virtual_device *vd,
					  uint virt_core_list, size_t size,
					  dma_addr_t dma_handle,
					  uint gxp_dma_flags);
/**
 * gxp_dma_unmap_allocated_coherent_buffer() - Unmap a coherent buffer
 * @gxp: The GXP device the buffer was allocated and mapped for
 * @vd: The virtual device including the virtual cores the mapping was for
 * @virt_core_list: A bitfield enumerating the virtual cores the mapping was for
 * @size: The size of the buffer, in bytes
 * @dma_handle: The device IOVA
 *
 * The caller must make sure @vd will not be released for the duration of the
 * call.
 */
void gxp_dma_unmap_allocated_coherent_buffer(struct gxp_dev *gxp,
					     struct gxp_virtual_device *vd,
					     uint virt_core_list, size_t size,
					     dma_addr_t dma_handle);
/**
 * gxp_dma_alloc_coherent() - Allocate and map a coherent buffer for a GXP core
 * @gxp: The GXP device to map the allocated buffer for
 * @vd: The virtual device including the virtual cores the mapping is for
 * @virt_core_list: A bitfield enumerating the virtual cores the mapping is for
 * @size: The size of the buffer to be allocated, in bytes
 * @dma_handle: Reference to a variable to be set to the allocated IOVA
 * @flag: The type of memory to allocate (see kmalloc)
 * @gxp_dma_flags: The type of mapping to create; Currently unused
 *
 * Return: Kernel virtual address of the allocated/mapped buffer
 *
 * If the passed @vd is a null pointer, this function will only allocate a
 * buffer but not map it to any particular core.
 *
 * The caller must make sure @vd will not be released for the duration of the
 * call.
 */
void *gxp_dma_alloc_coherent(struct gxp_dev *gxp, struct gxp_virtual_device *vd,
			     uint virt_core_list, size_t size,
			     dma_addr_t *dma_handle, gfp_t flag,
			     uint gxp_dma_flags);
/**
 * gxp_dma_free_coherent() - Unmap and free a coherent buffer
 * @gxp: The GXP device the buffer was allocated and mapped for
 * @vd: The virtual device including the virtual cores the mapping was for
 * @virt_core_list: A bitfield enumerating the virtual cores the mapping was for
 * @size: The size of the buffer, in bytes, passed to `gxp_dma_alloc_coherent()`
 * @cpu_addr: The kernel virtual address returned by `gxp_dma_alloc_coherent()`
 * @dma_handle: The device IOVA, set by `gxp_dma_alloc_coherent()`
 *
 * If the buffer is mapped via `gxp_dma_map_allocated_coherent_buffer`, the
 * caller must call `gxp_dma_unmap_allocated_coherent_buffer` to unmap before
 * freeing the buffer.
 *
 * If the passed @vd is a null pointer, this function will only free the buffer
 * but not do any unmapping.
 *
 * The caller must make sure @vd will not be released for the duration of the
 * call.
 */
void gxp_dma_free_coherent(struct gxp_dev *gxp, struct gxp_virtual_device *vd,
			   uint virt_core_list, size_t size, void *cpu_addr,
			   dma_addr_t dma_handle);

/**
 * gxp_dma_map_single() - Create a mapping for a kernel buffer
 * @gxp: The GXP device to map the buffer for
 * @vd: The virtual device including the virtual cores the mapping is for
 * @virt_core_list: A bitfield enumerating the virtual cores the mapping is for
 * @cpu_addr: The kernel virtual address of the buffer to map
 * @size: The size of the buffer to map, in bytes
 * @direction: DMA direction
 * @attrs: The same set of flags used by the base DMA API
 * @gxp_dma_flags: The type of mapping to create; Currently unused
 *
 * Return: The IOVA the buffer was mapped to
 *
 * The caller must make sure @vd will not be released for the duration of the
 * call.
 */
dma_addr_t gxp_dma_map_single(struct gxp_dev *gxp,
			      struct gxp_virtual_device *vd,
			      uint virt_core_list, void *cpu_addr, size_t size,
			      enum dma_data_direction direction,
			      unsigned long attrs, uint gxp_dma_flags);
/**
 * gxp_dma_unmap_single() - Unmap a kernel buffer
 * @gxp: The GXP device the buffer was mapped for
 * @vd: The virtual device including the virtual cores the mapping was for
 * @virt_core_list: A bitfield enumerating the virtual cores the mapping was for
 * @dma_addr: The device IOVA, returned by `gxp_dma_map_single()`
 * @size: The size of the mapping, which was passed to `gxp_dma_map_single()`
 * @direction: DMA direction; same as passed to `gxp_dma_map_single()`
 * @attrs: The same set of flags used by the base DMA API
 *
 * The caller must make sure @vd will not be released for the duration of the
 * call.
 */
void gxp_dma_unmap_single(struct gxp_dev *gxp, struct gxp_virtual_device *vd,
			  uint virt_core_list, dma_addr_t dma_addr, size_t size,
			  enum dma_data_direction direction,
			  unsigned long attrs);

/**
 * gxp_dma_map_page() - Create a mapping for a physical page of memory
 * @gxp: The GXP device to map the page for
 * @vd: The virtual device including the virtual cores the mapping is for
 * @virt_core_list: A bitfield enumerating the virtual cores the mapping is for
 * @page: The `struct page` of the physical page to create a mapping for
 * @offset: The offset into @page to begin the mapping at
 * @size: The number of bytes in @page to map
 * @direction: DMA direction
 * @attrs: The same set of flags used by the base DMA API
 * @gxp_dma_flags: The type of mapping to create; Currently unused
 *
 * Return: The IOVA the page was mapped to
 *
 * The caller must make sure @vd will not be released for the duration of the
 * call.
 */
dma_addr_t gxp_dma_map_page(struct gxp_dev *gxp, struct gxp_virtual_device *vd,
			    uint virt_core_list, struct page *page,
			    unsigned long offset, size_t size,
			    enum dma_data_direction direction,
			    unsigned long attrs, uint gxp_dma_flags);
/**
 * gxp_dma_unmap_page() - Unmap a physical page of memory
 * @gxp: The GXP device the page was mapped for
 * @vd: The virtual device including the virtual cores the mapping was for
 * @virt_core_list: A bitfield enumerating the virtual cores the mapping was for
 * @dma_addr: The device IOVA, returned by `gxp_dma_map_page()`
 * @size: The size of the mapping, which was passed to `gxp_dma_map_page()`
 * @direction: DMA direction; Same as passed to `gxp_dma_map_page()`
 * @attrs: The same set of flags used by the base DMA API
 *
 * The caller must make sure @vd will not be released for the duration of the
 * call.
 */
void gxp_dma_unmap_page(struct gxp_dev *gxp, struct gxp_virtual_device *vd,
			uint virt_core_list, dma_addr_t dma_addr, size_t size,
			enum dma_data_direction direction, unsigned long attrs);

/**
 * gxp_dma_map_resource() - Create a mapping for an MMIO resource
 * @gxp: The GXP device to map the resource for
 * @vd: The virtual device including the virtual cores the mapping is for
 * @virt_core_list: A bitfield enumerating the virtual cores the mapping is for
 * @phys_addr: The physical address of the MMIO resource to map
 * @size: The size of the MMIO region to map, in bytes
 * @direction: DMA direction
 * @attrs: The same set of flags used by the base DMA API
 * @gxp_dma_flags: The type of mapping to create; Currently unused
 *
 * Return: The IOVA the MMIO resource was mapped to
 *
 * The caller must make sure @vd will not be released for the duration of the
 * call.
 */
dma_addr_t gxp_dma_map_resource(struct gxp_dev *gxp,
				struct gxp_virtual_device *vd,
				uint virt_core_list, phys_addr_t phys_addr,
				size_t size, enum dma_data_direction direction,
				unsigned long attrs, uint gxp_dma_flags);
/**
 * gxp_dma_unmap_resource() - Unmap an MMIO resource
 * @gxp: The GXP device the MMIO resource was mapped for
 * @vd: The virtual device including the virtual cores the mapping was for
 * @virt_core_list: A bitfield enumerating the virtual cores the mapping was for
 * @dma_addr: The device IOVA, returned by `gxp_dma_map_resource()`
 * @size: The size of the mapping, which was passed to `gxp_dma_map_resource()`
 * @direction: DMA direction; Same as passed to `gxp_dma_map_resource()`
 * @attrs: The same set of flags used by the base DMA API
 *
 * The caller must make sure @vd will not be released for the duration of the
 * call.
 */
void gxp_dma_unmap_resource(struct gxp_dev *gxp, struct gxp_virtual_device *vd,
			    uint virt_core_list, dma_addr_t dma_addr,
			    size_t size, enum dma_data_direction direction,
			    unsigned long attrs);

/**
 * gxp_dma_map_sg() - Create a mapping for a scatter-gather list
 * @gxp: The GXP device to map the scatter-gather list for
 * @vd: The virtual device including the virtual cores the mapping is for
 * @virt_core_list: A bitfield enumerating the virtual cores the mapping is for
 * @sg: The scatter-gather list of the buffer to be mapped
 * @nents: The number of entries in @sg
 * @direction: DMA direction
 * @attrs: The same set of flags used by the base DMA API
 * @gxp_dma_flags: The type of mapping to create; Currently unused
 *
 * Return: The number of scatter-gather entries mapped to
 *
 * The caller must make sure @vd will not be released for the duration of the
 * call.
 */
int gxp_dma_map_sg(struct gxp_dev *gxp, struct gxp_virtual_device *vd,
		   int virt_core_list, struct scatterlist *sg, int nents,
		   enum dma_data_direction direction, unsigned long attrs,
		   uint gxp_dma_flags);
/**
 * gxp_dma_unmap_sg() - Unmap a scatter-gather list
 * @gxp: The GXP device the scatter-gather list was mapped for
 * @vd: The virtual device including the virtual cores the mapping was for
 * @virt_core_list: A bitfield enumerating the virtual cores the mapping was for
 * @sg: The scatter-gather list to unmap; The same one passed to
 *      `gxp_dma_map_sg()`
 * @nents: The number of entries in @sg; Same value passed to `gxp_dma_map_sg()`
 * @direction: DMA direction; Same as passed to `gxp_dma_map_sg()`
 * @attrs: The same set of flags used by the base DMA API
 *
 * The caller must make sure @vd will not be released for the duration of the
 * call.
 */
void gxp_dma_unmap_sg(struct gxp_dev *gxp, struct gxp_virtual_device *vd,
		      uint virt_core_list, struct scatterlist *sg, int nents,
		      enum dma_data_direction direction, unsigned long attrs);

/**
 * gxp_dma_sync_single_for_cpu() - Sync buffer for reading by the CPU
 * @gxp: The GXP device the mapping was created for
 * @dma_handle: The device IOVA, obtained from one of the `gxp_dma_map_*` APIs
 * @size: The size of the mapped region to sync
 * @direction: DMA direction
 */
void gxp_dma_sync_single_for_cpu(struct gxp_dev *gxp, dma_addr_t dma_handle,
				 size_t size,
				 enum dma_data_direction direction);
/**
 * gxp_dma_sync_single_for_device() - Sync buffer for reading by the device
 * @gxp: The GXP device the mapping was created for
 * @dma_handle: The device IOVA, obtained from one of the `gxp_dma_map_*` APIs
 * @size: The size of the mapped region to sync
 * @direction: DMA direction
 */
void gxp_dma_sync_single_for_device(struct gxp_dev *gxp, dma_addr_t dma_handle,
				    size_t size,
				    enum dma_data_direction direction);

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
 * @vd: The virtual device including the virtual cores the dma-buf is for
 * @virt_core_list: A bitfield enumerating the virtual cores the dma-buf is for
 * @attachment: An attachment, representing the dma-buf, obtained from
 *              `dma_buf_attach()`
 * @direction: DMA direction
 *
 * Return: A scatter-gather table describing the mapping of the dma-buf
 *         into the default IOMMU domain. Returns ERR_PTR on failure.
 *
 * The caller must make sure @vd will not be released for the duration of the
 * call.
 */
struct sg_table *gxp_dma_map_dmabuf_attachment(
	struct gxp_dev *gxp, struct gxp_virtual_device *vd, uint virt_core_list,
	struct dma_buf_attachment *attachment,
	enum dma_data_direction direction);

/**
 * gxp_dma_unmap_dmabuf_attachment() - Unmap a dma-buf
 * @gxp: The GXP device the dma-buf was mapped for
 * @vd: The virtual device including the virtual cores the dma-buf is for
 * @virt_core_list: A bitfield enumerating the virtual cores the dma-buf was for
 * @attachment: The attachment, representing the dma-buf, that was passed to
 *              `gxp_dma_map_dmabuf_attachment()` to create the mapping
 * @sgt: The scatter-gather table returned by `gxp_dma_map_dmabuf_attachment()`
 *       when mapping this dma-buf
 * @direction: DMA direction
 *
 * The caller must make sure @vd will not be released for the duration of the
 * call.
 */
void gxp_dma_unmap_dmabuf_attachment(struct gxp_dev *gxp,
				     struct gxp_virtual_device *vd,
				     uint virt_core_list,
				     struct dma_buf_attachment *attachment,
				     struct sg_table *sgt,
				     enum dma_data_direction direction);

#endif /* __GXP_DMA_H__ */
