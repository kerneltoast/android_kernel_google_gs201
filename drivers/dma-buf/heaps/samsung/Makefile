# SPDX-License-Identifier: GPL-2.0
obj-$(CONFIG_DMABUF_SAMSUNG_HEAPS) += samsung_dma_heap.o
samsung_dma_heap-y += heap_dma_buf.o samsung_heap.o
samsung_dma_heap-$(CONFIG_DMABUF_HEAPS_SAMSUNG_CHUNK) += chunk_heap.o
samsung_dma_heap-$(CONFIG_DMABUF_HEAPS_SAMSUNG_CMA) += cma_heap.o
samsung_dma_heap-$(CONFIG_DMABUF_HEAPS_SAMSUNG_CARVEOUT) += carveout_heap.o
samsung_dma_heap-$(CONFIG_DMABUF_HEAPS_SAMSUNG_SYSTEM) += system_heap.o
samsung_dma_heap-$(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION) += secure_buffer.o
