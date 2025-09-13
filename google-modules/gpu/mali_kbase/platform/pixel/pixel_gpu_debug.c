// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2022 Google LLC.
 *
 * Author: Jack Diver <diverj@google.com>
 */

/* Mali core includes */
#include <mali_kbase.h>
#include <device/mali_kbase_device.h>

/* Pixel integration includes */
#include "pixel_gpu_debug.h"

#define PIXEL_STACK_PDC_ADDR     0x000770DB
#define PIXEL_CG_PDC_ADDR        0x000760DB
#define PIXEL_SC_PDC_ADDR        0x000740DB
#define GPU_PDC_ADDR(offset, val)    ((offset) + ((val) << 8))
#define GPU_DBG_ACTIVE_BIT         (1 << 31)
#define GPU_DBG_ACTIVE_MAX_LOOPS    1000000
#define GPU_DBG_INVALID                (~0U)

static bool gpu_debug_check_dbg_active(struct kbase_device *kbdev)
{
	int i = 0;
	u32 val;

	lockdep_assert_held(&kbdev->hwaccess_lock);

	/* Wait for the active bit to drop, indicating the DBG command completed */
	do {
		val = kbase_reg_read32(kbdev, GPU_CONTROL_ENUM(GPU_STATUS));
	} while ((val & GPU_DBG_ACTIVE_BIT) && i++ < GPU_DBG_ACTIVE_MAX_LOOPS);

	if (val & GPU_DBG_ACTIVE_BIT) {
		dev_err(kbdev->dev, "Timed out waiting for GPU DBG command to complete");
		return false;
	}

	dev_dbg(kbdev->dev, "Waited for %d iterations before GPU DBG command completed", i);

	return true;
}

static u32 gpu_debug_read_pdc(struct kbase_device *kbdev, u32 pdc_offset)
{
	lockdep_assert_held(&kbdev->hwaccess_lock);

	/* Write the debug command */
	kbase_reg_write32(kbdev, GPU_CONTROL_ENUM(GPU_COMMAND), pdc_offset);
	/* Wait for the debug command to complete */
	if (!gpu_debug_check_dbg_active(kbdev))
		return GPU_DBG_INVALID;

	/* Read the result */
	return kbase_reg_read32(kbdev, GPU_CONTROL_ENUM(GPU_DBG));
}

static void gpu_debug_read_sparse_pdcs(struct kbase_device *kbdev, u32 *out, u64 available,
				       u64 offset, u64 logical_max)
{
	int sparse_idx, logical_idx = 0;

	for (sparse_idx = 0; sparse_idx < BITS_PER_TYPE(u64) && logical_idx < logical_max; ++sparse_idx) {
		/* Skip if we don't have this core in our configuration */
		if (!(available & BIT_ULL(sparse_idx)))
			continue;

		/* GPU debug command expects the sparse core index */
		out[logical_idx] = gpu_debug_read_pdc(kbdev, GPU_PDC_ADDR(offset, sparse_idx));

		++logical_idx;
	}
}

void gpu_debug_read_pdc_status(struct kbase_device *kbdev, struct pixel_gpu_pdc_status *status)
{
	struct kbase_gpu_props *raw_props;
	lockdep_assert_held(&kbdev->hwaccess_lock);

	status->meta = (struct pixel_gpu_pdc_status_metadata) {
		.magic = "pdcs",
		.version = 2,
	};

	/* If there's no external power we skip the register read/writes,
	 * We know all the PDC signals will be 0 in this case
	 */
	if (!kbase_io_is_gpu_powered(kbdev)) {
		memset(&status->state, 0, sizeof(status->state));
		return;
	}

	raw_props = &kbdev->gpu_props;

	status->state.core_group = gpu_debug_read_pdc(kbdev, PIXEL_CG_PDC_ADDR);
	gpu_debug_read_sparse_pdcs(kbdev, status->state.shader_cores, raw_props->shader_present,
				   PIXEL_SC_PDC_ADDR, PIXEL_MALI_SC_COUNT);
	gpu_debug_read_sparse_pdcs(kbdev, status->state.stacks, raw_props->stack_present,
				   PIXEL_STACK_PDC_ADDR, PIXEL_MALI_STACK_COUNT);
}

void gpu_debug_dump_pdc_status(struct kbase_device *kbdev)
{
	struct pixel_gpu_pdc_status status;
	int i;

	lockdep_assert_held(&kbdev->hwaccess_lock);

	if (!kbase_io_is_gpu_powered(kbdev)) {
		dev_err(kbdev->dev, "pixel_gpu_debug: GPU not powered to read PDC state.");
		return;
	}

	gpu_debug_read_pdc_status(kbdev, &status);

	dev_err(kbdev->dev, "pixel_gpu_debug: pdc coregroup state: 0x%x", status.state.core_group);
	for (i = 0; i < PIXEL_MALI_SC_COUNT; i++) {
		dev_err(kbdev->dev, "pixel_gpu_debug: pdc shadercore state: %d=0x%x",
			i, status.state.shader_cores[i]);
	}
	for (i = 0; i < PIXEL_MALI_STACK_COUNT; i++) {
		dev_err(kbdev->dev, "pixel_gpu_debug: pdc shaderstack state: %d=0x%x",
			i, status.state.stacks[i]);
	}
}
