// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2023 Google LLC.
 *
 * This platform component registers an ITMON notifier callback which filters
 * fabric fault reports where the GPU is identified as the initiator of the
 * transaction.
 *
 * When such a fault occurs, it searches for the faulting physical address in
 * the GPU page tables of all GPU contexts.  If the physical address appears in
 * a page table, the context and corresponding virtual address are logged.
 *
 * Otherwise, a message is logged indicating that the physical address does not
 * appear in any GPU page table.
 */

#if IS_ENABLED(CONFIG_EXYNOS_ITMON)

/* Linux includes */
#include <linux/of.h>

/* SOC includes */
#include <soc/google/exynos-itmon.h>

/* Mali core includes */
#include <mali_kbase.h>

/* Pixel integration includes */
#include "mali_kbase_config_platform.h"
#include "pixel_gpu_control.h"


/* GPU page tables may use more physical address bits than the bus, to encode
 * other information.  We'll need to mask those away to match with bus
 * addresses.
 */
#define PHYSICAL_ADDRESS_BITS 36
#define PHYSICAL_ADDRESS_MASK ((1ULL << (PHYSICAL_ADDRESS_BITS)) - 1)

/* Convert KBASE_MMU_PAGE_ENTRIES to number of bits */
#define KBASE_MMU_PAGE_ENTRIES_LOG2 const_ilog2(KBASE_MMU_PAGE_ENTRIES)


/**
 * pixel_gpu_itmon_search_pgd() - Search a page directory page.
 *
 * @mmu_mode:  The &struct kbase_mmu_mode PTE accessor functions.
 * @level:     The level of the page directory.
 * @pa_pgd:    The physical address of the page directory page.
 * @pa_search: The physical address to search for.
 * @va_prefix: The virtual address prefix above this level.
 *
 * Return: The virtual address mapped to the physical address, or zero.
 */
static u64 pixel_gpu_itmon_search_pgd(struct kbase_mmu_mode const *mmu_mode,
	int level, phys_addr_t pa_pgd, phys_addr_t pa_search, u64 va_prefix)
{
	u64 va_found = 0;
	int i;

	/* Map the page */
	const u64 *entry = kmap(pfn_to_page(PFN_DOWN(pa_pgd)));
	if (!entry)
		return 0;

	/* Shift the VA prefix left to make room for this new level */
	va_prefix <<= KBASE_MMU_PAGE_ENTRIES_LOG2;

	/* For each entry in the page directory */
	for (i = 0; i < KBASE_MMU_PAGE_ENTRIES; i++) {

		/* Is this a PTE, an ATE, or invalid? */
		if (mmu_mode->pte_is_valid(entry[i], level)) {

			/* PTE: Get the physical address of the next level PGD */
			phys_addr_t pa_next = mmu_mode->pte_to_phy_addr(entry[i])
				& PHYSICAL_ADDRESS_MASK;

			/* Recurse into it */
			if (pa_next) {
				va_found = pixel_gpu_itmon_search_pgd(mmu_mode, level + 1,
					pa_next, pa_search, va_prefix);
				if (va_found)
					break;
			}

		} else if (mmu_mode->ate_is_valid(entry[i], level)) {

			/* ATE: Get the page (or block) physical address */
			phys_addr_t pa_start = mmu_mode->pte_to_phy_addr(entry[i])
				& PHYSICAL_ADDRESS_MASK;

			if (pa_start) {
				/* Get the size of the block:
				 * this may be larger than a page, depending on level.
				 * A competent compiler will hoist this out of the loop.
				 */
				int remaining_levels = MIDGARD_MMU_BOTTOMLEVEL - level;
				size_t block_size = PAGE_SIZE <<
					(KBASE_MMU_PAGE_ENTRIES_LOG2 * remaining_levels);

				/* Test if the block contains the PA we are searching for */
				if ((pa_search >= pa_start) &&
				    (pa_search < (pa_start + block_size))) {

					/* Combine translated and non-translated address bits */
					va_found = (va_prefix * block_size) +
					           (pa_search % block_size);
					break;
				}
			}
		}

		/* Advance the virtual address prefix with each entry */
		va_prefix++;
	}

	kunmap(pfn_to_page(PFN_DOWN(pa_pgd)));

	return va_found;
}

/**
 * pixel_gpu_itmon_search_page_table() - Search a page table for a PA.
 *
 * @kbdev: The &struct kbase_device.
 * @table: The &struct kbase_mmu_table to search.
 * @pa:    The physical address to search for.
 *
 * Return: The virtual address mapped to the physical address, or zero.
 */
static u64 pixel_gpu_itmon_search_page_table(struct kbase_device *kbdev,
	struct kbase_mmu_table* table, phys_addr_t pa)
{
	u64 va;

	rt_mutex_lock(&table->mmu_lock);
	va = pixel_gpu_itmon_search_pgd(kbdev->mmu_mode, MIDGARD_MMU_TOPLEVEL,
		table->pgd, pa, 0);
	rt_mutex_unlock(&table->mmu_lock);

	return va;
}

/**
 * pixel_gpu_itmon_search_context() - Search the page tables of a context.
 *
 * @pc:   The &struct pixel_context.
 * @kctx: The &struct kbase_context to search.
 *
 * Return: True if the faulting physical address was found.
 */
static bool pixel_gpu_itmon_search_context(struct pixel_context *pc,
	struct kbase_context *kctx)
{
	u64 va = pixel_gpu_itmon_search_page_table(pc->kbdev, &kctx->mmu,
		pc->itmon.pa);

	/* If a mapping was found */
	if (va) {
		/* Get the task from the context */
		struct pid *pid_struct;
		struct task_struct *task;

		rcu_read_lock();
		pid_struct = find_get_pid(kctx->pid);
		task = pid_task(pid_struct, PIDTYPE_PID);

		/* And report it */
		dev_err(pc->kbdev->dev,
			"ITMON: Faulting physical address 0x%llX appears in page table of "
			"task %s (pid %u), mapped from virtual address 0x%llx (as %d)\n",
			pc->itmon.pa, task ? task->comm : "[null task]", kctx->pid, va,
			kctx->as_nr);

		put_pid(pid_struct);
		rcu_read_unlock();

		return true;
	}

	return false;
}

#if MALI_USE_CSF
/**
 * pixel_gpu_itmon_search_csffw() - Search the CSF MCU page table.
 *
 * @pc: The &struct pixel_context.
 *
 * Return: True if the faulting physical address was found.
 */
static bool pixel_gpu_itmon_search_csffw(struct pixel_context *pc)
{
	struct kbase_device *kbdev = pc->kbdev;

	u64 va = pixel_gpu_itmon_search_page_table(kbdev, &kbdev->csf.mcu_mmu,
		pc->itmon.pa);

	/* If a mapping was found */
	if (va) {
		dev_err(kbdev->dev,
			"ITMON: Faulting physical address 0x%llX appears in CSF MCU page "
			"table, mapped from virtual address 0x%llx (as 0)\n",
			pc->itmon.pa, va);
		return true;
	}

	return false;
}
#endif /* MALI_USE_CSF */

/**
 * pixel_gpu_itmon_worker() - ITMON fault worker.
 *
 * Required to be able to lock mutexes while searching page tables.
 *
 * @data: The &struct work_struct.
 */
static void pixel_gpu_itmon_worker(struct work_struct *data)
{
	/* Recover the pixel_context */
	struct pixel_context *pc = container_of(data, struct pixel_context,
		itmon.work);

	struct kbase_device *kbdev = pc->kbdev;
	struct kbase_context *kctx;
	bool found = false;

	/* Log that the work has started */
	dev_err(kbdev->dev,
		"ITMON: Searching for physical address 0x%llX across all GPU page "
		"tables...\n", pc->itmon.pa);

	/* Search the CSF MCU page table first */
#if MALI_USE_CSF
	found |= pixel_gpu_itmon_search_csffw(pc);
#endif

	mutex_lock(&kbdev->kctx_list_lock);

	/* Enumerate all contexts and search their page tables */
	list_for_each_entry(kctx, &kbdev->kctx_list, kctx_list_link) {
		found |= pixel_gpu_itmon_search_context(pc, kctx);
	}

	mutex_unlock(&kbdev->kctx_list_lock);

	/* For completeness, log that we did not find the fault address anywhere */
	if (!found) {
		dev_err(kbdev->dev,
			"ITMON: Faulting physical address 0x%llX NOT PRESENT in any GPU "
			"page table - GPU would not have initiated this access\n",
			pc->itmon.pa);
	}

	/* Let the ITMON ISR know that we're done and it can continue */
	atomic_dec(&pc->itmon.active);
}

/**
 * pixel_gpu_itmon_notifier() - Handle an ITMON fault report.
 *
 * @nb:      The &struct notifier_block inside &struct pixel_context.
 * @action:  Unused.
 * @nb_data: The ITMON report.
 *
 * Return: NOTIFY_OK to continue calling other notifier blocks.
 */
static int pixel_gpu_itmon_notifier(struct notifier_block *nb,
	unsigned long action, void *nb_data)
{
	/* Recover the pixel_context */
	struct pixel_context *pc = container_of(nb, struct pixel_context, itmon.nb);

	/* Get details of the ITMON report */
	struct itmon_notifier *itmon_info = nb_data;

	/* Filter out non-GPU ports */
	if ((!itmon_info->port) ||
	    (strncmp(itmon_info->port, "GPU", 3) &&
	     strncmp(itmon_info->port, "G3D", 3)))
		return NOTIFY_OK;

	/* Immediately acknowledge that this fault matched our filter */
	dev_err(pc->kbdev->dev,
		"Detected relevant ITMON fault report from %s to 0x%llX, "
		"enqueueing work...\n", itmon_info->port, (u64)itmon_info->target_addr);

	/* Make sure we have finished processing previous work */
	if (atomic_fetch_inc(&pc->itmon.active) != 0) {
		atomic_dec(&pc->itmon.active);
		dev_err(pc->kbdev->dev, "Previous work not yet finished, skipping\n");
		return NOTIFY_OK;
	}

	/* Save the PA to search for */
	pc->itmon.pa = itmon_info->target_addr;

	/* Access to GPU page tables is protected by a mutex, which we cannot lock
	 * here in an atomic context.  Queue work to another CPU to do the search.
	 */
	queue_work(pc->itmon.wq, &pc->itmon.work);

	/* (Try to) busy-wait for that work to complete, before we ramdump */
	{
		u64 start = ktime_get_ns();

		while (atomic_read(&pc->itmon.active) > 0) {

			if ((ktime_get_ns() - start) < (NSEC_PER_SEC / 2)) {
				udelay(10000);
			} else {
				dev_err(pc->kbdev->dev,
					"Timed out waiting for ITMON work, this is not an error\n");
				break;
			}
		}
	}

	return NOTIFY_OK;
}

/**
 * gpu_itmon_init() - Initialize ITMON notifier callback.
 *
 * @kbdev: The &struct kbase_device.
 *
 * Return: An error code, or 0 on success.
 */
int gpu_itmon_init(struct kbase_device *kbdev)
{
	struct pixel_context *pc = kbdev->platform_context;

	/* The additional diagnostic information offered by this callback is only
	 * useful if it can be collected as part of a ramdump.  Ramdumps are
	 * disabled in "user" builds, so query the build variant and skip
	 * initialization if that is the case.
	 */
	struct device_node *dpm = of_find_node_by_name(NULL, "dpm");
	const char *variant = NULL;
	if ((!dpm) || of_property_read_string(dpm, "variant", &variant) ||
	    (!strcmp(variant, "user")))
		return 0;

	/* Create a workqueue that can run on any CPU with high priority, so that
	 * it can run while we (try to) wait for it in the ITMON interrupt.
	 */
	pc->itmon.wq = alloc_workqueue("mali_itmon_wq", WQ_UNBOUND | WQ_HIGHPRI, 1);
	if (!pc->itmon.wq)
		return -ENOMEM;
	INIT_WORK(&pc->itmon.work, pixel_gpu_itmon_worker);

	/* Then register our ITMON notifier callback */
	pc->itmon.nb.notifier_call = pixel_gpu_itmon_notifier;
	itmon_notifier_chain_register(&pc->itmon.nb);

	return 0;
}

/**
 * gpu_itmon_term() - Terminate ITMON notifier callback.
 *
 * @kbdev: The &struct kbase_device.
 */
void gpu_itmon_term(struct kbase_device *kbdev)
{
	struct pixel_context *pc = kbdev->platform_context;

	if (pc->itmon.wq) {
		/* Unregister our ITMON notifier callback first */
		itmon_notifier_chain_unregister(&pc->itmon.nb);

		/* Then it's safe to destroy the workqueue */
		destroy_workqueue(pc->itmon.wq);
		pc->itmon.wq = NULL;
	}
}

/* Depend on ITMON driver */
MODULE_SOFTDEP("pre: itmon");

#endif /* IS_ENABLED(CONFIG_EXYNOS_ITMON) */