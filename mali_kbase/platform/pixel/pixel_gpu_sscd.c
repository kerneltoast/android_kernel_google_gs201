// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2021 Google LLC.
 *
 * Author: Jack Diver <diverj@google.com>
 */

/* Mali core includes */
#include <mali_kbase.h>
#include <csf/mali_kbase_csf_trace_buffer.h>
#include <csf/mali_kbase_csf_firmware.h>
#include <csf/mali_kbase_csf_firmware_cfg.h>
#include <csf/mali_kbase_csf_firmware_core_dump.h>

/* Pixel integration includes */
#include "mali_kbase_config_platform.h"
#include <mali_kbase_reset_gpu.h>
#include "pixel_gpu_sscd.h"
#include "pixel_gpu_debug.h"
#include "pixel_gpu_control.h"
#include <linux/platform_data/sscoredump.h>
#include <linux/platform_device.h>

/* Mali SSCD collection is only enabled for CSF GPUs*/

static void sscd_release(struct device *dev)
{
	(void)dev;
}

static struct sscd_platform_data sscd_pdata;
const static struct platform_device sscd_dev_init = { .name = "mali",
						      .driver_override = SSCD_NAME,
						      .id = -1,
						      .dev = {
							      .platform_data = &sscd_pdata,
							      .release = sscd_release,
						      } };
static struct platform_device sscd_dev;

/* There are more segments here than what we actually collect, we leave them here
 * to have backward compatibility with the parser.
 */
enum
{
	MCU_REGISTERS = 0x1,
	GPU_REGISTERS = 0x2,
	PRIVATE_MEM = 0x3,
	SHARED_MEM = 0x4,
	FW_TRACE = 0x5,
	PM_EVENT_LOG = 0x6,
	POWER_RAIL_LOG = 0x7,
	PDC_STATUS = 0x8,
	KTRACE = 0x9,
	CONTEXTS = 0xA,			/* "c@tx", contains active csg slot info */
	FW_CORE_DUMP = 0xB,
	KCTX_INFO = 0xC,		/* "kc7x", contains kctx information */
	NUM_SEGMENTS
} sscd_segs;

static void get_pm_event_log(struct kbase_device *kbdev, struct sscd_segment *seg)
{
	lockdep_assert_held(&kbdev->hwaccess_lock);

	if (seg->addr == NULL)
		return;

	if (kbase_pm_copy_event_log(kbdev, seg->addr, seg->size)) {
		dev_warn(kbdev->dev, "pixel: failed to report PM event log");
	}
}

/**
 * struct pixel_fw_trace_metadata - Info about the FW trace log
 *
 * @magic:          Always 'pfwt', helps find the log in memory dumps
 * @trace_address:  The memory address of the FW trace log
 * @trace_length:   Number of used bytes in the trace ring buffer.
 *                  The length will be <= (FW_TRACE_BUF_NR_PAGES << PAGE_SHIFT)
 * @version:        Updated whenever the binary layout changes
 * @_reserved:      Bytes reserved for future use
 **/
struct pixel_fw_trace_metadata {
	char magic[4];
	uint64_t trace_address;
	uint32_t trace_length;
	uint8_t version;
	char _reserved[31];
} __attribute__((packed));
_Static_assert(sizeof(struct pixel_fw_trace_metadata) == 48,
	       "Incorrect pixel_fw_trace_metadata size");

/**
 * struct pixel_fw_trace - The FW trace and associated meta data
 *
 * @meta:      Info about the trace log
 * @trace_log: The actual trace log
 **/
struct pixel_fw_trace {
	struct pixel_fw_trace_metadata meta;
	char trace_log[FW_TRACE_BUF_NR_PAGES << PAGE_SHIFT];
};

static void get_fw_trace(struct kbase_device *kbdev, struct sscd_segment *seg)
{
	struct firmware_trace_buffer *tb;
	struct pixel_fw_trace *fw_trace;

	lockdep_assert_held(&kbdev->hwaccess_lock);

	if (seg->addr == NULL)
		return;

	fw_trace = seg->addr;

	/* Write the default meta data */
	fw_trace->meta = (struct pixel_fw_trace_metadata) {
		.magic = "pfwt",
		.trace_address = 0,
		.trace_length = 0,
		.version = 1,
	};

	tb = kbase_csf_firmware_get_trace_buffer(kbdev, KBASE_CSFFW_LOG_BUF_NAME);

	if (tb == NULL) {
		dev_err(kbdev->dev, "pixel: failed to open firmware trace buffer");
		return;
	}

	/* Write the trace log */
	fw_trace->meta.trace_address = (uint64_t)tb;
	fw_trace->meta.trace_length = kbase_csf_firmware_trace_buffer_read_data(
		tb, fw_trace->trace_log, sizeof(fw_trace->trace_log));

	return;
}

/**
 * struct pixel_ktrace_metadata - Info about the ktrace log
 *
 * @magic:          Always 'ktra', helps find the log in memory dumps
 * @trace_address:  The memory address of the ktrace log
 * @trace_start:    Start of the ktrace ringbuffer
 * @trace_end:      End of the ktrace ringbuffer
 * @version_major:  Ktrace major version.
 * @version_minor:  Ktrace minor version.
 * @_reserved:      Bytes reserved for future use
 **/
struct pixel_ktrace_metadata {
	char magic[4];
	uint64_t trace_address;
	uint32_t trace_start;
	uint32_t trace_end;
	uint8_t version_major;
	uint8_t version_minor;
	char _reserved[28];
} __attribute__((packed));
_Static_assert(sizeof(struct pixel_ktrace_metadata) == 50,
	       "Incorrect pixel_ktrace_metadata size");

struct pixel_ktrace {
	struct pixel_ktrace_metadata meta;
#if KBASE_KTRACE_TARGET_RBUF
	struct kbase_ktrace_msg trace_log[KBASE_KTRACE_SIZE];
#endif
};
static void get_ktrace(struct kbase_device *kbdev,
			  struct sscd_segment *seg)
{
	struct pixel_ktrace *ktrace = seg->addr;
#if KBASE_KTRACE_TARGET_RBUF
	unsigned long flags;
	u32 entries_copied = 0;
#endif

	if (seg->addr == NULL)
		return;

	ktrace->meta = (struct pixel_ktrace_metadata) { .magic = "ktra" };
#if KBASE_KTRACE_TARGET_RBUF
	lockdep_assert_held(&kbdev->hwaccess_lock);
	spin_lock_irqsave(&kbdev->ktrace.lock, flags);
	ktrace->meta.trace_address = (uint64_t)kbdev->ktrace.rbuf;
	ktrace->meta.trace_start = kbdev->ktrace.first_out;
	ktrace->meta.trace_end = kbdev->ktrace.next_in;
	ktrace->meta.version_major = KBASE_KTRACE_VERSION_MAJOR;
	ktrace->meta.version_minor = KBASE_KTRACE_VERSION_MINOR;

	entries_copied = kbasep_ktrace_copy(kbdev, seg->addr, KBASE_KTRACE_SIZE);
	if (entries_copied != KBASE_KTRACE_SIZE)
		dev_warn(kbdev->dev, "only copied %i of %i ktrace entries",
			entries_copied, KBASE_KTRACE_SIZE);
	spin_unlock_irqrestore(&kbdev->ktrace.lock, flags);

	KBASE_KTRACE_RBUF_DUMP(kbdev);
#else
	dev_warn(kbdev->dev, "ktrace information not present");
#endif
}

/**
 * enum pixel_context_state - a coarse platform independent state for a context.
 *
 * @PIXEL_CONTEXT_ACTIVE:   The context is running (in some capacity) on GPU.
 * @PIXEL_CONTEXT_RUNNABLE: The context is runnable, but not running on GPU.
 * @PIXEL_CONTEXT_INACTIVE: The context is not acive.
 */
enum pixel_context_state {
	PIXEL_CONTEXT_ACTIVE = 0,
	PIXEL_CONTEXT_RUNNABLE,
	PIXEL_CONTEXT_INACTIVE
};

/**
 * struct pixel_context_metadata - metadata for context information.
 *
 * @magic: always "c@tx"
 * @version: version marker.
 * @platform: unique id for platform reporting context.
 * @_reserved: reserved.
 */
struct pixel_context_metadata {
	char magic[4];
	u8 version;
	u32 platform;
	char _reserved[27];
} __attribute__((packed));
_Static_assert(sizeof(struct pixel_context_metadata) == 36,
               "Incorrect pixel_context_metadata size");

/**
 * struct pixel_context_snapshot_entry - platform independent context record for
 *                                       crash reports.
 * @id:             The context id.
 * @pid:            The PID that owns this context.
 * @tgid:           The TGID that owns this context.
 * @context_state:  The coarse state for a context.
 * @priority:       The priority of this context.
 * @gpu_slot:       The handle that the context may have representing the
 *                  resource granted to run on the GPU.
 * @platform_state: The platform-dependendant state, if any.
 * @time_in_state:  The amount of time in ms that this context has been
 * 		    in @platform_state.
 */
struct pixel_context_snapshot_entry {
	u32 id;
	u32 pid;
	u32 tgid;
	u8 context_state;
	u32 priority;
	u32 gpu_slot;
	u32 platform_state;
	u64 time_in_state;
} __attribute__((packed));
_Static_assert(sizeof(struct pixel_context_snapshot_entry) == 33,
               "Incorrect pixel_context_metadata size");

/**
 * struct pixel_context_snapshot - list of platform independent context info.
 *
 * List of contexts of interest during SSCD generation time.
 *
 * @meta:         The metadata for the segment.
 * @num_contexts: The number of contexts in the list.
 * @contexts:     The context information.
 */
struct pixel_context_snapshot {
	struct pixel_context_metadata meta;
	u32 num_contexts;
	struct pixel_context_snapshot_entry contexts[];
} __attribute__((packed));

/* get_contexts - fill the CONTEXT segment
 *
 * @kbdev: kbase_device
 * @segment: the CONTEXT segment for report
 *
 * \returns: 0 on success.
 */
static int get_contexts(struct kbase_device *kbdev,
		 struct sscd_segment *segment)
{
	u32 csg_nr;
	u32 num_csg = kbdev->csf.global_iface.group_num;
	struct pixel_context_snapshot *context_snapshot;
	struct kbase_csf_scheduler *const scheduler = &kbdev->csf.scheduler;
	size_t num_entries;
	size_t entry_idx;

	if (segment->addr == NULL)
		return -ENOMEM;

	if (!rt_mutex_trylock(&kbdev->csf.scheduler.lock)) {
		dev_warn(kbdev->dev, "could not lock scheduler during dump.");
		return -EBUSY;
	}

	num_entries = bitmap_weight(scheduler->csg_inuse_bitmap, num_csg);

	context_snapshot = segment->addr;
	context_snapshot->num_contexts = num_entries;

	context_snapshot->meta = (struct pixel_context_metadata) {
		.magic = "c@tx",
		.platform = kbdev->gpu_props.gpu_id.product_id,
		.version = 1,
	};

	entry_idx = 0;
	for_each_set_bit(csg_nr, scheduler->csg_inuse_bitmap, num_csg) {
		struct kbase_csf_csg_slot *slot =
			&kbdev->csf.scheduler.csg_slots[csg_nr];
		struct pixel_context_snapshot_entry *entry =
			&context_snapshot->contexts[entry_idx++];
		entry->context_state = PIXEL_CONTEXT_ACTIVE;
		entry->gpu_slot = csg_nr;
		entry->platform_state = atomic_read(&slot->state);
		entry->priority = slot->priority;
		/* b/351116409 - TODO:gvamsi there is no trigger_jiffies in R50P0
		entry->time_in_state = (jiffies - slot->trigger_jiffies) / HZ;
		 */
		if (slot->resident_group) {
			entry->id = slot->resident_group->handle;
			entry->pid = slot->resident_group->kctx->pid;
			entry->tgid = slot->resident_group->kctx->tgid;
		}
	}

	rt_mutex_unlock(&kbdev->csf.scheduler.lock);
	return 0;
}

/**
 * struct pixel_kctx_info_metadata - metadata for kctx information.
 *
 * @magic: always "kc7x"
 * @version: version marker.
 * @platform: unique id for platform reporting context.
 * @_reserved: reserved.
 */
struct pixel_kctx_info_metadata {
	char magic[4];
	u8 version;
	u32 platform;
	char _reserved[27];
} __attribute__((packed));
_Static_assert(sizeof(struct pixel_kctx_info_metadata) == 36,
               "Incorrect pixel_kctx_info_metadata size");

struct pixel_kctx_info_entry {
	u32 id;
	u32 pid;
	u32 tgid;
	u64 fault_time_ns;
	u64 protm_enter_time_ns;
	u64 protm_exit_time_ns;
} __attribute__((packed));
_Static_assert(sizeof(struct pixel_kctx_info_entry) == 36,
               "Incorrect pixel_kctx_info_metadata size");

struct pixel_kctx_info {
	struct pixel_kctx_info_metadata meta;
	u32 num_entries;
	struct pixel_kctx_info_entry entries[];
} __attribute__((packed));

/* get_kctx_info - fill the KCTX_INFO segment
 *
 * @kbdev: kbase_device
 * @segment: the KCTX_INFO segment for report
 *
 * \returns: 0 on success.
 */
static int get_kctx_info(struct kbase_device *kbdev,
			struct sscd_segment *segment)
{
	struct pixel_kctx_info *kctx_info;
	struct kbase_context *kctx;
	size_t num_entries, idx;
	int err = 0;

	if (!segment) {
		err = -EINVAL;
		goto out;
	}

	if (kbase_reset_gpu_is_active(kbdev)) {
		dev_warn(kbdev->dev, "pixel_gpu_sscd: skipping kctx dump, reset is active.");
		err = -EINVAL;
		goto out;
	}

	if (!mutex_trylock(&kbdev->kctx_list_lock)) {
		dev_warn(kbdev->dev, "pixel_gpu_sscd: skipping kctx dump, kctx_list_lock busy.");
		err = -EBUSY;
		goto out;
	}

	num_entries = min_t(size_t, list_count_nodes(&kbdev->kctx_list), 16);
	segment->size = sizeof(struct pixel_kctx_info) +
		num_entries * sizeof(struct pixel_kctx_info_entry);
	segment->addr = vzalloc(segment->size);
	if (!segment->addr) {
		err = -ENOMEM;
		goto out_unlock;
	}

	kctx_info = (struct pixel_kctx_info *) segment->addr;
	kctx_info->meta = (struct pixel_kctx_info_metadata) {
		.magic = "kc7x",
		.platform = kbdev->gpu_props.gpu_id.product_id,
		.version = 1,
	};
	kctx_info->num_entries = num_entries;

	idx = 0;
	list_for_each_entry(kctx, &kbdev->kctx_list, kctx_list_link) {
		struct pixel_kctx_info_entry *entry = &kctx_info->entries[idx];

		entry->id = kctx->id;
		entry->pid = kctx->pid;
		entry->tgid = kctx->tgid;
		entry->fault_time_ns = ktime_to_ns(kbdev->csf.glb_fatal_ts);
		entry->protm_enter_time_ns = ktime_to_ns(kctx->protm_enter_ts);
		entry->protm_exit_time_ns = ktime_to_ns(kctx->protm_exit_ts);

		dev_info(kbdev->dev,
			"pixel_gpu_sscd: kctx=%d_%d_%d fault=%lld pmode_enter=%lld pmode_exit=%lld comm=%s",
			entry->tgid, entry->pid, entry->id,
			entry->fault_time_ns, entry->protm_enter_time_ns, entry->protm_exit_time_ns,
			kctx->comm);

		if (++idx >= num_entries)
			break;
	}

out_unlock:
	mutex_unlock(&kbdev->kctx_list_lock);
out:
	return err;
}

struct pixel_fw_core_dump {
	char magic[4];
	u32 reserved;
	char git_sha[BUILD_INFO_GIT_SHA_LEN];
	char core_dump[];
};

static void get_and_init_fw_core_dump(struct kbase_device *kbdev, struct sscd_segment *seg)
{
	const size_t core_dump_size = get_fw_core_dump_size(kbdev);

	int i;
	struct pixel_fw_core_dump *fw_core_dump;
	struct kbase_csf_firmware_interface *interface;
	struct page *page;
	u32 *p;
	size_t size;
	size_t write_size;

	if (core_dump_size == -1)
	{
		dev_err(kbdev->dev, "pixel: failed to get firmware core dump size");
	}

	seg->size = sizeof(struct pixel_fw_core_dump) + core_dump_size;
	seg->addr = vzalloc(seg->size);

	if (seg->addr == NULL) {
		seg->size = 0;
		return;
	}

	fw_core_dump = (struct pixel_fw_core_dump *) seg->addr;

	strncpy(fw_core_dump->magic, "fwcd", 4);
	memcpy(fw_core_dump->git_sha, fw_git_sha, BUILD_INFO_GIT_SHA_LEN);

	// Dumping ELF header
	{
		struct fw_core_dump_data private = {.kbdev = kbdev};
		struct seq_file m = {.private = &private, .buf = fw_core_dump->core_dump, .size = core_dump_size};
		fw_core_dump_write_elf_header(&m);
		size = m.count;
		if (unlikely(m.count >= m.size))
			dev_warn(kbdev->dev, "firmware core dump header may be larger than buffer size");
	}

	// Dumping pages
	list_for_each_entry(interface, &kbdev->csf.firmware_interfaces, node) {
		/* Skip memory sections that cannot be read or are protected. */
		if ((interface->flags & CSF_FIRMWARE_ENTRY_PROTECTED) ||
		    (interface->flags & CSF_FIRMWARE_ENTRY_READ) == 0)
			continue;

		for(i = 0; i < interface->num_pages; i++)
		{
			page = as_page(interface->phys[i]);
			write_size = size < core_dump_size ? min(core_dump_size - size, (size_t) FW_PAGE_SIZE) : 0;
			if (write_size)
			{
				p = kmap_atomic(page);
				memcpy(fw_core_dump->core_dump + size, p, write_size);
				kunmap_atomic(p);
			}
			size += FW_PAGE_SIZE;

			if (size < FW_PAGE_SIZE)
				break;
		}
	}

	if (unlikely(size != core_dump_size))
	{
		dev_err(kbdev->dev, "firmware core dump size and buffer size are different");
		vfree(seg->addr);
		seg->addr = NULL;
		seg->size = 0;
	}

	return;
}
static void get_rail_state_log(struct kbase_device *kbdev, struct sscd_segment *seg)
{
	lockdep_assert_held(&((struct pixel_context*)kbdev->platform_context)->pm.lock);

	seg->addr = gpu_pm_get_rail_state_log(kbdev);
	seg->size = gpu_pm_get_rail_state_log_size(kbdev);
}

static void get_pdc_state(struct kbase_device *kbdev, struct pixel_gpu_pdc_status *pdc_status,
			  struct sscd_segment *seg)
{
	lockdep_assert_held(&kbdev->hwaccess_lock);

	if (pdc_status == NULL) {
		dev_err(kbdev->dev, "pixel: failed to read PDC status, no storage");
		return;
	}
	gpu_debug_read_pdc_status(kbdev, pdc_status);
	seg->addr = pdc_status;
	seg->size = sizeof(*pdc_status);
}

static int segments_init(struct kbase_device *kbdev, struct sscd_segment* segments)
{
	/* Zero init everything for safety */
	memset(segments, 0, sizeof(struct sscd_segment) * NUM_SEGMENTS);

	segments[PM_EVENT_LOG].size = kbase_pm_max_event_log_size(kbdev);
	segments[PM_EVENT_LOG].addr = kzalloc(segments[PM_EVENT_LOG].size, GFP_KERNEL);

	if (!segments[PM_EVENT_LOG].addr) {
		segments[PM_EVENT_LOG].size = 0;
		return -ENOMEM;
	}

	segments[FW_TRACE].size = sizeof(struct pixel_fw_trace);
	segments[FW_TRACE].addr = kzalloc(sizeof(struct pixel_fw_trace), GFP_KERNEL);

	if (segments[FW_TRACE].addr == NULL) {
		segments[FW_TRACE].size = 0;
		return -ENOMEM;
	}

	segments[KTRACE].size = sizeof(struct pixel_ktrace);
	segments[KTRACE].addr = kzalloc(sizeof(struct pixel_ktrace), GFP_KERNEL);
	if (segments[KTRACE].addr == NULL) {
		segments[KTRACE].size = 0;
		return -ENOMEM;
	}

	segments[CONTEXTS].size = sizeof(struct pixel_context_snapshot) +
		MAX_SUPPORTED_CSGS * sizeof(struct pixel_context_snapshot_entry);
	segments[CONTEXTS].addr = kzalloc(segments[CONTEXTS].size, GFP_KERNEL);
	if (segments[CONTEXTS].addr == NULL) {
		segments[CONTEXTS].size = 0;
		return -ENOMEM;
	}

	return 0;
}

static void segments_term(struct kbase_device *kbdev, struct sscd_segment* segments)
{
	(void)kbdev;

	kfree(segments[FW_TRACE].addr);
	kfree(segments[PM_EVENT_LOG].addr);
	kfree(segments[KTRACE].addr);
	kfree(segments[CONTEXTS].addr);
	vfree(segments[FW_CORE_DUMP].addr);
	vfree(segments[KCTX_INFO].addr);
	/* Null out the pointers */
	memset(segments, 0, sizeof(struct sscd_segment) * NUM_SEGMENTS);
}

#define GPU_HANG_SSCD_TIMEOUT_MS (300000) /* 300s */

/**
 * gpu_sscd_dump() - Initiates and reports a subsystem core-dump of the GPU.
 *
 * @kbdev: The &struct kbase_device for the GPU.
 * @reason: A null terminated string containing a dump reason
 *
 * Context: Process context.
 */
void gpu_sscd_dump(struct kbase_device *kbdev, const char* reason)
{
	struct sscd_segment segs[NUM_SEGMENTS];
	struct sscd_platform_data *pdata = dev_get_platdata(&sscd_dev.dev);
	struct pixel_context *pc = kbdev->platform_context;
	int ec = 0;
	unsigned long flags, current_ts = jiffies;
	struct pixel_gpu_pdc_status pdc_status;
	static unsigned long last_hang_sscd_ts;

	if (!strcmp(reason, "GPU hang")) {
		/* GPU hang - avoid multiple coredumps for the same hang until
		 * GPU_HANG_SSCD_TIMEOUT_MS passes and GPU reset shows no failure.
		 */
		if (!last_hang_sscd_ts || (time_after(current_ts,
				last_hang_sscd_ts + msecs_to_jiffies(GPU_HANG_SSCD_TIMEOUT_MS)) &&
				!kbase_reset_gpu_failed(kbdev))) {
			last_hang_sscd_ts = current_ts;
		} else {
			dev_info(kbdev->dev, "pixel: skipping mali subsystem core dump");
			return;
		}
	}

	dev_info(kbdev->dev, "pixel: mali subsystem core dump in progress");
	/* No point in proceeding if we can't report the dumped data */
	if (!pdata->sscd_report) {
		dev_warn(kbdev->dev, "pixel: failed to report core dump, sscd_report was NULL");
		return;
	}

	ec = segments_init(kbdev, segs);
	if (ec != 0) {
		dev_err(kbdev->dev,
			"pixel: failed to init core dump segments (%d), partial dump in progress", ec);
	}

	/* We don't want anything messing with the HW while we dump */
	spin_lock_irqsave(&kbdev->hwaccess_lock, flags);

	/* Read the FW view of GPU PDC state, we get this early */
	get_pdc_state(kbdev, &pdc_status, &segs[PDC_STATUS]);

	get_fw_trace(kbdev, &segs[FW_TRACE]);

	get_pm_event_log(kbdev, &segs[PM_EVENT_LOG]);

	get_ktrace(kbdev, &segs[KTRACE]);
	spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);

	ec = get_contexts(kbdev, &segs[CONTEXTS]);
	if (ec) {
		dev_err(kbdev->dev,
			"could not collect active contexts: rc: %i", ec);
	}

	ec = get_kctx_info(kbdev, &segs[KCTX_INFO]);
	if (ec) {
		dev_err(kbdev->dev,
			"could not collect kctx info: rc: %i", ec);
	}

	if (!strcmp(reason, "Internal firmware error"))
		get_and_init_fw_core_dump(kbdev, &segs[FW_CORE_DUMP]);
	else
		dev_warn(kbdev->dev, "pixel: sscd: skipping FW core");

	/* Acquire the pm lock to prevent modifications to the rail state log */
	mutex_lock(&pc->pm.lock);

	get_rail_state_log(kbdev, &segs[POWER_RAIL_LOG]);

	/* Report the core dump and generate an ELF header for it */
	pdata->sscd_report(&sscd_dev, segs, NUM_SEGMENTS, SSCD_FLAGS_ELFARM64HDR, reason);

	/* Must be held until the dump completes, as the log is referenced rather than copied */
	mutex_unlock(&pc->pm.lock);

	segments_term(kbdev, segs);
}

/**
 * gpu_sscd_fw_log_init() - Set's the FW log verbosity.
 *
 * @kbdev: The &struct kbase_device for the GPU.
 * @level: The log verbosity.
 *
 * Context: Process context.
 *
 * Return: On success returns 0, otherwise returns an error code.
 */
int gpu_sscd_fw_log_init(struct kbase_device *kbdev, u32 level)
{
	u32 addr;
	int ec = kbase_csf_firmware_cfg_find_config_address(kbdev, "Log verbosity", &addr);

	if (!ec) {
		/* Update the FW log verbosity in FW memory */
		kbase_csf_update_firmware_memory(kbdev, addr, level);
	}

	return ec;
}

/**
 * gpu_sscd_init() - Registers the SSCD platform device.
 *
 * @kbdev: The &struct kbase_device for the GPU.
 *
 * Context: Process context.
 *
 * Return: On success, returns 0 otherwise returns an error code.
 */
int gpu_sscd_init(struct kbase_device *kbdev)
{
	sscd_dev = sscd_dev_init;
	return platform_device_register(&sscd_dev);
}

/**
 * gpu_sscd_term() - Unregisters the SSCD platform device.
 *
 * @kbdev: The &struct kbase_device for the GPU.
 *
 * Context: Process context.
 */
void gpu_sscd_term(struct kbase_device *kbdev)
{
	platform_device_unregister(&sscd_dev);
}
