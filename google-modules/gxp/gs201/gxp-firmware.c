// SPDX-License-Identifier: GPL-2.0
/*
 * GXP firmware loader.
 *
 * Copyright (C) 2021 Google LLC
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/elf.h>
#include <linux/gsa/gsa_image_auth.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/types.h>

#include <gcip/gcip-alloc-helper.h>
#include <gcip/gcip-common-image-header.h>
#include <gcip/gcip-image-config.h>

#include "gxp-bpm.h"
#include "gxp-config.h"
#include "gxp-core-telemetry.h"
#include "gxp-debug-dump.h"
#include "gxp-doorbell.h"
#include "gxp-firmware-data.h"
#include "gxp-firmware-loader.h"
#include "gxp-firmware.h"
#include "gxp-host-device-structs.h"
#include "gxp-internal.h"
#include "gxp-lpm.h"
#include "gxp-mailbox.h"
#include "gxp-notification.h"
#include "gxp-pm.h"
#include "gxp-vd.h"

#if IS_ENABLED(CONFIG_GXP_TEST)
#include "unittests/factory/fake-gxp-firmware.h"
#endif

#define FW_HEADER_SIZE		GCIP_FW_HEADER_SIZE

static int gxp_dsp_fw_auth_disable;
module_param_named(dsp_fw_auth_disable, gxp_dsp_fw_auth_disable, int, 0660);

static int
request_dsp_firmware(struct gxp_dev *gxp, char *name_prefix,
		     const struct firmware *out_firmwares[GXP_NUM_CORES])
{
	char *name_buf;
	/* 1 for NULL-terminator and up to 4 for core number */
	size_t name_len = strlen(name_prefix) + 5;
	int core;
	int ret = 0;

	name_buf = kzalloc(name_len, GFP_KERNEL);
	if (!name_buf)
		return -ENOMEM;

	for (core = 0; core < GXP_NUM_CORES; core++) {
		ret = snprintf(name_buf, name_len, "%s%d", name_prefix, core);
		if (ret <= 0 || ret >= name_len) {
			ret = -EINVAL;
			goto err;
		}

		dev_notice(gxp->dev, "Requesting dsp core %d firmware file: %s\n",
			   core, name_buf);
		ret = request_firmware(&out_firmwares[core], name_buf, NULL);
		if (ret < 0) {
			dev_err(gxp->dev,
				"Requesting dsp core %d firmware failed (ret=%d)\n",
				core, ret);
			goto err;
		}
		dev_dbg(gxp->dev, "dsp core %d firmware file obtained\n", core);
	}

	kfree(name_buf);
	return ret;

err:
	for (core -= 1; core >= 0; core--) {
		release_firmware(out_firmwares[core]);
		out_firmwares[core] = NULL;
	}
	kfree(name_buf);
	return ret;
}

static int elf_load_segments(struct gxp_dev *gxp, const u8 *elf_data,
			     size_t size,
			     const struct gxp_mapped_resource *buffer)
{
	struct elf32_hdr *ehdr;
	struct elf32_phdr *phdr;
	int i, ret = 0;

	ehdr = (struct elf32_hdr *)elf_data;
	phdr = (struct elf32_phdr *)(elf_data + ehdr->e_phoff);

	if ((ehdr->e_ident[EI_MAG0] != ELFMAG0) ||
	    (ehdr->e_ident[EI_MAG1] != ELFMAG1) ||
	    (ehdr->e_ident[EI_MAG2] != ELFMAG2) ||
	    (ehdr->e_ident[EI_MAG3] != ELFMAG3)) {
		dev_info(gxp->dev, "Firmware is not an ELF, treated as raw binary.");
		return 0;
	}

	/* go through the available ELF segments */
	for (i = 0; i < ehdr->e_phnum; i++, phdr++) {
		const u64 da = phdr->p_paddr;
		const u32 memsz = phdr->p_memsz;
		const u32 filesz = phdr->p_filesz;
		const u32 offset = phdr->p_offset;
		const u32 p_flags = phdr->p_flags;
		void *ptr;

		if (phdr->p_type != PT_LOAD)
			continue;

		if (!phdr->p_flags)
			continue;

		if (!memsz)
			continue;

		if (!(da >= buffer->daddr &&
		      da + memsz <= buffer->daddr + buffer->size)) {
			/*
			 * Some BSS data may be referenced from TCM, and can be
			 * skipped while loading
			 */
			dev_err(gxp->dev,
				"Segment out of bounds: da %#llx mem %#x. Skipping...",
				da, memsz);
			continue;
		}

		dev_info(gxp->dev,
			 "phdr: da %#llx memsz %#x filesz %#x perm %d", da,
			 memsz, filesz, p_flags);

		if (filesz > memsz) {
			dev_err(gxp->dev, "Bad phdr filesz %#x memsz %#x",
				filesz, memsz);
			ret = -EINVAL;
			break;
		}

		if (offset + filesz > size) {
			dev_err(gxp->dev, "Truncated fw: need %#x avail %#zx",
				offset + filesz, size);
			ret = -EINVAL;
			break;
		}

		/* grab the kernel address for this device address */
		ptr = buffer->vaddr + (da - buffer->daddr);
		if (!ptr) {
			dev_err(gxp->dev, "Bad phdr: da %#llx mem %#x", da,
				memsz);
			ret = -EINVAL;
			break;
		}

		/* put the segment where the remote processor expects it */
		if (phdr->p_filesz)
			memcpy_toio(ptr, elf_data + phdr->p_offset, filesz);

		/*
		 * Zero out remaining memory for this segment.
		 */
		if (memsz > filesz)
			memset(ptr + filesz, 0, memsz - filesz);
	}

	return ret;
}

static int
gxp_firmware_authenticate(struct gxp_dev *gxp,
			  const struct firmware *firmwares[GXP_NUM_CORES])
{
	const u8 *data;
	size_t size;
	void *header_vaddr;
	struct gxp_mapped_resource *buffer;
	dma_addr_t header_dma_addr;
	int core;
	int ret;

	if (gxp_dsp_fw_auth_disable) {
		dev_warn(gxp->dev,
			 "DSP FW authentication disabled, skipping\n");
		return 0;
	}

	if (!gxp->gsa_dev) {
		dev_warn(
			gxp->dev,
			"No GSA device available, skipping firmware authentication\n");
		return 0;
	}

	if (!gxp_is_direct_mode(gxp))
		return 0;

	for (core = 0; core < GXP_NUM_CORES; core++) {
		data = firmwares[core]->data;
		size = firmwares[core]->size;
		buffer = &gxp->fwbufs[core];

		if ((size - FW_HEADER_SIZE) > buffer->size) {
			dev_err(gxp->dev,
				"Firmware image does not fit (%zu > %llu)\n",
				size - FW_HEADER_SIZE, buffer->size);
			ret = -EINVAL;
			goto error;
		}

		dev_dbg(gxp->dev, "Authenticating firmware of core%u\n", core);

		/* Allocate coherent memory for the image header */
		header_vaddr = dma_alloc_coherent(gxp->gsa_dev, FW_HEADER_SIZE,
						  &header_dma_addr, GFP_KERNEL);
		if (!header_vaddr) {
			dev_err(gxp->dev,
				"Failed to allocate coherent memory for header\n");
			ret = -ENOMEM;
			goto error;
		}

		/* Copy the header to GSA coherent memory */
		memcpy(header_vaddr, data, FW_HEADER_SIZE);

		/* Copy the firmware image to the carveout location, skipping the header */
		memcpy_toio(buffer->vaddr, data + FW_HEADER_SIZE,
			    size - FW_HEADER_SIZE);

		dev_dbg(gxp->dev,
			"Requesting GSA authentication. meta = %pad payload = %pap",
			&header_dma_addr, &buffer->paddr);

		ret = gsa_authenticate_image(gxp->gsa_dev, header_dma_addr,
					     buffer->paddr);

		dma_free_coherent(gxp->gsa_dev, FW_HEADER_SIZE, header_vaddr,
				  header_dma_addr);

		if (ret) {
			dev_err(gxp->dev, "GSA authentication failed: %d\n",
				ret);
			memset_io(buffer->vaddr, 0, buffer->size);
			goto error;
		}
	}

	return 0;

error:
	/*
	 * Zero out firmware buffers if we got a authentication failure on any
	 * core.
	 */
	for (core -= 1; core >= 0; core--) {
		buffer = &gxp->fwbufs[core];
		memset_io(buffer->vaddr, 0, buffer->size);
	}
	return ret;
}

static void gxp_program_reset_vector(struct gxp_dev *gxp, uint core,
				     uint phys_core, bool verbose)
{
	u32 reset_vec;

	reset_vec = gxp_read_32(gxp, GXP_CORE_REG_ALT_RESET_VECTOR(phys_core));
	if (verbose)
		dev_notice(gxp->dev,
			   "Current Aurora reset vector for core %u: %#x\n",
			   phys_core, reset_vec);
	gxp_write_32(gxp, GXP_CORE_REG_ALT_RESET_VECTOR(phys_core),
		     gxp->fwbufs[core].daddr);
	if (verbose)
		dev_notice(gxp->dev,
			   "New Aurora reset vector for core %u: %#llx\n",
			   phys_core, gxp->fwbufs[core].daddr);
}

static void *get_scratchpad_base(struct gxp_dev *gxp,
				 struct gxp_virtual_device *vd, uint core)
{
	void *mem;
	size_t rw_size;

	if (vd && gxp_fw_data_use_per_vd_config(vd))
		return vd->core_cfg.vaddr +
		       (vd->core_cfg.size / GXP_NUM_CORES) * core;

	if (!vd || !vd->rwdata_sgt[core])
		return gxp->fwbufs[core].vaddr + AURORA_SCRATCHPAD_OFF;

	/* Return the last AURORA_SCRATCHPAD_LEN of rwdata_sgt. */
	mem = gcip_noncontiguous_sgt_to_mem(vd->rwdata_sgt[core]);
	rw_size = gxp->fwbufs[core].size - vd->fw_ro_size;
	return mem + rw_size - AURORA_SCRATCHPAD_LEN;
}

/* TODO(b/265562894): remove scratchpad region support */
static void flush_scratchpad_region(struct gxp_dev *gxp,
				   struct gxp_virtual_device *vd, uint core)
{
	if (!vd || gxp_fw_data_use_per_vd_config(vd) || !vd->rwdata_sgt[core])
		return;
	dma_sync_sg_for_device(gxp->dev, vd->rwdata_sgt[core]->sgl,
			       vd->rwdata_sgt[core]->orig_nents,
			       DMA_BIDIRECTIONAL);
}

static void invalidate_scratchpad_region(struct gxp_dev *gxp,
					struct gxp_virtual_device *vd,
					uint core)
{
	if (!vd || gxp_fw_data_use_per_vd_config(vd) || !vd->rwdata_sgt[core])
		return;
	dma_sync_sg_for_cpu(gxp->dev, vd->rwdata_sgt[core]->sgl,
			    vd->rwdata_sgt[core]->orig_nents,
			    DMA_BIDIRECTIONAL);
}

static void reset_core_config_region(struct gxp_dev *gxp,
				     struct gxp_virtual_device *vd, uint core)
{
	struct gxp_host_control_region *core_cfg;

	core_cfg = get_scratchpad_base(gxp, vd, core);
	if (gxp_fw_data_use_per_vd_config(vd)) {
		core_cfg->core_alive_magic = 0;
		core_cfg->top_access_ok = 0;
		core_cfg->boot_status = GXP_BOOT_STATUS_NONE;
		gxp_firmware_set_boot_mode(gxp, vd, core,
					   GXP_BOOT_MODE_COLD_BOOT);
	} else {
		memset(core_cfg, 0, AURORA_SCRATCHPAD_LEN);
		gxp_firmware_set_boot_mode(gxp, vd, core,
					   GXP_BOOT_MODE_REQUEST_COLD_BOOT);
	}
}

static int gxp_firmware_handshake(struct gxp_dev *gxp,
				  struct gxp_virtual_device *vd, uint core,
				  uint phys_core)
{
	u32 __maybe_unused expected_top_value;
	/* Prevent the read loop below from being optimized. */
	volatile struct gxp_host_control_region *core_cfg;
	int ctr;

	/* Wait for core to come up */
	dev_notice(gxp->dev, "Waiting for core %u to power up...\n", phys_core);
	ctr = 1000;
	while (ctr) {
		if (gxp_lpm_is_powered(gxp, CORE_TO_PSM(phys_core)))
			break;
		udelay(1 * GXP_TIME_DELAY_FACTOR);
		ctr--;
	}

	if (!ctr) {
		dev_notice(gxp->dev, "Failed!\n");
		return -ETIMEDOUT;
	}
	dev_notice(gxp->dev, "Powered up!\n");

	/* Wait for 500ms. Then check if Q7 core is alive */
	dev_notice(gxp->dev, "Waiting for core %u to respond...\n",
		   phys_core);

	core_cfg = get_scratchpad_base(gxp, vd, core);

	/*
	 * Currently, the hello_world FW writes a magic number
	 * (Q7_ALIVE_MAGIC) to offset MSG_CORE_ALIVE in the scratchpad
	 * space as an alive message
	 */
	ctr = 5000;
#if IS_ENABLED(CONFIG_GXP_TEST)
	fake_gxp_firmware_flush_work_all();
	/*
	 * As the fake firmware works are flushed, we don't have to busy-wait the response of
	 * the firmware. By setting @ctr to 1, just run the while loop below once for the code
	 * coverage.
	 */
	ctr = 1;
#endif
	usleep_range(50 * GXP_TIME_DELAY_FACTOR, 60 * GXP_TIME_DELAY_FACTOR);
	while (ctr--) {
		invalidate_scratchpad_region(gxp, vd, core);
		if (core_cfg->core_alive_magic == Q7_ALIVE_MAGIC)
			break;
		usleep_range(1 * GXP_TIME_DELAY_FACTOR,
			     10 * GXP_TIME_DELAY_FACTOR);
	}
	invalidate_scratchpad_region(gxp, vd, core);
	if (core_cfg->core_alive_magic != Q7_ALIVE_MAGIC) {
		dev_err(gxp->dev, "Core %u did not respond!\n", phys_core);
		return -EIO;
	}
	dev_notice(gxp->dev, "Core %u is alive!\n", phys_core);

#if !IS_ENABLED(CONFIG_GXP_GEM5)
	/*
	 * FW reads the INT_MASK0 register (written by the driver) to
	 * validate TOP access. The value read is echoed back by the FW to
	 * offset MSG_TOP_ACCESS_OK in the scratchpad space, which must be
	 * compared to the value written in the INT_MASK0 register by the
	 * driver for confirmation.
	 * On Gem5, FW will start early when lpm is up. This behavior will
	 * affect the order of reading/writing INT_MASK0, so ignore these
	 * handshakes in Gem5.
	 */
	ctr = 1000;
	expected_top_value = BIT(CORE_WAKEUP_DOORBELL(phys_core));
	while (ctr--) {
		invalidate_scratchpad_region(gxp, vd, core);
		if (core_cfg->top_access_ok == expected_top_value)
			break;
		udelay(1 * GXP_TIME_DELAY_FACTOR);
	}
	if (core_cfg->top_access_ok != expected_top_value) {
		dev_err(gxp->dev, "TOP access from core %u failed!\n", phys_core);
		return -EIO;
	}
	dev_notice(gxp->dev, "TOP access from core %u successful!\n", phys_core);
#endif

	/* Stop bus performance monitors */
	gxp_bpm_stop(gxp, phys_core);
	dev_notice(gxp->dev, "Core%u Instruction read transactions: 0x%x\n",
		   core, gxp_bpm_read_counter(gxp, phys_core, INST_BPM_OFFSET));
	dev_notice(gxp->dev, "Core%u Data write transactions: 0x%x\n",
		   phys_core,
		   gxp_bpm_read_counter(gxp, phys_core, DATA_BPM_OFFSET));

	return 0;
}

static int
gxp_firmware_load_into_memories(struct gxp_dev *gxp,
				const struct firmware *firmwares[GXP_NUM_CORES])
{
	int core;
	int ret;

	for (core = 0; core < GXP_NUM_CORES; core++) {
		/* Load firmware to System RAM */
		if (FW_HEADER_SIZE > firmwares[core]->size) {
			dev_err(gxp->dev,
				"Invalid Core %u firmware Image size (%d > %zu)\n",
				core, FW_HEADER_SIZE, firmwares[core]->size);
			ret = -EINVAL;
			goto error;
		}

		if ((firmwares[core]->size - FW_HEADER_SIZE) >
		    gxp->fwbufs[core].size) {
			dev_err(gxp->dev,
				"Core %u firmware image does not fit (%zu > %llu)\n",
				core, firmwares[core]->size - FW_HEADER_SIZE,
				gxp->fwbufs[core].size);
			ret = -EINVAL;
			goto error;
		}
		memcpy_toio(gxp->fwbufs[core].vaddr,
			    firmwares[core]->data + FW_HEADER_SIZE,
			    firmwares[core]->size - FW_HEADER_SIZE);
	}
	return 0;
error:
	/* Zero out firmware buffers if we got invalid size on any core. */
	for (core -= 1; core >= 0; core--)
		memset_io(gxp->fwbufs[core].vaddr, 0, gxp->fwbufs[core].size);
	return ret;
}

int gxp_firmware_rearrange_elf(struct gxp_dev *gxp,
			       const struct firmware *firmwares[GXP_NUM_CORES])
{
	int ret = 0;
	uint core;

	for (core = 0; core < GXP_NUM_CORES; core++) {
		/* Re-arrange ELF firmware in System RAM */
		ret = elf_load_segments(gxp,
					firmwares[core]->data + FW_HEADER_SIZE,
					firmwares[core]->size - FW_HEADER_SIZE,
					&gxp->fwbufs[core]);
		if (ret) {
			dev_err(gxp->dev,
				"Failed to parse ELF firmware on core %u\n",
				core);
			return ret;
		}
	}
	return ret;
}

/* Helper function to parse name written to sysfs "load_dsp_firmware" node */
static char *fw_name_from_attr_buf(const char *buf)
{
	size_t len;
	char *name;

	len = strlen(buf);
	if (len == 0 || buf[len - 1] != '\n')
		return ERR_PTR(-EINVAL);

	name = kstrdup(buf, GFP_KERNEL);
	if (!name)
		return ERR_PTR(-ENOMEM);

	name[len - 1] = '\0';
	return name;
}

/* sysfs node for loading custom firmware */

static ssize_t load_dsp_firmware_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct gxp_dev *gxp = dev_get_drvdata(dev);
	ssize_t ret;
	char *firmware_name = gxp_firmware_loader_get_core_fw_name(gxp);

	ret = scnprintf(buf, PAGE_SIZE, "%s\n", firmware_name);
	kfree(firmware_name);
	return ret;
}

static ssize_t load_dsp_firmware_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct gxp_dev *gxp = dev_get_drvdata(dev);
	struct gxp_firmware_manager *mgr = gxp->firmware_mgr;
	char *name_buf = NULL;
	int ret;

	/*
	 * Lock the VD semaphore to ensure no core is executing the firmware
	 * while requesting new firmware.
	 */
	down_read(&gxp->vd_semaphore);

	if (mgr->firmware_running) {
		dev_warn(dev, "Cannot update firmware when any core is running\n");
		ret = -EBUSY;
		goto err_out;
	}

	name_buf = fw_name_from_attr_buf(buf);
	if (IS_ERR(name_buf)) {
		dev_err(gxp->dev, "Invalid firmware prefix requested: %s\n",
			buf);
		ret = PTR_ERR(name_buf);
		goto err_out;
	}

	dev_notice(gxp->dev, "Requesting firmware be reloaded: %s\n", name_buf);

	/*
	 * It's possible a race condition bug here that someone opens a gxp
	 * device and loads the firmware between below unload/load functions in
	 * another thread, but this interface is only for developer debugging.
	 * We don't insist on preventing the race condition bug.
	 */
	gxp_firmware_loader_unload(gxp);
	gxp_firmware_loader_set_core_fw_name(gxp, name_buf);
	ret = gxp_firmware_loader_load_if_needed(gxp);
	if (ret) {
		dev_err(gxp->dev, "Failed to load core firmware: %s\n", name_buf);
		goto err_firmware_load;
	}

	kfree(name_buf);
	up_read(&gxp->vd_semaphore);
	return count;

err_firmware_load:
	kfree(name_buf);
err_out:
	up_read(&gxp->vd_semaphore);
	return ret;
}

static DEVICE_ATTR_RW(load_dsp_firmware);

static struct attribute *dev_attrs[] = {
	&dev_attr_load_dsp_firmware.attr,
	NULL,
};

static const struct attribute_group gxp_firmware_attr_group = {
	.attrs = dev_attrs,
};

int gxp_fw_init(struct gxp_dev *gxp)
{
	u32 ver, proc_id;
	uint core;
	struct resource r;
	int ret;
	struct gxp_firmware_manager *mgr;

	mgr = devm_kzalloc(gxp->dev, sizeof(*mgr), GFP_KERNEL);
	if (!mgr)
		return -ENOMEM;
	gxp->firmware_mgr = mgr;

	/* Power on BLK_AUR to read the revision and processor ID registers */
	gxp_pm_blk_on(gxp);

	ver = gxp_read_32(gxp, GXP_REG_AURORA_REVISION);
	dev_notice(gxp->dev, "Aurora version: 0x%x\n", ver);

	for (core = 0; core < GXP_NUM_CORES; core++) {
		proc_id = gxp_read_32(gxp, GXP_CORE_REG_PROCESSOR_ID(core));
		dev_notice(gxp->dev, "Aurora core %u processor ID: 0x%x\n",
			   core, proc_id);
	}

	/* Shut BLK_AUR down again to avoid interfering with power management */
	gxp_pm_blk_off(gxp);

	ret = gxp_acquire_rmem_resource(gxp, &r, "gxp-fw-region");
	if (ret) {
		dev_err(gxp->dev,
			"Unable to acquire firmware reserved memory\n");
		return ret;
	}

	for (core = 0; core < GXP_NUM_CORES; core++) {
		gxp->fwbufs[core].size =
			(resource_size(&r) / GXP_NUM_CORES) & PAGE_MASK;
		gxp->fwbufs[core].paddr =
			r.start + (core * gxp->fwbufs[core].size);
		/*
		 * Firmware buffers are not mapped into kernel VA space until
		 * firmware is ready to be loaded.
		 */
	}

	ret = gxp_acquire_rmem_resource(gxp, &r, "gxp-scratchpad-region");
	if (ret) {
		dev_err(gxp->dev,
			"Unable to acquire shared FW data reserved memory\n");
		return ret;
	}
	gxp->fwdatabuf.size = resource_size(&r);
	gxp->fwdatabuf.paddr = r.start;
	/*
	 * Scratchpad region is not mapped until the firmware data is
	 * initialized.
	 */

	for (core = 0; core < GXP_NUM_CORES; core++) {
		/*
		 * Currently, the Q7 FW needs to be statically linked to a base
		 * address where it would be loaded in memory. This requires the
		 * address (where the FW is to be loaded in DRAM) to be
		 * pre-defined, and hence not allocate-able dynamically (using
		 * the kernel's memory management system). Therefore, we are
		 * memremapping a static address and loading the FW there, while
		 * also having compiled the FW with this as the base address
		 * (used by the linker).
		 */
		gxp->fwbufs[core].vaddr =
			memremap(gxp->fwbufs[core].paddr,
				 gxp->fwbufs[core].size, MEMREMAP_WC);
		if (!(gxp->fwbufs[core].vaddr)) {
			dev_err(gxp->dev, "FW buf %d memremap failed\n", core);
			ret = -EINVAL;
			goto out_fw_destroy;
		}
	}

	ret = device_add_group(gxp->dev, &gxp_firmware_attr_group);
	if (ret)
		goto out_fw_destroy;

	mgr->firmware_running = 0;
	return 0;

out_fw_destroy:
	gxp_fw_destroy(gxp);
	return ret;
}

void gxp_fw_destroy(struct gxp_dev *gxp)
{
	uint core;
	struct gxp_firmware_manager *mgr = gxp->firmware_mgr;

	if (IS_GXP_TEST && !mgr)
		return;

	device_remove_group(gxp->dev, &gxp_firmware_attr_group);

	for (core = 0; core < GXP_NUM_CORES; core++) {
		if (gxp->fwbufs[core].vaddr) {
			memunmap(gxp->fwbufs[core].vaddr);
			gxp->fwbufs[core].vaddr = NULL;
		}
	}
}

int gxp_firmware_load_core_firmware(
	struct gxp_dev *gxp, char *name_prefix,
	const struct firmware *core_firmware[GXP_NUM_CORES])
{
	uint core;
	int ret;

	if (name_prefix == NULL)
		name_prefix = DSP_FIRMWARE_DEFAULT_PREFIX;
	ret = request_dsp_firmware(gxp, name_prefix, core_firmware);
	if (ret)
		return ret;
	ret = gxp_firmware_load_into_memories(gxp, core_firmware);
	if (ret)
		goto error;
	ret = gxp_firmware_authenticate(gxp, core_firmware);
	if (ret)
		goto error;

	return 0;
error:
	for (core = 0; core < GXP_NUM_CORES; core++) {
		release_firmware(core_firmware[core]);
		core_firmware[core] = NULL;
	}
	return ret;
}

/* TODO(b/253464747): Refactor these interrupts handlers and gxp-doorbell.c. */
static void enable_core_interrupts(struct gxp_dev *gxp, uint core)
{
	/*
	 * GXP_CORE_REG_COMMON_INT_MASK_0 is handled in doorbell module, so we
	 * don't need to enable it here.
	 */
	gxp_write_32(gxp, GXP_CORE_REG_COMMON_INT_MASK_1(core), 0xffffffff);
	gxp_write_32(gxp, GXP_CORE_REG_DEDICATED_INT_MASK(core), 0xffffffff);
}

void gxp_firmware_disable_ext_interrupts(struct gxp_dev *gxp, uint core)
{
	gxp_write_32(gxp, GXP_CORE_REG_COMMON_INT_MASK_0(core), 0);
	gxp_write_32(gxp, GXP_CORE_REG_COMMON_INT_MASK_1(core), 0);
	gxp_write_32(gxp, GXP_CORE_REG_DEDICATED_INT_MASK(core), 0);
}

static inline uint select_core(struct gxp_virtual_device *vd, uint virt_core,
			       uint phys_core)
{
	return gxp_fw_data_use_per_vd_config(vd) ? virt_core : phys_core;
}

static int gxp_firmware_setup(struct gxp_dev *gxp,
			      struct gxp_virtual_device *vd, uint core,
			      uint phys_core)
{
	int ret = 0;
	struct gxp_firmware_manager *mgr = gxp->firmware_mgr;

	if (gxp_core_boot(gxp) && mgr->firmware_running & BIT(phys_core)) {
		dev_err(gxp->dev, "Firmware is already running on core %u\n",
			phys_core);
		return -EBUSY;
	}

	/* Configure bus performance monitors */
	gxp_bpm_configure(gxp, phys_core, INST_BPM_OFFSET, BPM_EVENT_READ_XFER);
	gxp_bpm_configure(gxp, phys_core, DATA_BPM_OFFSET, BPM_EVENT_WRITE_XFER);

	/* Mark this as a cold boot */
	if (gxp_core_boot(gxp)) {
		reset_core_config_region(gxp, vd, core);
		ret = gxp_firmware_setup_hw_after_block_off(gxp, core,
							    phys_core,
							    /*verbose=*/true);
		if (ret) {
			dev_err(gxp->dev, "Failed to power up core %u\n", core);
			return ret;
		}
		enable_core_interrupts(gxp, phys_core);
	}

	return ret;
}

static void gxp_firmware_wakeup_cores(struct gxp_dev *gxp, uint core_list)
{
	uint core;

	/* Raise wakeup doorbell */
	for (core = 0; core < GXP_NUM_CORES; core++) {
		if (!(core_list & BIT(core)))
			continue;
#if !IS_ENABLED(CONFIG_GXP_GEM5)
		gxp_doorbell_enable_for_core(gxp, CORE_WAKEUP_DOORBELL(core),
					     core);
#endif
		gxp_doorbell_set(gxp, CORE_WAKEUP_DOORBELL(core));
	}
}

static int gxp_firmware_finish_startup(struct gxp_dev *gxp,
				       struct gxp_virtual_device *vd,
				       uint virt_core, uint phys_core)
{
	struct work_struct *work;
	struct gxp_firmware_manager *mgr = gxp->firmware_mgr;
	int ret = 0;
	uint core = select_core(vd, virt_core, phys_core);

	if (gxp_core_boot(gxp)) {
		ret = gxp_firmware_handshake(gxp, vd, core, phys_core);
		if (ret) {
			dev_err(gxp->dev,
				"Firmware handshake failed on core %u\n",
				phys_core);
			goto err_firmware_off;
		}

		/* Initialize mailbox */
		if (gxp->mailbox_mgr->allocate_mailbox) {
			gxp->mailbox_mgr->mailboxes[phys_core] =
				gxp->mailbox_mgr->allocate_mailbox(
					gxp->mailbox_mgr, vd, virt_core, phys_core);
			if (IS_ERR(gxp->mailbox_mgr->mailboxes[phys_core])) {
				dev_err(gxp->dev,
					"Unable to allocate mailbox (core=%u, ret=%ld)\n",
					phys_core,
					PTR_ERR(gxp->mailbox_mgr
							->mailboxes[phys_core]));
				ret = PTR_ERR(
					gxp->mailbox_mgr->mailboxes[phys_core]);
				gxp->mailbox_mgr->mailboxes[phys_core] = NULL;
				goto err_firmware_off;
			}
		}
		mgr->firmware_running |= BIT(phys_core);
	}

	work = gxp_debug_dump_get_notification_handler(gxp, phys_core);
	if (work)
		gxp_notification_register_handler(
			gxp, phys_core, HOST_NOTIF_DEBUG_DUMP_READY, work);

	work = gxp_core_telemetry_get_notification_handler(gxp, phys_core);
	if (work)
		gxp_notification_register_handler(
			gxp, phys_core, HOST_NOTIF_CORE_TELEMETRY_STATUS, work);

	return ret;

err_firmware_off:
	if (gxp_core_boot(gxp))
		gxp_pm_core_off(gxp, phys_core);
	return ret;
}

static void gxp_firmware_stop_core(struct gxp_dev *gxp,
				   struct gxp_virtual_device *vd,
				   uint virt_core, uint phys_core)
{
	struct gxp_firmware_manager *mgr = gxp->firmware_mgr;

	if (gxp_core_boot(gxp) && !(mgr->firmware_running & BIT(phys_core)))
		dev_err(gxp->dev, "Firmware is not running on core %u\n",
			phys_core);

	mgr->firmware_running &= ~BIT(phys_core);

	gxp_notification_unregister_handler(gxp, phys_core,
					    HOST_NOTIF_DEBUG_DUMP_READY);
	gxp_notification_unregister_handler(gxp, phys_core,
					    HOST_NOTIF_CORE_TELEMETRY_STATUS);

	if (gxp_core_boot(gxp)) {
		if (gxp->mailbox_mgr->release_mailbox) {
			gxp->mailbox_mgr->release_mailbox(
				gxp->mailbox_mgr, vd, virt_core,
				gxp->mailbox_mgr->mailboxes[phys_core]);
			dev_notice(gxp->dev, "Mailbox %u released\n",
				   phys_core);
		}

		if (vd->state == GXP_VD_RUNNING) {
			/*
			 * Disable interrupts to prevent cores from being woken up
			 * unexpectedly.
			 */
			gxp_firmware_disable_ext_interrupts(gxp, phys_core);
			gxp_pm_core_off(gxp, phys_core);
		}
	}
}

int gxp_firmware_run(struct gxp_dev *gxp, struct gxp_virtual_device *vd,
		     uint core_list)
{
	int ret;
	uint phys_core, virt_core;
	uint failed_cores = 0;
	int failed_ret;

	virt_core = 0;
	for (phys_core = 0; phys_core < GXP_NUM_CORES; phys_core++) {
		uint core = select_core(vd, virt_core, phys_core);

		if (!(core_list & BIT(phys_core)))
			continue;

		ret = gxp_firmware_setup(gxp, vd, core, phys_core);
		if (ret) {
			failed_cores |= BIT(phys_core);
			failed_ret = ret;
			dev_err(gxp->dev, "Failed to run firmware on core %u\n",
				phys_core);
		}
		virt_core++;
	}
	if (failed_cores != 0) {
		/*
		 * Shut down the cores which call `gxp_firmware_setup`
		 * successfully
		 */
		virt_core = 0;
		for (phys_core = 0; phys_core < GXP_NUM_CORES; phys_core++) {
			if (!(core_list & BIT(phys_core)))
				continue;
			if (!(failed_cores & BIT(phys_core))) {
				if (gxp_core_boot(gxp))
					gxp_pm_core_off(gxp, phys_core);
			}
			virt_core++;
		}
		return failed_ret;
	}
#if IS_ENABLED(CONFIG_GXP_GEM5)
	/*
	 * GEM5 starts firmware after LPM is programmed, so we need to call
	 * gxp_doorbell_enable_for_core here to set GXP_REG_COMMON_INT_MASK_0
	 * first to enable the firmware handshakes.
	 */
	for (phys_core = 0; phys_core < GXP_NUM_CORES; phys_core++) {
		if (!(core_list & BIT(phys_core)))
			continue;
		gxp_doorbell_enable_for_core(
			gxp, CORE_WAKEUP_DOORBELL(phys_core), phys_core);
	}
#endif
	/* Switch clock mux to the normal state to guarantee LPM works */
	if (gxp_core_boot(gxp)) {
		gxp_pm_force_clkmux_normal(gxp);
		gxp_firmware_wakeup_cores(gxp, core_list);
	}

	virt_core = 0;
	for (phys_core = 0; phys_core < GXP_NUM_CORES; phys_core++) {
		if (!(core_list & BIT(phys_core)))
			continue;
		ret = gxp_firmware_finish_startup(gxp, vd, virt_core,
						  phys_core);
		if (ret) {
			failed_cores |= BIT(phys_core);
			dev_err(gxp->dev, "Failed to run firmware on core %u\n",
				phys_core);
		}
		virt_core++;
	}

	if (failed_cores != 0) {
		virt_core = 0;
		for (phys_core = 0; phys_core < GXP_NUM_CORES; phys_core++) {
			if (!(core_list & BIT(phys_core)))
				continue;
			if (!(failed_cores & BIT(phys_core)))
				gxp_firmware_stop_core(gxp, vd, virt_core,
						       phys_core);
			virt_core++;
		}
	}
	/* Check if we need to set clock mux to low state as requested */
	if (gxp_core_boot(gxp))
		gxp_pm_resume_clkmux(gxp);

	return ret;
}

int gxp_firmware_setup_hw_after_block_off(struct gxp_dev *gxp, uint core,
					  uint phys_core, bool verbose)
{
	gxp_program_reset_vector(gxp, core, phys_core, verbose);
	return gxp_pm_core_on(gxp, phys_core, verbose);
}

void gxp_firmware_stop(struct gxp_dev *gxp, struct gxp_virtual_device *vd,
		       uint core_list)
{
	uint core, virt_core = 0;

	for (core = 0; core < GXP_NUM_CORES; core++) {
		if (core_list & BIT(core)) {
			gxp_firmware_stop_core(gxp, vd, virt_core, core);
			virt_core++;
		}
	}
}

void gxp_firmware_set_boot_mode(struct gxp_dev *gxp,
				struct gxp_virtual_device *vd, uint core,
				u32 mode)
{
	struct gxp_host_control_region *core_cfg;

	/* Callers shouldn't call the function under this condition. */
	if (!gxp->fwbufs[core].vaddr)
		return;

	core_cfg = get_scratchpad_base(gxp, vd, core);
	core_cfg->boot_mode = mode;
	flush_scratchpad_region(gxp, vd, core);
}

u32 gxp_firmware_get_boot_mode(struct gxp_dev *gxp,
			       struct gxp_virtual_device *vd, uint core)
{
	struct gxp_host_control_region *core_cfg;

	/* Callers shouldn't call the function under this condition. */
	if (!gxp->fwbufs[core].vaddr)
		return 0;

	core_cfg = get_scratchpad_base(gxp, vd, core);
	invalidate_scratchpad_region(gxp, vd, core);
	return core_cfg->boot_mode;
}

void gxp_firmware_set_boot_status(struct gxp_dev *gxp,
				  struct gxp_virtual_device *vd, uint core,
				  u32 status)
{
	struct gxp_host_control_region *core_cfg;

	core_cfg = get_scratchpad_base(gxp, vd, core);
	core_cfg->boot_status = status;
}

u32 gxp_firmware_get_boot_status(struct gxp_dev *gxp,
				 struct gxp_virtual_device *vd, uint core)
{
	struct gxp_host_control_region *core_cfg;

	core_cfg = get_scratchpad_base(gxp, vd, core);
	return core_cfg->boot_status;
}

bool gxp_core_boot(struct gxp_dev *gxp)
{
	return gxp_is_direct_mode(gxp);
}
