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

#include "gxp-bpm.h"
#include "gxp-debug-dump.h"
#include "gxp-doorbell.h"
#include "gxp-firmware.h"
#include "gxp-host-device-structs.h"
#include "gxp-internal.h"
#include "gxp-lpm.h"
#include "gxp-mailbox.h"
#include "gxp-notification.h"
#include "gxp-pm.h"
#include "gxp-telemetry.h"
#include "gxp-vd.h"

/* Files need to be copied to /lib/firmware */
#define DSP_FIRMWARE_DEFAULT_PREFIX	"gxp_fw_core"

#define FW_HEADER_SIZE		(0x1000)
#define FW_IMAGE_TYPE_OFFSET	(0x400)

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
	for (core -= 1; core >= 0; core--)
		release_firmware(out_firmwares[core]);
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
		dev_err(gxp->dev, "Cannot load FW! Invalid ELF format.\n");
		return -EINVAL;
	}

	/* go through the available ELF segments */
	for (i = 0; i < ehdr->e_phnum; i++, phdr++) {
		u64 da = phdr->p_paddr;
		u32 memsz = phdr->p_memsz;
		u32 filesz = phdr->p_filesz;
		u32 offset = phdr->p_offset;
		void *ptr;

		if (phdr->p_type != PT_LOAD)
			continue;

		if (!phdr->p_flags)
			continue;

		if (!memsz)
			continue;

		if (!((da >= (u32)buffer->daddr) &&
		   ((da + memsz) <= ((u32)buffer->daddr +
				     (u32)buffer->size)))) {
			/*
			 * Some BSS data may be referenced from TCM, and can be
			 * skipped while loading
			 */
			dev_err(gxp->dev, "Segment out of bounds: da 0x%llx mem 0x%x. Skipping...\n",
				da, memsz);
			continue;
		}

		dev_notice(gxp->dev, "phdr: type %d da 0x%llx memsz 0x%x filesz 0x%x\n",
			   phdr->p_type, da, memsz, filesz);

		if (filesz > memsz) {
			dev_err(gxp->dev, "Bad phdr filesz 0x%x memsz 0x%x\n",
				filesz, memsz);
			ret = -EINVAL;
			break;
		}

		if (offset + filesz > size) {
			dev_err(gxp->dev, "Truncated fw: need 0x%x avail 0x%zx\n",
				offset + filesz, size);
			ret = -EINVAL;
			break;
		}

		/* grab the kernel address for this device address */
		ptr = buffer->vaddr + (da - buffer->daddr);
		if (!ptr) {
			dev_err(gxp->dev, "Bad phdr: da 0x%llx mem 0x%x\n",
				da, memsz);
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

/* Forward declaration for usage inside gxp_firmware_load(..). */
static void gxp_firmware_unload(struct gxp_dev *gxp, uint core);

static void gxp_program_reset_vector(struct gxp_dev *gxp, uint core, bool verbose)
{
	u32 reset_vec;

	reset_vec = gxp_read_32_core(gxp, core,
				     GXP_REG_ALT_RESET_VECTOR);
	if (verbose)
		dev_notice(gxp->dev,
			   "Current Aurora reset vector for core %u: 0x%x\n",
			   core, reset_vec);
	gxp_write_32_core(gxp, core, GXP_REG_ALT_RESET_VECTOR,
			  gxp->fwbufs[core].daddr);
	if (verbose)
		dev_notice(gxp->dev,
			   "New Aurora reset vector for core %u: 0x%llx\n",
			   core, gxp->fwbufs[core].daddr);
}

static int gxp_firmware_load(struct gxp_dev *gxp, uint core)
{
	u32 offset;
	void __iomem *core_scratchpad_base;
	int ret;

	if (!gxp->firmwares[core])
		return -ENODEV;

	/* Load firmware to System RAM */
	ret = elf_load_segments(gxp,
				gxp->firmwares[core]->data + FW_HEADER_SIZE,
				gxp->firmwares[core]->size - FW_HEADER_SIZE,
				&gxp->fwbufs[core]);
	if (ret) {
		dev_err(gxp->dev, "Unable to load elf file\n");
		goto out_firmware_unload;
	}

	memset(gxp->fwbufs[core].vaddr + AURORA_SCRATCHPAD_OFF, 0,
	       AURORA_SCRATCHPAD_LEN);

	core_scratchpad_base = gxp->fwbufs[core].vaddr + AURORA_SCRATCHPAD_OFF;
	offset = SCRATCHPAD_MSG_OFFSET(MSG_CORE_ALIVE);
	writel(0, core_scratchpad_base + offset);
	offset = SCRATCHPAD_MSG_OFFSET(MSG_TOP_ACCESS_OK);
	writel(0, core_scratchpad_base + offset);

	/* TODO(b/188970444): Cleanup logging of addresses */
	dev_notice(gxp->dev,
		   "ELF loaded at virtual: %pK and physical: 0x%llx\n",
		   gxp->fwbufs[core].vaddr, gxp->fwbufs[core].paddr);

	/* Configure bus performance monitors */
	gxp_bpm_configure(gxp, core, INST_BPM_OFFSET, BPM_EVENT_READ_XFER);
	gxp_bpm_configure(gxp, core, DATA_BPM_OFFSET, BPM_EVENT_WRITE_XFER);

	return 0;

out_firmware_unload:
	gxp_firmware_unload(gxp, core);
	return ret;
}

static int gxp_firmware_handshake(struct gxp_dev *gxp, uint core)
{
	u32 offset;
	u32 expected_top_value;
	void __iomem *core_scratchpad_base;
	int ctr;

	/* Wait for core to come up */
	dev_notice(gxp->dev, "Waiting for core %u to power up...\n", core);
	ctr = 1000;
	while (ctr) {
		if (gxp_lpm_is_powered(gxp, core))
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
		   core);

	core_scratchpad_base = gxp->fwbufs[core].vaddr + AURORA_SCRATCHPAD_OFF;

	/*
	 * Currently, the hello_world FW writes a magic number
	 * (Q7_ALIVE_MAGIC) to offset MSG_CORE_ALIVE in the scratchpad
	 * space as an alive message
	 */
	ctr = 5000;
	offset = SCRATCHPAD_MSG_OFFSET(MSG_CORE_ALIVE);
	usleep_range(50 * GXP_TIME_DELAY_FACTOR, 60 * GXP_TIME_DELAY_FACTOR);
	while (ctr--) {
		if (readl(core_scratchpad_base + offset) == Q7_ALIVE_MAGIC)
			break;
		usleep_range(1 * GXP_TIME_DELAY_FACTOR,
			     10 * GXP_TIME_DELAY_FACTOR);
	}
	if (readl(core_scratchpad_base + offset) != Q7_ALIVE_MAGIC) {
		dev_err(gxp->dev, "Core %u did not respond!\n", core);
		return -EIO;
	}
	dev_notice(gxp->dev, "Core %u is alive!\n", core);

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
	offset = SCRATCHPAD_MSG_OFFSET(MSG_TOP_ACCESS_OK);
	expected_top_value = BIT(CORE_WAKEUP_DOORBELL(core));
	while (ctr--) {
		if (readl(core_scratchpad_base + offset) == expected_top_value)
			break;
		udelay(1 * GXP_TIME_DELAY_FACTOR);
	}
	if (readl(core_scratchpad_base + offset) != expected_top_value) {
		dev_err(gxp->dev, "TOP access from core %u failed!\n", core);
		return -EIO;
	}
	dev_notice(gxp->dev, "TOP access from core %u successful!\n", core);
#endif

	/* Stop bus performance monitors */
	gxp_bpm_stop(gxp, core);
	dev_notice(gxp->dev, "Core%u Instruction read transactions: 0x%x\n",
		   core, gxp_bpm_read_counter(gxp, core, INST_BPM_OFFSET));
	dev_notice(gxp->dev, "Core%u Data write transactions: 0x%x\n", core,
		   gxp_bpm_read_counter(gxp, core, DATA_BPM_OFFSET));

	return 0;
}

static void gxp_firmware_unload(struct gxp_dev *gxp, uint core)
{
	/* NO-OP for now. */
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

	mutex_lock(&gxp->dsp_firmware_lock);

	ret = scnprintf(buf, PAGE_SIZE, "%s\n",
			gxp->firmware_name ? gxp->firmware_name :
					     DSP_FIRMWARE_DEFAULT_PREFIX);

	mutex_unlock(&gxp->dsp_firmware_lock);

	return ret;
}

static ssize_t load_dsp_firmware_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct gxp_dev *gxp = dev_get_drvdata(dev);
	const struct firmware *firmwares[GXP_NUM_CORES];
	char *name_buf = NULL;
	int ret;
	int core;

	/*
	 * Lock the VD semaphore to ensure no core is executing the firmware
	 * while requesting new firmware.
	 */
	down_read(&gxp->vd_semaphore);

	if (gxp->firmware_running) {
		dev_warn(dev, "Cannot update firmware when any core is running\n");
		ret = -EBUSY;
		goto out;
	}

	name_buf = fw_name_from_attr_buf(buf);
	if (IS_ERR(name_buf)) {
		dev_err(gxp->dev, "Invalid firmware prefix requested: %s\n",
			buf);
		ret = PTR_ERR(name_buf);
		goto out;
	}

	mutex_lock(&gxp->dsp_firmware_lock);

	dev_notice(gxp->dev, "Requesting firmware be reloaded: %s\n", name_buf);

	ret = request_dsp_firmware(gxp, name_buf, firmwares);
	if (ret) {
		dev_err(gxp->dev,
			"Failed to request firmwares with names \"%sX\" (ret=%d)\n",
			name_buf, ret);
		goto err_request_firmware;
	}

	ret = gxp_firmware_authenticate(gxp, firmwares);
	if (ret)
		goto err_authenticate_firmware;

	for (core = 0; core < GXP_NUM_CORES; core++) {
		if (gxp->firmwares[core])
			release_firmware(gxp->firmwares[core]);
		gxp->firmwares[core] = firmwares[core];
	}

	kfree(gxp->firmware_name);
	gxp->firmware_name = name_buf;

	mutex_unlock(&gxp->dsp_firmware_lock);
out:
	up_read(&gxp->vd_semaphore);
	return count;

err_authenticate_firmware:
	for (core = 0; core < GXP_NUM_CORES; core++)
		release_firmware(firmwares[core]);
err_request_firmware:
	kfree(name_buf);
	mutex_unlock(&gxp->dsp_firmware_lock);
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

	/* Power on BLK_AUR to read the revision and processor ID registers */
	gxp_pm_blk_on(gxp);

	ver = gxp_read_32(gxp, GXP_REG_AURORA_REVISION);
	dev_notice(gxp->dev, "Aurora version: 0x%x\n", ver);

	for (core = 0; core < GXP_NUM_CORES; core++) {
		proc_id = gxp_read_32_core(gxp, core, GXP_REG_PROCESSOR_ID);
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

	gxp->firmware_running = 0;
	return 0;

out_fw_destroy:
	gxp_fw_destroy(gxp);
	return ret;
}

void gxp_fw_destroy(struct gxp_dev *gxp)
{
	uint core;

	device_remove_group(gxp->dev, &gxp_firmware_attr_group);

	for (core = 0; core < GXP_NUM_CORES; core++) {
		if (gxp->fwbufs[core].vaddr) {
			memunmap(gxp->fwbufs[core].vaddr);
			gxp->fwbufs[core].vaddr = NULL;
		}

		if (gxp->firmwares[core]) {
			release_firmware(gxp->firmwares[core]);
			gxp->firmwares[core] = NULL;
		}
	}

	kfree(gxp->firmware_name);
}

int gxp_firmware_request_if_needed(struct gxp_dev *gxp)
{
	int ret = 0;
	uint core;

	mutex_lock(&gxp->dsp_firmware_lock);

	if (gxp->is_firmware_requested)
		goto out;

	ret = request_dsp_firmware(gxp, DSP_FIRMWARE_DEFAULT_PREFIX,
				   gxp->firmwares);
	if (ret)
		goto out;

	ret = gxp_firmware_authenticate(gxp, gxp->firmwares);
	if (ret)
		goto err_authenticate_firmware;

	gxp->is_firmware_requested = true;

out:
	mutex_unlock(&gxp->dsp_firmware_lock);
	return ret;

err_authenticate_firmware:
	for (core = 0; core < GXP_NUM_CORES; core++) {
		release_firmware(gxp->firmwares[core]);
		gxp->firmwares[core] = NULL;
	}
	mutex_unlock(&gxp->dsp_firmware_lock);
	return ret;
}

static int gxp_firmware_setup(struct gxp_dev *gxp, uint core)
{
	int ret = 0;

	if (gxp->firmware_running & BIT(core)) {
		dev_err(gxp->dev, "Firmware is already running on core %u\n",
			core);
		return -EBUSY;
	}

	ret = gxp_firmware_load(gxp, core);
	if (ret) {
		dev_err(gxp->dev, "Failed to load firmware on core %u\n", core);
		return ret;
	}

	/* Mark this as a cold boot */
	gxp_firmware_set_boot_mode(gxp, core, GXP_BOOT_MODE_REQUEST_COLD_BOOT);

	ret = gxp_firmware_setup_hw_after_block_off(gxp, core,
						    /*verbose=*/true);
	if (ret) {
		dev_err(gxp->dev, "Failed to power up core %u\n", core);
		gxp_firmware_unload(gxp, core);
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
				       uint virt_core, uint core)
{
	int ret;
	struct work_struct *work;

	ret = gxp_firmware_handshake(gxp, core);
	if (ret) {
		dev_err(gxp->dev, "Firmware handshake failed on core %u\n",
			core);
		gxp_pm_core_off(gxp, core);
		goto out_firmware_unload;
	}

	/* Initialize mailbox */
	gxp->mailbox_mgr->mailboxes[core] =
		gxp_mailbox_alloc(gxp->mailbox_mgr, vd, virt_core, core);
	if (IS_ERR(gxp->mailbox_mgr->mailboxes[core])) {
		dev_err(gxp->dev,
			"Unable to allocate mailbox (core=%u, ret=%ld)\n", core,
			PTR_ERR(gxp->mailbox_mgr->mailboxes[core]));
		ret = PTR_ERR(gxp->mailbox_mgr->mailboxes[core]);
		gxp->mailbox_mgr->mailboxes[core] = NULL;
		goto out_firmware_unload;
	}

	work = gxp_debug_dump_get_notification_handler(gxp, core);
	if (work)
		gxp_notification_register_handler(
			gxp, core, HOST_NOTIF_DEBUG_DUMP_READY, work);

	work = gxp_telemetry_get_notification_handler(gxp, core);
	if (work)
		gxp_notification_register_handler(
			gxp, core, HOST_NOTIF_TELEMETRY_STATUS, work);

	gxp->firmware_running |= BIT(core);

	return ret;

out_firmware_unload:
	gxp_firmware_unload(gxp, core);
	return ret;
}

static void gxp_firmware_stop_core(struct gxp_dev *gxp,
				   struct gxp_virtual_device *vd,
				   uint virt_core, uint core)
{
	if (!(gxp->firmware_running & BIT(core)))
		dev_err(gxp->dev, "Firmware is not running on core %u\n", core);

	gxp->firmware_running &= ~BIT(core);

	gxp_notification_unregister_handler(gxp, core,
					    HOST_NOTIF_DEBUG_DUMP_READY);
	gxp_notification_unregister_handler(gxp, core,
					    HOST_NOTIF_TELEMETRY_STATUS);

	gxp_mailbox_release(gxp->mailbox_mgr, vd, virt_core,
			    gxp->mailbox_mgr->mailboxes[core]);
	dev_notice(gxp->dev, "Mailbox %u released\n", core);

	if (vd->state == GXP_VD_RUNNING)
		gxp_pm_core_off(gxp, core);
	gxp_firmware_unload(gxp, core);
}

int gxp_firmware_run(struct gxp_dev *gxp, struct gxp_virtual_device *vd,
		     uint core_list)
{
	int ret;
	uint core, virt_core;
	uint failed_cores = 0;
	int failed_ret;

	for (core = 0; core < GXP_NUM_CORES; core++) {
		if (core_list & BIT(core)) {
			ret = gxp_firmware_setup(gxp, core);
			if (ret) {
				failed_cores |= BIT(core);
				failed_ret = ret;
				dev_err(gxp->dev, "Failed to run firmware on core %u\n",
					core);
			}
		}
	}
	if (failed_cores != 0) {
		/*
		 * Shut down the cores which call `gxp_firmware_setup`
		 * successfully
		 */
		for (core = 0; core < GXP_NUM_CORES; core++) {
			if (core_list & BIT(core)) {
				if (!(failed_cores & BIT(core))) {
					gxp_pm_core_off(gxp, core);
					gxp_firmware_unload(gxp, core);
				}
			}
		}
		return failed_ret;
	}
#if IS_ENABLED(CONFIG_GXP_GEM5)
	/*
	 * GEM5 starts firmware after LPM is programmed, so we need to call
	 * gxp_doorbell_enable_for_core here to set GXP_REG_COMMON_INT_MASK_0
	 * first to enable the firmware handshakes.
	 */
	for (core = 0; core < GXP_NUM_CORES; core++) {
		if (!(core_list & BIT(core)))
			continue;
		gxp_doorbell_enable_for_core(gxp, CORE_WAKEUP_DOORBELL(core),
					     core);
	}
#endif
	/* Switch clock mux to the normal state to guarantee LPM works */
	gxp_pm_force_clkmux_normal(gxp);
	gxp_firmware_wakeup_cores(gxp, core_list);
	virt_core = 0;
	for (core = 0; core < GXP_NUM_CORES; core++) {
		if (core_list & BIT(core)) {
			ret = gxp_firmware_finish_startup(gxp, vd, virt_core,
							  core);
			if (ret) {
				failed_cores |= BIT(core);
				dev_err(gxp->dev,
					"Failed to run firmware on core %u\n",
					core);
			}
			virt_core++;
		}
	}

	if (failed_cores != 0) {
		virt_core = 0;
		for (core = 0; core < GXP_NUM_CORES; core++) {
			if (core_list & BIT(core)) {
				if (!(failed_cores & BIT(core))) {
					gxp_firmware_stop_core(gxp, vd,
							       virt_core, core);
				}
				virt_core++;
			}
		}
	}
	/* Check if we need to set clock mux to low state as requested */
	gxp_pm_resume_clkmux(gxp);

	return ret;
}

int gxp_firmware_setup_hw_after_block_off(struct gxp_dev *gxp, uint core,
					  bool verbose)
{
	gxp_program_reset_vector(gxp, core, verbose);
	return gxp_pm_core_on(gxp, core, verbose);
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

void gxp_firmware_set_boot_mode(struct gxp_dev *gxp, uint core, u32 mode)
{
	void __iomem *boot_mode_addr;

	/* Callers shouldn't call the function under this condition. */
	if (!gxp->fwbufs[core].vaddr)
		return;

	boot_mode_addr = gxp->fwbufs[core].vaddr + AURORA_SCRATCHPAD_OFF +
			 SCRATCHPAD_MSG_OFFSET(MSG_BOOT_MODE);

	writel(mode, boot_mode_addr);
}

u32 gxp_firmware_get_boot_mode(struct gxp_dev *gxp, uint core)
{
	void __iomem *boot_mode_addr;

	/* Callers shouldn't call the function under this condition. */
	if (!gxp->fwbufs[core].vaddr)
		return 0;

	boot_mode_addr = gxp->fwbufs[core].vaddr + AURORA_SCRATCHPAD_OFF +
			 SCRATCHPAD_MSG_OFFSET(MSG_BOOT_MODE);

	return readl(boot_mode_addr);
}
