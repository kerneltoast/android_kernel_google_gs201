// SPDX-License-Identifier: GPL-2.0-only
/*
 * Subsystem-coredump driver
 *
 * Copyright 2019-2020 Google LLC
 */
#include <linux/kernel.h>
#include <linux/binfmts.h>
#include <linux/elf.h>
#include <linux/elfcore.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/kobject.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/cdev.h>
#include <linux/time.h>

#include <linux/platform_data/sscoredump.h>
#include <uapi/misc/crashinfo.h>

#define SSCD_MAX_DEVS		128
#define SSCD_REPORT_TIMEOUT	1000 /* default timeout, ms */

#define SSCD_ELFHDR_MASK	(SSCD_FLAGS_ELFARM32HDR | \
				 SSCD_FLAGS_ELFARM64HDR)
#define ELF_STRTAB_SIZE		256

enum sscd_report_level {
	REPORT_CRASHINFO_ONLY = 0,
	REPORT_ALL
};

static const char shd_string_table[] = {
	"\0"
	".shstrtab\0"  /* 1-10 */
	".crashinfo\0" /* 11-21 */
};

/**
 * struct sscd_device - internal state container for sscd devices.
 * This state container holds most of the runtime variable data
 * for a sscd device.
 */
struct sscd_device {
	/**
	 * device related
	 * @node: common device list
	 * @rx_lock: protects access to structure
	 * @enabled: coredump enabled/disabled
	 * @opened: tracks active client
	 */
	struct device            dev;
	struct cdev              chrdev;
	struct device            *parent_dev;
	struct list_head         node;
	struct mutex             rx_lock; /* access to structure */

	bool                     enabled;
	atomic_t                 opened;
	u32                      read_timeout;
	wait_queue_head_t        read_wait_q;
	struct completion        read_completion;

	/**
	 * subsystem report data (includes extra segments for internal data)
	 * @report_active: active crash data
	 * @segs: crash segment data
	 * @nsegs: number of segments
	 * @read_offset: read offset within crash data
	 */
	atomic_t                 report_active;
	u32                      report_flags;
	struct crashinfo_img_hdr crash_hdr;
	struct sscd_segment      *segs;
	u16                      nsegs;
	loff_t                   read_offset;

	/**
	 * stats
	 */
	time64_t                 report_time;
	u64                      report_count; /* number of reports */
	u64                      read_count; /* number of read reports */
};

/* global class data */
static int sscd_major;
static u8 sscd_enabled = 1; /* default value */
static u8 sscd_level = REPORT_CRASHINFO_ONLY;
static LIST_HEAD(sscd_devs_list);
static DEFINE_IDA(sscd_ida);
static DEFINE_MUTEX(sscd_mutex); /* protects access to sscd dev list */


static void report_read_completed(struct sscd_device *sdev)
{
	if (atomic_cmpxchg(&sdev->report_active, 1, 0) == 1)
		complete(&sdev->read_completion);
}

static int sscd_dev_open(struct inode *inode, struct file *filp)
{
	struct sscd_device *sdev =
		container_of(inode->i_cdev, struct sscd_device, chrdev);

	/* support only single client */
	if (atomic_cmpxchg(&sdev->opened, 0, 1) != 0)
		return -EBUSY;

	filp->private_data = sdev;
	get_device(&sdev->dev);

	return 0;
}

static int sscd_dev_release(struct inode *nodp, struct file *filp)
{
	struct sscd_device *sdev = filp->private_data;

	atomic_set(&sdev->opened, 0);
	report_read_completed(sdev);
	put_device(&sdev->dev);

	return 0;
}

static unsigned int sscd_dev_poll(struct file *filp,
				  struct poll_table_struct *wait)
{
	struct sscd_device *sdev = filp->private_data;
	unsigned int mask = 0;

	poll_wait(filp, &sdev->read_wait_q, wait);
	if (atomic_read(&sdev->report_active) == 1)
		mask = POLLIN | POLLRDNORM;

	return mask;
}

static ssize_t sscd_dev_read(struct file *filp, char __user *ubuf, size_t count,
			     loff_t *f_pos)
{
	struct sscd_device *sdev = filp->private_data;
	size_t copied = 0;
	loff_t pos = *f_pos;
	u16 i;

	if (!count)
		return 0;

	if (pos < 0)
		return -EINVAL;

	if (!atomic_read(&sdev->report_active) && (filp->f_flags & O_NONBLOCK))
		return -EAGAIN;

	if (wait_event_interruptible(sdev->read_wait_q,
				     atomic_read(&sdev->report_active)))
		return -ERESTARTSYS;

	mutex_lock(&sdev->rx_lock);

	/*
	 * potentially, reader might not consume data fast enough causing
	 * timeout condition to occur. In such cases, we might end up reading
	 * data for new report.
	 * Enforce EOF condition if such condition detected.
	 */
	if (pos != sdev->read_offset) {
		dev_info(&sdev->dev, "Invalid offset(%lld/%lld) in stream, abort",
			 pos, sdev->read_offset);
		goto out;
	}

	/* copy the data */
	for (i = 0; i < sdev->nsegs; i++) {
		if (pos < (loff_t)sdev->segs[i].size && sdev->segs[i].addr) {
			ssize_t len = simple_read_from_buffer(ubuf, count, &pos,
					sdev->segs[i].addr, sdev->segs[i].size);
			if (len < 0)
				break;

			count -= len;
			copied += len;
			ubuf += len;
			dev_dbg(&sdev->dev, "seg[%hu] read %zd ", i, len);
		}
		pos -= sdev->segs[i].size;
		if (pos < 0)
			break;
	}

out:
	/* mark request as done only in case of error or EOF condition */
	if (copied == 0) {
		*f_pos = 0; /* reset position for new report */
		report_read_completed(sdev);
	} else {
		*f_pos += copied;
	}
	sdev->read_offset = *f_pos;

	mutex_unlock(&sdev->rx_lock);

	return copied;
}

static ssize_t sscd_dev_write(struct file *filp, const char __user *ubuf,
			      size_t count, loff_t *f_pos)
{
	struct sscd_device *sdev = filp->private_data;

	if (!count)
		return 0;

	/* report success if no active reports */
	if (!atomic_read(&sdev->report_active))
		return count;

	mutex_lock(&sdev->rx_lock);
	report_read_completed(sdev);
	*f_pos = 0;
	mutex_unlock(&sdev->rx_lock);

	return count;
}

static const struct file_operations sscd_dev_fops = {
	.owner = THIS_MODULE,
	.open = sscd_dev_open,
	.poll = sscd_dev_poll,
	.llseek = noop_llseek,
	.read = sscd_dev_read,
	.write = sscd_dev_write,
	.release = sscd_dev_release,
};

#define FILL_ELF_PHDR(elf_type)						\
static void fill_elf##elf_type##_phdr(void *elf_addr, u32 phdr_offset,	\
			   uint p_offset, struct sscd_segment *segs,	\
			   u16 nsegs)					\
{									\
	struct elf##elf_type##_phdr *phdr = elf_addr + phdr_offset;	\
	u16 i;								\
									\
	for (i = 0; i < nsegs; i++) {					\
		phdr[i].p_type = PT_LOAD;				\
		phdr[i].p_paddr = (Elf##elf_type##_Addr)(unsigned long) segs[i].paddr;	\
		phdr[i].p_vaddr = (Elf##elf_type##_Addr)(unsigned long) segs[i].vaddr;	\
		phdr[i].p_memsz = segs[i].size;				\
		phdr[i].p_filesz = segs[i].size;			\
		phdr[i].p_offset = p_offset;				\
		phdr[i].p_flags = PF_R | PF_W | PF_X;			\
		p_offset += phdr[i].p_memsz;				\
	}								\
}

#define FILL_ELF_SHDR(elf_type)						\
static void fill_elf##elf_type##_shdr(void *elf_addr, u32 shdr_offset,	\
			    u32 sh_offset, u16 nsegs,			\
			    const char *crash_info)			\
{									\
	struct elf##elf_type##_shdr *shdr = elf_addr + shdr_offset;	\
									\
	/* section 0 shall be 0 */					\
	shdr++;								\
	/* crashinfo section */						\
	shdr->sh_type = SHT_NOTE;					\
	shdr->sh_offset = sh_offset;					\
	shdr->sh_size = CRASHINFO_REASON_SIZE;				\
	shdr->sh_name = 11;						\
	strncpy((char *)(elf_addr + shdr->sh_offset),			\
		crash_info ? crash_info : "N/A", shdr->sh_size);	\
	sh_offset += shdr->sh_size;					\
	shdr++;								\
	/* string table section */					\
	shdr->sh_type = SHT_STRTAB;					\
	shdr->sh_offset = sh_offset;					\
	shdr->sh_size = ELF_STRTAB_SIZE;				\
	shdr->sh_name = 1;						\
	memcpy(elf_addr + shdr->sh_offset, shd_string_table,		\
		sizeof(shd_string_table));				\
}

#define FILL_ELF_HDR(elf_type)						\
static void fill_elf##elf_type##_hdr(int mach, void *addr,		\
			   u32 phoff, u16 phnum,			\
			   u32 shoff, u16 shnum)			\
{									\
	struct elf##elf_type##_hdr *hdr = addr;				\
									\
	memcpy(hdr->e_ident, ELFMAG, SELFMAG);				\
	hdr->e_ident[EI_CLASS] = ELFCLASS##elf_type;			\
	hdr->e_ident[EI_DATA] = ELFDATA2LSB;				\
	hdr->e_ident[EI_VERSION] = EV_CURRENT;				\
	hdr->e_ident[EI_OSABI] = ELFOSABI_NONE;				\
	hdr->e_type = ET_CORE;						\
	hdr->e_machine = mach;						\
	hdr->e_version = EV_CURRENT;					\
	hdr->e_phoff = phoff;						\
	hdr->e_ehsize = sizeof(*hdr);					\
	hdr->e_phentsize = sizeof(struct elf##elf_type##_phdr);		\
	hdr->e_phnum = phnum;						\
	hdr->e_shentsize = sizeof(struct elf##elf_type##_shdr);		\
	hdr->e_shoff = shoff;						\
	hdr->e_shnum = shnum;						\
	hdr->e_shstrndx = shnum - 1;					\
}

FILL_ELF_PHDR(32)
FILL_ELF_PHDR(64)
FILL_ELF_SHDR(32)
FILL_ELF_SHDR(64)
FILL_ELF_HDR(32)
FILL_ELF_HDR(64)

static int fill_elf(struct sscd_segment *elf, struct sscd_segment *segs,
		    u16 nsegs, u64 flags, const char *crash_info)
{
	u32 hdr_size, phdr_size, shdr_size;
	u32 phdr_off, shdr_off, sh_off;

	/*
	 * elf buffer [ memory layout]:
	 *	* elf header
	 *	* nsegs of program header entries
	 *	* 3 section headers
	 *	* 3 sections
	 *		[0] empty section
	 *		[1] .shstring table section
	 *		[2] .crashinfo SH_NOTE section
	 *		     (if crash_info == NULL : supply "N/A")
	 */
	if (flags & SSCD_FLAGS_ELFARM64HDR) {
		hdr_size = sizeof(struct elf64_hdr);
		phdr_size = sizeof(struct elf64_phdr) * nsegs;
		shdr_size = sizeof(struct elf64_shdr) * 3;
	} else {
		hdr_size = sizeof(struct elf32_hdr);
		phdr_size = sizeof(struct elf32_phdr) * nsegs;
		shdr_size = sizeof(struct elf32_shdr) * 3;
	}

	elf->size = hdr_size + phdr_size + shdr_size;         /* headers */
	elf->size += ELF_STRTAB_SIZE + CRASHINFO_REASON_SIZE; /* sections */
	elf->addr = kzalloc(elf->size, GFP_KERNEL);
	if (!elf->addr)
		return -ENOMEM;

	phdr_off = hdr_size;
	shdr_off = phdr_off + phdr_size;
	sh_off = shdr_off + shdr_size;

	if (flags & SSCD_FLAGS_ELFARM64HDR) {
		fill_elf64_hdr(EM_AARCH64, elf->addr, phdr_off, nsegs,
			       shdr_off, 3);
		fill_elf64_phdr(elf->addr, phdr_off, elf->size, segs, nsegs);
		fill_elf64_shdr(elf->addr, shdr_off, sh_off, 3, crash_info);
	} else {
		fill_elf32_hdr(EM_ARM, elf->addr, phdr_off, nsegs,
			       shdr_off, 3);
		fill_elf32_phdr(elf->addr, phdr_off, elf->size, segs, nsegs);
		fill_elf32_shdr(elf->addr, shdr_off, sh_off, 3, crash_info);
	}

	return 0;
}

static void free_report(struct sscd_device *sdev)
{
	if (sdev->report_flags & SSCD_ELFHDR_MASK && sdev->nsegs > 1)
		kfree(sdev->segs[1].addr);

	kfree(sdev->segs);
	sdev->segs = NULL;
	sdev->nsegs = 0;
}

/**
 * create report
 *
 * In addition to reported nsegs, we allocate extra up to 2 segments:
 *  +1 holds crashinfo header
 *  +1.(ELF32/64 flag set): holds ELF data:
 *       ELF header, ELF sections (hdr/data),
 *       program segments (hdr)
 * Segments order in memory:
 *  [0]        crashinfo header
 *  [1]        ELF segment (optional)
 *  [2..nsegs] crash data
 */
static int create_report(struct sscd_device *sdev, struct sscd_segment *segs,
			 u16 nsegs, u64 flags, const char *crash_info)
{
	u32 count = nsegs + 1; /* extra segment for crashinfo header */
	int offset = 1;        /* user data starts from offset 1 by default */
	int rc = 0;

	/*
	 * limit report to crash info if required
	 */
	if (sscd_level == REPORT_CRASHINFO_ONLY || !nsegs) {
		count = 1;
	} else if (flags & SSCD_ELFHDR_MASK) {
		/* supply additional segment in case of ELF */
		count++;
		offset++;
	}

	sdev->report_flags = flags;
	sdev->nsegs = count;
	sdev->crash_hdr.coredump_size = 0;
	sdev->read_offset = 0;
	sdev->segs = kcalloc(count, sizeof(*segs), GFP_KERNEL);
	if (!sdev->segs) {
		rc = -ENOMEM;
		goto err;
	}
	dev_dbg(&sdev->dev, "segs: reported %hu, report %hu", nsegs, sdev->nsegs);

	/*
	 * represent crashinfo as additional segment
	 */
	sdev->segs[0].addr = &sdev->crash_hdr;
	sdev->segs[0].size = sizeof(sdev->crash_hdr);

	/*
	 * coredump info (user segments + [elf_data segment] )
	 */
	if (sscd_level > REPORT_CRASHINFO_ONLY && nsegs) {
		u32 i;

		/* copy supplied segments info */
		memcpy(&sdev->segs[offset], segs, sizeof(*segs) * nsegs);

		/* fill elf info */
		if (flags & SSCD_ELFHDR_MASK) {
			rc = fill_elf(&sdev->segs[1], &sdev->segs[offset],
				      nsegs, flags, crash_info);
			if (rc)
				goto err;
		}

		/* update coredump size */
		for (i = 1; i < count; i++)
			sdev->crash_hdr.coredump_size += sdev->segs[i].size;
	}

	return 0;

err:
	free_report(sdev);

	return rc;
}

/**
 * sscd_report report a coredump/crashinfo.
 *
 * A zero is returned on success and a negative errno code for
 * failure.
 *
 * Should not be called from interrupt or atomic context.
 */
static int sscd_report(struct platform_device *pdev, struct sscd_segment *segs,
		       u16 nsegs, u64 flags, const char *crash_info)
{
	struct sscd_device *sdev = platform_get_drvdata(pdev);
	int rc = 0;

	dev_info(&sdev->dev, "crash: %s", crash_info);

	mutex_lock(&sdev->rx_lock);

	get_device(&sdev->dev);

	/* check if enabled */
	if (!sscd_enabled || !sdev->enabled)
		goto out;

	/* check if active report already present */
	if (atomic_read(&sdev->report_active)) {
		rc = -EBUSY;
		goto out;
	}

	/* preserve crash info */
	strncpy(sdev->crash_hdr.crashinfo, crash_info,
		sizeof(sdev->crash_hdr.crashinfo));

	/* check if active reader exists */
	if (!atomic_read(&sdev->opened)) {
		rc = -EAGAIN;
		goto out;
	}

	rc = create_report(sdev, segs, nsegs, flags, crash_info);
	if (rc) {
		dev_dbg(&sdev->dev, "Unable to prepare coredump");
		goto out;
	}

	/*
	 * wake pending read
	 */
	atomic_set(&sdev->report_active, 1);
	reinit_completion(&sdev->read_completion);
	wake_up(&sdev->read_wait_q);
	mutex_unlock(&sdev->rx_lock);

	/*
	 * wait until data is read or timeout
	 */
	rc = wait_for_completion_timeout(&sdev->read_completion,
					 msecs_to_jiffies(sdev->read_timeout));

	mutex_lock(&sdev->rx_lock);
	if (!rc) {
		dev_err(&sdev->dev, "timeout on completion");
		rc = -EPIPE;
	} else {
		rc = 0; /* reset error code */
		sdev->read_count++;
		dev_dbg(&sdev->dev, "wait_for_completion done (%d)", rc);
	}

	atomic_set(&sdev->report_active, 0);
	free_report(sdev);

out:
	sdev->report_count++;
	sdev->report_time = ktime_get_real_seconds(); /* capture report time */
	put_device(&sdev->dev);

	mutex_unlock(&sdev->rx_lock);

	return rc;
}

static ssize_t crash_info_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct sscd_device *sdev = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%s\n", sdev->crash_hdr.crashinfo);
}

static ssize_t sdev_enabled_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct sscd_device *sdev = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", (int)sdev->enabled);
}

static ssize_t sdev_enabled_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	int status;
	long value;
	struct sscd_device *sdev = dev_get_drvdata(dev);

	status = kstrtol(buf, 0, &value);
	if (!status) {
		mutex_lock(&sdev->rx_lock);
		sdev->enabled = !!value;
		mutex_unlock(&sdev->rx_lock);
	}
	return status ? status : count;
}

static ssize_t read_count_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct sscd_device *sdev = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%llu\n", sdev->read_count);
}

static ssize_t report_count_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct sscd_device *sdev = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%llu\n", sdev->report_count);
}

static ssize_t report_time_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct sscd_device *sdev = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%lld\n", sdev->report_time);
}

static ssize_t timeout_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct sscd_device *sdev = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%u\n", sdev->read_timeout);
}

static ssize_t timeout_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	int status;
	u32 value;
	struct sscd_device *sdev = dev_get_drvdata(dev);

	status = kstrtou32(buf, 0, &value);
	if (!status) {
		mutex_lock(&sdev->rx_lock);
		sdev->read_timeout = value;
		mutex_unlock(&sdev->rx_lock);
	}
	return status ? status : count;
}

static DEVICE_ATTR_RO(crash_info);
static DEVICE_ATTR(enabled, 0644, sdev_enabled_show, sdev_enabled_store);
static DEVICE_ATTR_RO(read_count);
static DEVICE_ATTR_RO(report_count);
static DEVICE_ATTR_RO(report_time);
static DEVICE_ATTR_RW(timeout);

static struct attribute *sscd_dev_attrs[] = {
	&dev_attr_crash_info.attr,
	&dev_attr_enabled.attr,
	&dev_attr_read_count.attr,
	&dev_attr_report_count.attr,
	&dev_attr_report_time.attr,
	&dev_attr_timeout.attr,
	NULL
};

ATTRIBUTE_GROUPS(sscd_dev);

/**
 * class attributes
 */
static ssize_t enabled_show(struct class *class, struct class_attribute *attr,
			    char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", sscd_enabled);
}

static ssize_t enabled_store(struct class *class, struct class_attribute *attr,
			     const char *buf, size_t count)
{
	int status;
	u8 value;

	status = kstrtou8(buf, 0, &value);
	if (!status)
		sscd_enabled = !!value;

	return status ? status : count;
}

static ssize_t level_show(struct class *class, struct class_attribute *attr,
			  char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", sscd_level);
}

static ssize_t level_store(struct class *class, struct class_attribute *attr,
			   const char *buf, size_t count)
{
	int status;
	u8 value;

	status = kstrtou8(buf, 0, &value);
	if (!status)
		sscd_level = !!value;

	return status ? status : count;
}

static ssize_t clients_show(struct class *class, struct class_attribute *attr,
			    char *buf)
{
	int n = 0;
	struct sscd_device *sdev;

	mutex_lock(&sscd_mutex);
	list_for_each_entry(sdev, &sscd_devs_list, node) {
		n += scnprintf(buf + n, PAGE_SIZE - n,
			       "%s\n", dev_name(&sdev->dev));
	}
	mutex_unlock(&sscd_mutex);

	return n;
}

static CLASS_ATTR_RO(clients);
static CLASS_ATTR_RW(enabled);
static CLASS_ATTR_RW(level);

static struct attribute *class_sscd_attrs[] = {
	&class_attr_clients.attr,
	&class_attr_enabled.attr,
	&class_attr_level.attr,
	NULL,
};

ATTRIBUTE_GROUPS(class_sscd);

static struct class sscd_class = {
	.owner		= THIS_MODULE,
	.name		= SSCD_NAME,
	.class_groups	= class_sscd_groups,
};

/**
 * sscoredump_release
 *
 * Release resources back to system.
 */
static void sscoredump_release(struct device *dev)
{
	struct sscd_device *sdev = container_of(dev, struct sscd_device, dev);

	mutex_destroy(&sdev->rx_lock);
	free_report(sdev);
	kfree(sdev);
}

/**
 * sscoredump_probe
 *
 * A zero is returned on success and a negative errno code for
 * failure.
 */
static int sscoredump_probe(struct platform_device *pdev)
{
	int minor;
	int ret;
	struct sscd_device *sdev;
	struct sscd_platform_data *pdata = dev_get_platdata(&pdev->dev);

	sdev = kzalloc(sizeof(*sdev), GFP_KERNEL);
	if (!sdev)
		return -ENOMEM;

	/* init data */
	init_waitqueue_head(&sdev->read_wait_q);
	init_completion(&sdev->read_completion);
	sdev->read_timeout = SSCD_REPORT_TIMEOUT;
	sdev->enabled = sscd_enabled;
	mutex_init(&sdev->rx_lock);

	/* fill crashinfo header data (static part) */
	memcpy(sdev->crash_hdr.magic, CRASHINFO_MAGIC, CRASHINFO_MAGIC_SIZE);
	sdev->crash_hdr.header_size = sizeof(struct crashinfo_img_hdr);

	/* initialize a new device of our class */
	minor = ida_simple_get(&sscd_ida, 0, SSCD_MAX_DEVS, GFP_KERNEL);
	if (minor < 0) {
		pr_err("unable to reserve dev_minor\n");
		goto fail1;
	}

	cdev_init(&sdev->chrdev, &sscd_dev_fops);
	sdev->chrdev.owner = THIS_MODULE;

	device_initialize(&sdev->dev);
	sdev->dev.devt = MKDEV(sscd_major, minor);
	sdev->dev.class = &sscd_class;
	sdev->dev.parent = &pdev->dev;
	sdev->dev.groups = sscd_dev_groups;
	sdev->dev.release = sscoredump_release;
	dev_set_drvdata(&sdev->dev, sdev);
	dev_set_name(&sdev->dev, "sscd_%s", dev_name(&pdev->dev));

	ret = cdev_device_add(&sdev->chrdev, &sdev->dev);
	if (ret < 0) {
		pr_err("failed to add cdev (%d)", ret);
		goto fail2;
	}

	/* add to list */
	mutex_lock(&sscd_mutex);
	list_add_tail(&sdev->node, &sscd_devs_list);
	platform_set_drvdata(pdev, sdev);

	/*
	 * function pointer for client
	 * TODO: do we have better way to pass function to client.
	 */
	pdata->sscd_report = sscd_report;
	mutex_unlock(&sscd_mutex);

	return 0;

fail2:
	ida_simple_remove(&sscd_ida, minor);
fail1:
	kfree(sdev);

	return -EAGAIN;
}

static int sscoredump_remove(struct platform_device *pdev)
{
	struct sscd_device *sdev = platform_get_drvdata(pdev);
	int minor;

	dev_info(&sdev->dev, "remove");

	if (!sdev)
		return -ENODEV;

	if (atomic_read(&sdev->report_active))
		return -EBUSY;

	minor = MINOR(sdev->chrdev.dev);

	mutex_lock(&sscd_mutex);
	list_del(&sdev->node);
	mutex_unlock(&sscd_mutex);

	cdev_device_del(&sdev->chrdev, &sdev->dev);
	ida_simple_remove(&sscd_ida, minor);

	put_device(&sdev->dev);

	return 0;
}

static struct platform_driver sscoredump_driver = {
	.probe = sscoredump_probe,
	.remove = sscoredump_remove,
	.driver = {
		.name = SSCD_NAME,
		.pm = NULL,
	}
};

static int sscoredump_init(void)
{
	int ret;
	dev_t dev;

	ret = class_register(&sscd_class);
	if (ret) {
		pr_err("unable to register sscd class\n");
		return ret;
	}

	ret = alloc_chrdev_region(&dev, 0, SSCD_MAX_DEVS, SSCD_NAME);
	sscd_major = MAJOR(dev);
	if (ret < 0) {
		pr_err("alloc_chrdev_region() failed for sscd\n");
		class_unregister(&sscd_class);
		return ret;
	}

	return platform_driver_register(&sscoredump_driver);
}

static void sscoredump_exit(void)
{
	platform_driver_unregister(&sscoredump_driver);
	unregister_chrdev_region(MKDEV(sscd_major, 0), SSCD_MAX_DEVS);
	class_unregister(&sscd_class);
}

module_init(sscoredump_init);
module_exit(sscoredump_exit);

MODULE_DESCRIPTION("Subsystem coredump driver");
MODULE_AUTHOR("Oleg Matcovschi");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.2a");
