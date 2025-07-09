// SPDX-License-Identifier: GPL-2.0-only
/*
 * Gadget Driver for ETR_MIU
 *
 * Copyright (C) 2019 Google LLC
 */

#include <linux/configfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/poll.h>
#include <linux/printk.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/usb/composite.h>
#include <linux/wait.h>
#include "../../dwc3/gadget.h"

#define MAX_INST_NAME_LEN 40

#define ETR_MIU_ON 0x20
#define ETR_MIU_ON__IDLE 0x10

#define ETR_MEM_CTRL_RD 0x38
#define ETR_MEM_CTRL_RD__DRAIN 0x1

#define ETR_DESC_ROM_RD(n) (0x80 + (n) * 0x4)

/* Values based on ETR_MIU datasheet */
#define ETR_MIU_TRANSFER_SIZE (64 * 1024)
#define ETR_MIU_DATA_ADDR 0x1800000000
#define ETR_MIU_TRB_ADDR 0x1880000000

/* Values based on usb dwc3 */
#define LINK_TRB_TRANSFER_SIZE 0x0
/* TRBCTL_LINK | HWO */
#define LINK_TRB_CTRL 0x81
/* TRBCTL_NORMAL | CHN | HWO */
#define DATA_TRB_CTRL 0x15

/* Limit drain to 1s */
#define MAX_DRAIN 1000000

/* Values based on usb descriptor allocation */
#define ETR_MIU_CLASS 0xff
#define ETR_MIU_SUBCLASS 0x56
#define ETR_MIU_PROTOCOL 0x1

static void __iomem *base;
static u64 tmc_buf_addr;
static u32 tmc_buf_size;

struct etr_miu_dev {
	struct usb_function function;
	struct usb_composite_dev *cdev;

	struct usb_ep *ep_in;

	int online;
	int alt;

	struct work_struct etr_off_work;
};

struct etr_miu_instance {
	const char *name;
	struct usb_function_instance func_inst;
	struct etr_miu_dev *dev;
};

static struct usb_interface_descriptor etr_miu_interface_alt0_desc = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	/* .bInterfaceNumber       = dynamic, */
	.bAlternateSetting = 0,
	.bNumEndpoints = 0,
	.bInterfaceClass = ETR_MIU_CLASS,
	.bInterfaceSubClass = ETR_MIU_SUBCLASS,
	.bInterfaceProtocol = ETR_MIU_PROTOCOL,
};

static struct usb_interface_descriptor etr_miu_interface_alt1_desc = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	/* .bInterfaceNumber       = dynamic, */
	.bAlternateSetting = 1,
	.bNumEndpoints = 1,
	.bInterfaceClass = ETR_MIU_CLASS,
	.bInterfaceSubClass = ETR_MIU_SUBCLASS,
	.bInterfaceProtocol = ETR_MIU_PROTOCOL,
};

static struct usb_endpoint_descriptor etr_miu_superspeedplus_in_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = USB_DIR_IN,
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize = cpu_to_le16(1024),
};

static struct usb_ss_ep_comp_descriptor etr_miu_superspeedplus_bulk_comp_desc = {
	.bLength = sizeof(etr_miu_superspeedplus_bulk_comp_desc),
	.bDescriptorType = USB_DT_SS_ENDPOINT_COMP,
};

static struct usb_endpoint_descriptor etr_miu_superspeed_in_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = USB_DIR_IN,
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize = cpu_to_le16(1024),
};

static struct usb_ss_ep_comp_descriptor etr_miu_superspeed_bulk_comp_desc = {
	.bLength = sizeof(etr_miu_superspeed_bulk_comp_desc),
	.bDescriptorType = USB_DT_SS_ENDPOINT_COMP,
};

static struct usb_endpoint_descriptor etr_miu_highspeed_in_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = USB_DIR_IN,
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize = cpu_to_le16(512),
};

static struct usb_endpoint_descriptor etr_miu_fullspeed_in_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = USB_DIR_IN,
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *fs_etr_miu_descs[] = {
	(struct usb_descriptor_header *)&etr_miu_interface_alt0_desc,
	(struct usb_descriptor_header *)&etr_miu_interface_alt1_desc,
	(struct usb_descriptor_header *)&etr_miu_fullspeed_in_desc,
	NULL,
};

static struct usb_descriptor_header *hs_etr_miu_descs[] = {
	(struct usb_descriptor_header *)&etr_miu_interface_alt0_desc,
	(struct usb_descriptor_header *)&etr_miu_interface_alt1_desc,
	(struct usb_descriptor_header *)&etr_miu_highspeed_in_desc,
	NULL,
};

static struct usb_descriptor_header *ss_etr_miu_descs[] = {
	(struct usb_descriptor_header *)&etr_miu_interface_alt0_desc,
	(struct usb_descriptor_header *)&etr_miu_interface_alt1_desc,
	(struct usb_descriptor_header *)&etr_miu_superspeed_in_desc,
	(struct usb_descriptor_header *)&etr_miu_superspeed_bulk_comp_desc,
	NULL,
};

static struct usb_descriptor_header *ssp_etr_miu_descs[] = {
	(struct usb_descriptor_header *)&etr_miu_interface_alt0_desc,
	(struct usb_descriptor_header *)&etr_miu_interface_alt1_desc,
	(struct usb_descriptor_header *)&etr_miu_superspeedplus_in_desc,
	(struct usb_descriptor_header *)&etr_miu_superspeedplus_bulk_comp_desc,
	NULL,
};

static void etr_miu_configure_trb(void __iomem *base);

static inline struct etr_miu_dev *func_to_etr_miu(struct usb_function *f)
{
	return container_of(f, struct etr_miu_dev, function);
}

static int
etr_miu_create_bulk_endpoints(struct etr_miu_dev *dev,
			      struct usb_endpoint_descriptor *in_desc)
{
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_ep *ep;

	DBG(cdev, "%s dev: %pK\n", __func__, dev);

	ep = usb_ep_autoconfig(cdev->gadget, in_desc);
	if (!ep) {
		DBG(cdev, "usb_ep_autoconfig for ep_in failed\n");
		return -ENODEV;
	}
	DBG(cdev, "usb_ep_autoconfig for ep_in got %s\n", ep->name);
	ep->driver_data = dev; /* claim the endpoint */
	dev->ep_in = ep;

	return 0;
}

extern int gs_coresight_etm_external_etr_on(u64 buf_addr, u32 buf_size);
extern int gs_coresight_etm_external_etr_off(void);

static int etr_miu_function_bind(struct usb_configuration *c,
				 struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct etr_miu_dev *dev = func_to_etr_miu(f);
	int id;
	int ret;

	dev->cdev = cdev;
	DBG(cdev, "%s dev: %pK\n", __func__, dev);

	/* allocate interface ID(s) */
	id = usb_interface_id(c, f);
	if (id < 0)
		return id;
	etr_miu_interface_alt0_desc.bInterfaceNumber = id;
	etr_miu_interface_alt1_desc.bInterfaceNumber = id;

	/* allocate endpoints */
	ret = etr_miu_create_bulk_endpoints(dev, &etr_miu_fullspeed_in_desc);
	if (ret)
		return ret;

	/* support high speed hardware */
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		etr_miu_highspeed_in_desc.bEndpointAddress =
			etr_miu_fullspeed_in_desc.bEndpointAddress;
	}

	/* support super speed hardware */
	if (gadget_is_superspeed(c->cdev->gadget)) {
		etr_miu_superspeed_in_desc.bEndpointAddress =
			etr_miu_fullspeed_in_desc.bEndpointAddress;
	}

	/* support super speed plus hardware */
	if (gadget_is_superspeed_plus(c->cdev->gadget)) {
		etr_miu_superspeedplus_in_desc.bEndpointAddress =
			etr_miu_fullspeed_in_desc.bEndpointAddress;
	}

	DBG(cdev, "%s speed %s: IN/%s\n",
	    (gadget_is_superspeed_plus(c->cdev->gadget) ? "super-plus" :
	     (gadget_is_superspeed(c->cdev->gadget) ? "super" :
	      (gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full"))),
	    f->name, dev->ep_in->name);

	etr_miu_configure_trb(base);
	cancel_work_sync(&dev->etr_off_work);
	gs_coresight_etm_external_etr_on(tmc_buf_addr, tmc_buf_size);

	return 0;
}

static void etr_miu_function_unbind(struct usb_configuration *c,
				    struct usb_function *f)
{
	struct etr_miu_dev *dev = func_to_etr_miu(f);

	dev->online = 0;

	gs_coresight_etm_external_etr_off();
}

static int etr_miu_function_get_alt(struct usb_function *f, unsigned int intf)
{
	struct etr_miu_dev *dev = func_to_etr_miu(f);

	return dev->alt;
}

static void etr_off_work(struct work_struct *data)
{
	gs_coresight_etm_external_etr_off();
}

static void ep_disable(struct usb_function *f)
{
	struct etr_miu_dev *dev = func_to_etr_miu(f);
	struct usb_composite_dev *cdev = dev->cdev;

	DBG(cdev, "%s cdev %pK\n", __func__, cdev);

	if (dev->online)
		usb_ep_disable(dev->ep_in);

	dev->online = 0;
	dev->alt = 0;

	VDBG(cdev, "%s disabled\n", dev->function.name);
}

static void etr_miu_function_disable(struct usb_function *f)
{
	struct etr_miu_dev *dev = func_to_etr_miu(f);

	ep_disable(f);
	schedule_work(&dev->etr_off_work);
}

static int etr_miu_function_set_alt(struct usb_function *f, unsigned int intf,
				    unsigned int alt)
{
	struct etr_miu_dev *dev = func_to_etr_miu(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	struct dwc3_ep *dep;
	dma_addr_t ep_dma;
	int ret = 0;

	DBG(cdev, "%s: %d alt: %d\n", __func__, intf, alt);

	if (alt == dev->alt)
		return ret;

	if (alt == 0)
		goto alt0;

	/* alt1: */
	ret = config_ep_by_speed(cdev->gadget, f, dev->ep_in);
	if (ret)
		return ret;

	__raw_writel(DATA_TRB_CTRL, base + ETR_DESC_ROM_RD(3));

	/*
	 * ETR_MIU's TRB use a hardware fixed physical address.
	 * Backup original dma address in dwc3 and replace it with ETR_MIU's
	 * one and restore after params of DWC3_DEPCMD_STARTTRANSFER sent.
	 */
	dep = to_dwc3_ep(dev->ep_in);
	ep_dma = dep->trb_pool_dma;
	dep->trb_pool_dma = (dma_addr_t)ETR_MIU_TRB_ADDR;
	ret = usb_ep_enable(dev->ep_in);
	dep->trb_pool_dma = ep_dma;
	if (ret)
		goto alt0;

	dev->online = 1;
	dev->alt = 1;

	return 0;

alt0:
	ep_disable(f);

	dev->alt = 0;

	return ret;
}

static int etr_miu_setup(struct etr_miu_instance *fi_etr_miu)
{
	struct etr_miu_dev *dev;

	if (!fi_etr_miu)
		return -EINVAL;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	INIT_WORK(&dev->etr_off_work, etr_off_work);
	fi_etr_miu->dev = dev;

	return 0;
}

static struct etr_miu_instance *to_etr_miu_instance(struct config_item *item)
{
	return container_of(to_config_group(item), struct etr_miu_instance,
			    func_inst.group);
}

static void etr_miu_attr_release(struct config_item *item)
{
	struct etr_miu_instance *fi_etr_miu = to_etr_miu_instance(item);

	usb_put_function_instance(&fi_etr_miu->func_inst);
}

static struct configfs_item_operations etr_miu_item_ops = {
	.release = etr_miu_attr_release,
};

static struct config_item_type etr_miu_func_type = {
	.ct_item_ops = &etr_miu_item_ops,
	.ct_owner = THIS_MODULE,
};

static struct etr_miu_instance *to_fi_etr_miu(struct usb_function_instance *fi)
{
	return container_of(fi, struct etr_miu_instance, func_inst);
}

static int etr_miu_set_inst_name(struct usb_function_instance *fi,
				 const char *name)
{
	struct etr_miu_instance *fi_etr_miu;
	char *ptr;
	int name_len;

	name_len = strlen(name) + 1;
	if (name_len > MAX_INST_NAME_LEN)
		return -ENAMETOOLONG;

	ptr = kstrndup(name, name_len, GFP_KERNEL);
	if (!ptr)
		return -ENOMEM;

	fi_etr_miu = to_fi_etr_miu(fi);
	fi_etr_miu->name = ptr;

	return 0;
}

static void etr_miu_free_func_inst(struct usb_function_instance *fi)
{
	struct etr_miu_instance *fi_etr_miu;

	fi_etr_miu = to_fi_etr_miu(fi);
	cancel_work_sync(&fi_etr_miu->dev->etr_off_work);
	kfree(fi_etr_miu->name);
	kfree(fi_etr_miu->dev);
	kfree(fi_etr_miu);
}

static struct usb_function_instance *etr_miu_alloc_inst(void)
{
	struct etr_miu_instance *fi_etr_miu;
	int ret = 0;

	fi_etr_miu = kzalloc(sizeof(*fi_etr_miu), GFP_KERNEL);
	if (!fi_etr_miu)
		return ERR_PTR(-ENOMEM);
	fi_etr_miu->func_inst.set_inst_name = etr_miu_set_inst_name;
	fi_etr_miu->func_inst.free_func_inst = etr_miu_free_func_inst;

	ret = etr_miu_setup(fi_etr_miu);
	if (ret) {
		kfree(fi_etr_miu);
		pr_err("Error setting etr_miu\n");
		return ERR_PTR(ret);
	}

	config_group_init_type_name(&fi_etr_miu->func_inst.group, "",
				    &etr_miu_func_type);

	return &fi_etr_miu->func_inst;
}

static void etr_miu_free(struct usb_function *f)
{
	/* NO-OP: no function specific resource allocation in etr_miu_alloc */
}

static struct usb_function *etr_miu_alloc_func(struct usb_function_instance *fi)
{
	struct etr_miu_instance *fi_etr_miu = to_fi_etr_miu(fi);
	struct etr_miu_dev *dev = fi_etr_miu->dev;

	dev->function.name = "etr_miu";
	dev->function.fs_descriptors = fs_etr_miu_descs;
	dev->function.hs_descriptors = hs_etr_miu_descs;
	dev->function.ss_descriptors = ss_etr_miu_descs;
	dev->function.ssp_descriptors = ssp_etr_miu_descs;
	dev->function.bind = etr_miu_function_bind;
	dev->function.unbind = etr_miu_function_unbind;
	dev->function.set_alt = etr_miu_function_set_alt;
	dev->function.get_alt = etr_miu_function_get_alt;
	dev->function.disable = etr_miu_function_disable;
	dev->function.free_func = etr_miu_free;

	return &dev->function;
}

static void etr_miu_configure_trb(void __iomem *base)
{
	/* Data TRB */
	__raw_writel(lower_32_bits(ETR_MIU_DATA_ADDR), base + ETR_DESC_ROM_RD(0));
	__raw_writel(upper_32_bits(ETR_MIU_DATA_ADDR), base + ETR_DESC_ROM_RD(1));
	__raw_writel(ETR_MIU_TRANSFER_SIZE, base + ETR_DESC_ROM_RD(2));
	__raw_writel(DATA_TRB_CTRL, base + ETR_DESC_ROM_RD(3));

	/* Link TRB */
	__raw_writel(lower_32_bits(ETR_MIU_TRB_ADDR), base + ETR_DESC_ROM_RD(4));
	__raw_writel(upper_32_bits(ETR_MIU_TRB_ADDR), base + ETR_DESC_ROM_RD(5));
	__raw_writel(LINK_TRB_TRANSFER_SIZE, base + ETR_DESC_ROM_RD(6));
	__raw_writel(LINK_TRB_CTRL, base + ETR_DESC_ROM_RD(7));
}

static const struct of_device_id etr_miu_match[] __initconst = {
	{
		.compatible = "google,gs101-etr-miu",
	},
	{},
};
MODULE_DEVICE_TABLE(of, etr_miu_match);

static int etr_miu_init(void)
{
	struct device_node *np;
	int ret = 0;

	np = of_find_matching_node(NULL, etr_miu_match);
	if (!np) {
		pr_err("Failed to find ETR MIU device\n");
		return -ENODEV;
	}

	ret = of_property_read_u64(np, "tmc_buf_addr", &tmc_buf_addr);
	if (ret) {
		pr_err("Failed to read TMC buffer address\n");
		goto out;
	}

	ret = of_property_read_u32(np, "tmc_buf_size", &tmc_buf_size);
	if (ret) {
		pr_err("Failed to read TMC buffer size\n");
		goto out;
	}

	base = of_iomap(np, 0);
	if (!base) {
		pr_err("Failed to get register region\n");
		ret = -ENOENT;
		goto out;
	}

	pr_info("ETR MIU configured\n");

out:
	of_node_put(np);
	return ret;
}

DECLARE_USB_FUNCTION(etr_miu, etr_miu_alloc_inst, etr_miu_alloc_func);

static ssize_t drain_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t size)
{
	unsigned long val;

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;

	if (val > MAX_DRAIN)
		val = MAX_DRAIN;

	while (val-- && !(ETR_MIU_ON__IDLE & __raw_readl(base + ETR_MIU_ON))) {
		__raw_writel(ETR_MEM_CTRL_RD__DRAIN, base + ETR_MEM_CTRL_RD);
		udelay(1);
	}

	return size;
}

static struct kobj_attribute etr_miu_drain_attr = __ATTR_WO(drain);

static ssize_t trb_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t size)
{
	etr_miu_configure_trb(base);

	return size;
}

static struct kobj_attribute etr_miu_trb_attr = __ATTR_WO(trb);

static ssize_t idle_show(struct kobject *kobj, struct kobj_attribute *attr,
			 char *buf)
{
	int idle = !!(ETR_MIU_ON__IDLE & __raw_readl(base + ETR_MIU_ON));

	return scnprintf(buf, PAGE_SIZE, "%d\n", idle);
}

static struct kobj_attribute etr_miu_idle_attr = __ATTR_RO(idle);

static struct attribute *etr_miu_sysfs_attrs[] = {
	&etr_miu_drain_attr.attr,
	&etr_miu_idle_attr.attr,
	&etr_miu_trb_attr.attr,
	NULL,
};

static struct attribute_group etr_miu_sysfs_group = {
	.attrs = etr_miu_sysfs_attrs,
};

static const struct attribute_group *etr_miu_sysfs_groups[] = {
	&etr_miu_sysfs_group,
	NULL,
};

static struct bus_type etr_miu_subsys = {
	.name = "exynos-etr-miu",
	.dev_name = "exynos-etr-miu",
};

static int __init etr_miu_mod_init(void)
{
	int ret = 0;

	ret = etr_miu_init();
	if (ret == -ENODEV) {
		return 0;
	} else if (ret) {
		pr_err("fail to init ETR MIU\n");
		return ret;
	}

	ret = subsys_system_register(&etr_miu_subsys, etr_miu_sysfs_groups);
	if (ret) {
		pr_err("fail to register ETR MIU subsys\n");
		return ret;
	}

	ret = usb_function_register(&etr_miuusb_func);
	if (ret) {
		pr_err("fail to register ETR MIU usb function\n");
		return ret;
	}

	return ret;
}

static void __exit etr_miu_mod_exit(void)
{
	if (base)
		iounmap(base);

	usb_function_unregister(&etr_miuusb_func);
}

module_init(etr_miu_mod_init);
module_exit(etr_miu_mod_exit);

MODULE_AUTHOR("Kevin Yang <kangyang@google.com>");
MODULE_LICENSE("GPL");
