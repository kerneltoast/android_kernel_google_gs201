// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Google Corp.
 *
 * Author:
 *  Howard.Yen <howardyen@google.com>
 */

#include <linux/dmapool.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/pm_wakeup.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/workqueue.h>
#include <linux/usb/hcd.h>

#include <trace/hooks/audio_usboffload.h>

#include "aoc_usb.h"
#include "xhci-exynos.h"
#include "xhci_offload_impl.h"

#if IS_ENABLED(CONFIG_USB_XHCI_GOOG_DMA)
#include <linux/iommu.h>
#include "xhci-goog-dma.h"
#endif

static struct xhci_offload_data *offload_data;
struct xhci_offload_data *xhci_get_offload_data(void)
{
	return offload_data;
}

static struct xhci_hcd *get_xhci_hcd_by_udev(struct usb_device *udev)
{
	struct usb_hcd *uhcd = container_of(udev->bus, struct usb_hcd, self);

	return hcd_to_xhci(uhcd);
}

static u32 xhci_get_endpoint_type(struct usb_host_endpoint *ep)
{
	int in;

	in = usb_endpoint_dir_in(&ep->desc);

	switch (usb_endpoint_type(&ep->desc)) {
	case USB_ENDPOINT_XFER_CONTROL:
		return CTRL_EP;
	case USB_ENDPOINT_XFER_BULK:
		return in ? BULK_IN_EP : BULK_OUT_EP;
	case USB_ENDPOINT_XFER_ISOC:
		return in ? ISOC_IN_EP : ISOC_OUT_EP;
	case USB_ENDPOINT_XFER_INT:
		return in ? INT_IN_EP : INT_OUT_EP;
	}
	return 0;
}

/*
 * If the Host connected to a hub, user may connect more than two USB audio
 * headsets or DACs. A caller can call this function to know how many USB
 * audio devices are connected now.
 */
int xhci_get_usb_audio_count(void)
{
	if (offload_data)
		return offload_data->usb_audio_count;
	else
		return 0;
}

/*
 * Determine if an USB device is a compatible devices:
 *     True: Devices are audio class and they contain ISOC endpoint
 *    False: Devices are not audio class or they're audio class but no ISOC endpoint
 */
static bool is_compatible_with_usb_audio_offload(struct usb_device *udev)
{
	struct usb_endpoint_descriptor *epd;
	struct usb_host_config *config;
	struct usb_host_interface *alt;
	struct usb_interface_cache *intfc;
	int i, j, k;
	bool is_audio = false;

	config = udev->config;
	for (i = 0; i < config->desc.bNumInterfaces; i++) {
		intfc = config->intf_cache[i];
		for (j = 0; j < intfc->num_altsetting; j++) {
			alt = &intfc->altsetting[j];

			if (alt->desc.bInterfaceClass == USB_CLASS_AUDIO) {
				for (k = 0; k < alt->desc.bNumEndpoints; k++) {
					epd = &alt->endpoint[k].desc;
					if (usb_endpoint_xfer_isoc(epd)) {
						is_audio = true;
						break;
					}
				}
			}
		}
	}

	return is_audio;
}

static void setup_transfer_ring(struct usb_device *udev, struct usb_host_endpoint *ep)
{
	struct xhci_hcd *xhci = get_xhci_hcd_by_udev(udev);
	struct xhci_virt_device *virt_dev;
	unsigned int ep_index;
	u32 endpoint_type;
	u16 dir;

	ep_index = xhci_get_endpoint_index(&ep->desc);
	endpoint_type = xhci_get_endpoint_type(ep);
	dir = endpoint_type == ISOC_IN_EP ? 0 : 1;

	virt_dev = xhci->devs[udev->slot_id];
	if (!virt_dev) {
		xhci_err(xhci, "%s: virt_dev not found!\n", __func__);
		return;
	}

	if (virt_dev->eps[ep_index].new_ring) {
		xhci_info(xhci, "%s: deliver transfer ring from new_ring\n", __func__);
		xhci_set_isoc_tr_info(0, dir, virt_dev->eps[ep_index].new_ring);
	} else if (virt_dev->eps[ep_index].ring) {
		xhci_info(xhci, "%s: deliver transfer ring from ring\n", __func__);
		xhci_set_isoc_tr_info(0, dir, virt_dev->eps[ep_index].ring);
	} else {
		xhci_err(xhci, "%s: transfer ring not found!\n", __func__);
	}
}

static void offload_connect_work(struct work_struct *work)
{
	struct xhci_hcd *xhci = offload_data->xhci;

	xhci_info(xhci, "Set offloading state %s\n",
		  offload_data->offload_state ? "true" : "false");
	notify_offload_state(offload_data->offload_state);
}

static void usb_audio_vendor_connect(void *unused, struct usb_interface *intf,
				     struct snd_usb_audio *chip)
{
	if (offload_data) {
		offload_data->offload_state = true;
		schedule_work(&offload_data->offload_connect_ws);
	}
}

static void usb_audio_vendor_disconnect(void *unused, struct usb_interface *intf)
{
	if (offload_data) {
		offload_data->offload_state = false;
		schedule_work(&offload_data->offload_connect_ws);
	}
}

static int xhci_udev_notify(struct notifier_block *self, unsigned long action,
			    void *dev)
{
	struct usb_device *udev = dev;

	switch (action) {
	case USB_DEVICE_ADD:
		if (is_compatible_with_usb_audio_offload(udev)) {
			dev_dbg(&udev->dev, "Compatible with usb audio offload\n");
			offload_data->usb_audio_count++;
			xhci_sync_conn_stat(udev->bus->busnum, udev->devnum, udev->slot_id,
					    USB_CONNECTED);
		}
		break;
	case USB_DEVICE_REMOVE:
		if (is_compatible_with_usb_audio_offload(udev)) {
			offload_data->usb_audio_count--;
			xhci_sync_conn_stat(udev->bus->busnum, udev->devnum, udev->slot_id,
					    USB_DISCONNECTED);
		}
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block xhci_udev_nb = {
	.notifier_call = xhci_udev_notify,
};

static int usb_audio_offload_init(struct xhci_hcd *xhci)
{
	struct device *dev = xhci_to_hcd(xhci)->self.sysdev;
	int ret;
	u32 out_val;
#if IS_ENABLED(CONFIG_USB_XHCI_GOOG_DMA)
	struct iommu_domain	*domain;
	struct reserved_mem	*rmem;
	struct device_node	*np;
	int i, count;
#endif

	offload_data = kzalloc(sizeof(struct xhci_offload_data), GFP_KERNEL);
	if (!offload_data)
		return -ENOMEM;

	if (!of_property_read_u32(dev->of_node, "offload", &out_val))
		offload_data->usb_audio_offload = (out_val == 1) ? true : false;

#if IS_ENABLED(CONFIG_USB_XHCI_GOOG_DMA)
	domain = iommu_get_domain_for_dev(dev);
	if (!domain) {
		dev_err(dev, "iommu domain not found.\n");
		ret = -ENOMEM;
		goto free_data;
	}

	xhci_goog_rmem_setup_latecall(dev);

	for (i = 0; i < XHCI_GOOG_DMA_RMEM_MAX; i++) {
		ret = of_reserved_mem_device_init_by_idx(dev, dev->of_node, i);
		if (ret) {
			dev_err(dev, "Could not get reserved memory index %d\n", i);
			goto release_rmem;
		}
	}

	count = of_property_count_elems_of_size(dev->of_node, "memory-region",
						sizeof(u32));

	for (i = 0; i < count; i++) {
		np = of_parse_phandle(dev->of_node, "memory-region", i);
		if (!np) {
			dev_err(dev, "memory-region not found\n");
			ret = -ENOMEM;
			goto unmap_iommu;
		}

		rmem = of_reserved_mem_lookup(np);
		if (!rmem) {
			dev_err(dev, "rmem lookup failed.\n");
			ret = -ENOMEM;
			goto unmap_iommu;
		}

		ret = iommu_map(domain, rmem->base, rmem->base, rmem->size,
				IOMMU_READ | IOMMU_WRITE);
		if (ret < 0) {
			dev_err(dev, "iommu_map error: %d\n", ret);
			goto unmap_iommu;
		}
	}

	xhci_goog_setup_dma_ops(dev);

#else

	ret = of_reserved_mem_device_init(dev);
	if (ret) {
		dev_err(dev, "Could not get reserved memory\n");
		kfree(offload_data);
		return ret;
	}

#endif
	offload_data->dt_direct_usb_access =
		of_property_read_bool(dev->of_node, "direct-usb-access") ? true : false;
	if (!offload_data->dt_direct_usb_access)
		dev_warn(dev, "Direct USB access is not supported\n");

	offload_data->offload_state = true;
	offload_data->usb_audio_count = 0;

	/* Notification for xhci driver probing */
	usb_host_mode_state_notify(USB_CONNECTED);

	aoc_alsa_usb_callback_register(setup_transfer_ring);
	usb_register_notify(&xhci_udev_nb);
	offload_data->xhci = xhci;

	INIT_WORK(&offload_data->offload_connect_ws, offload_connect_work);

	return 0;
#if IS_ENABLED(CONFIG_USB_XHCI_GOOG_DMA)
unmap_iommu:
	if (i > 0) {
		for (i = 0; i < count - 1; i++) {
			np = of_parse_phandle(dev->of_node, "memory-region", i);
			if (!np)
				continue;

			rmem = of_reserved_mem_lookup(np);
			if (!rmem)
				continue;

			iommu_unmap(domain, rmem->base, rmem->size);
		}
	}

release_rmem:
	of_reserved_mem_device_release(dev);

free_data:
	kfree(offload_data);

	return ret;
#endif
}

static int usb_audio_offload_setup(struct xhci_hcd *xhci)
{
	if (!xhci->dcbaa->dma) {
		xhci_err(xhci, "dma is null!\n");
		return -ENOMEM;
	}

	return xhci_setup_done();
}

static void usb_audio_offload_cleanup(struct xhci_hcd *xhci)
{
#if IS_ENABLED(CONFIG_USB_XHCI_GOOG_DMA)
	struct device *dev = xhci_to_hcd(xhci)->self.sysdev;
	struct iommu_domain	*domain;
	struct reserved_mem	*rmem;
	struct device_node	*np;
	int i, count, ret;
#endif

	offload_data->usb_audio_offload = false;
	offload_data->offload_state = false;
	offload_data->xhci = NULL;

	aoc_alsa_usb_callback_unregister();
	usb_unregister_notify(&xhci_udev_nb);

	/* Notification for xhci driver removing */
	usb_host_mode_state_notify(USB_DISCONNECTED);

	cancel_work_sync(&offload_data->offload_connect_ws);

#if IS_ENABLED(CONFIG_USB_XHCI_GOOG_DMA)
	xhci_goog_restore_dma_ops(dev);

	domain = iommu_get_domain_for_dev(dev);

	count = of_property_count_elems_of_size(dev->of_node, "memory-region",
						sizeof(u32));

	for (i = 0; i < count; i++) {
		if (!domain)
			break;

		np = of_parse_phandle(dev->of_node, "memory-region", i);
		if (!np) {
			dev_err(dev, "memory-region not found\n");
			continue;
		}

		rmem = of_reserved_mem_lookup(np);
		if (!rmem) {
			dev_err(dev, "rmem lookup failed.\n");
			continue;
		}

		ret = iommu_unmap(domain, rmem->base, rmem->size);
		if (ret < 0)
			dev_err(dev, "iommu_map error: %d\n", ret);
	}

	of_reserved_mem_device_release(dev);
#endif

	kfree(offload_data);
	offload_data = NULL;
}

static struct xhci_exynos_ops offload_ops = {
	.offload_init = usb_audio_offload_init,
	.offload_cleanup = usb_audio_offload_cleanup,
	.offload_setup = usb_audio_offload_setup,
};

int xhci_offload_helper_init(void)
{
	int ret;

	ret = register_trace_android_vh_audio_usb_offload_connect(usb_audio_vendor_connect, NULL);
	if (ret)
		pr_err("register_trace_android_vh_audio_usb_offload_connect failed: %d\n", ret);

	ret = register_trace_android_rvh_audio_usb_offload_disconnect(usb_audio_vendor_disconnect,
								      NULL);
	if (ret)
		pr_err("register_trace_android_rvh_audio_usb_offload_disconnect failed: %d\n", ret);

	return xhci_exynos_register_offload_ops(&offload_ops);
}
