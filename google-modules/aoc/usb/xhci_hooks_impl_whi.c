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

#include "xhci.h"
#include "xhci-plat.h"
#include "aoc_usb.h"

#define SRAM_BASE 0x19000000
#define SRAM_SIZE 0x600000
#define DRAM_BASE 0x94000000
#define DRAM_SIZE 0x3000000

static BLOCKING_NOTIFIER_HEAD(aoc_usb_notifier_list);

int register_aoc_usb_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&aoc_usb_notifier_list, nb);
}

int unregister_aoc_usb_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&aoc_usb_notifier_list, nb);
}

int xhci_send_feedback_ep_info(struct xhci_hcd *xhci, struct feedback_ep_info_args *cmd_args)
{
	if (!xhci || !cmd_args)
		return -EINVAL;

	xhci_dbg(xhci, "Send feedback EP info, Num = %u, Max packet size = %u, bInterval = %u, bRefresh = %u",
		 cmd_args->ep_num, cmd_args->max_packet,
		 cmd_args->binterval, cmd_args->brefresh);

	blocking_notifier_call_chain(&aoc_usb_notifier_list, SEND_FB_EP_INFO,
				     cmd_args);

	return 0;
}

/*
 * If the Host connected to a hub, user may connect more than two USB audio
 * headsets or DACs. A caller can call this function to know how many USB
 * audio devices are connected now.
 */
int xhci_get_usb_audio_count(struct xhci_hcd *xhci)
{
	struct xhci_vendor_data *vendor_data;

	if (!xhci)
		return -ENODEV;

	vendor_data = xhci_to_priv(xhci)->vendor_data;

	return vendor_data->usb_audio_count;
}

int xhci_set_offload_state(struct xhci_hcd *xhci, bool enabled)
{
	struct xhci_vendor_data *vendor_data;

	if (!xhci)
		return -EINVAL;

	vendor_data = xhci_to_priv(xhci)->vendor_data;

	if (!vendor_data->dt_direct_usb_access)
		return -EPERM;

	xhci_info(xhci, "Set offloading state %s\n", enabled ? "true" : "false");

	blocking_notifier_call_chain(&aoc_usb_notifier_list, SET_OFFLOAD_STATE,
				     &enabled);
	vendor_data->offload_state = enabled;

	return 0;
}

static int xhci_sync_dev_ctx(struct xhci_hcd *xhci, unsigned int slot_id)
{
	struct xhci_virt_device *dev;
	struct xhci_container_ctx *out_ctx_ref;
	struct xhci_slot_ctx *slot_ctx;
	struct xhci_ep_ctx *ep_ctx;
	struct get_dev_ctx_args args;
	u8 *dev_ctx;
	char			str[XHCI_MSG_MAX];

	if (IS_ERR_OR_NULL(xhci))
		return -ENODEV;

	if (xhci->xhc_state & XHCI_STATE_DYING ||
	    xhci->xhc_state & XHCI_STATE_REMOVING) {
		xhci_err(xhci, "xHCI dying, ignore sync_dev_ctx\n");
		return -ENODEV;
	}

	if (!xhci->devs[slot_id])
		return -ENODEV;

	dev = xhci->devs[slot_id];
	out_ctx_ref = dev->out_ctx;

	xhci_dbg(xhci, "slot_id=%u, out_ctx_ref->size=%u\n", slot_id,
		 out_ctx_ref->size);

	dev_ctx = kcalloc(1, out_ctx_ref->size, GFP_KERNEL);
	if (!dev_ctx)
		return -ENOMEM;

	args.slot_id = slot_id;
	args.length = out_ctx_ref->size;
	args.dev_ctx = dev_ctx;
	blocking_notifier_call_chain(&aoc_usb_notifier_list,
				     SYNC_DEVICE_CONTEXT, &args);

	memcpy(out_ctx_ref->bytes, dev_ctx, out_ctx_ref->size);

	slot_ctx = xhci_get_slot_ctx(xhci, out_ctx_ref);
	ep_ctx = xhci_get_ep_ctx(xhci, out_ctx_ref, 0); /* ep0 */

	xhci_dbg(xhci, "%s\n",
		 xhci_decode_slot_context(str,
			 slot_ctx->dev_info, slot_ctx->dev_info2,
			 slot_ctx->tt_info, slot_ctx->dev_state));
	xhci_dbg(xhci, "%s\n",
		 xhci_decode_ep_context(str, ep_ctx->ep_info, ep_ctx->ep_info2,
					ep_ctx->deq, ep_ctx->tx_info));

	kfree(dev_ctx);
	return 0;
}

static int xhci_get_dcbaa_ptr(u64 *aoc_dcbaa_ptr)
{
	blocking_notifier_call_chain(&aoc_usb_notifier_list, GET_DCBAA_PTR,
				     aoc_dcbaa_ptr);
	return 0;
}

static int xhci_set_dcbaa_ptr(u64 aoc_dcbaa_ptr)
{
	blocking_notifier_call_chain(&aoc_usb_notifier_list, SET_DCBAA_PTR,
					 &aoc_dcbaa_ptr);
	return 0;
}

static int xhci_setup_done(void)
{
	blocking_notifier_call_chain(&aoc_usb_notifier_list, SETUP_DONE, NULL);
	return 0;
}

static int xhci_sync_conn_stat(unsigned int bus_id, unsigned int dev_num, unsigned int slot_id,
			       unsigned int conn_stat)
{
	struct conn_stat_args args;

	args.bus_id = bus_id;
	args.dev_num = dev_num;
	args.slot_id = slot_id;
	args.conn_stat = conn_stat;
	blocking_notifier_call_chain(&aoc_usb_notifier_list, SYNC_CONN_STAT, &args);

	return 0;
}

int usb_host_mode_state_notify(enum aoc_usb_state usb_state)
{
	return xhci_sync_conn_stat(0, 0, 0, usb_state);
}

static int xhci_get_isoc_tr_info(u16 ep_id, u16 dir, struct xhci_ring *ep_ring)
{
	struct get_isoc_tr_info_args tr_info;

	tr_info.ep_id = ep_id;
	tr_info.dir = dir;
	blocking_notifier_call_chain(&aoc_usb_notifier_list, GET_ISOC_TR_INFO,
				     &tr_info);

	ep_ring->num_segs = tr_info.num_segs;
	ep_ring->bounce_buf_len = tr_info.max_packet;
	ep_ring->type = tr_info.type;
	ep_ring->first_seg->dma = tr_info.seg_ptr;
	ep_ring->cycle_state = tr_info.cycle_state;
	ep_ring->num_trbs_free = tr_info.num_trbs_free;

	return 0;
}

static int xhci_set_isoc_tr_info(u16 ep_id, u16 dir, struct xhci_ring *ep_ring)
{
	struct get_isoc_tr_info_args tr_info;

	tr_info.ep_id = ep_id;
	tr_info.dir = dir;
	tr_info.num_segs = ep_ring->num_segs;
	tr_info.max_packet = ep_ring->bounce_buf_len;
	tr_info.type = ep_ring->type;
	tr_info.seg_ptr = ep_ring->first_seg->dma;
	tr_info.cycle_state = ep_ring->cycle_state;
	tr_info.num_trbs_free = ep_ring->num_trbs_free;

	blocking_notifier_call_chain(&aoc_usb_notifier_list, SET_ISOC_TR_INFO,
				     &tr_info);

	return 0;
}

/*
 * Determine if an USB device is a compatible devices:
 *     True: Devices are audio class and they contain ISOC endpoint
 *    False: Devices are not audio class or they're audio class but no ISOC endpoint or
 *           they have at least one interface is video class
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

			if (alt->desc.bInterfaceClass == USB_CLASS_VIDEO) {
				is_audio = false;
				goto out;
			}

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

out:
	return is_audio;
}

/*
 * check the usb device including the video class:
 *     True: Devices contain video class
 *    False: Device doesn't contain video class
 */
static bool is_usb_video_device(struct usb_device *udev)
{
	struct usb_host_config *config;
	struct usb_host_interface *alt;
	struct usb_interface_cache *intfc;
	int i, j;
	bool is_video = false;

	if (!udev || !udev->config)
		return is_video;

	config = udev->config;

	for (i = 0; i < config->desc.bNumInterfaces; i++) {
		intfc = config->intf_cache[i];
		for (j = 0; j < intfc->num_altsetting; j++) {
			alt = &intfc->altsetting[j];

			if (alt->desc.bInterfaceClass == USB_CLASS_VIDEO) {
				is_video = true;
				goto out;
			}
		}
	}

out:
	return is_video;
}

struct xhci_hcd *get_xhci_hcd_by_udev(struct usb_device *udev)
{
	struct usb_hcd *uhcd = container_of(udev->bus, struct usb_hcd, self);

	return hcd_to_xhci(uhcd);
}


static int sync_dev_ctx(struct xhci_hcd *xhci, unsigned int slot_id)
{
	struct xhci_vendor_data *vendor_data = xhci_to_priv(xhci)->vendor_data;
	int ret = 0;

	if (vendor_data->op_mode == USB_OFFLOAD_SIMPLE_AUDIO_ACCESSORY)
		ret = xhci_sync_dev_ctx(xhci, slot_id);

	return ret;
}

static int xhci_udev_notify(struct notifier_block *self, unsigned long action,
			    void *dev)
{
	struct usb_device *udev = dev;
	struct xhci_vendor_data *vendor_data;
	struct xhci_hcd *xhci;

	switch (action) {
	case USB_DEVICE_ADD:
		xhci = get_xhci_hcd_by_udev(udev);
		vendor_data = xhci_to_priv(xhci)->vendor_data;
		if (is_compatible_with_usb_audio_offload(udev)) {
			dev_dbg(&udev->dev,
				 "Compatible with usb audio offload\n");
			if (vendor_data->op_mode ==
			    USB_OFFLOAD_SIMPLE_AUDIO_ACCESSORY ||
				vendor_data->op_mode ==
			    USB_OFFLOAD_DRAM) {
				vendor_data->usb_audio_count++;
				xhci_sync_conn_stat(udev->bus->busnum, udev->devnum, udev->slot_id,
						    USB_CONNECTED);
			}
		}
		vendor_data->usb_accessory_enabled = false;
		break;
	case USB_DEVICE_REMOVE:
		xhci = get_xhci_hcd_by_udev(udev);
		vendor_data = xhci_to_priv(xhci)->vendor_data;
		if (is_compatible_with_usb_audio_offload(udev) &&
		    (vendor_data->op_mode ==
		     USB_OFFLOAD_SIMPLE_AUDIO_ACCESSORY ||
			 vendor_data->op_mode ==
			 USB_OFFLOAD_DRAM)) {
			vendor_data->usb_audio_count--;
			xhci_sync_conn_stat(udev->bus->busnum, udev->devnum, udev->slot_id,
					    USB_DISCONNECTED);
		}
		vendor_data->usb_accessory_enabled = false;
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block xhci_udev_nb = {
	.notifier_call = xhci_udev_notify,
};

static void xhci_vendor_irq_work(struct work_struct *work)
{
	struct xhci_vendor_data *vendor_data =
		container_of(work, struct xhci_vendor_data, xhci_vendor_irq_work);
	struct xhci_hcd *xhci = vendor_data->xhci;
	struct usb_hcd *hcd = xhci_to_hcd(xhci);
	union xhci_trb *event_ring_deq;
	unsigned long flags;
	u64 temp_64;
	u32 status = 0;
	int event_loop = 0;
	unsigned int slot_id = 1;
	int ret;

	if (IS_ERR_OR_NULL(xhci)) {
		pr_err("xHCI null, ignore irq work\n");
		return;
	}

	if (xhci->xhc_state & XHCI_STATE_DYING ||
	    xhci->xhc_state & XHCI_STATE_REMOVING) {
		xhci_err(xhci, "xHCI dying, ignore irq work\n");
		return;
	}

	ret = sync_dev_ctx(xhci, slot_id);
	if (ret)
		xhci_warn(xhci, "Failed to sync device context failed, err=%d", ret);

	spin_lock_irqsave(&xhci->lock, flags);

	/*
	 * Clear the op reg interrupt status first,
	 * so we can receive interrupts from other MSI-X interrupters.
	 * Write 1 to clear the interrupt status.
	 */
	status |= STS_EINT;
	writel(status, &xhci->op_regs->status);

	if (!hcd->msi_enabled) {
		u32 irq_pending;

		irq_pending = readl(&xhci->ir_set->irq_pending);
		irq_pending |= IMAN_IP;
		writel(irq_pending, &xhci->ir_set->irq_pending);
	}

	if (xhci->xhc_state & XHCI_STATE_DYING ||
	    xhci->xhc_state & XHCI_STATE_HALTED) {
		xhci_err(xhci, "xHCI dying, ignoring interrupt. Shouldn't IRQs be disabled?\n");
		/*
		 * Clear the event handler busy flag (RW1C);
		 * the event ring should be empty.
		 */
		temp_64 = xhci_read_64(xhci, &xhci->ir_set->erst_dequeue);
		xhci_write_64(xhci, temp_64 | ERST_EHB,
			      &xhci->ir_set->erst_dequeue);
		goto out;
	}

	event_ring_deq = xhci->event_ring->dequeue;
	/*
	 * FIXME this should be a delayed service routine
	 * that clears the EHB.
	 */
	while (xhci_handle_event(xhci) > 0) {
		if (event_loop++ < TRBS_PER_SEGMENT / 2)
			continue;
		xhci_update_erst_dequeue(xhci, event_ring_deq);
		event_loop = 0;
	}

	xhci_update_erst_dequeue(xhci, event_ring_deq);

out:
	spin_unlock_irqrestore(&xhci->lock, flags);
}

static int xhci_vendor_init_irq_workqueue(struct xhci_vendor_data *vendor_data)
{
	vendor_data->irq_wq = alloc_workqueue("xhci_vendor_irq_work", WQ_UNBOUND, 0);

	if (!vendor_data->irq_wq)
		return -ENOMEM;

	INIT_WORK(&vendor_data->xhci_vendor_irq_work, xhci_vendor_irq_work);

	return 0;
}

static struct xhci_ring *
xhci_initialize_ring_info_for_remote_isoc(struct xhci_hcd *xhci,
					  u32 endpoint_type,
					  enum xhci_ring_type type,
					  unsigned int max_packet, gfp_t flags)
{
	struct xhci_ring *ring;
	struct xhci_segment *seg;
	u16 dir;
	struct device *dev = xhci_to_hcd(xhci)->self.sysdev;

	ring = kzalloc_node(sizeof(*ring), flags, dev_to_node(dev));
	if (!ring)
		return NULL;

	ring->type = TYPE_ISOC;
	INIT_LIST_HEAD(&ring->td_list);

	seg = kzalloc_node(sizeof(*seg), flags, dev_to_node(dev));
	if (!seg) {
		kfree(ring);
		return NULL;
	}

	ring->bounce_buf_len = max_packet;
	ring->first_seg = seg;
	ring->enq_seg = ring->first_seg;
	ring->deq_seg = ring->first_seg;
	dir = endpoint_type == ISOC_IN_EP ? 0 : 1;

	xhci_get_isoc_tr_info(0, dir, ring);

	xhci_dbg(xhci, "ring->first_seg->dma=0x%llx\n", ring->first_seg->dma);

	if (ring->first_seg->dma == 0) {
		kfree(seg);
		kfree(ring);
		return NULL;
	}

	return ring;
}

static int usb_audio_offload_init(struct xhci_hcd *xhci)
{
	struct device *dev = xhci_to_hcd(xhci)->self.sysdev;
	struct xhci_vendor_data *vendor_data;
	int ret;
	u32 out_val;

	if (!aoc_usb_probe_done) {
		dev_dbg(dev, "deferring the probe\n");
		return -EPROBE_DEFER;
	}

	vendor_data = kzalloc(sizeof(struct xhci_vendor_data), GFP_KERNEL);
	if (!vendor_data)
		return -ENOMEM;

	if (!of_property_read_u32(dev->of_node, "offload", &out_val))
		vendor_data->usb_audio_offload = (out_val == 1) ? true : false;

	ret = xhci_vendor_init_irq_workqueue(vendor_data);
	if (ret) {
		kfree(vendor_data);
		return ret;
	}

	ret = of_reserved_mem_device_init(dev);
	if (ret) {
		dev_err(dev, "Cound not get reserved memory\n");
		kfree(vendor_data);
		return ret;
	}

	vendor_data->dt_direct_usb_access =
		of_property_read_bool(dev->of_node, "direct-usb-access") ? true : false;
	if (!vendor_data->dt_direct_usb_access)
		dev_warn(dev, "Direct USB access is not supported\n");

	vendor_data->offload_state = true;
	vendor_data->usb_audio_count = 0;

	usb_register_notify(&xhci_udev_nb);
	vendor_data->op_mode = USB_OFFLOAD_DRAM;
	vendor_data->xhci = xhci;

	xhci_to_priv(xhci)->vendor_data = vendor_data;

	return 0;
}

static void usb_audio_offload_cleanup(struct xhci_hcd *xhci)
{
	struct xhci_vendor_data *vendor_data = xhci_to_priv(xhci)->vendor_data;

	vendor_data->usb_audio_offload = false;
	vendor_data->op_mode = USB_OFFLOAD_STOP;
	if (vendor_data->irq_wq)
		destroy_workqueue(vendor_data->irq_wq);
	vendor_data->irq_wq = NULL;
	vendor_data->xhci = NULL;

	usb_unregister_notify(&xhci_udev_nb);

	/* Notification for xhci driver removing */
	usb_host_mode_state_notify(USB_DISCONNECTED);

	kfree(vendor_data);
	xhci_to_priv(xhci)->vendor_data = NULL;
}

/* TODO: There is an issue that data missing happened when transfer ring
 * allocated in SRAM and working w/ high-speed mode 125us data packet interval.
 * b/188012253 to track the issue.
 */
static bool is_dma_for_offload(dma_addr_t dma)
{
	if ((dma >= SRAM_BASE && dma < SRAM_BASE + SRAM_SIZE) ||
	    (dma >= DRAM_BASE && dma < DRAM_BASE + DRAM_SIZE))
		return true;

	return false;
}

static bool is_usb_offload_enabled(struct xhci_hcd *xhci,
		struct xhci_virt_device *vdev, unsigned int ep_index)
{
	struct usb_device *udev;
	struct xhci_vendor_data *vendor_data = xhci_to_priv(xhci)->vendor_data;
	bool global_enabled = vendor_data->op_mode != USB_OFFLOAD_STOP;
	struct xhci_ring *ep_ring;

	if (vdev == NULL || vdev->eps[ep_index].ring == NULL)
		return global_enabled;

	udev = vdev->udev;

	if (global_enabled) {
		ep_ring = vdev->eps[ep_index].ring;
		if (vendor_data->op_mode == USB_OFFLOAD_SIMPLE_AUDIO_ACCESSORY) {
			if (is_dma_for_offload(ep_ring->first_seg->dma))
				return true;
		} else if (vendor_data->op_mode == USB_OFFLOAD_DRAM) {
			if (is_usb_video_device(udev))
				return false;
			else if (ep_ring->type == TYPE_ISOC)
				return vendor_data->offload_state;
		}
	}

	return false;
}

/* TODO: there is a issue if urb is submitted but not transfer
 * after USB connection is disconnected immediately.
 * b/189074283 is used to track the formal solution.
 */
static bool is_usb_bulk_transfer_enabled(struct xhci_hcd *xhci, struct urb *urb)
{
	struct xhci_vendor_data *vendor_data = xhci_to_priv(xhci)->vendor_data;
	struct usb_endpoint_descriptor *desc = &urb->ep->desc;
	int ep_type = usb_endpoint_type(desc);
	struct usb_ctrlrequest *cmd;
	bool skip_bulk = false;

	cmd = (struct usb_ctrlrequest *) urb->setup_packet;

	if (ep_type == USB_ENDPOINT_XFER_CONTROL) {
		if (!usb_endpoint_dir_in(desc) && cmd->bRequest == 0x35) {
			vendor_data->usb_accessory_enabled = true;
		} else {
			vendor_data->usb_accessory_enabled = false;
		}
	}

	if (ep_type == USB_ENDPOINT_XFER_BULK && !usb_endpoint_dir_in(desc))
		skip_bulk = vendor_data->usb_accessory_enabled;

	return skip_bulk;
}

static irqreturn_t queue_irq_work(struct xhci_hcd *xhci)
{
	struct xhci_vendor_data *vendor_data = xhci_to_priv(xhci)->vendor_data;
	irqreturn_t ret = IRQ_NONE;
	struct xhci_transfer_event *event;
	u32 trb_comp_code;

	if (vendor_data->op_mode != USB_OFFLOAD_SIMPLE_AUDIO_ACCESSORY)
		return IRQ_NONE;

	if (is_usb_offload_enabled(xhci, NULL, 0)) {
		event = &xhci->event_ring->dequeue->trans_event;
		trb_comp_code = GET_COMP_CODE(le32_to_cpu(event->transfer_len));
		if (trb_comp_code == COMP_STALL_ERROR) {
			if (!work_pending(&vendor_data->xhci_vendor_irq_work)) {
				queue_work(vendor_data->irq_wq,
					   &vendor_data->xhci_vendor_irq_work);
			}
			ret = IRQ_HANDLED;
		}
	}

	return ret;
}

static struct xhci_device_context_array *alloc_dcbaa(struct xhci_hcd *xhci,
						     gfp_t flags)
{
	dma_addr_t dma;
	struct device *dev = xhci_to_hcd(xhci)->self.sysdev;
	struct xhci_vendor_data *vendor_data = xhci_to_priv(xhci)->vendor_data;

	if (vendor_data->op_mode == USB_OFFLOAD_SIMPLE_AUDIO_ACCESSORY) {
		xhci->dcbaa = kcalloc(1, sizeof(*xhci->dcbaa), flags);
		if (!xhci->dcbaa)
			return NULL;

		if (xhci_get_dcbaa_ptr(&xhci->dcbaa->dma) != 0) {
			xhci_err(xhci, "Get DCBAA pointer failed\n");
			return NULL;
		}
		xhci_setup_done();

		xhci_dbg(xhci, "Get dcbaa_ptr=%llx\n", xhci->dcbaa->dma);
	} else if (vendor_data->op_mode == USB_OFFLOAD_DRAM) {
		xhci->dcbaa = dma_alloc_coherent(dev, sizeof(*xhci->dcbaa),
						 &dma, flags);
		if (!xhci->dcbaa)
			return NULL;

		xhci->dcbaa->dma = dma;
		if (xhci_set_dcbaa_ptr(xhci->dcbaa->dma) != 0) {
			xhci_err(xhci, "Set DCBAA pointer failed\n");
			return NULL;
		}
		xhci_setup_done();

		xhci_dbg(xhci, "Set dcbaa_ptr=%llx to AoC\n", xhci->dcbaa->dma);
	} else {
		xhci->dcbaa = dma_alloc_coherent(dev, sizeof(*xhci->dcbaa),
						 &dma, flags);
		if (!xhci->dcbaa)
			return NULL;

		xhci->dcbaa->dma = dma;
	}

	return xhci->dcbaa;
}

static void free_dcbaa(struct xhci_hcd *xhci)
{
	struct device *dev = xhci_to_hcd(xhci)->self.sysdev;
	struct xhci_vendor_data *vendor_data = xhci_to_priv(xhci)->vendor_data;

	if (!xhci->dcbaa)
		return;

	if (vendor_data->op_mode == USB_OFFLOAD_SIMPLE_AUDIO_ACCESSORY) {
		kfree(xhci->dcbaa);
	} else {
		dma_free_coherent(dev, sizeof(*xhci->dcbaa),
				  xhci->dcbaa, xhci->dcbaa->dma);
	}

	xhci->dcbaa = NULL;
}

static struct xhci_ring *alloc_transfer_ring(struct xhci_hcd *xhci,
		u32 endpoint_type, enum xhci_ring_type ring_type,
		unsigned int max_packet, gfp_t mem_flags)
{
	struct xhci_vendor_data *vendor_data = xhci_to_priv(xhci)->vendor_data;
	struct xhci_ring *ep_ring;
	u16 dir;

	if (vendor_data->op_mode == USB_OFFLOAD_SIMPLE_AUDIO_ACCESSORY) {
		ep_ring = xhci_initialize_ring_info_for_remote_isoc(xhci, endpoint_type,
							 ring_type, max_packet,
							 mem_flags);
	} else {
		ep_ring = xhci_ring_alloc(xhci, 1, 1, ring_type, max_packet, mem_flags);
		dir = endpoint_type == ISOC_IN_EP ? 0 : 1;

		xhci_set_isoc_tr_info(0, dir, ep_ring);
	}

	return ep_ring;
}

#if defined(CONFIG_MODULES) && defined(MODULE)
struct xhci_input_control_ctx *xhci_get_input_control_ctx(struct xhci_container_ctx *ctx)
{
	if (ctx->type != XHCI_CTX_TYPE_INPUT)
		return NULL;

	return (struct xhci_input_control_ctx *)ctx->bytes;
}
#endif

static void free_transfer_ring(struct xhci_hcd *xhci,
		struct xhci_virt_device *virt_dev, unsigned int ep_index)
{
	struct xhci_vendor_data *vendor_data = xhci_to_priv(xhci)->vendor_data;
	struct xhci_ring *ring, *new_ring;
	struct xhci_ep_ctx *ep_ctx;
	struct xhci_input_control_ctx *ctrl_ctx;
	u32 ep_type;
	u32 ep_is_added, ep_is_dropped;

	ring = virt_dev->eps[ep_index].ring;
	new_ring = virt_dev->eps[ep_index].new_ring;
	ep_ctx = xhci_get_ep_ctx(xhci, virt_dev->out_ctx, ep_index);
	ep_type = CTX_TO_EP_TYPE(le32_to_cpu(ep_ctx->ep_info2));

	ctrl_ctx = xhci_get_input_control_ctx(virt_dev->in_ctx);
	if (!ctrl_ctx) {
		xhci_warn(xhci, "%s: Could not get input context, bad type.\n", __func__);
		return;
	}
	ep_is_added = EP_IS_ADDED(ctrl_ctx, ep_index);
	ep_is_dropped = EP_IS_DROPPED(ctrl_ctx, ep_index);

	xhci_dbg(xhci, "%s: ep %u is added(0x%x), is dropped(0x%x)\n", __func__, ep_index,
		 ep_is_added, ep_is_dropped);

	if (ring) {
		xhci_dbg(xhci, "%s: ep_index=%u, ep_type=%u, ring type=%u, new_ring=%pK\n",
			 __func__, ep_index, ep_type, ring->type, new_ring);

		if (vendor_data->op_mode == USB_OFFLOAD_SIMPLE_AUDIO_ACCESSORY &&
			ring->type == TYPE_ISOC) {
			kfree(ring->first_seg);
			kfree(virt_dev->eps[ep_index].ring);
		} else
			xhci_ring_free(xhci, virt_dev->eps[ep_index].ring);

		virt_dev->eps[ep_index].ring = NULL;

		if (ep_is_added == 0 && ep_is_dropped == 0)
			return;
	}

	if (new_ring) {
		xhci_dbg(xhci, "%s: ep_index=%u, ep_type=%u, new_ring type=%u\n", __func__, ep_index,
			 ep_type, new_ring->type);

		if (vendor_data->op_mode == USB_OFFLOAD_SIMPLE_AUDIO_ACCESSORY &&
			new_ring->type == TYPE_ISOC) {
			kfree(new_ring->first_seg);
			kfree(virt_dev->eps[ep_index].new_ring);
		} else
			xhci_ring_free(xhci, virt_dev->eps[ep_index].new_ring);

		virt_dev->eps[ep_index].new_ring = NULL;

		return;
	}
}

static bool usb_offload_skip_urb(struct xhci_hcd *xhci, struct urb *urb)
{
	struct xhci_virt_device *vdev = xhci->devs[urb->dev->slot_id];
	struct usb_endpoint_descriptor *desc = &urb->ep->desc;
	int ep_type = usb_endpoint_type(desc);
	unsigned int ep_index;

	if (ep_type == USB_ENDPOINT_XFER_CONTROL)
		ep_index = (unsigned int)(usb_endpoint_num(desc)*2);
	else
		ep_index = (unsigned int)(usb_endpoint_num(desc)*2) +
			   (usb_endpoint_dir_in(desc) ? 1 : 0) - 1;

	xhci_dbg(xhci, "%s: ep_index=%u, ep_type=%d\n", __func__, ep_index, ep_type);

	if (is_usb_offload_enabled(xhci, vdev, ep_index))
		return true;

	if (is_usb_bulk_transfer_enabled(xhci, urb))
		return true;

	return false;
}

static void alloc_container_ctx(struct xhci_hcd *xhci, struct xhci_container_ctx *ctx,
				int type, gfp_t flags)
{
	ctx->bytes = dma_pool_zalloc(xhci->device_pool, flags, &ctx->dma);
	if (!ctx->bytes)
		xhci_err(xhci, "fail to allocate ctx->bytes\n");
}

static void free_container_ctx(struct xhci_hcd *xhci, struct xhci_container_ctx *ctx)
{
	dma_pool_free(xhci->device_pool, ctx->bytes, ctx->dma);
}

static struct xhci_vendor_ops ops = {
	.vendor_init = usb_audio_offload_init,
	.vendor_cleanup = usb_audio_offload_cleanup,
	.is_usb_offload_enabled = is_usb_offload_enabled,
	.queue_irq_work = queue_irq_work,
	.alloc_dcbaa = alloc_dcbaa,
	.free_dcbaa = free_dcbaa,
	.alloc_transfer_ring = alloc_transfer_ring,
	.free_transfer_ring = free_transfer_ring,
	.sync_dev_ctx = sync_dev_ctx,
	.usb_offload_skip_urb = usb_offload_skip_urb,
	.alloc_container_ctx = alloc_container_ctx,
	.free_container_ctx = free_container_ctx,
};

int xhci_vendor_helper_init(void)
{
	return xhci_exynos_register_vendor_ops(&ops);
}
