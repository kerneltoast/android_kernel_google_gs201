// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2022 Samsung Electronics.
 *
 */

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>

#include "gnss_prj.h"
#include "gnss_utils.h"


void gif_configure_irq(struct gnss_irq *irq, unsigned int num, const char *name,
	unsigned long flags)
{
	spin_lock_init(&irq->lock);
	irq->num = num;
	strncpy(irq->name, name, sizeof(irq->name) - 1);
	irq->flags = flags;
	gif_info("name:%s num:%d flags:0x%08lX\n", name, num, flags);
}

int gif_request_irq(struct gnss_irq *irq, irq_handler_t isr, void *data)
{
	int ret;

	ret = request_irq(irq->num, isr, irq->flags, irq->name, data);
	if (ret) {
		gif_err("%s: ERR! request_irq fail (%d)\n", irq->name, ret);
		return ret;
	}

	irq->active = irq->flags & IRQF_NO_AUTOEN ? false : true;
	irq->registered = true;

	gif_info("%s(#%d) handler registered (flags:0x%08lX)\n",
		irq->name, irq->num, irq->flags);

	return 0;
}

void gif_enable_irq(struct gnss_irq *irq)
{
	unsigned long flags;

	spin_lock_irqsave(&irq->lock, flags);

	if (irq->active)
		goto exit;

	enable_irq(irq->num);

	irq->active = true;

exit:
	spin_unlock_irqrestore(&irq->lock, flags);
}

void gif_disable_irq_nosync(struct gnss_irq *irq)
{
	unsigned long flags;

	if (irq->registered == false)
		return;

	spin_lock_irqsave(&irq->lock, flags);

	if (!irq->active)
		goto exit;
	disable_irq_nosync(irq->num);
	irq->active = false;

exit:
	spin_unlock_irqrestore(&irq->lock, flags);
}

void gif_disable_irq_sync(struct gnss_irq *irq)
{
	if (irq->registered == false)
		return;

	spin_lock(&irq->lock);

	if (!irq->active) {
		spin_unlock(&irq->lock);
		gif_debug("%s(#%d) is not active <%ps>\n",
				irq->name, irq->num, CALLER);
		return;
	}

	spin_unlock(&irq->lock);

	disable_irq(irq->num);

	spin_lock(&irq->lock);
	irq->active = false;
	spin_unlock(&irq->lock);

	gif_debug("%s(#%d) is disabled <%ps>\n",
			irq->name, irq->num, CALLER);
}
