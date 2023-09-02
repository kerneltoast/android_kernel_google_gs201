/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2020-2021 Qorvo US, Inc.
 *
 * This software is provided under the GNU General Public License, version 2
 * (GPLv2), as well as under a Qorvo commercial license.
 *
 * You may choose to use this software under the terms of the GPLv2 License,
 * version 2 ("GPLv2"), as published by the Free Software Foundation.
 * You should have received a copy of the GPLv2 along with this program.  If
 * not, see <http://www.gnu.org/licenses/>.
 *
 * This program is distributed under the GPLv2 in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GPLv2 for more
 * details.
 *
 * If you cannot meet the requirements of the GPLv2, you may not use this
 * software for any purpose without first obtaining a commercial license from
 * Qorvo. Please contact Qorvo to inquire about licensing terms.
 */
#include <linux/version.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/of.h>

#include "dw3000.h"
#include "dw3000_pm.h"
#include "dw3000_core.h"
#include "dw3000_stm.h"
#include "dw3000_mcps.h"
#include "dw3000_debugfs.h"

/* Default value for auto_deep_sleep_margin.
 * Set to -1 (disabled) until we want to have deep-sleep enabled by default. */
#define DW3000_AUTO_DEEP_SLEEP_MARGIN_US -1

int dw3000_qos_latency = 0;

unsigned dw3000_regulator_delay_us = 1000;

static int dw3000_lna_pa_mode = 0;
module_param_named(lna_pa_mode, dw3000_lna_pa_mode, int, 0444);
MODULE_PARM_DESC(
	lna_pa_mode,
	"Configure LNA/PA mode. May conflict with WiFi coexistence GPIO number, 0 for disabled (default)");

/**
 * dw3000_spi_probe() - Probe and initialize DW3000 SPI device
 * @spi: the SPI device to probe and initialize
 *
 * Called when module is loaded and an SPI controller driver exist.
 * This function allocates private device structure and then initialize
 *
 * - SPI, HRTimer, waitqueue,
 * - sysfs/debugfs,
 * - regulators, GPIO & IRQ from DT,
 * - IRQ & events processing thread,
 *
 * Then, if no errors, the device is registered to MCPS upper-layer and
 * the processing thread is started to do device probing.
 *
 * Return: 0 if device probed correctly else a negative error.
 */
static int dw3000_spi_probe(struct spi_device *spi)
{
	struct dw3000 *dw;
	int rc;
	int dw3000_thread_cpu;

	/* Allocate MCPS 802.15.4 device */
	dw = dw3000_mcps_alloc(&spi->dev);
	if (!dw) {
		rc = -ENOMEM;
		goto err_alloc_hw;
	}
	dw->llhw->hw->parent = &spi->dev;
	spi_set_drvdata(spi, dw);
	dw->spi = spi;

	/* Initialization of the wifi coex parameters */
	rc = dw3000_setup_wifi_coex(dw);
	if (rc != 0)
		goto err_wifi_coex;

	/* Initialization of thread cpu */
	rc = dw3000_setup_thread_cpu(dw, &dw3000_thread_cpu);
	if (rc != 0)
		goto err_thread_cpu;

	/* Initialization of qos latency */
	rc = dw3000_setup_qos_latency(dw);
	if (rc != 0)
		goto err_qos_latency;

	/* Initialization of regulator delay */
	rc = dw3000_setup_regulator_delay(dw);
	if (rc != 0)
		goto err_regulator_delay;

	dw->lna_pa_mode = (s8)dw3000_lna_pa_mode;
#if (KERNEL_VERSION(4, 13, 0) <= LINUX_VERSION_CODE)
#if (KERNEL_VERSION(5, 9, 0) <= LINUX_VERSION_CODE)
	dw->spi_pid = spi->controller->kworker->task->pid;
#else
	dw->spi_pid = spi->controller->kworker.task->pid;
#endif
#else
	dw->spi_pid = spi->master->kworker.task->pid;
#endif
	dw->auto_sleep_margin_us = DW3000_AUTO_DEEP_SLEEP_MARGIN_US;
	dw->current_operational_state = DW3000_OP_STATE_OFF;
	init_waitqueue_head(&dw->operational_state_wq);
	/* Initialization of the idle timer for wakeup. */
	hrtimer_init(&dw->idle_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	dw->idle_timer.function = dw3000_idle_timeout;

	dev_info(dw->dev, "Loading driver...");
	dw3000_sysfs_init(dw);

	/* Setup SPI parameters */
	dev_info(dw->dev, "setup mode: %d, %u bits/w, %u Hz max\n",
		 (int)(spi->mode & (SPI_CPOL | SPI_CPHA)), spi->bits_per_word,
		 spi->max_speed_hz);
	dev_info(dw->dev, "can_dma: %d\n", spi->master->can_dma != NULL);
	spi->bits_per_word = 8;
#if (KERNEL_VERSION(5, 3, 0) <= LINUX_VERSION_CODE)
	spi->rt = 1;
#endif
#if (KERNEL_VERSION(5, 9, 0) <= LINUX_VERSION_CODE)
	/* Quirk to force spi_set_cs() in spi_setup() to do something!
	   !!! Not doing this results in CS line stay LOW and SPIRDY IRQ
	   isn't fired later when powering-on the device. Previous kernel
	   don't have this bug as it always apply new CS state. */
	spi->master->last_cs_enable = true;
#endif
	/* Save configured device max speed */
	dw->of_max_speed_hz = spi->max_speed_hz;
	/* Setup SPI and put CS line in HIGH state! */
	rc = spi_setup(spi);
	if (rc != 0)
		goto err_spi_setup;

	/* Request and setup regulators if availables*/
	dw3000_setup_regulators(dw);

	/* Request and setup the reset GPIO pin */
	/* This leave the DW3000 in reset state until dw3000_hardreset() put
	   the GPIO back in input mode.
	   This also ensure no spurious IRQ fire during the dw3000_setup_irq()
	   call below. (If the GPIO state is well maintained low). */
	rc = dw3000_setup_reset_gpio(dw);
	if (rc != 0)
		goto err_setup_gpios;

	/* Allocate pre-computed SPI messages for fast access some registers */
	rc = dw3000_transfers_init(dw);
	if (rc != 0)
		goto err_transfers_init;

	/* Initialise state descriptor */
	/* This ensure wait queue exist before IRQ handler is setup in case
	   of spurious IRQ (mainly because hw problem with reset GPIO). */
	rc = dw3000_state_init(dw, dw3000_thread_cpu);
	if (rc != 0) {
		dev_err(dw->dev, "state machine initialisation failed: %d\n",
			rc);
		goto err_state_init;
	}

	dev_info(dw->dev, "SPI pid: %u, dw3000 pid: %u\n", dw->spi_pid,
		 dw->dw3000_pid);
	/* Request and setup the irq GPIO pin */
	rc = dw3000_setup_irq(dw);
	if (rc != 0)
		goto err_setup_irq;

	/*
	 * Initialize PM QoS. Using the default latency won't change anything
	 * to the QoS list
	 */
	dw3000_pm_qos_add_request(dw, PM_QOS_DEFAULT_VALUE);

	/* Start state machine & initialise device using high-prio thread */
	rc = dw3000_state_start(dw);
	if (rc != 0)
		goto err_state_start;

	/* Debugfs interface */
	rc = dw3000_debugsfs_init(dw);
	if (rc != 0)
		goto err_debugfs;

	/* Register MCPS 802.15.4 device */
	rc = dw3000_mcps_register(dw);
	if (rc != 0) {
		dev_err(&spi->dev, "could not register: %d\n", rc);
		goto err_register_hw;
	}

	/* All is ok */
	return 0;

err_register_hw:
	dw3000_debugfs_remove(dw);
err_debugfs:
err_state_start:
	dw3000_pm_qos_remove_request(dw);
err_setup_irq:
	dw3000_state_stop(dw);
err_state_init:
	dw3000_transfers_free(dw);
err_transfers_init:
err_setup_gpios:
err_spi_setup:
	dw3000_sysfs_remove(dw);
err_regulator_delay:
err_qos_latency:
err_thread_cpu:
err_wifi_coex:
	dw3000_mcps_free(dw);
	spi_set_drvdata(spi, NULL);
err_alloc_hw:
	return rc;
}

/**
 * dw3000_spi_remove() - Remove DW3000 SPI device
 * @spi: the SPI device to remove
 *
 * Called when module is unloaded, this function removes all
 * sysfs/debugfs files, unregister device from the MCPS
 * module and them free all remaining resources.
 *
 * Return: always 0
 */
static int dw3000_spi_remove(struct spi_device *spi)
{
	struct dw3000 *dw = spi_get_drvdata(spi);

	if (dw == NULL)
		/* Error during probe, all already freed */
		return 0;

	dev_dbg(dw->dev, "unloading...");

	/* Remove sysfs files */
	dw3000_debugfs_remove(dw);
	dw3000_sysfs_remove(dw);
	/* Unregister subsystems */
	dw3000_mcps_unregister(dw);
	/* Stop state machine */
	dw3000_state_stop(dw);
	dw3000_pm_qos_remove_request(dw);
	/* Free pre-computed SPI messages */
	dw3000_transfers_free(dw);
	/* Release the mcps 802.15.4 device */
	dw3000_cir_data_alloc_count(dw, 0);
	dw3000_mcps_free(dw);

	return 0;
}

enum { DW3000,
};

static const struct of_device_id dw3000_of_ids[] = {
	{ .compatible = "decawave,dw3000", .data = (void *)DW3000 },
	{},
};
MODULE_DEVICE_TABLE(of, dw3000_of_ids);

static const struct spi_device_id dw3000_spi_ids[] = {
	{ "dw3000", DW3000 },
	{},
};
MODULE_DEVICE_TABLE(spi, dw3000_spi_ids);

static struct spi_driver dw3000_driver = {
	.driver = {
		.name = "dw3000",
		.of_match_table = of_match_ptr(dw3000_of_ids),
	},
	.id_table = dw3000_spi_ids,
	.probe = dw3000_spi_probe,
	.remove = dw3000_spi_remove,
};
module_spi_driver(dw3000_driver);

#ifdef GITVERSION
MODULE_VERSION(GITVERSION);
#endif
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("David Girault <david.girault@qorvo.com>");
MODULE_DESCRIPTION("DecaWave DW3000 IEEE 802.15.4 driver");
