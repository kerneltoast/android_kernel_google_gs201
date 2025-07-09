/*
 * Broadcom SPI Host Controller Driver - Linux Per-port
 *
 * Copyright (C) 2025, Broadcom.
 *
 *      Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2 (the "GPL"),
 * available at http://www.broadcom.com/licenses/GPLv2.php, with the
 * following added to such license:
 *
 *      As a special exception, the copyright holders of this software give you
 * permission to link this software with independent modules, and to copy and
 * distribute the resulting executable under terms of your choice, provided that
 * you also meet, for each linked independent module, the terms and conditions of
 * the license of that module.  An independent module is a module which is not
 * derived from this software.  The special exception does not apply to any
 * modifications of the software.
 *
 *
 * <<Broadcom-WL-IPTag/Open:>>
 *
 * $Id$
 */

#include <typedefs.h>
#include <bcmutils.h>

#include <bcmsdbus.h>		/* bcmsdh to/from specific controller APIs */
#include <sdiovar.h>		/* to get msglevel bit values */

#include <bcmsdh.h>
#include <bcmspibrcm.h>
#include <linux/spi/spi.h>

struct sdos_info {
	sdioh_info_t *sd;
	spinlock_t lock;
};

static struct spi_device *gBCMSPI = NULL;

extern int bcmsdh_probe(struct device *dev);
extern int bcmsdh_remove(struct device *dev);

static int bcmsdh_spi_probe(struct spi_device *spi_dev)
{
	int ret = 0;

	gBCMSPI = spi_dev;

#ifdef SPI_PIO_32BIT_RW
	spi_dev->bits_per_word = 32;
#else
	spi_dev->bits_per_word = 8;
#endif /* SPI_PIO_32BIT_RW */
	ret = spi_setup(spi_dev);

	if (ret) {
		sd_err(("bcmsdh_spi_probe: spi_setup fail with %d\n", ret));
	}
	sd_err(("bcmsdh_spi_probe: spi_setup with %d, bits_per_word=%d\n",
		ret, spi_dev->bits_per_word));
	ret = bcmsdh_probe(&spi_dev->dev);

	return ret;
}

static int  bcmsdh_spi_remove(struct spi_device *spi_dev)
{
	int ret = 0;

	ret = bcmsdh_remove(&spi_dev->dev);
	gBCMSPI = NULL;

	return ret;
}

static struct spi_driver bcmsdh_spi_driver = {
	.probe		= bcmsdh_spi_probe,
	.remove		= bcmsdh_spi_remove,
	.driver		= {
		.name = "wlan_spi",
		.bus    = &spi_bus_type,
		.owner  = THIS_MODULE,
		},
};

/*
 * module init
*/
int bcmsdh_register_client_driver(void)
{
	int error = 0;
	sd_trace(("bcmsdh_gspi: %s Enter\n", __FUNCTION__));

	error = spi_register_driver(&bcmsdh_spi_driver);

	return error;
}

/*
 * module cleanup
*/
void bcmsdh_unregister_client_driver(void)
{
	sd_trace(("%s Enter\n", __FUNCTION__));
	spi_unregister_driver(&bcmsdh_spi_driver);
}

/* Register with Linux for interrupts */
int
spi_register_irq(sdioh_info_t *sd, uint irq)
{
	return SUCCESS;
}

/* Free Linux irq */
void
spi_free_irq(uint irq, sdioh_info_t *sd)
{
}

/* Map Host controller registers */

int
spi_osinit(sdioh_info_t *sd)
{
	struct sdos_info *sdos;

	sdos = (struct sdos_info*)MALLOC(sd->osh, sizeof(struct sdos_info));
	sd->sdos_info = (void*)sdos;
	if (sdos == NULL)
		return BCME_NOMEM;

	sdos->sd = sd;
	spin_lock_init(&sdos->lock);
	return BCME_OK;
}

void
spi_osfree(sdioh_info_t *sd)
{
	struct sdos_info *sdos;
	ASSERT(sd && sd->sdos_info);

	sdos = (struct sdos_info *)sd->sdos_info;
	MFREE(sd->osh, sdos, sizeof(struct sdos_info));
}

/* Interrupt enable/disable */
SDIOH_API_RC
sdioh_interrupt_set(sdioh_info_t *sd, bool enable)
{
	ulong flags;
	struct sdos_info *sdos;

	sd_trace(("%s: %s\n", __FUNCTION__, enable ? "Enabling" : "Disabling"));

	sdos = (struct sdos_info *)sd->sdos_info;
	ASSERT(sdos);

	if (!(sd->host_init_done && sd->card_init_done)) {
		sd_err(("%s: Card & Host are not initted - bailing\n", __FUNCTION__));
		return SDIOH_API_RC_FAIL;
	}

	/* Ensure atomicity for enable/disable calls */
	spin_lock_irqsave(&sdos->lock, flags);

	sd->client_intr_enabled = enable;

	spin_unlock_irqrestore(&sdos->lock, flags);

	return SDIOH_API_RC_SUCCESS;
}

/* Protect against reentrancy (disable device interrupts while executing) */
void
spi_lock(sdioh_info_t *sd)
{
	ulong flags;
	struct sdos_info *sdos;

	sdos = (struct sdos_info *)sd->sdos_info;
	ASSERT(sdos);

	sd_trace(("%s: %d\n", __FUNCTION__, sd->lockcount));

	spin_lock_irqsave(&sdos->lock, flags);
	if (sd->lockcount) {
		sd_err(("%s: Already locked!\n", __FUNCTION__));
		ASSERT(sd->lockcount == 0);
	}
	if (sd->client_intr_enabled)
		bcmsdh_oob_intr_set(0);
	sd->lockcount++;
	spin_unlock_irqrestore(&sdos->lock, flags);
}

/* Enable client interrupt */
void
spi_unlock(sdioh_info_t *sd)
{
	ulong flags;
	struct sdos_info *sdos;

	sd_trace(("%s: %d, %d\n", __FUNCTION__, sd->lockcount, sd->client_intr_enabled));
	ASSERT(sd->lockcount > 0);

	sdos = (struct sdos_info *)sd->sdos_info;
	ASSERT(sdos);

	spin_lock_irqsave(&sdos->lock, flags);
	if (--sd->lockcount == 0 && sd->client_intr_enabled) {
		bcmsdh_oob_intr_set(1);
	}
	spin_unlock_irqrestore(&sdos->lock, flags);
}

int bcmgspi_dump = 0;		/* Set to dump complete trace of all SPI bus transactions */

static void
hexdump(char *pfx, unsigned char *msg, int msglen)
{
	int i, col;
	char buf[80];

	ASSERT(strlen(pfx) + 49 <= sizeof(buf));

	col = 0;

	for (i = 0; i < msglen; i++, col++) {
		if (col % 16 == 0)
			strcpy(buf, pfx);
		sprintf(buf + strlen(buf), "%02x", msg[i]);
		if ((col + 1) % 16 == 0)
			printf("%s\n", buf);
		else
			sprintf(buf + strlen(buf), " ");
	}

	if (col % 16 != 0)
		printf("%s\n", buf);
}

/* Send/Receive an SPI Packet */
void
spi_sendrecv(sdioh_info_t *sd, uint8 *msg_out, uint8 *msg_in, int msglen)
{
	int write = 0;
	int tx_len = 0;
	struct spi_message msg;
	struct spi_transfer t[2];

	spi_message_init(&msg);
	memset(t, 0, 2*sizeof(struct spi_transfer));

	if (sd->wordlen == 2)
#if !(defined(SPI_PIO_RW_BIGENDIAN) && defined(SPI_PIO_32BIT_RW))
		write = msg_out[2] & 0x80;	/* XXX bit 7: read:0, write :1 */
#else
		write = msg_out[1] & 0x80;	/* XXX bit 7: read:0, write :1 */
#endif /* !(defined(SPI_PIO_RW_BIGENDIAN) && defined(SPI_PIO_32BIT_RW)) */
	if (sd->wordlen == 4)
#if !(defined(SPI_PIO_RW_BIGENDIAN) && defined(SPI_PIO_32BIT_RW))
		write = msg_out[0] & 0x80;	/* XXX bit 7: read:0, write :1 */
#else
		write = msg_out[3] & 0x80;	/* XXX bit 7: read:0, write :1 */
#endif /* !(defined(SPI_PIO_RW_BIGENDIAN) && defined(SPI_PIO_32BIT_RW)) */

	if (bcmgspi_dump) {
		hexdump(" OUT: ", msg_out, msglen);
	}

	tx_len = write ? msglen-4 : 4;

	sd_trace(("spi_sendrecv: %s, wordlen %d, cmd : 0x%02x 0x%02x 0x%02x 0x%02x\n",
		write ? "WR" : "RD", sd->wordlen,
		msg_out[0], msg_out[1], msg_out[2], msg_out[3]));

	t[0].tx_buf = (char *)&msg_out[0];
	t[0].rx_buf = 0;
	t[0].len = tx_len;

	spi_message_add_tail(&t[0], &msg);

	t[1].rx_buf = (char *)&msg_in[tx_len];
	t[1].tx_buf = 0;
	t[1].len = msglen-tx_len;

	spi_message_add_tail(&t[1], &msg);
	spi_sync(gBCMSPI, &msg);

	if (bcmgspi_dump) {
		hexdump(" IN  : ", msg_in, msglen);
	}
}
