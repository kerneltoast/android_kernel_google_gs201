// SPDX-License-Identifier: GPL-2.0
/* Copyright 2015 Broadcom Corporation
 *
 * The Broadcom GPS SPI driver
 *
 */

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/sysrq.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/kthread.h>
#include <linux/circ_buf.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/poll.h>
#include <linux/uaccess.h>
#include <linux/suspend.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/miscdevice.h>
#include <linux/time.h>
#include <linux/io.h>
#include <asm/irq.h>
#include <linux/kernel_stat.h>

#include "bbd.h"
#include "bcm_gps_spi.h"


/**
 * bcm_dreg_write - write to direct addressable registers in command controller
 * @priv:           @bcm_spi_priv structure data
 * @id:             name of indirect register, just for debug print
 * @offset:         the command offset of the direct addressable register
 * @buf:            pointer of data to be written to the register
 * @size:           number of bytes to write
 * Return:          upon successful completion, this function returns number of
 *                  written bytes otherwise exit with specific error code <0
 */
int bcm_dreg_write(struct bcm_spi_priv *priv, char *id, u8 offset, u8 *buf, u8 size)
{

	int i;
	struct bcm_ssi_tx_frame *tx = priv->tx_buf;
	struct bcm_ssi_rx_frame *rx = priv->rx_buf;

	if  (size > MAX_SPI_DREG_FRAME_LEN)
		return -1;

	/*
	 * writing in 1 transaction
	 * 0x80 : < CHWXX > = half-duplex, SPI write command packet
	 */
	tx->cmd = SSI_MODE_DEBUG | SSI_MODE_HALF_DUPLEX | SSI_WRITE_TRANS;

	/*
	 * bit-8 is a write transaction and the lower 7-bits is
	 * the command offset of the
	 */
	tx->data[0] = offset & 0x7F;
	/* the number of valid bytes available on MOSI following this byte */
	tx->data[1] = size;
	memcpy(&tx->data[2], buf, size);
	rx->status = 0;

	if (bcm_spi_sync(priv, tx, rx, size + 3, size + 3))
		return -1;

	for (i = 0; i < size; i++) {
		dev_dbg(&priv->spi->dev, "regW REG %s @ [%02X]: %08X", id,
				offset, buf[i]);
	}

	return size;
}


/**
 * bcm_dreg_read - read direct addressable registers in command controller
 * @priv:          @bcm_spi_priv structure data
 * @offset:        the command offset of the direct addressable register.
 * @buf            pointer data to be read from the register
 * @size           number of bytes to read
 * Return:         upon successful completion, this function returns number of
 *                 read bytes otherwise exit with specific error code <0
 */
int bcm_dreg_read(struct bcm_spi_priv *priv, char *id, u8 offset, u8 *buf, u8 size)
{
	/* Reading in 2 transactions */
	int i = 0;
	/* int status; */
	struct bcm_ssi_tx_frame *tx = priv->tx_buf;
	struct bcm_ssi_rx_frame *rx = priv->rx_buf;

	if  (size > MAX_SPI_DREG_FRAME_LEN)
		return -1;

	/*
	 * First transaction will setup SPI Command Logic
	 * to Read Data from Register.
	 * 0x80 : < CHWXX > = half-duplex, SPI write command packet
	 */
	tx->cmd = SSI_MODE_DEBUG | SSI_MODE_HALF_DUPLEX | SSI_WRITE_TRANS;

	/*
	 * <8?h1000_0100> = Bit-8 is a read transaction and the lower 7-bits
	 * is the command offset of the register
	 */
	tx->data[0] = 0x80 | (offset & 0x7F);
	/*
	 * The number of bytes host will read from this offset
	 * in next packet
	 */
	tx->data[1] = size;
	rx->status = 0;

	if (bcm_spi_sync(priv, tx, rx, 3, 3))
		return -1;

	dev_dbg(&priv->spi->dev, "regR: REG(W) %s @ [%02X]: %08X ", id,
			offset, size);

	/*
	 *  Second Transaction will read data
	 *  0xa0 : < CHRXX > = half-duplex, SPI read command packet
	 */
	tx->cmd = SSI_MODE_DEBUG | SSI_MODE_HALF_DUPLEX | SSI_READ_TRANS;
	/*
	 * READ : the number of valid read bytes available plus one
	 * to account for the read offset address that accompanies
	 * the read data == <cmdByteNum+1>
	 */
	tx->data[0] = 0;
	/* READ : the command offset of the register == <cmdRegOffset>. */
	tx->data[1] = 0;
	rx->status  = 0;

	memset(&tx->data[2], 0, size);

	if (bcm_spi_sync(priv, tx, rx, size + 3, size + 3))
		return -1;

	memcpy(buf, &rx->data[2], size);

	for (i = 0 ; i < size ; i++)
		dev_dbg(&priv->spi->dev, "regR: REG(R) %s @ [%02X]: %08X", id,
			offset, buf[i]);

	return size;
}


/**
 * bcm_ireg_write - write to indirect addressable register
 * @priv:           @bcm_spi_priv structure data
 * @id:             name of indirect register, just for debug print
 * @regaddr:        the stream address of the indirect addressable register.
 * @regval:         pointer data to be read from the register
 * Return:          upon successful completion, this function returns number
 *                  of written bytes otherwise exit with specific error code <0
 */
int bcm_ireg_write(struct bcm_spi_priv *priv, char *id, u32 regaddr, u32 regval)
{
	union long_union_t  swap_addr, swap_reg;
	struct bcm_ssi_tx_frame *tx = priv->tx_buf;
	struct bcm_ssi_rx_frame *rx = priv->rx_buf;

	/*
	 * Writing in 2 transactions
	 * First transaction will set up the SPI Debug Logic to Write Data
	 * into Configuration Register or Memory Location.
	 *
	 * 0x80 : <Command Byte>
	 * 0xD1 : <SPI Address Left Shifted with Write Bit as LSB>
	 * 0x00 : < SPI Offset of DMA Start Addr >
	 * 0x00, 0x00, 0x00, 0x00 : < Start Address>,
	 *                          Should be set from 'regaddr'
	 * 0x04, 0x00 : <Number of bytes to write>
	 * 0x01 : <Write Enable>
	 *
	 * uint8_t transaction_1st[13] = {
	 * 0x80, 0xD1,  0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00,  0x03 };
	 * uint8_t transaction_1st[19] = {
	 * 0x00, 0x00,  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	 * 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	 */

	swap_addr.ul = regaddr;
	swap_reg.ul  = regval;

	/*
	 * First transaction will setup SPI Command Logic
	 * to Read Data from Register.
	 * HSI_MOSI_COMMAND_PCKT | HSI_MOSI_HALF_DUPLEX | HSI_MOSI_WRITE_TRANS;
	 * 0x80 : < CHWXX > = half-duplex, SPI write command packet
	 */
	tx->cmd = 0x80;
	/*
	 * <8’h0100_0000> = Bit-8 is a write transaction.
	 * The lower 7-bits is the command offset of
	 */
	tx->data[0] = 0x20;
	/*
	 * The register (CONFIG_REG_DATA) the host is starting to write from.
	 * the number of valid bytes available on MOSI following this byte
	 */
	tx->data[1] = 9;

	tx->data[2] = swap_reg.uc[0];
	tx->data[3] = swap_reg.uc[1];
	tx->data[4] = swap_reg.uc[2];
	tx->data[5] = swap_reg.uc[3];

	tx->data[6] = swap_addr.uc[0];
	tx->data[7] = swap_addr.uc[1];
	tx->data[8] = swap_addr.uc[2];
	tx->data[9] = swap_addr.uc[3];

	tx->data[10] = 0x03;
	tx->data[11] = 0x00;
	tx->data[12] = 0x00;
	tx->data[13] = 0x00;
	rx->status = 0;

	if (bcm_spi_sync(priv, tx, rx, 15, 15))
		return -1;

	if (id)
		dev_dbg(&priv->spi->dev, "reg32w: %s @ : [%08X] %08X ", id,
				(unsigned int)swap_addr.ul,
				(unsigned int)swap_reg.ul);

	return 1;
}

/**
 * bcm_ireg_read - read indirect addressable register
 * @priv:          @bcm_spi_priv structure data
 * @id:            the name of indirect register. Just for debug print.
 * @regaddr:       the stream address of the indirect addressable register.
 * @regval:        the pointer data to be read from the register
 * Return:         upon successful completion, this function returns number of
 *                 read bytes otherwise exit with specific error code <0
 */
int bcm_ireg_read(struct bcm_spi_priv *priv, char *id, u32 regaddr,
	  u32 *regval, s32 n)
{
	s32 i;
	union long_union_t  swap_addr, swap_reg;
	union long_union_t  swap_addr2;
	struct bcm_ssi_tx_frame *tx = priv->tx_buf;
	struct bcm_ssi_rx_frame *rx = priv->rx_buf;

	for (i = 0; i < n; i++) {
		swap_addr.ul = regaddr + (i * 4);

		/*
		 * First transaction will setup SPI Command Logic
		 * to Read Data from Register.
		 * HSI_MOSI_COMMAND_PCKT |
		 * HSI_MOSI_HALF_DUPLEX |
		 * HSI_MOSI_WRITE_TRANS;
		 * 0x80 : < CHWXX > = half-duplex, SPI write command packet
		 */
		tx->cmd = 0x80;

		/*
		 * <8?h0100_0000> = Bit-8 is a write transaction.
		 * The lower 7-bits is the command offset of
		 */
		tx->data[0] = 0x24;
		/*
		 * The register (RFIFO Read DATA) the host is
		 * starting to write from. the number of valid bytes
		 * available on MOSI following this byte
		 */
		tx->data[1] = 5;
		tx->data[2] = swap_addr.uc[0];
		tx->data[3] = swap_addr.uc[1];
		tx->data[4] = swap_addr.uc[2];
		tx->data[5] = swap_addr.uc[3];
		tx->data[6] = 0x02;   /* AHB read transaction */
		rx->status = 0;

		if (bcm_spi_sync(priv, tx, rx, 8, 8))
			return -1;

		tx->cmd = 0xa0;
		memset(tx->data, 0, 11);
		rx->status = 0;

		if (bcm_spi_sync(priv, tx, rx, 12, 8))
			return -1;

		swap_addr2.uc[0] = rx->data[1];
		swap_addr2.uc[1] = rx->data[2];
		swap_addr2.uc[2] = rx->data[3];
		swap_addr2.uc[3] = rx->data[4];

		swap_reg.uc[0] = rx->data[5];
		swap_reg.uc[1] = rx->data[6];
		swap_reg.uc[2] = rx->data[7];
		swap_reg.uc[3] = rx->data[8];

		if (id)
			dev_dbg(&priv->spi->dev, "reg32r: %s @ : [%08X] %08X ", id,
				(unsigned int)swap_addr2.ul,
				(unsigned int)swap_reg.ul);

		if (regval)
			*regval++ = swap_reg.ul;

	}

	return i;
}
