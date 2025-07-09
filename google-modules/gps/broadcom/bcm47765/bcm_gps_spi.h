/* SPDX-License-Identifier: GPL-2.0
 * Copyright 2015 Broadcom Corporation
 *
 * The Broadcom GPS SPI driver
 *
 */

#ifndef __BCM_GPS_SPI_H__
#define __BCM_GPS_SPI_H__

#include <linux/platform_data/sscoredump.h>

#define WORD_BURST_SIZE			4
#define CONFIG_SPI_DMA_BYTES_PER_WORD	4
#define CONFIG_SPI_DMA_BITS_PER_WORD	(CONFIG_SPI_DMA_BYTES_PER_WORD * 8)
#define MIN_DMA_SIZE			64
#define DELAY_FOR_SYSTEM_OVERLOADED_MS	100
#define READ_SIZE_FOR_SYSTEM_OVERLOADED	1024

#define SSI_MODE_STREAM		0x00
#define SSI_MODE_DEBUG		0x80

#define SSI_MODE_HALF_DUPLEX	0x00
#define SSI_MODE_FULL_DUPLEX	0x40

#define SSI_WRITE_TRANS		0x00
#define SSI_READ_TRANS		0x20

#define SSI_PCKT_1B_LENGTH        0
#define SSI_PCKT_2B_LENGTH        0x10

#define SSI_FLOW_CONTROL_DISABLED 0
#define SSI_FLOW_CONTROL_ENABLED  0x08

#define SSI_WRITE_HD (SSI_WRITE_TRANS | SSI_MODE_HALF_DUPLEX)
#define SSI_READ_HD  (SSI_READ_TRANS  | SSI_MODE_HALF_DUPLEX)

#define HSI_F_MOSI_CTRL_CNT_SHIFT  0
#define HSI_F_MOSI_CTRL_CNT_SIZE   3
#define HSI_F_MOSI_CTRL_CNT_MASK   0x07

#define HSI_F_MOSI_CTRL_SZE_SHIFT  3
#define HSI_F_MOSI_CTRL_SZE_SIZE   2
#define HSI_F_MOSI_CTRL_SZE_MASK   0x18

#define HSI_F_MOSI_CTRL_RSV_SHIFT  5
#define HSI_F_MOSI_CTRL_RSV_SIZE   1
#define HSI_F_MOSI_CTRL_RSV_MASK   0x20

#define HSI_F_MOSI_CTRL_PE_SHIFT   6
#define HSI_F_MOSI_CTRL_PE_SIZE    1
#define HSI_F_MOSI_CTRL_PE_MASK    0x40

#define HSI_F_MOSI_CTRL_PZC_SHIFT   0
#define HSI_F_MOSI_CTRL_PZC_SIZE    (HSI_F_MOSI_CTRL_CNT_SIZE + \
		HSI_F_MOSI_CTRL_SZE_SIZE + HSI_F_MOSI_CTRL_PE_SIZE)
#define HSI_F_MOSI_CTRL_PZC_MASK    (HSI_F_MOSI_CTRL_CNT_MASK | \
		HSI_F_MOSI_CTRL_SZE_MASK | HSI_F_MOSI_CTRL_PE_MASK)

/*
 * Transport receive buffer size, bytes
 * #define TRANSPORT_RX_BUFFER_SIZE 6144
 */
#define HSI_PZC_MAX_RX_BUFFER      6144

#define DEBUG_TIME_STAT
/* #define CONFIG_TRANSFER_STAT */
/* #define CONFIG_REG_IO */
#define CONFIG_PACKET_RECEIVED (31*1024)
/* rngdma_tx_end_addr_ptr */
#define CONFIG_RNGDMA_TX_END_ADDR_PTR_MIN (128)
#define WORK_TYPE_RX 1
#define WORK_TYPE_TX 2
#define WORK_TYPE_RXTX 3

#define CONFIG_MCU_WAKEUP

/* TODO: Use proper kerenl type */
struct bcm_spi_strm_protocol  {
	int pckt_len;
	int fc_len;
	int ctrl_len;
	unsigned char ctrl_byte;
	unsigned short frame_len;
} __attribute__((__packed__));


#ifdef CONFIG_TRANSFER_STAT
struct bcm_spi_transfer_stat  {
	int len_255;
	int len_1K;
	int len_2K;
	int len_4K;
	int len_8K;
	int len_16K;
	int len_32K;
	int len_64K;
	unsigned long len_total;
	unsigned long len_max;
	unsigned long len_min;
} __attribute__((__packed__));
#endif


/* class FailSafe : struct DataRecordList */
struct bcm_failsafe_data_recordlist {
	unsigned long start_addr;
	unsigned int size;
	int mem_type;
};

/*******************
 *
 *	Structs
 *
 *******************/

#define BCM_SPI_READ_BUF_SIZE	(16 * PAGE_SIZE)
#define BCM_SPI_WRITE_BUF_SIZE	(16 * PAGE_SIZE)

/* TODO: limit max payload to 254 because of exynos3 bug */
#define MAX_SPI_DREG_FRAME_LEN 254

/*
 * TODO: MAX_SPI_FRAME_LEN = 8K should be less TRANSPORT_RX_BUFFER_SIZE,
 * Now it is 12K (SWGNSSAND-1647)
 * #define MAX_SPI_FRAME_LEN (BCM_SPI_READ_BUF_SIZE / 8)
 *
 * TODO : hardcoded because of Tizen has PAGE_SIZE == 8K not 4K
 * as it is expected. Need to confirm.
 */
#define MAX_SPI_FRAME_LEN (1024 * 8)

struct bcm_ssi_tx_frame {
	unsigned char cmd;
	unsigned char data[MAX_SPI_FRAME_LEN-1];
} __attribute__((__packed__));

struct bcm_ssi_rx_frame {
	unsigned char status;
	unsigned char data[MAX_SPI_FRAME_LEN-1];
} __attribute__((__packed__));

struct bbd_device;

struct bcm_spi_priv {
	struct spi_device *spi;

	/* Char device stuff */
	struct miscdevice misc;
	bool busy;
	struct circ_buf read_buf;
	struct circ_buf write_buf;
	struct mutex rlock;			/* Lock for read_buf */
	struct mutex wlock;			/* Lock for write_buf */
	char _read_buf[BCM_SPI_READ_BUF_SIZE];
	char _write_buf[BCM_SPI_WRITE_BUF_SIZE];
	wait_queue_head_t poll_wait;		/* for poll */

	/* GPIO pins */
	int host_req;
	int mcu_req;
	int mcu_resp;
	int nstandby;

	/* IRQ and its control */
	atomic_t irq_enabled;
	spinlock_t irq_lock;

	/* Work */
	struct work_struct rxtx_work;
	struct workqueue_struct *serial_wq;
	atomic_t suspending;

	/* SPI tx/rx buf */
	struct bcm_ssi_tx_frame *tx_buf;
	struct bcm_ssi_rx_frame *rx_buf;

	/* 4775 SPI tx/rx strm protocol */
	struct bcm_spi_strm_protocol tx_strm;
	struct bcm_spi_strm_protocol rx_strm;

#ifdef CONFIG_TRANSFER_STAT
	struct bcm_spi_transfer_stat trans_stat[2];
#endif
	struct pinctrl *ts_pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;

	/* some chip-set(BCM4775) needs to skip validity check */
	bool skip_validity_check;

	bool irq_wakeup_enabled;
	/* struct wake_lock bcm_wake_lock; */

	/* To make sure  that interface is detected on chip side !=0 */
	unsigned long packet_received;

	/* Suspend/Resume semaphore */
	int ssi_pm_semaphore;

	/* Overrun counter */
	unsigned long skip_count;
	unsigned long last_tick;

	bool ssi_dbg;
	bool ssi_dbg_pzc;
	bool ssi_dbg_rng;

	/*
	 * Calculating TX transfer failure operation in bus driver,
	 * reset in bcm_ssi_open()
	 */
	int ssi_tx_fail;
	/* Calculating TX pzc retries, reset in bcm_ssi_open() */
	int ssi_tx_pzc_retries;
	/* Calculating TX pzc retry delays, reset in bcm_ssi_open() */
	int ssi_tx_pzc_retry_delays;

	unsigned long rx_buffer_avail_bytes;// = HSI_PZC_MAX_RX_BUFFER;
	/* Should be more MAX_SPI_FRAME_LEN. See below */
	struct bbd_device *bbd;
	/* To register ssrdump platform */
	struct platform_device sscd_dev;
	/* To register ssrdump driver */
	struct sscd_platform_data sscd_pdata;
};

/* bcm_gps_regs.cpp */
int bcm_dreg_write(struct bcm_spi_priv *priv, char *id, u8 offset, u8 *buf,
	  u8 size);
int bcm_dreg_read(struct bcm_spi_priv *priv, char *id, u8 offset, u8 *buf,
	  u8 size);
int bcm_ireg_write(struct bcm_spi_priv *priv, char *id, unsigned int regaddr,
		unsigned int regval);
int bcm_ireg_read(struct bcm_spi_priv *priv, char *id, unsigned int regaddr,
		unsigned int *regval, int n);

/* bcm_gps_spi.cpp */
int bcm_spi_sync(struct bcm_spi_priv *priv, void *tx_buf, void *rx_buf,
		int len, int bits_per_word);

#endif /* __BBD_H__ */
