/* SPDX-License-Identifier: GPL-2.0
 * Copyright 2014 Broadcom Corporation
 *
 * The BBD (Broadcom Bridge Driver)
 *
 */

#ifndef __BBD_H__
#define __BBD_H__

#define BBD_PWR_STATUS

union long_union_t {
	u8 uc[sizeof(u32)];
	u32 ul;
} __packed __aligned(4);

union short_union_t {
	u8  uc[sizeof(u16)];
	u16 us;
} __packed __aligned(4);

enum {
	BBD_MINOR_SHMD	    = 0,
	BBD_MINOR_SENSOR    = 1,
	BBD_MINOR_CONTROL   = 2,
	BBD_MINOR_PATCH     = 3,
#ifdef BBD_PWR_STATUS
	BBD_MINOR_PWRSTAT   = 4,
#endif
	/* BBD_MINOR_SSI_SPI_DEBUG = 5, */ /* NOT supported yet */
	BBD_DEVICE_INDEX
};

#define BBD_MAX_DATA_SIZE	 4096  /* max data size for transition  */

#define BBD_CTRL_RESET_REQ	"BBD:RESET_REQ"
#define ESW_CTRL_READY		"ESW:READY"
#define ESW_CTRL_NOTREADY	"ESW:NOTREADY"
#define ESW_CTRL_CRASHED	"ESW:CRASHED"
#define BBD_CTRL_DEBUG_ON       "BBD:DEBUG=1"
#define BBD_CTRL_DEBUG_OFF      "BBD:DEBUG=0"
#define SSP_DEBUG_ON		"SSP:DEBUG=1"
#define SSP_DEBUG_OFF		"SSP:DEBUG=0"
#define SSI_DEBUG_ON		"SSI:DEBUG=1"
#define SSI_DEBUG_OFF		"SSI:DEBUG=0"
#define PZC_DEBUG_ON		"PZC:DEBUG=1"
#define PZC_DEBUG_OFF		"PZC:DEBUG=0"
#define RNG_DEBUG_ON		"RNG:DEBUG=1"
#define RNG_DEBUG_OFF		"RNG:DEBUG=0"
#define BBD_CTRL_SSI_PATCH_BEGIN	"SSI:PatchBegin"
#define BBD_CTRL_SSI_PATCH_END		"SSI:PatchEnd"
#define GPSD_SENSOR_ON		"GPSD:SENSOR_ON"
#define GPSD_SENSOR_OFF		"GPSD:SENSOR_OFF"
#define GPSD_CORE_ON		"GPSD:CORE_ON"
#define GPSD_CORE_OFF		"GPSD:CORE_OFF"

/* #define DEBUG_1HZ_STAT */

#define HSI_ERROR_STATUS                  0x2C
#define HSI_ERROR_STATUS_LPBK_ERROR       0x01
#define HSI_ERROR_STATUS_STRM_FIFO_OVFL   0x02
#define HSI_ERROR_STATUS_AHB_BUS_ERROR    0x04
#define HSI_ERROR_STATUS_PATCH_ERROR      0x10
#define HSI_ERROR_STATUS_ALL_ERRORS       (HSI_ERROR_STATUS_LPBK_ERROR|\
		HSI_ERROR_STATUS_STRM_FIFO_OVFL|HSI_ERROR_STATUS_AHB_BUS_ERROR)

#define HSI_STATUS                          0x30
#define HSI_INTR_MASK                 0x40104034
#define HSI_RNGDMA_RX_BASE_ADDR       0x40104040
#define HSI_RNGDMA_RX_SW_ADDR_OFFSET  0x40104050
#define HSI_RNGDMA_TX_BASE_ADDR       0x40104060
#define HSI_RNGDMA_TX_SW_ADDR_OFFSET  0x40104070
#define HSI_CTRL                      0x40104080
#define HSI_RESETN                    0x40104090
#define HSI_ADL_ABR_CONTROL           0x401040a0
#define HSI_STRM_FIFO_STATUS          0x40104100
#define HSI_CMND_FIFO_STATUS          0x40104104

#ifdef DEBUG_1HZ_STAT

enum {
	STAT_TX_LHD = 0,
	STAT_TX_SSP,
	STAT_TX_RPC,
	STAT_TX_TL,
	STAT_TX_SSI,

	STAT_RX_SSI,
	STAT_RX_TL,
	STAT_RX_RPC,
	STAT_RX_SSP,
	STAT_RX_LHD,

	STAT_MAX
};


struct bbd_stat {
	bool enabled;

	u64 ts_irq;

	u64 min_rx_lat; /* = (u64)-1 */
	u64 min_rx_dur; /* = (u64)-1 */

	u64 max_rx_lat; /* = 0 */
	u64 max_rx_dur; /* = 0 */

	u64 stat[STAT_MAX];

	struct timer_list timer;
	struct work_struct work;
	struct workqueue_struct *workq;
};

extern struct bbd_stat stat1hz;

void bbd_update_stat(int index, unsigned int count);
void bbd_enable_stat(void);
void bbd_disable_stat(void);
#endif

#ifdef BBD_PWR_STATUS
enum {
	STAT_GPS_OFF = 0,
	STAT_GPS_ON,
	STAT_GPS_MAX
};
#endif /* BBD_PWR_STATUS */

/** callback for incoming data from 477x to senser hub driver **/
struct bbd_callbacks {
	int (*on_packet)(void *ssh_data, const char *buf, size_t size);
	int (*on_packet_alarm)(void *ssh_data);
	int (*on_control)(void *ssh_data, const char *str_ctrl);
	int (*on_mcu_ready)(void *ssh_data, bool ready);
};

extern void	bbd_register(void *ext_data, struct bbd_callbacks *pcallbacks);
extern ssize_t	bbd_send_packet(unsigned char *buf, size_t size);
extern ssize_t	bbd_pull_packet(
		unsigned char *buf, size_t size, unsigned int timeout_ms);
extern int	bbd_mcu_reset(void);
extern int	bbd_init(struct device *dev, bool legacy_patch);

#endif /* __BBD_H__ */
