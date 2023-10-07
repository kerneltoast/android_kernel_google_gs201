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
#ifndef __DW3000_CORE_H
#define __DW3000_CORE_H

#include "dw3000.h"

/* Maximum SPI bus speed when PLL is not yet locked */
#define DW3000_SPI_SLOW_HZ 3000000

#define DW3000_GPIO_COUNT 9 /* GPIO0 to GPIO8 */

/* DW3000 wake-up latency. At least 2ms is required. */
#define DW3000_WAKEUP_LATENCY_US 15000

/* Define DW3000 PDOA modes */
#define DW3000_PDOA_M0 0x0 /* PDOA mode is off */
#define DW3000_PDOA_M1 0x1 /* PDOA mode 1 */
#define DW3000_PDOA_M2 0x2 /* PDOA mode 2 (reserved or not supported) */
#define DW3000_PDOA_M3 0x3 /* PDOA mode 3 */
#define DW3000_PDOA_CONFIG_MASK 0x3

/* Define DW3000 STS modes */
#define DW3000_STS_MODE_OFF 0x0 /* STS is off */
#define DW3000_STS_MODE_1 0x1 /* STS mode 1 */
#define DW3000_STS_MODE_2 0x2 /* STS mode 2 */
#define DW3000_STS_MODE_ND 0x3 /* STS with no data */
#define DW3000_STS_BASIC_MODES_MASK 0x3 /* STS basic modes */
#define DW3000_STS_MODE_SDC 0x8 /* Enable Super Deterministic Codes */
#define DW3000_STS_CONFIG_MASK 0xB

/* Offset of each area in CIR accumulator memory */
#define DW3000_ACC_MEM_IPATOV_OFFSET 0x0 /* Base address */
#define DW3000_ACC_MEM_PRF64_SIZE 1016 /* Bank size when PRF=64 MHz */
#define DW3000_ACC_MEM_PRF16_SIZE 992 /* Bank size when PRF=16 MHz */
#define DW3000_ACC_MEM_STS_OFFSET \
	1024 /* Base address when STS enabled and pdoa != 3 */
#define DW3000_ACC_MEM_STS_SIZE \
	512 /* Bank size when STS enabled and pdoa != 3 */
#define DW3000_ACC_MEM_STS_PDOA3_OFFSET \
	(DW3000_ACC_MEM_STS_OFFSET +    \
	 DW3000_ACC_MEM_STS_SIZE) /* Base address when sts enabled and pdoa mode 3 */
#define DW3000_ACC_MEM_STS_PDOA3_SZ \
	512 /* Bank size when sts enabled and pdoa mode 3 */

/**
 * DW3000_GET_STS_LEN_UNIT_VALUE() - Convert STS length enum into unit value
 * @x: value from enum dw3000_sts_lengths
 *
 * Return: the STS length in unit of 8-symbols block.
 */
#define DW3000_GET_STS_LEN_UNIT_VALUE(x) ((u16)(1 << (x)))

/* Constants for specifying TX Preamble length in symbols */
#define DW3000_PLEN_4096 0x03 /* Standard preamble length 4096 symbols */
#define DW3000_PLEN_2048 0x0A /* Non-standard preamble length 2048 symbols */
#define DW3000_PLEN_1536 0x06 /* Non-standard preamble length 1536 symbols */
#define DW3000_PLEN_1024 0x02 /* Standard preamble length 1024 symbols */
#define DW3000_PLEN_512 0x0d /* Non-standard preamble length 512 symbols */
#define DW3000_PLEN_256 0x09 /* Non-standard preamble length 256 symbols */
#define DW3000_PLEN_128 0x05 /* Non-standard preamble length 128 symbols */
#define DW3000_PLEN_72 0x07 /* Non-standard length 72 */
#define DW3000_PLEN_32 0x04 /* Non-standard length 32 */
#define DW3000_PLEN_64 0x01 /* Standard preamble length 64 symbols */

/* Constants for specifying the (Nominal) mean Pulse Repetition Frequency
   These are defined for direct write (with a shift if necessary) to CHAN_CTRL
   and TX_FCTRL regs */
#define DW3000_PRF_16M 1 /* UWB PRF 16 MHz */
#define DW3000_PRF_64M 2 /* UWB PRF 64 MHz */
#define DW3000_PRF_SCP 3 /* SCP UWB PRF ~100 MHz */

/* Constants for specifying Start of Frame Delimiter (SFD) type */
#define DW3000_SFD_TYPE_STD 0 /* Standard short IEEE802154 SFD (8 symbols) */
#define DW3000_SFD_TYPE_DW_8 1 /* Decawave-defined 8 symbols  SFD */
#define DW3000_SFD_TYPE_DW_16 2 /* Decawave-defined 16 symbols SFD */
#define DW3000_SFD_TYPE_4Z 3 /* IEEE802154z SFD (8 symbols) */

/* Constants for selecting the bit rate for data TX (and RX)
   These are defined for write (with just a shift) the TX_FCTRL register */
#define DW3000_BR_850K 0 /* UWB bit rate 850 kbits/s */
#define DW3000_BR_6M8 1 /* UWB bit rate 6.8 Mbits/s */

/* Constants for selecting the PHR mode */
#define DW3000_PHRMODE_STD 0x0 /* standard PHR mode */
#define DW3000_PHRMODE_EXT 0x1 /* DW proprietary extended frames PHR mode */

/* Constants for selecting the bit rate for the PHR */
#define DW3000_PHRRATE_STD 0x0 /* standard PHR rate, 850 kbits/s */
#define DW3000_PHRRATE_DTA 0x1 /* PHR at data rate 6.8 Mbits/s */

/* Constants for specifying Preamble Acquisition Chunk (PAC) Size in symbols */
#define DW3000_PAC8 0 /* recommended for RX of preamble length  128 and below */
#define DW3000_PAC16 1 /* recommended for RX of preamble length 256 */
#define DW3000_PAC32 2 /* recommended for RX of preamble length 512 */
#define DW3000_PAC4 3 /* recommended for RX of preamble length < 127 */

enum spi_modes {
	DW3000_SPI_RD_BIT = 0x0000U,
	DW3000_SPI_WR_BIT = 0x8000U,
	DW3000_SPI_AND_OR_8 = 0x8001U,
	DW3000_SPI_AND_OR_16 = 0x8002U,
	DW3000_SPI_AND_OR_32 = 0x8003U,
	DW3000_SPI_AND_OR_MSK = 0x0003U,
};

/* Frame filtering configuration options */
#define DW3000_FF_ENABLE_802_15_4 0x2 /* use 802.15.4 filtering rules */
#define DW3000_FF_DISABLE 0x0 /* disable FF */
#define DW3000_FF_BEACON_EN 0x001 /* beacon frames allowed */
#define DW3000_FF_DATA_EN 0x002 /* data frames allowed */
#define DW3000_FF_ACK_EN 0x004 /* ack frames allowed */
#define DW3000_FF_MAC_EN 0x008 /* mac control frames allowed */
#define DW3000_FF_RSVD_EN 0x010 /* reserved frame types allowed */
#define DW3000_FF_MULTI_EN 0x020 /* multipurpose frames allowed */
#define DW3000_FF_FRAG_EN 0x040 /* fragmented frame types allowed */
#define DW3000_FF_EXTEND_EN 0x080 /* extended frame types allowed */
#define DW3000_FF_COORD_EN \
	0x100 /* behave as coordinator (can receive frames
				    with no dest address (need PAN ID match)) */
#define DW3000_FF_IMPBRCAST_EN 0x200 /* allow MAC implicit broadcast */

/* DW3000 soft reset options */
#define DW3000_RESET_ALL 0x00
#define DW3000_RESET_CTRX 0x0f
#define DW3000_RESET_RX 0xef
#define DW3000_RESET_CLEAR 0xff

/* SYS_STATE_LO register information */
/* TSE is in IDLE (IDLE_PLL) */
#define DW3000_SYS_STATE_IDLE 0x3
/* TSE is in TX but TX is in IDLE */
#define DW3000_SYS_STATE_TXERR 0xD0000

/* Fast commands */
/* Turn off TX or RX, clear any TX/RX events and put DW3000 into IDLE */
#define DW3000_CMD_TXRXOFF 0x0
/* Start TX */
#define DW3000_CMD_TX 0x1
/* Enable RX */
#define DW3000_CMD_RX 0x2
/* Start delayed TX (RMARKER will be @ time set in DX_TIME register) */
#define DW3000_CMD_DTX 0x3
/* Enable RX @ time specified in DX_TIME register */
#define DW3000_CMD_DRX 0x4
/* Start delayed TX (RMARKER will be @ time = TX_TIME + DX_TIME) */
#define DW3000_CMD_DTX_TS 0x5
/* Enable RX @ time = TX_TIME + DX_TIME */
#define DW3000_CMD_DRX_TS 0x6
/* Start delayed TX (RMARKER will be @ time = RX_TIME + DX_TIME) */
#define DW3000_CMD_DTX_RS 0x7
/* Enable RX @ time = RX_TIME + DX_TIME */
#define DW3000_CMD_DRX_RS 0x8
/* Start delayed TX (RMARKER will be @ time = DREF_TIME + DX_TIME) */
#define DW3000_CMD_DTX_REF 0x9
/* Enable RX @ time = DREF_TIME + DX_TIME */
#define DW3000_CMD_DRX_REF 0xa
/* Start delayed TX (as DTX below), enable RX when TX done */
#define DW3000_CMD_DTX_W4R 0xd
/* Start TX (as below), enable RX when TX done */
#define DW3000_CMD_TX_W4R 0xc
/* Toggle double buffer pointer */
#define DW3000_CMD_DB_TOGGLE 0x13
/* Write to the Semaphore and try to reserve access (if it hasn't already been
   reserved by the other master) */
#define DW3000_CMD_SEMA_REQ 0x14
/* Release the semaphore if it is currently reserved by this master. */
#define DW3000_CMD_SEMA_REL 0x15
/* Only SPI 2 can issue this command. Force access regardless of current
   semaphore value. */
#define DW3000_CMD_SEMA_FORCE 0x16
/* Global digital reset including of the semaphore */
#define DW3000_CMD_SEMA_RESET 0x18
/* Global digital reset without reset of the semaphore */
#define DW3000_CMD_SEMA_RESET_NO_SEM 0x19
/* Enters sleep/deep sleep according to ANA_CFG - DEEPSLEEP_EN */
#define DW3000_CMD_ENTER_SLEEP 0x1A

/* Size of RX LUT configuration tables */
#define DW3000_CONFIGMRXLUT_MAX 7
#define DW3000_DGC_CFG 0x38
#define DW3000_DGC_CFG0 0x00000240
#define DW3000_DGC_CFG1 0x1a491248
#define DW3000_DGC_CFG2 0x2db248db

/* DW3000 SLEEP and WAKEUP configuration parameters */
#define DW3000_PGFCAL 0x0800
#define DW3000_GOTORX 0x0200
#define DW3000_GOTOIDLE 0x0100
#define DW3000_SEL_GEAR3 0x00C0
#define DW3000_SEL_GEAR2 0x0080 /* Short gear table */
#define DW3000_SEL_GEAR1 0x0040 /* SCP */
#define DW3000_SEL_GEAR0 0x0000 /* Long gear table */
#define DW3000_ALT_GEAR 0x0020
#define DW3000_LOADLDO 0x0010
#define DW3000_LOADDGC 0x0008
#define DW3000_LOADBIAS 0x0004
#define DW3000_RUNSAR 0x0002

/* OTP addresses definitions */
#define DW3000_LDOTUNELO_ADDRESS (0x04)
#define DW3000_LDOTUNEHI_ADDRESS (0x05)
#define DW3000_PARTID_ADDRESS (0x06)
#define DW3000_LOTID_ADDRESS (0x07)
#define DW3000_VBAT_ADDRESS (0x08)
#define DW3000_VTEMP_ADDRESS (0x09)
#define DW3000_XTRIM_ADDRESS (0x1E)
#define DW3000_OTPREV_ADDRESS (0x1F)
#define DW3000_BIAS_TUNE_ADDRESS (0xA)
#define DW3000_DGC_TUNE_ADDRESS (0x20)
#define DW3000_PLL_CC_ADDRESS (0x35)

/* Bit fields to select information to retrieve from OTP memory */
#define DW3000_READ_OTP_PID 0x10 /* read part ID from OTP */
#define DW3000_READ_OTP_LID 0x20 /* read lot ID from OTP */
#define DW3000_READ_OTP_BAT 0x40 /* read ref voltage from OTP */
#define DW3000_READ_OTP_TMP 0x80 /* read ref temperature from OTP */

/* The mean XTAL TRIM value measured on multiple E0 samples.
 * During the initialization the XTAL TRIM value can be read from the OTP and
 * in case it is not present, the default would be used instead. */
#define DW3000_DEFAULT_XTAL_TRIM 0x1f

/* The XTAL TRIM BIAS value for +/- 5PPM offset */
#define DW3000_XTAL_BIAS 0

/* Clock offset value under which the PDoA value is assumed bad. */
#define DW3000_CFO_THRESHOLD ((s16)(4 * (1 << 26) / 1000000))

/* All RX errors mask */
#define DW3000_SYS_STATUS_ALL_RX_ERR                                           \
	(DW3000_SYS_STATUS_RXPHE_BIT_MASK | DW3000_SYS_STATUS_RXFCE_BIT_MASK | \
	 DW3000_SYS_STATUS_RXFSL_BIT_MASK | DW3000_SYS_STATUS_RXSTO_BIT_MASK | \
	 DW3000_SYS_STATUS_ARFE_BIT_MASK | DW3000_SYS_STATUS_CIAERR_BIT_MASK | \
	 DW3000_SYS_STATUS_CPERR_BIT_MASK |                                    \
	 DW3000_SYS_STATUS_LCSSERR_BIT_MASK)

/* User defined RX timeouts (frame wait timeout and preamble detect timeout)
   mask. */
#define DW3000_SYS_STATUS_ALL_RX_TO \
	(DW3000_SYS_STATUS_RXFTO_BIT_MASK | DW3000_SYS_STATUS_RXPTO_BIT_MASK)

/* All RX events after a correct packet reception mask */
#define DW3000_SYS_STATUS_ALL_RX_GOOD                                         \
	(DW3000_SYS_STATUS_RXFR_BIT_MASK | DW3000_SYS_STATUS_RXFCG_BIT_MASK | \
	 DW3000_SYS_STATUS_RXPRD_BIT_MASK |                                   \
	 DW3000_SYS_STATUS_RXSFDD_BIT_MASK |                                  \
	 DW3000_SYS_STATUS_RXPHD_BIT_MASK |                                   \
	 DW3000_SYS_STATUS_CIA_DONE_BIT_MASK)

/* All TX events mask */
#define DW3000_SYS_STATUS_ALL_TX                                               \
	(DW3000_SYS_STATUS_AAT_BIT_MASK | DW3000_SYS_STATUS_TXFRB_BIT_MASK |   \
	 DW3000_SYS_STATUS_TXPRS_BIT_MASK | DW3000_SYS_STATUS_TXPHS_BIT_MASK | \
	 DW3000_SYS_STATUS_TXFRS_BIT_MASK)

void dw3000_init_config(struct dw3000 *dw);

int dw3000_init(struct dw3000 *dw, bool check_idlerc);
void dw3000_remove(struct dw3000 *dw);

int dw3000_transfers_init(struct dw3000 *dw);
void dw3000_transfers_free(struct dw3000 *dw);

void dw3000_spitests(struct dw3000 *dw);
bool dw3000_spitests_enabled(struct dw3000 *dw);

int dw3000_wait_idle_state(struct dw3000 *dw);
int dw3000_poweron(struct dw3000 *dw);
int dw3000_poweroff(struct dw3000 *dw);
int dw3000_hardreset(struct dw3000 *dw);

int dw3000_softreset(struct dw3000 *dw);
int dw3000_check_devid(struct dw3000 *dw);

void dw3000_setup_regulators(struct dw3000 *dw);
int dw3000_setup_reset_gpio(struct dw3000 *dw);
int dw3000_setup_irq(struct dw3000 *dw);
int dw3000_setup_wifi_coex(struct dw3000 *dw);
int dw3000_setup_thread_cpu(struct dw3000 *dw, int *dw3000_thread_cpu);
int dw3000_setup_qos_latency(struct dw3000 *dw);
int dw3000_setup_regulator_delay(struct dw3000 *dw);

void dw3000_spi_queue_start(struct dw3000 *dw);
int dw3000_spi_queue_flush(struct dw3000 *dw);
int dw3000_spi_queue_reset(struct dw3000 *dw, int rc);

int dw3000_reg_read_fast(struct dw3000 *dw, u32 reg_fileid, u16 reg_offset,
			 u16 length, void *buffer);
int dw3000_reg_read32(struct dw3000 *dw, u32 reg_fileid, u16 reg_offset,
		      u32 *val);
int dw3000_reg_read16(struct dw3000 *dw, u32 reg_fileid, u16 reg_offset,
		      u16 *val);
int dw3000_reg_read8(struct dw3000 *dw, u32 reg_fileid, u16 reg_offset,
		     u8 *val);
int dw3000_reg_write_fast(struct dw3000 *dw, u32 reg_fileid, u16 reg_offset,
			  u16 length, const void *buffer, enum spi_modes mode);
int dw3000_reg_write32(struct dw3000 *dw, u32 reg_fileid, u16 reg_offset,
		       u32 val);
int dw3000_reg_write16(struct dw3000 *dw, u32 reg_fileid, u16 reg_offset,
		       u16 val);
int dw3000_reg_write8(struct dw3000 *dw, u32 reg_fileid, u16 reg_offset,
		      u8 val);
int dw3000_reg_modify32(struct dw3000 *dw, u32 reg_fileid, u16 reg_offset,
			u32 _and, u32 _or);
int dw3000_reg_modify16(struct dw3000 *dw, u32 reg_fileid, u16 reg_offset,
			u16 _and, u16 _or);
int dw3000_reg_modify8(struct dw3000 *dw, u32 reg_fileid, u16 reg_offset,
		       u8 _and, u8 _or);

int dw3000_write_fastcmd(struct dw3000 *dw, u8 cmd);

int dw3000_xfer(struct dw3000 *dw, u32 reg_fileid, u16 reg_offset, u16 length,
		void *buffer, enum spi_modes mode);
#define dw3000_reg_or8(dw, addr, offset, or_val) \
	dw3000_reg_modify8(dw, addr, offset, -1, or_val)
#define dw3000_reg_and8(dw, addr, offset, and_val) \
	dw3000_reg_modify8(dw, addr, offset, and_val, 0)

#define dw3000_reg_or16(dw, addr, offset, or_val) \
	dw3000_reg_modify16(dw, addr, offset, -1, or_val)
#define dw3000_reg_and16(dw, addr, offset, and_val) \
	dw3000_reg_modify16(dw, addr, offset, and_val, 0)

#define dw3000_reg_or32(dw, addr, offset, or_val) \
	dw3000_reg_modify32(dw, addr, offset, -1, or_val)
#define dw3000_reg_and32(dw, addr, offset, and_val) \
	dw3000_reg_modify32(dw, addr, offset, and_val, 0)

#define sts_to_pdoa(s, pdoa_enabled) \
	((s) && (pdoa_enabled) ? DW3000_PDOA_M1 : DW3000_PDOA_M0)

#define dw3000_get_chip_name(dw) (dw3000_chip_versions[dw->chip_idx].name)

int dw3000_enable(struct dw3000 *dw);
int dw3000_disable(struct dw3000 *dw);

int dw3000_configure_chan(struct dw3000 *dw);
int dw3000_configure_pcode(struct dw3000 *dw);
int dw3000_configure_sfd_type(struct dw3000 *dw);
int dw3000_configure_phr_rate(struct dw3000 *dw);
int dw3000_configure_preamble_length_and_datarate(struct dw3000 *dw,
						  bool update_sfd_toc_pac);

int dw3000_set_eui64(struct dw3000 *dw, __le64 val);
int dw3000_set_panid(struct dw3000 *dw, __le16 val);
int dw3000_set_shortaddr(struct dw3000 *dw, __le16 val);
int dw3000_set_pancoord(struct dw3000 *dw, bool active);
int dw3000_set_promiscuous(struct dw3000 *dw, bool on);
int dw3000_set_sts_pdoa(struct dw3000 *dw, u8 sts_mode, u8 pdoa_mode);
int dw3000_set_sts_length(struct dw3000 *dw, enum dw3000_sts_lengths len);
int dw3000_configure_sts_key(struct dw3000 *dw, const u8 *key);
int dw3000_configure_sts_iv(struct dw3000 *dw, const u8 *iv);
int dw3000_load_sts_iv(struct dw3000 *dw);
int dw3000_configure_sys_cfg(struct dw3000 *dw, struct dw3000_config *config);
int dw3000_configure_hw_addr_filt(struct dw3000 *dw, unsigned long changed);
int dw3000_enable_auto_fcs(struct dw3000 *dw, bool on);

int dw3000_clear_sys_status(struct dw3000 *dw, u32 clear_bits);
int dw3000_clear_dss_status(struct dw3000 *dw, u8 clear_bits);
int dw3000_clear_spi_collision_status(struct dw3000 *dw, u8 clear_bits);
int dw3000_read_rx_timestamp(struct dw3000 *dw, u64 *rx_ts);
int dw3000_read_rdb_status(struct dw3000 *dw, u8 *status);
int dw3000_read_sys_status(struct dw3000 *dw, u32 *status);
int dw3000_read_sys_time(struct dw3000 *dw, u32 *sys_time);

int dw3000_rx_store_rssi(struct dw3000 *dw, struct dw3000_rssi *rssi,
			 u8 pkt_sts);
int dw3000_rx_calc_rssi(struct dw3000 *dw, struct dw3000_rssi *rssi,
			struct mcps802154_rx_frame_info *info, u8 pkt_sts);
int dw3000_rx_stats_inc(struct dw3000 *dw, const enum dw3000_stats_items item,
			struct dw3000_rssi *rssi);

u32 dw3000_get_dtu_time(struct dw3000 *dw);

int dw3000_forcetrxoff(struct dw3000 *dw);

int dw3000_do_rx_enable(struct dw3000 *dw,
			const struct mcps802154_rx_frame_config *config,
			int frame_idx);
int dw3000_rx_enable(struct dw3000 *dw, bool rx_delayed, u32 date_dtu,
		     u32 timeout_pac);
int dw3000_rx_disable(struct dw3000 *dw);
bool dw3000_rx_busy(struct dw3000 *dw, bool busy);

int dw3000_rx_stats_enable(struct dw3000 *dw, bool on);
void dw3000_rx_stats_clear(struct dw3000 *dw);

int dw3000_enable_autoack(struct dw3000 *dw, bool force);
int dw3000_disable_autoack(struct dw3000 *dw, bool force);

struct mcps802154_tx_frame_config;
int dw3000_do_tx_frame(struct dw3000 *dw,
		       const struct mcps802154_tx_frame_config *config,
		       struct sk_buff *skb, int frame_idx);

int dw3000_tx_setcwtone(struct dw3000 *dw, bool on);

int dw3000_config_antenna_gpios(struct dw3000 *dw);
int dw3000_set_tx_antenna(struct dw3000 *dw, int ant_set_id);
int dw3000_set_rx_antennas(struct dw3000 *dw, int ant_set_id,
			   bool pdoa_enabled, int frame_idx);

s16 dw3000_read_pdoa(struct dw3000 *dw);
s16 dw3000_pdoa_to_aoa_lut(struct dw3000 *dw, s16 pdoa_rad_q11);
int dw3000_read_sts_timestamp(struct dw3000 *dw, u64 *sts_ts);
int dw3000_read_sts_quality(struct dw3000 *dw, s16 *acc_qual);
int dw3000_read_clockoffset(struct dw3000 *dw, s16 *cfo);
int dw3000_prog_xtrim(struct dw3000 *dw);

int dw3000_set_gpio_mode(struct dw3000 *dw, u32 mask, u32 mode);
int dw3000_set_gpio_dir(struct dw3000 *dw, u16 mask, u16 dir);
int dw3000_set_gpio_out(struct dw3000 *dw, u16 reset, u16 set);

int dw3000_otp_read32(struct dw3000 *dw, u16 addr, u32 *val);
int dw3000_otp_write32(struct dw3000 *dw, u16 addr, u32 data);
int dw3000_read_otp(struct dw3000 *dw, int mode);

int dw3000_read_frame_cir_data(struct dw3000 *dw,
			       struct mcps802154_rx_frame_info *info,
			       u64 utime);
int dw3000_cir_data_alloc_count(struct dw3000 *dw, u16 nrecord);

void dw3000_sysfs_init(struct dw3000 *dw);
void dw3000_sysfs_remove(struct dw3000 *dw);

void dw3000_isr(struct dw3000 *dw);
enum hrtimer_restart dw3000_idle_timeout(struct hrtimer *timer);
int dw3000_idle_cancel_timer(struct dw3000 *dw);
void dw3000_wakeup_timer_start(struct dw3000 *dw, int delay_us);
void dw3000_wakeup_and_wait(struct dw3000 *dw);
int dw3000_deep_sleep_and_wakeup(struct dw3000 *dw, int delay_us);
int dw3000_idle(struct dw3000 *dw, bool timestamp, u32 timestamp_dtu,
		dw3000_idle_timeout_cb idle_timeout_cb,
		enum operational_state next_operational_state);
int dw3000_deepsleep_wakeup_now(struct dw3000 *dw,
				dw3000_idle_timeout_cb idle_timeout_cb,
				u32 timestamp_dtu,
				enum operational_state next_operational_state);
int dw3000_can_deep_sleep(struct dw3000 *dw, int delay_us);
int dw3000_trace_rssi_info(struct dw3000 *dw, u32 regid, char *chipver);

int dw3000_testmode_continuous_tx_start(struct dw3000 *dw, u32 frame_length,
					u32 rate);
int dw3000_testmode_continuous_tx_stop(struct dw3000 *dw);
int dw3000_nfcc_coex_prepare_config(struct dw3000 *dw);
int dw3000_nfcc_coex_restore_config(struct dw3000 *dw);

/* Preamble length related information. */
struct dw3000_plen_info {
	/* Preamble length in symbols. */
	int symb;
	/* PAC size in symbols. */
	int pac_symb;
	/* Register value for this preamble length. */
	uint8_t dw_reg;
	/* Register value for PAC size. */
	uint8_t dw_pac_reg;
};
extern const struct dw3000_plen_info _plen_info[];

/* Chip per symbol information. */
extern const int _chip_per_symbol_info[];

/* PRF related information. */
struct dw3000_prf_info {
	/* Number of chips per symbol. */
	int chip_per_symb;
};
extern const struct dw3000_prf_info _prf_info[];

static inline int dw3000_get_sfd_symb(struct dw3000 *dw)
{
	/*
	 * SFD_TYPE_STD, SFD_TYPE_DW_8, SFD_TYPE_4Z : 8 symbols
	 * SFD_TYPE_DW_16 : 16 symbols
	 */
	return dw->config.sfdType == DW3000_SFD_TYPE_DW_16 ? 16 : 8;
}

static inline int dw3000_compute_shr_dtu(struct dw3000 *dw)
{
	const struct dw3000_plen_info *plen_info =
		&_plen_info[dw->config.txPreambLength - 1];
	const int chip_per_symb =
		_prf_info[dw->config.txCode >= 9 ? DW3000_PRF_64M :
						   DW3000_PRF_16M]
			.chip_per_symb;
	const int shr_symb = plen_info->symb + dw3000_get_sfd_symb(dw);
	return shr_symb * chip_per_symb / DW3000_CHIP_PER_DTU;
}

static inline int compute_shr_dtu_from_conf(
	const struct mcps802154_hrp_uwb_params *hrp_uwb_params)
{
	const int preamble_symb = hrp_uwb_params->psr;
	const int chip_per_symb =
		_prf_info[hrp_uwb_params->prf == MCPS802154_PRF_64 ?
				  DW3000_PRF_64M :
				  DW3000_PRF_16M]
			.chip_per_symb;
	/* The only possible sfd number of symbols is 8. */
	const int sfd_symb = 8;
	const int shr_symb = preamble_symb + sfd_symb;
	return shr_symb * chip_per_symb / DW3000_CHIP_PER_DTU;
}

static inline int dw3000_compute_symbol_dtu(struct dw3000 *dw)
{
	const int chip_per_symb =
		_prf_info[dw->config.txCode >= 9 ? DW3000_PRF_64M :
						   DW3000_PRF_16M]
			.chip_per_symb;
	return chip_per_symb / DW3000_CHIP_PER_DTU;
}

static inline int dw3000_compute_chips_per_pac(struct dw3000 *dw)
{
	const int pac_symb = _plen_info[dw->config.txPreambLength - 1].pac_symb;
	const int chip_per_symb =
		_prf_info[dw->config.txCode >= 9 ? DW3000_PRF_64M :
						   DW3000_PRF_16M]
			.chip_per_symb;
	return chip_per_symb * pac_symb;
}

static inline int dw3000_compute_pre_timeout_pac(struct dw3000 *dw)
{
	/* Must be called AFTER dw->chips_per_pac initialisation */
	const int symb = _plen_info[dw->config.txPreambLength - 1].symb;
	const int pac_symb = _plen_info[dw->config.txPreambLength - 1].pac_symb;

	return (DW3000_RX_ENABLE_STARTUP_DLY * DW3000_CHIP_PER_DLY +
		dw->chips_per_pac - 1) /
		       dw->chips_per_pac +
	       symb / pac_symb + 2;
}

static inline int dw3000_frame_duration_dtu(struct dw3000 *dw,
					    int payload_bytes, bool with_shr)
{
	const struct dw3000_prf_info *prf_info =
		&_prf_info[dw->config.txCode >= 9 ? DW3000_PRF_64M :
						    DW3000_PRF_16M];
	int chip_per_symbol_phr = _chip_per_symbol_info[dw->config.phrRate];
	int chip_per_symbol_data = _chip_per_symbol_info[dw->config.dataRate];
	/* STS part */
	const u8 sts_mode = dw->config.stsMode & DW3000_STS_BASIC_MODES_MASK;
	const int sts_symb =
		sts_mode == DW3000_STS_MODE_OFF ? 0 : 8 << dw->config.stsLength;
	const int sts_chips = sts_symb * prf_info->chip_per_symb;
	/* PHR part. */
	const int phr_tail_bits = sts_mode == DW3000_STS_MODE_ND ? 0 : 19 + 2;
	const int phr_chips = phr_tail_bits /* 1 bit/symbol */
			      * chip_per_symbol_phr;
	/* Data part, 48 Reed-Solomon bits per 330 bits. */
	const int data_bits = sts_mode == DW3000_STS_MODE_ND ?
				      0 :
				      (payload_bytes + IEEE802154_FCS_LEN) * 8;

	const int data_rs_bits = data_bits + (data_bits + 329) / 330 * 48;
	const int data_chips = data_rs_bits /* 1 bit/symbol */
			       * chip_per_symbol_data;
	/* Done, convert to dtu. */
	return ((sts_chips + phr_chips + data_chips) / DW3000_CHIP_PER_DTU) +
	       (with_shr ? dw->llhw->shr_dtu : 0);
}

static inline void dw3000_update_timings(struct dw3000 *dw)
{
	struct mcps802154_llhw *llhw = dw->llhw;
	/* Update configuration dependent timings */
	llhw->shr_dtu = dw3000_compute_shr_dtu(dw);
	llhw->symbol_dtu = dw3000_compute_symbol_dtu(dw);
	/* The CCA detection time shall be equivalent to 40 data symbol periods,
	   Tdsym, for a nominal 850 kb/s, or equivalently, at least 8 (multiplexed)
	   preamble symbols should be captured in the CCA detection time. */
	llhw->cca_dtu = 8 * llhw->symbol_dtu;
	dw->chips_per_pac = dw3000_compute_chips_per_pac(dw);
	dw->pre_timeout_pac = dw3000_compute_pre_timeout_pac(dw);
}

/**
 * dw3000_dtu_to_ktime() - compute absolute ktime for the specified DTU time
 * @dw: the DW device
 * @ts_dtu: timestamp in DTU unit to convert
 *
 * Formula:
 * ktime = (ts_dtu - dtu0) * D / N + ktime0
 * Where:
 * N = DW3000_DTU_FREQ = 15600000
 * D = 1000000000
 *
 * D/N = 1000000000/15600000 = 10000/156
 * dtu0 always 0.
 *
 * Return: the computed kernel time in ns
 */
static inline s64 dw3000_dtu_to_ktime(struct dw3000 *dw, u32 ts_dtu)
{
	return dw->time_zero_ns +
	       (10000ll * ts_dtu / (DW3000_DTU_FREQ / 100000));
}

/**
 * dw3000_ktime_to_dtu() - compute current DTU time
 * @dw: the DW device
 * @timestamp_ns: kernel time in ns
 *
 * Formula:
 * dtu = (ktime - ktime0) * N / D + dtu0
 * Where:
 * N = DW3000_DTU_FREQ = 15600000
 * D = 1000000000
 *
 * N/D = 15600000/1000000000 = 156/10000
 * dtu0 always 0.
 *
 * Return: driver DTU time
 */
static inline u32 dw3000_ktime_to_dtu(struct dw3000 *dw, s64 timestamp_ns)
{
	timestamp_ns -= dw->time_zero_ns;
	return (u32)(timestamp_ns * (DW3000_DTU_FREQ / 100000) / 10000);
}

/**
 * dw3000_dtu_to_sys_time() - compute DW SYS_TIME from DTU time
 * @dw: the DW device
 * @dtu: the DTU timestamp to convert to SYS_TIME
 *
 * Return: the value to write to SYS_TIME register
 */
static inline u32 dw3000_dtu_to_sys_time(struct dw3000 *dw, u32 dtu)
{
	const int N = DW3000_DTU_PER_SYS_POWER;
	u32 dtu_sync = dw->dtu_sync;
	u32 sys_time_sync = dw->sys_time_sync;
	return ((dtu - dtu_sync) << N) + sys_time_sync;
}

/**
 * dw3000_sys_time_to_dtu() - compute current DTU time from SYS_TIME
 * @dw: the DW device
 * @sys_time: the DW device SYS_TIME register value to convert to DTU
 * @dtu_near: a DTU time which must be in the past relative to sys_time, at less
 * than half the SYS_TIME rollover period
 *
 * Return: the corresponding DTU time
 */
static inline u32 dw3000_sys_time_to_dtu(struct dw3000 *dw, u32 sys_time,
					 u32 dtu_near)
{
	const int N = DW3000_DTU_PER_SYS_POWER;
	u32 dtu_sync = dw->dtu_sync;
	u32 sys_time_sync = dw->sys_time_sync;
	u32 dtu_lsb = (sys_time - (sys_time_sync - (dtu_sync << N))) >> N;
	u32 dtu_add = ((~dtu_lsb & dtu_near) & (1 << (31 - N))) << 1;
	u32 mask = (1 << (32 - N)) - 1;
	return ((dtu_near & ~mask) | dtu_lsb) + dtu_add;
}

/**
 * dw3000_sys_time_rctu_to_dtu() - compute current DTU time from RCTU.
 * @dw: the DW device.
 * @timestamp_rctu: The DW device RX_STAMP register value in RCTU to convert to DTU.
 * The RCTU, Ranging Counter Time Unit, is approximately 15.65 picoseconds long.
 *
 * Return: The corresponding DTU time.
 */
static inline u32 dw3000_sys_time_rctu_to_dtu(struct dw3000 *dw,
					      u64 timestamp_rctu)
{
	u32 sys_time = (u32)(timestamp_rctu / DW3000_RCTU_PER_SYS);
	u32 dtu_near = dw3000_get_dtu_time(dw) - DW3000_DTU_FREQ;

	return dw3000_sys_time_to_dtu(dw, sys_time, dtu_near);
}

/**
 * dw3000_reset_rctu_conv_state() - reset RCTU converter
 * @dw: the DW device
 *
 * Called during a stop, rx_disable, reset and idle.
 */
static inline void dw3000_reset_rctu_conv_state(struct dw3000 *dw)
{
	dw->rctu_conv.state = UNALIGNED;
}

/**
 * dw3000_resync_rctu_conv_state() - resync RCTU converter
 * @dw: the DW device
 *
 * Called during a wake-up from deep sleep.
 */
static inline void dw3000_resync_rctu_conv_state(struct dw3000 *dw)
{
	if (dw->rctu_conv.state == ALIGNED_SYNCED)
		dw->rctu_conv.state = ALIGNED;
}

static inline int pac_to_dly(struct mcps802154_llhw *llhw, int pac)
{
	struct dw3000 *dw = llhw->priv;

	return (pac * dw->chips_per_pac / DW3000_CHIP_PER_DLY);
}

static inline int dtu_to_pac(struct mcps802154_llhw *llhw, int timeout_dtu)
{
	struct dw3000 *dw = llhw->priv;

	return (timeout_dtu * DW3000_CHIP_PER_DTU + dw->chips_per_pac - 1) /
	       dw->chips_per_pac;
}

static inline int dtu_to_dly(struct mcps802154_llhw *llhw, int dtu)
{
	return (dtu * DW3000_CHIP_PER_DTU / DW3000_CHIP_PER_DLY);
}

#endif /* __DW3000_CORE_H */
