/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __DT_GOOGLE_AOC_H
#define __DT_GOOGLE_AOC_H

#define I2S_0_RX		0x80000000
#define I2S_0_TX		0xC0000001
#define I2S_1_RX		0x80000002
#define I2S_1_TX		0xC0000003
#define I2S_2_RX		0x80000004
#define I2S_2_TX		0xC0000005
#define TDM_0_RX		0x80000006
#define TDM_0_TX		0xC0000007
#define TDM_1_RX		0x80000008
#define TDM_1_TX		0xC0000009
#define INTERNAL_MIC_TX	        0xC000000A
#define BT_RX			0x8000000B
#define BT_TX			0xC000000C
#define USB_RX			0x8000000D
#define USB_TX			0xC000000E
#define INCALL_RX		0x8000000F
#define INCALL_TX		0xC0000010
#define HAPTIC_RX		0x80000011
#define ERASER_TX	        0xC0000012
#define INTERNAL_MIC_US_TX	0xC0000013
#define DP_DMA_RX		0x80000014

#define IDX_EP1_RX		0x0
#define IDX_EP2_RX		0x1
#define IDX_EP3_RX		0x2
#define IDX_EP4_RX		0x3
#define IDX_EP5_RX		0x4
#define IDX_EP6_RX		0x5
#define IDX_EP7_RX		0x6
#define IDX_EP8_RX              0x7
#define IDX_NOHOST1_RX		0x20000008
#define IDX_RAW_RX              0x9
#define IDX_VOIP_RX             0xa
#define IDX_INCALL_PB0_RX       0xb
#define IDX_INCALL_PB1_RX       0xc
#define IDX_HIFI_RX             0x10
#define IDX_HAPTIC_NoHOST_RX	0x20000011
#define IDX_US_RX		0x13
#define IDX_INCALL_PB2_RX       0x14
#define IDX_IMSV_RX             0x15
#define IDX_CAP_INJ_RX          0x16
#define IDX_DP_DMA_NoHOST_RX	0x20000018

#define IDX_EP1_TX		0x40000000
#define IDX_EP2_TX		0x40000001
#define IDX_EP3_TX		0x40000002
#define IDX_EP4_TX		0x40000003
#define IDX_EP5_TX		0x40000004
#define IDX_EP6_TX		0x40000005
#define IDX_EP7_TX		0x40000006
#define IDX_EP8_TX		0x40000007
#define IDX_NOHOST1_TX		0x60000008
#define IDX_RAW_TX	        0x40000009
#define IDX_VOIP_TX		0x4000000a
#define IDX_INCALL_CAP0_TX	0x4000000d
#define IDX_INCALL_CAP1_TX	0x4000000e
#define IDX_INCALL_CAP2_TX	0x4000000f
#define IDX_HIFI_TX	        0x40000010
#define IDX_ANDROID_AEC_TX	0x40000012
#define IDX_HOTWORD_TAP_TX	0x40000017
#define IDX_INCALL_CAP3_TX	0x40000019
#endif /* __DT_GOOGLE_AOC_H */
