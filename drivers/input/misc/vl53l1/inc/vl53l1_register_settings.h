/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53L1 and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */




#ifndef _VL53L1_REGISTER_SETTINGS_H_
#define _VL53L1_REGISTER_SETTINGS_H_






#define VL53L1_DEVICESCHEDULERMODE_PSEUDO_SOLO  0x00
#define VL53L1_DEVICESCHEDULERMODE_STREAMING    0x01
#define VL53L1_DEVICESCHEDULERMODE_HISTOGRAM    0x02





#define VL53L1_DEVICEREADOUTMODE_SINGLE_SD        (0x00 << 2)
#define VL53L1_DEVICEREADOUTMODE_DUAL_SD          (0x01 << 2)
#define VL53L1_DEVICEREADOUTMODE_SPLIT_READOUT    (0x02 << 2)
#define VL53L1_DEVICEREADOUTMODE_SPLIT_MANUAL     (0x03 << 2)






#define VL53L1_DEVICEMEASUREMENTMODE_MODE_MASK          0xF0
#define VL53L1_DEVICEMEASUREMENTMODE_STOP_MASK          0x0F

#define VL53L1_GROUPEDPARAMETERHOLD_ID_MASK             0x02



#define VL53L1_EWOK_I2C_DEV_ADDR_DEFAULT                0x29

#define VL53L1_OSC_FREQUENCY                            0x00
#define VL53L1_OSC_TRIM_DEFAULT                         0x00
#define VL53L1_OSC_FREQ_SET_DEFAULT                     0x00

#define VL53L1_RANGE_HISTOGRAM_REF                      0x08
#define VL53L1_RANGE_HISTOGRAM_RET                      0x10
#define VL53L1_RANGE_HISTOGRAM_BOTH                     0x18
#define VL53L1_RANGE_HISTOGRAM_INIT                     0x20
#define VL53L1_RANGE_VHV_INIT                           0x40


#define VL53L1_RESULT_RANGE_STATUS                      0x1F


#define VL53L1_SYSTEM__SEED_CONFIG__MANUAL              0x00
#define VL53L1_SYSTEM__SEED_CONFIG__STANDARD            0x01
#define VL53L1_SYSTEM__SEED_CONFIG__EVEN_UPDATE_ONLY    0x02


#define VL53L1_INTERRUPT_CONFIG_LEVEL_LOW               0x00
#define VL53L1_INTERRUPT_CONFIG_LEVEL_HIGH              0x01
#define VL53L1_INTERRUPT_CONFIG_OUT_OF_WINDOW           0x02
#define VL53L1_INTERRUPT_CONFIG_IN_WINDOW               0x03
#define VL53L1_INTERRUPT_CONFIG_NEW_SAMPLE_READY        0x20


#define VL53L1_CLEAR_RANGE_INT                          0x01
#define VL53L1_CLEAR_ERROR_INT                          0x02


#define VL53L1_SEQUENCE_VHV_EN                          0x01
#define VL53L1_SEQUENCE_PHASECAL_EN                     0x02
#define VL53L1_SEQUENCE_REFERENCE_PHASE_EN              0x04
#define VL53L1_SEQUENCE_DSS1_EN                         0x08
#define VL53L1_SEQUENCE_DSS2_EN                         0x10
#define VL53L1_SEQUENCE_MM1_EN                          0x20
#define VL53L1_SEQUENCE_MM2_EN                          0x40
#define VL53L1_SEQUENCE_RANGE_EN                        0x80


#define VL53L1_DSS_CONTROL__ROI_SUBTRACT                0x20
#define VL53L1_DSS_CONTROL__ROI_INTERSECT               0x10

#define VL53L1_DSS_CONTROL__MODE_DISABLED               0x00
#define VL53L1_DSS_CONTROL__MODE_TARGET_RATE            0x01
#define VL53L1_DSS_CONTROL__MODE_EFFSPADS               0x02
#define VL53L1_DSS_CONTROL__MODE_BLOCKSELECT            0x03



#define VL53L1_RANGING_CORE__SPAD_READOUT__STANDARD              0x45
#define VL53L1_RANGING_CORE__SPAD_READOUT__RETURN_ARRAY_ONLY     0x05
#define VL53L1_RANGING_CORE__SPAD_READOUT__REFERENCE_ARRAY_ONLY  0x55
#define VL53L1_RANGING_CORE__SPAD_READOUT__RETURN_SPLIT_ARRAY    0x25
#define VL53L1_RANGING_CORE__SPAD_READOUT__CALIB_PULSES          0xF5


#define VL53L1_LASER_SAFETY__KEY_VALUE                  0x6C



#define VL53L1_RANGE_STATUS__RANGE_STATUS_MASK          0x1F
#define VL53L1_RANGE_STATUS__MAX_THRESHOLD_HIT_MASK     0x20
#define VL53L1_RANGE_STATUS__MIN_THRESHOLD_HIT_MASK     0x40
#define VL53L1_RANGE_STATUS__GPH_ID_RANGE_STATUS_MASK   0x80



#define VL53L1_INTERRUPT_STATUS__INT_STATUS_MASK            0x07
#define VL53L1_INTERRUPT_STATUS__INT_ERROR_STATUS_MASK      0x18
#define VL53L1_INTERRUPT_STATUS__GPH_ID_INT_STATUS_MASK     0x20



/* Register setting for ULP mode */
static const uint8_t VL53L1_ULP_DEFAULT_CONFIGURATION[] = {
	0x00, 0x00, 0x00, 0x11, 0x02, 0x00, 0x02, 0x08,
	0x00, 0x08, 0x10, 0x01, 0x01, 0x00, 0x00, 0x00,
	0x00, 0xff, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x20, 0x0b, 0x00, 0x00, 0x02, 0x14, 0x21,
	0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xc8,
	0x00, 0x00, 0x38, 0xff, 0x01, 0x00, 0x08, 0x00,
	0x00, 0x00, 0x01, 0x07, 0x00, 0x02, 0x05, 0x00,
	0xb4, 0x00, 0xbb, 0x08, 0x38, 0x00, 0x00, 0x00,
	0x00, 0x0f, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x07, 0x05, 0x06, 0x06, 0x00,
	0x00, 0x02, 0xc7, 0xff, 0x9B, 0x00, 0x00, 0x00,
	0x01, 0x00, 0x00
};

#endif /* _VL53L1_REGISTER_MAP_H_ */
