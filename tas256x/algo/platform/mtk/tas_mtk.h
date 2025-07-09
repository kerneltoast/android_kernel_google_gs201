/*
 * TAS256X Algorithm Support
 * Qualcomm Platform Support File
 *
 * Author: Vijeth P O
 * Date: 21-05-20
 */

#ifndef __TAS_MTK__
#define __TAS_MTK__

#include "../../../../mediatek/common/mtk-sp-spk-amp.h"

int afe_smartamp_algo_ctrl(uint8_t *data_buff, uint32_t param_id,
	uint8_t get_set, uint8_t length);

#endif /*__TAS_MTK__*/