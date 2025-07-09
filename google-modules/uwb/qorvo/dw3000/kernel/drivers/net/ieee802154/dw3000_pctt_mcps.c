/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2021 Qorvo US, Inc.
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
#include "dw3000_pctt_mcps.h"
#include "dw3000.h"
#include "dw3000_core.h"

#include <net/vendor_cmd.h>

int dw3000_pctt_vendor_cmd(struct dw3000 *dw, u32 vendor_id, u32 subcmd,
			   void *data, size_t data_len)
{
	struct llhw_vendor_cmd_pctt_setup_hw *info = data;
	struct dw3000_config *config = &dw->config;
	int rc;

	if (info) {
		if (sizeof(*info) != data_len)
			return -EINVAL;

		if (dw->pctt.enabled)
			return -EBUSY;

		config->chan = info->chan;
		config->txPreambLength = !info->preamble_duration ?
						 DW3000_PLEN_32 :
						 DW3000_PLEN_64;
		config->txCode = config->rxCode = info->preamble_code_index;
		config->sfdType = !info->sfd_id ? DW3000_SFD_TYPE_STD :
						  DW3000_SFD_TYPE_4Z;
		config->dataRate = !info->psdu_data_rate ? DW3000_BR_6M8 :
							   DW3000_BR_850K;
		config->stsMode = info->rframe_config;

		rc = dw3000_set_promiscuous(dw, true);
		if (rc)
			return rc;
		rc = dw3000_configure_sys_cfg(dw, config);
		if (rc)
			return rc;
		rc = dw3000_configure_chan(dw);
		if (rc)
			return rc;
	}
	dw->pctt.enabled = !!info;
	return dw3000_enable_auto_fcs(dw, !dw->pctt.enabled);
}
