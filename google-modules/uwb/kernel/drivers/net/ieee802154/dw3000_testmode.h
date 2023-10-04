#ifndef DW3000_TESTMODE_H
#define DW3000_TESTMODE_H

#include <net/mcps802154.h>

#ifdef CONFIG_MCPS802154_TESTMODE

int dw3000_tm_cmd(struct mcps802154_llhw *llhw, void *data, int len);

#else

static inline int dw3000_tm_cmd(struct mcps802154_llhw *llhw, void *data,
				int len)
{
	return 0;
}

#endif /* CONFIG_MCPS802154_TESTMODE */

#endif /* DW3000_TESTMODE_H */
