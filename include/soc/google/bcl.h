/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __BCL_H
#define __BCL_H

#if IS_ENABLED(CONFIG_GOOGLE_BCL)
extern int gs101_set_mpmm(unsigned int value);
extern int gs101_set_ppm(unsigned int value);
#else
static inline int gs101_set_ppm(unsigned int value)
{
	return 0;
}
static inline int gs101_set_mpmm(unsigned int value)
{
	return 0;
}
#endif /* IS_ENABLED(CONFIG_GOOGLE_BCL) */

#endif /* __BCL_H */
