// SPDX-License-Identifier: GPL-2.0-only
/* topology.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2022 Google LLC
 */

#include <linux/sched.h>
#if IS_ENABLED(CONFIG_VH_SCHED) && IS_ENABLED(CONFIG_PIXEL_EM)
#include "../../include/pixel_em.h"
extern struct pixel_em_profile **vendor_sched_pixel_em_profile;
#endif

#if IS_ENABLED(CONFIG_VH_SCHED) && IS_ENABLED(CONFIG_PIXEL_EM)
void vh_arch_set_freq_scale_pixel_mod(void *data, const struct cpumask *cpus,
                                      unsigned long freq,
                                      unsigned long max, unsigned long *scale)
{
        int i;
        struct pixel_em_profile **profile_ptr_snapshot;
        profile_ptr_snapshot = READ_ONCE(vendor_sched_pixel_em_profile);
        if (profile_ptr_snapshot) {
                struct pixel_em_profile *profile = READ_ONCE(*profile_ptr_snapshot);
                if (profile) {
                        struct pixel_em_cluster *cluster;
                        struct pixel_em_opp *max_opp;
                        struct pixel_em_opp *opp;

                        cluster = profile->cpu_to_cluster[cpumask_first(cpus)];
                        max_opp = &cluster->opps[cluster->num_opps - 1];

                        for (i = 0; i < cluster->num_opps; i++) {
                                opp = &cluster->opps[i];
                                if (opp->freq >= freq)
                                        break;
                        }

                        *scale = (opp->capacity << SCHED_CAPACITY_SHIFT) /
                                  max_opp->capacity;
                }
        }
}
EXPORT_SYMBOL_GPL(vh_arch_set_freq_scale_pixel_mod);
#endif

void android_vh_use_amu_fie_pixel_mod(void* data, bool *use_amu_fie)
{
	*use_amu_fie = false;
}
