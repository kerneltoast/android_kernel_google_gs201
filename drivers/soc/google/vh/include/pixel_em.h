/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Pixel Energy Model (EM).
 *
 * Copyright (C) 2022 Google, Inc.
 */

#ifndef __PIXEL_EM_H__
#define __PIXEL_EM_H__

#if IS_ENABLED(CONFIG_PIXEL_EM)

struct pixel_em_opp {
  unsigned int freq;
  unsigned int capacity;
  unsigned int power;
  unsigned long cost;
};

struct pixel_em_cluster {
  cpumask_t cpus;
  int num_opps;
  struct pixel_em_opp *opps;
};

struct pixel_em_profile {
  struct list_head list;
  struct profile_sysfs_helper *sysfs_helper;
  const char *name;
  int num_clusters;
  struct pixel_em_cluster *clusters;
  int num_cpus;
  struct pixel_em_cluster **cpu_to_cluster; // Maps CPU index to a cluster pointer
};

#endif /* CONFIG_PIXEL_EM */

#endif /* __PIXEL_EM_H__ */
