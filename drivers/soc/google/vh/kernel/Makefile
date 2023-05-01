## SPDX-License-Identifier: GPL-2.0-only

# vh/kernel/sched
ifneq ($(CONFIG_VH_SCHED),)
obj-$(CONFIG_VH_SCHED)	+= sched/
else
obj-y			+= no_vh_sched.o
endif

# systrace
obj-$(CONFIG_VH_SYSTRACE) += systrace.o

# vh/kernel/thermal
obj-$(CONFIG_VH_THERMAL)  += thermal/

# vh/kernel/fs
obj-$(CONFIG_VH_FS)	+= fs/

# vh/kernel/mm
obj-$(CONFIG_VH_MM)     += mm/

# vh/kernel/cgroup
obj-$(CONFIG_VH_CGROUP)	+= cgroup/

# vh/kernel/i2c
obj-$(CONFIG_VH_I2C)     += i2c/

# vh/kernel/pixel_em
ifneq ($(CONFIG_PIXEL_EM),)
obj-$(CONFIG_PIXEL_EM)   += pixel_em/
else
obj-y			 += no_pixel_em.o
endif

# pixel metrics
obj-$(CONFIG_PIXEL_METRICS)	+= metrics/
