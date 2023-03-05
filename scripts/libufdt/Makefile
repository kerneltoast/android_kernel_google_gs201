# SPDX-License-Identifier: GPL-2.0

hostprogs-always-y := mkdtimg

# Directly compile in libfdt from dtc-aosp instead of linking to libfdt
libfdt-objs := fdt.o fdt_ro.o fdt_wip.o fdt_sw.o fdt_rw.o fdt_strerror.o \
	       fdt_empty_tree.o fdt_addresses.o fdt_overlay.o fdt_check.o acpi.o
libfdt-objs := $(addprefix ../dtc-aosp/dtc/libfdt/,$(libfdt-objs))

# Sources for mkdtimg
mkdtimg-objs := mkdtimg.o mkdtimg_cfg_create.o mkdtimg_core.o mkdtimg_create.o \
		mkdtimg_dump.o dt_table.o
mkdtimg-objs := $(addprefix utils/src/,$(mkdtimg-objs))
mkdtimg-objs += sysdeps/libufdt_sysdeps_posix.o
mkdtimg-objs += $(libfdt-objs)

HOST_EXTRACFLAGS := -I$(srctree)/scripts/dtc-aosp/dtc/libfdt \
		    -I$(srctree)/$(src)/sysdeps/include
HOST_EXTRACFLAGS += -std=gnu99 -Wno-stringop-truncation
