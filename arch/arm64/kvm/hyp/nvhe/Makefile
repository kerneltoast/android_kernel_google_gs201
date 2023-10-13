# SPDX-License-Identifier: GPL-2.0

lib-objs := clear_page.o copy_page.o memcpy.o memset.o
lib-objs := $(addprefix ../../../lib/, $(lib-objs))

hyp-obj-y := timer-sr.o sysreg-sr.o debug-sr.o switch.o tlb.o hyp-init.o host.o \
	 hyp-main.o hyp-smp.o psci-relay.o early_alloc.o page_alloc.o \
	 cache.o setup.o mm.o mem_protect.o sys_regs.o pkvm.o stacktrace.o ffa.o iommu.o \
	 serial.o
hyp-obj-y += ../vgic-v3-sr.o ../aarch32.o ../vgic-v2-cpuif-proxy.o ../entry.o \
	 ../fpsimd.o ../hyp-entry.o ../exception.o ../pgtable.o
hyp-obj-$(CONFIG_TRACING) += clock.o events.o trace.o
hyp-obj-$(CONFIG_DEBUG_LIST) += list_debug.o
hyp-obj-$(CONFIG_MODULES) += modules.o
hyp-obj-y += $(lib-objs)

ifdef CONFIG_PKVM_S2MPU
SRC_ROOT := ../../../../..
GS_CMN := google-modules/soc/gs/drivers/soc/google/pkvm-s2mpu/common
GS_HYP := $(SRC_ROOT)/$(GS_CMN)/hyp
hyp-obj-y += $(GS_HYP)/s2mpu.o $(GS_HYP)/io-mpt-s2mpu.o

GS_HYP_CFLAGS := -I$(srctree)/$(GS_CMN) -DS2MPU_V1
CFLAGS_$(GS_HYP)/s2mpu.nvhe.o := $(GS_HYP_CFLAGS)
CFLAGS_$(GS_HYP)/io-mpt-s2mpu.nvhe.o := $(GS_HYP_CFLAGS)
endif

$(obj)/hyp.lds: $(src)/hyp.lds.S FORCE
	$(call if_changed_dep,cpp_lds_S)

include $(srctree)/arch/arm64/kvm/hyp/nvhe/Makefile.nvhe
obj-y := kvm_nvhe.o
