subdir-ccflags-y += -I$(srctree)/$(src)/../common/
subdir-ccflags-y += -I$(srctree)/arch/arm64/kvm/hyp/include/
subdir-ccflags-y += -DS2MPU_V9

obj-m += pkvm-s2mpu-v9.o

$(obj)/../common/hyp/kvm_nvhe.o: FORCE
		$(Q)$(MAKE) $(build)=$(obj)/../common/hyp $(obj)/../common/hyp/kvm_nvhe.o

clean-files := hyp/hyp.lds hyp/hyp-reloc.S

$(obj)/pkvm_s2mpu.o: $(src)/../common/pkvm_s2mpu.c FORCE
	$(call if_changed_rule,cc_o_c)

$(obj)/s2mpu.o: $(src)/../common/s2mpu.c FORCE
	$(call if_changed_rule,cc_o_c)

$(obj)/exports.o: $(src)/../common/exports.c FORCE
	$(call if_changed_rule,cc_o_c)

pkvm-s2mpu-v9-y := pkvm_s2mpu.o s2mpu.o pkvm-s2mpu-faults-v9.o exports.o ../common/hyp/kvm_nvhe.o