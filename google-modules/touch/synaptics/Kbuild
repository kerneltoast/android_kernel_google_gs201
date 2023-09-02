TCM_CORE=tcm/

obj-$(CONFIG_TOUCHSCREEN_SYNA_TCM2) = syna_touch.o
syna_touch-objs += \
			syna_tcm2.o \
			$(TCM_CORE)synaptics_touchcom_core_v1.o \
			$(TCM_CORE)synaptics_touchcom_core_v2.o \
			$(TCM_CORE)synaptics_touchcom_func_base.o \
			$(TCM_CORE)synaptics_touchcom_func_touch.o \
			$(TCM_CORE)synaptics_touchcom_func_reflash.o \
			$(TCM_CORE)synaptics_touchcom_func_romboot.o \
			syna_tcm2_platform_spi.o \
			syna_tcm2_sysfs.o \
			syna_tcm2_testing.o
