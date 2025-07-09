
ccflags-y += -I$(srctree)/drivers/thermal

ifneq ($(CONFIG_MFD_S2MPG13),)
obj-$(CONFIG_GSOC_PMIC_THERMAL)	+= s2mpg13_spmic_thermal.o
endif

ifneq ($(CONFIG_MFD_S2MPG11),)
obj-$(CONFIG_GSOC_PMIC_THERMAL)	+= gs101_spmic_thermal.o
endif
obj-$(CONFIG_GSOC_PMIC_THERMAL_CAMBRIA)	+= s2mpg15_spmic_thermal.o
obj-$(CONFIG_THERMAL_SENSOR_STS4X)	+= sts4x_ambient_i2c.o
obj-$(CONFIG_THERMAL_UCLAMP_COOLING_DEVICE)	+= cdev_uclamp.o
