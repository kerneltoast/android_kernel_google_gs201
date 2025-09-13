DTS File Changes:
=================
&i2c_8 {
	status = "ok";
	tas25xx@48 {
		#sound-dai-cells = <0>;
		compatible = "ti,tas25xx";
		reg = <0x48>; /* marvel */
		ti,max-channels = <2>;
		ti,channel-0 = <0x48>;
		ti,channel-1 = <0x49>;
		ti,reset-gpio = <&tlmm 117 0>;
		ti,irq-gpio = <&tlmm 118 0>;
		...
		status = "ok";
	};
};

Kernel Configurations:
=====================
* CONFIG_FW_LOADER_USER_HELPER_FALLBACK=y
* CONFIG_SND_SOC_TAS25XX=m
* CONFIG_TAS25XX_MISC=y
* CONFIG_TAS25XX_IRQ_BD=y
