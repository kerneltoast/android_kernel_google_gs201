# LWIS Device Tree Programming Guide

[TOC]

## Overview

LWIS device tree provides crucial device-specific information to ensure proper
execution of the LWIS kernel. All device tree properties used are listed in this
document, with supplemental references to the kernel documentation regarding
individual binding types.

This guide assumes familiarity with Linux's Device Tree configuration framework.
See the
[upstream documentation for Device Tree](https://elinux.org/Device_Tree_Reference)
for more information.

## Properties

A list of properties that are being used in LWIS device tree. Most properties
are optional, unless otherwise specified.

Only properties listed under General are used for top devices, all other
properties are ignored.

### General {#general}

`compatible`
:   **Required for all devices.**
:   Expected input: *string*
:   `compatible` property is used to specify the type of LWIS device used. It
    must be one of the following three strings:

    -   `google,lwis-top-device`
        -   Top LWIS device, should only have one per device. This is the device
            to perform generic operations (e.g. buffer allocation) and as an
            overseer of all other devices.
    -   `google,lwis-i2c-device`
        -   LWIS i2c device, register access via I2C bus. Common devices like
            sensors and actuators would likely be an i2c device.
    -   `google,lwis-ioreg-device`
        -   LWIS ioreg device, register access via mapped memory blocks. Common
            devices are the SOC’s ISP hardware blocks.

`node-name`
:   **Required for all devices.**

:   Expected input: *string*

:   This specifies the name of the LWIS device. For example, if `node-name` =
    “foo”, then a character device will be created in the system at
    `/dev/lwis-foo`.

`transaction-process-limit`
:   Expected input: *32-bit unsigned integer value.*

:   Set this property to a value greater than 0 to limit the number of entries
    to process in a single transaction for a given LWIS device. User can set
    this limit if the device has a large number of entries in single
    transactions and it is not desirable for the device to occupy the bus for a
    long period of time. If this property is not set, all entries in a
    transaction will be processed at the same time.

### Register Access {#reg-access}

`reg`
:   **Required for IOREG devices.**
:   Expected input: *a list of tuples: \<64-bit addresses, 32-bit address space
    sizes\>*
:   This defines the register spaces the LWIS IOREG device can access.

`reg-names`
:   **Required for IOREG devices.**
:   Expected input: *a list of string, list size must be the same as reg.*
:   This provides names to each of the register spaces defined in reg for the
    ability to search for a register space by name and clarity in debug
    messages.

`reg-addr-bitwidth`
`reg-value-bitwidth`
:   Expected input: *a single 32-bit value, in units of bits. Valid values: {8,
    16, 32, 64}*
:   By default, both address and value bitwidths are set to 32 bits for all
    devices. These properties are to specify the bitwidths in case they are
    different from default.

### I2C {#i2c}

`i2c-bus`
:   **Required for I2C devices.**
:   Expected input: *a
    [phandle](https://elinux.org/Device_Tree_Mysteries#Phandle) to i2c entry.*
:   This specifies the I2C bus the LWIS device belongs to.

`i2c-addr`
:   **Required for I2C devices.**
:   Expected input: *a 32-bit value.*
:   Device address of the I2C device.

`i2c-lock-group-id`
:   Expected input: *a 32-bit value.* Current acceptable values are from 0 to 6.
:   Lock group index to use in the power up / down sequence. Devices with
    distinct lock groups are powered up in parallel. If not set, uses a single
    global lock group. Example: there are three camera modules A, B, C, each of
    which has a sensor, an EEPROM, a focus actuator and an OIS actuator, for a
    total of 12 devices. Assigning lock group 1 to all devices in module A,
    group 2 to all devices in module B and group 3 to all devices in module C
    allows each module to power up in parallel, but still serializes the
    power-up of the sensor, EEPROM, focus and OIS devices within each module.

`i2c-device-priority`
:   Expected input: *32-bit unsigned integer value.*
:   Current acceptable values are from 0 to 2.
:   Use this property to set the device priority for the I2C device.
    Set this to 0 for high device priority, 1 for medium device priority and
    2 for low device priority. There is one I2C worker thread per bus.
    When this I2C worker thread is scheduled to process the transactions on a
    given bus, the high priority queue is checked first then the worker moves
    to processing the medium priority queue and finally the low priority queue.

### Clock {#clock}

Reference:
[Generic information regarding clocks](https://www.kernel.org/doc/Documentation/devicetree/bindings/clock/clock-bindings.txt)

`clocks`
:   Expected input: *a list of tuples: \<clock phandle, 32-bit values for clock
    defines\>*
:   Clocks that are associated with this device. These clocks will be enabled
    when device is enabled.

`clock-names`
:   Expected input: *a list of strings, list size must be the same as the list
    size of clocks.*
:   This provides names to each of the clocks defined in clocks for the ability
    to search for a clock by name and clarity in debug messages.

`clock-rates`
:   Expected input: *a list of 32-bit values.*
:   An option to set the rate of a clock to a particular value. The size of this
    list can be smaller than that of clocks. For any clock that does not have a
    corresponding value, or the value being set to 0, the default rate specified
    by the clock driver will be used.

### GPIO {#gpio}

Reference:
[Generic information regarding GPIOs](https://www.kernel.org/doc/Documentation/devicetree/bindings/gpio/gpio.txt)

### Regulator {#regulator}

`*-supply`
:   Expected input: *a phandle to a regulator.*<br>
    The * should be replaced to the supply name.
:   Example:

    ```
    vdig-supply = <&regulator_ldo1>;
    vdda-supply = <&regulator_ldo4>;
    vddio-supply = <&regulator_ldo8>;
    ```

:   We also need to override the regulator to the correct voltage settings.
:   Example:

    ```
    &regulator_ldo1 {
    	regulator-min-microvolt = <2850000>;
    	regulator-max-microvolt = <2850000>;
    };

    &regulator_ldo4 {
    	regulator-min-microvolt = <2900000>;
    	regulator-max-microvolt = <2900000>;
    };

    &regulator_ldo8 {
    	regulator-min-microvolt = <1100000>;
    	regulator-max-microvolt = <1100000>;
    };
    ```

### Power Domain {#power-domain}

`power-domains`
:   Expected input: *a list of power domain phandles.*
:   Power domains that the device belongs to. When the device is enabled, the
    power management driver will turn on the domains specified.

### Pin Controller {#pin-controller}

Reference:
[Generic information regarding pin controllers](https://www.kernel.org/doc/Documentation/devicetree/bindings/pinctrl/pinctrl-bindings.txt)

`pinctrl-names`
:   Expected input: *a list of strings, size of this list should be the same as
    the number of configurations specified.*
:   These strings assign names to the pin configuration so they can be
    discovered by name.

`pinctrl-<n>`
:   Expected input: *a phandle to a pin configurations entry.*
:   This should point to a pin configuration under a pin controller. All N
    pinctrl configurations should be under the same pin controller.

`shared-pinctrl`
:   Expected input: *A flag to indicate if pinctrls are shared with other
    devices.*
:   shared-pinctrl can be any of positive integer value, the same number defined
    means they are sharing pinctrl resources.
    -   For example:
        -   Device A: shared-pinctrl = <1>
        -   Device B: shared-pinctrl = <1>
        -   Device C: shared-pinctrl = <2>
        -   Device D: shared-pinctrl = <2>
        -   Then A shares with B, C shares with D.

### Power Up / Down Sequence {#power-sequence}

`power-up-seqs`
:   Expected input: *a list of strings.*
:   List of pin names to activate when powering up the device. During device
    power-up, the pins are activated in the order specified in this list, with
    a time delay after activating each pin specified by `power-up-seq-delays-us`
    . The list `power-up-seq-types` gives the type of each pin.

`power-up-seq-types`
:   Expected input: *a list of strings.* Valid values: {"regulator", "gpio",
    "pinctrl"}. List size must equal to the list size of power-up-seqs.
:   Pin type for each of the power-up pins specified in `power-up-seqs`.

`power-up-seq-delays-us`
:   Expected input: *a list of 32-bit values.* List size must equal to the list
    size of power-up-seqs.
:   Delay time in microseconds after activating each pin in `power-up-seqs`. Use
    0 to activate a pin simultaneously with the next one.

`power-down-seqs`
:   Expected input: *a list of strings.*
:   List of pin names to activate when powering down the device. During device
    power-down, the pins are activated in the order specified in this list, with
    a time delay after activating each pin specified by
    `power-down-seq-delays-us`. The list `power-down-seq-types` gives the type
    of each pin.

`power-down-seq-types`
:   Expected input: *a list of strings.* Valid values: {"regulator", "gpio",
    "pinctrl"}. List size must equal to the list size of power-down-seqs.
:   Pin type for each of the power-down pins specified in `power-down-seqs`.

`power-down-seq-delays-us`
:   Expected input: *a list of 32-bit values.* List size must equal to the list
    size of power-down-seqs.
:   Delay time in microseconds after activating each pin in `power-down-seqs`.
    Use 0 to activate a pin simultaneously with the next one.

### IOMMU {#iommu}

Reference:
[Generic information about IOMMUs](https://www.kernel.org/doc/Documentation/devicetree/bindings/iommu/iommu.txt)

`iommus`
:   Expected input: *a list of IOMMU phandles.*
:   Any device that provides buffer management functionality should have
    IOMMU(s) defined. These IOMMUs are responsible for memory mapping hardware
    addresses to virtual addresses.

### PHY {#phy}

Reference:
[Generic information regarding PHYs](https://www.kernel.org/doc/Documentation/devicetree/bindings/phy/phy-bindings.txt)

`phys`
:   Expected input: *a list of tuples: \<phandle to phy entry, [additional
    specifier]\>*
:   Additional specifiers depends on the definition of `#phy-cells` in higher
    level device trees.
:   List of PHYs that are associated with the device.

`phy-names`
:   Expected input: *a list of strings, list size must be the same as the list
    size of phys.*
:   This provides names to each of the PHY defined in phys for the ability to
    search for a PHY device by name and clarity in debug messages.

### Interrupts & Events {#events}

Reference:
[Generic information regarding interrupts](https://www.kernel.org/doc/Documentation/devicetree/bindings/interrupt-controller/interrupts.txt)

Note: Interrupts are not supported to I2C devices yet.

`interrupts`
:   Expected input: *a list of 32-bit values with interrupt enums. Actual size
    of each entry in the list depends on the definition of `#interrupt-cells`.*
:   A list of interrupts that will be observed by this LWIS device. Each
    interrupt is expected to have their own handler registered.

`interrupt-names`
:   Expected input: *a list of strings, list size must be the same as the list
    size of interrupts.*
:   This provides names to each of the interrupts defined in interrupts for the
    ability to search for an interrupt by name and clarity in debug messages.

`interrupt-event-infos`
:   Expected input: *a list of phandles with information about the interrupt
    vectors.*
:   This entry is crucial for interrupt handling in LWIS. It provides
    information to enable/disable/clear interrupts, as well as mapping interrupt
    vector bits into proper LWIS event IDs. (See
    [Interrupt Event Info section](#event-info))

#### Interrupt Event Info

`irq-reg-space`
:   **Required property for `interrupt-event-infos`.**
:   Expected input: *string*
:   This should correspond to one of the `reg-names` specified for the device,
    such that the kernel can read/write the registers specified below properly.

`irq-src-reg`
:   **Required property for `interrupt-event-infos`.**
:   Expected input: *a 64-bit value.*
:   Specifies the register offset for the interrupt source register (a.k.a.
    status register).

`irq-reset-reg`
:   **Required property for `interrupt-event-infos`.**
:   Expected input: *Two 32-bit values to be used as a 64-bit address.*
:   Specifies the register offset for the interrupt reset register (a.k.a. clear
    register).

`irq-mask-reg`
:   **Required property for `interrupt-event-infos`.**
:   Expected input: *Two 32-bit values to be used as a 64-bit address.*
:   Specifies the register offset for the interrupt mask register (a.k.a. enable
    register).

`irq-overflow-reg`
:   Expected input: *Two 32-bit values to be used as a 64-bit address.*
:   Specifies the register offset for the interrupt overflow register (might not
    be present in all hardware blocks)

`irq-mask-reg-toggle`
:   Expected input: *boolean.*
:   For some hardware blocks, reverse logic is implemented for the mask
    register, i.e. setting bit fields to 1 would disable (instead of enable) an
    interrupt. In that case, define this property and set it to true. Otherwise,
    this is false by default.

`irq-reg-bitwidth`
:   Expected input: *a 32-bit value. Valid values: {8, 16, 32, 64}*
:   For some hardware blocks, the access size of the interrupt registers is
    different from the native bus bitwidth. In that case, define this property
    and set it to the correct bitwidth. Otherwise, this is equal to native bus
    bitwidth by default.

`irq-events`
:   **Required property for `interrupt-event-infos`.**
:   Expected input: *a list of 32-bit values (i.e. number of events times 2
    32-bit values for each event ID), list size must equal to the list size of
    int-reg-bits.*
:   A list of LWIS event IDs that will be generated by this interrupt vector.

`int-reg-bits`
:   **Required property for `interrupt-event-infos`.**
:   Expected input: *a list of 32-bit values (i.e. number of events times 2
    32-bit values for each event ID), list size must equal to the list size of
    irq-events.*
:   Bit definitions of the interrupt vector. The ordering of this list must map
    to the correct event IDs above.

`critical-irq-events`
:   Expected input: *a list of 32-bit values (i.e. number of events times 2
    32-bit values for each event ID). Event IDs in this list must be equal to or
    a subset of irq-events.*
:   Log critical ISP events in the kernel for debugging. Define the critical
    events in the device tree.

`irq-type`
:   **Required property for `interrupt-event-infos` of aggregate and leaf
    interrupts.**
:   Expected input: *string. Valid values: {"regular", "aggregate", "leaf"}*
:   Specifies the type of the interrupt.

`irq-leaf-nodes`
:   **Required property for `interrupt-event-infos` of aggregate interrupts.**
:   Expected input: *a list of phandles with information about the leaf nodes.*
:   Specifies the aggregate interrupt bit-leaf mapping. Each phandles should
    point at a leaf node containing information about the group of leaf
    interrupts that a bit is associated with. The ordering of this list must map
    to the correct register bit above.

#### Interrupt Group

`leaf-interrupt-names`
:   **Required property**
:   Expected input: *a list of string.*
:   A list of interrupt names. The interrupt groups specifies the list of leaf
    interrupts in a leaf node.

#### Aggregate and Leaf Interrupt Example

Aggregate interrupts are higher level hierarchy interrupts that can be used to
indicate the signals of underlying interrupts. The aggregation mapping is also
defined in the event info file.

Leaf-level Interrupt
:   An underlying interrupt that's covered by an aggregate interrupt. The
    `irq-type` field is required for a leaf interrupt, and the value should be
    set to "leaf".
:   Example:

    ```
    isp_dev_leaf_interrupt_example0: isp_dev-event-info@29 {
        irq-reg-space = "isp-dev";
        ...
        irq-type = "leaf";

        irq-events =
            <LWIS_PLATFORM_EVENT_ID_ISP_LEAF_EVETNT0>,
            <LWIS_PLATFORM_EVENT_ID_ISP_LEAF_EVETNT1>,
            ...;
        int-reg-bits =
            <ISP_LEAF_BIT0>,
            <ISP_LEAF_BIT1>,
            ...;
    };
    ```

Interrupt Group
:   Specifies a list of leaf interrupt names that a leaf node contains. These
    leaf interrupt signals are gated to one bit of an aggregate interrupt.

:   Example:

    ```
    isp_interrupt_group_eg0: isp_dev-interrupt-group@0 {
        leaf-interrupt-names =
            "isp_dev_leaf_interrupt_example0",
            "isp_dev_leaf_interrupt_example1",
            ...;
    };
    ```

Aggregate Interrupt
:   A higher-level interrupt. The bits list and the interrupt groups list
    represents the aggregation mapping between aggregate interrupts and leaf
    interrupts. The `irq-type` field is required for a leaf interrupt, and the
    value should be set to "aggregate".

:   Example:

    ```
    isp_aggregate_interrupt_example: isp_dev-event-info@24 {
        irq-reg-space = "isp-dev";
        ...
        irq-type = "aggregate";

        irq-leaf-nodes =
            <&isp_interrupt_group_eg0>,
            <&isp_interrupt_group_eg1>,
            ...;
        irq-events =
            <LWIS_PLATFORM_EVENT_ID_ISP_AGGREGATE_EVETNT0>,
            <LWIS_PLATFORM_EVENT_ID_ISP_AGGREGATE_EVETNT1>,
            ...;
        int-reg-bits =
            <ISP_AGGREGATE_BIT0>,
            <ISP_AGGREGATE_BIT1>,
            ...;
    };
    ```

### Power management

`pm-hibernation`
:   Expected input: *a single 32-bit value.(i.e. 1 : enable to keep original mechanism, 0 : disable to disallow to get into the suspension mode).*
:   This property prevents the camera driver from being powered off while the
    system gets into the suspension mode.
:   Example:

    ```
    flash0: flash@0 {
        compatible = "google,lwis-i2c-device";

        /* Device node name */
        node-name = "flash-foo";

        /* Power Management hibernation (deep sleep) */
        /* 1 : enable, 0 : disable */
        pm-hibernation = <0>;
    };
    ```

### Device access mode

`lwis,read-only`
:   Some drivers (i.e. EEPROM) should not arbitrarily allow write access. This property sets device as read access only to avoid people with bad intentions breaking the drivers. In that case, define this property to deny write access to this particular device. Otherwise, read-write permissions will be granted.
:   Example:

```
    eeprom: eeprom@ {
        compatible = "google,lwis-i2c-device";

        /* Device node name */
        node-name = "eeprom-model-sensor";

        /* Access mode property*/
        lwis,read-only;
    };
```

### Thread priority {#thread-priority}

`transaction-thread-priority`
:   Expected input: *a single 32-bit value.* Valid values: from 1 to 139.
:   Assigns the given priority to the transaction processing kernel thread of
    this device. Higher values indicate lower priorities. Regular priorities
    range from 139 (lowest priority) to 100 (highest priority). Real-time
    priorities range from 99 (lowest priority) to 1 (highest priority).


## Coding Style

```
    lwis_gdc0: lwis_gdc@1BA40000 {
        compatible = "google,lwis-ioreg-device";
        /* Device node name */
        node-name = "gdc0";
        /* Register space */
        reg =
            <0x0 0x1BA40000 0x10000>,   /* GDC0 */
            <0x0 0x1BB34000 0xA00>;     /* GDC0 SSMT */
        reg-names =
            "gdc",
            "gdc_ssmt";
    };
```

-   Use tabs instead of spaces
-   Use tab stop of 8
-   Kernel style comments, i.e. `/* ... */`
-   Properties that do not fit within one line should start first entry from the
    line after the property name, and an extra tab from the beginning of the
    property name.

## Example

[ISP Device Tree for Pixel 6 (Raviole)](https://android.googlesource.com/kernel/google-modules/raviole-device/+/refs/heads/android-gs-raviole-mainline/arch/arm64/boot/dts/google/gs101-isp.dtsi)
