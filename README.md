# Ethernet PHY Driver

## Overview

- The library provides the Ethernet PHY-related interface APIs as required by Ethernet connection manager library to enable the completion of Ethernet-based applications on supported platforms.
The current version of ethernet-phy-driver library has Ethernet PHY-related interface APIs implemented for PHY chip DP83867IR and DP83825I.

## Features and functionality

- The library contains the necessary ethernet PHY interface APIs for the ethernet-connection-manager library, enabling ethernet-based applications to operate on supported ethernet PHY chips.
- Supports the Ethernet PHY driver for the PHY chip DP83867IR.
- Supports the Ethernet PHY driver for the PHY chip DP83825I.
- Supports the Ethernet PHY driver for the PHY chip LAN8710AI.

## Supported platforms

This library and its features are supported on the following Infineon platforms:

- [XMC7200 Evaluation Kit (KIT_XMC72_EVK)](https://www.infineon.com/KIT_XMC72_EVK)
- [XMC7100 Evaluation Kit (KIT_XMC71_EVK_LITE_V1)](https://www.infineon.com/KIT_XMC71_EVK_LITE_V1)
- PSOC&trade; Edge E84 Evaluation Kit

## Quick Start

1. To download ethernet PHY driver, create the following *ethernet-phy-driver.mtb* file in application deps folder.
   - *ethernet-phy-driver.mtb:* <br>
      `https://github.com/Infineon/ethernet-phy-driver#latest-v1.X#$$ASSET_REPO$$/ethernet-phy-driver/latest-v1.X`

2. To use ethernet-phy-driver library on FreeRTOS, lwIP, and Mbed TLS combination, the application should pull [ethernet-core-freertos-lwip-mbedtls](https://github.com/Infineon/ethernet-core-freertos-lwip-mbedtls) library version 2.X which will internally pull secure-sockets, ethernet-connection-manager, FreeRTOS, lwIP, Mbed TLS and other dependent modules.<br>
To pull ethernet-core-freertos-lwip-mbedtls create the following *.mtb* file in deps folder.
   - *ethernet-core-freertos-lwip-mbedtls.mtb:*
      `https://github.com/Infineon/ethernet-core-freertos-lwip-mbedtls#latest-v2.X#$$ASSET_REPO$$/ethernet-core-freertos-lwip-mbedtls/latest-v2.X`

3. To configure the pin connections on [XMC7200 Evaluation kit (KIT_XMC72_EVK)](https://www.infineon.com/KIT_XMC72_EVK) with PHY chip DP83867IR, open design.modus file and do the following configuration settings in the ModusToolbox&trade; Device Configurator.
    - Switch to the "Peripherals" tab.
    - Select Communication->"Ethernet 1".
    - In "Ethernet - Parameters" pane, go to "Connections" section and configure each pin as shown below.
    - Select pin "ref_clk" from the drop-down.
        - Click on "Go to signal" of selected pin,
        - Configure "Drive mode" to "Digital HIGH-Z" from the drop-down.
        - Configure "Initial Drive state" to "Low (0)" from the drop-down.
        - Configure "Drive Strength" to "1/2" from the drop-down.
    - Select pin "mdc" from the drop-down.
        - Click on "Go to signal" of selected pin,
        - Configure "Drive mode" to "Strong Drive, Input buffer off" from the drop-down.
        - Configure "Initial Drive state" to "Low (0)" from the drop-down.
        - Configure "Drive Strength" to "1/8" from the drop-down.
    - Select pin "mdio" from the drop-down.
        - Click on "Go to signal" of selected pin,
        - Configure "Drive mode" to "Strong Drive, Input buffer on" from the drop-down.
        - Configure "Initial Drive state" to "Low (0)" from the drop-down.
        - Configure "Drive Strength" to "1/8" from the drop-down.
    - Select pin "tx_clk" from the drop-down.
        - Click on "Go to signal" of selected pin,
        - Configure "Drive mode" to "Strong Drive, Input buffer off" from the drop-down.
        - Configure "Initial Drive state" to "Low (0)" from the drop-down.
        - Configure "Drive Strength" to "1/2" from the drop-down.
    - Select pin "tx_ctl" from the drop-down.
        - Click on "Go to signal" of selected pin,
        - Configure "Drive mode" to "Strong Drive, Input buffer off" from the drop-down.
        - Configure "Initial Drive state" to "Low (0)" from the drop-down.
        - Configure "Drive Strength" to "1/2" from the drop-down.
    - Select pin "txd[0]" from the drop-down.
        - Click on "Go to signal" of selected pin,
        - Configure "Drive mode" to "Strong Drive, Input buffer off" from the drop-down.
        - Configure "Initial Drive state" to "Low (0)" from the drop-down.
        - Configure "Drive Strength" to "1/2" from the drop-down.
    - Select pin "txd[1]" from the drop-down.
        - Click on "Go to signal" of selected pin,
        - Configure "Drive mode" to "Strong Drive, Input buffer off" from the drop-down.
        - Configure "Initial Drive state" to "Low (0)" from the drop-down.
        - Configure "Drive Strength" to "1/2" from the drop-down.
    - Select pin "txd[2]" from the drop-down.
        - Click on "Go to signal" of selected pin,
        - Configure "Drive mode" to "Strong Drive, Input buffer off" from the drop-down.
        - Configure "Initial Drive state" to "Low (0)" from the drop-down.
        - Configure "Drive Strength" to "1/2" from the drop-down.
    - Select pin "txd[3]" from the drop-down.
        - Click on "Go to signal" of selected pin,
        - Configure "Drive mode" to "Strong Drive, Input buffer off" from the drop-down.
        - Configure "Initial Drive state" to "Low (0)" from the drop-down.
        - Configure "Drive Strength" to "1/2" from the drop-down.
    - Select pin "rx_clk" from the drop-down.
        - Click on "Go to signal" of selected pin,
        - Configure "Drive mode" to "Digital HIGH-Z, Input buffer on" from the drop-down.
        - Configure "Initial Drive state" to "Low (0)" from the drop-down.
        - Configure "Drive Strength" to "1/2" from the drop-down.
    - Select pin "rx_ctl" from the drop-down.
        - Click on "Go to signal" of selected pin,
        - Configure "Drive mode" to "Digital HIGH-Z, Input buffer on" from the drop-down.
        - Configure "Initial Drive state" to "Low (0)" from the drop-down.
        - Configure "Drive Strength" to "1/2" from the drop-down.
    - Select pin "rxd[0]" from the drop-down.
        - Click on "Go to signal" of selected pin,
        - Configure "Drive mode" to "Digital HIGH-Z, Input buffer on" from the drop-down.
        - Configure "Initial Drive state" to "Low (0)" from the drop-down.
        - Configure "Drive Strength" to "1/2" from the drop-down.
    - Select pin "rxd[1]" from the drop-down.
        - Click on "Go to signal" of selected pin,
        - Configure "Drive mode" to "Digital HIGH-Z, Input buffer on" from the drop-down.
        - Configure "Initial Drive state" to "Low (0)" from the drop-down.
        - Configure "Drive Strength" to "1/2" from the drop-down.
    - Select pin "rxd[2]" from the drop-down.
        - Click on "Go to signal" of selected pin,
        - Configure "Drive mode" to "Digital HIGH-Z, Input buffer on" from the drop-down.
        - Configure "Initial Drive state" to "Low (0)" from the drop-down.
        - Configure "Drive Strength" to "1/2" from the drop-down.
    - Select pin "rxd[3]" from the drop-down.
        - Click on "Go to signal" of selected pin,
        - Configure "Drive mode" to "Digital HIGH-Z, Input buffer on" from the drop-down.
        - Configure "Initial Drive state" to "Low (0)" from the drop-down.
        - Configure "Drive Strength" to "1/2" from the drop-down.
    - Save the configuration to generate the necessary code.

4. To configure the pin connections on [XMC7100 Evaluation Kit (KIT_XMC71_EVK_LITE_V1)](https://www.infineon.com/KIT_XMC71_EVK_LITE_V1) with PHY chip DP83825I, open design.modus file and do the following configuration settings in the ModusToolbox&trade; Device Configurator.
    - Switch to the "Peripherals" tab.
    - Select Communication->"Ethernet 0".
    - In "Ethernet - Parameters" pane, go to "Connections" section and configure each pin as shown below.
    - Select pin "ref_clk" from the drop-down.
        - Click on "Go to signal" of selected pin,
        - Configure "Drive mode" to "Digital HIGH-Z" from the drop-down.
        - Configure "Initial Drive state" to "Low (0)" from the drop-down.
        - Configure "Drive Strength" to "1/2" from the drop-down.
    - Select pin "mdc" from the drop-down.
        - Click on "Go to signal" of selected pin,
        - Configure "Drive mode" to "Strong Drive, Input buffer off" from the drop-down.
        - Configure "Initial Drive state" to "Low (0)" from the drop-down.
        - Configure "Drive Strength" to "1/8" from the drop-down.
    - Select pin "mdio" from the drop-down.
        - Click on "Go to signal" of selected pin,
        - Configure "Drive mode" to "Strong Drive, Input buffer on" from the drop-down.
        - Configure "Initial Drive state" to "Low (0)" from the drop-down.
        - Configure "Drive Strength" to "1/8" from the drop-down.
    - Select pin "tx_ctl" from the drop-down.
        - Click on "Go to signal" of selected pin,
        - Configure "Drive mode" to "Strong Drive, Input buffer off" from the drop-down.
        - Configure "Initial Drive state" to "Low (0)" from the drop-down.
    - Select pin "txd[0]" from the drop-down.
        - Click on "Go to signal" of selected pin,
        - Configure "Drive mode" to "Strong Drive, Input buffer off" from the drop-down.
        - Configure "Initial Drive state" to "Low (0)" from the drop-down.
    - Select pin "txd[1]" from the drop-down.
        - Click on "Go to signal" of selected pin,
        - Configure "Drive mode" to "Strong Drive, Input buffer off" from the drop-down.
        - Configure "Initial Drive state" to "Low (0)" from the drop-down.
    - Select pin "rx_ctl" from the drop-down.
        - Click on "Go to signal" of selected pin,
        - Configure "Drive mode" to "Digital HIGH-Z, Input buffer on" from the drop-down.
        - Configure "Initial Drive state" to "Low (0)" from the drop-down.
    - Select pin "rx_er" from the drop-down.
        - Click on "Go to signal" of selected pin,
        - Configure "Drive mode" to "Digital HIGH-Z, Input buffer on" from the drop-down.
        - Configure "Initial Drive state" to "Low (0)" from the drop-down.
    - Select pin "rxd[0]" from the drop-down.
        - Click on "Go to signal" of selected pin,
        - Configure "Drive mode" to "Digital HIGH-Z, Input buffer on" from the drop-down.
        - Configure "Initial Drive state" to "Low (0)" from the drop-down.
    - Select pin "rxd[1]" from the drop-down.
        - Click on "Go to signal" of selected pin,
        - Configure "Drive mode" to "Digital HIGH-Z, Input buffer on" from the drop-down.
        - Configure "Initial Drive state" to "Low (0)" from the drop-down.
    - Select pin "rxd[1]" from the drop-down.
        - Click on "Go to signal" of selected pin,
        - Configure "Drive mode" to "Digital HIGH-Z, Input buffer on" from the drop-down.
        - Configure "Initial Drive state" to "Low (0)" from the drop-down.
    - KIT_XMC71_EVK_LITE_V1 device needs additional RST_N pin to configure. To do the same, Switch to the "Pins" tab.
    - Select Port22->"P22[3]".
    - In "p22[3] - Parameters" pane, configure each pin as shown below.
        - Configure "Drive mode" to "Strong Drive, Input buffer off" from the drop-down.
        - Configure "Initial Drive state" to "HIGH (1)" from the drop-down.
    - Save the configuration to generate the necessary code.

5. To configure the pin connections on [PSOC™ Edge E84 Evaluation Kit (KIT_PSE84_EVAL)](https://www.infineon.com/KIT_PSE84_EVAL) with PHY chip LAN8710AI, open design.modus file and do the following configuration settings in the ModusToolbox&trade; Device Configurator.
    - Switch to the "Pins" tab.
    - Deselect the "Smart I/O 11".
    - Switch to the "Peripherals" tab.
    - Deselect the "Serial Communication Block (SCB) 4".
    - Select Communication->"Ethernet 0".
    - In "Ethernet - Parameters" pane, go to "Network Interface Settings" section and configure PHY parameters.
    - Configure "PHY Interconnect" to "MII" from the drop-down.
    - Configure "PHY Device" to "LAN8710AI" from the drop-down.
    - In "Ethernet - Parameters" pane, go to "Connections" section and configure each pin as shown below.
    - Select pin "mdc" from the checkbox in the pop-up.
        - Click on "Go to signal" of selected pin,
        - Configure "Drive mode" to "Strong Drive, Input buffer off" from the drop-down.
        - Configure "Initial Drive state" to "Low (0)" from the drop-down.
        - Configure "Drive Strength" to "1/8" from the drop-down.
    - Select pin "mdio" from the drop-down.
        - Click on "Go to signal" of selected pin,
        - Configure "Drive mode" to "Strong Drive, Input buffer on" from the drop-down.
        - Configure "Initial Drive state" to "Low (0)" from the drop-down.
        - Configure "Drive Strength" to "1/8" from the drop-down.
    - Select pin "tx_clk" from the drop-down.
        - Click on "Go to signal" of selected pin,
        - Configure "Drive mode" to "Digital HIGH-Z, Input buffer on" from the drop-down.
        - Configure "Initial Drive state" to "Low (0)" from the drop-down.
    - Select pin "tx_ctl" from the checkbox in the pop-up.
        - Click on "Go to signal" of selected pin,
        - Configure "Drive mode" to "Strong Drive, Input buffer off" from the drop-down.
        - Configure "Initial Drive state" to "Low (0)" from the drop-down.
    - Select pin "tx_er" from the checkbox in the pop-up.
        - Click on "Go to signal" of selected pin,
        - Configure "Drive mode" to "Strong Drive, Input buffer off" from the drop-down.
        - Configure "Initial Drive state" to "Low (0)" from the drop-down.
    - Select pin "txd[0]" from the checkbox in the pop-up.
        - Click on "Go to signal" of selected pin,
        - Configure "Drive mode" to "Strong Drive, Input buffer off" from the drop-down.
        - Configure "Initial Drive state" to "Low (0)" from the drop-down.
    - Select pin "txd[1]" from the checkbox in the pop-up.
        - Click on "Go to signal" of selected pin,
        - Configure "Drive mode" to "Strong Drive, Input buffer off" from the drop-down.
        - Configure "Initial Drive state" to "Low (0)" from the drop-down.
    - Select pin "txd[2]" from the checkbox in the pop-up.
        - Click on "Go to signal" of selected pin,
        - Configure "Drive mode" to "Strong Drive, Input buffer off" from the drop-down.
        - Configure "Initial Drive state" to "Low (0)" from the drop-down.
    - Select pin "txd[3]" from the checkbox in the pop-up.
        - Click on "Go to signal" of selected pin,
        - Configure "Drive mode" to "Strong Drive, Input buffer off" from the drop-down.
        - Configure "Initial Drive state" to "Low (0)" from the drop-down.
    - Select pin "rx_clk" from the drop-down.
        - Click on "Go to signal" of selected pin,
        - Configure "Drive mode" to "Digital HIGH-Z, Input buffer on" from the drop-down.
        - Configure "Initial Drive state" to "Low (0)" from the drop-down.
    - Select pin "rx_ctl" from the drop-down.
        - Click on "Go to signal" of selected pin,
        - Configure "Drive mode" to "Digital HIGH-Z, Input buffer on" from the drop-down.
        - Configure "Initial Drive state" to "Low (0)" from the drop-down.
    - Select pin "rx_er" from the drop-down.
        - Click on "Go to signal" of selected pin,
        - Configure "Drive mode" to "Digital HIGH-Z, Input buffer on" from the drop-down.
        - Configure "Initial Drive state" to "Low (0)" from the drop-down.
    - Select pin "rxd[0]" from the drop-down.
        - Click on "Go to signal" of selected pin,
        - Configure "Drive mode" to "Digital HIGH-Z, Input buffer on" from the drop-down.
        - Configure "Initial Drive state" to "Low (0)" from the drop-down.
    - Select pin "rxd[1]" from the drop-down.
        - Click on "Go to signal" of selected pin,
        - Configure "Drive mode" to "Digital HIGH-Z, Input buffer on" from the drop-down.
        - Configure "Initial Drive state" to "Low (0)" from the drop-down.
    - Select pin "rxd[2]" from the drop-down.
        - Click on "Go to signal" of selected pin,
        - Configure "Drive mode" to "Digital HIGH-Z, Input buffer on" from the drop-down.
        - Configure "Initial Drive state" to "Low (0)" from the drop-down.
    - Select pin "rxd[3]" from the drop-down.
        - Click on "Go to signal" of selected pin,
        - Configure "Drive mode" to "Digital HIGH-Z, Input buffer on" from the drop-down.
        - Configure "Initial Drive state" to "Low (0)" from the drop-down.
    - Save the configuration to generate the necessary code.

6. The *ethernet PHY driver* library disables all the debug log messages by default. To enable log messages, the application must perform the following:
    - Add the `ENABLE_ETH_PHY_DRIVER_LOGS` macro to the *DEFINES* in the code example's Makefile. The Makefile entry would look like as follows:
       ```
       DEFINES+=ENABLE_ETH_PHY_DRIVER_LOGS
       ```
    - Call the `cy_log_init()` function provided by the *cy-log* module. cy-log is part of the *connectivity-utilities* library. See [connectivity-utilities library API documentation](https://infineon.github.io/connectivity-utilities/api_reference_manual/html/group__logging__utils.html) for cy-log details.

## Additional information

- [Ethernet PHY Driver RELEASE.md](./RELEASE.md)

- [Ethernet PHY Driver version](./version.xml)

- [Ethernet PHY Driver API documentation](https://Infineon.github.io/ethernet-phy-driver/api_reference_manual/html/index.html)

- [Connectivity Utilities API documentation - for cy-log details](https://Infineon.github.io/connectivity-utilities/api_reference_manual/html/group__logging__utils.html)

- [ModusToolbox&trade; software environment, quick start guide, documentation, and videos](https://www.cypress.com/products/modustoolbox-software-environment)

- [ModusToolbox&trade; cloud connectivity code examples](https://github.com/Infineon?q=mtb-example-anycloud%20NOT%20Deprecated)
