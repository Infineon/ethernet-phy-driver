# Ethernet PHY Driver

## Overview

- The library provides the Ethernet PHY-related interface APIs as required by Ethernet connection manager library to enable the completion of Ethernet-based applications on supported platforms. 
The current version of ethernet-phy-driver library has Ethernet PHY-related interface APIs implemented for PHY chip DP83867IR.

## Features and functionality

- The library contains the necessary ethernet PHY interface APIs for the ethernet-connection-manager library, enabling ethernet-based applications to operate on supported ethernet PHY chips.
- Supports the Ethernet PHY driver for the PHY chip DP83867IR.

## Supported platforms

This library and its features are supported on the following Infineon platforms:

- [XMC7200D-E272K8384 kit (KIT-XMC72-EVK)](https://www.infineon.com/KIT_XMC72_EVK)

## Quick Start
1. To download ethernet PHY driver, create the following *ethernet-phy-driver.mtb* file in application deps folder.
   - *ethernet-phy-driver.mtb:* <br>
      `https://github.com/Infineon/ethernet-phy-driver#latest-v1.X#$$ASSET_REPO$$/ethernet-phy-driver/latest-v1.X`

2. To use ethernet-phy-driver library on FreeRTOS, lwIP, and Mbed TLS combination, the application should pull [ethernet-core-freertos-lwip-mbedtls](https://github.com/Infineon/ethernet-core-freertos-lwip-mbedtls) library version 2.X which will internally pull secure-sockets, ethernet-connection-manager, FreeRTOS, lwIP, Mbed TLS and other dependent modules.<br>
To pull ethernet-core-freertos-lwip-mbedtls create the following *.mtb* file in deps folder.
   - *ethernet-core-freertos-lwip-mbedtls.mtb:*
      `https://github.com/Infineon/ethernet-core-freertos-lwip-mbedtls#latest-v2.X#$$ASSET_REPO$$/ethernet-core-freertos-lwip-mbedtls/latest-v2.X`

3. The *ethernet PHY driver* library disables all the debug log messages by default. To enable log messages, the application must perform the following:
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
