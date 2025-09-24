# Ethernet PHY Driver

## What's included?

See the [README.md](./README.md) for a complete description of the [Ethernet PHY Driver](https://github.com/Infineon/ethernet-phy-driver) library.

## Known issues
| Problem | Workaround |
| ------- | ---------- |
| IAR 9.40 toolchain throws build errors on Debug mode, if application explicitly includes iar_dlmalloc.h file | Add '--advance-heap' to LDFLAGS in application Makefile. |

## Changelog

### v1.2.0
- Added support for PSOC&trade; Edge E84 (PSE84) platform.
- Enabled LLVM_ARM toolchain support.

### v1.1.0

- Provides ethernet APIs to configuring ethernet PHY drivers for DP83825I.
- Removed Ethernet port pin Configurations. The pins are configured from device configurator and initialization will happen during bsp init.
- Added support to read parameters from device configurator.

### v1.0.0

- Initial release for Ethernet PHY Driver library
- Exposes ethernet APIs to configuring ethernet PHY drivers for DP83867IR.

### Supported software and tools

This version of the library was validated for compatibility with the following software and tools:

| Software and tools                                         | Version |
| :---                                                       | :----:  |
| ModusToolbox&trade; software environment                   | 3.6     |
| ModusToolbox&trade; Device Configurator                    | 5.50    |
| GCC Compiler                                               | 14.2.1  |
| IAR Compiler                                               | 9.50.2  |
| Arm&reg; Compiler 6                                        | 6.22    |
| LLVM compiler                                              | 19.1.5  |
