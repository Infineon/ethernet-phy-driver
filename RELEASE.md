# Ethernet PHY Driver

## What's included?

See the [README.md](./README.md) for a complete description of the [Ethernet PHY Driver](https://github.com/Infineon/ethernet-phy-driver) library.

## Known issues
| Problem | Workaround |
| ------- | ---------- |
| IAR 9.40 toolchain throws build errors on Debug mode, if application explicitly includes iar_dlmalloc.h file | Add '--advance-heap' to LDFLAGS in application Makefile. |

## Changelog

### v1.0.0

- Initial release for Ethernet PHY Driver library
- Exposes ethernet APIs to configuring ethernet PHY drivers for DP83867IR.

### Supported software and tools

This version of the library was validated for compatibility with the following software and tools:

| Software and tools                                         | Version |
| :---                                                       | :----:  |
| ModusToolbox&trade; software environment                   | 3.2     |
| ModusToolbox&trade; Device Configurator                    | 4.20    |
| GCC Compiler                                               | 11.3.1  |
| IAR Compiler                                               | 9.40    |
| Arm&reg; Compiler 6                                        | 6.16    |
