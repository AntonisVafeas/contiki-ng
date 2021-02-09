# Contiki-NG devel branch fork

SPHERE SPG-2 and Wearable3 Drivers and Bluetooth implementation on Contiki-NG devel branch fork
This repository is a branch of Contiki-NG develop branch later updated to release-4.6

## Additions to Contiki-NG

```sh
.
├── .devcontainer                        
│   ├── devcontainer.json                   # VSCode container development settings
│   └── Dockerfile                          # Docker Container build file
├── .vscode
│   ├── c_cpp_properties.json               # Intellisense configurations for different SPHERE hardware examples 
│   └── settings.json                       # VSCode workspace settings
├── arch
│   ├── cpu
│   │   ├── cc26x0-cc13x0
│   │   │   ├── rf-core
│   │   │   │   ├── ble-hal                 # Implementation of BLE queue for Advertising/Scanning (modifications)
│   │   │   │   │   ├── ble-hal-cc26xx.c    # BLE advertisement and scanning queue and whitelisting implementation
│   │   │   │   │   ├── ble-hal-cc26xx.h    # Interrupt Poll Buffers
│   │   │   │   │   ├── rf-ble-cmd.c        # Advertising and scanning queue instructions with whitelisting 
│   │   │   │   │   └── rf-ble-cmd.h        # Advertising and scanning queue instructions with whitelisting 
│   │   │   │   ├── ble-cc2650.c            # Whitelisting, scanner status,advertising queue and scan request implementation
│   │   │   │   ├── rf-ble.c                # Enable Debugging
│   │   │   │   ├── rf-core.c               # Implementation for directive BLE_CONF_WITH_SCANNER
│   │   │   │   ├── rf-core.h               # IRQ handling for BLE Transmissions
│   └── platform
│       ├── cc26x0-cc13x0
│       │   ├── common                      # Drivers ported from sphere-cng
│       │   ├── common-sphere
│       │   │   ├── ble-adv-def.h           # BLE Advertisement Definitions
│       │   │   ├── board-buttons.c         # Touch button pull configuration
│       │   │   ├── fgbridge.*              # SPI master implementation on SPG-2
│       │   │   ├── flash-stats.*           # Ported state saving from sphere-cng
│       │   │   ├── gslave.*                # SPI slave implementation on SPG-2
│       │   │   ├── hw-watchdog.*           # Hardware watchdog for SPG-2 
│       │   │   ├── icm20948.*              # Gyroscope driver for SPHERE Wearable3 v2.1
│       │   │   ├── mc3672.*                # Accelerometer driver for SPHERE Wearable3 v2.1
│       │   │   ├── mcp73832t.*             # Battery charging status driver for SPHERE Wearable3 v2.1
│       │   │   ├── oled.*                  # Screen driver for SPHERE Wearable3 v2.1
│       │   │   ├── touch-button.c          # OPSUN33F Capacitive button for SPHERE Wearable3 v2.1
│       │   │   ├── tps62746.*              # Battery voltage switch driver for SPHERE Wearable3 v2.1
│       │   ├── spg-2
│       │   │   ├── cc2650
│       │   │   │   ├── board.h             # Header file with definitions related to the I/O connections on the SPG-2 
│       │   │   │   └── Makefile.cc2650     # SPG-2  makefile
│       │   │   ├── board-peripherals.h     # Defines related to the peripherals of the SPHERE SPG-2
│       │   │   ├── board.c                 # Board initialisation driver for the SPHERE SPG-2
│       │   │   ├── leds-arch.c             # SPHERE SPG-2 led driver
│       │   │   ├── Makefile.spg-2          # SPHERE SPG-2 makefile
│       │   │   └── spg-2-sensors.c          # Generic module controlling sensors on the SPHERE SPG-2
│       │   ├── wearable3
│       │   │   ├── cc2650
│       │   │   │   ├── board.h             # Header file with definitions related to the I/O connections on the SPHERE Wearable3 v2.1
│       │   │   │   ├── leds-arch.c         # SPHERE Wearable3 v2.1 led driver
│       │   │   │   └── Makefile.cc2650     # SPHERE Wearable3 v2.1 makefile
│       │   │   ├── board-peripherals.h     # Defines related to the peripherals of the SPHERE Wearable3 v2.1
│       │   │   ├── board.c                 # Board initialisation driver for the SPHERE Wearable3 v2.1
│       │   │   ├── Makefile.wearable3      # SPHERE Wearable3 v2.1 makefile
│       │   │   └── wearable3-sensors.c     # Generic module controlling sensors on the SPHERE Wearable3 v2.1
│       │   ├── Makefile.cc26x0-cc13x0      # Platform makefile with SPHERE Wearable3 v2.1 and SPG-2 
│       │   └── platform.c                  # Hardware watchdog initialisation addition
├── examples
│   ├── sphere
│   │   ├── generic
│   │   │   ├── ble-adv-example             # Compatibility generic BLE advertisement example
│   │   │   │   ├── Makefile
│   │   │   │   ├── node.c
│   │   │   │   └── project-conf.h
│   │   │   └── ble-scanner-example         # BLE Scanner with queues and interrupt callbacks
│   │   │       ├── Makefile
│   │   │       ├── node.c
│   │   │       └── project-conf.h
│   │   ├── sphere-wearable3
│   │   │   ├── ble-adv-test                # Accelerometer BLE on-demand advertisement with radio queue implementation
│   │   │   │   ├── Makefile
│   │   │   │   ├── project-conf.h
│   │   │   │   └── sphere-wearable.c
│   │   │   └── screen-interfacing          # Example with button and screen functionality
│   │   │       ├── Makefile
│   │   │       ├── program.sh
│   │   │       ├── project-conf.h
│   │   │       ├── sphere-wearable.c
│   │   │       └── sphere-wearable.cc26x0-cc13x0
│   │   └── Makefile                        # Makefile for all examples
├── os
│   ├── dev
│   │   ├── ble-hal.h                       # Implementation of configurable scan request
│   │   ├── spi.h                           # SPI slave implementation
├── README.md                               
```

<img src="https://github.com/contiki-ng/contiki-ng.github.io/blob/master/images/logo/Contiki_logo_2RGB.png" alt="Logo" width="256">

## Contiki-NG: The OS for Next Generation IoT Devices

[![Github Actions](https://github.com/contiki-ng/contiki-ng/workflows/CI/badge.svg?branch=develop)](https://github.com/contiki-ng/contiki-ng/actions)
[![Documentation Status](https://readthedocs.org/projects/contiki-ng/badge/?version=master)](https://contiki-ng.readthedocs.io/en/master/?badge=master)
[![license](https://img.shields.io/badge/license-3--clause%20bsd-brightgreen.svg)](https://github.com/contiki-ng/contiki-ng/blob/master/LICENSE.md)
[![Latest release](https://img.shields.io/github/release/contiki-ng/contiki-ng.svg)](https://github.com/contiki-ng/contiki-ng/releases/latest)
[![GitHub Release Date](https://img.shields.io/github/release-date/contiki-ng/contiki-ng.svg)](https://github.com/contiki-ng/contiki-ng/releases/latest)
[![Last commit](https://img.shields.io/github/last-commit/contiki-ng/contiki-ng.svg)](https://github.com/contiki-ng/contiki-ng/commit/HEAD)

Contiki-NG is an open-source, cross-platform operating system for Next-Generation IoT devices. It focuses on dependable (secure and reliable) low-power communication and standard protocols, such as IPv6/6LoWPAN, 6TiSCH, RPL, and CoAP. Contiki-NG comes with extensive documentation, tutorials, a roadmap, release cycle, and well-defined development flow for smooth integration of community contributions.

Unless explicitly stated otherwise, Contiki-NG sources are distributed under
the terms of the [3-clause BSD license](LICENSE.md). This license gives
everyone the right to use and distribute the code, either in binary or
source code format, as long as the copyright license is retained in
the source code.

Contiki-NG started as a fork of the Contiki OS and retains some of its original features.

Find out more:

* GitHub repository: https://github.com/contiki-ng/contiki-ng
* Documentation: https://github.com/contiki-ng/contiki-ng/wiki
* Web site: http://contiki-ng.org
* Nightly testbed runs: https://contiki-ng.github.io/testbed

Engage with the community:

* Contiki-NG tag on Stack Overflow: https://stackoverflow.com/questions/tagged/contiki-ng
* Gitter: https://gitter.im/contiki-ng
* Twitter: https://twitter.com/contiki_ng