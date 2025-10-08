# Welcome to **OpenOCD**

**OpenOCD (Open On-Chip Debugger)** provides on-chip programming and debugging support with a layered architecture of **JTAG interface** and **TAP** support including:

---

## Features

- **(X)SVF playback** – facilitates automated boundary scan and FPGA/CPLD programming  
- **Debug target support** (e.g., ARM, MIPS): single-stepping, breakpoints/watchpoints, gprof profiling, etc.  
- **Flash chip drivers** (e.g., CFI, NAND, internal flash)  
- **Embedded Tcl interpreter** for easy scripting  

---

## Network Interfaces

Several network interfaces are available for interacting with OpenOCD: **Telnet**, **Tcl**, and **GDB**.  
The **GDB server** enables OpenOCD to function as a _remote target_ for source-level debugging of embedded systems using **GNU GDB** (and other tools that support the GDB protocol, e.g., **IDA Pro**).

---

## Table of Contents

- [Quickstart](#quickstart)
- [Documentation](#documentation)
- [Supported Hardware](#supported-hardware)
- [Installing OpenOCD](#installing-openocd)
- [Building OpenOCD](#building-openocd)
- [Obtaining OpenOCD from GIT](#obtaining-openocd-from-git)


---

## Quickstart

If you have a popular board then just start OpenOCD with its config, e.g.:

```bash
openocd -f board/stm32f4discovery.cfg
```

If you are connecting a particular adapter with a specific target, you need to source both the JTAG interface and the target configs, e.g.:

```bash
openocd -f interface/ftdi/jtagkey2.cfg -c "transport select jtag" \
        -f target/ti_calypso.cfg
```

or

```bash
openocd -f interface/stlink.cfg -c "transport select swd" \
        -f target/stm32l0.cfg
```
After OpenOCD startup, connect GDB with:

```bash
(gdb) target extended-remote localhost:3333
```

## Documentation
In addition to the in-tree documentation, the latest manuals may be viewed online at:

- OpenOCD User's Guide - http://openocd.org/doc/html/index.html
- OpenOCD Developer's Manual - http://openocd.org/doc/doxygen/html/index.html
- For more information, refer to these documents or contact the developers by subscribing to the OpenOCD developer mailing list - [openocd-devel@lists.sourceforge.net](https://openocd-devel.narkive.com/)

### Building the OpenOCD Documentation
By default, the OpenOCD build process prepares documentation in the Info format and installs it in the standard way, so that `info openocd` can access it.

Additionally, the OpenOCD User's Guide can be produced in the following different formats:

- PDF - `make pdf && ${PDFVIEWER} doc/openocd.pdf`
- HTML -  `make html && ${HTMLVIEWER} doc/openocd.html/index.html`
  
The OpenOCD Developer Manual contains information about the internal architecture and other details about the code:

Make sure doxygen is installed:

```bash
doxygen --version
```

make doxygen && ${HTMLVIEWER} doxygen/index.html

## Supported Hardware
### JTAG Adapters

| Adapter | Adapter | Adapter | Adapter | Adapter | Adapter |
|---------|---------|---------|---------|---------|---------|
| AM335x | ARM-JTAG-EW | ARM-USB-OCD | ARM-USB-TINY | AT91RM9200 | axm0432 |
| BCM2835 | Bus Blaster | Buspirate | Cadence DPI | Cadence vdebug | Chameleon |
| CMSIS-DAP | Cortino | Cypress KitProg | DENX | Digilent JTAG-SMT2 | DLC 5 |
| DLP-USB1232H | embedded projects | Espressif USB JTAG Programmer | eStick | FlashLINK | FlossJTAG |
| Flyswatter | Flyswatter2 | FTDI FT232R | Gateworks | Hoegl | ICDI |
| ICEBear | J-Link | JTAG VPI | JTAGkey | JTAGkey2 | JTAG-lock-pick |
| KT-Link | Linux GPIOD | Lisa/L | LPC1768-Stick | Mellanox rshim | MiniModule |
| NGX | Nuvoton Nu-Link | Nu-Link2 | NXHX | NXP IMX GPIO | OOCDLink |
| Opendous | OpenJTAG | Openmoko | OpenRD | OSBDM | Presto |
| Redbee | Remote Bitbang | RLink | SheevaPlug devkit | Stellaris evkits | ST-LINK (SWO tracing supported) |
| STM32-PerformanceStick | STR9-comStick | sysfsgpio | Tigard | TI XDS110 | TUMPA |
| Turtelizer | ULINK | USB-A9260 | USB-Blaster | USB-JTAG | USBprog |
| VPACLink | VSLLink | Wiggler | XDS100v2 | Xilinx XVC/PCIe | Xverve |

### Debug Targets
- ARM: AArch64, ARM11, ARM7, ARM9, Cortex-A/R (v7-A/R), Cortex-M (ARMv{6/7/8}-M),
- FA526, Feroceon/Dragonite, XScale.
- ARCv2, AVR32, DSP563xx, DSP5680xx, EnSilica eSi-RISC, EJTAG (MIPS32, MIPS64),
- ESP32, ESP32-S2, ESP32-S3, Intel Quark, LS102x-SAP, RISC-V, ST STM8, Xtensa.

### Flash Drivers
| Flask Driver          | Flask Driver      | Flask Driver      | Flask Driver      | Flask Driver      | Flask Driver      |
|----------------------|-----------------|-----------------|-----------------|-----------------|-----------------|
| ADUC702x             | AT91SAM          | AT91SAM9 (NAND) | ATH79             | ATmega128RFA1    | Atmel SAM        |
| AVR                  | CFI              | DSP5680xx        | EFM32             | EM357            | eSi-RISC         |
| eSi-TSMC             | EZR32HG          | FM3              | FM4               | Freedom E SPI    | GD32             |
| i.MX31               | Kinetis          | LPC8xx           | LPC1xxx           | LPC2xxx          | LPC541xx         |
| LPC2900              | LPC3180          | LPC32xx          | LPCSPIFI          | Marvell QSPI     | MAX32            |
| Milandr              | MXC              | NIIET            | nRF51             | nRF52            | NuMicro          |
| NUC910               | Nuvoton NPCX     | onsemi RSL10     | Orion/Kirkwood    | PIC32mx          | PSoC4/5LP/6      |
| Raspberry RP2040      | Renesas RPC HF   | SH QSPI          | S3C24xx           | S3C6400          | SiM3x            |
| SiFive Freedom E      | Stellaris        | ST BlueNRG       | STM32             | STM32 QUAD/OCTO-SPI | STMSMI       |
| STR7x                | STR9x            | SWM050           | TI CC13xx         | TI CC26xx        | TI CC32xx        |
| TI MSP432            | Winner Micro w600| Xilinx XCF       | XMC1xxx           | XMC4xxx          |                  |

## Installing OpenOCD
### A Note to OpenOCD Users
If you prefer to work with OpenOCD rather than on it, your OS or JTAG interface supplier may provide binaries for you.

Such packages may be more stable than git mainline versions.

Users of these binary versions must contact their Packager for support or updates — OpenOCD developers do not support binary packages directly. These **Packagers** produce
binary releases of OpenOCD after the developers produces new **release**
versions of the source code. Previous versions of OpenOCD cannot be
used to diagnose problems with the current release, so users are
encouraged to keep in contact with their distribution package
maintainers or interface vendors to ensure suitable upgrades appear
regularly.

Users of these binary versions of OpenOCD must contact their Packager to
ask for support or newer versions of the binaries; the OpenOCD
developers do not support packages directly.

### A Note on OpenOCD Packagers
#### You are a PACKAGER of OpenOCD if you:

- Sell dongles and include pre-built binaries
- Supply tools or IDEs (a development solution integrating OpenOCD)
- Build packages (e.g. RPM or DEB files for a GNU/Linux distribution)

#### Packager guidelines:

As a PACKAGER, you will experience first reports of most issues.
When you fix those problems for your users, your solution may help
prevent hundreds (if not thousands) of other questions from other users.

If something does not work for you, please work to inform the OpenOCD
developers know how to improve the system or documentation to avoid
future problems, and follow-up to help us ensure the issue will be fully
resolved in our future releases.

## Building OpenOCD
The INSTALL file contains detailed instructions for running configure and compiling OpenOCD.
Below is a quick summary.

### Dependencies
#### Required:

- gcc or clang
- make
- libtool
- pkg-config >= 0.23 or pkgconf
- libjim >= 0.79

#### For building from Git:

- autoconf >= 2.69
- automake >= 1.14
- texinfo >= 5.0

#### Optional:

- libusb-1.0 – USB-based adapters
- libftdi – USB-Blaster, ASIX Presto, OpenJTAG
- hidapi – CMSIS-DAP
- libgpiod – linuxgpiod
- libjaylink – J-Link
- capstone – ARM disassembly

#### Optional script tools:

- perl
- python
- python-ply

### Permissions Delegation
Running OpenOCD as root is discouraged.
For USB devices on Linux, use:
contrib/60-openocd.rules

Place it in /etc/udev/rules.d and add your user to the plugdev group.

**For parallel port adapters:**

- **Linux/FreeBSD:** Change permissions on ppdev (parport* or ppi*) device nodes.
- **Windows:** Run install_giveio.bat (or use ioperm with Cygwin).

### Compiling OpenOCD
Build with:
```bash
./bootstrap
./configure [options]
make
sudo make install
```

- `bootstrap` — only for Git builds
- `configure` — generates Makefiles
- `make` — builds in ./src/
- `make install` — installs system-wide

To list all options:
```bash
./configure --help
```

### Cross-Compiling Options
Cross-compiling example (Windows 32-bit, MinGW on Debian):

```bash
./configure --host=i686-w64-mingw32 [options]
```

For pkg-config cross-compiling setup:
[https://autotools.io/pkgconfig/cross-compiling.html](https://autotools.info/pkgconfig/cross-compiling.html)

Alternatively, specify `*_CFLAGS` and `*_LIBS` directly.

You can use the script:
```bash
contrib/cross-build.sh
```

### Parallel Port Dongles
For PPDEV interface:
```bash
--enable-parport --enable-parport-ppdev
```

For giveio (Windows):
```bash
--enable-parport --enable-parport-giveio
```

## Obtaining OpenOCD from GIT
Download the current version:
```bash
git clone git://git.code.sf.net/p/openocd/code openocd
```
Update with:
```bash
git pull
```

### Alternative mirrors:

- http://repo.or.cz/r/openocd.git
- git://repo.or.cz/openocd.git

## Web interface and snapshots:

🔗 [http://repo.or.cz/w/openocd.git](https://repo.or.cz/w/openocd.git)

Snapshots are compressed tarballs of the source tree (~1.3 MB).
