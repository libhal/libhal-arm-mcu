# libhal-arm-mcu

[![✅ CI](https://github.com/libhal/libhal-armcortex/actions/workflows/ci.yml/badge.svg)](https://github.com/libhal/libhal-armcortex/actions/workflows/ci.yml)
[![GitHub stars](https://img.shields.io/github/stars/libhal/libhal-armcortex.svg)](https://github.com/libhal/libhal-armcortex/stargazers)
[![GitHub forks](https://img.shields.io/github/forks/libhal/libhal-armcortex.svg)](https://github.com/libhal/libhal-armcortex/network)
[![GitHub issues](https://img.shields.io/github/issues/libhal/libhal-armcortex.svg)](https://github.com/libhal/libhal-armcortex/issues)

This repo contains libhal compatible libraries for numerous ARM Cortex-M
processor microcontrollers (MCUs). This is a platform library supporting
generic ARM processor APIs and peripheral drivers from many different
microcontrollers.

## 📚 Software APIs & Usage

To learn about the available drivers and APIs see the headers
[`include/libhal-arm-mcu`](https://github.com/libhal/libhal-arm-mcu/tree/main/include/libhal-arm-mcu)
directory.

To see how each driver is used see the
[`demos/`](https://github.com/libhal/libhal-arm-mcu/tree/main/demos) directory.

Fully rendered Doxygen APIs will be provided when
[issue#37](https://github.com/libhal/libhal-arm-mcu/issues/37) is closed.

## 🧰 Setup

To get started with libhal, follow the
[🚀 Getting Started](https://libhal.github.io/latest/getting_started/) guide.

## 📡 Installing Profiles

Profiles define which platform you mean to build your project against. These
profiles are needed by the conan files to determine which files to build for
demos and for libraries. In order to build binaries using libhal for ARM
microcontrollers, you'll need to install both the ARM GNU toolchain and the ARM
MCU profiles.

```bash
conan config install -sf conan/profiles/v1 -tf profiles https://github.com/libhal/arm-gnu-toolchain.git
conan config install -sf conan/profiles/v1 -tf profiles https://github.com/libhal/libhal-arm-mcu.git
```

The first command installs the ARM GNU compiler profiles, while the second adds
the ARM MCU platform profiles to your conan `profiles` directory. Note that
running these commands multiple times is safe - they will simply overwrite the
old files with the latest versions.

Now that you have the profiles installed, you can build demos and libraries for
ARM microcontrollers.

## 🏗️ Building Demo Applications

To build demos, start at the root of the repo and execute the following command:

```bash
conan build demos -pr lpc4078 -s arm-gcc-12.3
```

This will build the demos for the `lpc4078` microcontroller in `MinSizeRel`
mode. Replace `lpc4078` with any of the other complete profiles found in the
`./conan/profiles/v1/`. An example of an incomplete profiles do not match an
exact device such as `lpc40` or `stm32f1` which are used to build full
profiles. You can also select the compiler you want to use with the
`arm-gcc-12.3` compiler flag.

Add the flag `-s build_type=Debug` to build in debug mode:

```bash
conan build demos -pr lpc4078 -s arm-gcc-12.3 -s build_type=Debug
```

Build type `Debug`, `MinSizeRel`, and `Release` are all available.

## 💾 Flashing/Programming

There are a few ways to flash an LPC40 series MCU. The recommended methods are
via USB or using a debugger JTAG/SWD.

### Flashing NXP MCUs

[`nxpprog`](https://github.com/libhal/nxpprog) is a script for programming and
flashing LPC40 series chips over serial/UART. Using it will require a USB to
serial/uart adaptor.

See the README on [`nxpprog`](https://github.com/libhal/nxpprog), for details on
how to use NXPPROG.

To install `nxpprog`:

```bash
pipx install nxpprog
```

To flash command is:

```bash
nxpprog --control --binary demos/build/lpc4078/MinSizeRel/blinker.elf.bin --device /dev/tty.usbserial-10
```

- Replace `demos/build/lpc4078/MinSizeRel/blinker.elf.bin` with the path to the
  binary you'd like to flash.
- Replace `/dev/tty.usbserial-10` with the path to your serial port on your
  machine.

### Flashing STM32 Processors

[`stm32loader`](https://pypi.org/project/stm32loader/) is a script for
programming and flashing STM32 series chips over serial/UART. Using it will
require a USB to serial/uart adaptor.

For more information, please refer to the README of
[`stm32loader`](https://pypi.org/project/stm32loader/).

To install stm32loader:

```bash
pipx install stm32loader
```

To flash command is:

```bash
stm32loader -p /dev/tty.usbserial-10 -e -w -v demos/build/stm32f103c8/MinSizeRel/blinker.elf.bin
```

- Replace `demos/build/stm32f103c8/MinSizeRel/blinker.elf.bin` with the path to
  the binary you'd like to flash.
- Replace `/dev/tty.usbserial-10` with the path to your serial port on your
  machine.

### Using JTAG/SWD over PyOCD

`PyOCD` is a debugging interface for programming and also debugging ARM Cortex M
processor devices over JTAG and SWD.

This will require a JTAG or SWD debugger. The recommended debugger for the
LPC40 series of devices is the STLink v2 (cheap variants can be found on
Amazon).

See [PyOCD Installation Page](https://pyocd.io/docs/installing) for installation
details.

For reference the flashing command is:

```bash
pyocd flash --target lpc4088 demos/build/lpc4078/MinSizeRel/blinker.elf.bin
pyocd flash --target stm32f103rc demos/build/stm32f103c8/MinSizeRel/blinker.elf.bin
```

Note that the targets for your exact part may not exist in `pyocd`. Because of
this, it means that the bounds of the memory may not fit your device. It is up
to you to make sure you do not flash a binary larger than what can fit on your
device.

## 📦 Adding `libhal-arm-mcu` to your project

This section assumes you are using the
[`libhal-starter`](https://github.com/libhal/libhal-starter)
project.

Make sure to add the following options and default options to your app's
`ConanFile` class:

```python
    options = {"platform": ["ANY"]}
    default_options = {"platform": "unspecified"}
```

Add the following to your `requirements()` method:

```python
    def requirements(self):
        self.requires("libhal-arm-mcu/[^1.0.0]")
```

The version number can be changed to whatever is appropriate for your
application. If you don't know, using the latest is usually a good choice.

The CMake from the starter project will already be ready to support the new
platform library. No change needed.

To perform a test build simple run `conan build .` as is done above with the
desired target platform profile.

## ❌ Using `libhal-arm-mcu` in your library

This library is a platform library and as such should only be depended upon by
applications. Platform libraries do not require ABI stability and thus do not
guarantee it. Depending on a platform library is undefined behavior if an ABI
break occurs.

## 🌟 Package Semantic Versioning Explained

In libhal, different libraries have different requirements and expectations for
how their libraries will be used and how to interpret changes in the semantic
version of a library.

If you are not familiar with [SEMVER](https://semver.org/) you can click the
link to learn more.

### 💥 Major changes

The major number will increment in the event of:

1. An API break
2. A behavior change

We define an API break as an intentional change to the public interface, found
within the `include/` directory, that removes or changes an API in such a way
that code that previously built would no longer be capable of building.

We define a "behavior change" as an intentional change to the documentation of
a public API that would change the API's behavior such that previous and later
versions of the same API would do observably different things.

The usage of the term "intentional" means that the break or behavior change was
expected and accepted for a release. If an API break occurs on accident when it
wasn't previously desired, then such a change should be rolled back and an
alternative non-API breaking solution should be found.

You can depend on the major number to provide API and behavioral
stability for your application. If you upgrade to a new major numbered version
of libhal, your code and applications may or may not continue to work as
expected or compile. Because of this, we try our best to not update the
major number.

### 🚀 Minor changes

The minor number will increment if a new interface, API, or type is introduced
into the public interface OR an ABI break has occurred. ABI breaks with
applications cause no issue and thus are allowed to be minor implementation
breaking changes.

### 🐞 Patch Changes

The patch number will increment if:

1. Bug fixes that align code to the behavior of an API, improves performance
   or improves code size efficiency.
2. Any changes occur within the `/include/libhal-arm-mcu/experimental`
   directory.

For now, you cannot expect ABI or API stability with anything in the
`/include/libhal-arm-mcu/experimental` directory.

## 🏁 Startup & Initialization

Startup is managed by the [`picolibc`](https://keithp.com/picolibc/) runtime.
In terms of startup `picolibc` has to manage doing two things. For one, it must
construct a minimal interrupt vector table with two entries. The 1st entry is
the address of the top of the stack. The 2nd entry is the address of the
function that will be executed on reset. `picolibc` sets this to its own
`_start` function. `_start` does the following:

1. Sets the main stack registers
2. Write the `.data` section from read-only memory
3. Set the `.bss` section to all zeros
4. Enable FPU if present for the core architecture
5. Calls all globally constructed C++ objects
6. Calls `main()`

If the `.data` or `.bss` sections must initialized manually, there are functions
provided:

```C++
#include <libhal-armcortex/startup.hpp>

hal::cortex_m::initialize_data_section();
hal::cortex_m::initialize_bss_section();
hal::cortex_m::initialize_floating_point_unit();
```

### 🏎️ Setting Clock Speed

To setting the CPU clock speed to the maximum of 120MHz, include the line below,
with the rest of the includes:

```C++
#include <libhal-arm-mcu/lpc40/clock.hpp>
#include <libhal-arm-mcu/stm32f1/clock.hpp>
#include <libhal-arm-mcu/stm32f4/clock.hpp>
// etc..
#include <libhal-arm-mcu/rp2040/clock.hpp>
```

Next run the following command but replace `12.0_MHz` with the crystal
oscillator frequency connected to the microcontroller. This command REQUIRES
that there be a crystal oscillator attached to the microcontroller. Calling
this without the oscillator will cause the device to freeze as it will attempt
to use a clock that does not exist.

```C++
hal::lpc40::maximum(12.0_MHz);
hal::stm32f1::maximum(8.0_MHz);
hal::stm32f4::maximum(10.0_MHz);
// etc...
hal::rp2040::maximum(16.0_MHz);
```

To set the clock rate to the max speed using the internal oscillator:

```C++
hal::lpc40::maximum_speed_using_internal_oscillator();
hal::stm32f1::maximum_speed_using_internal_oscillator();
hal::stm32f4::maximum_speed_using_internal_oscillator();
// etc...
hal::rp2040::maximum_speed_using_internal_oscillator();
```

These APIs may not always exist for all systems, so be sure to check if the API
exists.

#### 🕰️ Detailed Clock Tree Control

Coming soon...

## 🔎 On Chip Software Debugging

### Using PyOCD (✅ RECOMMENDED)

In one terminal:

```bash
pyocd gdbserver --target=lpc4088 --persist
```

In another terminal:

```bash
arm-none-eabi-gdb demos/build/lpc4078/blinker.elf -ex "target remote :3333"
```

Replace `demos/build/lpc4078/blinker.elf` with the path to the elf file you'd
like to use for the debugging session.

### Using OpenOCD

Coming soon... (its more complicated)

## 📦 Building & Installing the Library Package

If you'd like to build and install this package into your local conan cache,
execute the following command:

```bash
conan create . -pr stm32f103c8 -pr arm-gcc-12.3 --version=latest
```

- Replace `latest` with the SEMVER version that fits the changes you've made. Or
  just choose a number greater than whats been released.
- Replace `-pr stm32f103c8` with your desired platform.
- Replace `-pr arm-gcc-12.3` with your desired compiler.
- Add `-s build_type=` to specify the build type you want to build for.

If you want to build the package unit tests without creating a package
installed within your cache, you can replace `create` with `build` and remove
the `--version` flag like so:

```bash
conan build . -pr stm32f103c8 -pr arm-gcc-12.3
```

> [!NOTE]
> Currently, we do not support `clang-tidy` checks on cross builds. So if you
> want to check the package against `clang-tidy` you will need to execute the
> command `conan build .` which will build the package for your computer which
> will allow the unit tests to be executable on your machine. The unit test
> will be executed at the end of the build process.

## :busts_in_silhouette: Contributing

See [`CONTRIBUTING.md`](CONTRIBUTING.md) for details.

## License

Apache 2.0; see [`LICENSE`](LICENSE) for details.

## Source of initial files put into this library

The original files came from the soon to be archived repos:

- [`libhal/libhal-lpc40`](https://github.com/libhal/libhal-lpc40)
- [`libhal/libhal-stm32f1`](https://github.com/libhal/libhal-stm32f1)
- [`libhal/libhal-stm32f4`](https://github.com/libhal/libhal-stm32f4)
- [`libhal/libhal-armcortex`](https://github.com/libhal/libhal-armcortex)
