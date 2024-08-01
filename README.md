# libhal-arm-mcu

[![✅ CI](https://github.com/libhal/libhal-armcortex/actions/workflows/ci.yml/badge.svg)](https://github.com/libhal/libhal-armcortex/actions/workflows/ci.yml)
[![GitHub stars](https://img.shields.io/github/stars/libhal/libhal-armcortex.svg)](https://github.com/libhal/libhal-armcortex/stargazers)
[![GitHub forks](https://img.shields.io/github/forks/libhal/libhal-armcortex.svg)](https://github.com/libhal/libhal-armcortex/network)
[![GitHub issues](https://img.shields.io/github/issues/libhal/libhal-armcortex.svg)](https://github.com/libhal/libhal-armcortex/issues)

This repo contains libhal compatible libraries for numerous ARM Cortex-M
processor microcontrollers (MCUs). This is a platform library supporting
generic ARM processor APIs and peripheral drivers from many different
microcontrollers.

<!--
## 🏁 Startup & Initialization

Startup function to initialize the data section. This is REQUIRED for systems
that do not load themselves from storage into RAM. If the executable is executed
from flash then `hal::cortex_m::initialize_data_section()` is required.

```C++
#include <libhal-armcortex/startup.hpp>

hal::cortex_m::initialize_data_section();
```

The `arm-gnu-toolchain` package provides the `crt0.s` startup file which
initializes the BSS (uninitialized) section of memory. If this startup file
is not used, then a call to `hal::cortex_m::initialize_bss_section()` is
required.

```C++
#include <libhal-armcortex/startup.hpp>

hal::cortex_m::initialize_data_section();
hal::cortex_m::initialize_bss_section();
```

If the device has an FPU (floating point unit) then a call to
`hal::cortex_m::initialize_floating_point_unit()` is required before any
floating point unit registers or floating point instructions.

```C++
#include <libhal-armcortex/system_control.hpp>

hal::cortex_m::initialize_floating_point_unit();
```

## Creating platform profiles

> [!IMPORTANT]
> Section missing.

## How Linker Scripts Work

Linker scripts are used to control the memory layout of the final binary file.
They define the memory regions and sections of the binary, and specify where and
how the linker should place the code and data.

In libhal-armcortex, there is a standard linker script named `standard.ld`
located in the `linker_scripts/libhal-armcortex` directory. This script is
included by other linker scripts in the library to provide a common base
configuration. When `lihal-armcortex` is added as a dependancy of a library or
application the path to the linker scripts in `linker_scripts` directory are
added to the linker flags of the build, making them accessible within other
linker scripts via the `INCLUDE "libhal-armcortex/standard.ld` command.

Here is the content of `standard.ld`:

```ld
INCLUDE "libhal-armcortex/third_party/standard.ld"
```

This script includes another script `third_party/standard.ld` which contains the
actual linker commands. This script has 4 variables that must be defined for
the linker script to work as intended. These variables are

- **`__flash`**: Memory mapped flash memory start address
- **`__flash_size`**: Size of flash memory
- **`__ram`**: Start of RAM address
- **`__ram_size`**: RAM size
- **`__stack_size`** (optional): Size of stack memory. Why is this important?
  Stack memory is needed for functions to operate. It provides them with the
  memory hold local variables. This value provides a safety buffer for the
  system's stack memory. If an application uses up enough statically defined
  memory as to leave no room for the applications stack, exceeds this amount,
  then the linker script will issue an error about running out of memory.

Currently, libhal only provides `standard.ld` which supports a single memory
mapped flash and ram block.

Additional linker scripts for multi-ram, multi-flash, and execute from RAM only
systems are planned to be provided at a later date when systems with those
requirements appear in the ecosystem.

## Using Linker Scripts in a Platform Library

Linker scripts are used in platform libraries to define the memory layout
specific to the platform. For example, in the libhal-lpc40 library, there are
several linker scripts in the `linker_scripts/libhal-lpc40` directory. Each of
these scripts corresponds to a specific model of the lpc40 series of MCUs.

Here is an example of the `lpc4076.ld` linker script from libhal-lpc40:

```ld
__flash = 0x00000000;
__flash_size = 256K;
__ram = 0x10000000;
__ram_size = 64K;
__stack_size = 1K;

INCLUDE "standard.ld"
```

This script defines the memory layout for the lpc4076 MCU. It specifies the
start addresses and sizes of the flash and RAM memory regions, as well as the
stack size. It then includes the standard linker script from libhal-armcortex to
provide the common base configuration.

> [!NOTE]
> libhal-armcortex does not yet support a multiple flash and ram regions.

Every supported microcontroller in the platform library should have an
associated linker script with its name like so `platform_name.ld`. The platform
library should have a `package_info()` section like this in their
`conanfile.py`:

```python
def add_linker_scripts_to_link_flags(self):
    platform = str(self.options.platform)
    self.cpp_info.exelinkflags = [
        "-L" + os.path.join(self.package_folder, "linker_scripts"),
        "-T" + os.path.join("libhal-lpc40", platform + ".ld"),
    ]

def package_info(self):
    self.cpp_info.libs = ["libhal-lpc40"]
    self.cpp_info.set_property("cmake_target_name", "libhal::lpc40")

    if self.settings.os == "baremetal" and self._use_linker_script:
        self.add_linker_scripts_to_link_flags()
        self.buildenv_info.define("LIBHAL_PLATFORM",
                                  str(self.options.platform))
        self.buildenv_info.define("LIBHAL_PLATFORM_LIBRARY",
                                  "lpc40")
```

This will add the platform linker scripts to the linker script flags. The os
check for "baremetal" is important because we only want to add the linker
scripts to baremetal ARM devices, otherwise the OSes linker scripts should be used.

-->

## Contributing

See [`CONTRIBUTING.md`](CONTRIBUTING.md) for details.

## License

Apache 2.0; see [`LICENSE`](LICENSE) for details.

## Source of original finals

The original files came from the soon to be archived repos:

- `libhal/libhal-lpc40`
- `libhal/libhal-stm32f1`
- `libhal/libhal-stm32f4`
- `libhal/libhal-armcortex`
