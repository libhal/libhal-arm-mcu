#!/usr/bin/python
#
# Copyright 2024 - 2025 Khalil Estell and the libhal contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from conan import ConanFile
from conan.tools.cmake import CMakeDeps, CMakeToolchain
from conan.tools.env import VirtualBuildEnv
from conan.errors import ConanInvalidConfiguration
from pathlib import Path
import os


required_conan_version = ">=2.0.14"


class libhal_arm_mcu_conan(ConanFile):
    name = "libhal-arm-mcu"
    license = "Apache-2.0"
    homepage = "https://github.com/libhal/libhal-arm-mcu"
    description = ("A collection of libhal drivers and libraries for the "
                   "Cortex M series ARM processors and microcontrollers.")
    topics = ("arm", "cortex", "cortex-m", "cortex-m0", "cortex-m0plus",
              "cortex-m1", "cortex-m3", "cortex-m4", "cortex-m4f", "cortex-m7",
              "cortex-m23", "cortex-m55", "cortex-m35p", "cortex-m33", "lpc",
              "lpc40", "lpc40xx", "lpc4072", "lpc4074", "lpc4078", "lpc4088",
              "stm32f1", "stm32f103")
    settings = "compiler", "build_type", "os", "arch"

    python_requires = "libhal-bootstrap/[>=4.3.0 <5]"
    python_requires_extend = "libhal-bootstrap.library"

    options = {
        "platform": ["ANY"],
        "use_libhal_exceptions": [True, False],
        "use_picolibc": [True, False],
        "use_default_linker_script": [True, False],
        "variant": [None, "ANY"],
        "board": [None, "ANY"],
        "replace_std_terminate": [True, False],
        "use_semihosting": [True, False],
    }
    default_options = {
        "platform": "ANY",
        "use_libhal_exceptions": True,
        "use_picolibc": True,
        "use_default_linker_script": True,
        "replace_std_terminate": True,
        "use_semihosting": True,
        "variant": None,
        "board": None,
    }
    options_description = {
        "platform": "Specifies which platform to provide binaries and build information for",
        "use_libhal_exceptions": "Reserved for backwards compatibility. This option is currently unused and will become functional when libhal-exceptions is feature complete.",
        "use_picolibc": "Use picolibc as the libc runtime for ARM GCC. Note: ARM's LLVM fork always uses picolibc and ignores this option.",
        "use_default_linker_script": "Enable automatic linker script selection based on the specified platform",
        "replace_std_terminate": "Replace the default std::terminate handler to reduce binary size by avoiding verbose text rendering",
        "use_semihosting": "Enables semihosting support, allowing the MCU to perform host based I/O like writing to stdout or reading from files via the debug port. With LLVM from arm-toolchain, semihosting is enabled via the compiler and must be disabled via a build profile option and not this option.",
    }

    def set_version(self):
        # Use latest if not specified via command line
        if not self.version:
            self.version = "latest"

    def requirements(self):
        self.requires("libhal/[^4.18.0]", transitive_headers=True)
        self.requires("libhal-util/[^5.8.1]", transitive_headers=True)
        self.requires("ring-span-lite/[^0.7.0]", transitive_headers=True)
        self.requires("scope-lite/0.2.0")

        if (self.options.use_picolibc and
                self.settings.os == "baremetal" and
                self.settings.compiler == "gcc"):
            CV = str(self.settings.compiler.version)

            CRT0 = "semihost" if self.options.use_semihosting else "default"
            OSLIB = "semihost" if self.options.use_semihosting else None
            self.requires("prebuilt-picolibc/" + CV,
                          options={"crt0": CRT0, "oslib": OSLIB})
    
        if str(self.options.platform).startswith("rp2"):
            self.requires("picosdk/2.2.1-alpha")
            self.tool_requires("pioasm/2.2.0")
            if self.options.board.value.startswith("libhal_"):
                board = self.options.board.value.removeprefix('libhal_').replace('_', '-')
                self.requires(f"rp-board-header-{board}/latest", visible=True)

    def handle_stm32f1_linker_scripts(self):
        linker_script_name = list(str(self.options.platform))
        # Replace the MCU number and pin count number with 'x' (don't care)
        # to map to the linker script
        linker_script_name[8] = 'x'
        linker_script_name[9] = 'x'
        linker_script_name = "".join(linker_script_name)

        self.cpp_info.exelinkflags.extend([
            "-L" + str(Path(self.package_folder) / "linker_scripts"),
            "-T" + str(Path("libhal-stm32f1") / linker_script_name + ".ld"),
        ])

    def _macro(self, string):
        return string.upper().replace("-", "_")

    def generate(self):
        virt = VirtualBuildEnv(self)
        virt.generate()
        tc = CMakeToolchain(self)
        if str(self.options.platform).startswith("rp2"):
            tc.cache_variables["DO_NOT_BUILD_BOOT_HAL"] = True
            tc.preprocessor_definitions["PICO_STDIO_SHORT_CIRCUIT_CLIB_FUNCS"] = "0"
            if self.options.board:
                tc.cache_variables["PICO_BOARD"] = str(self.options.board)
        if self.options.variant:
            tc.preprocessor_definitions["LIBHAL_VARIANT_" + self._macro(str(self.options.variant))] = "1"
        tc.preprocessor_definitions["LIBHAL_PLATFORM_" + self._macro(str(self.options.platform))] = "1"
        tc.generate()
        cmake = CMakeDeps(self)
        cmake.generate()

    def validate(self):
        if str(self.options.platform).startswith("rp2"):
            if not self.options.board:
                raise ConanInvalidConfiguration("RP board not specified")
            if "rp2350" in str(self.options.platform):
                if not self.options.variant:
                    raise ConanInvalidConfiguration("RP2350 variant not specified")
                if self.options.variant not in ["rp2350a", "rp2350b"]:
                    raise ConanInvalidConfiguration("Invalid RP2350 variant specified")
                if not self.options.board:
                    raise ConanInvalidConfiguration("Board must be specified during build")
        super().validate()

    def package_info(self):
        self.cpp_info.libs = ["libhal-arm-mcu"]
        self.cpp_info.set_property("cmake_target_name", "libhal::arm-mcu")
        self.cpp_info.set_property("cmake_target_aliases", [
            "libhal::lpc40",
            "libhal::stm32f1",
            "libhal::stm32f4",
            "libhal::rp2350"
        ])

        PLATFORM = str(self.options.platform)
        self.buildenv_info.define("LIBHAL_PLATFORM", PLATFORM)
        self.buildenv_info.define("LIBHAL_PLATFORM_LIBRARY", "arm-mcu")
        if str(self.options.platform).startswith("rp2"):
            defines = []
            if self.options.variant:
                defines.append("LIBHAL_VARIANT_" + self._macro(str(self.options.variant)) + "=1")
            defines.append("LIBHAL_PLATFORM_" + self._macro(str(self.options.platform)) + "=1")
            defines.append("PICO_STDIO_SHORT_CIRCUIT_CLIB_FUNCS=0")
            self.cpp_info.defines = defines

        self.cpp_info.exelinkflags = []
        if self.settings.os == "baremetal":
            self.setup_baremetal(PLATFORM)

    def package_id(self):
        self.info.python_requires.major_mode()
        self.info.options.clear()

    def setup_baremetal(self, platform: str):
        if self.options.replace_std_terminate:
            self.cpp_info.exelinkflags.extend([
                # Override picolibc's default hard fault handler to gracefully
                # handle semihosting BKPT instructions when no debugger is
                # attached. Without this, binaries linked with semihosting
                # libraries will hang in an infinite loop if executed without a
                # debugger. This wrapper detects BKPT-induced faults, skips the
                # instruction, and allows execution to continue, enabling test
                # packages to link successfully while allowing applications to
                # run standalone.
                "-Wl,--wrap=arm_hardfault_isr",
                # Override the default standard set and get terminate functions
                # to prevent linking in the original default verbose terminate
                # implementation.
                "-Wl,--wrap=_ZSt13set_terminatePFvvE",
                "-Wl,--wrap=_ZSt13get_terminatev",
            ])

        if self.options.replace_std_terminate:
            if self.settings.compiler == "clang":
                self.cpp_info.exelinkflags.extend([
                    # Overrides the terminate handler from LLVM
                    # This results in a large reduction in binary size since this
                    # terminate handler renders text and that text rendering is
                    # expensive.
                    "-Wl,--wrap=__cxa_terminate_handler",
                ])
            if self.settings.compiler == "gcc":
                self.cpp_info.exelinkflags.extend([
                    # Override the terminate handler for GCC.
                    # This results in a large reduction in binary size since this
                    # terminate handler renders text and that text rendering is
                    # expensive.
                    "-Wl,--wrap=_ZN10__cxxabiv119__terminate_handlerE",
                ])

        if self.options.use_default_linker_script:
            LINKER_SCRIPTS_PATH = Path(self.package_folder) / "linker_scripts"
            # If the platform matches the linker script, just use that linker
            # script
            self.cpp_info.exelinkflags.append("-L" + str(LINKER_SCRIPTS_PATH))

            FULL_LINKER_PATH: Path = LINKER_SCRIPTS_PATH / (platform + ".ld")
            # if the file exists, then we should use it as the linker
            if FULL_LINKER_PATH.exists():
                self.output.info(f"linker file '{FULL_LINKER_PATH}' found!")
                self.cpp_info.exelinkflags.append("-T" + platform + ".ld")
            else:
                # if there is no match, then the linker script could be a
                # pattern based on the name of the platform
                self.append_linker_using_platform(platform)

            if self.settings.compiler == "gcc":
                self.cpp_info.exelinkflags.append("-Tpicolibc_gcc.ld")
            if self.settings.compiler == "clang":
                self.cpp_info.exelinkflags.append("-Tpicolibc_llvm.ld")

        package_folder = Path(self.package_folder)
        LIB_PATH = package_folder / 'lib' / 'liblibhal-arm-mcu.a'
        self.cpp_info.exelinkflags.extend([
            # Ensure that all symbols are added to the linker's symbol table
            # This is critical in order for the wrapped symbols to make it to
            # the final link binary with --gc-sections enabled.
            # NOTE: gc sections still works as expected, it just doesn't miss
            # any symbols from this archive.
            "-Wl,--whole-archive",
            str(LIB_PATH),
            "-Wl,--no-whole-archive",
        ])

    def append_linker_using_platform(self, platform: str):
        if platform.startswith("stm32f1"):
            linker_script_name = list(str(self.options.platform))
            # Replace the MCU number and pin count number with 'x' (don't care)
            # to map to the linker script
            linker_script_name[8] = 'x'
            linker_script_name[9] = 'x'
            linker_script_name = "".join(linker_script_name)
            self.cpp_info.exelinkflags.append(
                "-T" + linker_script_name + ".ld")
            return
        # Add additional script searching queries here
