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
        "replace_std_terminate": [True, False],
    }

    default_options = {
        "platform": "ANY",
        "use_libhal_exceptions": True,
        "use_picolibc": True,
        "use_default_linker_script": True,
        "replace_std_terminate": [True, False],
    }
    options_description = {
        "platform": "Tells libhal-arm-mcu which platform to provide binaries and build information for",
        "use_libhal_exceptions": "This option is no longer used and is only here for backwards compatibility. This option will become usable in the future when libhal-exceptions is feature complete.",
        "use_picolibc": "Use picolibc as the libc runtime for ARM GCC. ARM's LLVM fork uses picolibc as its default libc and cannot be changed by this option.",
        "use_default_linker_script": "",
        "replace_std_terminate": "",
    }

    def requirements(self):
        self.requires("libhal/[^4.18.0]", transitive_headers=True)
        self.requires("libhal-util/[^5.8.1]", transitive_headers=True)
        self.requires("ring-span-lite/[^0.7.0]", transitive_headers=True)
        self.requires("scope-lite/0.2.0")

        if (self.options.use_picolibc and
                self.settings.os == "baremetal" and
                self.settings.compiler == "gcc"):
            CV = str(self.settings.compiler.version)
            # we use hosted because arm-mcu provides its own weak
            # implementation of the _exit() API which simply spins. The default
            # crt0 doesn't seem to work so we'ved decided on this.
            self.requires("prebuilt-picolibc/" + CV,
                          options={"crt0": "hosted"})

    def handle_stm32f1_linker_scripts(self):
        linker_script_name = list(str(self.options.platform))
        # Replace the MCU number and pin count number with 'x' (don't care)
        # to map to the linker script
        linker_script_name[8] = 'x'
        linker_script_name[9] = 'x'
        linker_script_name = "".join(linker_script_name)

        self.cpp_info.exelinkflags.append([
            "-L" + os.path.join(self.package_folder, "linker_scripts"),
            "-T" + os.path.join("libhal-stm32f1", linker_script_name + ".ld"),
        ])

    def package_info(self):
        self.cpp_info.libs = ["libhal-arm-mcu"]
        self.cpp_info.set_property("cmake_target_name", "libhal::arm-mcu")
        self.cpp_info.set_property("cmake_target_aliases", [
            "libhal::lpc40",
            "libhal::stm32f1",
            "libhal::stm32f4",
        ])

        self.cpp_info.exelinkflags = []

        if self.settings.os == "baremetal":
            self.cpp_info.exelinkflags.extend([
                # Overrides the terminate handler from LLVM
                # This results in a large reduction in binary size since this
                # terminate handler renders text and that text rendering is
                # expensive.
                "-Wl,--wrap=__cxa_terminate_handler",
                # Override the terminate handler for GCC.
                # This results in a large reduction in binary size since this
                # terminate handler renders text and that text rendering is
                # expensive.
                "-Wl,--wrap=_ZN10__cxxabiv119__terminate_handlerE",
                # Override picolibc's default hard fault handler to gracefully
                # handle semihosting BKPT instructions when no debugger is
                # attached. Without this, binaries linked with semihosting
                # libraries will hang in an infinite loop if executed without a
                # debugger. This wrapper detects BKPT-induced faults, skips the
                # instruction, and allows execution to continue, enabling test
                # packages to link successfully while allowing applications to
                # run standalone. "-Wl,--wrap=arm_hardfault_isr", Override the
                # default standard set and get terminate functions to prevent
                # linking in the original default verbose terminate
                # implementation.
                "-Wl,--wrap=_ZSt13set_terminatePFvvE",

                # ==============================================================
                # NOTE: Uncomment line below to replace the std::get_terminate
                #      function
                # ==============================================================
                # "-Wl,--wrap=_ZSt13get_terminatev",
            ])

        platform = str(self.options.platform)
        self.buildenv_info.define("LIBHAL_PLATFORM", platform)
        self.buildenv_info.define("LIBHAL_PLATFORM_LIBRARY", "arm-mcu")

        LINKER_SCRIPTS_PATH = Path(self.package_folder) / "linker_scripts"

        if (self.settings.os == "baremetal" and
                self.options.use_default_linker_script):

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

    def package_id(self):
        self.info.python_requires.major_mode()
        del self.info.options.use_picolibc
        del self.info.options.use_libhal_exceptions
        del self.info.options.platform
        del self.info.options.use_default_linker_script

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
