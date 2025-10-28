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
        "use_libhal_exceptions": [True, False],
        "use_picolibc": [True, False],
        "platform": ["ANY"],
        "use_default_linker_script": [True, False],
    }

    default_options = {
        "use_libhal_exceptions": True,
        "use_picolibc": True,
        "platform": "ANY",
        "use_default_linker_script": True,
    }

    def requirements(self):
        self.requires("libhal/[^4.18.1]", transitive_headers=True)
        self.requires("libhal-util/[^5.8.1]", transitive_headers=True)
        self.requires("ring-span-lite/[^0.7.0]", transitive_headers=True)
        self.requires("scope-lite/0.2.0")

        if self.settings.os == "baremetal" and self.settings.compiler == "gcc":
            if self.options.use_libhal_exceptions:
                self.requires(
                    "libhal-exceptions/[^1.1.1]", transitive_headers=True)
            if self.options.use_picolibc:
                compiler_version = str(self.settings.compiler.version)
                self.requires("prebuilt-picolibc/" + compiler_version)

    def handle_stm32f1_linker_scripts(self):
        linker_script_name = list(str(self.options.platform))
        # Replace the MCU number and pin count number with 'x' (don't care)
        # to map to the linker script
        linker_script_name[8] = 'x'
        linker_script_name[9] = 'x'
        linker_script_name = "".join(linker_script_name)

        self.cpp_info.exelinkflags = [
            "-L" + os.path.join(self.package_folder, "linker_scripts"),
            "-T" + os.path.join("libhal-stm32f1", linker_script_name + ".ld"),
        ]

    def package_info(self):
        self.cpp_info.libs = ["libhal-arm-mcu"]
        self.cpp_info.set_property("cmake_target_name", "libhal::arm-mcu")
        self.cpp_info.set_property("cmake_target_aliases", [
            "libhal::lpc40",
            "libhal::stm32f1",
            "libhal::stm32f4",
        ])
        self.cpp_info.exelinkflags = []

        platform = str(self.options.platform)
        self.buildenv_info.define("LIBHAL_PLATFORM", platform)
        self.buildenv_info.define("LIBHAL_PLATFORM_LIBRARY", "arm-mcu")

        if (self.settings.os == "baremetal" and
                self.options.use_default_linker_script):
            # If the platform matches the linker script, just use that linker
            # script
            self.cpp_info.exelinkflags = [
                "-L" + os.path.join(self.package_folder, "linker_scripts")]

            full_linker_path = os.path.join(
                self.package_folder, "linker_scripts", platform + ".ld")
            # if the file exists, then we should use it as the linker
            if os.path.isfile(full_linker_path):
                self.output.info(f"linker file '{full_linker_path}' found!")
                self.cpp_info.exelinkflags.append("-T" + platform + ".ld")

            # if there is no match, then the linker script could be a pattern
            # based on the name of the platform
            else:
                self.append_linker_using_platform(platform)

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
