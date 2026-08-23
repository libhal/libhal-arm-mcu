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
from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout


class demos(ConanFile):
    settings = "compiler", "build_type", "os", "arch"
    options = {"platform": ["ANY"]}
    default_options = {"platform": "stm32f103c8"}

    def layout(self):
        build_path = Path("build") / (
            str(self.options.platform) + "-" +
            str(self.settings.compiler) + "-" +
            str(self.settings.compiler.version)
        )
        cmake_layout(self, build_folder=str(build_path))

    def build_requirements(self):
        self.tool_requires("cmake/[^4.0.0]")
        self.tool_requires("ninja/[^1.13.1]")
        self.tool_requires("libhal-cmake-util/[^5.0.5]")

    def requirements(self):
        self.requires("libhal-arm-mcu/[^2.0.0]")
        self.requires("libhal-usb/latest")
        self.requires("libhal/[^5.0.0]")
        self.requires("libhal-util/[^6.0.0]")

    def generate(self):
        tc = CMakeToolchain(self)
        tc.generator = "Ninja"
        tc.variables["LIBHAL_PLATFORM"] = str(self.options.platform)
        tc.generate()

        deps = CMakeDeps(self)
        deps.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()
