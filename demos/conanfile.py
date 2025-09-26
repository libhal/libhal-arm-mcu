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

import os

from conan import ConanFile
from conan.tools.cmake import CMake

class demos(ConanFile):
    python_requires = "libhal-bootstrap/[>=4.3.0 <5]"
    python_requires_extend = "libhal-bootstrap.demo"

    def requirements(self):
        self.requires("libhal-util/[^5.4.0]")
        self.requires("libhal-arm-mcu/[1.9.1 || latest]")
        self.tool_requires("picotool/2.1.1")
        self.requires("minimp3/cci.20211201")
        self.requires("micromod-header/1.0.0")
    
    # This is kinda sketch, but needs to be done manually until https://github.com/conan-io/conan/issues/13372
    # gets implemented
    def build(self):
        cmake = CMake(self)
        defs = {
            "CMAKE_ASM_FLAGS_INIT": "-mcpu=cortex-m33 -mfloat-abi=soft",
            "PICO_BOARD": "rp2350_micromod",
            "PICO_CXX_ENABLE_EXCEPTIONS": "1"
        }
        cmake.configure(variables = defs)
        cmake.build()
