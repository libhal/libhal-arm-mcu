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
from conan.tools.cmake import CMake, cmake_layout, CMakeToolchain, CMakeDeps
from conan.tools.files import copy
from conan.errors import ConanInvalidConfiguration
from conan.tools.build import check_min_cppstd
from conan.tools.scm import Version
from pathlib import Path


required_conan_version = ">=2.2.0"


class libhal_arm_mcu_conan(ConanFile):
    name = "libhal-arm-mcu"
    license = "Apache-2.0"
    url = "https://github.com/libhal/libhal-arm-mcu"
    description = ()
    topics = ()
    settings = "compiler", "build_type", "os", "arch"
    exports_sources = "modules/*", "src/*", "tests/*", "CMakeLists.txt", "LICENSE", "linker_scripts/*"
    package_type = "static-library"
    shared = False

    options = {
        "platform": ["ANY"],
        "use_libhal_exceptions": [True, False],
        "use_picolibc": [True, False],
        "use_default_linker_script": [True, False],
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
    }
    options_description = {
        "platform": "Specifies which platform to provide binaries and build information for",
        "use_libhal_exceptions": "Reserved for backwards compatibility. This option is currently unused and will become functional when libhal-exceptions is feature complete.",
        "use_picolibc": "Use picolibc as the libc runtime for ARM GCC. Note: ARM's LLVM fork always uses picolibc and ignores this option.",
        "use_default_linker_script": "Enable automatic linker script selection based on the specified platform",
        "replace_std_terminate": "Replace the default std::terminate handler to reduce binary size by avoiding verbose text rendering",
        "use_semihosting": "Enables semihosting support, allowing the MCU to perform host based I/O like writing to stdout or reading from files via the debug port. With LLVM from arm-toolchain, semihosting is enabled via the compiler and must be disabled via a build profile option and not this option.",
    }

    @property
    def _min_cppstd(self):
        return "23"

    @property
    def _compilers_minimum_version(self):
        # We may reduce these in the future.
        return {
            "gcc": ("14", "GCC 14+ required for libhal"),
            "clang": (
                "19",
                "Clang 19+ required for libhal"
            ),
            "apple-clang": (
                "19.0.0",
                "Apple Clang 19+ for libhal"
            ),
            "msvc": (
                "193.4",
                "MSVC 14.34+ (Visual Studio 17.4+) for libhal"
            )
        }

    def _validate_compiler_version(self):
        """Validate compiler version against minimum requirements"""
        compiler = str(self.settings.compiler)
        version = str(self.settings.compiler.version)

        # Map Visual Studio to msvc for consistency
        compiler_key = "msvc" if compiler == "Visual Studio" else compiler

        min_versions = self._compilers_minimum_version
        if compiler_key not in min_versions:
            raise ConanInvalidConfiguration(
                f"Compiler {compiler} is not supported for C++20 modules")

        min_version, error_msg = min_versions[compiler_key]
        if Version(version) < min_version:
            raise ConanInvalidConfiguration(error_msg)

    def set_version(self):
        # Use latest if not specified via command line
        if not self.version:
            self.version = "latest"

    def validate(self):
        if self.settings.get_safe("compiler.cppstd"):
            check_min_cppstd(self, self._min_cppstd)

        self._validate_compiler_version()

    def build_requirements(self):
        self.tool_requires("cmake/[^4.0.0]")
        self.tool_requires("ninja/[^1.3.0]")
        self.test_requires("boost-ext-ut/2.3.1")
        self.tool_requires("libhal-cmake-util/[^5.0.5]")

    def requirements(self):
        self.requires("libhal/5.0.0")
        self.requires("libhal-util/6.0.0")

    def layout(self):
        build_path = Path("build") / (
            str(self.settings.arch) + "-" +
            str(self.settings.os) + "-" +
            str(self.settings.compiler) + "-" +
            str(self.settings.compiler.version)
        )
        cmake_layout(self, build_folder=str(build_path))

    def generate(self):
        tc = CMakeToolchain(self)
        tc.generator = "Ninja"
        tc.variables["LIBHAL_REPLACE_STD_TERMINATE"] = bool(
            self.options.replace_std_terminate)
        tc.variables["LIBHAL_USE_DEFAULT_LINKER_SCRIPT"] = bool(
            self.options.use_default_linker_script)
        if self.options.use_default_linker_script:
            tc.variables["LIBHAL_LINKER_SCRIPT"] = self._resolve_linker_script(
                str(self.options.platform))
        tc.generate()

        deps = CMakeDeps(self)
        deps.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()
        if not self.conf.get("tools.build:skip_test", default=False):
            cmake.ctest(["--output-on-failure"])

    def package(self):
        cmake = CMake(self)
        cmake.install()

        copy(self, "LICENSE",
             dst=Path(self.package_folder) / "licenses",
             src=self.source_folder)

    def package_info(self):

        PLATFORM = str(self.options.platform)
        self.buildenv_info.define("LIBHAL_PLATFORM", PLATFORM)
        self.buildenv_info.define("LIBHAL_PLATFORM_LIBRARY", "arm-mcu")

        # DISABLE Conan's config file generation. The real CMake config is
        # exported by libhal_install_library() in CMakeLists.txt, which
        # already carries the wrap/linker-script/whole-archive flags baked
        # in as INTERFACE link options (set from the toolchain variables
        # computed in generate()).
        self.cpp_info.set_property("cmake_find_mode", "none")
        # Tell CMake to include this directory in its search path
        self.cpp_info.builddirs.append("lib/cmake")

    def _resolve_linker_script(self, platform: str) -> str:
        """Resolve the platform name to a linker script base name (without
        the .ld extension). If no script matches the platform name exactly,
        falls back to pattern matching based on the platform name."""
        scripts_dir = Path(self.source_folder) / "linker_scripts"

        if (scripts_dir / f"{platform}.ld").exists():
            return platform

        if platform.startswith("stm32f1"):
            # Replace the MCU density code and pin count number with 'x'
            # (don't care) to map to the linker script,
            # e.g. stm32f103c8 -> stm32f10xx8
            linker_script_name = list(platform)
            linker_script_name[8] = 'x'
            linker_script_name[9] = 'x'
            return "".join(linker_script_name)

        # Add additional pattern matching here
        return platform
