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

        self.cpp_info.exelinkflags = []
        # if self.settings.os == "baremetal":
        self._setup_baremetal(PLATFORM)

        # DISABLE Conan's config file generation
        self.cpp_info.set_property("cmake_find_mode", "none")
        # Tell CMake to include this directory in its search path
        self.cpp_info.builddirs.append("lib/cmake")


    def _setup_baremetal(self, platform: str):
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
                self._append_linker_using_platform(platform)

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

    def _append_linker_using_platform(self, platform: str):
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
