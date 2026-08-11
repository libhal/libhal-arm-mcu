# Copyright 2024 - 2025 Khalil Estell and the libhal contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 4.0)
set(LIBHAL_ENABLE_CLANG_TIDY OFF)
project(arm-mcu LANGUAGES CXX)

# libhal cmake utilities from libhal-cmake-util
# Source: https://github.com/libhal/libhal-cmake-util
find_package(LibhalCMakeUtil REQUIRED)

find_package(libhal REQUIRED)
find_package(libhal-util REQUIRED)

libhal_project_init()
libhal_add_library(arm-mcu
    MODULES
        modules/arm_mcu.cppm


        modules/cortex_m/cortex_m.cppm
        modules/cortex_m/dwt_counter.cppm
        modules/cortex_m/interrupt.cppm
        modules/cortex_m/startup.cppm
        modules/cortex_m/system_control.cppm
        modules/cortex_m/systick_timer.cppm
        modules/cortex_m/terminate_handler.cpp

        modules/stm32_generic/stm32_generic.cppm
        modules/stm32_generic/registers.cppm
        modules/stm32_generic/i2c.cppm
        modules/stm32_generic/pwm.cppm
        modules/stm32_generic/spi.cppm
        modules/stm32_generic/timer.cppm
        modules/stm32_generic/uart.cppm

        modules/stm32f1/stm32f1.cppm
        modules/stm32f1/constants.cppm
        modules/stm32f1/power.cppm
        modules/stm32f1/clock.cppm
        modules/stm32f1/dma.cppm
        modules/stm32f1/pin.cppm
        modules/stm32f1/gpio.cppm
        modules/stm32f1/usart.cppm
        modules/stm32f1/adc.cppm
        modules/stm32f1/spi.cppm
        modules/stm32f1/independent_watchdog.cppm
    SOURCES
        src/dwt_counter.cpp

        src/stm32f1/gpio.cpp
        src/stm32f1/clock.cpp
        src/stm32f1/usart.cpp
        src/stm32f1/adc.cpp
        src/stm32f1/spi.cpp
        src/stm32f1/independent_watchdog.cpp
)

target_link_libraries(arm-mcu
    PRIVATE
        libhal::libhal
        libhal::util
)

# ---------------------------------------------------------------------------
# Baremetal link configuration
#
# LIBHAL_REPLACE_STD_TERMINATE, LIBHAL_USE_DEFAULT_LINKER_SCRIPT and
# LIBHAL_LINKER_SCRIPT are forwarded from the Conan recipe via CMakeToolchain
# variables (see conanfile.py generate(), where the platform name is resolved
# to a concrete linker script file). They're baked into arm-mcu's INTERFACE
# link options here so the resulting flags are captured in the *_targets
# export written by libhal_install_library() below, and therefore reach
# consumers through find_package(libhal-arm-mcu) even though Conan's own
# CMakeDeps config generation is disabled for this package
# (cmake_find_mode = "none" in package_info()).
# ---------------------------------------------------------------------------
option(LIBHAL_REPLACE_STD_TERMINATE "Wrap std::terminate to reduce binary size" ON)
option(LIBHAL_USE_DEFAULT_LINKER_SCRIPT "Select a linker script based on platform" ON)
set(LIBHAL_LINKER_SCRIPT "" CACHE STRING "Resolved linker script base name (without .ld), computed by the Conan recipe")

if(CMAKE_CROSSCOMPILING)
if(LIBHAL_REPLACE_STD_TERMINATE)
    target_link_options(arm-mcu INTERFACE
        # Override picolibc's default hard fault handler to gracefully handle
        # semihosting BKPT instructions when no debugger is attached. Without
        # this, binaries linked with semihosting libraries will hang in an
        # infinite loop if executed without a debugger. This wrapper detects
        # BKPT-induced faults, skips the instruction, and allows execution to
        # continue, enabling test packages to link successfully while
        # allowing applications to run standalone.
        -Wl,--wrap=arm_hardfault_isr
        # Override the default standard set/get terminate functions to
        # prevent linking in the original default verbose terminate
        # implementation.
        -Wl,--wrap=_ZSt13set_terminatePFvvE
        -Wl,--wrap=_ZSt13get_terminatev

        -Wl,--wrap=sys_semihost
        -Wl,--wrap=sys_semihost_get_cmdline
        # Override the terminate handler. This results in a large reduction
        # in binary size since the default terminate handler renders text
        # and that text rendering is expensive.
        "$<$<CXX_COMPILER_ID:Clang>:-Wl,--wrap=__cxa_terminate_handler>"
        "$<$<CXX_COMPILER_ID:GNU>:-Wl,--wrap=_ZN10__cxxabiv119__terminate_handlerE>"
    )
endif()

if(LIBHAL_USE_DEFAULT_LINKER_SCRIPT AND LIBHAL_LINKER_SCRIPT)
    target_link_options(arm-mcu INTERFACE
        "$<BUILD_INTERFACE:-L${CMAKE_CURRENT_SOURCE_DIR}/linker_scripts>"
        "$<INSTALL_INTERFACE:-L$<INSTALL_PREFIX>/linker_scripts>"
        "-T${LIBHAL_LINKER_SCRIPT}.ld"
        "$<$<CXX_COMPILER_ID:GNU>:-Tpicolibc_gcc.ld>"
        "$<$<CXX_COMPILER_ID:Clang>:-Tpicolibc_llvm.ld>"
    )

    install(
        DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/linker_scripts/
        DESTINATION linker_scripts
    )
endif()
# Ensure that all symbols in this archive are added to the linker's symbol
# table. This is critical in order for the wrapped symbols above to make it
# to the final link binary with --gc-sections enabled. NOTE: gc sections
# still works as expected, it just doesn't miss any symbols from this
# archive.
#
# NOTE: the archive file name is computed here (rather than via a
# $<TARGET_FILE:arm-mcu> generator expression) because self-referencing
# TARGET_FILE genexes only get rewritten to the imported target on export
# when used as a whole INTERFACE_LINK_LIBRARIES entry, not when spliced into
# a raw flag string via target_link_options - consumers would otherwise fail
# to resolve a target literally named "arm-mcu".
set(_libhal_arm_mcu_archive_name "${CMAKE_STATIC_LIBRARY_PREFIX}arm-mcu${CMAKE_STATIC_LIBRARY_SUFFIX}")
target_link_options(arm-mcu INTERFACE
    -Wl,--whole-archive
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/${_libhal_arm_mcu_archive_name}>"
    "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_LIBDIR}/${_libhal_arm_mcu_archive_name}>"
    -Wl,--no-whole-archive
)
endif()

libhal_apply_compile_options(arm-mcu)
libhal_install_library(arm-mcu NAMESPACE libhal PACKAGE_NAME libhal-arm-mcu)
libhal_add_tests(arm-mcu
    TEST_NAMES
        gpio
)
