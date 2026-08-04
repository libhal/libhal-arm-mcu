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
    SOURCES
        src/stm32f1/gpio.cpp
)

target_link_libraries(arm-mcu
    PRIVATE
        libhal::libhal
        libhal::util
)

libhal_apply_compile_options(arm-mcu)
libhal_install_library(arm-mcu NAMESPACE libhal PACKAGE_NAME libhal-arm-mcu)
libhal_add_tests(arm-mcu
    TEST_NAMES
        gpio
)
