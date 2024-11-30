# Copyright 2024 Khalil Estell
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

cmake_minimum_required(VERSION 3.15)

project(libhal-arm-mcu LANGUAGES CXX)

libhal_test_and_make_library(
  LIBRARY_NAME libhal-arm-mcu

  SOURCES
  src/system_controller.cpp
  src/dwt_counter.cpp
  src/interrupt.cpp
  src/systick_timer.cpp

  # lpc40
  src/lpc40/adc.cpp
  src/lpc40/can.cpp
  src/lpc40/clock.cpp
  src/lpc40/dac.cpp
  src/lpc40/dma.cpp
  src/lpc40/i2c.cpp
  src/lpc40/input_pin.cpp
  src/lpc40/interrupt_pin.cpp
  src/lpc40/interrupt.cpp
  src/lpc40/output_pin.cpp
  src/lpc40/pin.cpp
  src/lpc40/power.cpp
  src/lpc40/pwm.cpp
  src/lpc40/spi.cpp
  src/lpc40/dma_spi.cpp
  src/lpc40/stream_dac.cpp
  src/lpc40/uart.cpp

  # stm32_generic
  src/stm32_generic/spi.cpp

  # stm32f1
  src/stm32f1/can_reg.hpp
  src/stm32f1/can.cpp
  src/stm32f1/clock.cpp
  src/stm32f1/dma.hpp
  src/stm32f1/flash_reg.hpp
  src/stm32f1/input_pin.cpp
  src/stm32f1/interrupt.cpp
  src/stm32f1/output_pin.cpp
  src/stm32f1/pin.cpp
  src/stm32f1/pin.hpp
  src/stm32f1/power.cpp
  src/stm32f1/power.hpp
  src/stm32f1/rcc_reg.hpp
  src/stm32f1/uart_reg.hpp
  src/stm32f1/uart.cpp
  src/stm32f1/spi.cpp

  # stm32f411
  src/stm32f411/input_pin.cpp
  src/stm32f411/output_pin.cpp
  src/stm32f411/pin.cpp
  src/stm32f411/power.cpp
  src/stm32f411/spi.cpp
  src/stm32f411/clock.cpp

  # stm32f40
  src/stm32f40/output_pin.cpp

  TEST_SOURCES
  # cortex_m
  tests/dwt_counter.test.cpp
  tests/interrupt.test.cpp
  tests/main.test.cpp
  tests/systick_timer.test.cpp

  # lpc40
  tests/lpc40/adc.test.cpp
  tests/lpc40/can.test.cpp
  tests/lpc40/i2c.test.cpp
  tests/lpc40/input_pin.test.cpp
  tests/lpc40/interrupt_pin.test.cpp
  tests/lpc40/output_pin.test.cpp
  tests/lpc40/pwm.test.cpp
  tests/lpc40/spi.test.cpp
  tests/lpc40/stream_dac.test.cpp
  tests/lpc40/uart.test.cpp

  # stm32f1
  tests/stm32f1/can.test.cpp
  tests/stm32f1/output_pin.test.cpp
  tests/stm32f1/uart.test.cpp
  tests/stm32f1/spi.test.cpp

  # stm32f411
  tests/stm32f411/output_pin.test.cpp
  tests/stm32f411/spi.test.cpp

  PACKAGES
  libhal
  libhal-util
  ring-span-lite
  scope-lite

  LINK_LIBRARIES
  libhal::libhal
  libhal::util
  nonstd::ring-span-lite
  nonstd::scope-lite
)
