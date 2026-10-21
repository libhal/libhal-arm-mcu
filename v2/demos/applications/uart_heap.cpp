// Copyright 2026 Khalil Estell and the libhal contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

module;

#include <array>
#include <chrono>
#include <cstddef>
#include <memory_resource>
#include <string_view>

module arm_mcu_demos;

import hal;
import hal.util;
import hal.arm_mcu.stm32f1;

// Shows a USART serial port on the heap. The serial port can't be copied or
// moved, so hal::make_owner_from() constructs it directly in the allocation.
// The manager and the receive buffer must outlive the serial port, so they
// stay static here.
void application()
{
  using namespace std::chrono_literals;
  using namespace std::string_view_literals;

  // main.cpp makes operator new terminate, so allocate from a fixed arena.
  static std::array<std::byte, 256> arena{};
  static std::pmr::monotonic_buffer_resource resource(
    arena.data(), arena.size(), std::pmr::null_memory_resource());

  static hal::stm32f1::usart usart1(hal::stm32f1::peripheral::usart1);
  static std::array<hal::byte, 128> receive_buffer{};

  auto console = hal::make_owner_from<hal::serial>(
    &resource, [&] { return usart1.acquire_serial(receive_buffer); });

  while (true) {
    hal::write(*console, "Hello from the heap!\n"sv);
    hal::sleep_for(1s);
  }
}
