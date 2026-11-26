// Copyright 2024 - 2025 Khalil Estell and the libhal contributors
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

#include <chrono>
#include <cstddef>
#include <exception>
#include <new>

import arm_mcu_demos;
import hal;
import hal.util;

namespace {
/**
 * @brief Blink the status LED forever: two short blinks, then a pause
 *
 * Busy-waits on the steady clock, since interrupts may not work anymore.
 */
[[noreturn]] void terminate_handler() noexcept
{
  auto led = resources::status_led();
  auto clock = resources::clock();
  auto const wait = [&clock](hal::time_duration p_duration) {
    auto const deadline = hal::future_deadline(*clock, p_duration);
    while (clock->uptime() < deadline) {
    }
  };

  while (true) {
    using namespace std::chrono_literals;
    led->level(true);
    wait(100ms);
    led->level(false);
    wait(100ms);
    led->level(true);
    wait(100ms);
    led->level(false);
    wait(1s);
  }
}
}  // namespace

int main()
{
  // Set first, so that a failure while initializing the platform blinks too
  std::set_terminate(terminate_handler);
  initialize_platform();
  application();
  std::terminate();
}

void* operator new(std::size_t)
{
  std::terminate();
}

void* operator new[](std::size_t)
{
  std::terminate();
}

void* operator new(unsigned int, std::align_val_t)
{
  std::terminate();
}

void operator delete(void*) noexcept
{
}

void operator delete[](void*) noexcept
{
}

void operator delete(void*, std::size_t) noexcept
{
}

void operator delete[](void*, std::size_t) noexcept
{
}

void operator delete[](void*, std::align_val_t) noexcept
{
}

void operator delete(void*, std::align_val_t) noexcept
{
}
