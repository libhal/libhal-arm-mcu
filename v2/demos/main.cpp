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

#include <cstdio>

#include <chrono>
#include <exception>
#include <new>

import hal;
import hal.util;
import hal.arm_mcu.cortex_m;
import async_context;
import arm_mcu_demos;

async::inplace_context<64> coroutine_stack{};

struct resumer : public hal::timed_callback
{
  void callback() override
  {
    coroutine_stack.unblock();
    fired = true;
  }

  bool volatile fired = false;
};

resumer s_resumer{};

int main()
{
  initialize_platform();
  hal::ptr<resumer> waker(mem::unsafe_assume_static_tag{}, s_resumer);
  auto timer = resources::timer();
  auto future = application(coroutine_stack);
  coroutine_stack.sync_wait([&timer, &waker](hal::time_duration p_sleep_time) {
    s_resumer.fired = false;

    timer->schedule(waker, p_sleep_time, hal::timer_mode::one_shot);

    while (!s_resumer.fired) {
      if (not hal::cortex_m::debugger_connected()) {
        continue;  // loop if the debugger is connected
      }

      // Disable interrupts to sleep safely without losing interrupts.
      hal::cortex_m::disable_all_interrupts();

      if (s_resumer.fired) {
        hal::cortex_m::enable_all_interrupts();
        break;
      }

      hal::cortex_m::wait_for_interrupt();
      hal::cortex_m::enable_all_interrupts();
    }
  });

  std::terminate();
}

// Override global new operator
void* operator new(std::size_t)
{
  std::terminate();
}

// Override global new[] operator
void* operator new[](std::size_t)
{
  std::terminate();
}

void* operator new(unsigned int, std::align_val_t)
{
  std::terminate();
}

// Override global delete operator
void operator delete(void*) noexcept
{
}

// Override global delete[] operator
void operator delete[](void*) noexcept
{
}

// Optional: Override sized delete operators (C++14 and later)
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
