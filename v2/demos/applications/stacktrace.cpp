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

// Throws an exception that nothing catches. The terminate handler prints the
// calls that led to it, which llvm-symbolizer turns into file and line
// numbers:
//
//   llvm-symbolizer --obj=stacktrace.elf --inlines 0x08001a3e 0x080020f2 ...

module;

#include <array>
#include <cstddef>
#include <exception>
#include <span>
#include <string_view>

module arm_mcu_demos;

import hal;
import hal.arm_mcu;
import hal.util;

namespace {
using namespace std::string_view_literals;

void write_hex(hal::serial& p_console, hal::uptr p_value)
{
  constexpr std::string_view digits = "0123456789abcdef";
  constexpr std::size_t nibbles = sizeof(p_value) * 2;
  std::array<char, nibbles + 2> text{ '0', 'x' };
  for (std::size_t index = 0; index < nibbles; index++) {
    auto const shift = ((nibbles - 1) - index) * 4;
    text[index + 2] = digits[(p_value >> shift) & 0xF];
  }
  hal::write(p_console, std::string_view(text.data(), text.size()));
}

void print_stacktrace()
{
  // Taken first, so the printing below doesn't change it
  std::array<hal::uptr, 32> buffer{};
  auto const calls = hal::cortex_m::stacktrace(buffer);

  auto console = resources::console();
  hal::write(*console, "\nstd::terminate() was called from:\n"sv);
  for (auto const address : calls) {
    hal::write(*console, "  "sv);
    write_hex(*console, address);
    hal::write(*console, "\n"sv);
  }

  // Halt. The loop's body stays empty: the compiler may delete a loop that
  // has statements but no side effects, even `continue;`.
  while (true) {
  }
}

// Each level does something after its call, so the compiler can't turn the
// call into a jump, which would leave that level out of the trace.
int volatile levels_returned = 0;

[[gnu::noinline]] void read_sensor()
{
  throw hal::io_error(nullptr);
}

[[gnu::noinline]] void update_controller()
{
  read_sensor();
  levels_returned = levels_returned + 1;
}

[[gnu::noinline]] void run_control_loop()
{
  update_controller();
  levels_returned = levels_returned + 1;
}
}  // namespace

void application()
{
  std::set_terminate(print_stacktrace);
  run_control_loop();
}
