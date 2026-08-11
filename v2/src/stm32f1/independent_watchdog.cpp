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

#include <cstdint>

#include <chrono>

module hal.arm_mcu.stm32f1;

import hal;
import hal.util;

import :constants;
import :independent_watchdog;

namespace hal::stm32f1 {
namespace {

/// @brief Independent Watchdog register block
struct independent_watchdog_registers
{
  void set_kr(uint16_t p_value)
  {
    constexpr hal::bit_mask writeable_kr = hal::bit_mask::from(0, 15);
    hal::bit_modify(kr).insert<writeable_kr>(p_value);
  }
  void set_pr(uint8_t p_value)
  {
    constexpr hal::bit_mask writeable_pr = hal::bit_mask::from(0, 2);
    hal::bit_modify(pr).insert<writeable_pr>(p_value);
  }
  void set_rlr(uint16_t p_value)
  {
    constexpr hal::bit_mask writeable_rlr = hal::bit_mask::from(0, 11);
    hal::bit_modify(rlr).insert<writeable_rlr>(p_value);
  }

  uint32_t volatile kr;
  uint32_t volatile pr;
  uint32_t volatile rlr;
  uint32_t volatile sr;
};

// NOLINTBEGIN(performance-no-int-to-ptr)
// IWDG register base address (RM0008 Table 4)
inline auto* const iwdg_regs =
  reinterpret_cast<independent_watchdog_registers*>(0x4000'3000);

// Reset status register (RM0008 section 8.3.10)
inline auto* const reset_status_register =
  reinterpret_cast<uint32_t*>(0x4002'1000 + 0x24);
// NOLINTEND(performance-no-int-to-ptr)

}  // namespace

struct independent_watchdog::impl
{
  // Nothing to store — all operations are direct register accesses.
};

hal::ptr<independent_watchdog> independent_watchdog::create(
  hal::allocator p_allocator)
{
  return hal::allocate<independent_watchdog>(
    p_allocator, private_key{}, p_allocator);
}

independent_watchdog::independent_watchdog(private_key,
                                           hal::allocator p_allocator)
  : pimpl(p_allocator, impl{})
{
}

void independent_watchdog::start()
{
  iwdg_regs->set_kr(0xCCCC);
}

void independent_watchdog::reset()
{
  iwdg_regs->set_kr(0xAAAA);
}

void independent_watchdog::set_countdown_time(hal::time_duration p_wait_time)
{
  using namespace std::chrono_literals;

  // Convert to counts of 40 kHz clock (figure 11, RM0008 page 126)
  constexpr auto nano_seconds_per_clock_cycle = 1'000'000'000ns / 40'000;
  long long cycle_count = p_wait_time / nano_seconds_per_clock_cycle;

  // FRQ divider starts at /4 (table 96, RM0008 page 495)
  cycle_count = cycle_count >> 2;

  if (cycle_count == 0) {
    throw hal::operation_not_supported(nullptr);
  }

  hal::byte freq_divider = 0;
  while (cycle_count > 0x1000 && freq_divider <= 7) {
    cycle_count = cycle_count >> 1;
    freq_divider++;
  }

  if (freq_divider >= 7) {
    throw hal::operation_not_supported(nullptr);
  }

  // Registers shouldn't be edited when bits are 1 (section 19.4.4, RM0008)
  if (hal::bit_extract(hal::bit_mask::from(0, 1), iwdg_regs->sr)) {
    throw hal::resource_unavailable_try_again(nullptr);
  }

  iwdg_regs->set_kr(0x5555);
  iwdg_regs->set_pr(freq_divider);
  iwdg_regs->set_rlr(cycle_count - 1);
}

bool independent_watchdog::check_flag()
{
  // Section 8.3.10, RM0008 page 152
  constexpr hal::bit_mask flag = hal::bit_mask::from(29);
  return hal::bit_extract(flag, *reset_status_register);
}

void independent_watchdog::clear_flag()
{
  // Section 8.3.10, RM0008 page 152
  constexpr hal::bit_mask reset_flag = hal::bit_mask::from(24);
  hal::bit_modify(*reset_status_register).set(reset_flag);
}

independent_watchdog::~independent_watchdog() = default;

}  // namespace hal::stm32f1
