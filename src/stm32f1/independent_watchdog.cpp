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

#include <cstdint>
#include <libhal-arm-mcu/stm32f1/independent_watchdog.hpp>
#include <libhal-util/bit.hpp>
#include <libhal/error.hpp>

using namespace std::chrono_literals;

namespace {
struct independent_watchdog_registers
{
  // setters are to not acidentily write in reserved memory

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

auto* const iwdg_regs =
  reinterpret_cast<independent_watchdog_registers*>(0x40003000);
// NOLINTBEGIN(performance-no-int-to-ptr)
uint32_t* const reset_status_register =
  reinterpret_cast<uint32_t*>(0x40021000 + 0x24);
// NOLINTEND(performance-no-int-to-ptr)

}  // namespace

namespace hal::stm32f1 {
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
  // convert to counts of 40khz clock (figure 11, pg. 126)
  constexpr hal::time_duration nano_seconds_per_clock_cycle =
    1'000'000'000ns / 40'000;
  long long cycle_count = p_wait_time / nano_seconds_per_clock_cycle;
  // frq divider starts a /4 (table 96, pg. 495)
  cycle_count = cycle_count >> 2;
  if (cycle_count == 0) {
    throw hal::operation_not_supported(nullptr);
  }

  hal::byte frq_divider = 0;
  while (cycle_count > 0x1000 && frq_divider <= 7) {
    cycle_count = cycle_count >> 1;
    frq_divider++;
  }
  if (frq_divider >= 7) {
    throw hal::operation_not_supported(nullptr);
  } else {
    // register's shouldn't be edited when bits are 1 (sec. 19.4.4, pg. 498)
    if (bit_extract(bit_mask::from(0, 1), iwdg_regs->sr)) {
      throw hal::resource_unavailable_try_again(nullptr);
    }
    iwdg_regs->set_kr(0x5555);
    iwdg_regs->set_pr(frq_divider);
    iwdg_regs->set_rlr(cycle_count - 1);
  }
}

bool independent_watchdog::check_flag()
{
  // sec. 8.3.10, pg. 152
  bit_mask const flag = bit_mask::from(29);
  return bit_extract(flag, *reset_status_register);
}

void independent_watchdog::clear_flag()
{
  // sec. 8.3.10, pg. 152
  bit_mask const reset_flag = bit_mask::from(24);
  bit_modify(*reset_status_register).set(reset_flag);
}

}  // namespace hal::stm32f1
