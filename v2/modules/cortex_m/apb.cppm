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

export module hal.arm_mcu.cortex_m:apb;

import hal;
import hal.util;

namespace hal::cortex_m {
/**
 * @brief The fewest cycles of its clock that an APB transfer takes
 *
 * Every transfer on an AMBA APB bus has a setup phase and an access phase, of
 * at least one cycle of the bus clock each (AMBA APB Protocol Specification,
 * Arm IHI 0024). Wait states and the bridge from the system bus only add
 * cycles. So reading a register of a peripheral on APB takes at least this
 * many cycles.
 */
export constexpr hal::u32 apb_cycles_per_transfer = 2;

/**
 * @brief The fewest polls of a peripheral on APB that take at least a
 * duration
 *
 * See `hal::polls_for()`. Each poll must read at least one register of the
 * peripheral. Peripherals on AHB can be read in fewer cycles, so this count
 * is too low for them.
 *
 * @param p_apb_clock - the clock of the APB bus the peripheral is on, which
 * isn't always the peripheral's own clock
 * @param p_duration - how long the polls must take at least
 * @return hal::u32 - the number of polls, as `hal::polls_for()` returns it
 */
export [[nodiscard]] constexpr hal::u32 apb_polls_for(
  hal::hertz p_apb_clock,
  hal::time_duration p_duration)
{
  return hal::polls_for(p_apb_clock, p_duration, apb_cycles_per_transfer);
}
}  // namespace hal::cortex_m
