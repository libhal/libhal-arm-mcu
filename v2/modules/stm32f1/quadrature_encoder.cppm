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

export module hal.arm_mcu.stm32f1:quadrature_encoder;

import hal;
import hal.util;
import hal.arm_mcu.stm32_generic;

import :constants;
import :pin;
import :timer;

namespace hal::stm32f1 {
/// Settings for a `quadrature_encoder`
export struct quadrature_encoder_settings
{
  /// Which pins the A and B inputs use: CH1 and CH2 of the timer with this
  /// remap, see `timer_remap`
  timer_remap remap = timer_remap::none;
};

/**
 * @brief One STM32F1 timer as a quadrature encoder `hal::rotation_sensor`
 *
 * A relative, infinite rotation sensor: it reads 0 at construction and counts
 * turns in both directions with no limit. The encoder's A output goes to the
 * timer's CH1 pin and B to CH2. A leading B counts up; swap the wires, or
 * negate the reading, to count the other way.
 *
 * The timer counts every edge of both inputs, four counts per pulse of one
 * output, and its update interrupt counts each wrap of the 16-bit counter,
 * so the count is 64 bits and `read()` never needs to be called on a
 * schedule. The interrupt comes once every 65,536 counts: 15 times a second
 * at 1 million counts a second. Both inputs have a 40 kOhm pull-up, for open
 * collector encoders, and a filter: an edge must hold for 8 timer clock
 * cycles, 111 ns at 72 MHz, to count.
 *
 * Claims the whole timer: a timer that counts an encoder can't also run PWM
 * or count delays, since those use the same counter. TIM1 to TIM5 and TIM8
 * have encoder inputs; TIM6 and TIM7 don't. TIM5 and TIM8 exist only on
 * high-density and larger parts, such as the STM32F103ZG. On other parts,
 * constructing them is undefined behavior.
 */
export class quadrature_encoder final : public hal::rotation_sensor
{
public:
  /**
   * @brief Claim a timer and its CH1 and CH2 pins, and start counting from 0
   *
   * @param p_id - the timer: TIM1 to TIM5 or TIM8
   * @param p_pulses_per_revolution - pulses of one encoder output per turn
   * of what `read()` measures: the encoder's PPR or line count, times any
   * gear ratio after it. Need not be a whole number.
   * @param p_settings - the inputs' pins
   * @throws hal::argument_out_of_domain - if `p_pulses_per_revolution` isn't
   * a positive, finite number
   * @throws hal::operation_not_supported - if the timer has no encoder inputs
   * or no such remap
   * @throws hal::device_or_resource_busy - if another driver has claimed the
   * timer or either pin
   */
  quadrature_encoder(peripheral p_id,
                     float p_pulses_per_revolution,
                     quadrature_encoder_settings const& p_settings = {});

  quadrature_encoder(quadrature_encoder const&) = delete;
  quadrature_encoder& operator=(quadrature_encoder const&) = delete;
  quadrature_encoder(quadrature_encoder&&) = delete;
  quadrature_encoder& operator=(quadrature_encoder&&) = delete;

  /// Stop counting, release the pins, power the timer off and release it
  ~quadrature_encoder();

  /**
   * @brief Counts since construction: four per pulse, signed
   *
   * The exact count that `read()` converts to revolutions. A `float` holds
   * whole counts only up to 16,777,216, so for long distances, such as
   * odometry, use this instead.
   *
   * @return i64 - counts since construction, negative if it turned backward
   */
  [[nodiscard]] i64 count();

private:
  static void handle_update(void* p_self) noexcept;

  hal::revolutions driver_read() override;

  float m_counts_per_revolution;
  std::array<pin, 2> m_pins;
  u8 m_index;
  hal::stm32_generic::quadrature_counter m_counter;
};
}  // namespace hal::stm32f1
