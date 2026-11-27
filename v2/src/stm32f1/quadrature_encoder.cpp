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
#include <cmath>

module hal.arm_mcu.stm32f1;

import hal;
import hal.util;
import hal.arm_mcu.stm32_generic;

import :constants;
import :pin;
import :power;
import :timer;
import :quadrature_encoder;

namespace hal::stm32f1 {
namespace {
/// @throws hal::argument_out_of_domain - if it isn't positive and finite
float counts_per_revolution(float p_pulses_per_revolution)
{
  if (not std::isfinite(p_pulses_per_revolution) or
      (p_pulses_per_revolution <= 0.0f)) {
    throw hal::argument_out_of_domain(nullptr);
  }
  // Encoder mode 3 counts both edges of both outputs
  return p_pulses_per_revolution * 4.0f;
}

/**
 * @brief The CH1 and CH2 pins, checked free
 *
 * Runs before the timer is claimed, so nothing needs undoing.
 *
 * @throws hal::operation_not_supported - see `timer_pins()`
 * @throws hal::device_or_resource_busy - if either pin is claimed
 */
std::array<pin, 2> encoder_pins(peripheral p_id, timer_remap p_remap)
{
  auto const set = timer_pins(p_id, p_remap);
  std::array<pin, 2> const pins{ set.channels[0], set.channels[1] };
  if (is_pin_claimed(pins[0]) or is_pin_claimed(pins[1])) {
    throw hal::device_or_resource_busy(nullptr);
  }
  return pins;
}
}  // namespace

quadrature_encoder::quadrature_encoder(
  peripheral p_id,
  float p_pulses_per_revolution,
  quadrature_encoder_settings const& p_settings)
  : m_counts_per_revolution(counts_per_revolution(p_pulses_per_revolution))
  , m_pins(encoder_pins(p_id, p_settings.remap))
  // The last call that can throw, so nothing needs undoing after it
  , m_index(claim_timer(p_id))
  , m_counter(timer_address(m_index))
{
  // AFIO is shared, so it is powered on, never claimed.
  power_on(peripheral::afio);
  apply_timer_remap(p_id, timer_pins(p_id, p_settings.remap).remap_value);
  for (auto const& input : m_pins) {
    claim_pin(input);
    configure_pin(input, input_pull_up);
  }
  route_timer_interrupt(m_index, &handle_update, this);
  m_counter.start();
}

quadrature_encoder::~quadrature_encoder()
{
  m_counter.stop();
  for (auto const& input : m_pins) {
    reset_pin(input);
    release_pin(input);
  }
  release_timer(m_index);
}

void quadrature_encoder::handle_update(void* p_self) noexcept
{
  static_cast<quadrature_encoder*>(p_self)->m_counter.update();
}

i64 quadrature_encoder::count()
{
  return m_counter.count();
}

hal::revolutions quadrature_encoder::driver_read()
{
  auto const turns =
    static_cast<float>(m_counter.count()) / m_counts_per_revolution;
  return turns * mp_units::angular::revolution;
}
}  // namespace hal::stm32f1
