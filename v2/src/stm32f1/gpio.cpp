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

module hal.arm_mcu.stm32f1;

import hal;
import hal.util;

import :pin;

namespace hal::stm32f1 {
namespace {
pin_config_t output_config(hal::pin_settings const& p_settings,
                           output_speed p_speed)
{
  return {
    .CNF1 = static_cast<u8>(p_settings.open_drain ? 1 : 0),
    .CNF0 = 0,
    .MODE = static_cast<u8>(p_speed),
    .PxODR = 0,
  };
}

class input final : public hal::input_pin
{
public:
  input(pin p_pin, pin_settings const& p_settings)
    : m_pin(p_pin)
  {
    reset_pin(m_pin);
    configure(p_settings);
  }

  input(input const&) = delete;
  input& operator=(input const&) = delete;
  input(input&&) noexcept = default;
  input& operator=(input&&) noexcept = default;
  ~input() override = default;

private:
  void configure(pin_settings const& p_settings)
  {
    reset_pin(m_pin);

    if (p_settings.resistor == pin_resistor::pull_up) {
      configure_pin(m_pin, input_pull_up);
    } else if (p_settings.resistor == pin_resistor::pull_down) {
      configure_pin(m_pin, input_pull_down);
    } else {
      configure_pin(m_pin, input_float);
    }
  }

  async::future<void> driver_configure(async::context&,
                                       pin_settings const& p_settings) override
  {
    configure(p_settings);
    return {};
  }

  async::future<bool> driver_level(async::context&) override
  {
    auto const& reg = gpio_reg(m_pin.port);
    auto const pin_value = bit_extract(bit_mask::from(m_pin.pin), reg.idr);
    return static_cast<bool>(pin_value);
  }

  pin m_pin;
};

class output final : public hal::output_pin
{
public:
  output(pin p_pin, pin_settings const& p_settings, output_speed p_speed)
    : m_pin(p_pin)
    , m_speed(p_speed)
  {
    throw_if_pin_is_unavailable(m_pin);
    configure(p_settings);
  }

  output(output const&) = delete;
  output& operator=(output const&) = delete;
  output(output&&) noexcept = default;
  output& operator=(output&&) noexcept = default;
  ~output() override = default;

private:
  void configure(pin_settings const& p_settings)
  {
    reset_pin(m_pin);
    configure_pin(m_pin, output_config(p_settings, m_speed));
  }

  async::future<void> driver_configure(async::context&,
                                       pin_settings const& p_settings) override
  {
    configure(p_settings);
    return {};
  }

  async::future<void> driver_level(async::context&, bool p_high) override
  {
    if (p_high) {
      // The first 16 bits of the register set the output state
      gpio_reg(m_pin.port).bsrr = 1U << m_pin.pin;
    } else {
      // The last 16 bits of the register reset the output state
      gpio_reg(m_pin.port).bsrr = 1U << (16 + m_pin.pin);
    }
    return {};
  }

  async::future<bool> driver_level(async::context&) override
  {
    auto const& reg = gpio_reg(m_pin.port);
    auto const pin_value = bit_extract(bit_mask::from(m_pin.pin), reg.idr);
    return static_cast<bool>(pin_value);
  }

  pin m_pin;
  output_speed m_speed;
};
}  // namespace

hal::ptr<hal::input_pin> input_pin::create(hal::allocator p_allocator,
                                           pin p_pin,
                                           pin_settings const& p_settings)
{
  return hal::allocate<input>(p_allocator, p_pin, p_settings);
}

hal::ptr<hal::output_pin> output_pin::create(hal::allocator p_allocator,
                                             pin p_pin,
                                             pin_settings const& p_settings,
                                             output_speed p_speed)
{
  return hal::allocate<output>(p_allocator, p_pin, p_settings, p_speed);
}
}  // namespace hal::stm32f1
