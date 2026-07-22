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

import :constants;
import :power;
import :pin;

namespace hal::stm32f1 {
namespace {
u8 peripheral_to_letter(peripheral p_peripheral)
{
  // The numeric value of `peripheral::gpio_a` to `peripheral::gpio_g` are
  // contiguous in numeric value thus we can map letters 'A' to 'G' by doing
  // this math here.
  auto const offset = value(p_peripheral) - value(peripheral::gpio_a);
  return static_cast<u8>('A' + offset);
}
}  // namespace

struct gpio_manager::impl
{
  peripheral port;
};

gpio_manager::gpio_manager(private_key,
                           hal::allocator p_allocator,
                           peripheral p_select)
  : pimpl(p_allocator, impl{ .port = p_select })
{
  if (not is_on(inner().port)) {
    power_on(inner().port);
  }
}

#if 0
void gpio_manager::configure_gpio_pin(bool p_input)
{
  if (p_input) {
    reset_pin(m_pin);
    if (p_settings.open_drain) {
      configure_pin(m_pin, open_drain_gpio_output);
    } else {
      configure_pin(m_pin, push_pull_gpio_output);
    }
  } else {
  }
}
#endif

hal::ptr<gpio_manager> gpio_manager::create(hal::allocator p_allocator,
                                            peripheral p_select)
{
  return hal::allocate<gpio_manager>(
    p_allocator, private_key{}, p_allocator, p_select);
}

class input final : public hal::input_pin
{
public:
  input(peripheral p_port, u8 p_pin, input_pin::settings const& p_settings)
    : m_pin({ .port = peripheral_to_letter(p_port), .pin = p_pin })
  {
    reset_pin(m_pin);
    async::context ctx{};
    input::driver_configure(ctx, p_settings);
  }

  input(input const&) = delete;
  input& operator=(input const&) = delete;
  input(input&&) noexcept = default;
  input& operator=(input&&) noexcept = default;
  ~input() override = default;

private:
  async::future<void> driver_configure(async::context&,
                                       settings const& p_settings) override
  {
    reset_pin(m_pin);

    if (p_settings.resistor == pin_resistor::pull_up) {
      configure_pin(m_pin, input_pull_up);
    } else if (p_settings.resistor == pin_resistor::pull_down) {
      configure_pin(m_pin, input_pull_down);
    } else {
      configure_pin(m_pin, input_float);
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
};

class output final : public hal::output_pin
{
public:
  output(peripheral p_port, u8 p_pin, settings const& p_settings)
    : m_pin({ .port = peripheral_to_letter(p_port), .pin = p_pin })
  {
    throw_if_pin_is_unavailable(m_pin);
    async::context ctx{};
    output::driver_configure(ctx, p_settings);
  }

  output(output const&) = delete;
  output& operator=(output const&) = delete;
  output(output&&) noexcept = default;
  output& operator=(output&&) noexcept = default;
  ~output() override = default;

private:
  friend class gpio_manager;

  async::future<void> driver_configure(
    async::context&,
    [[maybe_unused]] settings const& p_settings) override
  {
#if 0
    reset_pin(m_pin);
    if (p_settings.resistor == pin_resistor::pull_up) {
      configure_pin(m_pin, input_pull_up);
    } else if (p_settings.resistor == pin_resistor::pull_down) {
      configure_pin(m_pin, input_pull_down);
    } else {
      configure_pin(m_pin, input_float);
    }
#endif
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
};

hal::ptr<hal::input_pin> gpio_manager::acquire_input_pin(
  hal::allocator p_allocator,
  u8 p_pin,
  input_pin::settings const& p_settings)
{
  return hal::allocate<input>(p_allocator, m_peripheral, p_pin, p_settings);
}

hal::ptr<hal::output_pin> gpio_manager::acquire_output_pin(
  hal::allocator p_allocator,
  u8 p_pin,
  output_pin::settings const& p_settings)
{
  return hal::allocate<output>(p_allocator, m_peripheral, p_pin, p_settings);
}
}  // namespace hal::stm32f1
