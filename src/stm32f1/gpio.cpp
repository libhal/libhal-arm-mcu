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
#include <bit>
#include <libhal/pointers.hpp>
#include <optional>

#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal-arm-mcu/stm32f1/gpio.hpp>
#include <libhal-arm-mcu/stm32f1/interrupt.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/enum.hpp>
#include <libhal/error.hpp>
#include <libhal/interrupt_pin.hpp>
#include <libhal/units.hpp>

#include "pin.hpp"
#include "power.hpp"

namespace hal::stm32f1 {
namespace {
struct exti_reg_t
{
  /// Offset: 0x00 Interrupt Mask Register (R/W)
  u32 volatile interrupt_mask;
  /// Offset: 0x04 Event Mask Register (R/W)
  u32 volatile event_mask;
  /// Offset: 0x08 Rising Trigger Selection Register (R/W)
  u32 volatile rising_trigger_selection;
  /// Offset: 0x0C Falling Trigger Selection Register (R/W)
  u32 volatile falling_trigger_selection;
  /// Offset: 0x10 Software Interrupt Event Register Register (R/W)
  u32 volatile software_interrupt_event;
  /// Offset: 0x14 Pending Register (RC/W1)
  u32 volatile pending;
};

constexpr std::uintptr_t stm_exti_addr = 0x4001'0400UL;
// NOLINTNEXTLINE(performance-no-int-to-ptr)
inline auto* exti_reg = reinterpret_cast<exti_reg_t*>(stm_exti_addr);

std::array<std::optional<hal::callback<void(bool)>>, 16>
  interrupt_handlers = {};

void external_interrupt_isr()
{
  // Find first interrupt to service
  static constexpr auto pr_mask = hal::bit_mask::from(0, 15);
  auto pending_reg = bit_extract(pr_mask, exti_reg->pending);
  auto const pin_index = std::countr_zero(pending_reg);

  if (pin_index <= 15 && interrupt_handlers[pin_index]) {
    // Determine which port is selected for given pin
    int const afio_exticr_num = pin_index / 4;
    int const afio_exti_bit_offset = (pin_index % 4) * 4;
    auto const port_mask =
      hal::bit_mask::from(afio_exti_bit_offset, afio_exti_bit_offset + 3);
    auto const port =
      bit_extract(port_mask, alternative_function_io->exticr[afio_exticr_num]);

    // Grab input pin value
    auto const& gpio_reg = hal::stm32f1::gpio_reg('A' + port);
    auto const pin_value = bit_extract(bit_mask::from(pin_index), gpio_reg.idr);

    // Call and handle callback
    (*interrupt_handlers[pin_index])(pin_value);
    auto const pending_mask = hal::bit_mask::from(pin_index);
    hal::bit_modify(exti_reg->pending).set(pending_mask);
  }
}

u8 peripheral_to_letter(peripheral p_peripheral)
{
  // The numeric value of `peripheral::gpio_a` to ``peripheral::gpio_g` are
  // contiguous in numeric value thus we can map letters 'A' to 'G' by doing
  // this math here.
  auto const offset = value(p_peripheral) - value(peripheral::gpio_a);
  return 'A' + offset;
}
}  // namespace

gpio_manager::gpio_manager(peripheral p_port)
  : m_port(p_port)
{
  if (not is_on(m_port)) {
    power_on(m_port);
  }
}

gpio_manager::input gpio_manager::acquire_input_pin(
  u8 p_pin,
  input_pin::settings const& p_settings)
{
  return { m_port, p_pin, p_settings };
}

gpio_manager::output gpio_manager::acquire_output_pin(
  u8 p_pin,
  output_pin::settings const& p_settings)
{
  return { m_port, p_pin, p_settings };
}

gpio_manager::interrupt gpio_manager::acquire_interrupt_pin(
  u8 p_pin,
  interrupt_pin::settings const& p_settings)
{
  return { m_port, p_pin, p_settings };
}

gpio_manager::input::input(peripheral p_port,
                           u8 p_pin,
                           input_pin::settings const& p_settings)
  : m_pin({ .port = peripheral_to_letter(p_port), .pin = p_pin })
{
  reset_pin(m_pin);
  gpio_manager::input::driver_configure(p_settings);
}

void gpio_manager::input::driver_configure(settings const& p_settings)
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

bool gpio_manager::input::driver_level()
{
  auto const& reg = gpio_reg(m_pin.port);
  auto const pin_value = bit_extract(bit_mask::from(m_pin.pin), reg.idr);
  return static_cast<bool>(pin_value);
}

gpio_manager::output::output(peripheral p_port,
                             u8 p_pin,
                             output_pin::settings const& p_settings)
  : m_pin({ .port = peripheral_to_letter(p_port), .pin = p_pin })
{
  throw_if_pin_is_unavailable(m_pin);
  gpio_manager::output::driver_configure(p_settings);
}

void gpio_manager::output::driver_configure(settings const& p_settings)
{
  reset_pin(m_pin);
  if (p_settings.open_drain) {
    configure_pin(m_pin, open_drain_gpio_output);
  } else {
    configure_pin(m_pin, push_pull_gpio_output);
  }
  // NOTE: The `resistor` field is ignored in this function
}

void gpio_manager::output::driver_level(bool p_high)
{
  if (p_high) {
    // The first 16 bits of the register set the output state
    gpio_reg(m_pin.port).bsrr = 1 << m_pin.pin;
  } else {
    // The last 16 bits of the register reset the output state
    gpio_reg(m_pin.port).bsrr = 1 << (16 + m_pin.pin);
  }
}

bool gpio_manager::output::driver_level()
{
  auto const& reg = gpio_reg(m_pin.port);
  auto const pin_value = bit_extract(bit_mask::from(m_pin.pin), reg.idr);
  return static_cast<bool>(pin_value);
}

gpio_manager::interrupt::interrupt(peripheral p_port,
                                   u8 p_pin,
                                   interrupt_pin::settings const& p_settings)
  : m_pin({ .port = peripheral_to_letter(p_port), .pin = p_pin })
{
  throw_if_pin_is_unavailable(m_pin);
  setup_interrupt();
  driver_configure(p_settings);
}

void gpio_manager::interrupt::setup_interrupt()
{
  // Verify EXTI line is not already in use
  auto const exti_mask = hal::bit_mask::from(m_pin.pin);
  auto const interrupt_enabled =
    bit_extract(exti_mask, exti_reg->interrupt_mask);
  auto const event_enabled = bit_extract(exti_mask, exti_reg->event_mask);
  if (interrupt_enabled || event_enabled) {
    hal::safe_throw(hal::device_or_resource_busy(this));
  }

  // Determine location of port selection for given pin and set the port
  int const afio_exticr_num = m_pin.pin / 4;
  int const afio_exti_bit_offset = (m_pin.pin % 4) * 4;
  auto const port_mask =
    hal::bit_mask::from(afio_exti_bit_offset, afio_exti_bit_offset + 3);
  hal::bit_modify(alternative_function_io->exticr[afio_exticr_num])
    .insert(port_mask, static_cast<u8>(m_pin.port - 'A'));

  initialize_interrupts();
  // Some EXTI lines share the same IRQ, this skips enabling if this is the
  // case.
  auto const irq = get_irq();
  if (not hal::cortex_m::is_interrupt_enabled(irq)) {
    cortex_m::enable_interrupt(irq, external_interrupt_isr);
  }
}

void gpio_manager::interrupt::driver_configure(settings const& p_settings)
{
  auto const exti_mask = hal::bit_mask::from(m_pin.pin);
  hal::bit_modify(exti_reg->interrupt_mask).clear(exti_mask);

  reset_pin(m_pin);
  if (p_settings.resistor == pin_resistor::pull_up) {
    configure_pin(m_pin, input_pull_up);
  } else if (p_settings.resistor == pin_resistor::pull_down) {
    configure_pin(m_pin, input_pull_down);
  } else {
    configure_pin(m_pin, input_float);
  }

  if (p_settings.trigger == trigger_edge::rising) {
    hal::bit_modify(exti_reg->rising_trigger_selection).set(exti_mask);
    hal::bit_modify(exti_reg->falling_trigger_selection).clear(exti_mask);
  } else if (p_settings.trigger == trigger_edge::falling) {
    hal::bit_modify(exti_reg->rising_trigger_selection).clear(exti_mask);
    hal::bit_modify(exti_reg->falling_trigger_selection).set(exti_mask);
  } else {
    hal::bit_modify(exti_reg->rising_trigger_selection).set(exti_mask);
    hal::bit_modify(exti_reg->falling_trigger_selection).set(exti_mask);
  }

  hal::bit_modify(exti_reg->interrupt_mask).set(exti_mask);
}

void gpio_manager::interrupt::driver_on_trigger(
  hal::callback<interrupt_pin::handler> p_callback)
{
  interrupt_handlers[m_pin.pin] = p_callback;
}

cortex_m::irq_t gpio_manager::interrupt::get_irq()
{
  switch (m_pin.pin) {
    case 0:
      return static_cast<cortex_m::irq_t>(irq::exti0);
    case 1:
      return static_cast<cortex_m::irq_t>(irq::exti1);
    case 2:
      return static_cast<cortex_m::irq_t>(irq::exti2);
    case 3:
      return static_cast<cortex_m::irq_t>(irq::exti3);
    case 4:
      return static_cast<cortex_m::irq_t>(irq::exti4);
    case 5:
      [[fallthrough]];
    case 6:
      [[fallthrough]];
    case 7:
      [[fallthrough]];
    case 8:
      [[fallthrough]];
    case 9:
      return static_cast<cortex_m::irq_t>(irq::exti9_5);
    case 10:
      [[fallthrough]];
    case 11:
      [[fallthrough]];
    case 12:
      [[fallthrough]];
    case 13:
      [[fallthrough]];
    case 14:
      [[fallthrough]];
    case 15:
      [[fallthrough]];
    default:
      return static_cast<cortex_m::irq_t>(irq::exti15_10);
  }
}

hal::v5::strong_ptr<hal::input_pin> acquire_input_pin(
  std::pmr::polymorphic_allocator<> p_allocator,
  hal::v5::strong_ptr<gpio_manager> const& p_manager,
  u8 p_pin,
  input_pin::settings const& p_settings)
{
  return hal::v5::make_strong_ptr<gpio_manager::input>(
    p_allocator, p_manager->acquire_input_pin(p_pin, p_settings));
}

hal::v5::strong_ptr<hal::output_pin> acquire_output_pin(
  std::pmr::polymorphic_allocator<> p_allocator,
  hal::v5::strong_ptr<gpio_manager> const& p_manager,
  u8 p_pin,
  output_pin::settings const& p_settings)
{
  return hal::v5::make_strong_ptr<gpio_manager::output>(
    p_allocator, p_manager->acquire_output_pin(p_pin, p_settings));
}
}  // namespace hal::stm32f1
