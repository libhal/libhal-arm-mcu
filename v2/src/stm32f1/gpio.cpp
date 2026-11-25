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
#include <bit>
#include <bitset>
#include <cstdint>

module hal.arm_mcu.stm32f1;

import hal;
import hal.util;
import hal.arm_mcu.cortex_m;

import :claim;
import :constants;
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

bool read_level(pin p_pin)
{
  auto const& reg = gpio_reg(p_pin.port);
  auto const pin_value = bit_extract(bit_mask::from(p_pin.pin), reg.idr);
  return static_cast<bool>(pin_value);
}

/// External interrupt/event controller (EXTI) register map, RM0008 10.3
struct exti_reg_t
{
  /// Offset: 0x00 Interrupt mask register
  u32 volatile interrupt_mask;
  /// Offset: 0x04 Event mask register
  u32 volatile event_mask;
  /// Offset: 0x08 Rising trigger selection register
  u32 volatile rising_trigger;
  /// Offset: 0x0C Falling trigger selection register
  u32 volatile falling_trigger;
  /// Offset: 0x10 Software interrupt event register
  u32 volatile software_interrupt_event;
  /// Offset: 0x14 Pending register, cleared by writing 1
  u32 volatile pending;
};
static_assert(sizeof(exti_reg_t) == 0x18);

constexpr std::uintptr_t exti_address = 0x4001'0400;
// NOLINTNEXTLINE(performance-no-int-to-ptr)
inline auto* const exti = reinterpret_cast<exti_reg_t*>(exti_address);

/// Lines 0 to 15 serve GPIO pins. Lines 16 and up serve the PVD, RTC alarm
/// and USB wakeup, which have their own drivers.
constexpr std::size_t gpio_line_count = 16;

/// The callback of each line, null while none is registered
constinit std::array<hal::edge_triggered_callback*, gpio_line_count>
  callbacks{};
/// Set while an edge_triggered_interrupt owns the line
constinit std::bitset<gpio_line_count> claimed_lines{};

/// The 4-bit EXTICR field that selects the port of a line
bit_mask port_select_mask(u8 p_line)
{
  return { .position = static_cast<u32>((p_line % 4U) * 4U), .width = 4 };
}

[[gnu::noinline]] void handle_lines(u32 p_lines)
{
  auto pending = exti->pending & p_lines;
  // Clear before the callbacks run, so an edge during a callback sets the
  // pending bit again and runs the interrupt again.
  exti->pending = pending;
  while (pending != 0) {
    auto const line = static_cast<u8>(std::countr_zero(pending));
    pending = pending & (pending - 1U);
    if (auto* const callback = callbacks[line]; callback != nullptr) {
      auto const port = bit_extract(port_select_mask(line),
                                    alternative_function_io->exticr[line / 4U]);
      callback->callback(
        read_level({ .port = static_cast<u8>('A' + port), .pin = line }));
    }
  }
}

template<u32 Lines>
void exti_isr()
{
  handle_lines(Lines);
}

struct line_interrupt
{
  irq request;
  cortex_m::interrupt_pointer isr;
  /// Every line that shares the interrupt
  u32 lines;
};

line_interrupt interrupt_for(u8 p_line)
{
  // The lines that share the EXTI9_5 and EXTI15_10 interrupts
  constexpr u32 lines_5_to_9 = 0b0000'0011'1110'0000;
  constexpr u32 lines_10_to_15 = 0b1111'1100'0000'0000;

  switch (p_line) {
    case 0:
      return { irq::exti0, &exti_isr<1U << 0U>, 1U << 0U };
    case 1:
      return { irq::exti1, &exti_isr<1U << 1U>, 1U << 1U };
    case 2:
      return { irq::exti2, &exti_isr<1U << 2U>, 1U << 2U };
    case 3:
      return { irq::exti3, &exti_isr<1U << 3U>, 1U << 3U };
    case 4:
      return { irq::exti4, &exti_isr<1U << 4U>, 1U << 4U };
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
      return { irq::exti9_5, &exti_isr<lines_5_to_9>, lines_5_to_9 };
    default:
      return { irq::exti15_10, &exti_isr<lines_10_to_15>, lines_10_to_15 };
  }
}
}  // namespace

input_pin::input_pin(pin p_pin, pin_settings const& p_settings)
  : m_pin(p_pin)
{
  claim_pin(m_pin);
  configure(p_settings);
}

input_pin::~input_pin()
{
  reset_pin(m_pin);
  release_pin(m_pin);
}

void input_pin::configure(pin_settings const& p_settings)
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

void input_pin::driver_configure(pin_settings const& p_settings)
{
  configure(p_settings);
}

bool input_pin::driver_level()
{
  return read_level(m_pin);
}

output_pin::output_pin(pin p_pin,
                       pin_settings const& p_settings,
                       output_speed p_speed)
  : m_pin(p_pin)
  , m_speed(p_speed)
{
  claim_pin(m_pin);
  configure(p_settings);
}

output_pin::~output_pin()
{
  reset_pin(m_pin);
  release_pin(m_pin);
}

void output_pin::configure(pin_settings const& p_settings)
{
  reset_pin(m_pin);
  configure_pin(m_pin, output_config(p_settings, m_speed));
}

void output_pin::driver_configure(pin_settings const& p_settings)
{
  configure(p_settings);
}

void output_pin::driver_level(bool p_high)
{
  if (p_high) {
    // The first 16 bits of the register set the output state
    gpio_reg(m_pin.port).bsrr = 1U << m_pin.pin;
  } else {
    // The last 16 bits of the register reset the output state
    gpio_reg(m_pin.port).bsrr = 1U << (16 + m_pin.pin);
  }
}

bool output_pin::driver_level()
{
  return read_level(m_pin);
}

edge_triggered_interrupt::edge_triggered_interrupt(pin p_pin,
                                                   pin_resistor p_resistor,
                                                   settings const& p_settings)
  : m_pin(p_pin)
{
  // Another port's pin can hold the line. Check the line before claiming the
  // pin, so this throws before it has claimed anything. claim_pin() then
  // checks that the pin exists, which keeps the line's index in range.
  throw_if_claimed(claimed_lines, m_pin.pin, this);
  claim_pin(m_pin);
  claimed_lines[m_pin.pin] = true;

  reset_pin(m_pin);
  if (p_resistor == pin_resistor::pull_up) {
    configure_pin(m_pin, input_pull_up);
  } else if (p_resistor == pin_resistor::pull_down) {
    configure_pin(m_pin, input_pull_down);
  } else {
    configure_pin(m_pin, input_float);
  }

  // configure_pin() powered on AFIO, which selects the line's port
  bit_modify(alternative_function_io->exticr[m_pin.pin / 4U])
    .insert(port_select_mask(m_pin.pin), static_cast<u32>(m_pin.port - 'A'));

  driver_configure(p_settings);

  // The line stays masked until a callback is registered
  auto const interrupt = interrupt_for(m_pin.pin);
  cortex_m::enable_interrupt(interrupt.request, interrupt.isr);
}

edge_triggered_interrupt::~edge_triggered_interrupt()
{
  driver_clear_trigger();
  auto const line = bit_mask::from(m_pin.pin);
  bit_modify(exti->rising_trigger).clear(line);
  bit_modify(exti->falling_trigger).clear(line);
  exti->pending = 1U << m_pin.pin;  // Writing 1 clears the pending bit

  claimed_lines.reset(m_pin.pin);
  // A shared interrupt stays on while another of its lines is in use
  auto const interrupt = interrupt_for(m_pin.pin);
  if ((claimed_lines.to_ulong() & interrupt.lines) == 0) {
    cortex_m::disable_interrupt(interrupt.request);
  }

  reset_pin(m_pin);
  release_pin(m_pin);
}

void edge_triggered_interrupt::driver_configure(settings const& p_settings)
{
  auto const line = bit_mask::from(m_pin.pin);
  // edge_trigger::both is neither falling nor rising, so it enables both
  // edges. Written as two "not" checks, this is 8 B smaller than checking for
  // each of rising, falling and both.
  auto const rising = (p_settings.trigger != edge_trigger::falling);
  auto const falling = (p_settings.trigger != edge_trigger::rising);
  bit_modify(exti->rising_trigger).insert(line, static_cast<u32>(rising));
  bit_modify(exti->falling_trigger).insert(line, static_cast<u32>(falling));
}

void edge_triggered_interrupt::driver_on_trigger(
  hal::edge_triggered_callback& p_callback)
{
  if (callbacks[m_pin.pin] != nullptr) {
    throw hal::subscription_busy(this);
  }
  callbacks[m_pin.pin] = &p_callback;

  // Edges before the callback was registered are lost
  auto const line = bit_mask::from(m_pin.pin);
  exti->pending = 1U << m_pin.pin;  // Writing 1 clears the pending bit
  bit_modify(exti->interrupt_mask).set(line);
}

void edge_triggered_interrupt::driver_clear_trigger() noexcept
{
  bit_modify(exti->interrupt_mask).clear(bit_mask::from(m_pin.pin));
  callbacks[m_pin.pin] = nullptr;
}
}  // namespace hal::stm32f1
