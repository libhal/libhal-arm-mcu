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

#include <libhal-armcortex/interrupt.hpp>
#include <libhal-lpc40/constants.hpp>
#include <libhal-lpc40/interrupt.hpp>
#include <libhal-lpc40/interrupt_pin.hpp>
#include <libhal-lpc40/pin.hpp>
#include <libhal-util/enum.hpp>

#include "gpio_reg.hpp"
#include "interrupt_pin_reg.hpp"

namespace hal::lpc40 {

void interrupt_pin_handler()
{
  std::uint32_t triggered_port = interrupt_pin_reg->status >> 2;
  std::uint32_t triggered_pin = 0;
  bit_mask triggered_pin_mask;
  std::uint32_t status = 0;

  if (triggered_port == 0) {
    // To keep the number of handler functions to a minimum, this library does
    // not support separate handlers for rising and falling edges. Therefore
    // it does not matter if a rising or falling edge triggered this
    // interrupt. OR both status together and clear them together below.
    status = interrupt_pin_reg->raising_status_port0 |
             interrupt_pin_reg->falling_status_port0;
  } else {
    // Same documentation as the port 0 case but with port 2 here.
    status = interrupt_pin_reg->raising_status_port2 |
             interrupt_pin_reg->falling_status_port2;
  }

  // Figure out which bit triggered this interrupt by checking the number of
  // zeros starting from the least significant bit. If there is 5 zeros,
  // then the next bit must be a 1 and thus, the pin that set off this
  // interrupt is pin 5. If there are 0 zeros, then the first bit must be
  // set to a 1 and thus pin 0 is what set off this interrupt. And so on for
  // all other bits.
  triggered_pin = static_cast<std::uint32_t>(std::countr_zero(status));
  triggered_pin_mask = bit_mask::from(triggered_pin);

  if (triggered_port == 0) {
    // Clear interrupt flag on port 0. This is important as not doing this
    // will result in this interrupt being repeatedly called.
    bit_modify(interrupt_pin_reg->clear_interrupt_port0)
      .set(triggered_pin_mask);
  } else {
    bit_modify(interrupt_pin_reg->clear_interrupt_port2)
      .set(triggered_pin_mask);
  }

  bool pin_level =
    bit_extract(triggered_pin_mask, gpio_reg[triggered_port]->pin);

  interrupt_pin_handlers[triggered_port][triggered_pin](pin_level);
}

interrupt_pin::interrupt_pin(std::uint8_t p_port,  // NOLINT
                             std::uint8_t p_pin,
                             settings const& p_settings)
  : m_port(p_port)
  , m_pin(p_pin)
{
  initialize_interrupts();
  interrupt_pin::driver_configure(p_settings);
}

interrupt_pin::~interrupt_pin()
{
  if (m_port == 0) {
    bit_modify(interrupt_pin_reg->enable_raising_port0).clear(pin_mask(m_pin));
    bit_modify(interrupt_pin_reg->enable_falling_port0).clear(pin_mask(m_pin));
  } else if (m_port == 2) {
    bit_modify(interrupt_pin_reg->enable_raising_port2).clear(pin_mask(m_pin));
    bit_modify(interrupt_pin_reg->enable_falling_port2).clear(pin_mask(m_pin));
  }
}

void interrupt_pin::driver_configure(settings const& p_settings)
{
  // Set pin as input
  bit_modify(gpio_reg[m_port]->direction).clear(pin_mask(m_pin));

  // Configure pin to use gpio function, use setting resistor and set the rest
  // to false.
  pin(m_port, m_pin)
    .function(0)
    .dac(false)
    .analog(false)
    .open_drain(false)
    .resistor(p_settings.resistor);

  // Enable interrupt for gpio and use interrupt handler as our handler.
  cortex_m::enable_interrupt(irq::gpio, interrupt_pin_handler);

  if (p_settings.trigger == trigger_edge::both ||
      p_settings.trigger == trigger_edge::rising) {
    if (m_port == 0) {
      bit_modify(interrupt_pin_reg->enable_raising_port0).set(pin_mask(m_pin));
    } else if (m_port == 2) {
      bit_modify(interrupt_pin_reg->enable_raising_port2).set(pin_mask(m_pin));
    }
  }

  if (p_settings.trigger == trigger_edge::both ||
      p_settings.trigger == trigger_edge::falling) {
    if (m_port == 0) {
      bit_modify(interrupt_pin_reg->enable_falling_port0).set(pin_mask(m_pin));
    } else if (m_port == 2) {
      bit_modify(interrupt_pin_reg->enable_falling_port2).set(pin_mask(m_pin));
    }
  }
}

void interrupt_pin::driver_on_trigger(hal::callback<handler> p_callback)
{
  if (m_port == 0) {
    interrupt_pin_handlers[0][m_pin] = p_callback;
  } else {
    interrupt_pin_handlers[1][m_pin] = p_callback;
  }
}
}  // namespace hal::lpc40
