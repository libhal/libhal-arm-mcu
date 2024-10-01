// Copyright 2024 Khalil Estell
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

#include <libhal-lpc40/i2c.hpp>

#include <libhal-armcortex/interrupt.hpp>
#include <libhal-lpc40/clock.hpp>
#include <libhal-lpc40/constants.hpp>
#include <libhal-lpc40/interrupt.hpp>
#include <libhal-lpc40/power.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/i2c.hpp>
#include <libhal-util/static_callable.hpp>

#include "i2c_reg.hpp"

namespace hal::lpc40 {
i2c_reg_t* get_i2c_reg(peripheral p_id)
{
  switch (p_id) {
    case peripheral::i2c0:
      return i2c_reg0;
    case peripheral::i2c1:
      return i2c_reg1;
    case peripheral::i2c2:
    default:
      return i2c_reg2;
  }
}

void disable(i2c::bus_info& p_info)
{
  auto* reg = get_i2c_reg(p_info.peripheral_id);

  // Disable i2c interface
  reg->control_clear = i2c_control::interface_enable;

  // Enable interrupt service routine.
  cortex_m::disable_interrupt(p_info.irq_number);
}

void i2c::interrupt()
{
  auto* reg = get_i2c_reg(m_bus.peripheral_id);

  auto state = i2c_host_state(reg->stat);
  auto& data = reg->dat;
  uint32_t clear_mask = 0;
  uint32_t set_mask = 0;
  bool transaction_finished = false;

  switch (state) {
    case i2c_host_state::bus_error: {
      m_status = error_state::io_error;
      set_mask = i2c_control::assert_acknowledge | i2c_control::stop;
      break;
    }
    case i2c_host_state::start_condition:
    case i2c_host_state::repeated_start: {
      if (m_write_iterator != m_write_end) {
        data = to_8_bit_address(m_address, i2c_operation::write);
      } else {
        data = to_8_bit_address(m_address, i2c_operation::read);
      }
      break;
    }
    case i2c_host_state::peripheral_address_write_sent_received_ack: {
      clear_mask = i2c_control::start;
      if (m_write_iterator == m_write_end) {
        transaction_finished = true;
        set_mask = i2c_control::stop;
      } else {
        data = *m_write_iterator++;
      }
      break;
    }
    case i2c_host_state::peripheral_address_write_sent_received_nack: {
      clear_mask = i2c_control::start;
      transaction_finished = true;
      m_status = error_state::no_such_device;
      set_mask = i2c_control::stop;
      break;
    }
    case i2c_host_state::transmitted_data_received_ack: {
      if (m_write_iterator == m_write_end) {
        if (m_read_iterator != m_read_end) {
          set_mask = i2c_control::start;
        } else {
          transaction_finished = true;
          set_mask = i2c_control::stop;
        }
      } else {
        data = *(m_write_iterator++);
      }
      break;
    }
    case i2c_host_state::transmitted_data_received_nack: {
      transaction_finished = true;
      set_mask = i2c_control::stop;
      break;
    }
    case i2c_host_state::arbitration_lost: {
      set_mask = i2c_control::start;
      break;
    }
    case i2c_host_state::peripheral_address_read_sent_received_ack: {
      clear_mask = i2c_control::start;
      if (m_read_iterator == m_read_end) {
        set_mask = i2c_control::stop;
      }
      // If we only want 1 byte, make sure to nack that byte
      else if (m_read_iterator + 1 == m_read_end) {
        clear_mask |= i2c_control::assert_acknowledge;
      }
      // If we want more then 1 byte, make sure to ack the first byte
      else {
        set_mask = i2c_control::assert_acknowledge;
      }
      break;
    }
    case i2c_host_state::peripheral_address_read_sent_received_nack: {
      clear_mask = i2c_control::start;
      m_status = error_state::no_such_device;
      transaction_finished = true;
      set_mask = i2c_control::stop;
      break;
    }
    case i2c_host_state::received_data_transmitted_ack: {
      if (m_read_iterator != m_read_end) {
        *m_read_iterator++ = static_cast<hal::byte>(data);
      }
      // Check if the buffer has been exhausted
      if (m_read_iterator + 1 == m_read_end) {
        // Next state will be `received_data_transmitted_nack`
        clear_mask = i2c_control::assert_acknowledge;
      } else {
        set_mask = i2c_control::assert_acknowledge;
      }
      break;
    }
    case i2c_host_state::received_data_transmitted_nack: {
      transaction_finished = true;
      if (m_read_iterator != m_read_end) {
        *m_read_iterator++ = static_cast<hal::byte>(data);
      }
      set_mask = i2c_control::stop;
      break;
    }
    case i2c_host_state::do_nothing: {
      break;
    }
    default: {
      clear_mask = i2c_control::stop;
      break;
    }
  }

  clear_mask |= i2c_control::interrupt;

  reg->control_set = set_mask;
  reg->control_clear = clear_mask;

  if (transaction_finished) {
    m_busy = false;
  }
}

i2c::i2c(std::uint8_t p_bus_number,
         i2c::settings const& p_settings,
         hal::io_waiter& p_waiter)
  : m_waiter(&p_waiter)
{
  // UM10562: Chapter 7: LPC408x/407x I/O configuration page 13
  switch (p_bus_number) {
    case 0:
      /// Definition for i2c bus 0 for LPC40xx.
      m_bus = {
        .peripheral_id = peripheral::i2c0,
        .irq_number = irq::i2c0,
        .sda = pin(1, 30),
        .sda_function = 0b100,
        .scl = pin(1, 31),
        .scl_function = 0b100,
      };
      break;
    case 1:
      /// Definition for i2c bus 1 for LPC40xx.
      m_bus = {
        .peripheral_id = peripheral::i2c1,
        .irq_number = irq::i2c1,
        .sda = pin(0, 0),
        .sda_function = 0b011,
        .scl = pin(0, 1),
        .scl_function = 0b011,
      };
      break;
    case 2:
      /// Definition for i2c bus 2 for LPC40xx.
      m_bus = {
        .peripheral_id = peripheral::i2c2,
        .irq_number = irq::i2c2,
        .sda = pin(0, 10),
        .sda_function = 0b010,
        .scl = pin(0, 11),
        .scl_function = 0b010,
      };
      break;
    default: {
      safe_throw(hal::operation_not_supported(this));
      break;
    }
  }

  cortex_m::initialize_interrupts<value(irq::max)>();
  i2c::driver_configure(p_settings);
}

i2c::~i2c()
{
  disable(m_bus);
}

i2c::i2c(bus_info const& p_bus,
         i2c::settings const& p_settings,
         hal::io_waiter& p_waiter)
  : m_bus(p_bus)
  , m_waiter(&p_waiter)
{
  initialize_interrupts();
  i2c::driver_configure(p_settings);
}

void i2c::driver_configure(settings const& p_settings)
{
  auto* reg = get_i2c_reg(m_bus.peripheral_id);

  // Setup i2c operating frequency
  auto const input_clock = get_frequency(m_bus.peripheral_id);
  auto const clock_divider = input_clock / p_settings.clock_rate;
  auto const high_side_clocks = clock_divider * m_bus.duty_cycle;
  auto const low_side_clocks = clock_divider - high_side_clocks;

  if (low_side_clocks < 1.0f || high_side_clocks < 1.0f) {
    safe_throw(hal::operation_not_supported(this));
  }

  // Power on peripheral
  power_on(m_bus.peripheral_id);

  // Setup pins for SDA and SCL
  pin(m_bus.sda)
    .function(m_bus.sda_function)
    .resistor(pin_resistor::none)
    .open_drain(true);

  pin(m_bus.scl)
    .function(m_bus.scl_function)
    .resistor(pin_resistor::none)
    .open_drain(true);

  using high_t = std::remove_volatile_t<decltype(reg->duty_cycle_high)>;
  using low_t = std::remove_volatile_t<decltype(reg->duty_cycle_low)>;

  reg->duty_cycle_high = static_cast<high_t>(high_side_clocks);
  reg->duty_cycle_low = static_cast<low_t>(low_side_clocks);

  // Clear all transmission flags
  reg->control_clear = i2c_control::assert_acknowledge | i2c_control::start |
                       i2c_control::stop | i2c_control::interrupt;
  // Enable i2c interface
  reg->control_set = i2c_control::interface_enable;

  setup_interrupt();
}

void i2c::setup_interrupt()
{
  // Create a lambda to call the interrupt() method
  auto isr = [this]() { interrupt(); };

  // A pointer to save the static_callable isr address to.
  cortex_m::interrupt_pointer handler;

  switch (m_bus.irq_number) {
    case irq::i2c0:
      handler = static_callable<i2c, 0, void(void)>(isr).get_handler();
      break;
    case irq::i2c1:
      handler = static_callable<i2c, 1, void(void)>(isr).get_handler();
      break;
    case irq::i2c2:
    default:
      handler = static_callable<i2c, 2, void(void)>(isr).get_handler();
      break;
  }

  // Enable interrupt service routine.
  cortex_m::enable_interrupt(m_bus.irq_number, handler);
}

void i2c::driver_transaction(hal::byte p_address,
                             std::span<const hal::byte> p_data_out,
                             std::span<hal::byte> p_data_in,
                             hal::function_ref<hal::timeout_function> p_timeout)
{
  auto* reg = get_i2c_reg(m_bus.peripheral_id);

  m_status = error_state::no_error;
  m_address = p_address;
  m_write_iterator = p_data_out.begin();
  m_write_end = p_data_out.end();
  m_read_iterator = p_data_in.begin();
  m_read_end = p_data_in.end();
  m_busy = true;

  // Start the transaction
  reg->control_set = i2c_control::start;

  // i2c::interrupt() will set this to false when the transaction has finished.
  while (m_busy) {
    try {
      p_timeout();
      m_waiter->wait();
    } catch (...) {
      // The expected exception is hal::timed_out, but it could be something
      // else. Let rethrow the exception so the caller handle it.
      // A better option here would be to use a std::scope_failure handler.
      reg->control_set = i2c_control::stop;
      throw;
    }
  }

  switch (m_status) {
    case error_state::no_error: {
      break;
    }
    case error_state::no_such_device: {
      safe_throw(hal::no_such_device(m_address, this));
      break;
    }
    case error_state::io_error: {
      safe_throw(hal::io_error(this));
      break;
    }
    case error_state::arbitration_lost: {
      safe_throw(hal::resource_unavailable_try_again(this));
      break;
    }
  };
}
}  // namespace hal::lpc40
