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
#include <atomic>

module hal.arm_mcu.stm32f1;

import hal;
import hal.util;
import hal.arm_mcu.stm32_generic;

import :constants;
import :pin;
import :power;
import :clock;

namespace hal::stm32f1 {
namespace {
/// SCL and SDA pins of a supported I2C peripheral
struct i2c_pins
{
  pin scl;
  pin sda;
};

/// @throws hal::operation_not_supported - if `p_id` is not `i2c1` or `i2c2`.
uptr peripheral_to_register(peripheral p_id)
{
  // See Chapter 3.3 "Memory map" in RM0008 for these magic numbers
  switch (p_id) {
    case peripheral::i2c1:
      return 0x4000'5400;
    case peripheral::i2c2:
      return 0x4000'5800;
    default:
      throw hal::operation_not_supported(nullptr);
  }
}

/// @throws hal::operation_not_supported - if `p_id` is not `i2c1` or `i2c2`.
i2c_pins pins_for(peripheral p_id)
{
  // See the pin definitions table in the STM32F103 datasheets (DS5319,
  // DS5792). I2C1 uses its default pins; its PB8/PB9 remap isn't supported.
  switch (p_id) {
    case peripheral::i2c1:
      return { .scl = { .port = 'B', .pin = 6 },
               .sda = { .port = 'B', .pin = 7 } };
    case peripheral::i2c2:
      return { .scl = { .port = 'B', .pin = 10 },
               .sda = { .port = 'B', .pin = 11 } };
    default:
      throw hal::operation_not_supported(nullptr);
  }
}

void* register_address(peripheral p_id)
{
  // NOLINTNEXTLINE(performance-no-int-to-ptr)
  return reinterpret_cast<void*>(peripheral_to_register(p_id));
}

/// Marks the bus busy for its lifetime, and clears the mark even if the
/// operation throws
class busy_scope
{
public:
  /// @throws hal::device_or_resource_busy - if another context or an
  /// interrupt is using the bus. hal::thread_safe_i2c makes callers wait
  /// instead.
  busy_scope(std::atomic_flag& p_busy, void const* p_instance)
    : m_busy(p_busy)
  {
    if (m_busy.test_and_set(std::memory_order_acquire)) {
      throw hal::device_or_resource_busy(p_instance);
    }
  }

  busy_scope(busy_scope const&) = delete;
  busy_scope& operator=(busy_scope const&) = delete;
  busy_scope(busy_scope&&) = delete;
  busy_scope& operator=(busy_scope&&) = delete;

  ~busy_scope()
  {
    m_busy.clear(std::memory_order_release);
  }

private:
  std::atomic_flag& m_busy;
};

/**
 * @brief Claim the peripheral and its pins, power it on and configure it
 *
 * If configuring throws, undoes all of it.
 *
 * @throws hal::device_or_resource_busy - if another driver has claimed the
 * peripheral or one of its pins.
 * @throws hal::operation_not_supported - if `p_settings` can't be achieved.
 */
void start_peripheral(peripheral p_id,
                      hal::stm32_generic::i2c& p_driver,
                      hal::i2c_settings const& p_settings,
                      void const* p_instance)
{
  auto const pins = pins_for(p_id);

  // Claim every pin or none: check them all before claiming any.
  for (auto const& selected : std::array{ pins.scl, pins.sda }) {
    if (is_pin_claimed(selected)) {
      throw hal::device_or_resource_busy(p_instance);
    }
  }
  claim(p_id);
  claim_pin(pins.scl);
  claim_pin(pins.sda);

  power_on(p_id);
  // Start from the reset state, whatever ran before (e.g. a bootloader) left.
  reset_peripheral(p_id);

  if (p_id == peripheral::i2c1) {
    // AFIO is shared, so it is powered on, never claimed.
    power_on(peripheral::afio);
    bit_modify(alternative_function_io->mapr).clear<pin_remap::i2c1_remap>();
  }

  configure_pin(pins.scl, open_drain_alternative_output);
  configure_pin(pins.sda, open_drain_alternative_output);

  try {
    p_driver.configure(p_settings, frequency(p_id));
  } catch (...) {
    for (auto const& selected : std::array{ pins.scl, pins.sda }) {
      reset_pin(selected);
      release_pin(selected);
    }
    reset_peripheral(p_id);
    power_off(p_id);
    release(p_id);
    throw;
  }
}

/// Reset and release the pins, then disable, power off and release the
/// peripheral
void stop_peripheral(peripheral p_id)
{
  auto const pins = pins_for(p_id);
  for (auto const& selected : std::array{ pins.scl, pins.sda }) {
    reset_pin(selected);
    release_pin(selected);
  }
  // Disables the peripheral. The generic driver doesn't touch the registers
  // when it is destroyed after this.
  reset_peripheral(p_id);
  power_off(p_id);
  release(p_id);
}
}  // namespace

polled_i2c::polled_i2c(peripheral p_id,
                       hal::ref<hal::steady_clock> p_clock,
                       hal::i2c_settings const& p_settings)
  : m_clock(p_clock)
  , m_id(p_id)
  , m_driver(register_address(p_id))  // Rejects unsupported peripherals
{
  start_peripheral(m_id, m_driver, p_settings, this);
}

polled_i2c::~polled_i2c()
{
  stop_peripheral(m_id);
}

void polled_i2c::configure(hal::i2c_settings const& p_settings)
{
  // Changing the clock rate mid-transaction would corrupt it.
  busy_scope const busy(m_busy, this);
  m_driver.configure(p_settings, frequency(m_id));
}

void polled_i2c::driver_transaction(
  hal::byte p_address,
  mem::scatter_span<hal::byte const> p_data_out,
  mem::scatter_span<hal::byte> p_data_in)
{
  busy_scope const busy(m_busy, this);
  m_driver.transaction(p_address, p_data_out, p_data_in, *m_clock);
}

}  // namespace hal::stm32f1
