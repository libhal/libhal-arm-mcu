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
#include <mutex>

module hal.arm_mcu.stm32f1;

import hal;
import hal.util;

import :constants;
import :pin;
import :power;
import :clock;

namespace hal::stm32f1 {
namespace {
/// ADC register map
struct adc_reg_t
{
  /// Number of injected channels
  static constexpr std::size_t injected_channel_length = 4;
  /// Offset: 0x00 A/D Status Register (RC/W0)
  hal::u32 volatile status;
  /// Offset: 0x04 A/D Control Register 1 (R/W)
  hal::u32 volatile control_1;
  /// Offset: 0x08 A/D Control Register 2 (R/W)
  hal::u32 volatile control_2;
  /// Offset: 0x0C A/D Sample Time Register 1 (R/W)
  hal::u32 volatile sample_time_1;
  /// Offset: 0x10 A/D Sample Time Register 2 (R/W)
  hal::u32 volatile sample_time_2;
  /// Offset: 0x14-0x20 A/D Injected Channel 0..3 Data Offset Register (R/W)
  std::array<hal::u32 volatile, injected_channel_length>
    injected_channel_data_offset;
  /// Offset: 0x24 A/D Watchdog High Threshold Register (R/W)
  hal::u32 volatile watchdog_high_threshold;
  /// Offset: 0x28 A/D Watchdog Low Threshold Register (R/W)
  hal::u32 volatile watchdog_low_threshold;
  /// Offset: 0x2C A/D Regular Sequence Register 1 (R/W)
  hal::u32 volatile regular_sequence_1;
  /// Offset: 0x30 A/D Regular Sequence Register 2 (R/W)
  hal::u32 volatile regular_sequence_2;
  /// Offset: 0x34 A/D Regular Sequence Register 3 (R/W)
  hal::u32 volatile regular_sequence_3;
  /// Offset: 0x38 A/D Injected Sequence Register (R/W)
  hal::u32 volatile injected_sequence;
  /// Offset: 0x3C-0x48 A/D Injected Data Register 0..3 (R/ )
  std::array<hal::u32 volatile, injected_channel_length> injected_data;
  /// Offset: 0x4C A/D Regular Data Register (R/ )
  hal::u32 volatile regular_data;
};

/// Bit masks for the ADC Status register
namespace adc_status_register {
/// This bit is set by hardware at the end of a group channel conversion
/// (regular or injected). It is cleared by software or by reading the
/// ADC_DR.
constexpr auto end_of_conversion = hal::bit_mask::from(1);
}  // namespace adc_status_register

/// Bit masks for the ADC Control register 2
namespace adc_control_register_2 {
/// This bit is set and cleared by software. If this bit holds a value of
/// zero and a 1 is written to it then it wakes up the ADC from Power Down
/// state. Conversion starts when this bit holds a value of 1 and a 1 is
/// written to it.
/// 0: Disable ADC conversion/calibration and go to power down mode.
/// 1: Enable ADC and to start conversion
constexpr auto ad_converter_on = hal::bit_mask::from(0);

/// This bit is set by software to start the calibration. It is reset by
/// hardware after calibration is complete.
constexpr auto ad_calibration = hal::bit_mask::from(2);
}  // namespace adc_control_register_2

/// Bit masks for the ADC Regular Sequence register 3
namespace adc_regular_sequence_register_3 {
/// First channel conversion in regular sequence.
constexpr auto first_conversion = hal::bit_mask::from(0, 4);
}  // namespace adc_regular_sequence_register_3

/// Bit masks for the ADC Regular Data register
namespace adc_regular_data_register {
/// These bits are read only. They contain the conversion result from the
/// regular channels.
constexpr auto regular_data = hal::bit_mask::from(0, 15);
}  // namespace adc_regular_data_register

/// Maximum rated clock frequency for the stm32f1 ADC peripheral
constexpr auto max_adc_frequency = 14 * mp_units::si::unit_symbols::MHz;

/// @throws hal::operation_not_supported - if `p_id` is not `adc1` or
/// `adc2`.
uptr peripheral_to_register(peripheral p_id)
{
  switch (p_id) {
    case peripheral::adc1:
      return 0x4001'2400;
    case peripheral::adc2:
      return 0x4001'2800;
    default:
      throw hal::operation_not_supported(nullptr);
  }
}

adc_reg_t& registers_of(peripheral p_id)
{
  // NOLINTNEXTLINE(performance-no-int-to-ptr)
  return *reinterpret_cast<adc_reg_t*>(peripheral_to_register(p_id));
}

pin to_pin(adc_pins p_pin)
{
  auto const value = hal::value(p_pin);
  if (value <= 7) {
    return { .port = 'A', .pin = value };
  }
  if (value <= 9) {
    return { .port = 'B', .pin = static_cast<u8>(value - 8) };
  }
  return { .port = 'C', .pin = static_cast<u8>(value - 10) };
}

/// Busy wait until at least `p_ticks` full ticks of `p_clock` have passed
void wait_ticks(steady_clock& p_clock, u64 p_ticks)
{
  // The tick in progress now only partly counts, so wait one more.
  auto const end = p_clock.uptime() + p_ticks + 1;
  while (p_clock.uptime() < end) {
    continue;
  }
}

/// Runs one conversion of `p_pin` on `p_reg`. Doesn't throw.
u16 convert(adc_reg_t& p_reg, adc_pins p_pin)
{
  bit_modify(p_reg.regular_sequence_3)
    .insert<adc_regular_sequence_register_3::first_conversion>(
      hal::value(p_pin));

  // ADON is already set, so setting it again starts a conversion.
  bit_modify(p_reg.control_2).set<adc_control_register_2::ad_converter_on>();

  while (
    not bit_extract<adc_status_register::end_of_conversion>(p_reg.status)) {
    continue;
  }

  // Reading the data register clears the end of conversion flag.
  auto const sample = static_cast<hal::u16>(
    bit_extract<adc_regular_data_register::regular_data>(p_reg.regular_data));

  return upscale<u16, 12>(sample);
}
}  // namespace

adc::adc(peripheral p_id, steady_clock& p_clock, hal::ref<basic_lock> p_lock)
  : m_lock(p_lock)
  , m_id(p_id)
{
  // Rejects unsupported peripherals before anything is claimed.
  auto& reg = registers_of(p_id);
  auto const adc_frequency = frequency(p_id);
  if (adc_frequency > max_adc_frequency) {
    throw hal::operation_not_supported(this);
  }

  claim(p_id);
  power_on(p_id);
  // Start from the reset state, whatever ran before (e.g. a bootloader) left.
  // That also means ADON is clear, so setting it below powers the ADC up
  // instead of starting a conversion.
  reset_peripheral(p_id);

  bit_modify(reg.control_2).set<adc_control_register_2::ad_converter_on>();

  // The ADC must be powered up for tSTAB (1 us, DS5319) before it converts,
  // and for at least two ADC clock cycles before calibration (RM0008 section
  // 11.4). Wait for both, each rounded up to whole clock ticks.
  auto const clock_hz =
    p_clock.frequency().numerical_value_in(hal::hertz::unit);
  auto const adc_hz = adc_frequency.numerical_value_in(hal::hertz::unit);
  auto const tstab_ticks = (clock_hz / 1'000'000) + 1;
  auto const two_adc_cycles_ticks = 2 * ((clock_hz / adc_hz) + 1);
  wait_ticks(p_clock, tstab_ticks + two_adc_cycles_ticks);

  bit_modify(reg.control_2).set<adc_control_register_2::ad_calibration>();
  while (bit_extract<adc_control_register_2::ad_calibration>(reg.control_2)) {
    continue;
  }
}

adc::~adc()
{
  power_off(m_id);
  release(m_id);
}

adc::channel adc::acquire_channel(adc_pins p_pin)
{
  return { *this, p_pin };
}

u16 adc::read(adc_pins p_pin)
{
  // Waits while another context is converting. Not std::scoped_lock, which
  // libc++ built without threads doesn't provide.
  std::lock_guard const hold(*m_lock);

  // A lock that blocks keeps other contexts out, so this is only true if a
  // lock that doesn't block, such as hal::no_op_lock, let this read interrupt
  // another one. Converting would switch the channel under that read.
  if (m_converting.test_and_set(std::memory_order_acquire)) {
    throw hal::device_or_resource_busy(this);
  }
  auto const sample = convert(registers_of(m_id), p_pin);
  m_converting.clear(std::memory_order_release);
  return sample;
}

adc::channel::channel(adc& p_manager, adc_pins p_pin)
  : m_manager(p_manager)
  , m_pin(p_pin)
{
  auto const selected = to_pin(p_pin);
  claim_pin(selected);
  configure_pin(selected, input_analog);
}

adc::channel::~channel()
{
  auto const selected = to_pin(m_pin);
  reset_pin(selected);
  release_pin(selected);
}

u16 adc::channel::driver_read()
{
  return m_manager->read(m_pin);
}
}  // namespace hal::stm32f1
