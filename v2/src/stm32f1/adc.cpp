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
#include <coroutine>

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

/// Upscale a 12-bit ADC sample to a full 16-bit value by shifting it to the
/// most significant bits and duplicating its own most significant bits into
/// the remaining least significant bits. This minimizes proportionality
/// distortion versus a plain left-shift; see `hal::adc16::read()`.
constexpr hal::u16 upscale_12_bit_to_16_bit(hal::u16 p_sample)
{
  // TODO(kammce): Remove when libhal-util has the upscale API
  return static_cast<hal::u16>((p_sample << 4) | (p_sample >> 8));
}

class channel final : public hal::adc16
{
public:
  channel(hal::ptr<adc> const& p_manager, adc_pins p_pin)
    : m_manager(p_manager)
    , m_pin(p_pin)
  {
    configure_pin(to_pin(p_pin), input_analog);
  }

  channel(channel const&) = delete;
  channel& operator=(channel const&) = delete;
  channel(channel&&) = delete;
  channel& operator=(channel&&) = delete;

  ~channel() override
  {
    reset_pin(to_pin(m_pin));
  }

private:
  async::future<hal::u16> driver_read(async::context& p_context) override
  {
    return m_manager->read(p_context, m_pin);
  }

  hal::ptr<adc> m_manager;
  adc_pins m_pin;
};
}  // namespace

struct adc::impl
{
  peripheral id;
  void* reg;
  async::mutex conversion_lock;
};

hal::ptr<adc> adc::create(hal::allocator p_allocator, peripheral p_id)
{
  if (frequency(p_id) > max_adc_frequency) {
    throw hal::operation_not_supported(nullptr);
  }

  return hal::allocate<adc>(p_allocator, private_key{}, p_allocator, p_id);
}

adc::adc(private_key, hal::allocator p_allocator, peripheral p_id)
  : pimpl(p_allocator,
          impl{ .id = p_id,
                // NOLINTNEXTLINE(performance-no-int-to-ptr)
                .reg = reinterpret_cast<void*>(peripheral_to_register(p_id)),
                .conversion_lock = {} })
{
  power_on(p_id);

  auto& reg = *static_cast<adc_reg_t*>(inner().reg);

  // Turns on and calibrates the adc only if its the first time power-on.
  // This is to prevent accidentally toggling the start of a new conversion
  // as it uses the same bit.
  if (not bit_extract<adc_control_register_2::ad_converter_on>(reg.control_2)) {
    bit_modify(reg.control_2).set<adc_control_register_2::ad_converter_on>();
    bit_modify(reg.control_2).set<adc_control_register_2::ad_calibration>();

    while (bit_extract<adc_control_register_2::ad_calibration>(reg.control_2)) {
      continue;
    }
  }
}

adc::~adc()
{
  power_off(inner().id);
}

hal::ptr<hal::adc16> adc::acquire_channel(adc_pins p_pin)
{
  return hal::allocate<channel>(memory_resource(), strong_from_this(), p_pin);
}

async::future<hal::u16> adc::read(async::context& p_context, adc_pins p_pin)
{
  auto const guard = co_await inner().conversion_lock.lock(p_context);

  auto& reg = *static_cast<adc_reg_t*>(inner().reg);

  bit_modify(reg.regular_sequence_3)
    .insert<adc_regular_sequence_register_3::first_conversion>(
      hal::value(p_pin));

  bit_modify(reg.control_2).set<adc_control_register_2::ad_converter_on>();

  while (not bit_extract<adc_status_register::end_of_conversion>(reg.status)) {
    continue;
  }

  auto const sample = static_cast<hal::u16>(
    bit_extract<adc_regular_data_register::regular_data>(reg.regular_data));

  co_return upscale_12_bit_to_16_bit(sample);
}
}  // namespace hal::stm32f1
