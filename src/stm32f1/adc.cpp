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

#include <array>
#include <mutex>

#include <libhal-arm-mcu/stm32f1/adc.hpp>
#include <libhal-arm-mcu/stm32f1/clock.hpp>
#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/bit_limits.hpp>
#include <libhal-util/enum.hpp>
#include <libhal/error.hpp>
#include <libhal/lock.hpp>
#include <libhal/units.hpp>

#include "pin.hpp"
#include "power.hpp"

namespace hal::stm32f1 {

namespace {

/// adc register map
struct adc_reg_t
{
  /// Number of injected channels
  static constexpr size_t injected_channel_length = 4;
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
  /// Offset: 0x24 A/D Watchdog High Treshold Register (R/W)
  hal::u32 volatile watchdog_high_threshold;
  /// Offset: 0x28 A/D Watchdog Low Treshold Register (R/W)
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

/// Namespace containing the bit_mask objects that are use to manipulate the
/// stm32f1 ADC Status register.
namespace adc_status_register {
/// This bit is set by hardware when the converted voltage crosses the values
/// programmed in the ADC_LTR and ADC_HTR registers. It is cleared by software.
[[maybe_unused]] static constexpr auto analog_watchdog_flag =
  hal::bit_mask::from(0);

/// This bit is set by hardware at the end of a group channel conversion
/// (regular or injected). It is cleared by software or by reading the ADC_DR.
static constexpr auto end_of_conversion = hal::bit_mask::from(1);

/// This bit is set by hardware at the end of all injected group channel
/// conversion. It is cleared by software.
[[maybe_unused]] static constexpr auto injected_channel_end_of_conversion =
  hal::bit_mask::from(2);

/// This bit is set by hardware when injected channel conversion starts. It is
/// cleared by software.
[[maybe_unused]] static constexpr auto injected_channel_start_flag =
  hal::bit_mask::from(3);

/// This bit is set by hardware when regular channel conversion starts. It is
/// cleared by software.
[[maybe_unused]] static constexpr auto regular_channel_start_flag =
  hal::bit_mask::from(4);
};  // namespace adc_status_register

/// Namespace containing the bit_mask objects that are use to manipulate the
/// stm32f1 ADC Control register 2.
namespace adc_control_register_2 {
/// This bit is set and cleared by software. If this bit holds a value of zero
/// and a 1 is written to it then it wakes up the ADC from Power Down state.
/// Conversion starts when this bit holds a value of 1 and a 1 is written to it.
/// The application should allow a delay of tSTAB between power up and start of
/// conversion. Refer to Figure 23.
/// 0: Disable ADC conversion/calibration and go to power down mode.
/// 1: Enable ADC and to start conversion
/// Note: If any other bit in this register apart from ADON is changed at the
/// same time, then conversion is not triggered. This is to prevent triggering
/// an erroneous conversion.
static constexpr auto ad_converter_on = hal::bit_mask::from(0);

/// This bit is set and cleared by software. If set conversion takes place
/// continuously till this bit is reset.
[[maybe_unused]] static constexpr auto continuous_conversion =
  hal::bit_mask::from(1);

/// This bit is set by software to start the calibration. It is reset by
/// hardware after calibration is complete.
static constexpr auto ad_calibration = hal::bit_mask::from(2);

/// This bit is set by software and cleared by hardware, and is used to reset
/// the ADC calibration. It is cleared after the calibration registers are
/// initialized.
[[maybe_unused]] static constexpr auto reset_calibration =
  hal::bit_mask::from(3);

/// This bit is set and cleared by software to enable or disable DMA mode. If
/// its 0 then its disabled, and if its 1 then its enabled.
[[maybe_unused]] static constexpr auto direct_memory_access_mode =
  hal::bit_mask::from(8);

/// This bit is set and cleared by software to determine which data alignment to
/// use. If its 0 then its right-aligned, if its 1 then its left-aligned.
[[maybe_unused]] static constexpr auto data_alignment = hal::bit_mask::from(11);

/// These bits select the external event used to trigger the start of conversion
/// of an injected group.
[[maybe_unused]] static constexpr auto external_event_select_injected_group =
  hal::bit_mask::from(12, 14);

/// This bit is set and cleared by software to enable/disable the external
/// trigger used to start conversion of an injected channel group.
[[maybe_unused]] static constexpr auto
  external_trigger_conversion_mode_injected_channels = hal::bit_mask::from(15);

/// These bits select the external event used to trigger the start of conversion
/// of a regular group.
[[maybe_unused]] static constexpr auto external_event_select_regular_group =
  hal::bit_mask::from(17, 19);

/// This bit is set and cleared by software to enable/disable the external
/// trigger used to start conversion of a regular channel group.
[[maybe_unused]] static constexpr auto
  external_trigger_conversion_mode_regular_channel = hal::bit_mask::from(20);

/// This bit is set by software and cleared by software or by hardware as soon
/// as the conversion starts. It starts a conversion of a group of injected
/// channels (if JSWSTART is selected as trigger event by the JEXTSEL[2:0] bits.
[[maybe_unused]] static constexpr auto start_conversion_injected_channels =
  hal::bit_mask::from(21);

/// This bit is set by software to start conversion and cleared by hardware as
/// soon as conversion starts. It starts a conversion of a group of regular
/// channels if SWSTART is selected as trigger event by the EXTSEL[2:0] bits.
[[maybe_unused]] static constexpr auto start_conversion_regular_channels =
  hal::bit_mask::from(22);

/// This bit is set and cleared by software to enable/disable the temperature
/// sensor and VREFINT channel. In devices with dual ADCs this bit is present
/// only in ADC1.
[[maybe_unused]] static constexpr auto
  temperature_sensor_and_reference_voltage_enable = hal::bit_mask::from(23);
};  // namespace adc_control_register_2

/// Namespace containing the bit_mask objects that are use to manipulate the
/// stm32f1 ADC Regular Sequence register 3.
namespace adc_regular_sequence_register_3 {
/// First channel conversion in regular sequence.
static constexpr auto first_conversion = hal::bit_mask::from(0, 4);

/// Second channel conversion in regular sequence.
[[maybe_unused]] static constexpr auto second_conversion =
  hal::bit_mask::from(5, 9);

/// Third channel conversion in regular sequence.
[[maybe_unused]] static constexpr auto third_conversion =
  hal::bit_mask::from(10, 14);

/// Fourth channel conversion in regular sequence.
[[maybe_unused]] static constexpr auto fourth_conversion =
  hal::bit_mask::from(15, 19);

/// Fifth channel conversion in regular sequence.
[[maybe_unused]] static constexpr auto fifth_conversion =
  hal::bit_mask::from(20, 24);

/// Sixth channel conversion in regular sequence.
[[maybe_unused]] static constexpr auto sixth_conversion =
  hal::bit_mask::from(25, 29);
};  // namespace adc_regular_sequence_register_3

/// Namespace containing the bit_mask objects that are use to manipulate the
/// stm32f1 ADC Regular Data register.
namespace adc_regular_data_register {
/// These bits are read only. They contain the conversion result from the
/// regular channels. The data is left or right-aligned depending on bit 11 in
/// ADC_CR2.
static constexpr auto regular_data = hal::bit_mask::from(0, 15);

/// In ADC1: In dual mode, these bits contain the regular data of ADC2. Refer to
/// Section 11.9: Dual ADC mode.
/// In ADC2 and ADC3: these bits are not used.
[[maybe_unused]] static constexpr auto dual_mode_data =
  hal::bit_mask::from(16, 31);
};  // namespace adc_regular_data_register

adc_reg_t& to_reg(void* p_address)
{
  return *reinterpret_cast<adc_reg_t*>(p_address);
}

void setup_adc(hal::stm32f1::peripheral p_adc_peripheral, void* p_address)
{
  auto& adc_reg = to_reg(p_address);

  // Verify adc's clock is not higher than the maximum frequency.
  auto const adc_frequency = frequency(p_adc_peripheral);
  if (adc_frequency > 14.0_MHz) {
    hal::safe_throw(hal::operation_not_supported(nullptr));
  }

  // Power on adc clock.
  power_on(p_adc_peripheral);

  // Turns on and calibrates the adc only if its the first time power-on. This
  // is to prevent accidentally toggling the start of a new conversion as it
  // uses the same bit.
  if (bit_extract<adc_control_register_2::ad_converter_on>(adc_reg.control_2) ==
      0) {
    // Power on the adc.
    hal::bit_modify(adc_reg.control_2)
      .set<adc_control_register_2::ad_converter_on>();

    // Start adc calibration. ADC must have been in power-on state for a minimum
    // of 2 clock cycles before starting calibration.
    hal::bit_modify(adc_reg.control_2)
      .set<adc_control_register_2::ad_calibration>();

    // Wait for calibration to complete.
    while (bit_extract<adc_control_register_2::ad_calibration>(
             adc_reg.control_2) == 1) {
    }
  }
}

void setup_pin(adc_peripheral_manager::pins const& p_pin)
{
  // Derive port and pin from the enum.
  hal::u8 port, pin;
  if (hal::value(p_pin) <= 7) {
    port = 'A';
    pin = hal::value(p_pin);
  } else if (hal::value(p_pin) <= 9) {
    port = 'B';
    pin = hal::value(p_pin) - 8;
  } else {
    port = 'C';
    pin = hal::value(p_pin) - 10;
  }

  // Set specified pin to analog input mode.
  configure_pin({ .port = port, .pin = pin }, input_analog);
}
}  // namespace

adc_peripheral_manager::adc_peripheral_manager(adc_selection p_adc_selection,
                                               hal::basic_lock& p_lock)
  : m_lock(&p_lock)
  , adc_reg_location(nullptr)
{
  // Base address for apb2 bus.
  constexpr std::uintptr_t stm_apb2_base = 0x40000000UL;
  // Determines the appropriate adc peripheral to use and its memory offset.
  std::uintptr_t adc_offset;
  hal::stm32f1::peripheral adc_peripheral;
  switch (p_adc_selection) {
    case adc_selection::adc1:
      adc_offset = 0x12400;
      adc_peripheral = peripheral::adc1;
      break;
    case adc_selection::adc2:
      adc_offset = 0x12800;
      adc_peripheral = peripheral::adc2;
      break;
    default:
      adc_offset = 0x12400;
      adc_peripheral = peripheral::adc1;
      break;
  }
  // Stores address of specified adc peripheral's config registers to the
  // manager object.
  // NOLINTNEXTLINE(performance-no-int-to-ptr)
  adc_reg_location = reinterpret_cast<void*>(stm_apb2_base + adc_offset);

  setup_adc(adc_peripheral, adc_reg_location);
}

adc_peripheral_manager::channel adc_peripheral_manager::acquire_channel(
  pins p_pin)
{
  return { *this, p_pin };
}

float adc_peripheral_manager::read_channel(pins p_pin)
{
  // Lock the lock.
  std::lock_guard<hal::basic_lock> acquire_lock(*m_lock);

  auto& adc_reg = to_reg(adc_reg_location);
  // Set the specified channel to be sampled.
  hal::bit_modify(adc_reg.regular_sequence_3)
    .insert<adc_regular_sequence_register_3::first_conversion>(
      hal::value(p_pin));

  // Start adc conversion.
  hal::bit_modify(adc_reg.control_2)
    .set<adc_control_register_2::ad_converter_on>();

  // Wait for conversion to complete.
  while (
    not bit_extract<adc_status_register::end_of_conversion>(adc_reg.status)) {
  }

  auto constexpr full_scale_max = bit_limits<12, size_t>::max();
  auto constexpr full_scale_float = static_cast<float>(full_scale_max);
  // Read sample from peripheral's memory.
  auto const sample_integer =
    hal::bit_extract<adc_regular_data_register::regular_data>(
      adc_reg.regular_data);
  auto const sample = static_cast<float>(sample_integer);
  return sample / full_scale_float;
}

adc_peripheral_manager::channel::channel(adc_peripheral_manager& p_manager,
                                         adc_peripheral_manager::pins p_pin)
  : m_manager(&p_manager)
  , m_pin(p_pin)
{
  setup_pin(m_pin);
}

float adc_peripheral_manager::channel::driver_read()
{
  return m_manager->read_channel(m_pin);
}

}  // namespace hal::stm32f1
