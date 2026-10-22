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

export module hal.arm_mcu.stm32f1:gpio;

import hal;
import hal.util;

import :pin;

namespace hal::stm32f1 {

/**
 * @brief Maximum output slew-rate for a push-pull/open-drain output pin
 *
 * Values match the MODE[1:0] bits for output pins (see "Table 21. Output
 * MODE bits" in RM0008).
 */
export enum class output_speed : hal::u8
{
  max_10_mhz = 0b01,
  max_2_mhz = 0b10,
  max_50_mhz = 0b11,
};

/**
 * @brief STM32F1 GPIO input pin
 *
 * Construct one per pin, wherever it should live: on the stack, as a static,
 * or on the heap with `hal::make_owner<hal::stm32f1::input_pin>(...)`.
 *
 * ```cpp
 * // The button connects the pin to ground when pressed, so the pull-up holds
 * // it high until then: pressed reads as low (active low).
 * hal::stm32f1::input_pin button({ .port = 'B', .pin = 4 },
 *                                { .resistor = hal::pin_resistor::pull_up });
 * bool const pressed = not button.level();
 * ```
 */
export class input_pin final : public hal::input_pin
{
public:
  /**
   * @brief Configure a pin as an input
   *
   * @param p_pin - port & pin selection
   * @param p_settings - settings to apply to the input pin
   * @throws hal::device_or_resource_busy - if another driver has claimed the
   * pin (see `claim_pin()`)
   */
  explicit input_pin(pin p_pin, hal::pin_settings const& p_settings = {});

  input_pin(input_pin const&) = delete;
  input_pin& operator=(input_pin const&) = delete;
  input_pin(input_pin&&) = delete;
  input_pin& operator=(input_pin&&) = delete;

  /// Return the pin to its reset state and release its claim
  ~input_pin();

private:
  void configure(hal::pin_settings const& p_settings);
  void driver_configure(hal::pin_settings const& p_settings) override;
  bool driver_level() override;

  pin m_pin;
};

/**
 * @brief STM32F1 GPIO output pin
 *
 * Construct one per pin, wherever it should live: on the stack, as a static,
 * or on the heap with `hal::make_owner<hal::stm32f1::output_pin>(...)`.
 *
 * ```cpp
 * hal::stm32f1::output_pin led({ .port = 'C', .pin = 13 });
 * led.level(true);
 * ```
 */
export class output_pin final : public hal::output_pin
{
public:
  /**
   * @brief Configure a pin as an output
   *
   * @param p_pin - port & pin selection
   * @param p_settings - settings to apply to the output pin
   * @param p_speed - maximum output slew-rate for the pin
   * @throws hal::device_or_resource_busy - if another driver has claimed the
   * pin (see `claim_pin()`)
   */
  explicit output_pin(pin p_pin,
                      hal::pin_settings const& p_settings = {},
                      output_speed p_speed = output_speed::max_50_mhz);

  output_pin(output_pin const&) = delete;
  output_pin& operator=(output_pin const&) = delete;
  output_pin(output_pin&&) = delete;
  output_pin& operator=(output_pin&&) = delete;

  /// Return the pin to its reset state and release its claim
  ~output_pin();

private:
  void configure(hal::pin_settings const& p_settings);
  void driver_configure(hal::pin_settings const& p_settings) override;
  void driver_level(bool p_high) override;
  bool driver_level() override;

  pin m_pin;
  output_speed m_speed;
};
}  // namespace hal::stm32f1
