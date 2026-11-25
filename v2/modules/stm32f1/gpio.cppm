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
/**
 * @brief STM32F1 GPIO pin that calls a callback when its level changes
 *
 * Uses the external interrupt controller (EXTI). EXTI line n serves pin n of
 * one port, so only one port's pin n can be an edge-triggered interrupt at a
 * time: PA0 and PB0 can't both be.
 *
 * The pin is configured as an input. The callback gets the pin's level as it
 * reads when the interrupt runs, which may already differ from the level
 * right after the edge if the signal bounces.
 *
 * Construct one per pin, wherever it should live: on the stack, as a static,
 * or on the heap with `hal::make_owner<...>(...)`.
 *
 * ```cpp
 * // The button connects the pin to ground when pressed, so the pull-up holds
 * // it high until then: a press is a falling edge.
 * hal::stm32f1::edge_triggered_interrupt button(
 *   { .port = 'B', .pin = 4 },
 *   hal::pin_resistor::pull_up,
 *   { .trigger = hal::edge_trigger::falling });
 * auto subscription = button.on_trigger(my_callback);
 * ```
 */
export class edge_triggered_interrupt final
  : public hal::edge_triggered_interrupt
{
public:
  /**
   * @brief Configure a pin as an input that triggers an interrupt on an edge
   *
   * The interrupt stays masked until a callback is registered with
   * `on_trigger()`, so edges before that are lost.
   *
   * @param p_pin - port & pin selection
   * @param p_resistor - pull resistor for the pin
   * @param p_settings - which edges trigger the callback
   * @throws hal::device_or_resource_busy - if another driver has claimed the
   * pin (see `claim_pin()`), or another port's pin with the same number is
   * already an edge-triggered interrupt
   * @throws hal::argument_out_of_domain - if the port isn't 'A' to 'G' or the
   * pin isn't 0 to 15
   */
  explicit edge_triggered_interrupt(
    pin p_pin,
    hal::pin_resistor p_resistor = hal::pin_resistor::pull_up,
    settings const& p_settings = {});

  edge_triggered_interrupt(edge_triggered_interrupt const&) = delete;
  edge_triggered_interrupt& operator=(edge_triggered_interrupt const&) = delete;
  edge_triggered_interrupt(edge_triggered_interrupt&&) = delete;
  edge_triggered_interrupt& operator=(edge_triggered_interrupt&&) = delete;

  /// Mask the interrupt, return the pin to its reset state and release its
  /// claims
  // The interface's destructor is protected and not virtual, and this class
  // has the same name, so clang-tidy reads this as hiding it. It doesn't.
  // NOLINTNEXTLINE(bugprone-derived-method-shadowing-base-method)
  ~edge_triggered_interrupt();

private:
  void driver_configure(settings const& p_settings) override;
  void driver_on_trigger(hal::edge_triggered_callback& p_callback) override;
  void driver_clear_trigger() noexcept override;

  pin m_pin;
};
}  // namespace hal::stm32f1
