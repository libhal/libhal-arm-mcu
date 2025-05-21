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

#pragma once

#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal/units.hpp>

namespace hal::stm32f1 {
/**
 * @brief Power on the peripheral
 *
 * This API also acts as a resource overlap detector. If this API is called
 * twice on the same peripheral, it will throw an exception. Only drivers with
 * control over the entire peripheral should call this API for their respective
 * peripheral. This allows this API to detect when two driver attempt to utilize
 * the same resource.
 *
 * @throws hal::device_or_resource_busy - if the peripheral is already powered
 * on, constituting a violation of the 1 peripheral manager per peripheral rule.
 * @throws hal::argument_out_of_domain - if the peripheral's value is outside of
 * the bounds of the enum class OR if there is on enable register for that
 * peripheral.
 */
void power_on(peripheral p_peripheral);

/**
 * @brief Power off peripheral
 *
 * If the peripheral is already powered off, this does nothing.
 */
void power_off(peripheral p_peripheral);

/**
 * @brief Check if the peripheral is powered on
 *
 * @return true - peripheral is on
 * @return false - peripheral is off
 */
[[nodiscard]] bool is_on(peripheral p_peripheral);

/**
 * @brief Resets the peripheral
 *
 * This will reset all the peripheral's registers to their reset/default values.
 *
 */
void reset_peripheral(peripheral p_peripheral);
}  // namespace hal::stm32f1
