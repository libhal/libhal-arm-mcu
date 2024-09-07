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

#pragma once

#include <array>
#include <cstdint>
#include <limits>

#include <libhal-util/math.hpp>

namespace hal::lpc40 {
/// Structure containing the exact clock divider and multiplier tuning values
/// for the uart peripheral
struct uart_baud_t
{
  /// main clock divider
  uint32_t divider;
  /// tuning numerator
  uint32_t numerator;
  /// tuning denominator
  uint32_t denominator;
};

/// Structure holds the fractional tuning information that follows this
/// equation:
///
/// 1 + (numerator / denominator)
///
/// Thus the end value will always be 1.XXX value. The fractional value is used
/// to tune the baud rate down slightly to reach a target baud rate which
/// couldn't be achieved with
struct fractional_divider_t
{
  /// The tuning ratio multiplier to scale up baud rate by
  int ratio;
  /// The numerator value of the ratio must always be smaller than or equal to
  /// the denominator
  uint32_t numerator;
  /// The denominator value of the ratio
  uint32_t denominator;
};

/// Table of all of the possible fractional values allowed for the UART hardware
/// and their ratios
constexpr std::array fractional_table{
  fractional_divider_t{ .ratio = 1000, .numerator = 0, .denominator = 1 },
  fractional_divider_t{ .ratio = 1250, .numerator = 1, .denominator = 4 },
  fractional_divider_t{ .ratio = 1500, .numerator = 1, .denominator = 2 },
  fractional_divider_t{ .ratio = 1750, .numerator = 3, .denominator = 4 },
  fractional_divider_t{ .ratio = 1067, .numerator = 1, .denominator = 15 },
  fractional_divider_t{ .ratio = 1267, .numerator = 4, .denominator = 15 },
  fractional_divider_t{ .ratio = 1533, .numerator = 8, .denominator = 15 },
  fractional_divider_t{ .ratio = 1769, .numerator = 10, .denominator = 13 },
  fractional_divider_t{ .ratio = 1071, .numerator = 1, .denominator = 14 },
  fractional_divider_t{ .ratio = 1273, .numerator = 3, .denominator = 11 },
  fractional_divider_t{ .ratio = 1538, .numerator = 7, .denominator = 13 },
  fractional_divider_t{ .ratio = 1778, .numerator = 7, .denominator = 9 },
  fractional_divider_t{ .ratio = 1077, .numerator = 1, .denominator = 13 },
  fractional_divider_t{ .ratio = 1286, .numerator = 2, .denominator = 7 },
  fractional_divider_t{ .ratio = 1545, .numerator = 6, .denominator = 11 },
  fractional_divider_t{ .ratio = 1786, .numerator = 11, .denominator = 14 },
  fractional_divider_t{ .ratio = 1083, .numerator = 1, .denominator = 12 },
  fractional_divider_t{ .ratio = 1300, .numerator = 3, .denominator = 10 },
  fractional_divider_t{ .ratio = 1556, .numerator = 5, .denominator = 9 },
  fractional_divider_t{ .ratio = 1800, .numerator = 4, .denominator = 5 },
  fractional_divider_t{ .ratio = 1091, .numerator = 1, .denominator = 11 },
  fractional_divider_t{ .ratio = 1308, .numerator = 4, .denominator = 13 },
  fractional_divider_t{ .ratio = 1571, .numerator = 4, .denominator = 7 },
  fractional_divider_t{ .ratio = 1818, .numerator = 9, .denominator = 11 },
  fractional_divider_t{ .ratio = 1100, .numerator = 1, .denominator = 10 },
  fractional_divider_t{ .ratio = 1333, .numerator = 1, .denominator = 3 },
  fractional_divider_t{ .ratio = 1583, .numerator = 7, .denominator = 12 },
  fractional_divider_t{ .ratio = 1833, .numerator = 5, .denominator = 6 },
  fractional_divider_t{ .ratio = 1111, .numerator = 1, .denominator = 9 },
  fractional_divider_t{ .ratio = 1357, .numerator = 5, .denominator = 14 },
  fractional_divider_t{ .ratio = 1600, .numerator = 3, .denominator = 5 },
  fractional_divider_t{ .ratio = 1846, .numerator = 11, .denominator = 13 },
  fractional_divider_t{ .ratio = 1125, .numerator = 1, .denominator = 8 },
  fractional_divider_t{ .ratio = 1364, .numerator = 4, .denominator = 11 },
  fractional_divider_t{ .ratio = 1615, .numerator = 8, .denominator = 13 },
  fractional_divider_t{ .ratio = 1857, .numerator = 6, .denominator = 7 },
  fractional_divider_t{ .ratio = 1133, .numerator = 2, .denominator = 15 },
  fractional_divider_t{ .ratio = 1375, .numerator = 3, .denominator = 8 },
  fractional_divider_t{ .ratio = 1625, .numerator = 5, .denominator = 8 },
  fractional_divider_t{ .ratio = 1867, .numerator = 13, .denominator = 15 },
  fractional_divider_t{ .ratio = 1143, .numerator = 1, .denominator = 7 },
  fractional_divider_t{ .ratio = 1385, .numerator = 5, .denominator = 13 },
  fractional_divider_t{ .ratio = 1636, .numerator = 7, .denominator = 11 },
  fractional_divider_t{ .ratio = 1875, .numerator = 7, .denominator = 8 },
  fractional_divider_t{ .ratio = 1154, .numerator = 2, .denominator = 13 },
  fractional_divider_t{ .ratio = 1400, .numerator = 2, .denominator = 5 },
  fractional_divider_t{ .ratio = 1643, .numerator = 9, .denominator = 14 },
  fractional_divider_t{ .ratio = 1889, .numerator = 8, .denominator = 9 },
  fractional_divider_t{ .ratio = 1167, .numerator = 1, .denominator = 6 },
  fractional_divider_t{ .ratio = 1417, .numerator = 5, .denominator = 12 },
  fractional_divider_t{ .ratio = 1667, .numerator = 2, .denominator = 3 },
  fractional_divider_t{ .ratio = 1900, .numerator = 9, .denominator = 10 },
  fractional_divider_t{ .ratio = 1182, .numerator = 2, .denominator = 11 },
  fractional_divider_t{ .ratio = 1429, .numerator = 3, .denominator = 7 },
  fractional_divider_t{ .ratio = 1692, .numerator = 9, .denominator = 13 },
  fractional_divider_t{ .ratio = 1909, .numerator = 10, .denominator = 11 },
  fractional_divider_t{ .ratio = 1200, .numerator = 1, .denominator = 5 },
  fractional_divider_t{ .ratio = 1444, .numerator = 4, .denominator = 9 },
  fractional_divider_t{ .ratio = 1700, .numerator = 7, .denominator = 10 },
  fractional_divider_t{ .ratio = 1917, .numerator = 11, .denominator = 12 },
  fractional_divider_t{ .ratio = 1214, .numerator = 3, .denominator = 14 },
  fractional_divider_t{ .ratio = 1455, .numerator = 5, .denominator = 11 },
  fractional_divider_t{ .ratio = 1714, .numerator = 5, .denominator = 7 },
  fractional_divider_t{ .ratio = 1923, .numerator = 12, .denominator = 13 },
  fractional_divider_t{ .ratio = 1222, .numerator = 2, .denominator = 9 },
  fractional_divider_t{ .ratio = 1462, .numerator = 6, .denominator = 13 },
  fractional_divider_t{ .ratio = 1727, .numerator = 8, .denominator = 11 },
  fractional_divider_t{ .ratio = 1929, .numerator = 13, .denominator = 14 },
  fractional_divider_t{ .ratio = 1231, .numerator = 3, .denominator = 13 },
  fractional_divider_t{ .ratio = 1467, .numerator = 7, .denominator = 15 },
  fractional_divider_t{ .ratio = 1733, .numerator = 11, .denominator = 15 },
  fractional_divider_t{ .ratio = 1933, .numerator = 14, .denominator = 15 },
};

/**
 * @brief Find the closest fractional value to the target ratio
 *
 * @param p_ratio - target ratio to hit the target baud rate
 * @return constexpr fractional_divider_t - fractional value representing the
 * closest approximation to the target ratio.
 */
constexpr fractional_divider_t closest_fractional(int32_t p_ratio)
{
  fractional_divider_t result = fractional_table[0];
  auto difference = std::numeric_limits<int32_t>::max();

  for (auto const& fraction : fractional_table) {
    int32_t new_difference = hal::absolute_value(p_ratio - fraction.ratio);
    if (new_difference < difference) {
      result = fraction;
      difference = new_difference;
    }
  }

  return result;
}
/**
 * @brief Calculate the baud rate register values
 *
 * @param p_baud_rate - the target baud rate
 * @param p_frequency_hz - clock frequency driving uart peripheral
 * @return constexpr uart_baud_t - returns the baud rate register values
 */
constexpr uart_baud_t calculate_baud(uint32_t p_baud_rate,
                                     uint32_t p_frequency_hz)
{
  // The number of samples per UART frame bit.
  // This is used as a multiplier for the baudrate.
  constexpr uint32_t samples_per_bit_rate = 16;
  constexpr uint64_t thousands = 1000;

  // Hold the result return value.
  uart_baud_t result{};

  uint64_t const frequency_1000 = p_frequency_hz * thousands;
  uint32_t const sample_rate = p_baud_rate * samples_per_bit_rate;

  // Compute the integer divider for the baud rate
  uint32_t const integer_divider = (p_frequency_hz / sample_rate);

  // Computer the integer divider for the baud rate except multiplied by 1000
  // in order to get 3 additional decimal places.
  auto const divider_1000 = static_cast<uint32_t>(frequency_1000 / sample_rate);

  // Save divider to result
  result.divider = integer_divider;

  // Check if the integer divider is not zero because not doing that will cause
  // a division error. Also note that an integer divider of 0 represents a
  // failure.
  if (integer_divider != 0) {
    auto const multiplier_ratio =
      static_cast<std::int32_t>(divider_1000 / integer_divider);
    fractional_divider_t const fraction = closest_fractional(multiplier_ratio);
    result.numerator = fraction.numerator;
    result.denominator = fraction.denominator;
  }

  return result;
}
}  // namespace hal::lpc40
