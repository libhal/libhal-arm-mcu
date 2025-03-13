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

#include <optional>

#include <libhal-util/as_bytes.hpp>
#include <libhal/adc.hpp>
#include <libhal/can.hpp>
#include <libhal/dac.hpp>
#include <libhal/functional.hpp>
#include <libhal/i2c.hpp>
#include <libhal/input_pin.hpp>
#include <libhal/interrupt_pin.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/pwm.hpp>
#include <libhal/serial.hpp>
#include <libhal/spi.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/stream_dac.hpp>
#include <libhal/zero_copy_serial.hpp>

struct resource_list
{
  hal::callback<void()> reset;
  // Each resource is made optional because some mcus do not support all
  // possible drivers in the resource list. If an application needs a driver it
  // will access them via `std::optional::value()` which will throw an exception
  // if the value, is not present. That exception will be caught in main and a
  // message will be printed if the `console` field has been set, then call
  // std::terminate.
  std::optional<hal::zero_copy_serial*> console;
  std::optional<hal::serial*> old_uart;
  std::optional<hal::output_pin*> status_led;
  std::optional<hal::steady_clock*> clock;
  std::optional<hal::can_transceiver*> can_transceiver;
  std::optional<hal::can_bus_manager*> can_bus_manager;
  std::optional<hal::can_interrupt*> can_interrupt;
  std::optional<hal::adc*> adc;
  std::optional<hal::input_pin*> input_pin;
  std::optional<hal::i2c*> i2c;
  std::optional<hal::interrupt_pin*> interrupt_pin;
  std::optional<hal::pwm*> pwm;
  std::optional<hal::pwm16_channel*> pwm_channel;
  std::optional<hal::pwm_group_manager*> pwm_frequency;
  std::optional<hal::spi*> spi;
  std::optional<hal::output_pin*> spi_chip_select;
  std::optional<hal::stream_dac_u8*> stream_dac;
  std::optional<hal::dac*> dac;
};

// Each application file should have this function implemented
void application(resource_list& p_map);

// Each platform file should have this function implemented
void initialize_platform(resource_list& p_resources);

namespace hal {

/**
 * @ingroup Serial
 * @brief Write std::span of const char to a serial port
 *
 * @param p_serial - the serial port that will be written to
 * @param p_data_out - chars to be written out the port
 * @param p_timeout - timeout callable that throws a std::errc::timed_out
 * exception when the timeout expires.
 * @throws std::errc::timed_out - if p_timeout expires
 */
inline void write(zero_copy_serial& p_serial, std::span<byte> p_data_out)
{
  p_serial.write(p_data_out);
}

/**
 * @ingroup Serial
 * @brief Write std::span of const char to a serial port
 *
 * @param p_serial - the serial port that will be written to
 * @param p_data_out - chars to be written out the port
 * @param p_timeout - timeout callable that throws a std::errc::timed_out
 * exception when the timeout expires.
 * @throws std::errc::timed_out - if p_timeout expires
 */
inline void write(zero_copy_serial& p_serial, std::string_view p_data_out)
{
  p_serial.write(as_bytes(p_data_out));
}

/**
 * @ingroup Serial
 * @brief Write data to serial buffer and drop return value
 *
 * Only use this with serial ports with infallible write operations, meaning
 * they will never return an error result.
 *
 * @tparam byte_array_t - data array type
 * @param p_serial - serial port to write data to
 * @param p_data - data to be sent over the serial port
 */
template<typename byte_array_t>
void print(zero_copy_serial& p_serial, byte_array_t&& p_data)
{
  write(p_serial, p_data);
}

/**
 * @ingroup Serial
 * @brief Write formatted string data to serial buffer and drop return value
 *
 * Uses snprintf internally and writes to a local statically allocated an array.
 * This function will never dynamically allocate like how standard std::printf
 * does.
 *
 * This function does NOT include the NULL character when transmitting the data
 * over the serial port.
 *
 * @tparam buffer_size - Size of the buffer to allocate on the stack to store
 * the formatted message.
 * @tparam Parameters - printf arguments
 * @param p_serial - serial port to write data to
 * @param p_format - printf style null terminated format string
 * @param p_parameters - printf arguments
 */
template<size_t buffer_size, typename... Parameters>
void print(zero_copy_serial& p_serial,
           char const* p_format,
           Parameters... p_parameters)
{
  static_assert(buffer_size > 2);
  constexpr int unterminated_max_string_size =
    static_cast<int>(buffer_size) - 1;

  std::array<char, buffer_size> buffer{};
  int length =
    std::snprintf(buffer.data(), buffer.size(), p_format, p_parameters...);

  if (length > unterminated_max_string_size) {
    // Print out what was able to be written to the buffer
    length = unterminated_max_string_size;
  }

  write(p_serial, std::string_view(buffer.data(), length));
}
}  // namespace hal
