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

#include <libhal-util/as_bytes.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/timeout.hpp>

#include <libhal/zero_copy_serial.hpp>
#include <resource_list.hpp>

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
void print(hal::zero_copy_serial& p_serial, byte_array_t&& p_data)
{
  p_serial.write(hal::as_bytes(p_data));
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
void print(hal::zero_copy_serial& p_serial,
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

  p_serial.write(hal::as_bytes(buffer).first(length));
}

void application(resource_list& p_map)
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  auto& clock = *p_map.clock.value();
  auto& console = *p_map.zero_copy_serial.value();

  auto previous_cursor = console.receive_cursor();

  while (true) {
    using namespace std::chrono_literals;
    using namespace std::string_view_literals;

    std::string_view message = "Hello, World!\n";
    print(console, message);

    auto const new_cursor = console.receive_cursor();
    print<64>(console, "cursor = %zu\n", new_cursor);

    hal::usize byte_count = 0;
    if (new_cursor < previous_cursor) {
      byte_count = console.receive_buffer().size() - new_cursor;
      // Echo anything received
      console.write(
        console.receive_buffer().subspan(previous_cursor, byte_count));
      previous_cursor = 0;
    } else {
      byte_count = new_cursor - previous_cursor;
      // Echo anything received
      console.write(
        console.receive_buffer().subspan(previous_cursor, byte_count));
      previous_cursor = new_cursor;
    }

    hal::delay(clock, 1s);
  }
}
