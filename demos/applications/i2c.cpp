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

#include <libhal-util/i2c.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

#include <resource_list.hpp>

void application()
{
  auto clock = resources::clock();
  auto console = resources::console();
  auto i2c = resources::i2c();

  hal::print(*console, "Starting I2C Probe Demonstration!\n\n");
  hal::print(
    *console,
    "This application will probe the entire i2c address space looking for a\n"
    "response. When it gets one it will print it out. This demonstration \n"
    "can be used to identify the addresses of devices on your i2c bus.\n");

  while (true) {
    using namespace std::literals;

    constexpr hal::byte first_i2c_address = 0x08;
    constexpr hal::byte last_i2c_address = 0x78;

    hal::print(*console, "I2C devices found: ");

    for (hal::byte address = first_i2c_address; address < last_i2c_address;
         address++) {
      // This can only fail if the device is not present
      if (hal::probe(*i2c, address)) {
        hal::print<12>(*console, "0x%02X ", address);
      }
    }

    hal::print(*console, "\n");
    hal::delay(*clock, 1s);
  }
}
