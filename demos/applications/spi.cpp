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

#include <libhal-util/serial.hpp>
#include <libhal-util/spi.hpp>
#include <libhal-util/steady_clock.hpp>

#include <resource_list.hpp>

void application(resource_list& p_map)
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  auto& clock = *p_map.clock.value();
  auto& console = *p_map.console.value();
  auto& spi = *p_map.spi.value();
  auto& chip_select = *p_map.spi_chip_select.value();

  chip_select.level(true);

  hal::print(console, "Starting SPI Application...\n");

  while (true) {
    using namespace std::literals;

    std::array<hal::byte, 4> const payload{ 0xDE, 0xAD, 0xBE, 0xEF };
    std::array<hal::byte, 8> buffer{};

    hal::print(console, "Write operation\n");
    chip_select.level(false);
    hal::write(spi, payload);
    chip_select.level(true);
    hal::delay(clock, 1s);

    hal::print(console, "Read operation: [ ");
    hal::read(spi, buffer);

    for (auto const& byte : buffer) {
      hal::print<32>(console, "0x%02X ", byte);
    }

    hal::print(console, "]\n");
    hal::delay(clock, 1s);

    hal::print(console, "Full-duplex transfer\n");
    chip_select.level(false);
    spi.transfer(payload, buffer);
    chip_select.level(true);
    hal::delay(clock, 1s);

    hal::print(console, "Half-duplex transfer\n");
    chip_select.level(false);
    hal::write_then_read(spi, payload, buffer);
    chip_select.level(true);
    hal::delay(clock, 1s);

    {
      std::array read_manufacturer_id{ hal::byte{ 0x9F } };
      std::array<hal::byte, 4> id_data{};

      chip_select.level(false);
      hal::delay(clock, 250ns);  // wait at least 250ns
      hal::write_then_read(spi, read_manufacturer_id, id_data, 0xA5);
      chip_select.level(true);

      hal::print(console, "SPI Flash Memory ID info: ");
      hal::print(console, "[ ");
      for (auto const& byte : id_data) {
        hal::print<32>(console, "0x%02X ", byte);
      }
      hal::print(console, "]\n");
    }

    hal::delay(clock, 1s);
  }
}
