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
#include <chrono>

module arm_mcu_demos;

import hal;
import hal.util;

void application()
{
  auto console = resources::console();
  auto spi = resources::spi_channel();

  hal::write(*console, "SPI Application Starting...\n");
  hal::write(*console, "Connect COPI to CIPO to read back what is sent.\n");

  std::array<hal::byte, 4> const sent{ 0xDE, 0xAD, 0xBE, 0xEF };

  while (true) {
    using namespace std::chrono_literals;

    std::array<hal::byte, 4> received{};
    // Selects the device around the transfer.
    spi->transfer({ sent }, { received });

    hal::print<64>(*console,
                   "received: %02X %02X %02X %02X\n",
                   static_cast<unsigned>(received[0]),
                   static_cast<unsigned>(received[1]),
                   static_cast<unsigned>(received[2]),
                   static_cast<unsigned>(received[3]));

    hal::sleep_for(1s);
  }
}
