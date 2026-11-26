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
#include <cinttypes>
#include <cstdint>

module arm_mcu_demos;

import hal;
import hal.util;

namespace {
void print_can_message(hal::serial& p_console,
                       hal::can_message const& p_message)
{
  hal::print<64>(p_console,
                 "Received { id: 0x%" PRIX32 ", length: %u, payload: [ ",
                 p_message.id,
                 static_cast<unsigned>(p_message.length));
  for (hal::usize i = 0; i < p_message.length; i++) {
    hal::print<8>(
      p_console, "0x%02X ", static_cast<unsigned>(p_message.payload[i]));
  }
  hal::write(p_console, "] }\n");
}
}  // namespace

/// Sends four messages every second, and prints the ones received with ID
/// 0x111, the only ID the filter lets through.
void application()
{
  using namespace std::chrono_literals;

  auto console = resources::console();
  auto transceiver = resources::can_transceiver();
  auto bus_manager = resources::can_bus_manager();
  auto id_filter = resources::can_identifier_filter();

  hal::write(*console, "Starting CAN demo!\n");

  bus_manager->baud_rate(1'000'000);

  constexpr std::uint16_t allowed_id = 0x111;
  id_filter->allow(allowed_id);
  bus_manager->filter_mode(hal::can_message_acceptance::filtered);
  hal::print<64>(
    *console, "Allowing ID [0x%" PRIX16 "] through the filter!\n", allowed_id);

  hal::can_message_finder message_finder(*transceiver, allowed_id);

  std::array<hal::can_message, 4> const messages{ {
    {
      .id = 0x111,
      .length = 8,
      .payload = { 0xAA, 0xBB, 0xCC, 0xDD, 0xDE, 0xAD, 0xBE, 0xEF },
    },
    { .id = 0x333, .length = 0 },
    {
      .id = 0x0123'4567,
      .extended = true,
      .length = 3,
      .payload = { 0xAA, 0xBB, 0xCC },
    },
    {
      .id = 0x0222'0005,
      .extended = true,
      .length = 3,
      .payload = { 0xAA, 0xBB, 0xCC },
    },
  } };

  while (true) {
    hal::write(*console, "Sending 4 messages...\n");
    try {
      for (auto const& message : messages) {
        transceiver->send(message);
      }
    } catch (hal::timed_out const&) {
      hal::write(*console,
                 "No node is acknowledging the messages. Trying again...\n");
    } catch (hal::operation_not_permitted const&) {
      hal::write(*console, "The bus is off. Rejoining...\n");
      bus_manager->bus_on();
    }

    hal::sleep_for(1s);

    for (auto message = message_finder.find(); message.has_value();
         message = message_finder.find()) {
      print_can_message(*console, *message);
    }
  }
}
