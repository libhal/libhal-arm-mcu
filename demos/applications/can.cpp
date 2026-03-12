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

#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>

#include <libhal/error.hpp>
#include <resource_list.hpp>

void print_can_message(hal::serial& p_console,
                       hal::can_message const& p_message)
{
  hal::print<96>(p_console,
                 "📩 Received new hal::can_message { \n"
                 "    id: 0x%lX,\n"
                 "    length: %u \n"
                 "    payload = [ ",
                 p_message.id,
                 p_message.length);

  for (auto const& byte : p_message.payload) {
    hal::print<8>(p_console, "0x%02X, ", byte);
  }

  hal::print(p_console, "]\n}\n");
}

void application()
{
  using namespace hal::literals;

  auto clock = resources::clock();
  auto can_transceiver = resources::can_transceiver();
  auto can_bus_manager = resources::can_bus_manager();
  auto can_interrupt = resources::can_interrupt();
  auto can_id_filter = resources::can_identifier_filter();
  auto console = resources::console();

  // Change the CAN baudrate here.
  static constexpr auto baudrate = 100.0_kHz;

  hal::print(*console, "🚀 Starting CAN demo!\n");

  can_bus_manager->baud_rate(baudrate);
  can_interrupt->on_receive([&console](hal::can_interrupt::on_receive_tag,
                                       hal::can_message const& p_message) {
    hal::print<64>(
      *console, "Can message with id = 0x%lX from interrupt!\n", p_message.id);
  });

  hal::print<32>(*console,
                 "Receiver buffer size = %zu\n",
                 can_transceiver->receive_buffer().size());

  constexpr auto allowed_id = 0x111;
  can_id_filter->allow(allowed_id);
  hal::print<64>(
    *console, "🆔 Allowing ID [0x%lX] through the filter!\n", allowed_id);

  hal::can_message_finder message_finder(*can_transceiver, 0x111);

  while (true) {
    using namespace std::chrono_literals;
    hal::can_message standard_message {
      .id=0x112,
      .extended=false,
      .remote_request=false,
      .length = 8,
      .payload = {
        0xAA, 0xBB, 0xCC, 0xDD, 0xDE, 0xAD, 0xBE, 0xEF,
      },
    };

    hal::can_message standard_message2{
      .id = 0x333,
      .length = 0,
    };

    hal::can_message extended_message{
      .id = 0x0123'4567,
      .extended = true,
      .length = 3,
      .payload = { 0xAA, 0xBB, 0xCC },
    };

    hal::can_message extended_message2 {
      .id = 0x0222'0005,
      .extended = true,
      .length = 3,
      .payload = {
        0xAA, 0xBB, 0xCC,
      },
    };

    hal::print(*console, "📮 Sending 4x payloads...\n");

    try {
      can_transceiver->send(standard_message);
      can_transceiver->send(standard_message2);
      can_transceiver->send(extended_message);
      can_transceiver->send(extended_message2);
    } catch (hal::resource_unavailable_try_again const&) {
      hal::print(
        *console,
        "❌ CAN messages are not getting acknowledged by the bus! Trying "
        "again...\n");
    }

    hal::delay(*clock, 1s);

    for (auto msg = message_finder.find(); msg.has_value();
         msg = message_finder.find()) {
      print_can_message(*console, *msg);
    }
  }
}
