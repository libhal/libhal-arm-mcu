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

#include <chrono>
#include <coroutine>

module arm_mcu_demos;

import hal;
import hal.util;

namespace {
async::future<void> print_can_message(async::context& p_ctx,
                                      hal::serial& p_console,
                                      hal::can_message const& p_message)
{
  co_await hal::print<96>(p_ctx,
                          p_console,
                          "Received new hal::can_message {\n"
                          "    id: 0x%lX,\n"
                          "    length: %u\n"
                          "    payload = [ ",
                          p_message.id,
                          p_message.length);

  for (auto const& byte : p_message.payload) {
    co_await hal::print<8>(p_ctx, p_console, "0x%02X, ", byte);
  }

  co_await hal::write(p_ctx, p_console, "]\n}\n");
}
}  // namespace

hal::task application(async::context& p_ctx)
{
  using namespace std::chrono_literals;

  auto console = resources::console();
  auto transceiver = resources::can_transceiver();
  auto bus_manager = resources::can_bus_manager();
  auto id_filter = resources::can_identifier_filter();

  co_await hal::write(p_ctx, *console, "Starting CAN demo!\n");

  co_await bus_manager->baud_rate(p_ctx, 100'000);

  constexpr auto allowed_id = 0x111;
  co_await id_filter->allow(p_ctx, allowed_id);
  co_await hal::print<64>(
    p_ctx, *console, "Allowing ID [0x%lX] through the filter!\n", allowed_id);

  hal::can_message_finder message_finder(*transceiver, allowed_id);

  while (true) {
    hal::can_message const standard_message{
      .id = 0x111,
      .extended = false,
      .remote_request = false,
      .length = 8,
      .payload = { 0xAA, 0xBB, 0xCC, 0xDD, 0xDE, 0xAD, 0xBE, 0xEF },
    };

    hal::can_message const standard_message2{
      .id = 0x333,
      .length = 0,
    };

    hal::can_message const extended_message{
      .id = 0x0123'4567,
      .extended = true,
      .length = 3,
      .payload = { 0xAA, 0xBB, 0xCC },
    };

    hal::can_message const extended_message2{
      .id = 0x0222'0005,
      .extended = true,
      .length = 3,
      .payload = { 0xAA, 0xBB, 0xCC },
    };

    co_await hal::write(p_ctx, *console, "Sending 4x payloads...\n");

    int send_count = 0;
    bool bus_unresponsive = false;
    try {
      co_await transceiver->send(p_ctx, standard_message);
      send_count++;
      co_await transceiver->send(p_ctx, standard_message2);
      send_count++;
      co_await transceiver->send(p_ctx, extended_message);
      send_count++;
      co_await transceiver->send(p_ctx, extended_message2);
      send_count++;
    } catch (hal::resource_unavailable_try_again const&) {
      bus_unresponsive = send_count == 0;
    }

    if (bus_unresponsive) {
      co_await hal::write(p_ctx,
                          *console,
                          "CAN messages are not getting acknowledged by "
                          "the bus! Trying again...\n");
    }

    co_await 1s;

    for (auto msg = message_finder.find(); msg.has_value();
        msg = message_finder.find()) {
      co_await print_can_message(p_ctx, *console, *msg);
      co_await hal::print<64>(p_ctx,
                              *console,
                              "Receive cursor: %zu\n\n",
                              transceiver->receive_cursor());
    }
  }
}
