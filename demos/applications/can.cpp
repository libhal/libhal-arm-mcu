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
#include <libhal-util/steady_clock.hpp>

#include <libhal/can.hpp>
#include <resource_list.hpp>

void application(resource_list& p_map)
{
  using namespace hal::literals;

  auto& clock = *p_map.clock.value();
  auto& can_transceiver = *p_map.can_transceiver.value();
  auto& can_bus_manager = *p_map.can_bus_manager.value();
  auto& can_interrupt = *p_map.can_interrupt.value();
  auto& console = *p_map.console.value();

  // Change the CAN baudrate here.
  static constexpr auto baudrate = 100.0_kHz;

  hal::print(console, "Starting CAN demo!\n");

  can_bus_manager.baud_rate(baudrate);

  auto receive_handler = [&console](hal::can_interrupt::on_receive_tag,
                                    hal::can_message const& p_message) {
    hal::print<1024>(console,
                     "Received Message from ID: 0x%lX, length: %u \n"
                     "payload = [ 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, "
                     "0x%02X, 0x%02X, 0x%02X ]\n",
                     p_message.id(),
                     p_message.length,
                     p_message.payload[0],
                     p_message.payload[1],
                     p_message.payload[2],
                     p_message.payload[3],
                     p_message.payload[4],
                     p_message.payload[5],
                     p_message.payload[6],
                     p_message.payload[7]);
  };

  can_interrupt.on_receive(receive_handler);

  while (true) {
    using namespace std::chrono_literals;

    hal::can_message my_message{
      .length = 8,
      .payload = { 0xAA, 0xBB, 0xCC, 0xDD, 0xDE, 0xAD, 0xBE, 0xEF },
    };
    my_message.remote_request(false).extended(false).id(0x111);

    hal::print(console, "Sending payload...\n");
    can_transceiver.send(my_message);

    hal::delay(clock, 1s);
  }
}
