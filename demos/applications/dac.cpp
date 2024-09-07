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

#include <libhal-arm-mcu/dwt_counter.hpp>
#include <libhal-arm-mcu/lpc40/clock.hpp>
#include <libhal-arm-mcu/lpc40/constants.hpp>
#include <libhal-arm-mcu/lpc40/dac.hpp>
#include <libhal-arm-mcu/lpc40/uart.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

#include <resource_list.hpp>

void application(resource_list& p_map)
{
  auto& dac = *p_map.dac.value();
  auto& counter = *p_map.clock.value();
  auto& uart0 = *p_map.console.value();
  while (true) {
    using namespace std::chrono_literals;
    float f1 = 0.0f;
    float f2 = 0.5f;
    float f3 = 1.0f;
    dac.write(f1);
    hal::print<32>(uart0, "Written %f\n", f1);
    hal::delay(counter, 5s);
    dac.write(f2);
    hal::print<32>(uart0, "Written %f\n", f2);
    hal::delay(counter, 5s);
    dac.write(f3);
    hal::print<32>(uart0, "Written %f\n", f3);
    hal::delay(counter, 5s);
  }
}
