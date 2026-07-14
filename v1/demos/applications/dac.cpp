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

#include <libhal-arm-mcu/dwt_counter.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

#include <resource_list.hpp>

void application()
{
  auto dac = resources::dac();
  auto clock = resources::clock();
  auto console = resources::console();
  while (true) {
    using namespace std::literals;
    float f1 = 0.0f;
    float f2 = 0.5f;
    float f3 = 1.0f;
    dac->write(f1);
    hal::print<32>(*console, "Written %f\n", f1);
    hal::delay(*clock, 5s);
    dac->write(f2);
    hal::print<32>(*console, "Written %f\n", f2);
    hal::delay(*clock, 5s);
    dac->write(f3);
    hal::print<32>(*console, "Written %f\n", f3);
    hal::delay(*clock, 5s);
  }
}
