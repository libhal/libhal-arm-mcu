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

#include <cinttypes>

#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

#include <resource_list.hpp>

void application(resource_list& p_map)
{
  auto& clock = *p_map.clock.value();
  auto& console = *p_map.console.value();
  auto& callback_timer = *p_map.callback_timer.value();

  hal::print(console, "Callback timer demo starting...\n");

  auto func_to_call = [&console] () 
  {
    hal::print<128>(console, "Callback function executed!\n");
  };
  hal::time_duration delay_amount(1'000'000'000);

  callback_timer.schedule(func_to_call, delay_amount);
  while (true) {
    using namespace std::chrono_literals;
    auto uptime = clock.uptime();
    hal::print<128>(console,
                    "%" PRIu32 "ns\n",
                    static_cast<std::uint32_t>(uptime));
    hal::delay(clock, 100ms);
  }
}
