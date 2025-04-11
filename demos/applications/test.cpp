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
  using namespace std::chrono_literals;

  auto& clock = *p_map.clock.value();
  auto& console = *p_map.console.value();
  auto& callback_timer = *p_map.callback_timer.value();

  hal::print(console, "Callback timer demo starting...\n");

  volatile bool interrupt_toggled = false;







  // &interrupt_toggled <-- add this back to capture list later
  auto func_to_call = [&console, &clock]() {
    auto stop_timer = clock.uptime();
    hal::print<128>(console, "\nCallback function executed! Clock cycle %" PRIu32 "\n", static_cast<std::uint32_t>(stop_timer));
    //interrupt_toggled = true;
  };

  hal::print(console,
             "Scheduling callback function to occur in 3 seconds...\n");

  callback_timer.schedule(func_to_call, 250ms);
  auto start_timer = clock.uptime();
  hal::u64 uptime = 0;
  while (static_cast<std::uint32_t>(uptime) < 231281530) {
    uptime = clock.uptime();
    hal::print<128>(console,
                    "Cycles: %" PRIu32 "\n",
                    static_cast<std::uint32_t>(uptime));
    hal::delay(clock, 100ms);
  }
  hal::print<128>(console, "Start Clock cycle %" PRIu32 "\n", static_cast<std::uint32_t>(start_timer));





  hal::delay(clock, 1000ms);
  hal::print(console, "Testing is_scheduled() and cancel() function...\n");
  callback_timer.schedule(func_to_call, hal::time_duration(10'000'000'000));
  hal::delay(clock, 500ms);
  auto is_scheduled = callback_timer.is_running();
  auto is_scheduled_string = (is_scheduled) ? "true" : "false";
  hal::print<128>(console, "Callback scheduled (Expected True)?: %s\n", is_scheduled_string);
  callback_timer.cancel();
  is_scheduled = callback_timer.is_running();
  is_scheduled_string = (is_scheduled) ? "true" : "false";
  hal::print<128>(console, "Callback scheduled (Expected False)?: %s\n", is_scheduled_string);

  hal::delay(clock, 1000ms);
  hal::print(console, "Testing overwriting callback functionality...\n");
  interrupt_toggled = false;
  callback_timer.schedule(func_to_call, hal::time_duration(20'000'000'000));
  callback_timer.schedule(func_to_call, hal::time_duration(1'000'000'000));
  while (interrupt_toggled == false) {
    auto uptime2 = clock.uptime();
    hal::print<128>(console,
                    "Current Uptime: %" PRIu32 "ns\n",
                    static_cast<std::uint32_t>(uptime2));
    hal::delay(clock, 100ms);
  }
}
