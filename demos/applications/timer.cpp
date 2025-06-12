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

  hal::print(console, "Callback timer demo starting...\n\n");

  hal::u64 start_timer = 0;
  hal::u64 stop_timer = 0;
  bool volatile interrupt_toggled = false;
  auto cpu_frequency = static_cast<hal::u32>(clock.frequency());

  struct interrupt_capture_variables
  {
    hal::steady_clock& clock;
    hal::serial& console;
    hal::u64& stop_timer;
    bool volatile& interrupt_toggled;
  };
  interrupt_capture_variables capture_vars{
    clock, console, stop_timer, interrupt_toggled
  };

  auto func_to_call = [&capture_vars]() {
    capture_vars.stop_timer = capture_vars.clock.uptime();
    hal::print<128>(capture_vars.console, "\nCallback function executed!\n");
    capture_vars.interrupt_toggled = true;
  };

  hal::u64 accumulator = 0;
  for (int i = 0; i < 10; i++) {
    start_timer = clock.uptime();
    callback_timer.schedule(func_to_call, hal::time_duration(10s));
    stop_timer = clock.uptime();
    callback_timer.cancel();
    accumulator += (stop_timer - start_timer);
  }
  accumulator /= 10;
  hal::print<128>(console,
                  "Average amount of CC to schedule a callback is:%" PRIu64
                  "\n\n",
                  accumulator);

  hal::print(console, "Scheduling callback function to occur in 250ms...\n");
  start_timer = clock.uptime();
  callback_timer.schedule(func_to_call, 250ms);
  while (interrupt_toggled == false) {
    hal::print(console, "Waiting...\n");
    hal::delay(clock, 50ms);
  }
  hal::u64 scale_factor = 1'000'000;
  auto time_elapsed = (stop_timer - start_timer) * scale_factor / cpu_frequency;
  hal::print<128>(console, "Time Elapsed:%" PRIu64 "us\n", time_elapsed);

  hal::delay(clock, 1s);
  hal::print(console, "\nTesting is_scheduled() and cancel() function...\n");
  callback_timer.schedule(func_to_call, hal::time_duration(10s));
  hal::delay(clock, 500ms);
  auto is_scheduled = callback_timer.is_running();
  auto is_scheduled_string = (is_scheduled) ? "True" : "False";
  hal::print<128>(
    console, "Callback scheduled? (Expected True): %s\n", is_scheduled_string);
  callback_timer.cancel();
  is_scheduled = callback_timer.is_running();
  is_scheduled_string = (is_scheduled) ? "True" : "False";
  hal::print<128>(
    console, "Callback scheduled? (Expected False): %s\n", is_scheduled_string);

  hal::delay(clock, 1000ms);
  hal::print(
    console,
    "\nTesting overwriting callback functionality (Should take 500ms)...\n");
  interrupt_toggled = false;
  callback_timer.schedule(func_to_call, hal::time_duration(10s));
  start_timer = clock.uptime();
  callback_timer.schedule(func_to_call, hal::time_duration(500ms));
  while (interrupt_toggled == false) {
    hal::print(console, "Waiting...\n");
    hal::delay(clock, 50ms);
  }
  time_elapsed = (stop_timer - start_timer) * scale_factor / cpu_frequency;
  hal::print<128>(console, "Time Elapsed:%" PRIu64 "us\n", time_elapsed);
}
