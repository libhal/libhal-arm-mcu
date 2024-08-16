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

#include <libhal-exceptions/control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

#include <resource_list.hpp>

resource_list resources{};

[[noreturn]] void terminate_handler() noexcept
{

  if (not resources.status_led && not resources.status_led) {
    // spin here until debugger is connected
    while (true) {
      continue;
    }
  }

  // Otherwise, blink the led in a pattern

  auto& led = *resources.status_led.value();
  auto& clock = *resources.clock.value();

  while (true) {
    using namespace std::chrono_literals;
    led.level(false);
    hal::delay(clock, 100ms);
    led.level(true);
    hal::delay(clock, 100ms);
    led.level(false);
    hal::delay(clock, 100ms);
    led.level(true);
    hal::delay(clock, 1000ms);
  }
}

int main()
{
  hal::set_terminate(terminate_handler);

  resources = initialize_platform();

  try {
    application(resources);
  } catch (std::bad_optional_access const& e) {
    if (resources.console) {
      hal::print(*resources.console.value(),
                 "A resource required by the application was not available!\n"
                 "Calling terminate!\n");
    }
  }  // Allow any other exceptions to terminate the application

  std::terminate();
}

extern "C"
{
  // This gets rid of an issue with libhal-exceptions in Debug mode.
  void __assert_func()
  {
  }
}
