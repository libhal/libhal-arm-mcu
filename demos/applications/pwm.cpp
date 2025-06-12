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

#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

#include <resource_list.hpp>

void application(resource_list& p_map)
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  auto& pwm = *p_map.pwm.value();
  auto& clock = *p_map.clock.value();
  auto& console = *p_map.console.value();

  while (true) {
    pwm.duty_cycle(0.0f);
    pwm.frequency(1.0_kHz);
    hal::print(console, "Sweeping duty cycle from 0 to 1 \n");
    hal::delay(clock, 1s);
    float constexpr duty_cycle_step_count = 20;
    float const duty_cycle_step = 1 / duty_cycle_step_count;
    for (float duty_cycle = 0; duty_cycle < 1; duty_cycle += duty_cycle_step) {
      hal::print<64>(console, ">> Duty: %.2f \n", duty_cycle);
      pwm.duty_cycle(duty_cycle);
      hal::delay(clock, 100ms);
    }

    pwm.duty_cycle(0.0f);

    hal::print(console, "Sweeping frequency from 1kHz to 20kHz\n");
    hal::print(console, ">> pwm Duty Cycle = 50%\n");
    hal::delay(clock, 1s);
    pwm.duty_cycle(1.0f / 2);  // 50% duty cycle

    for (float multiplier = 1; multiplier < 20; multiplier++) {
      float frequency = 1000 /* Hz */ * multiplier;
      pwm.frequency(frequency);
      hal::print<64>(console, ">> Freq: %f Hz\n", frequency);
      hal::delay(clock, 100ms);
    }

    hal::print(console, "\n");
  }
}
