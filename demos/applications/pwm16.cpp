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
  using namespace hal::literals;

  auto& pwm = resource_contract_assert(p_map.pwm_channel);
  auto& pwm_frequency = resource_contract_assert(p_map.pwm_frequency);
  auto& console = resource_contract_assert(p_map.console);
  auto& clock = resource_contract_assert(p_map.clock);

  while (true) {
    pwm.duty_cycle(0);
    pwm_frequency.frequency(1.0_kHz);
    hal::print(console,
               "Sweeping duty cycle from 0% (0x0000) to 100% (0xFFFF)\n");
    hal::print<32>(
      console, ">> PWM Frequency = %" PRIu32 "Hz\n", pwm.frequency());
    hal::delay(clock, 1s);
    auto constexpr duty_cycle_step_count = 20;
    hal::u16 const duty_cycle_step = 0xFFFF / duty_cycle_step_count;
    for (hal::u32 duty_cycle = 0; duty_cycle < 0xFFFF;
         duty_cycle += duty_cycle_step) {
      hal::print<64>(
        console, ">> Duty: 0x%04" PRIX32 " / 0xFFFF \n", duty_cycle);
      pwm.duty_cycle(duty_cycle);
      hal::delay(clock, 100ms);
    }

    pwm.duty_cycle(0);

    hal::print(console, "Sweeping frequency from 1kHz to 20kHz\n");
    hal::print(console, ">> PWM Duty Cycle = 50%\n");
    hal::delay(clock, 1s);
    pwm.duty_cycle(0xFFFF / 2);  // 50% duty cycle

    for (hal::u32 multiplier = 1; multiplier < 20; multiplier++) {
      auto frequency = 1000 /* Hz */ * multiplier;
      pwm_frequency.frequency(frequency);
      hal::print<64>(console, ">> Freq: %" PRIu32 "Hz\n", pwm.frequency());
      hal::delay(clock, 100ms);
    }

    hal::print(console, "\n");
  }
}
