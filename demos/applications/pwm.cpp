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

#include <resource_list.hpp>

void application(resource_list& p_map)
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  auto& pwm = *p_map.pwm.value();
  auto& serial = *p_map.console.value();
  auto& clock = *p_map.clock.value();
  auto& pwm2 = *p_map.pwm2.value();
  // auto& led = *p_map.status_led.value();
  hal::print(serial, "PWM Application starting...\n");

  while (true) {

    hal::print(serial, "Setting PWM frequency to 1kHz\n");
    pwm.frequency(1.0_kHz);
    pwm2.frequency(1.0_kHz);

    for (unsigned iteration = 0; iteration <= 100; iteration += 1) {
      auto duty_cycle = static_cast<float>(iteration) / 100.0f;
      pwm.duty_cycle(duty_cycle);
      pwm2.duty_cycle(duty_cycle);
      // led.level(true);
      hal::delay(clock, 10ms);
    }

    pwm.duty_cycle(0.5f);

    for (unsigned iteration = 0; iteration < 100; iteration++) {
      auto frequency = 100.0_Hz * (static_cast<float>(iteration) * 10);
      pwm.frequency(frequency);
      pwm2.frequency(frequency);
      // led.level(false);

      hal::delay(clock, 100ms);
    }
  }
}
