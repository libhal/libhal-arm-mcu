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
  auto& clock = *p_map.clock.value();

  while (true) {
    pwm.frequency(1.0_kHz);

    for (unsigned iteration = 0; iteration <= 100; iteration += 1) {
      auto duty_cycle = static_cast<float>(iteration) / 100.0f;
      pwm.duty_cycle(duty_cycle);
      hal::delay(clock, 100ms);
    }

    pwm.duty_cycle(0.5f);

    for (unsigned iteration = 0; iteration < 100; iteration++) {
      auto frequency = 100.0_Hz * (static_cast<float>(iteration) * 10);
      pwm.frequency(frequency);
      hal::delay(clock, 100ms);
    }
  }
}