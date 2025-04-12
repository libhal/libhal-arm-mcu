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

  resource_contract_assert(p_map.pwm);
  resource_contract_assert(p_map.clock);

  auto& pwm = *p_map.pwm;
  auto& clock = *p_map.clock;

  while (true) {
    pwm.frequency(1.0_kHz);

    for (unsigned iteration = 0; iteration <= 100; iteration += 1) {
      auto duty_cycle = static_cast<float>(iteration) / 100.0f;
      pwm.duty_cycle(duty_cycle);
      hal::delay(clock, 100ms);
    }

    pwm.duty_cycle(0.5f);

    for (unsigned iteration = 1; iteration < 20; iteration++) {
      auto const frequency = static_cast<hal::hertz>(100_Hz * iteration);
      pwm.frequency(frequency);
      hal::delay(clock, 100ms);
    }
  }
}
