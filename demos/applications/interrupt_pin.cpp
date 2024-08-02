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

#include <resource_list.hpp>

void application(resource_list& p_map)
{
  auto& led = *p_map.status_led.value();
  auto& button = *p_map.interrupt_pin.value();

  led.level(false);
  button.configure({});

  auto handler = [&led]([[maybe_unused]] bool p_level) {
    auto previous_state = led.level();
    (void)led.level(!previous_state);
  };

  button.on_trigger(handler);

  while (true) {
    continue;
  }
}
