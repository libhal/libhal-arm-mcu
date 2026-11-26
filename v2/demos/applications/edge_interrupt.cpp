// Copyright 2026 Khalil Estell and the libhal contributors
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
module;

#include <chrono>
#include <cinttypes>
#include <cstdint>

module arm_mcu_demos;

import hal;
import hal.util;

namespace {
/// Counts the edges and remembers the pin's level after the last one. Runs in
/// interrupt context, so it only stores.
class edge_counter final : public hal::edge_triggered_callback
{
public:
  void callback(bool p_state) noexcept override
  {
    m_count = m_count + 1U;
    m_level = p_state;
  }

  [[nodiscard]] std::uint32_t count() const
  {
    return m_count;
  }

  [[nodiscard]] bool level() const
  {
    return m_level;
  }

private:
  std::uint32_t volatile m_count = 0;
  bool volatile m_level = false;
};
}  // namespace

/// Prints a count each time the interrupt pin sees an edge, and mirrors the
/// pin's level onto the status LED.
void application()
{
  using namespace std::chrono_literals;

  auto console = resources::console();
  auto led = resources::status_led();
  auto interrupt = resources::edge_interrupt();

  hal::write(*console, "Edge interrupt demo: press the button\n");

  edge_counter counter;
  auto const subscription = interrupt->on_trigger(counter);

  std::uint32_t last_count = 0;
  while (true) {
    auto const count = counter.count();
    if (count != last_count) {
      last_count = count;
      led->level(counter.level());
      hal::print<32>(*console, "edges: %" PRIu32 "\n", count);
    }
    hal::sleep_for(10ms);
  }
}
