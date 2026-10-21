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

// claim_pin() and release_pin() only touch software state, so they run on the
// host. configure_pin() and reset_pin() write GPIO registers and can't.

#include <boost/ut.hpp>

import hal;
import hal.arm_mcu;

namespace {
void pin_test()
{
  using namespace boost::ut;
  using hal::stm32f1::pin;

  "claim then release frees the pin"_test = []() {
    // Setup
    constexpr pin led{ .port = 'C', .pin = 13 };
    bool const claimed_before = hal::stm32f1::is_pin_claimed(led);

    // Exercise
    hal::stm32f1::claim_pin(led);
    bool const claimed_during = hal::stm32f1::is_pin_claimed(led);
    hal::stm32f1::release_pin(led);

    // Verify
    expect(not claimed_before);
    expect(claimed_during);
    expect(not hal::stm32f1::is_pin_claimed(led));
  };

  "claiming a claimed pin throws"_test = []() {
    // Setup
    constexpr pin rx{ .port = 'A', .pin = 10 };
    hal::stm32f1::claim_pin(rx);

    // Exercise
    bool threw = false;
    try {
      hal::stm32f1::claim_pin(rx);
    } catch (hal::device_or_resource_busy const&) {
      threw = true;
    }

    // Verify
    expect(threw);
    expect(hal::stm32f1::is_pin_claimed(rx));
    hal::stm32f1::release_pin(rx);
  };

  "claims are independent per pin and per port"_test = []() {
    // Setup
    hal::stm32f1::claim_pin({ .port = 'A', .pin = 0 });

    // Exercise
    bool const a1 = hal::stm32f1::is_pin_claimed({ .port = 'A', .pin = 1 });
    bool const b0 = hal::stm32f1::is_pin_claimed({ .port = 'B', .pin = 0 });
    bool const g15 = hal::stm32f1::is_pin_claimed({ .port = 'G', .pin = 15 });

    // Verify
    expect(not a1);
    expect(not b0);
    expect(not g15);
    hal::stm32f1::release_pin({ .port = 'A', .pin = 0 });
  };

  "the last pin of port G can be claimed"_test = []() {
    // Setup
    constexpr pin last{ .port = 'G', .pin = 15 };

    // Exercise
    hal::stm32f1::claim_pin(last);
    bool const claimed = hal::stm32f1::is_pin_claimed(last);
    hal::stm32f1::release_pin(last);

    // Verify
    expect(claimed);
  };

  "pins that don't exist can't be claimed"_test = []() {
    // Exercise
    auto const rejects = [](pin p_pin) {
      try {
        hal::stm32f1::claim_pin(p_pin);
      } catch (hal::argument_out_of_domain const&) {
        return true;
      }
      return false;
    };

    // Verify
    expect(rejects({ .port = 'H', .pin = 0 }));
    expect(rejects({ .port = 'A', .pin = 16 }));
    expect(rejects({ .port = '@', .pin = 0 }));
    expect(not hal::stm32f1::is_pin_claimed({ .port = 'H', .pin = 0 }));
  };

  "releasing a free or nonexistent pin does nothing"_test = []() {
    // Exercise
    hal::stm32f1::release_pin({ .port = 'B', .pin = 7 });
    hal::stm32f1::release_pin({ .port = 'Z', .pin = 99 });

    // Verify
    expect(not hal::stm32f1::is_pin_claimed({ .port = 'B', .pin = 7 }));
  };
}
}  // namespace

int main()
{
  pin_test();
}
