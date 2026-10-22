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

// claim() and release() only touch software state, so they run on the host.
// power_on() and friends write RCC registers and can't.

#include <boost/ut.hpp>

import hal;
import hal.arm_mcu;

namespace {
void power_test()
{
  using namespace boost::ut;
  using hal::stm32f1::peripheral;

  "claim then release frees the peripheral"_test = []() {
    // Setup
    bool const claimed_before = hal::stm32f1::is_claimed(peripheral::usart1);

    // Exercise
    hal::stm32f1::claim(peripheral::usart1);
    bool const claimed_during = hal::stm32f1::is_claimed(peripheral::usart1);
    hal::stm32f1::release(peripheral::usart1);

    // Verify
    expect(not claimed_before);
    expect(claimed_during);
    expect(not hal::stm32f1::is_claimed(peripheral::usart1));
  };

  "claiming a claimed peripheral throws"_test = []() {
    // Setup
    hal::stm32f1::claim(peripheral::spi2);

    // Exercise
    bool threw = false;
    try {
      hal::stm32f1::claim(peripheral::spi2);
    } catch (hal::device_or_resource_busy const&) {
      threw = true;
    }

    // Verify
    expect(threw);
    expect(hal::stm32f1::is_claimed(peripheral::spi2));
    hal::stm32f1::release(peripheral::spi2);
  };

  "claims are independent per peripheral and per bus"_test = []() {
    // Setup
    // The same bit on each bus: AHB bit 0, APB1 bit 0, APB2 bit 0.
    hal::stm32f1::claim(peripheral::dma1);

    // Exercise
    bool const timer2 = hal::stm32f1::is_claimed(peripheral::timer2);
    bool const afio = hal::stm32f1::is_claimed(peripheral::afio);

    // Verify
    expect(not timer2);
    expect(not afio);
    hal::stm32f1::release(peripheral::dma1);
  };

  "peripherals without an enable register can't be claimed"_test = []() {
    // Exercise
    bool threw = false;
    try {
      hal::stm32f1::claim(peripheral::cpu);
    } catch (hal::argument_out_of_domain const&) {
      threw = true;
    }

    // Verify
    expect(threw);
    expect(not hal::stm32f1::is_claimed(peripheral::cpu));
  };

  "releasing a free peripheral does nothing"_test = []() {
    // Exercise
    hal::stm32f1::release(peripheral::usart2);
    hal::stm32f1::release(peripheral::cpu);

    // Verify
    expect(not hal::stm32f1::is_claimed(peripheral::usart2));
  };
}
}  // namespace

int main()
{
  power_test();
}
