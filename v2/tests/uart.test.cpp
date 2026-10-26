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

// compute_usart_brr() is pure arithmetic, so it runs on the host.

#include <tuple>

#include <boost/ut.hpp>

import hal;
import hal.arm_mcu;

namespace {
void uart_test()
{
  using namespace boost::ut;
  using namespace mp_units::si::unit_symbols;
  using hal::stm32_generic::compute_usart_brr;

  "keeps the fraction of USARTDIV"_test = []() {
    // Exercise
    // 8 MHz / (16 * 115200) = 4.34: mantissa 4, fraction 5.4 rounds to 5,
    // which runs at 115,942 baud (+0.6%). Dropping the fraction would give
    // 0x40, which runs at 125,000 baud (+8.5%).
    auto const brr = compute_usart_brr(8 * MHz, 115'200 * Hz);

    // Verify
    expect(that % 0x45 == brr);
  };

  "an exact divisor has no error"_test = []() {
    // Exercise
    // 72 MHz / 115200 = 625
    auto const brr = compute_usart_brr(72 * MHz, 115'200 * Hz);

    // Verify
    expect(that % 625 == brr);
  };

  "rounds to the nearest step"_test = []() {
    // Exercise
    // 36 MHz / 115200 = 312.5, which rounds up
    auto const brr = compute_usart_brr(36 * MHz, 115'200 * Hz);

    // Verify
    expect(that % 313 == brr);
  };

  "rejects baud rates the divider can't reach"_test = []() {
    // Exercise + Verify
    // Zero would divide by zero.
    expect(throws<hal::operation_not_supported>(
      [] { std::ignore = compute_usart_brr(8 * MHz, 0 * Hz); }));
    // 8 MHz / 1 MHz = 8, a USARTDIV below 1.
    expect(throws<hal::operation_not_supported>(
      [] { std::ignore = compute_usart_brr(8 * MHz, 1 * MHz); }));
    // 72 MHz / 1000 = 72,000, more than BRR holds.
    expect(throws<hal::operation_not_supported>(
      [] { std::ignore = compute_usart_brr(72 * MHz, 1'000 * Hz); }));
  };

  "accepts both ends of the divider's range"_test = []() {
    // Exercise + Verify
    expect(that % 16 == compute_usart_brr(8 * MHz, 500 * kHz));
    expect(that % 0xFFFF == compute_usart_brr(65'535 * kHz, 1'000 * Hz));
  };
}
}  // namespace

int main()
{
  uart_test();
}
