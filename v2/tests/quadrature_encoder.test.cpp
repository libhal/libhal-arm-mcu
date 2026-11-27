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

// quadrature_wrap() and quadrature_position() are pure arithmetic, so they
// run on the host.

#include <cstdint>

#include <boost/ut.hpp>

import hal;
import hal.arm_mcu;

namespace {
void quadrature_encoder_test()
{
  using namespace boost::ut;
  using hal::stm32_generic::quadrature_position;
  using hal::stm32_generic::quadrature_wrap;

  "a count near 0 after a wrap is a wrap counting up"_test = []() {
    // Exercise + Verify
    expect(that % 1 == quadrature_wrap(0));
    expect(that % 1 == quadrature_wrap(12));
    expect(that % 1 == quadrature_wrap(0x7FFF));
  };

  "a count near 65,535 after a wrap is a wrap counting down"_test = []() {
    // Exercise + Verify
    expect(that % -1 == quadrature_wrap(0xFFFF));
    expect(that % -1 == quadrature_wrap(0xFFF0));
    expect(that % -1 == quadrature_wrap(0x8000));
  };

  "wraps and count combine into a signed position"_test = []() {
    // Exercise + Verify
    expect(that % 0 == quadrature_position(0, 0));
    expect(that % 100 == quadrature_position(0, 100));
    expect(that % 65'536 == quadrature_position(1, 0));
    // One count backward from the start wraps down to 65,535
    expect(that % -1 == quadrature_position(-1, 0xFFFF));
    expect(that % -65'536 == quadrature_position(-1, 0));
    expect(that % 3'000'000'000LL == quadrature_position(45'776, 24'064));
  };
}
}  // namespace

int main()
{
  quadrature_encoder_test();
}
