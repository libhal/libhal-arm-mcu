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

#include <libhal-arm-mcu/systick_timer.hpp>

#include <libhal-arm-mcu/interrupt.hpp>
#include <libhal/units.hpp>

#include "helper.hpp"
#include "systick_timer_reg.hpp"

#include <boost/ut.hpp>

namespace hal::cortex_m {
boost::ut::suite systick_timer_test = []() {
  using namespace boost::ut;
  using namespace std::chrono_literals;
  using namespace hal::literals;

  auto saved_registers = setup_interrupts_for_unit_testing();
  auto stub_out_sys_tick = stub_out_registers(&sys_tick);
  initialize_interrupts<1>();
  systick_timer test_subject(1.0_MHz);

  should("systick_timer::systick_timer()") = [&] {
    // Setup
    // Exercise
    // Verify
  };

  should("systick_timer::is_running()") = [&] {
    // Setup
    // Exercise
    // Verify
  };

  should("systick_timer::clear()") = [&] {
    // Setup
    // Exercise
    // Verify
  };

  should("systick_timer::schedule()") = [&] {
    // Setup
    // Exercise
    // Verify
  };

  should("systick_timer::~systick_timer()") = [&] {
    // Setup
    // Exercise
    // Verify
  };
};
}  // namespace hal::cortex_m
