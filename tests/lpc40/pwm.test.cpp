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

#include <libhal-arm-mcu/lpc40/pwm.hpp>

#include "lpc40/gpio_reg.hpp"
#include "lpc40/helper.hpp"
#include "lpc40/pin_reg.hpp"
#include "lpc40/pwm_reg.hpp"
#include "lpc40/system_controller_reg.hpp"

#include <boost/ut.hpp>

namespace hal::lpc40 {
boost::ut::suite pwm_test = []() {
  using namespace boost::ut;

  auto stub_out_pin_map = stub_out_registers(&pin_map);
  auto stub_out_gpio0 = stub_out_registers(&gpio_reg[0]);
  auto stub_out_gpio1 = stub_out_registers(&gpio_reg[1]);
  auto stub_out_gpio2 = stub_out_registers(&gpio_reg[2]);
  auto stub_out_gpio3 = stub_out_registers(&gpio_reg[3]);
  auto stub_out_gpio4 = stub_out_registers(&gpio_reg[4]);
  auto stub_out_gpio5 = stub_out_registers(&gpio_reg[5]);
  auto stub_out_sys = stub_out_registers(&system_controller_reg);
  auto stub_out_pwm0 = stub_out_registers(&pwm_reg0);
  auto stub_out_pwm1 = stub_out_registers(&pwm_reg1);

  // [[maybe_unused]] auto test_pwm = hal::lpc40::pwm::get(1, 4).value();

  "pwm::ctor()"_test = []() {};
  "pwm::frequency()"_test = []() {};
  "pwm::duty_cycle()"_test = []() {};
};
}  // namespace hal::lpc40
