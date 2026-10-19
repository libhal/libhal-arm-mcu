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

// #include <print>

import hal.arm_mcu;

class test_pwm : public hal::pwm16_channel
{
private:
  hal::hertz driver_frequency() final
  {
    using namespace mp_units::si::unit_symbols;
    return 10 * kHz;
  }

  void driver_duty_cycle(hal::u16) final
  {
    // std::println("duty cycle = {}/{}", p_duty_cycle, (1 << 16) - 1);
  }
};

int app_main(hal::pwm16_channel& p_pwm)
{
  try {
    [[maybe_unused]] auto const pwm_frequency_int =
      p_pwm.frequency().numerical_value_in(mp_units::si::hertz);
    // std::println("PWM frequency = {}", pwm_frequency_int);
    p_pwm.duty_cycle(1 << 15);
    p_pwm.duty_cycle(1 << 14);
    p_pwm.duty_cycle(1 << 13);
    p_pwm.duty_cycle(1 << 12);
  } catch (hal::argument_out_of_domain const&) {
    // std::println("Caught argument_out_of_domain error successfully!");
  } catch (...) {
    // std::println("Unknown error!");
    return -1;
  }
  return 0;
}

int main()
{
  try {
    test_pwm pwm;
    return app_main(pwm);
  } catch (...) {
    return -2;
  }
}
