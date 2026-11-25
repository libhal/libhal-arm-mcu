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

// Each test file includes a driver's header and runs an empty test, so the
// headers are compiled on the host. Tests are called from here rather than
// registered as global boost::ut::suite objects: those never ran, and their
// registry was reported as a leak on Linux.
namespace hal::cortex_m {
void dwt_counter_test();
void interrupt_test();
void systick_timer_test();
}  // namespace hal::cortex_m
namespace hal::lpc40 {
void adc_test();
void can_test();
void i2c_test();
void input_pin_test();
void interrupt_pin_test();
void output_pin_test();
void pwm_test();
void spi_test();
void stream_dac_test();
void uart_test();
}  // namespace hal::lpc40
namespace hal::stm32f1 {
void adc_test();
void can_test();
void can2_test();
void output_pin_test();
void spi_test();
void uart_test();
void usb_test();
}  // namespace hal::stm32f1
namespace hal::stm32f411 {
void output_pin_test();
void spi_test();
}  // namespace hal::stm32f411

int main()
{
  hal::cortex_m::dwt_counter_test();
  hal::cortex_m::interrupt_test();
  hal::cortex_m::systick_timer_test();
  hal::lpc40::adc_test();
  hal::lpc40::can_test();
  hal::lpc40::i2c_test();
  hal::lpc40::input_pin_test();
  hal::lpc40::interrupt_pin_test();
  hal::lpc40::output_pin_test();
  hal::lpc40::pwm_test();
  hal::lpc40::spi_test();
  hal::lpc40::stream_dac_test();
  hal::lpc40::uart_test();
  hal::stm32f1::adc_test();
  hal::stm32f1::can_test();
  hal::stm32f1::can2_test();
  hal::stm32f1::output_pin_test();
  hal::stm32f1::spi_test();
  hal::stm32f1::uart_test();
  hal::stm32f1::usb_test();
  hal::stm32f411::output_pin_test();
  hal::stm32f411::spi_test();
}
