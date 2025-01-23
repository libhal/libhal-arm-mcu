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

#pragma once

#include <cstdint>

#include <libhal/pwm.hpp>

namespace hal::stm32f1 {
struct pwm_reg_t
{
  /// Offset: 0x00 Control Register (R/W)
  std::uint32_t volatile control_register;  // sets up timers
  /// Offset: 0x04 Control Register 2 (R/W)
  std::uint32_t volatile control_register_2;
  /// Offset: 0x08 Peripheral Mode Control Register (R/W)
  std::uint32_t volatile peripheral_control_register;
  /// Offset: 0x0C DMA/Interrupt enable register (R/W)
  std::uint32_t volatile interuupt_enable_register;
  /// Offset: 0x10 Status Register register (R/W)
  std::uint32_t volatile status_register;
  /// Offset: 0x14 Event Generator Register register (R/W)
  std::uint32_t volatile event_generator_register;
  /// Offset: 0x18 Capture/Compare mode register (R/W)
  std::uint32_t volatile capture_compare_mode_register;  // set up modes for the
  /// Offset: 0x1C Capture/Compare mode register (R/W)
  std::uint32_t volatile capture_compare_mode_register_2;
  /// Offset: 0x20 Capture/Compare Enable register (R/W)
  std::uint32_t volatile cc_enable_register;
  /// Offset: 0x24 Counter (R/W)
  std::uint32_t volatile counter_register;
  /// Offset: 0x28 Prescalar (R/W)
  std::uint32_t volatile prescale_register;
  /// Offset: 0x2C Auto Reload Register (R/W)
  std::uint32_t volatile auto_reload_register;  // affects frequency
  /// Offset: 0x30 Repetition Counter Register (R/W)
  std::uint32_t volatile repetition_counter_register;
  /// Offset: 0x34 Capture Compare Register (R/W)
  std::uint32_t volatile capture_compare_register;  // affects duty cycles
  /// Offset: 0x38 Capture Compare Register (R/W)
  std::uint32_t volatile capture_compare_register_2;
  // Offset: 0x3C Capture Compare Register (R/W)
  std::uint32_t volatile capture_compare_register_3;
  // Offset: 0x40 Capture Compare Register (R/W)
  std::uint32_t volatile capture_compare_register_4;
  /// Offset: 0x44 Break and dead-time register
  std::uint32_t volatile break_and_deadtime_register;
  /// Offset: 0x48 DMA control register
  std::uint32_t volatile dma_control_register;
  /// Offset: 0x4C DMA address for full transfer
  std::uint32_t volatile dma_address_register;
};

inline pwm_reg_t* pwm_timer1 =
  reinterpret_cast<pwm_reg_t*>(0x4001'2C00);  // TIM1 timer
inline pwm_reg_t* pwm_timer2 = reinterpret_cast<pwm_reg_t*>(0x4000'0000);
inline pwm_reg_t* pwm_timer3 = reinterpret_cast<pwm_reg_t*>(0x4000'0400);
inline pwm_reg_t* pwm_timer4 = reinterpret_cast<pwm_reg_t*>(0x4000'0800);
inline pwm_reg_t* pwm_timer5 = reinterpret_cast<pwm_reg_t*>(
  0x4000'0C00);  // does not exist on the stm32f103x8 chips
inline pwm_reg_t* pwm_timer8 = reinterpret_cast<pwm_reg_t*>(
  0x4001'3400);  // TIM8 timer (Does not exist on stm32f103c8 chip)
// Note: Timers 6 and 7 do not have pwm functionality.
// Note: Timers 9/12 have only 2 channels of pwm available
// Note: Timers 10,11,13,14 have 1 channel of pwm.
class timer1_pwm final : public hal::pwm
{
public:

  /**
   * @brief Default pins for the various pwm channels
   */
  enum class pwm_pins : std::uint8_t
  {
    pa0 = 0,
    pa1 = 1,
    pa2 = 2,
    pa3 = 3,
    pa6 = 4,
    pa7 = 5,
    pa8 = 6,
    pa9 = 7,
    pa10 = 8,
    pa11 = 9,
    pb0 = 10,
    pb1 = 11,
    pb6 = 12,
    pb7 = 13,
    pb8 = 14,
    pb9 = 15,
  };
  timer1_pwm(pwm_pins pwm_pin);

  timer1_pwm(timer1_pwm const& p_other) = delete;
  timer1_pwm& operator=(timer1_pwm const& p_other) = delete;
  timer1_pwm(timer1_pwm&& p_other) noexcept = delete;
  timer1_pwm& operator=(timer1_pwm&& p_other) noexcept = delete;
  virtual ~timer1_pwm() = default;

private:
  void driver_frequency(hertz p_frequency) override;
  void driver_duty_cycle(float p_duty_cycle) override;

  pwm_pins m_pin;
  uint8_t m_channel;
  uint32_t volatile* m_compare_register_addr;
};
}  // namespace hal::stm32f1
