#pragma once
#include "libhal-arm-mcu/stm32f1/constants.hpp"
#include <libhal-util/bit.hpp>
#include <libhal/pwm.hpp>
#include <libhal/units.hpp>
namespace hal::stm32f1 {
class pwm_wrapper;
}  // namespace hal::stm32f1

namespace hal::stm32_generic {
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
  std::uint32_t volatile capture_compare_mode_register;  // set up modes for
                                                         // channel
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

struct pwm_settings
{
  int channel;
  hertz frequency;
  bool is_advanced;
};

/**
 * @brief This class cannot be called directly. The user must instantiate a
 * General Purpose or Advanced timer class first, and then acquire a pwm pin
 * through that class.
 */
class pwm final : public hal::pwm
{
public:
  pwm(pwm const& p_other) = delete;
  pwm& operator=(pwm const& p_other) = delete;
  pwm(pwm&& p_other) noexcept = delete;
  pwm& operator=(pwm&& p_other) noexcept = delete;
  /**
   * @brief when it is destroyed the corresponding
   * peripheral is powered off
   */
  ~pwm() = default;

  friend class hal::stm32f1::pwm_wrapper;

private:
  /**
   * @brief The pwm constructor is private because the only way one should be
   * able to access pwm is through the timer class
   */
  pwm(void* pwm_pin,
      pwm_settings settings);  // this just needs to take a p_reg address and a
                               // channel number

  void driver_frequency(hertz p_frequency) override;
  void driver_duty_cycle(float p_duty_cycle) override;

  uint32_t volatile* m_compare_register_addr;
  stm32_generic::pwm_reg_t* m_reg;
  int m_channel;
  hertz m_clock_freq;
};
}  // namespace hal::stm32_generic
