export module hal.arm_mcu.stm32_generic:registers;

import hal;
import hal.util;

namespace hal::stm32_generic {
/// STM32 basic, general-purpose and advanced timer register map, shared by
/// the timer drivers
struct timer_reg
{
  /// Offset: 0x00 Control Register (R/W)
  hal::u32 volatile control_register;  // sets up timers
  /// Offset: 0x04 Control Register 2 (R/W)
  hal::u32 volatile control_register_2;
  /// Offset: 0x08 Peripheral Mode Control Register (R/W)
  hal::u32 volatile peripheral_control_register;
  /// Offset: 0x0C DMA/Interrupt enable register (R/W)
  hal::u32 volatile interrupt_enable_register;
  /// Offset: 0x10 Status Register register (R/W)
  hal::u32 volatile status_register;
  /// Offset: 0x14 Event Generator Register register (R/W)
  hal::u32 volatile event_generator_register;
  /// Offset: 0x18 Capture/Compare mode register (R/W)
  hal::u32 volatile capture_compare_mode_register;  // set up modes for
                                                    // channel
  /// Offset: 0x1C Capture/Compare mode register (R/W)
  hal::u32 volatile capture_compare_mode_register_2;
  /// Offset: 0x20 Capture/Compare Enable register (R/W)
  hal::u32 volatile cc_enable_register;
  /// Offset: 0x24 Counter (R/W)
  hal::u32 volatile counter_register;
  /// Offset: 0x28 Prescalar (R/W)
  hal::u32 volatile prescale_register;
  /// Offset: 0x2C Auto Reload Register (R/W)
  hal::u32 volatile auto_reload_register;  // affects frequency
  /// Offset: 0x30 Repetition Counter Register (R/W)
  hal::u32 volatile repetition_counter_register;
  /// Offset: 0x34 Capture Compare Register (R/W)
  hal::u32 volatile capture_compare_register;  // affects duty cycles
  /// Offset: 0x38 Capture Compare Register (R/W)
  hal::u32 volatile capture_compare_register_2;
  // Offset: 0x3C Capture Compare Register (R/W)
  hal::u32 volatile capture_compare_register_3;
  // Offset: 0x40 Capture Compare Register (R/W)
  hal::u32 volatile capture_compare_register_4;
  /// Offset: 0x44 Break and dead-time register
  hal::u32 volatile break_and_deadtime_register;
  /// Offset: 0x48 DMA control register
  hal::u32 volatile dma_control_register;
  /// Offset: 0x4C DMA address for full transfer
  hal::u32 volatile dma_address_register;

  [[nodiscard]] static timer_reg* from(void* p_address)
  {
    return reinterpret_cast<timer_reg*>(p_address);
  }
};

/**
 * @brief The timer register bits the drivers use (RM0008 sections 14.4 and
 * 15.4)
 *
 * Kept in one place so the drivers that share these registers can't drift
 * apart. Named after the RM0008 fields they hold.
 */
struct timer_bits
{
  /// TIMx_CR1
  struct control
  {
    /// CEN
    static constexpr auto counter_enable = hal::bit_mask::from<0>();
    /// URS: only a counter overflow or underflow raises an update interrupt
    static constexpr auto update_request_source = hal::bit_mask::from<2>();
    /// OPM
    static constexpr auto one_pulse_mode = hal::bit_mask::from<3>();
    /// DIR
    static constexpr auto direction = hal::bit_mask::from<4>();
    /// CMS
    static constexpr auto center_aligned_mode = hal::bit_mask::from<5, 6>();
    /// ARPE
    static constexpr auto auto_reload_preload = hal::bit_mask::from<7>();
  };

  /// TIMx_CR2
  struct control_2
  {
    /// MMS: what the timer sends out on TRGO
    static constexpr auto master_mode = hal::bit_mask::from<4, 6>();
    /// MMS = 0b010: send each update event out on TRGO
    static constexpr u32 master_mode_update = 0b010;
  };

  /// TIMx_SMCR
  struct slave_mode_control
  {
    /// SMS
    static constexpr auto slave_mode = hal::bit_mask::from<0, 2>();
    /// SMS = 0b011: encoder mode 3, count both edges of both inputs
    static constexpr u32 slave_mode_encoder_3 = 0b011;
    /// SMS = 0b111: external clock mode 1, count each rising edge of TRGI
    static constexpr u32 slave_mode_external_clock = 0b111;
    /// TS: which trigger input is TRGI
    static constexpr auto trigger_selection = hal::bit_mask::from<4, 6>();
  };

  /// TIMx_CCMR1 and TIMx_CCMR2 in input capture mode, for the first channel
  /// of each (CH1 and CH3). The second (CH2 and CH4) is 8 bits higher.
  struct capture_compare_input
  {
    /// CCxS: which input the channel captures
    static constexpr auto select = hal::bit_mask::from<0, 1>();
    /// CCxS = 0b01: the channel's own input (TI1 for CH1, TI2 for CH2)
    static constexpr u32 select_own_input = 0b01;
    /// ICxF
    static constexpr auto filter = hal::bit_mask::from<4, 7>();
  };

  /// UIE in TIMx_DIER
  static constexpr auto update_interrupt_enable = hal::bit_mask::from<0>();
  /// UIF in TIMx_SR
  static constexpr auto update_interrupt_flag = hal::bit_mask::from<0>();
  /// UG in TIMx_EGR
  static constexpr auto update_generation = hal::bit_mask::from<0>();
  /// MOE in TIMx_BDTR
  static constexpr auto main_output_enable = hal::bit_mask::from<15>();
};
}  // namespace hal::stm32_generic
