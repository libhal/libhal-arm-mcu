export module hal.arm_mcu.stm32_generic:registers;

import hal;

namespace hal::stm32_generic {
/// stm32 general purpose/advanced timer register map, shared by the pwm
/// helper driver.
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

}  // namespace hal::stm32_generic
