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

module;

#include <array>

#include <cstdint>

export module hal.arm_mcu.stm32f1:constants;

import hal;
import hal.util;
import hal.arm_mcu.cortex_m;

namespace hal::stm32f1 {
/// Number of bits between each enable register
constexpr u32 bus_id_offset = 32;
/// Bit position of AHB
constexpr u32 ahb_bus = bus_id_offset * 0;
/// Bit position of APB1
constexpr u32 apb1_bus = bus_id_offset * 1;
/// Bit position of AHB2
constexpr u32 apb2_bus = bus_id_offset * 2;
/// Bit position of systems outside of any bus
constexpr u32 beyond_bus = bus_id_offset * 3;

/// List of each peripheral and their power on id number for this platform
export enum class peripheral : u8 {
  dma1 = ahb_bus + 0,
  dma2 = ahb_bus + 1,
  sram = ahb_bus + 2,
  flitf = ahb_bus + 4,
  crc = ahb_bus + 6,
  fsmc = ahb_bus + 8,
  sdio = ahb_bus + 10,

  timer2 = apb1_bus + 0,
  timer3 = apb1_bus + 1,
  timer4 = apb1_bus + 2,
  timer5 = apb1_bus + 3,
  timer6 = apb1_bus + 4,
  timer7 = apb1_bus + 5,
  timer12 = apb1_bus + 6,
  timer13 = apb1_bus + 7,
  timer14 = apb1_bus + 8,
  window_watchdog = apb1_bus + 11,
  spi2 = apb1_bus + 14,
  spi3 = apb1_bus + 15,
  usart2 = apb1_bus + 17,
  usart3 = apb1_bus + 18,
  uart4 = apb1_bus + 19,
  uart5 = apb1_bus + 20,
  i2c1 = apb1_bus + 21,
  i2c2 = apb1_bus + 22,
  usb = apb1_bus + 23,
  can1 = apb1_bus + 25,
  backup_clock = apb1_bus + 27,
  power = apb1_bus + 28,
  dac = apb1_bus + 29,

  afio = apb2_bus + 0,
  gpio_a = apb2_bus + 2,
  gpio_b = apb2_bus + 3,
  gpio_c = apb2_bus + 4,
  gpio_d = apb2_bus + 5,
  gpio_e = apb2_bus + 6,
  gpio_f = apb2_bus + 7,
  gpio_g = apb2_bus + 8,
  adc1 = apb2_bus + 9,
  adc2 = apb2_bus + 10,
  timer1 = apb2_bus + 11,
  spi1 = apb2_bus + 12,
  timer8 = apb2_bus + 13,
  usart1 = apb2_bus + 14,
  adc3 = apb2_bus + 15,
  timer9 = apb2_bus + 19,
  timer10 = apb2_bus + 20,
  timer11 = apb2_bus + 21,

  cpu = beyond_bus + 0,
  system_timer = beyond_bus + 1,
  i2s = beyond_bus + 2,
};

// NOLINTBEGIN(performance-enum-size): Underlying type must match
// cortex_m::irq_t exactly to satisfy the irq_enum concept used by the
// interrupt handling APIs.
/// List of interrupt request numbers for this platform
export enum class irq : cortex_m::irq_t
{
  /// Window WatchDog
  window_watchdog = 0,
  /// PVD through EXTI Line detection
  pvd = 1,
  /// Tamper
  tamper = 2,
  /// RTC
  rtc = 3,
  /// FLASH
  flash = 4,
  /// RCC
  rcc = 5,
  /// EXTI Line0
  exti0 = 6,
  /// EXTI Line1
  exti1 = 7,
  /// EXTI Line2
  exti2 = 8,
  /// EXTI Line3
  exti3 = 9,
  /// EXTI Line4
  exti4 = 10,
  /// DMA1 Channel 1
  dma1_channel1 = 11,
  /// DMA1 Channel 2
  dma1_channel2 = 12,
  /// DMA1 Channel 3
  dma1_channel3 = 13,
  /// DMA1 Channel 4
  dma1_channel4 = 14,
  /// DMA1 Channel 5
  dma1_channel5 = 15,
  /// DMA1 Channel 6
  dma1_channel6 = 16,
  /// DMA1 Channel 7
  dma1_channel7 = 17,
  /// ADC1
  adc1 = 18,
  /// ADC1 and ADC2
  adc1_2 = 18,
  /// USB Device High Priority or CAN1 TX
  usb_hp_can1_tx = 19,
  /// USB Device High Priority or CAN1 TX
  can1_tx = 19,
  /// USB Device Low Priority or CAN1 RX0
  usb_lp_can1_rx0 = 20,
  /// USB Device Low Priority or CAN1 RX0
  can1_rx0 = 20,
  /// CAN1 RX1
  can1_rx1 = 21,
  /// CAN1 SCE
  can1_sce = 22,
  /// External Line[9:5]
  exti9_5 = 23,
  /// TIM1 Break
  tim1_brk = 24,
  /// TIM1 Break and TIM15
  tim1_brk_tim15 = 24,
  /// TIM1 Break and TIM9
  tim1_brk_tim9 = 24,
  /// TIM1 Update
  tim1_up = 25,
  /// TIM1 Update and TIM16
  tim1_up_tim16 = 25,
  /// TIM1 Update and TIM10
  tim1_up_tim10 = 25,
  /// TIM1 Trigger and Commutation
  tim1_trg_com = 26,
  /// TIM1 Trigger and Commutation
  tim1_trg_com_tim11 = 26,
  /// TIM1 Capture Compare
  tim1_cc = 27,
  /// TIM2
  tim2 = 28,
  /// TIM3
  tim3 = 29,
  /// TIM4
  tim4 = 30,
  /// I2C1 Event
  i2c1_ev = 31,
  /// I2C1 Error
  i2c1_er = 32,
  /// I2C2 Event
  i2c2_ev = 33,
  /// I2C2 Error
  i2c2_er = 34,
  /// SPI1
  spi1 = 35,
  /// SPI2
  spi2 = 36,
  /// USART1
  usart1 = 37,
  /// USART2
  usart2 = 38,
  /// USART3
  usart3 = 39,
  /// External Line[15:10]
  exti15_10 = 40,
  /// RTC Alarm through EXTI Line
  rtcalarm = 41,
  /// USB Device WakeUp
  usbwakeup = 42,
  /// HDMI-CEC
  cec = 42,
  /// USB OTG FS WakeUp
  otg_fs_wkup = 42,
  /// TIM8 Break
  tim8_brk = 43,
  /// TIM12
  tim12 = 43,
  /// TIM8 Break and TIM12
  tim8_brk_tim12 = 43,
  /// TIM8 Update
  tim8_up = 44,
  /// TIM13
  tim13 = 44,
  /// TIM8 Update and TIM13
  tim8_up_tim13 = 44,
  /// TIM8 Trigger and Commutation
  tim8_trg_com = 45,
  /// TIM14
  tim14 = 45,
  /// TIM8 Trigger and Commutation
  tim8_trg_com_tim14 = 45,
  /// TIM8 Capture Compare
  tim8_cc = 46,
  /// ADC3
  adc3 = 47,
  /// FSMC
  fsmc = 48,
  /// SDIO
  sdio = 49,
  /// TIM5
  tim5 = 50,
  /// SPI3
  spi3 = 51,
  /// UART4
  uart4 = 52,
  /// UART5
  uart5 = 53,
  /// TIM6 and DAC underrun
  tim6_dac = 54,
  /// TIM6
  tim6 = 54,
  /// TIM7
  tim7 = 55,
  /// DMA2 Channel 1
  dma2_channel1 = 56,
  /// DMA2 Channel 2
  dma2_channel2 = 57,
  /// DMA2 Channel 3
  dma2_channel3 = 58,
  /// DMA2 Channel 4 and Channel 5
  dma2_channel4_5 = 59,
  /// DMA2 Channel 4
  dma2_channel4 = 59,
  /// DMA2 Channel 5
  dma2_channel5 = 60,
  /// Ethernet
  eth = 61,
  /// Ethernet Wakeup through EXTI line
  eth_wkup = 62,
  /// CAN2 TX
  can2_tx = 63,
  /// CAN2 RX0
  can2_rx0 = 64,
  /// CAN2 RX1
  can2_rx1 = 65,
  /// CAN2 SCE
  can2_sce = 66,
  /// USB OTG FS
  otg_fs = 67,
  max,
};
// NOLINTEND(performance-enum-size)

/// Register map for the Reset and Clock Control (RCC) peripheral
struct reset_and_clock_control_t
{
  u32 volatile cr;
  u32 volatile cfgr;
  u32 volatile cir;
  u32 volatile apb2rstr;
  u32 volatile apb1rstr;
  u32 volatile ahbenr;
  u32 volatile apb2enr;
  u32 volatile apb1enr;
  u32 volatile bdcr;
  u32 volatile csr;
  u32 volatile ahbrstr;
  u32 volatile cfgr2;
};

constexpr uptr rcc_address = 0x40000000 + 0x20000 + 0x1000;

/// Reset and Clock Control (RCC) peripheral register
// NOLINTNEXTLINE(performance-no-int-to-ptr)
inline auto* rcc = reinterpret_cast<reset_and_clock_control_t*>(rcc_address);

/// Bit masks for the CFGR register
struct clock_configuration
{
  /// Controls which clock signal is sent to the MCO pin
  static constexpr auto mco = bit_mask::from<24, 26>();
  /// Sets the USB clock divider
  static constexpr auto usb_prescalar = bit_mask::from<22>();
  /// Sets the PLL multiplier
  static constexpr auto pll_mul = bit_mask::from<18, 21>();
  /// If set to 1, will divide the HSE signal by 2 before sending to PLL
  static constexpr auto hse_pre_divider = bit_mask::from<17>();
  /// Sets which source the PLL will take as input
  static constexpr auto pll_source = bit_mask::from<16>();
  /// Sets the clock divider for the ADC peripherals
  static constexpr auto adc_divider = bit_mask::from<14, 15>();
  /// Sets the divider for peripherals on the APB2 bus
  static constexpr auto apb_2_divider = bit_mask::from<11, 13>();
  /// Sets the divider for peripherals on the APB1 bus
  static constexpr auto apb_1_divider = bit_mask::from<8, 10>();
  /// Sets the divider for peripherals on the AHB bus
  static constexpr auto ahb_divider = bit_mask::from<4, 7>();
  /// Used to check if the system clock has taken the new system clock
  /// settings.
  static constexpr auto system_clock_status = bit_mask::from<2, 3>();
  /// Set which clock will be used for the system clock.
  static constexpr auto system_clock_select = bit_mask::from<0, 1>();
};

/// Bit masks for the CR register
struct clock_control
{
  /// Indicates if the PLL is enabled and ready
  static constexpr auto pll_ready = bit_mask::from<25>();
  /// Used to enable the PLL
  static constexpr auto pll_enable = bit_mask::from<24>();
  /// Indicates if the external oscillator is ready for use
  static constexpr auto external_osc_ready = bit_mask::from<17>();
  /// Used to enable the external oscillator
  static constexpr auto external_osc_enable = bit_mask::from<16>();
};

/// Bitmasks for the BDCR register
struct rtc_register
{
  /// Will reset all clock states for the RTC
  static constexpr auto backup_domain_reset = bit_mask::from<16>();
  /// Enables the RTC clock
  static constexpr auto rtc_enable = bit_mask::from<15>();
  /// Selects the clock source for the RTC
  static constexpr auto rtc_source_select = bit_mask::from<8, 9>();
  /// Indicates if the LSE is ready for use
  static constexpr auto low_speed_osc_ready = bit_mask::from<1>();
  /// Used to enable the LSE
  static constexpr auto low_speed_osc_enable = bit_mask::from<0>();
};

/// Information about where a peripheral's enable/reset bit lives
struct rcc_register_info
{
  u32 volatile* reg;
  hal::bit_mask mask;
};

rcc_register_info get_enable_register_info(peripheral p_peripheral)
{
  auto const peripheral_value = hal::value(p_peripheral);
  auto const bus_number = peripheral_value / bus_id_offset;
  auto const mask = bit_mask::from(peripheral_value % bus_id_offset);
  switch (bus_number) {
    case 0:
      return { .reg = &rcc->ahbenr, .mask = mask };
    case 1:
      return { .reg = &rcc->apb1enr, .mask = mask };
    case 2:
      return { .reg = &rcc->apb2enr, .mask = mask };
    default:
      throw hal::argument_out_of_domain(nullptr);
  }
}

rcc_register_info get_reset_register_info(peripheral p_peripheral)
{
  auto const peripheral_value = hal::value(p_peripheral);
  auto const bus_number = peripheral_value / bus_id_offset;
  auto const mask = bit_mask::from(peripheral_value % bus_id_offset);
  switch (bus_number) {
    case 0:
      return { .reg = &rcc->ahbrstr, .mask = mask };
    case 1:
      return { .reg = &rcc->apb1rstr, .mask = mask };
    case 2:
      [[fallthrough]];
    default:
      return { .reg = &rcc->apb2rstr, .mask = mask };
  }
}

/**
 * @brief Power on the peripheral
 *
 * This API also acts as a resource overlap detector. If this API is called
 * twice on the same peripheral, it will throw an exception. Only drivers
 * with control over the entire peripheral should call this API for their
 * respective peripheral. This allows this API to detect when two drivers
 * attempt to utilize the same resource.
 *
 * @throws hal::device_or_resource_busy - if the peripheral is already
 * powered on, constituting a violation of the 1 peripheral manager per
 * peripheral rule.
 * @throws hal::argument_out_of_domain - if the peripheral's value is
 * outside of the bounds of the enum class OR if there is no enable
 * register for that peripheral.
 */
export void power_on(peripheral p_peripheral)
{
  auto const info = get_enable_register_info(p_peripheral);

  if (hal::bit_extract(info.mask, *info.reg)) {
    throw hal::device_or_resource_busy(nullptr);
  }

  hal::bit_modify(*info.reg).set(info.mask);
}

/**
 * @brief Power off peripheral
 *
 * If the peripheral is already powered off, this does nothing.
 */
export void power_off(peripheral p_peripheral)
{
  auto const info = get_enable_register_info(p_peripheral);
  hal::bit_modify(*info.reg).clear(info.mask);
}

/**
 * @brief Check if the peripheral is powered on
 *
 * @return true - peripheral is on
 * @return false - peripheral is off
 */
export [[nodiscard]] bool is_on(peripheral p_peripheral)
{
  auto const info = get_enable_register_info(p_peripheral);
  return hal::bit_extract(info.mask, *info.reg);
}

/**
 * @brief Resets the peripheral
 *
 * This will reset all the peripheral's registers to their reset/default
 * values.
 */
export void reset_peripheral(peripheral p_peripheral)
{
  auto const info = get_reset_register_info(p_peripheral);
  hal::bit_modify(*info.reg).set(info.mask);
  hal::bit_modify(*info.reg).clear(info.mask);
}

/// Maximum length of a buffer that the stm32f1xxx series dma controller can
/// handle.
export constexpr u32 max_dma_length = 65'535;

/// Namespace for the DMA controller's register map and channel configuration
/// (CCR) bit masks.
namespace dma {
/// Declare this channel for Memory to memory mode
constexpr auto memory_to_memory = hal::bit_mask::from<14>();

/// Configure the channel priority for this channel.
/// 0b00: Low
/// 0b01: Medium
/// 0b10: High
/// 0b11: Very high
constexpr auto channel_priority = hal::bit_mask::from<12, 13>();

/// The size of each element of the memory.
/// 0b00: 8-bits
/// 0b01: 16-bits
/// 0b10: 32-bits
/// 0b11: Reserved
constexpr auto memory_size = hal::bit_mask::from<10, 11>();

/// The peripheral register size.
/// 0b00: 8-bits
/// 0b01: 16-bits
/// 0b10: 32-bits
/// 0b11: Reserved
constexpr auto peripheral_size = hal::bit_mask::from<8, 9>();

/// Activate memory increment mode, which will increment the memory address
/// with each transfer
constexpr auto memory_increment_enable = hal::bit_mask::from<7>();

/// Activate memory increment mode, which will increment the peripheral
/// address with each transfer
constexpr auto peripheral_increment_enable = hal::bit_mask::from<6>();

/// DMA will continuous load bytes into the buffer supplied in a circular
/// buffer manner.
constexpr auto circular_mode = hal::bit_mask::from<5>();

/// Data transfer direction
/// 0: Read from peripheral
/// 1: Read from memory
constexpr auto data_transfer_direction = hal::bit_mask::from<4>();

/// Enable interrupt on transfer error
constexpr auto transfer_error_interrupt_enable = hal::bit_mask::from<3>();

/// Enable interrupt on half of data transferred
constexpr auto half_transfer_interrupt_enable = hal::bit_mask::from<2>();

/// Enable interrupt on complete transfer
constexpr auto transfer_complete_interrupt_enable = hal::bit_mask::from<1>();

/// Enable this DMA channel
constexpr auto enable = hal::bit_mask::from<0>();

/// Register map for a single DMA channel
struct dma_channel_t
{
  u32 volatile configuration;
  u32 volatile transfer_amount;
  u32 volatile peripheral_address;
  u32 volatile memory_address;
  u32 volatile reserved;
};

/// Register map for the DMA controller
struct dma_t
{
  u32 volatile interrupt_status;
  u32 volatile interrupt_flag_clear;
  std::array<dma_channel_t, 7> channel;
};

/// DMA1 controller address
constexpr auto dma1_addr = static_cast<uptr>(0x4002'0000);
/// DMA2 controller address
constexpr auto dma2_addr = static_cast<uptr>(0x4002'0400);
// NOLINTBEGIN(performance-no-int-to-ptr)
/// DMA1 controller register
inline auto* dma1 = reinterpret_cast<dma_t*>(dma1_addr);
/// DMA2 controller register
inline auto* dma2 = reinterpret_cast<dma_t*>(dma2_addr);
// NOLINTEND(performance-no-int-to-ptr)
}  // namespace dma
}  // namespace hal::stm32f1
