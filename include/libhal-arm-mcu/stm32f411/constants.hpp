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

namespace hal::stm32f411 {

/// Number of bits between each enable register
static constexpr std::uint32_t bus_id_offset = 32;
/// Bit position of AHB1
static constexpr std::uint32_t ahb1_bus = bus_id_offset * 0;
/// Bit position of AHB2
static constexpr std::uint32_t ahb2_bus = bus_id_offset * 1;
/// Bit position of AHB3
static constexpr std::uint32_t ahb3_bus = bus_id_offset * 2;

/// Bit position of APB1
static constexpr std::uint32_t apb1_bus = bus_id_offset * 3;
/// Bit position of AHB2
static constexpr std::uint32_t apb2_bus = bus_id_offset * 4;

/// Bit position of outside any bus
static constexpr std::uint32_t beyond = bus_id_offset * 5;

/// List of each peripheral and their power on id number for this platform
enum class peripheral : std::uint32_t
{
  gpio_a = ahb1_bus + 0,
  gpio_b = ahb1_bus + 1,
  gpio_c = ahb1_bus + 2,
  gpio_d = ahb1_bus + 3,
  gpio_e = ahb1_bus + 4,
  gpio_h = ahb1_bus + 7,
  crc = ahb1_bus + 12,
  dma1 = ahb1_bus + 21,
  dma2 = ahb1_bus + 22,

  usb_otg = ahb2_bus + 7,

  timer2 = apb1_bus + 0,
  timer3 = apb1_bus + 1,
  timer4 = apb1_bus + 2,
  timer5 = apb1_bus + 3,
  window_watchdog = apb1_bus + 11,
  spi2 = apb1_bus + 14,
  spi3 = apb1_bus + 15,
  usart2 = apb1_bus + 17,
  i2c1 = apb1_bus + 21,
  i2c2 = apb1_bus + 22,
  i2c3 = apb1_bus + 23,
  power = apb1_bus + 28,

  timer1 = apb2_bus + 0,
  usart1 = apb2_bus + 4,
  usart6 = apb2_bus + 5,
  adc1 = apb2_bus + 8,
  sdio = apb2_bus + 11,
  spi1 = apb2_bus + 12,
  spi4 = apb2_bus + 13,
  system_config_controller = apb2_bus + 14,
  timer9 = apb2_bus + 16,
  timer10 = apb2_bus + 17,
  timer11 = apb2_bus + 18,
  spi5 = apb2_bus + 20,

  cpu = beyond + 0,
  system_timer = beyond + 1,
  i2s = beyond + 2
};

/// List of interrupt request numbers for this platform
enum class irq : std::uint16_t
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
  /// DMA1 Channel 0
  dma1_channel0 = 11,
  /// DMA1 Channel 1
  dma1_channel1 = 12,
  /// DMA1 Channel 2
  dma1_channel2 = 13,
  /// DMA1 Channel 3
  dma1_channel3 = 14,
  /// DMA1 Channel 4
  dma1_channel4 = 15,
  /// DMA1 Channel 5
  dma1_channel5 = 16,
  /// DMA1 Channel 6
  dma1_channel6 = 17,
  /// ADC1
  adc1 = 18,
  /// EXTI Line[9:5]
  exi9_5 = 23,
  /// TIM1 Break
  tim1_brk = 24,
  /// TIM1 Break and TIM19
  tim1_brk_tim19 = 24,
  /// TIM1 Update
  tim1_up = 25,
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
  /// External Line[15:10]
  exti15_10 = 40,
  /// EXTI Line17
  exti17 = 41,
  /// RTC Alarm through EXTI Line
  rtcalarm = 41,
  /// EXTI18 Line18
  exti18 = 42,
  /// USB OTG FS WakeUp
  otg_fs_wkup = 42,
  /// DMA1_Channel7
  dma1_channel7 = 47,
  /// SDIO
  sdio = 49,
  /// TIM5
  tim5 = 50,
  /// SPI3
  spi3 = 51,
  /// DMA2 Channel 0
  dma2_channel0 = 56,
  /// DMA2 Channel 1
  dma2_channel1 = 57,
  /// DMA2 Channel 2
  dma2_channel2 = 58,
  /// DMA2 Channel 3
  dma2_channel3 = 59,
  /// DMA2 Channel 4
  dma2_channel4 = 60,
  /// USB OTG FS
  otg_fs = 67,
  /// DMA2 Channel 5
  dma2_channel5 = 68,
  /// DMA2 Channel 6
  dma2_channel6 = 69,
  /// DMA2 Channel 7
  dma2_channel7 = 70,
  /// USART6
  usart6 = 71,
  /// I2C3 Event
  i2c3_ev = 72,
  /// I2C3 Error
  i2c3_er = 73,
  /// FPU
  fpu = 81,
  /// SPI4
  spi4 = 84,
  /// SPI5
  spi5 = 85,
  max,
};
}  // namespace hal::stm32f411
