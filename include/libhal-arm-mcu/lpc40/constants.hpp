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

#pragma once

#include <libhal-armcortex/interrupt.hpp>
#include <libhal/units.hpp>

/**
 * @brief libhal drivers for the lpc40 series of microcontrollers from NXP
 *
 */
namespace hal::lpc40 {
/// List of each peripheral and their power on id number for this platform
enum class peripheral : u8
{
  lcd = 0,
  timer0 = 1,
  timer1 = 2,
  uart0 = 3,
  uart1 = 4,
  pwm0 = 5,
  pwm1 = 6,
  i2c0 = 7,
  uart4 = 8,
  rtc = 9,
  ssp1 = 10,
  emc = 11,
  adc = 12,
  can1 = 13,
  can2 = 14,
  gpio = 15,
  spifi = 16,
  motor_control_pwm = 17,
  quadrature_encoder = 18,
  i2c1 = 19,
  ssp2 = 20,
  ssp0 = 21,
  timer2 = 22,
  timer3 = 23,
  uart2 = 24,
  uart3 = 25,
  i2c2 = 26,
  i2s = 27,
  sdcard = 28,
  gpdma = 29,
  ethernet = 30,
  usb = 31,
  cpu,  // always on
  dac,  // always on
};

/// List of interrupt request numbers for this platform
enum class irq : cortex_m::irq_t  // NOLINT(performance-enum-size)
{
  /// Watchdog Timer Interrupt
  wdt = 0,
  /// Timer0 Interrupt
  timer0 = 1,
  /// Timer1 Interrupt
  timer1 = 2,
  /// Timer2 Interrupt
  timer2 = 3,
  /// Timer3 Interrupt
  timer3 = 4,
  /// UART0 Interrupt
  uart0 = 5,
  /// UART1 Interrupt
  uart1 = 6,
  /// UART2 Interrupt
  uart2 = 7,
  /// UART3 Interrupt
  uart3 = 8,
  /// PWM1 Interrupt
  pwm1 = 9,
  /// I2C0 Interrupt
  i2c0 = 10,
  /// I2C1 Interrupt
  i2c1 = 11,
  /// I2C2 Interrupt
  i2c2 = 12,
  /// Reserved
  reserved0 = 13,
  /// SSP0 Interrupt
  ssp0 = 14,
  /// SSP1 Interrupt
  ssp1 = 15,
  /// PLL0 Lock (Main PLL) Interrupt
  pll0 = 16,
  /// Real Time Clock Interrupt
  rtc = 17,
  /// External Interrupt 0 Interrupt
  eint0 = 18,
  /// External Interrupt 1 Interrupt
  eint1 = 19,
  /// External Interrupt 2 Interrupt
  eint2 = 20,
  /// External Interrupt 3 Interrupt
  eint3 = 21,
  /// A/D Converter Interrupt
  adc = 22,
  /// Brown-Out Detect Interrupt
  bod = 23,
  /// USB Interrupt
  usb = 24,
  /// CAN Interrupt
  can = 25,
  /// General Purpose DMA Interrupt
  dma = 26,
  /// I2S Interrupt
  i2s = 27,
  /// Ethernet Interrupt
  enet = 28,
  /// SD/MMC card I/F Interrupt
  mci = 29,
  /// Motor Control PWM Interrupt
  mcpwm = 30,
  /// Quadrature Encoder Interface Interrupt
  qei = 31,
  /// PLL1 Lock (USB PLL) Interrupt
  pll1 = 32,
  /// USB Activity interrupt
  usbactivity = 33,
  /// CAN Activity interrupt
  canactivity = 34,
  /// UART4 Interrupt
  uart4 = 35,
  /// SSP2 Interrupt
  ssp2 = 36,
  /// LCD Interrupt
  lcd = 37,
  /// GPIO Interrupt
  gpio = 38,
  ///  PWM0 Interrupt
  pwm0 = 39,
  ///  EEPROM Interrupt
  eeprom = 40,
  ///  CMP0 Interrupt
  cmp0 = 41,
  ///  CMP1 Interrupt
  cmp1 = 42,
  max,
};
}  // namespace hal::lpc40
