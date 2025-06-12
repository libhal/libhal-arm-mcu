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

#include <array>

#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal-arm-mcu/stm32f1/pin.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/enum.hpp>
#include <libhal/error.hpp>

namespace hal::stm32f1 {

struct alternative_function_io_t
{
  u32 volatile evcr;
  u32 volatile mapr;
  std::array<u32, 4> volatile exticr;
  u32 reserved0;
  u32 volatile mapr2;
};

/**
 * @brief GPIO register map
 *
 */
struct gpio_t
{
  u32 volatile crl;
  u32 volatile crh;
  u32 volatile idr;
  u32 volatile odr;
  u32 volatile bsrr;
  u32 volatile brr;
  u32 volatile lckr;
};

/**
 * @brief Map the CONFIG flags for each pin use case
 *
 */
struct pin_config_t
{
  /// Configuration bit 1
  u8 CNF1;
  /// Configuration bit 0
  u8 CNF0;
  /// Mode bits
  u8 MODE;
  /// Output data register
  u8 PxODR;
};

static constexpr pin_config_t push_pull_gpio_output = {
  .CNF1 = 0,
  .CNF0 = 0,
  .MODE = 0b11,  // Default to high speed 50 MHz
  .PxODR = 0b0,  // Default to 0 LOW Voltage
};

static constexpr pin_config_t open_drain_gpio_output = {
  .CNF1 = 0,
  .CNF0 = 1,
  .MODE = 0b11,  // Default to high speed 50 MHz
  .PxODR = 0b0,  // Default to 0 LOW Voltage
};

static constexpr pin_config_t push_pull_alternative_output = {
  .CNF1 = 1,
  .CNF0 = 0,
  .MODE = 0b11,  // Default to high speed 50 MHz
  .PxODR = 0b0,  // Default to 0 LOW Voltage
};

static constexpr pin_config_t open_drain_alternative_output = {
  .CNF1 = 1,
  .CNF0 = 1,
  .MODE = 0b11,  // Default to high speed 50 MHz
  .PxODR = 0b0,  // Default to 0 LOW Voltage
};

static constexpr pin_config_t input_analog = {
  .CNF1 = 0,
  .CNF0 = 0,
  .MODE = 0b00,
  .PxODR = 0b0,  // Don't care
};

static constexpr pin_config_t input_float = {
  .CNF1 = 0,
  .CNF0 = 1,
  .MODE = 0b00,
  .PxODR = 0b0,  // Don't care
};

static constexpr pin_config_t input_pull_down = {
  .CNF1 = 1,
  .CNF0 = 0,
  .MODE = 0b00,
  .PxODR = 0b0,  // Pull Down
};

static constexpr pin_config_t input_pull_up = {
  .CNF1 = 1,
  .CNF0 = 0,
  .MODE = 0b00,
  .PxODR = 0b1,  // Pull Up
};

constexpr auto cnf1 = bit_mask::from<3>();
constexpr auto cnf0 = bit_mask::from<2>();
constexpr auto mode = bit_mask::from<0, 1>();

/**
 * @brief This is the default state of each pin when the stm32f1 resets
 *
 * This is used to determine if a pin is in the reset state prior to
 * configuration. This is also used to set the pin state when `reset_pin` is
 * called.
 */
static constexpr auto reset_pin_config = bit_value<u32>(0)
                                           .insert<cnf1>(input_float.CNF1)
                                           .insert<cnf0>(input_float.CNF0)
                                           .insert<mode>(input_float.MODE)
                                           .get();

/**
 * @brief Construct pin manipulation object
 *
 * @param p_pin_select - the pin to configure
 * @param p_config - Configuration to set the pin to
 * @throw hal::argument_out_of_domain - pin select is outside of the range of
 * available pins.
 * @throw hal::device_or_resource_busy - pin has already been configured once
 * from its reset state and thus is in use by something else in the code.
 */
void configure_pin(pin_select p_pin_select, pin_config_t p_config);

/**
 * @brief Set pin to the system reset state
 *
 * This releases control over the pin and allows the pin to be reused by other
 * drivers.
 *
 * @param p_pin_select - the pin to configure
 * @param p_config - Configuration to set the pin to
 */
void reset_pin(pin_select p_pin_select);

/**
 * @brief Throws an exception if a pin is not available
 *
 * Use this function to validate if a pin is available.
 *
 * @param p_pin_select - the pin to validate
 * @throw hal::device_or_resource_busy - if the pin is not available, meaning it
 * was not in the reset state.
 */
void throw_if_pin_is_unavailable(pin_select p_pin_select);

/**
 * @brief Remap can pins
 *
 * @param p_pin_select - pair of pins to select
 */
void remap_pins(can_pins p_pin_select);

/**
 * @brief Returns the gpio register based on the port
 *
 * @param p_port - port letter, must be from 'A' to 'G'
 * @return gpio_t& - gpio register map
 */
gpio_t& gpio_reg(u8 p_port);

constexpr auto gpio_peripheral_offset(peripheral p_peripheral)
{
  // The numeric value of `peripheral::gpio_a` to ``peripheral::gpio_g` are
  // contiguous in numeric value thus we can map letters 'A' to 'G' by doing
  // this math here.
  return value(p_peripheral) - value(peripheral::gpio_a);
}

inline auto* alternative_function_io =
  reinterpret_cast<alternative_function_io_t*>(0x4001'0000);

struct pin_remap
{
  static constexpr auto adc2_etrgreg_remap = hal::bit_mask::from<20>();
  static constexpr auto adc2_etrginj_remap = hal::bit_mask::from<19>();
  static constexpr auto adc1_etrgreg_remap = hal::bit_mask::from<18>();
  static constexpr auto adc1_etrginj_remap = hal::bit_mask::from<17>();
  static constexpr auto tim5ch4_iremap = hal::bit_mask::from<16>();
  static constexpr auto pd01_remap = hal::bit_mask::from<15>();
  static constexpr auto can1_remap = hal::bit_mask::from<13, 14>();
  static constexpr auto tim4_rempap = hal::bit_mask::from<12>();
  static constexpr auto tim3_rempap = hal::bit_mask::from<10, 11>();
  static constexpr auto tim2_rempap = hal::bit_mask::from<8, 9>();
  static constexpr auto tim1_rempap = hal::bit_mask::from<6, 7>();
  static constexpr auto usart3_remap = hal::bit_mask::from<4, 5>();
  static constexpr auto usart2_remap = hal::bit_mask::from<3>();
  static constexpr auto usart1_remap = hal::bit_mask::from<2>();
  static constexpr auto i2c1_remap = hal::bit_mask::from<1>();
  static constexpr auto spi1_remap = hal::bit_mask::from<0>();
};

struct pin_remap2
{
  static constexpr auto ptp_pps_remap = hal::bit_mask::from<30>();
  static constexpr auto tim2itr1_iremap = hal::bit_mask::from<29>();
  static constexpr auto spi3_remap = hal::bit_mask::from<28>();
  static constexpr auto swj_cfg = hal::bit_mask::from<26, 24>();
  static constexpr auto mii_rmii_sel = hal::bit_mask::from<23>();
  static constexpr auto can2_remap = hal::bit_mask::from<22>();
  static constexpr auto eth_remap = hal::bit_mask::from<21>();

  static constexpr auto tim5ch4_iremap = hal::bit_mask::from<16>();
  static constexpr auto pd01_remap = hal::bit_mask::from<15>();
  static constexpr auto can1_remap = hal::bit_mask::from<14, 13>();
  static constexpr auto tim4_rempap = hal::bit_mask::from<12>();
  static constexpr auto tim3_rempap = hal::bit_mask::from<11, 10>();
  static constexpr auto tim2_rempap = hal::bit_mask::from<9, 8>();
  static constexpr auto tim1_rempap = hal::bit_mask::from<7, 6>();
  static constexpr auto usart3_remap = hal::bit_mask::from<5, 4>();
  static constexpr auto usart2_remap = hal::bit_mask::from<3>();
  static constexpr auto usart1_remap = hal::bit_mask::from<2>();
  static constexpr auto i2c1_remap = hal::bit_mask::from<1>();
  static constexpr auto spi1_remap = hal::bit_mask::from<0>();
};
}  // namespace hal::stm32f1
