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

#include <cstddef>
#include <cstdint>

#include <array>
#include <bitset>

export module hal.arm_mcu.stm32f1:pin;

import hal;
import hal.util;

import :constants;
import :power;
import :clock;

namespace hal::stm32f1 {
/**
 * @brief Structure to hold a port & pin selection
 *
 */
export struct pin
{
  /// @brief Port letter: must be a capitol letter from 'A' to 'G'
  u8 port;
  /// @brief Pin number: must be between 0 to 15
  u8 pin;
};

/**
 * @brief Main (Master) Clock Output options
 *
 * The following options are available to be sent to the output of the MCO
 * pin
 */
export enum class mco_source : std::uint8_t
{
  system_clock = 0b100,
  high_speed_internal = 0b101,
  high_speed_external = 0b110,
  pll_clock_divided_by_2 = 0b111,
};

/**
 * @brief Remap pins for can bus peripheral
 *
 */
export enum class can_pins : std::uint8_t
{
  pa11_pa12 = 0b00,
  pb9_pb8 = 0b10,
  pd0_pd1 = 0b11,
};

/// Alternate function I/O (AFIO) register map
struct alternative_function_io_t
{
  u32 volatile evcr;
  u32 volatile mapr;
  std::array<u32 volatile, 4> exticr;
  u32 reserved0;
  u32 volatile mapr2;
};

/// AFIO peripheral register
// NOLINTNEXTLINE(performance-no-int-to-ptr)
inline auto* alternative_function_io =
  reinterpret_cast<alternative_function_io_t*>(0x4001'0000);

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
export struct pin_config_t
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

/// Configuration for a push-pull GPIO output pin
export constexpr pin_config_t push_pull_gpio_output = {
  .CNF1 = 0,
  .CNF0 = 0,
  .MODE = 0b11,  // Default to high speed 50 MHz
  .PxODR = 0b0,  // Default to 0 LOW Voltage
};

/// Configuration for an open-drain GPIO output pin
export constexpr pin_config_t open_drain_gpio_output = {
  .CNF1 = 0,
  .CNF0 = 1,
  .MODE = 0b11,  // Default to high speed 50 MHz
  .PxODR = 0b0,  // Default to 0 LOW Voltage
};

/// Configuration for a push-pull alternative function output pin
export constexpr pin_config_t push_pull_alternative_output = {
  .CNF1 = 1,
  .CNF0 = 0,
  .MODE = 0b11,  // Default to high speed 50 MHz
  .PxODR = 0b0,  // Default to 0 LOW Voltage
};

/// Configuration for an open-drain alternative function output pin
export constexpr pin_config_t open_drain_alternative_output = {
  .CNF1 = 1,
  .CNF0 = 1,
  .MODE = 0b11,  // Default to high speed 50 MHz
  .PxODR = 0b0,  // Default to 0 LOW Voltage
};

/// Configuration for an analog input pin
export constexpr pin_config_t input_analog = {
  .CNF1 = 0,
  .CNF0 = 0,
  .MODE = 0b00,
  .PxODR = 0b0,  // Don't care
};

/// Configuration for a floating input pin
export constexpr pin_config_t input_float = {
  .CNF1 = 0,
  .CNF0 = 1,
  .MODE = 0b00,
  .PxODR = 0b0,  // Don't care
};

/// Configuration for an input pin with a pull down resistor
export constexpr pin_config_t input_pull_down = {
  .CNF1 = 1,
  .CNF0 = 0,
  .MODE = 0b00,
  .PxODR = 0b0,  // Pull Down
};

/// Configuration for an input pin with a pull up resistor
export constexpr pin_config_t input_pull_up = {
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
 * This is used to set the pin state when `reset_pin` is called.
 */
constexpr auto reset_pin_config = bit_value<u32>(0)
                                    .insert<cnf1>(input_float.CNF1)
                                    .insert<cnf0>(input_float.CNF0)
                                    .insert<mode>(input_float.MODE)
                                    .get();

/// Bit masks for the AFIO MAPR register
export struct pin_remap
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

/// Bit masks for the AFIO MAPR2 register
export struct pin_remap2
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

/// Returns a bit mask indicating where the config bits are in the config
/// registers.
bit_mask config_mask(u8 p_pin)
{
  return {
    .position = static_cast<std::uint32_t>((p_pin * 4) % 32),
    .width = 4,
  };
}

/// Returns a bit mask indicating where the odr bit is in the output data
/// register.
bit_mask odr_mask(u8 p_pin)
{
  return {
    .position = static_cast<std::uint32_t>(p_pin),
    .width = 1,
  };
}

std::array<gpio_t*, 8> gpio_reg_map{
  reinterpret_cast<gpio_t*>(0x4001'0800),  // 'A'
  reinterpret_cast<gpio_t*>(0x4001'0C00),  // 'B'
  reinterpret_cast<gpio_t*>(0x4001'1000),  // 'C'
  reinterpret_cast<gpio_t*>(0x4001'1400),  // 'D'
  reinterpret_cast<gpio_t*>(0x4001'1800),  // 'E'
  reinterpret_cast<gpio_t*>(0x4001'1c00),  // 'F'
  reinterpret_cast<gpio_t*>(0x4001'2000),  // 'G'
};

/**
 * @brief Returns the gpio register based on the port
 *
 * @param p_port - port letter, must be from 'A' to 'G'
 * @return gpio_t& - gpio register map
 */
gpio_t& gpio_reg(u8 p_port)
{
  auto const offset = p_port - 'A';
  return *gpio_reg_map.at(offset);
}

/// Returns the configuration control register for the specific pin.
/// Pins 0 - 7 are in CRL and Pins 8 - 15 are in CRH.
std::uint32_t volatile& config_register(pin const& p_pin)
{
  if (p_pin.pin <= 7) {
    return gpio_reg(p_pin.port).crl;
  }
  return gpio_reg(p_pin.port).crh;
}

/// Returns the output data register for the specific pin.
std::uint32_t volatile& odr_register(pin const& p_pin)
{
  return gpio_reg(p_pin.port).odr;
}

void safely_power_on(pin const& p_pin)
{
  // Ensure that AFIO is powered on before attempting to access it
  if (not is_on(peripheral::afio)) {
    power_on(peripheral::afio);
  }

  switch (p_pin.port) {
    case 'A':
      if (not is_on(peripheral::gpio_a)) {
        power_on(peripheral::gpio_a);
      }
      break;
    case 'B':
      if (not is_on(peripheral::gpio_b)) {
        power_on(peripheral::gpio_b);
      }
      break;
    case 'C':
      if (not is_on(peripheral::gpio_c)) {
        power_on(peripheral::gpio_c);
      }
      break;
    case 'D':
      if (not is_on(peripheral::gpio_d)) {
        power_on(peripheral::gpio_d);
      }
      break;
    case 'E':
      if (not is_on(peripheral::gpio_e)) {
        power_on(peripheral::gpio_e);
      }
      break;
    case 'F':
      if (not is_on(peripheral::gpio_f)) {
        power_on(peripheral::gpio_f);
      }
      break;
    case 'G':
      if (not is_on(peripheral::gpio_g)) {
        power_on(peripheral::gpio_g);
      }
      break;
    default:
      throw hal::argument_out_of_domain(nullptr);
  }
}

/**
 * @brief Make JTAG pins not associated with SWD available as IO
 *
 * The GPIO pins PB3, PB4, and PA15 are default initalized to be used for
 * JTAG purposes. If you are using SWD and want to use these pins as GPIO or
 * as other alternative functions, this function MUST be called.
 *
 */
export void release_jtag_pins()
{
  // Ensure that AFIO is powered on before attempting to access it
  if (not is_on(peripheral::afio)) {
    power_on(peripheral::afio);
  }
  // Set the JTAG Release code
  bit_modify(alternative_function_io->mapr)
    .insert<bit_mask::from<24, 26>()>(0b010U);
}

/// One bit per pin of ports A to G, set while a driver has claimed the pin.
constinit std::bitset<7 * 16> claimed_pins{};

/// @throws hal::argument_out_of_domain - if the port isn't 'A' to 'G' or the
/// pin isn't 0 to 15
std::size_t pin_index(pin p_pin)
{
  if (p_pin.port < 'A' || p_pin.port > 'G' || p_pin.pin > 15) {
    throw hal::argument_out_of_domain(nullptr);
  }
  return static_cast<std::size_t>(p_pin.port - 'A') * 16 + p_pin.pin;
}

/**
 * @brief Claim a pin for one driver's exclusive use
 *
 * Every driver that configures a pin claims it first, and releases it when
 * destroyed, so two drivers using the same pin are detected no matter which
 * is constructed first. Claims are tracked in software, so a pin a bootloader
 * left configured can still be claimed.
 *
 * @param p_pin - the pin to claim
 * @throws hal::device_or_resource_busy - if the pin is already claimed
 * @throws hal::argument_out_of_domain - if the port isn't 'A' to 'G' or the
 * pin isn't 0 to 15
 */
export void claim_pin(pin p_pin)
{
  auto const index = pin_index(p_pin);
  if (claimed_pins.test(index)) {
    throw hal::device_or_resource_busy(nullptr);
  }
  claimed_pins.set(index);
}

/**
 * @brief Release a pin claimed with claim_pin()
 *
 * Releasing a pin that isn't claimed, or that doesn't exist, does nothing.
 *
 * @param p_pin - the pin to release
 */
export void release_pin(pin p_pin) noexcept
{
  if (p_pin.port >= 'A' && p_pin.port <= 'G' && p_pin.pin <= 15) {
    claimed_pins.reset(pin_index(p_pin));
  }
}

/**
 * @brief Check if a pin is claimed
 *
 * @param p_pin - the pin to check
 * @return true - a driver has claimed the pin
 * @return false - the pin is free, or doesn't exist
 */
export [[nodiscard]] bool is_pin_claimed(pin p_pin) noexcept
{
  if (p_pin.port < 'A' || p_pin.port > 'G' || p_pin.pin > 15) {
    return false;
  }
  return claimed_pins.test(pin_index(p_pin));
}

/**
 * @brief Construct pin manipulation object
 *
 * @param p_pin - the pin to configure
 * @param p_config - Configuration to set the pin to
 * Claim the pin with claim_pin() before configuring it.
 *
 * @throw hal::argument_out_of_domain - pin select is outside of the range of
 * available pins.
 */
export void configure_pin(pin p_pin, pin_config_t p_config)
{
  // The GPIO pins PB3, PB4, and PA15 are default initalized to be used for
  // JTAG purposes. This releases them if they are being configured
  if ((p_pin.port == 'B' && p_pin.pin == 3) ||
      (p_pin.port == 'B' && p_pin.pin == 4) ||
      (p_pin.port == 'A' && p_pin.pin == 15)) {
    release_jtag_pins();
  }

  auto& config_reg = config_register(p_pin);
  auto& odr_reg = odr_register(p_pin);
  safely_power_on(p_pin);

  auto const config = bit_value<u32>(0)
                        .insert<cnf1>(p_config.CNF1)
                        .insert<cnf0>(p_config.CNF0)
                        .insert<mode>(p_config.MODE)
                        .get();

  bit_modify(config_reg).insert(config_mask(p_pin.pin), config);
  bit_modify(odr_reg).insert(odr_mask(p_pin.pin), p_config.PxODR);
}

/**
 * @brief Set pin to the system reset state
 *
 * This does not release the pin's claim; call release_pin() for that.
 *
 * @param p_pin - the pin to configure
 */
export void reset_pin(pin p_pin)
{
  auto& config_reg = config_register(p_pin);
  config_reg = bit_modify(config_reg)
                 .insert(config_mask(p_pin.pin), reset_pin_config)
                 .to<u32>();
}

/**
 * @brief Output a clock on the PA8 pin
 *
 * Claims PA8 until reset_mco_pa8() is called.
 *
 * @param p_source - source clock to channel to the PA8 pin
 * @throws hal::device_or_resource_busy - if PA8 is already claimed
 */
export void activate_mco_pa8(mco_source p_source)
{
  claim_pin({ .port = 'A', .pin = 8 });
  configure_pin({ .port = 'A', .pin = 8 }, push_pull_alternative_output);
  bit_modify(rcc->cfgr).insert<clock_configuration::mco>(value(p_source));
}

/**
 * @brief Reset pin back to default
 */
export void reset_mco_pa8()
{
  reset_pin({ .port = 'A', .pin = 8 });
  release_pin({ .port = 'A', .pin = 8 });
}

/**
 * @brief Remap can pins
 *
 * @param p_pin - pair of pins to select
 */
export void remap_pins(can_pins p_pin)
{
  constexpr auto can_pin_remap = bit_mask::from<14, 13>();
  bit_modify(alternative_function_io->mapr).insert<can_pin_remap>(value(p_pin));
}
}  // namespace hal::stm32f1
