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

#include <libhal-stm32f1/pin.hpp>

#include <cstdint>

#include <libhal-util/bit.hpp>
#include <libhal-util/enum.hpp>

#include "pin.hpp"
#include "power.hpp"
#include "rcc_reg.hpp"

namespace hal::stm32f1 {
namespace {
/// Returns a bit mask indicating where the config bits are in the config
/// registers.
bit_mask mask(std::uint8_t p_pin)
{
  return {
    .position = static_cast<uint32_t>((p_pin * 4) % 32),
    .width = 4,
  };
}

/// Returns the configuration control register for the specific pin.
/// Pins 0 - 7 are in CRL and Pins 8 - 15 are in CRH.
uint32_t volatile& config_register(pin_select_t const& p_pin_select)
{
  if (p_pin_select.pin <= 7) {
    return gpio(p_pin_select.port).crl;
  }
  return gpio(p_pin_select.port).crh;
}
}  // namespace

gpio_t& gpio(std::uint8_t p_port)
{
  static gpio_t out_of_bounds_result{};

  switch (p_port) {
    case 'A':
      return *gpio_a_reg;
    case 'B':
      return *gpio_b_reg;
    case 'C':
      return *gpio_c_reg;
    case 'D':
      return *gpio_d_reg;
    case 'E':
      return *gpio_e_reg;
    case 'F':
      return *gpio_f_reg;
    case 'G':
      return *gpio_g_reg;
    default:
      return out_of_bounds_result;
  }
};

void configure_pin(pin_select_t p_pin_select, pin_config_t p_config)
{
  constexpr auto cnf1 = bit_mask::from<3>();
  constexpr auto cnf0 = bit_mask::from<2>();
  constexpr auto mode = bit_mask::from<0, 1>();

  // Ensure that AFIO is powered on before attempting to access it
  power_on(peripheral::afio);

  switch (p_pin_select.port) {
    case 'A':
      power_on(peripheral::gpio_a);
      break;
    case 'B':
      power_on(peripheral::gpio_b);
      break;
    case 'C':
      power_on(peripheral::gpio_c);
      break;
    case 'D':
      power_on(peripheral::gpio_d);
      break;
    case 'E':
      power_on(peripheral::gpio_e);
      break;
    default:
      hal::safe_throw(hal::argument_out_of_domain(nullptr));
  }

  auto config = bit_value<std::uint32_t>(0)
                  .insert<cnf1>(p_config.CNF1)
                  .insert<cnf0>(p_config.CNF0)
                  .insert<mode>(p_config.MODE)
                  .get();

  config_register(p_pin_select) = bit_modify(config_register(p_pin_select))
                                    .insert(mask(p_pin_select.pin), config)
                                    .to<std::uint32_t>();
}

void release_jtag_pins()
{
  // Ensure that AFIO is powered on before attempting to access it
  power_on(peripheral::afio);
  // Set the JTAG Release code
  bit_modify(alternative_function_io->mapr)
    .insert<bit_mask::from<24, 26>()>(0b010U);
}

void activate_mco_pa8(mco_source p_source)
{
  configure_pin({ .port = 'A', .pin = 8 }, push_pull_alternative_output);
  bit_modify(rcc->cfgr).insert<clock_configuration::mco>(value(p_source));
}

void remap_pins(can_pins p_pin_select)
{
  constexpr auto can_pin_remap = bit_mask::from<14, 13>();
  bit_modify(alternative_function_io->mapr)
    .insert<can_pin_remap>(value(p_pin_select));
}

}  // namespace hal::stm32f1
