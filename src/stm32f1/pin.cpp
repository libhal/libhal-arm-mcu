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
#include <libhal/error.hpp>

#include "pin.hpp"
#include "power.hpp"
#include "rcc_reg.hpp"

namespace hal::stm32f1 {
namespace {
/// Returns a bit mask indicating where the config bits are in the config
/// registers.
bit_mask mask(u8 p_pin)
{
  return {
    .position = static_cast<uint32_t>((p_pin * 4) % 32),
    .width = 4,
  };
}

/// Returns the configuration control register for the specific pin.
/// Pins 0 - 7 are in CRL and Pins 8 - 15 are in CRH.
uint32_t volatile& config_register(pin_select const& p_pin_select)
{
  if (p_pin_select.pin <= 7) {
    return gpio_reg(p_pin_select.port).crl;
  }
  return gpio_reg(p_pin_select.port).crh;
}

void safely_power_on(pin_select const& p_pin_select)
{
  // Ensure that AFIO is powered on before attempting to access it
  if (not is_on(peripheral::afio)) {
    power_on(peripheral::afio);
  }

  switch (p_pin_select.port) {
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
    default:
      hal::safe_throw(hal::argument_out_of_domain(nullptr));
  }
}

std::array<gpio_t*, 8> gpio_reg_map{
  reinterpret_cast<gpio_t*>(0x4001'0800),  // 'A'
  reinterpret_cast<gpio_t*>(0x4001'0c00),  // 'B'
  reinterpret_cast<gpio_t*>(0x4001'1000),  // 'C'
  reinterpret_cast<gpio_t*>(0x4001'1400),  // 'D'
  reinterpret_cast<gpio_t*>(0x4001'1800),  // 'E'
  reinterpret_cast<gpio_t*>(0x4001'1c00),  // 'F'
  reinterpret_cast<gpio_t*>(0x4001'2000),  // 'G'
};

bool is_pin_reset(pin_select p_pin_select)
{
  auto& config_reg = config_register(p_pin_select);
  auto const current_configuration =
    bit_extract(mask(p_pin_select.pin), config_reg);

  return reset_pin_config == current_configuration;
}
}  // namespace

gpio_t& gpio_reg(u8 p_port)
{
  auto const offset = p_port - 'A';
  return *gpio_reg_map.at(offset);
}

void throw_if_pin_is_unavailable(pin_select p_pin_select)
{
  if (not is_pin_reset(p_pin_select)) {
    hal::safe_throw(hal::device_or_resource_busy(nullptr));
  }
}

void configure_pin(pin_select p_pin_select, pin_config_t p_config)
{
  auto& config_reg = config_register(p_pin_select);
  safely_power_on(p_pin_select);
  throw_if_pin_is_unavailable(p_pin_select);

  auto const config = bit_value<std::uint32_t>(0)
                        .insert<cnf1>(p_config.CNF1)
                        .insert<cnf0>(p_config.CNF0)
                        .insert<mode>(p_config.MODE)
                        .get();

  bit_modify(config_reg).insert(mask(p_pin_select.pin), config);
}

void reset_pin(pin_select p_pin_select)
{
  auto& config_reg = config_register(p_pin_select);
  config_reg = bit_modify(config_reg)
                 .insert(mask(p_pin_select.pin), reset_pin_config)
                 .to<std::uint32_t>();
}

void release_jtag_pins()
{
  // Ensure that AFIO is powered on before attempting to access it
  if (not is_on(peripheral::afio)) {
    power_on(peripheral::afio);
  }
  // Set the JTAG Release code
  bit_modify(alternative_function_io->mapr)
    .insert<bit_mask::from<24, 26>()>(0b010U);
}

void activate_mco_pa8(mco_source p_source)
{
  configure_pin({ .port = 'A', .pin = 8 }, push_pull_alternative_output);
  bit_modify(rcc->cfgr).insert<clock_configuration::mco>(value(p_source));
}

void reset_mco_pa8()
{
  reset_pin({ .port = 'A', .pin = 8 });
}

void remap_pins(can_pins p_pin_select)
{
  constexpr auto can_pin_remap = bit_mask::from<14, 13>();
  bit_modify(alternative_function_io->mapr)
    .insert<can_pin_remap>(value(p_pin_select));
}
}  // namespace hal::stm32f1
