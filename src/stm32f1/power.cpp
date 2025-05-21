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

#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/enum.hpp>
#include <libhal/error.hpp>

#include "power.hpp"
#include "rcc_reg.hpp"

namespace hal::stm32f1 {
namespace {
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
      hal::safe_throw(hal::argument_out_of_domain(nullptr));
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
}  // namespace

void power_on(peripheral p_peripheral)
{
  auto const info = get_enable_register_info(p_peripheral);

  if (hal::bit_extract(info.mask, *info.reg)) {
    hal::safe_throw(hal::device_or_resource_busy(nullptr));
  }

  hal::bit_modify(*info.reg).set(info.mask);
}

void power_off(peripheral p_peripheral)
{
  auto const info = get_enable_register_info(p_peripheral);
  hal::bit_modify(*info.reg).clear(info.mask);
}

bool is_on(peripheral p_peripheral)
{
  auto const info = get_enable_register_info(p_peripheral);
  return hal::bit_extract(info.mask, *info.reg);
}

void reset_peripheral(peripheral p_peripheral)
{
  auto const info = get_reset_register_info(p_peripheral);
  hal::bit_modify(*info.reg).set(info.mask);
  hal::bit_modify(*info.reg).clear(info.mask);
}

}  // namespace hal::stm32f1
