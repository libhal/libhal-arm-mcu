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

#include <libhal-util/bit.hpp>
#include <libhal-util/enum.hpp>

#include <libhal-stm32f1/constants.hpp>

#include "power.hpp"
#include "rcc_reg.hpp"

namespace hal::stm32f1 {
namespace {
uint32_t volatile* get_enable_register(std::uint32_t p_bus_index)
{
  switch (p_bus_index) {
    case 0:
      return &rcc->ahbenr;
    case 1:
      return &rcc->apb1enr;
    case 2:
      return &rcc->apb2enr;
    default:
      return nullptr;
  }
}
}  // namespace

void power_on(peripheral p_peripheral)
{
  auto peripheral_value = hal::value(p_peripheral);
  auto bus_number = peripheral_value / bus_id_offset;

  auto bit_position =
    static_cast<std::uint8_t>(peripheral_value % bus_id_offset);
  auto enable_register = get_enable_register(bus_number);

  if (enable_register) {
    hal::bit_modify(*enable_register).set(bit_mask::from(bit_position));
  }
}

void power_off(peripheral p_peripheral)
{
  auto peripheral_value = hal::value(p_peripheral);
  auto bus_number = peripheral_value / bus_id_offset;

  auto bit_position =
    static_cast<std::uint8_t>(peripheral_value % bus_id_offset);
  auto enable_register = get_enable_register(bus_number);
  if (enable_register) {
    hal::bit_modify(*enable_register).clear(bit_mask::from(bit_position));
  }
}

bool is_on(peripheral p_peripheral)
{
  auto peripheral_value = hal::value(p_peripheral);
  auto bus_number = peripheral_value / bus_id_offset;

  auto bit_position =
    static_cast<std::uint8_t>(peripheral_value % bus_id_offset);
  auto enable_register = get_enable_register(bus_number);

  if (enable_register) {
    return hal::bit_extract(bit_mask::from(bit_position), *enable_register);
  }
  return true;
}
}  // namespace hal::stm32f1