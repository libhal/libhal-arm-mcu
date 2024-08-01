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

#include <libhal-arm-mcu/stm32f411/constants.hpp>

#include "power.hpp"
#include "rcc_reg.hpp"

namespace hal::stm32f411 {
namespace {
uint32_t volatile* enable(uint32_t p_bus_index)
{
  switch (p_bus_index) {
    case 0:
      return &rcc->ahb1enr;
    case 1:
      return &rcc->ahb2enr;
    case 2:
      [[fallthrough]];
    case 3:
      return &rcc->apb1enr;
    case 4:
      return &rcc->apb2enr;
    case 5:
      [[fallthrough]];
    default:
      return nullptr;
  }
}
}  // namespace

power::power(peripheral p_peripheral)
{
  auto peripheral_value = hal::value(p_peripheral);
  auto bus_number = peripheral_value / bus_id_offset;

  m_bit_position = static_cast<std::uint8_t>(peripheral_value % bus_id_offset);
  m_enable_register = enable(static_cast<uint32_t>(bus_number));
}

void power::on()
{
  if (m_enable_register) {
    hal::bit_modify(*m_enable_register).set(bit_mask::from(m_bit_position));
  }
}

bool power::is_on()
{
  if (m_enable_register) {
    return hal::bit_extract(bit_mask::from(m_bit_position), *m_enable_register);
  }
  return true;
}

void power::off()
{
  if (m_enable_register) {
    hal::bit_modify(*m_enable_register).clear(bit_mask::from(m_bit_position));
  }
}
}  // namespace hal::stm32f411