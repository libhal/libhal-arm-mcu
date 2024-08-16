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

#include <libhal/can.hpp>

#include "pin.hpp"

namespace hal::stm32f1 {
class can final : public hal::can
{
public:
  can(can::settings const& p_settings = {},
      can_pins p_pins = can_pins::pa11_pa12);
  void enable_self_test(bool p_enable);
  ~can() override;

private:
  void driver_configure(settings const& p_settings) override;
  void driver_bus_on() override;
  void driver_send(message_t const& p_message) override;
  void driver_on_receive(hal::callback<handler> p_handler) override;
};
}  // namespace hal::stm32f1
