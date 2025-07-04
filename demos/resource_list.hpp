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

#include <optional>

#include <libhal/adc.hpp>
#include <libhal/can.hpp>
#include <libhal/dac.hpp>
#include <libhal/functional.hpp>
#include <libhal/i2c.hpp>
#include <libhal/input_pin.hpp>
#include <libhal/interrupt_pin.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/pwm.hpp>
#include <libhal/serial.hpp>
#include <libhal/spi.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/stream_dac.hpp>
#include <libhal/timer.hpp>
#include <libhal/zero_copy_serial.hpp>
#pragma once

#include <libhal/units.hpp>

namespace hal {

class watchdog
{
public:
  void start();
  void reset();
  void set_countdown_time(hal::time_duration p_wait_time);
  bool check_flag();
  void clear_flag();
  virtual ~watchdog() = default;
};

}  // namespace hal

struct resource_list
{
  hal::callback<void()> reset;
  // Each resource is made optional because some mcus do not support all
  // possible drivers in the resource list. If an application needs a driver it
  // will access them via `std::optional::value()` which will throw an exception
  // if the value, is not present. That exception will be caught in main and a
  // message will be printed if the `console` field has been set, then call
  // std::terminate.
  std::optional<hal::serial*> console;
  std::optional<hal::zero_copy_serial*> zero_copy_serial;
  std::optional<hal::output_pin*> status_led;
  std::optional<hal::steady_clock*> clock;
  std::optional<hal::can_transceiver*> can_transceiver;
  std::optional<hal::can_bus_manager*> can_bus_manager;
  std::optional<hal::can_interrupt*> can_interrupt;
  std::optional<hal::adc*> adc;
  std::optional<hal::input_pin*> input_pin;
  std::optional<hal::i2c*> i2c;
  std::optional<hal::interrupt_pin*> interrupt_pin;
  std::optional<hal::pwm*> pwm;
  std::optional<hal::timer*> callback_timer;
  std::optional<hal::pwm16_channel*> pwm_channel;
  std::optional<hal::pwm_group_manager*> pwm_frequency;
  std::optional<hal::spi*> spi;
  std::optional<hal::output_pin*> spi_chip_select;
  std::optional<hal::stream_dac_u8*> stream_dac;
  std::optional<hal::dac*> dac;
  std::optional<hal::watchdog*> watchdog;
};

// Each application file should have this function implemented
void application(resource_list& p_map);

// Each platform file should have this function implemented
void initialize_platform(resource_list& p_resources);
