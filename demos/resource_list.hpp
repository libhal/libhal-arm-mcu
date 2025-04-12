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

#include <memory>

#include <libhal/adc.hpp>
#include <libhal/can.hpp>
#include <libhal/dac.hpp>
#include <libhal/error.hpp>
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
#include <libhal/zero_copy_serial.hpp>

struct resource_list
{
  hal::callback<void()> reset;
  // Each resource is made optional because some mcus do not support all
  // possible drivers in the resource list. If an application needs a driver it
  // will access them via `std::optional::value()` which will throw an exception
  // if the value, is not present. That exception will be caught in main and a
  // message will be printed if the `console` field has been set, then call
  // std::terminate.
  std::shared_ptr<hal::serial> console;
  std::shared_ptr<hal::zero_copy_serial> zero_copy_serial;
  std::shared_ptr<hal::output_pin> status_led;
  std::shared_ptr<hal::steady_clock> clock;
  std::shared_ptr<hal::can_transceiver> can_transceiver;
  std::shared_ptr<hal::can_mask_filter> can_mask_filter;
  std::shared_ptr<hal::can_bus_manager> can_bus_manager;
  std::shared_ptr<hal::can_interrupt> can_interrupt;
  std::shared_ptr<hal::adc> adc;
  std::shared_ptr<hal::input_pin> input_pin;
  std::shared_ptr<hal::i2c> i2c;
  std::shared_ptr<hal::interrupt_pin> interrupt_pin;
  std::shared_ptr<hal::pwm> pwm;
  std::shared_ptr<hal::pwm16_channel> pwm_channel;
  std::shared_ptr<hal::pwm_group_manager> pwm_frequency;
  std::shared_ptr<hal::spi> spi;
  std::shared_ptr<hal::output_pin> spi_chip_select;
  std::shared_ptr<hal::stream_dac_u8> stream_dac;
  std::shared_ptr<hal::dac> dac;
};

namespace hal {
// class empty
}

template<class T>
void resource_contract_assert(std::shared_ptr<T> p_object)
{
  if (not p_object) {
    hal::safe_throw(hal::unknown(nullptr));
  }
}

// Each application file should have this function implemented
void application(resource_list& p_map);

// Each platform file should have this function implemented
void initialize_platform(resource_list& p_resources);
