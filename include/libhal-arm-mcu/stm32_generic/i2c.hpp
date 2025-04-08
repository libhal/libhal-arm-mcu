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

#include <cstdint>
#include <libhal/output_pin.hpp>
#include <span>

#include <libhal/i2c.hpp>
#include <libhal/initializers.hpp>
#include <libhal/io_waiter.hpp>

namespace hal::stm32_generic {
class i2c
{
public:
  i2c(void* p_i2c, hal::io_waiter& p_waiter);
  i2c();
  void transaction(hal::byte p_address,
                   std::span<hal::byte const> p_data_out,
                   std::span<hal::byte> p_data_in,
                   hal::function_ref<hal::timeout_function> p_timeout);
  void configure(hal::i2c::settings const& p_settings, hertz p_frequency);

  void clear_error_flag(hal::output_pin *sda, hal::output_pin *scl);

  void handle_i2c_event() noexcept;
  void handle_i2c_error() noexcept;
  ~i2c();
private:
  enum class error_state
  {
    no_error = 0,
    no_such_device,
    io_error,
    arbitration_lost,
  };

  std::span<hal::byte const> m_data_out;
  std::span<hal::byte> m_data_in;
  error_state m_status{};
  hal::byte m_address = hal::byte{ 0x00 };
  bool m_busy = false;
  bool m_transmitter = false;
  bool m_reciever = false;
  void* m_i2c;
  hal::io_waiter* m_waiter = nullptr;
};
}  // namespace hal::stm32_generic
