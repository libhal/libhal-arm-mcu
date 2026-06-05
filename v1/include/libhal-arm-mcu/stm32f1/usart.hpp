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

#include <span>

#include <libhal-arm-mcu/stm32f1/pin.hpp>
#include <libhal/initializers.hpp>
#include <libhal/units.hpp>
#include <libhal/zero_copy_serial.hpp>

#include "constants.hpp"

namespace hal::stm32f1 {

class usart_manager
{
public:
  usart_manager(usart_manager const&) = delete;
  usart_manager& operator=(usart_manager const&) = delete;
  usart_manager(usart_manager&&) noexcept = default;
  usart_manager& operator=(usart_manager&&) noexcept = default;
  ~usart_manager();

  class serial;

  serial acquire_serial(std::span<byte> p_buffer,
                        hal::serial::settings const& p_settings = {});

  auto acquire_serial(hal::buffer_param auto p_buffer,
                      hal::serial::settings const& p_settings = {})
  {
    return acquire_serial(hal::create_unique_static_buffer(p_buffer),
                          p_settings);
  }

  // NOTE: Move these out when they are available.
#if 0
  class sync_serial;
  class half_duplex_serial;
  class irda;
  class lin;
  sync_serial acquire_sync_serial();
  half_duplex_serial acquire_half_duplex_serial();
  irda acquire_irda();
  lin acquire_lin();
#endif

protected:
  usart_manager(peripheral p_id);

  uptr m_reg;
  peripheral m_id;
};

template<peripheral id>
class usart final : public usart_manager
{
public:
  static_assert(id == peripheral::usart1 or id == peripheral::usart2 or
                  id == peripheral::usart3,
                "Only peripherals usart1, usart2, usart3 are allowed! Support "
                "for uart4, and uart5 not available currently.");

  usart()
    : usart_manager(id)
  {
  }
};

class usart_manager::serial final : public hal::zero_copy_serial
{
public:
  template<peripheral id>
  serial(usart<id>& p_usart_manager,
         std::span<byte> p_buffer,
         hal::serial::settings const& p_settings = {})
    : serial(p_usart_manager, p_buffer, p_settings)
  {
  }

  serial(serial const&) = delete;
  serial& operator=(serial const&) = delete;
  serial(serial&&) noexcept = default;
  serial& operator=(serial&&) noexcept = default;
  ~serial() override;

private:
  friend class usart_manager;

  serial(usart_manager& p_usart_manager,
         std::span<byte> p_buffer,
         hal::serial::settings const& p_settings = {});
  void driver_configure(hal::serial::settings const& p_settings) override;
  void driver_write(std::span<hal::byte const> p_data) override;
  std::span<hal::byte const> driver_receive_buffer() override;
  std::size_t driver_cursor() override;

  usart_manager* m_usart_manager;
  std::span<byte> m_buffer;
  u8 m_dma_channel;
  pin_select m_tx;
  pin_select m_rx;
};
}  // namespace hal::stm32f1
