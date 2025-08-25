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

#include <memory_resource>
#include <variant>

#include <libhal/functional.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/pointers.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>
#include <libhal/usb.hpp>

namespace hal::stm32f1 {

class usb
{
public:
  using ctrl_receive_tag = hal::v5::usb_control_endpoint::on_receive_tag;
  using out_receive_tag = hal::v5::usb_out_endpoint::on_receive_tag;
  using callback_variant_t = std::variant<hal::callback<void(ctrl_receive_tag)>,
                                          hal::callback<void(out_receive_tag)>>;

  static constexpr std::size_t usb_endpoint_count = 8;

  usb(hal::steady_clock& p_clock,
      hal::time_duration p_write_timeout = std::chrono::milliseconds(5));
  usb(usb&) = delete;
  usb operator=(usb&) = delete;
  usb(usb&&) = delete;
  usb operator=(usb&&) = delete;
  ~usb();

  bool disrupted();

  void interrupt_handler() noexcept;
  void flush_endpoint(u8 p_endpoint);
  void fill_endpoint(hal::u8 p_endpoint,
                     std::span<hal::byte const> p_data,
                     hal::u16 p_max_length);
  void write_to_endpoint(hal::u8 p_endpoint, std::span<hal::byte const> p_data);
  usize read_endpoint(u8 p_endpoint,
                      std::span<hal::byte> p_buffer,
                      u16& p_bytes_read);
  void wait_for_endpoint_transfer_completion(hal::u8 p_endpoint);
  void set_callback(hal::u8 p_endpoint, callback_variant_t const& p_callback);
  void reset(u8 p_endpoint_number);
  void zero_length_packet(u8 p_endpoint);

  // Starts at 1 because endpoint 0 is always occupied by the control endpoint
  hal::u8 m_endpoints_allocated = 1;

private:
  friend class control_endpoint;
  friend class interrupt_in_endpoint;
  friend class bulk_in_endpoint;
  friend class interrupt_out_endpoint;
  friend class bulk_out_endpoint;

  std::array<callback_variant_t, usb_endpoint_count> m_out_callbacks;
  hal::steady_clock* m_clock;
  hal::time_duration m_write_timeout;
  std::uint16_t m_available_endpoint_memory;
};

/**
 * @brief
 *
 * @param p_allocator
 * @param p_usb
 * @return hal::v5::strong_ptr<hal::v5::usb_control_endpoint>
 */
hal::v5::strong_ptr<hal::v5::usb_control_endpoint> acquire_usb_control_endpoint(
  std::pmr::polymorphic_allocator<> p_allocator,
  hal::v5::strong_ptr<usb> const& p_usb);

/**
 * @brief
 *
 * @param p_allocator
 * @param p_usb
 * @return std::pair<hal::v5::strong_ptr<hal::v5::usb_interrupt_out_endpoint>,
 * hal::v5::strong_ptr<hal::v5::usb_interrupt_in_endpoint>>
 */
std::pair<hal::v5::strong_ptr<hal::v5::usb_interrupt_out_endpoint>,
          hal::v5::strong_ptr<hal::v5::usb_interrupt_in_endpoint>>
acquire_usb_interrupt_endpoint(std::pmr::polymorphic_allocator<> p_allocator,
                               hal::v5::strong_ptr<usb> const& p_usb);
/**
 * @brief
 *
 * @param p_allocator
 * @param p_usb
 * @return std::pair<hal::v5::strong_ptr<hal::v5::usb_bulk_out_endpoint>,
 * hal::v5::strong_ptr<hal::v5::usb_bulk_in_endpoint>>
 */
std::pair<hal::v5::strong_ptr<hal::v5::usb_bulk_out_endpoint>,
          hal::v5::strong_ptr<hal::v5::usb_bulk_in_endpoint>>
acquire_usb_bulk_endpoint(std::pmr::polymorphic_allocator<> p_allocator,
                          hal::v5::strong_ptr<usb> const& p_usb);
}  // namespace hal::stm32f1

namespace hal {
using hal::stm32f1::acquire_usb_bulk_endpoint;
using hal::stm32f1::acquire_usb_control_endpoint;
using hal::stm32f1::acquire_usb_interrupt_endpoint;
}  // namespace hal
