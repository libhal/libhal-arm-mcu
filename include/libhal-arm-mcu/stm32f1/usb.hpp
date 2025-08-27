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

/**
 * @brief USB interrupt endpoint pairs returned from
 * `acquire_usb_interrupt_endpoint`
 *
 * Each pair should have the same endpoint number and a separate out and in
 * endpoint.
 */
struct usb_interrupt_endpoint_pair
{
  /// The out endpoint implementation for this interrupt endpoint
  hal::v5::strong_ptr<hal::v5::usb_interrupt_out_endpoint> out;
  /// The out endpoint implementation for this interrupt endpoint
  hal::v5::strong_ptr<hal::v5::usb_interrupt_in_endpoint> in;
};

/**
 * @brief USB bulk endpoint pairs returned from
 * `acquire_usb_bulk_endpoint`
 *
 * Each pair should have the same endpoint number and a separate out and in
 * endpoint.
 */
struct usb_bulk_endpoint_pair
{
  /// The out endpoint implementation for this bulk endpoint
  hal::v5::strong_ptr<hal::v5::usb_bulk_out_endpoint> out;
  /// The out endpoint implementation for this bulk endpoint
  hal::v5::strong_ptr<hal::v5::usb_bulk_in_endpoint> in;
};

/**
 * @brief USB peripheral manager class for the stm32f1 processor
 *
 * In order to acquire USB endpoint resources, this class must be constructed
 * successfully.
 */
class usb
{
public:
  using ctrl_receive_tag = hal::v5::usb_control_endpoint::on_receive_tag;
  using out_receive_tag = hal::v5::usb_out_endpoint::on_receive_tag;
  using callback_variant_t = std::variant<hal::callback<void(ctrl_receive_tag)>,
                                          hal::callback<void(out_receive_tag)>>;

  static constexpr std::size_t usb_endpoint_count = 8;

  /**
   * @brief Construct a new usb object
   *
   * @param p_clock - clocked used for detecting endpoint stalls for write
   * operations. This can occur when the device is disconnected, the host has
   * stopped sending OUT packets, or something wrong with the bus.
   * @param p_write_timeout - the amount of time before throwing a timed_out
   * exception for an OUT endpoint with data to transmit but the HOST has not
   * requested it yet.
   */
  usb(hal::v5::strong_ptr_only_token,
      hal::v5::strong_ptr<hal::steady_clock> const& p_clock,
      hal::time_duration p_write_timeout = std::chrono::milliseconds(5));
  usb(usb&) = delete;
  usb operator=(usb&) = delete;
  usb(usb&&) = delete;
  usb operator=(usb&&) = delete;
  ~usb();

private:
  void interrupt_handler() noexcept;
  void fill_endpoint(hal::u8 p_endpoint,
                     std::span<hal::byte const> p_data,
                     hal::u16 p_max_length);
  void flush_endpoint(u8 p_endpoint);
  void write_to_endpoint(hal::u8 p_endpoint, std::span<hal::byte const> p_data);
  usize read_endpoint(u8 p_endpoint,
                      std::span<hal::byte> p_buffer,
                      u16& p_bytes_read);
  void wait_for_endpoint_transfer_completion(hal::u8 p_endpoint);
  void set_callback(hal::u8 p_endpoint, callback_variant_t const& p_callback);

  friend usb_interrupt_endpoint_pair acquire_usb_interrupt_endpoint(
    std::pmr::polymorphic_allocator<> p_allocator,
    hal::v5::strong_ptr<usb> const& p_usb);

  friend usb_bulk_endpoint_pair acquire_usb_bulk_endpoint(
    std::pmr::polymorphic_allocator<> p_allocator,
    hal::v5::strong_ptr<usb> const& p_usb);

  friend class control_endpoint;

  template<hal::v5::out_endpoint_type Interface>
  friend class out_endpoint;

  template<hal::v5::in_endpoint_type Interface>
  friend class in_endpoint;

  std::array<callback_variant_t, usb_endpoint_count> m_out_callbacks;
  hal::v5::strong_ptr<hal::steady_clock> m_clock;
  hal::time_duration m_write_timeout;
  std::uint16_t m_available_endpoint_memory;
  // Starts at 1 because endpoint 0 is always occupied by the control endpoint
  hal::u8 m_endpoints_allocated = 1;
};

/**
 * @brief Acquire a USB control endpoint
 *
 * @param p_allocator - the allocator to allocate the control endpoint's memory.
 * @param p_usb - the usb peripheral manager that you want to acquire a control
 * endpoint from.
 * @return hal::v5::strong_ptr<hal::v5::usb_control_endpoint> - the control
 * endpoint from the usb peripheral.
 */
hal::v5::strong_ptr<hal::v5::usb_control_endpoint> acquire_usb_control_endpoint(
  std::pmr::polymorphic_allocator<> p_allocator,
  hal::v5::strong_ptr<usb> const& p_usb);

/**
 * @brief Acquire a USB interrupt endpoint
 *
 * @param p_allocator - the allocator to allocate the interrupt endpoint's
 * memory.
 * @param p_usb - the usb peripheral manager that you want to acquire an
 * interrupt endpoint from.
 * @return usb_interrupt_endpoint_pair - an interrupt endpoint pair with both
 * out and in endpoints available.
 */
usb_interrupt_endpoint_pair acquire_usb_interrupt_endpoint(
  std::pmr::polymorphic_allocator<> p_allocator,
  hal::v5::strong_ptr<usb> const& p_usb);

/**
 * @brief Acquire a USB bulk endpoint
 *
 * @param p_allocator - the allocator to allocate the bulk endpoint's memory.
 * @param p_usb - the usb peripheral manager that you want to acquire an
 * bulk endpoint from.
 * @return usb_bulk_endpoint_pair - an bulk endpoint pair with both out and in
 * endpoints available.
 */
usb_bulk_endpoint_pair acquire_usb_bulk_endpoint(
  std::pmr::polymorphic_allocator<> p_allocator,
  hal::v5::strong_ptr<usb> const& p_usb);
}  // namespace hal::stm32f1

namespace hal {
using hal::stm32f1::acquire_usb_bulk_endpoint;
using hal::stm32f1::acquire_usb_control_endpoint;
using hal::stm32f1::acquire_usb_interrupt_endpoint;
}  // namespace hal
