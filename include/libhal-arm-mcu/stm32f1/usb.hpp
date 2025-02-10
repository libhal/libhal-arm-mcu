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

#include <chrono>
#include <utility>
#include <variant>

#include <libhal/experimental/usb.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>

namespace hal::stm32f1 {

static constexpr std::size_t usb_endpoint_count = 8;

/// TODO(kammce): Move to src directory
struct usb_endpoint_register_t
{
  struct [[gnu::packed]] bits
  {
    unsigned ea : 4;
    unsigned stat_tx : 2;
    unsigned dogtx : 1;
    unsigned tx_complete : 1;
    unsigned ep_kind : 1;
    unsigned ep_type : 2;
    unsigned setup : 1;
    unsigned stat_rx : 2;
    unsigned dtog_rx : 1;
    unsigned rx_complete : 1;
  };
  // Write operations should not use read-modify-write but write directly.
  // Writing zeros to this register does not change
  union
  {
    hal::u32 volatile EPR;
    bits volatile bit;
  };
};

/// TODO(kammce): Move to src directory
struct usb_reg_t
{
  /// Endpoint Registers
  std::array<usb_endpoint_register_t, usb_endpoint_count> EP;
  /// Reserved Registers
  std::array<hal::u32, usb_endpoint_count> reserved;
  /// Control Register
  hal::u32 volatile CNTR;
  /// Interrupt Status Register
  hal::u32 volatile ISTR;
  /// Frame Number Register
  hal::u32 volatile FNR;
  /// Device Address Register
  hal::u32 volatile DADDR;
  /// Buffer Table Address Register
  hal::u32 volatile BTABLE;
};

// Ensure that CNTR is put in the correct location
static_assert(offsetof(usb_reg_t, CNTR) == 0x40);
static_assert(offsetof(usb_reg_t, BTABLE) == 0x50);

inline auto* usb_reg = reinterpret_cast<usb_reg_t*>(0x4000'5C00);

class usb
{
public:
  class control_endpoint;
  class interrupt_in_endpoint;
  class bulk_in_endpoint;
  class interrupt_out_endpoint;
  class bulk_out_endpoint;

  usb(hal::steady_clock& p_clock,
      hal::time_duration p_power_on_time = std::chrono::microseconds(1));
  usb(usb&) = delete;
  usb operator=(usb&) = delete;
  usb(usb&&) = delete;
  usb operator=(usb&&) = delete;
  ~usb();

  control_endpoint acquire_control_endpoint();
  std::pair<interrupt_in_endpoint, interrupt_out_endpoint>
  acquire_interrupt_endpoint();
  std::pair<bulk_in_endpoint, bulk_out_endpoint> acquire_bulk_endpoint();

  bool disrupted();

private:
  friend class control_endpoint;
  friend class interrupt_in_endpoint;
  friend class bulk_in_endpoint;
  friend class interrupt_out_endpoint;
  friend class bulk_out_endpoint;

  void interrupt_handler() noexcept;
  void write_to_endpoint(std::uint8_t p_endpoint,
                         std::span<hal::byte const> p_data);
  void write_to_control_endpoint(std::span<hal::byte const> p_data);
  std::span<u8 const> read_endpoint(u8 p_endpoint,
                                    std::span<u8> p_buffer,
                                    u16& p_bytes_read);
  void wait_for_endpoint_transfer_completion(std::uint8_t p_endpoint);

  using ctrl_request_tag =
    hal::experimental::usb_control_endpoint::on_request_tag;
  using bulk_receive_tag =
    hal::experimental::usb_bulk_out_endpoint::on_receive_tag;
  using interrupt_receive_tag =
    hal::experimental::usb_interrupt_out_endpoint::on_receive_tag;

  using callback_variant_t =
    std::variant<hal::callback<void(ctrl_request_tag)>,
                 hal::callback<void(bulk_receive_tag)>,
                 hal::callback<void(interrupt_receive_tag)>>;
  std::array<callback_variant_t, usb_endpoint_count> m_out_callbacks;
  hal::steady_clock* m_clock;
  std::uint16_t m_available_endpoint_memory;
  // Starts at 1 because endpoint 0 is always occupied by the control endpoint
  std::uint8_t m_endpoints_allocated = 1;
};

/**
 * @brief USB Control Endpoint Interface
 *
 * This class represents the control endpoint of a USB device. The control
 * endpoint is crucial for USB communication as it handles device enumeration,
 * configuration, and general control operations.
 *
 * Use cases:
 * - Initiating USB connections
 * - Handling USB enumeration process
 * - Setting device addresses
 * - Responding to standard USB requests
 * - Sending and receiving control data
 *
 */
class usb::control_endpoint : public hal::experimental::usb_control_endpoint
{
public:
  ~control_endpoint();
  control_endpoint(control_endpoint&) = delete;
  control_endpoint operator=(control_endpoint&) = delete;
  control_endpoint(control_endpoint&&) = delete;
  control_endpoint operator=(control_endpoint&&) = delete;

  bool in_setup_stage();
  void enable_rx();

private:
  friend class usb;

  control_endpoint(usb& p_usb);

  bool stalled() const;

  void driver_connect(bool p_should_connect) override;
  void driver_set_address(std::uint8_t p_address) override;
  void driver_write(std::span<hal::byte const> p_data) override;
  void driver_on_request(
    hal::callback<void(on_request_tag)> p_callback) override;
  hal::experimental::usb_endpoint_info driver_info() const override;
  void driver_stall(bool p_should_stall) override;
  std::optional<std::array<u8, 8>> driver_read() override;

  usb* m_usb;
  u16 m_bytes_read = 0;
};

/**
 * @brief USB Interrupt IN Endpoint Interface
 *
 * This class represents an interrupt IN endpoint of a USB device. Interrupt IN
 * endpoints are used for small, time-sensitive data transfers from the device
 * to the host.
 *
 * Use cases:
 * - Sending periodic status updates
 * - Transmitting small amounts of data with guaranteed latency
 * - Ideal for devices like keyboards, mice, or game controllers
 */
class usb::interrupt_in_endpoint
  : public hal::experimental::usb_interrupt_in_endpoint
{
public:
  ~interrupt_in_endpoint();
  void reset();

private:
  bool stalled() const;
  friend class usb;
  interrupt_in_endpoint(usb& p_usb, u8 p_endpoint_number);
  void driver_write(std::span<hal::byte const> p_data) override;
  hal::experimental::usb_endpoint_info driver_info() const override;
  void driver_stall(bool p_should_stall) override;

  usb* m_usb;
  u16 m_bytes_read = 0;
  u8 m_endpoint_number;
};

/**
 * @brief USB Bulk IN Endpoint Interface
 *
 * This class represents a bulk IN endpoint of a USB device. Bulk IN endpoints
 * are used for large, non-time-critical data transfers from the device to the
 * host.
 *
 * Use cases:
 * - Transferring large amounts of data
 * - Sending data when timing is not critical
 * - Ideal for devices like printers, scanners, or external storage
 */
class usb::bulk_in_endpoint : public hal::experimental::usb_bulk_in_endpoint
{
public:
  ~bulk_in_endpoint() override;
  void reset();

private:
  bool stalled() const;
  friend class usb;

  bulk_in_endpoint(usb& p_usb, u8 p_endpoint_number);

  void driver_write(std::span<hal::byte const> p_data) override;
  hal::experimental::usb_endpoint_info driver_info() const override;
  void driver_stall(bool p_should_stall) override;

  usb* m_usb;
  u8 m_endpoint_number;
};

/**
 * @brief USB Interrupt OUT Endpoint Interface
 *
 * This class represents an interrupt OUT endpoint of a USB device. Interrupt
 * OUT endpoints are used for small, time-sensitive data transfers from the host
 * to the device.
 *
 * Use cases:
 * - Receiving periodic commands or settings
 * - Handling small amounts of data with guaranteed latency
 * - Ideal for devices that need quick responses to host commands
 */
class usb::interrupt_out_endpoint
  : public hal::experimental::usb_interrupt_out_endpoint
{
public:
  ~interrupt_out_endpoint();
  void reset();

private:
  bool stalled() const;
  friend class usb;

  interrupt_out_endpoint(usb& p_usb, u8 p_endpoint_number);

  void driver_on_receive(
    hal::callback<void(on_receive_tag)> p_callback) override;
  hal::experimental::usb_endpoint_info driver_info() const override;
  void driver_stall(bool p_should_stall) override;
  virtual std::span<u8 const> driver_read(std::span<u8> p_buffer) override;

  usb* m_usb;
  u16 m_bytes_read = 0;
  u8 m_endpoint_number;
};

/**
 * @brief USB Bulk OUT Endpoint Interface
 *
 * This class represents a bulk OUT endpoint of a USB device. Bulk OUT endpoints
 * are used for large, non-time-critical data transfers from the host to the
 * device.
 *
 * Use cases:
 * - Receiving large amounts of data
 * - Handling data when timing is not critical
 * - Ideal for devices like printers, scanners, or external storage
 */
class usb::bulk_out_endpoint : public hal::experimental::usb_bulk_out_endpoint
{
public:
  ~bulk_out_endpoint();
  void reset();

private:
  bool stalled() const;
  friend class usb;

  bulk_out_endpoint(usb& p_usb, u8 p_endpoint_number);

  void driver_on_receive(
    hal::callback<void(on_receive_tag)> p_callback) override;
  hal::experimental::usb_endpoint_info driver_info() const override;
  void driver_stall(bool p_should_stall) override;
  virtual std::span<u8 const> driver_read(std::span<u8> p_buffer) override;

  usb* m_usb;
  u16 m_bytes_read = 0;
  u8 m_endpoint_number;
};
}  // namespace hal::stm32f1
