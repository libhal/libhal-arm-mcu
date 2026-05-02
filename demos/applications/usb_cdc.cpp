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

#include <cinttypes>
#include <cmath>

#include <array>
#include <string_view>

#include <libhal-util/as_bytes.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal-util/usb.hpp>
#include <libhal-util/usb/descriptors.hpp>
#include <libhal/pointers.hpp>
#include <libhal/scatter_span.hpp>
#include <libhal/timeout.hpp>
#include <libhal/units.hpp>
#include <libhal/usb.hpp>

#include <libhal-arm-mcu/stm32f1/usb.hpp>

#include <resource_list.hpp>

hal::optional_ptr<hal::serial> g_console;

class usb_serial : public hal::usb::interface
{
public:
  usb_serial(
    hal::strong_ptr<hal::usb::bulk_out_endpoint> const& p_serial_data_ep_out,
    hal::strong_ptr<hal::usb::bulk_in_endpoint> const& p_serial_data_ep_in,
    hal::strong_ptr<hal::usb::interrupt_out_endpoint> const& p_status_ep_out,
    hal::strong_ptr<hal::usb::interrupt_in_endpoint> const& p_status_ep_in)
    : m_serial_data_ep_out(p_serial_data_ep_out)
    , m_serial_data_ep_in(p_serial_data_ep_in)
    , m_status_ep_out(p_status_ep_out)
    , m_status_ep_in(p_status_ep_in)
  {
  }

  void write_hello_world()
  {
    using namespace std::literals;

    m_serial_data_ep_in->write(hal::v5::make_scatter_array<hal::u8 const>(
      hal::as_bytes("Hello, World\n"sv)));
  }

private:
  hal::usb::interface::descriptor_count driver_write_descriptors(
    descriptor_start p_start,
    hal::usb::endpoint_io& p_endpoint) override
  {
    auto const start = p_start.interface.value_or(0);
    auto const idx1 = static_cast<hal::u8>(start + 0);
    auto const idx2 = static_cast<hal::u8>(start + 1);

    auto config_descriptor = std::to_array<hal::u8>({
      // Interface Association Descriptor
      0x08,  // bLength
      0x0B,  // bDescriptorType (Interface Association)
      idx1,  // bFirstInterface
      0x02,  // bInterfaceCount
      0x02,  // bFunctionClass (CDC)
      0x02,  // bFunctionSubClass (Abstract Control Model)
      0x01,  // bFunctionProtocol
      0x00,  // iFunction (String Index)

      // Interface Descriptor (Control)
      0x09,  // bLength
      0x04,  // bDescriptorType (Interface)
      0x00,  // bInterfaceNumber
      0x00,  // bAlternateSetting
      0x01,  // bNumEndpoints
      0x02,  // bInterfaceClass (CDC)
      0x02,  // bInterfaceSubClass (Abstract Control Model)
      0x01,  // bInterfaceProtocol (AT Commands V.250)
      0x00,  // iInterface (String Index)

      // CDC Header Functional Descriptor
      0x05,  // bLength
      0x24,  // bDescriptorType (CS_INTERFACE)
      0x00,  // bDescriptorSubtype (Header)
      0x10,
      0x01,  // bcdCDC (1.10)

      // CDC ACM Functional Descriptor
      0x04,  // bLength
      0x24,  // bDescriptorType (CS_INTERFACE)
      0x02,  // bDescriptorSubtype (Abstract Control Management)
      0x02,  // bmCapabilities

      // CDC Union Functional Descriptor
      0x05,  // bLength
      0x24,  // bDescriptorType (CS_INTERFACE)
      0x06,  // bDescriptorSubtype (Union)
      0x00,  // bControlInterface
      0x01,  // bSubordinateInterface0

      // CDC Call Management Functional Descriptor
      0x05,  // bLength
      0x24,  // bDescriptorType (CS_INTERFACE)
      0x01,  // bDescriptorSubtype (Call Management)
      0x00,  // bmCapabilities
      0x01,  // bDataInterface

      // Endpoint Descriptor (Control IN)
      0x07,                           // bLength
      0x05,                           // bDescriptorType (Endpoint)
      m_status_ep_in->info().number,  // bEndpointAddress (IN 2)
      0x03,                           // bmAttributes (Interrupt)
      static_cast<hal::u8>(m_status_ep_in->info().size),
      0x00,  // wMaxPacketSize 16
      0x10,  // bInterval (16 ms)

      // Interface Descriptor (Data)
      0x09,  // bLength
      0x04,  // bDescriptorType (Interface)
      idx2,  // bInterfaceNumber
      0x00,  // bAlternateSetting
      0x02,  // bNumEndpoints
      0x0A,  // bInterfaceClass (CDC-Data)
      0x00,  // bInterfaceSubClass
      0x00,  // bInterfaceProtocol
      0x00,  // iInterface (String Index)

      // Endpoint Descriptor (Data OUT)
      0x07,                                 // bLength
      0x05,                                 // bDescriptorType (Endpoint)
      m_serial_data_ep_out->info().number,  // bEndpointAddress (OUT + 1)
      0x02,                                 // bmAttributes (Bulk)
      static_cast<hal::u8>(m_serial_data_ep_out->info().size),
      0x00,  // wMaxPacketSize 16
      0x00,  // bInterval (Ignored for Bulk)

      // Endpoint Descriptor (Data IN)
      0x07,                                // bLength
      0x05,                                // bDescriptorType (Endpoint)
      m_serial_data_ep_in->info().number,  // bEndpointAddress (IN + 1)
      0x02,                                // bmAttributes (Bulk)
      static_cast<hal::u8>(m_serial_data_ep_in->info().size),
      0x00,  // wMaxPacketSize 16
      0x00   // bInterval (Ignored for Bulk)
    });

    p_endpoint.write(hal::make_scatter_array<hal::u8 const>(config_descriptor));

    return hal::usb::interface::descriptor_count{ .interface = 2, .string = 0 };
  }

  bool driver_write_string_descriptor(hal::u8, hal::usb::endpoint_io&) override
  {
    if (g_console) {
      hal::print(*g_console, "W");
    }
    return false;
  }

  bool driver_handle_request(hal::usb::setup_packet const& p_setup,
                             hal::usb::endpoint_io& p_endpoint) override
  {
    // CDC Class-Specific Request Codes
    constexpr hal::u8 cdc_set_line_coding = 0x20;
    constexpr hal::u8 cdc_get_line_coding = 0x21;
    constexpr hal::u8 cdc_set_control_line_state = 0x22;
    constexpr hal::u8 cdc_send_break = 0x23;

    switch (p_setup.request()) {
      case cdc_get_line_coding: {
        if (p_setup.is_device_to_host()) {  // Direction: Device to Host
          // Line Coding Structure (7 bytes)
          static auto const line_coding = std::to_array<uint8_t>({
            0x00,
            0x96,
            0x00,
            0x00,  // Baud rate: 38400 (Little Endian)
            0x00,  // Stop bits: 1
            0x00,  // Parity: None
            0x08   // Data bits: 8
          });

          // Send current line coding
          p_endpoint.write(hal::make_scatter_array<hal::u8 const>(line_coding));
          return true;
        }
        break;
      }

      case cdc_set_line_coding:
        if (not p_setup.is_device_to_host()) {  // Direction: Host to Device
          std::array<hal::u8, 8> rx_buffer{};
          std::ignore =
            p_endpoint.read(hal::v5::make_writable_scatter_bytes(rx_buffer));
          p_endpoint.write({});
          m_port_connected = true;
          return true;
        }
        break;

      case cdc_set_control_line_state:
        if (not p_setup.is_device_to_host()) {
          // wValue contains the control signals:
          // Bit 0: DTR state
          // Bit 1: RTS state
          m_dtr_rts.set(p_setup.value() & 0x01);
          m_dtr_rts.set(p_setup.value() & 0x02);
          p_endpoint.write({});
          return true;
        }
        break;

      case cdc_send_break:
        if (not p_setup.is_device_to_host()) {
          p_endpoint.write({});
          // SEND BREAK
          return true;
        }
        break;
      default:
        return false;
    }
    // Command not handled
    return false;
  }

  hal::strong_ptr<hal::usb::bulk_out_endpoint> m_serial_data_ep_out;
  hal::strong_ptr<hal::usb::bulk_in_endpoint> m_serial_data_ep_in;
  hal::strong_ptr<hal::usb::interrupt_out_endpoint> m_status_ep_out;
  hal::strong_ptr<hal::usb::interrupt_in_endpoint> m_status_ep_in;
  descriptor_start m_start;
  std::bitset<2> m_dtr_rts{ 0 };
  bool m_port_connected = false;
};

std::string_view to_string_view(std::errc p_errc)
{
  switch (p_errc) {
    case std::errc::address_family_not_supported:
      return "address_family_not_supported";
    case std::errc::address_in_use:
      return "address_in_use";
    case std::errc::address_not_available:
      return "address_not_available";
    case std::errc::already_connected:
      return "already_connected";
    case std::errc::argument_list_too_long:
      return "argument_list_too_long";
    case std::errc::argument_out_of_domain:
      return "argument_out_of_domain";
    case std::errc::bad_address:
      return "bad_address";
    case std::errc::bad_file_descriptor:
      return "bad_file_descriptor";
    case std::errc::bad_message:
      return "bad_message";
    case std::errc::broken_pipe:
      return "broken_pipe";
    case std::errc::connection_aborted:
      return "connection_aborted";
    case std::errc::connection_already_in_progress:
      return "connection_already_in_progress";
    case std::errc::connection_refused:
      return "connection_refused";
    case std::errc::connection_reset:
      return "connection_reset";
    case std::errc::cross_device_link:
      return "cross_device_link";
    case std::errc::destination_address_required:
      return "destination_address_required";
    case std::errc::device_or_resource_busy:
      return "device_or_resource_busy";
    case std::errc::directory_not_empty:
      return "directory_not_empty";
    case std::errc::executable_format_error:
      return "executable_format_error";
    case std::errc::file_exists:
      return "file_exists";
    case std::errc::file_too_large:
      return "file_too_large";
    case std::errc::filename_too_long:
      return "filename_too_long";
    case std::errc::function_not_supported:
      return "function_not_supported";
    case std::errc::host_unreachable:
      return "host_unreachable";
    case std::errc::identifier_removed:
      return "identifier_removed";
    case std::errc::illegal_byte_sequence:
      return "illegal_byte_sequence";
    case std::errc::inappropriate_io_control_operation:
      return "inappropriate_io_control_operation";
    case std::errc::interrupted:
      return "interrupted";
    case std::errc::invalid_argument:
      return "invalid_argument";
    case std::errc::invalid_seek:
      return "invalid_seek";
    case std::errc::io_error:
      return "io_error";
    case std::errc::is_a_directory:
      return "is_a_directory";
    case std::errc::message_size:
      return "message_size";
    case std::errc::network_down:
      return "network_down";
    case std::errc::network_reset:
      return "network_reset";
    case std::errc::network_unreachable:
      return "network_unreachable";
    case std::errc::no_buffer_space:
      return "no_buffer_space";
    case std::errc::no_child_process:
      return "no_child_process";
    case std::errc::no_link:
      return "no_link";
    case std::errc::no_lock_available:
      return "no_lock_available";
    case std::errc::no_message:
      return "no_message";
    case std::errc::no_protocol_option:
      return "no_protocol_option";
    case std::errc::no_space_on_device:
      return "no_space_on_device";
    case std::errc::no_such_device_or_address:
      return "no_such_device_or_address";
    case std::errc::no_such_device:
      return "no_such_device";
    case std::errc::no_such_file_or_directory:
      return "no_such_file_or_directory";
    case std::errc::no_such_process:
      return "no_such_process";
    case std::errc::not_a_directory:
      return "not_a_directory";
    case std::errc::not_a_socket:
      return "not_a_socket";
    case std::errc::not_connected:
      return "not_connected";
    case std::errc::not_enough_memory:
      return "not_enough_memory";
    case std::errc::not_supported:
      return "not_supported";
    case std::errc::operation_canceled:
      return "operation_canceled";
    case std::errc::operation_in_progress:
      return "operation_in_progress";
    case std::errc::operation_not_permitted:
      return "operation_not_permitted";
    case std::errc::operation_not_supported:
      return "operation_not_supported";
    case std::errc::operation_would_block:  // resource_unavailable_try_again
      return "resource_unavailable_try_again";
    case std::errc::owner_dead:
      return "owner_dead";
    case std::errc::permission_denied:
      return "permission_denied";
    case std::errc::protocol_error:
      return "protocol_error";
    case std::errc::protocol_not_supported:
      return "protocol_not_supported";
    case std::errc::read_only_file_system:
      return "read_only_file_system";
    case std::errc::resource_deadlock_would_occur:
      return "resource_deadlock_would_occur";
    case std::errc::result_out_of_range:
      return "result_out_of_range";
    case std::errc::state_not_recoverable:
      return "state_not_recoverable";
    case std::errc::text_file_busy:
      return "text_file_busy";
    case std::errc::timed_out:
      return "timed_out";
    case std::errc::too_many_files_open_in_system:
      return "too_many_files_open_in_system";
    case std::errc::too_many_files_open:
      return "too_many_files_open";
    case std::errc::too_many_links:
      return "too_many_links";
    case std::errc::too_many_symbolic_link_levels:
      return "too_many_symbolic_link_levels";
    case std::errc::value_too_large:
      return "value_too_large";
    case std::errc::wrong_protocol_type:
      return "wrong_protocol_type";
    default:
      return "unknown";
  }
}

void application()
{
  using namespace hal::literals;
  using namespace std::literals;
  using namespace std::chrono_literals;

  auto clock = resources::clock();
  auto console = resources::console();
  g_console = console;

  hal::print(*console, "Staring USB CDC application...\n");

  auto allocator = resources::driver_allocator();
  auto control_endpoint = resources::usb_control_endpoint();
  auto serial_data_ep_out = resources::usb_bulk_out_endpoint1();
  auto serial_data_ep_in = resources::usb_bulk_in_endpoint1();
  auto status_ep_out = resources::usb_interrupt_out_endpoint1();
  auto status_ep_in = resources::usb_interrupt_in_endpoint1();

  auto usb_device = hal::make_strong_ptr<hal::usb::device>(
    allocator,
    hal::usb::device::device_arguments{
      .bcd_usb = 0x02,
      .device_class = hal::usb::class_code::cdc_control,
      .device_subclass = 0x2,
      .device_protocol = 0x0,
      .id_vendor = 0xDEAD,
      .id_product = 0xBEEF,
      .bcd_device = 0x01,
      .p_manufacturer = u"libhal"sv,
      .p_product = u"libhal serial device"sv,
      .p_serial_number_str = u"aabbupdownleftright"sv,
    });

  auto configs = hal::make_strong_ptr<std::array<hal::usb::configuration, 1>>(
    allocator,
    std::to_array<hal::usb::configuration>(
      { hal::usb::configuration{ hal::usb::configuration::configuration_info{
        .name = u"my_config"sv,
        .attributes = hal::usb::configuration::bitmap(false, false),
        .max_power = 200,
        .allocator = allocator,
      } } }));

  hal::usb::enumerator<1> usb_enumerator(hal::usb::enumerator<1>::args{
    .ctrl_ep = control_endpoint,
    .device = usb_device,
    .configs = configs,
    .lang_str = 0x00,
    .retry_max = 255,
  });

  while (true) {
    try {
      usb_enumerator.process_ctrl_transfer();
    } catch (hal::exception const& p_error) {
      auto const view = to_string_view(p_error.error_code());
      hal::print<256>(*console, "exception caught with ID %s\n", view.data());
    }
  }
}
