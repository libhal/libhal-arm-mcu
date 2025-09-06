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

#include <libhal-util/bit.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal-util/usb.hpp>
#include <libhal/pointers.hpp>
#include <libhal/scatter_span.hpp>
#include <libhal/timeout.hpp>
#include <libhal/units.hpp>
#include <libhal/usb.hpp>

#include <libhal-arm-mcu/stm32f1/usb.hpp>

#include <resource_list.hpp>

using ctrl_receive_tag = hal::v5::usb::control_endpoint::on_receive_tag;
using bulk_receive_tag = hal::v5::usb::bulk_out_endpoint::on_receive_tag;

bool host_command_available = false;

void control_endpoint_handler(ctrl_receive_tag)
{
  host_command_available = true;
}

void write_string_descriptor(hal::v5::usb::control_endpoint& p_control_endpoint,
                             hal::u8 p_descriptor_index,  // NOLINT
                             std::size_t p_length,
                             std::span<std::u16string_view> p_strings)
{
  // Our span of strings is zero indexed and p_descriptor_index is 1 indexed
  // with respect to the p_string.
  p_descriptor_index -= 1;
  if (p_descriptor_index >= p_strings.size()) {
    throw std::out_of_range("OOR");
  }

  auto const str = p_strings[p_descriptor_index];
  auto str_span = hal::as_bytes(str);

  std::array<hal::u8, 2> const header = {
    static_cast<hal::u8>(str_span.size() + 2),
    0x03,
  };

  auto const header_write_length = std::min(header.size(), p_length);

  p_control_endpoint.write(
    hal::v5::make_scatter_bytes(std::span(header).first(header_write_length)));
  p_length -= header_write_length;  // deduce the amount written

  auto const string_write_length = std::min(str_span.size(), p_length);
  if (string_write_length > 0) {
    str_span = str_span.first(string_write_length);
    p_control_endpoint.write(hal::v5::make_scatter_bytes(str_span));
  }

  p_control_endpoint.write({});
}

void application()
{
  using namespace hal::literals;

  auto clock = resources::clock();
  auto console = resources::console();

  hal::print(*console, "Staring USB CDC application...\n");

  using namespace std::chrono_literals;

  auto control_endpoint = resources::usb_control_endpoint();
  auto serial_data_ep_out = resources::usb_bulk_out_endpoint1();
  auto serial_data_ep_in = resources::usb_bulk_in_endpoint1();
  auto status_ep_out = resources::usb_interrupt_out_endpoint1();
  auto status_ep_in = resources::usb_interrupt_in_endpoint1();

  hal::print(*console, "ctrl\n");

  control_endpoint->on_receive(control_endpoint_handler);
  hal::print(*console, "ctrl -> on_receive\n");

  // Device Descriptor (18 bytes)
  // NOLINTNEXTLINE
  uint8_t const device_descriptor[] = {
    0x12,        // bLength (18 bytes)
    0x01,        // bDescriptorType (Device)
    0x00, 0x02,  // bcdUSB (2.0)
    0x02,        // bDeviceClass (0x02 CDC)
    0x02,        // bDeviceSubClass (0x02 ACM)
    0x00,        // bDeviceProtocol
    16,          // bMaxPacketSize0 (16 bytes)
    0xEF, 0xBE,  // idVendor (0xBEEF)
    0xAD, 0xDE,  // idProduct (0xDEAD)
    0x00, 0x01,  // bcdDevice (1.0)
    0x01,        // iManufacturer (String #1)
    0x02,        // iProduct (String #2)
    0x03,        // iSerialNumber (String #3)
    0x01         // bNumConfigurations
  };

  // NOLINTNEXTLINE
  hal::u8 config_descriptor[] = {
    // Configuration Descriptor
    0x09,  // bLength
    0x02,  // bDescriptorType (Configuration)
    75,
    0x00,  // wTotalLength (75 bytes)
    0x02,  // bNumInterfaces
    0x01,  // bConfigurationValue
    0x00,  // iConfiguration (String Index)
    0x80,  // bmAttributes (Bus Powered)
    0x32,  // bMaxPower (100mA)

    // Interface Association Descriptor
    0x08,  // bLength
    0x0B,  // bDescriptorType (Interface Association)
    0x00,  // bFirstInterface
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
    0x07,                         // bLength
    0x05,                         // bDescriptorType (Endpoint)
    status_ep_in->info().number,  // bEndpointAddress (IN 2)
    0x03,                         // bmAttributes (Interrupt)
    static_cast<hal::u8>(status_ep_in->info().size),
    0x00,  // wMaxPacketSize 16
    0x10,  // bInterval (16 ms)

    // Interface Descriptor (Data)
    0x09,  // bLength
    0x04,  // bDescriptorType (Interface)
    0x01,  // bInterfaceNumber
    0x00,  // bAlternateSetting
    0x02,  // bNumEndpoints
    0x0A,  // bInterfaceClass (CDC-Data)
    0x00,  // bInterfaceSubClass
    0x00,  // bInterfaceProtocol
    0x00,  // iInterface (String Index)

    // Endpoint Descriptor (Data OUT)
    0x07,                               // bLength
    0x05,                               // bDescriptorType (Endpoint)
    serial_data_ep_out->info().number,  // bEndpointAddress (OUT + 1)
    0x02,                               // bmAttributes (Bulk)
    static_cast<hal::u8>(serial_data_ep_out->info().size),
    0x00,  // wMaxPacketSize 16
    0x00,  // bInterval (Ignored for Bulk)

    // Endpoint Descriptor (Data IN)
    0x07,                              // bLength
    0x05,                              // bDescriptorType (Endpoint)
    serial_data_ep_in->info().number,  // bEndpointAddress (IN + 1)
    0x02,                              // bmAttributes (Bulk)
    static_cast<hal::u8>(serial_data_ep_in->info().size),
    0x00,  // wMaxPacketSize 16
    0x00   // bInterval (Ignored for Bulk)
  };

  hal::print<64>(*console,
                 ">>>> status_ep_in->info().number = 0x%02X\n",
                 status_ep_in->info().number);
  hal::print<64>(*console,
                 ">>>> status_ep_in->info().size = %d\n",
                 status_ep_in->info().size);
  hal::print<64>(*console,
                 ">>>> serial_data_ep_out->info().number = 0x%02X\n",
                 serial_data_ep_out->info().number);
  hal::print<64>(*console,
                 ">>>> serial_data_ep_out->info().size = %d\n",
                 serial_data_ep_out->info().size);
  hal::print<64>(*console,
                 ">>>> serial_data_ep_in->info().number = 0x%02X\n",
                 serial_data_ep_in->info().number);
  hal::print<64>(*console,
                 ">>>> serial_data_ep_in->info().size = %d\n",
                 serial_data_ep_in->info().size);

  // String Descriptor 0 (Language ID)
  std::array<hal::u8, 4> const lang_descriptor{
    0x04,  // bLength
    0x03,  // bDescriptorType (String)
    0x09,  // wLANGID[0] (0x0409: English-US)
    0x04,  // wLANGID[0] (0x0409: English-US)
  };

  using namespace std::string_view_literals;

  constexpr auto manufacturer_str = u"libhal inc"sv;
  constexpr auto product_name_str = u"libhal USB device"sv;
  constexpr auto serial_str = u"ab01"sv;
  std::array<std::u16string_view, 3> strings = {
    manufacturer_str,
    product_name_str,
    serial_str,
  };

  // Example notification with DSR and DCD set
  // NOLINTNEXTLINE
  [[maybe_unused]] static uint8_t constexpr serial_state_notification[] = {
    0xA1,        // bmRequestType (Device to Host | Class | Interface)
    0x20,        // bNotification (SERIAL_STATE)
    0x00, 0x00,  // wValue (zero)
    0x00, 0x00,  // wIndex (Interface number)
    0x02, 0x00,  // wLength (2 bytes of data)
    0x03, 0x00   // serialState (DCD | DSR = 0x03)
  };

  bool serial_data_available = false;

  serial_data_ep_out->on_receive([&serial_data_available](bulk_receive_tag) {
    serial_data_available = true;
  });

  // CDC Class-Specific Request Codes
  constexpr hal::u8 cdc_set_line_coding = 0x20;
  constexpr hal::u8 cdc_get_line_coding = 0x21;
  constexpr hal::u8 cdc_set_control_line_state = 0x22;
  constexpr hal::u8 cdc_send_break = 0x23;

  // Line Coding Structure (7 bytes)
  static auto line_coding = std::to_array<uint8_t>({
    0x00,
    0x96,
    0x00,
    0x00,  // Baud rate: 38400 (Little Endian)
    0x00,  // Stop bits: 1
    0x00,  // Parity: None
    0x08   // Data bits: 8
  });

  // Wait for the message number to increment
  auto deadline = hal::future_deadline(*clock, 1s);
  bool port_connected = false;

  auto handle_serial = [&serial_data_available,
                        &port_connected,
                        deadline,
                        &serial_data_ep_out,
                        &serial_data_ep_in,
                        &console,
                        &clock]() mutable {
    if (serial_data_available) {
      // Drain endpoint of content...
      serial_data_available = false;
      // NOTE: Pulling out only 3 bytes isn't for memory reduction but to see
      // how well the read() API pulls data out from the endpoint memory. This
      // was used to find bugs with odd numbered buffer sizes.
      std::array<hal::u8, 3> buffer{};
      auto data_received =
        serial_data_ep_out->read(hal::v5::make_writable_scatter_bytes(buffer));

      hal::print(*console, "[");
      while (data_received != 0) {
        hal::write(*console,
                   std::span(buffer).first(data_received),
                   hal::never_timeout());
        data_received = serial_data_ep_out->read(
          hal::v5::make_writable_scatter_bytes(buffer));
      }
      hal::print(*console, "]");
    }
    // Send a '.' every second
    if (deadline < clock->uptime()) {
      using namespace std::string_view_literals;
      try {
        hal::v5::write_and_flush(*serial_data_ep_in, hal::as_bytes("."sv));
        hal::print(*console, ">");
        deadline = hal::future_deadline(*clock, 1s);
      } catch (hal::timed_out const&) {
        hal::print(*console,
                   "\n\n\033[48;5;9mEP TIMEOUT! PORT DISCONNECTED!\033[0m\n\n");
        port_connected = false;
        return;
      }
    }
  };

  hal::u8 configuration = 0;

  // Example handler for CDC class-specific setup packets
  auto handle_cdc_setup = [&port_connected, &control_endpoint, &console](
                            hal::u8 bmRequestType,  // NOLINT
                            hal::u8 bRequest,       // NOLINT
                            hal::u16 wValue         // NOLINT
                            ) -> bool {
    switch (bRequest) {
      case cdc_get_line_coding:
        if (bmRequestType == 0xA1) {  // Direction: Device to Host
          // Send current line coding
          hal::v5::write_and_flush(*control_endpoint, line_coding);
          hal::print(*console, "cdc_get_line_coding\n");
          return true;
        }
        break;

      case cdc_set_line_coding:
        if (bmRequestType == 0x21) {  // Direction: Host to Device
          while (true) {
            if (not host_command_available) {  // nothing received
              continue;
            }

            std::array<hal::u8, 8> rx_buffer;
            auto bytes_read = control_endpoint->read(
              hal::v5::make_writable_scatter_bytes(rx_buffer));
            if (bytes_read == 0) {
              continue;
            }
            auto host_command = std::span(rx_buffer).first(bytes_read);
            host_command_available = false;
            control_endpoint->write({});

            hal::print<64>(*console, "{{ ");
            for (auto const byte : host_command) {
              hal::print<8>(*console, "0x%" PRIx8 ", ", byte);
            }
            hal::print<64>(*console, "}}\n");
            std::copy_n(
              host_command.begin(), line_coding.size(), line_coding.begin());
            break;
          }
          // NOTE: there is no proper location to decide when a port is
          // connected.
          port_connected = true;
          hal::print(*console, "cdc_set_line_coding+\n");
          return true;
        }
        break;

      case cdc_set_control_line_state:
        if (bmRequestType == 0x21) {  // Direction: Host to Device
          // wValue contains the control signals:
          // Bit 0: DTR state
          // Bit 1: RTS state
          bool const dtr = wValue & 0x01;
          bool const rts = wValue & 0x02;
          // Handle control line state change
          // Most basic implementation just returns success
          control_endpoint->write({});
          hal::print<32>(*console, "DTR = %d, RTS = %d\n", dtr, rts);
          return true;
        }
        break;

      case cdc_send_break:
        if (bmRequestType == 0x21) {  // Direction: Host to Device
          // wValue contains break duration in milliseconds
          // Most basic implementation just returns success
          control_endpoint->write({});
          hal::print(*console, "SEND_BREAK\n");
          return true;
        }
        break;
      default:
        return false;
    }
    return false;  // Command not handled
  };

  auto command_count = 0;

  std::array<std::array<hal::v5::strong_ptr<hal::v5::usb::endpoint>, 2>, 2>
    map = {
      std::array<hal::v5::strong_ptr<hal::v5::usb::endpoint>, 2>{
        serial_data_ep_out,
        serial_data_ep_in,
      },
      {
        status_ep_out,
        status_ep_in,
      },
    };

  control_endpoint->connect(true);
  hal::print(*console, "connect\n");

  while (true) {
    if (configuration == 1 && port_connected) {
      handle_serial();
    }

    if (not host_command_available) {
      continue;
    }

    std::array<hal::u8, 8> rx_buffer;
    auto bytes_read =
      control_endpoint->read(hal::v5::make_writable_scatter_bytes(rx_buffer));
    if (bytes_read == 0) {
      host_command_available = false;
      continue;
    }

    auto buffer = std::span(rx_buffer).first(bytes_read);

    auto print_buffer = [&buffer, &console]() {
      for (auto const byte : buffer) {
        hal::print<8>(*console, "0x%" PRIx8 ", ", byte);
      }
    };

    // NOLINTBEGIN(readability-identifier-naming)
    // Extract bmRequestType and bRequest
    hal::u8 const bmRequestType = buffer[0];
    hal::u8 const bRequest = buffer[1];
    std::size_t const wValue = (buffer[3] << 8) | buffer[2];
    std::size_t const wIndex = (buffer[5] << 8) | buffer[4];
    std::size_t const wLength = (buffer[7] << 8) | buffer[6];
    // NOLINTEND(readability-identifier-naming)

    try {
      if (bmRequestType == 0x00 && bRequest == 0x05) {
        control_endpoint->write({});
        control_endpoint->set_address(buffer[2]);
        hal::print<32>(*console, "ZLP+SET_ADDR[%d]\n", buffer[2]);
      } else if (bmRequestType == 0x00 && bRequest == 0x09) {
        hal::u8 const descriptor_index = buffer[2];
        // SET_CONFIGURATION
        configuration = descriptor_index;
        serial_data_ep_out->reset();
        status_ep_out->reset();
        control_endpoint->write({});
        hal::print<16>(*console, "SC%" PRIu8 "\n", descriptor_index);
      } else if (bmRequestType == 0x80) {  // Device-to-host
        if (bRequest == 0x06) {            // GET_DESCRIPTOR
          hal::u8 const descriptor_index = buffer[2];
          hal::u8 const descriptor_type = buffer[3];
          switch (descriptor_type) {
            case 0x01: {  // Device Descriptor
              hal::print<16>(*console, "DD%" PRIu16 "\n", wLength);
              hal::v5::write_and_flush(
                *control_endpoint, std::span(device_descriptor).first(wLength));
              hal::print(*console, "DD+\n");
              break;
            }
            case 0x02:  // Configuration Descriptor
            {
              hal::print<32>(*console, "CD%" PRIu16 "\n", wLength);
              hal::v5::write_and_flush(
                *control_endpoint, std::span(config_descriptor).first(wLength));
              hal::print(*console, "CD+\n");
              break;
            }
            case 0x03: {  // String Descriptor
              hal::print<16>(*console,
                             "S%" PRIu8 ":%" PRIu16 "\n",
                             descriptor_index,
                             wLength);
              if (descriptor_index == 0) {
                auto const first =
                  std::min(lang_descriptor.size(), size_t{ wLength });
                auto const payload_span =
                  std::span(lang_descriptor).first(first);
                hal::v5::write_and_flush(*control_endpoint, payload_span);
              } else {
                write_string_descriptor(
                  *control_endpoint, descriptor_index, wLength, strings);
              }
              hal::print(*console, "S+\n");
              break;
            }
            default:
              hal::print(*console, "bmRequestType?\n");
              break;
          }
        }
      } else if (hal::bit_extract<hal::bit_mask::from(5, 6)>(bmRequestType) ==
                 0x1) {
        // Class-specific request
        auto const handled = handle_cdc_setup(bmRequestType, bRequest, wValue);
        if (handled) {
          hal::print(*console, "CDC-CLASS+\n");
        } else {
          hal::print(*console, "CDC-CLASS!\n");
        }
      } else if (bmRequestType == 0x02) {

        // Standard USB request codes
        constexpr hal::u8 get_status = 0x00;
        constexpr hal::u8 clear_feature = 0x01;
        constexpr hal::u8 set_feature = 0x03;

        auto const ep_select = wIndex & 0xF;
        auto const direction = hal::bit_extract<hal::bit_mask::from(7)>(wIndex);
        auto& selected_ep = *map.at(ep_select - 1).at(direction);

        switch (bRequest) {
          case get_status:
            hal::v5::write_and_flush(
              *control_endpoint,
              std::to_array<hal::byte>({ selected_ep.info().stalled, 0 }));
            hal::print<16>(
              *console, "STALLED: 0x%02" PRIx8, selected_ep.info().stalled);
            break;
          case clear_feature:
            selected_ep.stall(false);
            control_endpoint->write({});
            hal::print(*console, "CLEAR");
            break;
          case set_feature:
            hal::print(*console, "SET_FEATURE");
            break;
          default:
            hal::print<16>(*console, "DEFAULT:0x%02X", bRequest);
            break;
        }
        hal::print<32>(
          *console, " :EP[%" PRIu8 ", %d]\n", ep_select, direction);
      }

      hal::print<16>(*console, "COMMAND[%zu]: {", command_count++);
      print_buffer();
      hal::print(*console, "}\n\n");
    } catch (hal::timed_out const&) {
      hal::print(*console, "EP write operation timed out!\n");
    }
  }
}
