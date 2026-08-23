// Copyright 2026 Khalil Estell and the libhal contributors
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

module;

#include <algorithm>
#include <array>
#include <chrono>
#include <coroutine>
#include <span>
#include <string_view>

module arm_mcu_demos;

import hal;
import hal.util;

namespace {
// Writes `p_data` to an IN-capable endpoint (control or in_endpoint derived)
// and finishes the transfer with a flush (empty write), matching the
// two-step "fill then flush" contract documented on `hal::usb::in_endpoint::
// write()`. hal.util has no v6 USB module yet, so this stands in for v1's
// `hal::v5::write_and_flush()`.
template<typename Endpoint>
async::future<void> write_and_flush(async::context& p_ctx,
                                    Endpoint& p_endpoint,
                                    std::span<hal::byte const> p_data)
{
  co_await p_endpoint.write(p_ctx, { p_data });
  co_await p_endpoint.write(p_ctx, {});
}

async::future<void> write_string_descriptor(
  async::context& p_ctx,
  hal::usb::control_endpoint& p_control_endpoint,
  hal::u8 p_descriptor_index,
  hal::usize p_length,
  std::span<std::u16string_view const> p_strings)
{
  // Our span of strings is zero indexed; p_descriptor_index is 1-indexed.
  p_descriptor_index -= 1;
  if (p_descriptor_index >= p_strings.size()) {
    throw hal::argument_out_of_domain(&p_control_endpoint);
  }

  auto const str = p_strings[p_descriptor_index];
  auto str_span = std::as_bytes(std::span(str));

  std::array<hal::byte, 2> const header = {
    static_cast<hal::byte>(str_span.size() + 2),
    0x03,
  };

  auto const header_write_length = std::min(header.size(), p_length);
  co_await p_control_endpoint.write(
    p_ctx, { std::span(header).first(header_write_length) });
  p_length -= header_write_length;

  auto const string_write_length = std::min(str_span.size(), p_length);
  if (string_write_length > 0) {
    co_await p_control_endpoint.write(
      p_ctx,
      { std::span<hal::byte const>(
        reinterpret_cast<hal::byte const*>(str_span.data()),
        string_write_length) });
  }

  co_await p_control_endpoint.write(p_ctx, {});
}
}  // namespace

hal::task application(async::context& p_ctx)
{
  using namespace std::chrono_literals;
  using namespace std::string_view_literals;

  auto console = resources::console();

  co_await hal::write(p_ctx, *console, "Starting USB CDC (raw) demo...\n");

  auto control_endpoint = co_await resources::usb_control_endpoint(p_ctx);
  auto serial_data_ep_out = co_await resources::usb_bulk_out_endpoint1(p_ctx);
  auto serial_data_ep_in = co_await resources::usb_bulk_in_endpoint1(p_ctx);
  auto status_ep_out = co_await resources::usb_interrupt_out_endpoint1(p_ctx);
  auto status_ep_in = co_await resources::usb_interrupt_in_endpoint1(p_ctx);

  // Device Descriptor (18 bytes)
  std::array<hal::byte, 18> const device_descriptor = {
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
    0x01,        // bNumConfigurations
  };

  std::array<hal::byte, 75> config_descriptor = {
    // Configuration Descriptor
    0x09,        // bLength
    0x02,        // bDescriptorType (Configuration)
    75, 0x00,    // wTotalLength (75 bytes)
    0x02,        // bNumInterfaces
    0x01,        // bConfigurationValue
    0x00,        // iConfiguration (String Index)
    0x80,        // bmAttributes (Bus Powered)
    0x32,        // bMaxPower (100mA)

    // Interface Association Descriptor
    0x08,        // bLength
    0x0B,        // bDescriptorType (Interface Association)
    0x00,        // bFirstInterface
    0x02,        // bInterfaceCount
    0x02,        // bFunctionClass (CDC)
    0x02,        // bFunctionSubClass (Abstract Control Model)
    0x01,        // bFunctionProtocol
    0x00,        // iFunction (String Index)

    // Interface Descriptor (Control)
    0x09,        // bLength
    0x04,        // bDescriptorType (Interface)
    0x00,        // bInterfaceNumber
    0x00,        // bAlternateSetting
    0x01,        // bNumEndpoints
    0x02,        // bInterfaceClass (CDC)
    0x02,        // bInterfaceSubClass (Abstract Control Model)
    0x01,        // bInterfaceProtocol (AT Commands V.250)
    0x00,        // iInterface (String Index)

    // CDC Header Functional Descriptor
    0x05, 0x24, 0x00, 0x10, 0x01,  // bcdCDC (1.10)

    // CDC ACM Functional Descriptor
    0x04, 0x24, 0x02, 0x02,  // bmCapabilities

    // CDC Union Functional Descriptor
    0x05, 0x24, 0x06, 0x00, 0x01,

    // CDC Call Management Functional Descriptor
    0x05, 0x24, 0x01, 0x00, 0x01,

    // Endpoint Descriptor (Control IN)
    0x07,                                             // bLength
    0x05,                                             // bDescriptorType
    static_cast<hal::byte>(status_ep_in->info().number),
    0x03,                                             // bmAttributes (Interrupt)
    static_cast<hal::byte>(status_ep_in->info().size),
    0x00,
    0x10,  // bInterval (16 ms)

    // Interface Descriptor (Data)
    0x09, 0x04, 0x01, 0x00, 0x02, 0x0A, 0x00, 0x00, 0x00,

    // Endpoint Descriptor (Data OUT)
    0x07,
    0x05,
    static_cast<hal::byte>(serial_data_ep_out->info().number),
    0x02,  // bmAttributes (Bulk)
    static_cast<hal::byte>(serial_data_ep_out->info().size),
    0x00,
    0x00,

    // Endpoint Descriptor (Data IN)
    0x07,
    0x05,
    static_cast<hal::byte>(serial_data_ep_in->info().number),
    0x02,
    static_cast<hal::byte>(serial_data_ep_in->info().size),
    0x00,
    0x00,
  };

  // String Descriptor 0 (Language ID)
  std::array<hal::byte, 4> const lang_descriptor{
    0x04,  // bLength
    0x03,  // bDescriptorType (String)
    0x09,  // wLANGID[0] (0x0409: English-US)
    0x04,
  };

  constexpr auto manufacturer_str = u"libhal inc"sv;
  constexpr auto product_name_str = u"libhal USB device"sv;
  constexpr auto serial_str = u"ab01"sv;
  std::array<std::u16string_view, 3> const strings = {
    manufacturer_str,
    product_name_str,
    serial_str,
  };

  // CDC Class-Specific Request Codes
  constexpr hal::u8 cdc_set_line_coding = 0x20;
  constexpr hal::u8 cdc_get_line_coding = 0x21;
  constexpr hal::u8 cdc_set_control_line_state = 0x22;
  constexpr hal::u8 cdc_send_break = 0x23;

  // Line Coding Structure (7 bytes): 38400 baud, 1 stop bit, no parity, 8
  // data bits.
  std::array<hal::byte, 7> line_coding = {
    0x00, 0x96, 0x00, 0x00, 0x00, 0x00, 0x08,
  };

  hal::u8 configuration = 0;
  hal::usize command_count = 0;

  co_await control_endpoint->connect(p_ctx, true);
  co_await hal::write(p_ctx, *console, "USB connected, awaiting host...\n");

  while (true) {
    // Echo any bulk OUT data received from the host straight to the console.
    std::array<hal::byte, 16> bulk_buffer{};
    auto bulk_bytes = co_await serial_data_ep_out->read(p_ctx, { bulk_buffer });
    while (bulk_bytes != 0) {
      co_await hal::write(
        p_ctx, *console, { std::span(bulk_buffer).first(bulk_bytes) });
      bulk_bytes = co_await serial_data_ep_out->read(p_ctx, { bulk_buffer });
    }

    // Poll the control endpoint for a new SETUP or DATA packet. This driver
    // supports real setup-packet detection via `on_bus_event()`, but this
    // demo intentionally uses the documented fallback of treating every
    // non-empty read as a command, to keep the loop's structure simple and
    // avoid blocking on a single-waiter await that would starve the bulk
    // endpoint above.
    std::array<hal::byte, 8> rx_buffer{};
    auto const bytes_read =
      co_await control_endpoint->read(p_ctx, { rx_buffer });
    if (bytes_read == 0) {
      continue;
    }

    auto const buffer = std::span(rx_buffer).first(bytes_read);
    hal::u8 const bm_request_type = buffer[0];
    hal::u8 const b_request = buffer[1];
    hal::u16 const w_value =
      static_cast<hal::u16>((buffer[3] << 8) | buffer[2]);
    hal::u16 const w_index =
      static_cast<hal::u16>((buffer[5] << 8) | buffer[4]);
    hal::u16 const w_length =
      static_cast<hal::u16>((buffer[7] << 8) | buffer[6]);

    if (bm_request_type == 0x00 && b_request == 0x05) {
      // SET_ADDRESS
      co_await control_endpoint->write(p_ctx, {});
      co_await control_endpoint->set_address(p_ctx, buffer[2]);
      co_await hal::print<32>(
        p_ctx, *console, "ZLP+SET_ADDR[%d]\n", buffer[2]);
    } else if (bm_request_type == 0x00 && b_request == 0x09) {
      // SET_CONFIGURATION
      configuration = buffer[2];
      co_await serial_data_ep_out->reset(p_ctx);
      co_await status_ep_out->reset(p_ctx);
      co_await control_endpoint->write(p_ctx, {});
      co_await hal::print<16>(p_ctx, *console, "SET_CONFIG[%u]\n", configuration);
    } else if (bm_request_type == 0x80) {  // Device-to-host
      if (b_request == 0x06) {             // GET_DESCRIPTOR
        hal::u8 const descriptor_index = buffer[2];
        hal::u8 const descriptor_type = buffer[3];
        switch (descriptor_type) {
          case 0x01:  // Device Descriptor
            co_await write_and_flush(
              p_ctx,
              *control_endpoint,
              std::span(device_descriptor).first(w_length));
            break;
          case 0x02:  // Configuration Descriptor
            co_await write_and_flush(
              p_ctx,
              *control_endpoint,
              std::span(config_descriptor).first(w_length));
            break;
          case 0x03:  // String Descriptor
            if (descriptor_index == 0) {
              auto const length =
                std::min<hal::usize>(lang_descriptor.size(), w_length);
              co_await write_and_flush(
                p_ctx,
                *control_endpoint,
                std::span(lang_descriptor).first(length));
            } else {
              co_await write_string_descriptor(
                p_ctx, *control_endpoint, descriptor_index, w_length, strings);
            }
            break;
          default:
            break;
        }
      }
    } else if (hal::bit_extract<hal::bit_mask::from(5, 6)>(bm_request_type) ==
              0x1) {
      // Class-specific (CDC) request
      switch (b_request) {
        case cdc_get_line_coding:
          if (bm_request_type == 0xA1) {
            co_await write_and_flush(p_ctx, *control_endpoint, line_coding);
          }
          break;
        case cdc_set_line_coding:
          if (bm_request_type == 0x21) {
            std::array<hal::byte, 8> host_rx{};
            auto const host_bytes =
              co_await control_endpoint->read(p_ctx, { host_rx });
            if (host_bytes > 0) {
              std::copy_n(host_rx.begin(), line_coding.size(),
                         line_coding.begin());
            }
            co_await control_endpoint->write(p_ctx, {});
            co_await hal::write(p_ctx, *console, "cdc_set_line_coding\n");
          }
          break;
        case cdc_set_control_line_state:
          if (bm_request_type == 0x21) {
            bool const dtr = (w_value & 0x01) != 0;
            bool const rts = (w_value & 0x02) != 0;
            co_await control_endpoint->write(p_ctx, {});
            co_await hal::print<32>(
              p_ctx, *console, "DTR = %d, RTS = %d\n", dtr, rts);
          }
          break;
        case cdc_send_break:
          if (bm_request_type == 0x21) {
            co_await control_endpoint->write(p_ctx, {});
            co_await hal::write(p_ctx, *console, "SEND_BREAK\n");
          }
          break;
        default:
          break;
      }
    } else if (bm_request_type == 0x02) {
      // Standard endpoint request. `hal::usb::endpoint` (the common base of
      // every endpoint type) isn't part of the public interface, so the two
      // endpoint pairs are dispatched by hand instead of through a lookup
      // table of generic endpoint pointers.
      constexpr hal::u8 get_status = 0x00;
      constexpr hal::u8 clear_feature = 0x01;

      auto const ep_select = w_index & 0xF;
      bool const in_direction =
        hal::bit_extract<hal::bit_mask::from(7)>(w_index);

      hal::usb::endpoint_info const selected_info =
        ep_select == 1
          ? (in_direction ? serial_data_ep_in->info() : serial_data_ep_out->info())
          : (in_direction ? status_ep_in->info() : status_ep_out->info());

      switch (b_request) {
        case get_status: {
          std::array<hal::byte, 2> const status{
            static_cast<hal::byte>(selected_info.stalled), 0
          };
          co_await write_and_flush(p_ctx, *control_endpoint, status);
          break;
        }
        case clear_feature:
          if (ep_select == 1) {
            if (in_direction) {
              co_await serial_data_ep_in->stall(p_ctx, false);
            } else {
              co_await serial_data_ep_out->stall(p_ctx, false);
            }
          } else {
            if (in_direction) {
              co_await status_ep_in->stall(p_ctx, false);
            } else {
              co_await status_ep_out->stall(p_ctx, false);
            }
          }
          co_await control_endpoint->write(p_ctx, {});
          break;
        default:
          break;
      }
    }

    co_await hal::print<16>(p_ctx, *console, "COMMAND[%zu]\n", command_count++);
  }
}
