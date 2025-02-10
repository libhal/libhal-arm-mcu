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

#include <cinttypes>

#include <libhal-arm-mcu/dwt_counter.hpp>
#include <libhal-arm-mcu/startup.hpp>
#include <libhal-arm-mcu/stm32f1/can.hpp>
#include <libhal-arm-mcu/stm32f1/clock.hpp>
#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal-arm-mcu/stm32f1/input_pin.hpp>
#include <libhal-arm-mcu/stm32f1/output_pin.hpp>
#include <libhal-arm-mcu/stm32f1/pin.hpp>
#include <libhal-arm-mcu/stm32f1/spi.hpp>
#include <libhal-arm-mcu/stm32f1/uart.hpp>
#include <libhal-arm-mcu/stm32f1/usb.hpp>
#include <libhal-arm-mcu/system_control.hpp>
#include <libhal-util/as_bytes.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/bit_bang_i2c.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <nonstd/ring_span.hpp>

#include <resource_list.hpp>

using setup_command_buffer = std::array<hal::byte, 8>;

using ctrl_request_tag =
  hal::experimental::usb_control_endpoint::on_request_tag;
using bulk_receive_tag =
  hal::experimental::usb_bulk_out_endpoint::on_receive_tag;
using interrupt_receive_tag =
  hal::experimental::usb_interrupt_out_endpoint::on_receive_tag;

bool host_command_available = false;

void control_endpoint_handler(ctrl_request_tag)
{
  host_command_available = true;
}

void initialize_platform(resource_list& p_resource)
{
  using namespace hal::literals;

  p_resource.reset = []() { hal::cortex_m::reset(); };
  hal::stm32f1::release_jtag_pins();

  hal::stm32f1::configure_clocks(hal::stm32f1::clock_tree{
    .high_speed_external = 8.0_MHz,
    .pll = {
      .enable = true,
      .source = hal::stm32f1::pll_source::high_speed_external,
      .multiply = hal::stm32f1::pll_multiply::multiply_by_9,
      .usb = {
        .divider = hal::stm32f1::usb_divider::divide_by_1_point_5,
      }
    },
    .system_clock = hal::stm32f1::system_clock_select::pll,
    .ahb = {
      .divider = hal::stm32f1::ahb_divider::divide_by_1,
      .apb1 = {
        .divider = hal::stm32f1::apb_divider::divide_by_2,
      },
      .apb2 = {
        .divider = hal::stm32f1::apb_divider::divide_by_1,
        .adc = {
          .divider = hal::stm32f1::adc_divider::divide_by_6,
        }
      },
    },
  });
  hal::stm32f1::activate_mco_pa8(
    hal::stm32f1::mco_source::pll_clock_divided_by_2);

  auto cpu_frequency = hal::stm32f1::frequency(hal::stm32f1::peripheral::cpu);
  static hal::cortex_m::dwt_counter steady_clock(cpu_frequency);
  p_resource.clock = &steady_clock;

  static hal::stm32f1::uart uart1(hal::port<1>, hal::buffer<128>);
  p_resource.console = &uart1;

#if 0
  static hal::stm32f1::can_peripheral_manager can(
    100_kHz, hal::stm32f1::can_pins::pb9_pb8, {}, true);

  static std::array<hal::can_message, 8> receive_buffer{};
  static auto can_transceiver = can.acquire_transceiver(receive_buffer);
  p_resource.can_transceiver = &can_transceiver;

  static auto can_bus_manager = can.acquire_bus_manager();
  p_resource.can_bus_manager = &can_bus_manager;

  static auto can_interrupt = can.acquire_interrupt();
  p_resource.can_interrupt = &can_interrupt;

  // Allow all messages
  static auto mask_id_filters_x2 = can.acquire_mask_filter();
  mask_id_filters_x2.filter[0].allow({ { .id = 0, .mask = 0 } });
#endif

  static hal::stm32f1::output_pin led('C', 13);
  p_resource.status_led = &led;

  // pin G0 on the STM micromod is port B, pin 4
  static hal::stm32f1::input_pin input_pin('B', 4);
  p_resource.input_pin = &input_pin;

  static hal::stm32f1::output_pin sda_output_pin('A', 0);
  static hal::stm32f1::output_pin scl_output_pin('A', 15);

  sda_output_pin.configure({
    .resistor = hal::pin_resistor::pull_up,
    .open_drain = true,
  });
  scl_output_pin.configure({
    .resistor = hal::pin_resistor::pull_up,
    .open_drain = true,
  });
  static hal::bit_bang_i2c::pins bit_bang_pins{
    .sda = &sda_output_pin,
    .scl = &scl_output_pin,
  };
  static hal::bit_bang_i2c bit_bang_i2c(bit_bang_pins, steady_clock);
  p_resource.i2c = &bit_bang_i2c;

  static hal::stm32f1::output_pin spi_chip_select('A', 4);
  p_resource.spi_chip_select = &spi_chip_select;

#if 0
  static hal::stm32f1::spi spi1(hal::bus<1>,
                                {
                                  .clock_rate = 250.0_kHz,
                                  .clock_polarity = false,
                                  .clock_phase = false,
                                });
  p_resource.spi = &spi1;
#endif

  hal::print(uart1, "USB test starting in init...\n");

  using namespace std::chrono_literals;
  static hal::stm32f1::usb usb(steady_clock);
  hal::print(uart1, "USB\n");

  static auto control_endpoint = usb.acquire_control_endpoint();
  static auto serial_data_ep = usb.acquire_bulk_endpoint();
  static auto status_ep = usb.acquire_interrupt_endpoint();
  hal::print(uart1, "ctrl\n");

  control_endpoint.on_request(control_endpoint_handler);
  hal::print(uart1, "on_request\n");

  control_endpoint.connect(true);
  hal::print(uart1, "connect\n");

  // Device Descriptor
  // Device Descriptor (18 bytes)
  uint8_t const device_descriptor[] = {
    0x12,        // bLength (18 bytes)
    0x01,        // bDescriptorType (Device)
    0x00, 0x02,  // bcdUSB (2.0)
    0x00,        // bDeviceClass (0 = defer to interface)
    0x00,        // bDeviceSubClass
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

#if 0
  // Configuration Descriptor (9+9 = 18 bytes total)
  uint8_t const config_descriptor[] = {
    // Configuration Descriptor
    0x09,  // bLength
    0x02,  // bDescriptorType (Configuration)
    18,
    0x00,  // wTotalLength (18 bytes)
    0x01,  // bNumInterfaces
    0x01,  // bConfigurationValue
    0x00,  // iConfiguration (No string)
    0x80,  // bmAttributes (Bus powered)
    0x32,  // bMaxPower (100mA)

    // Interface Descriptor
    0x09,  // bLength
    0x04,  // bDescriptorType (Interface)
    0x00,  // bInterfaceNumber
    0x00,  // bAlternateSetting
    0x00,  // bNumEndpoints
    0x00,  // bInterfaceClass
    0x00,  // bInterfaceSubClass
    0x00,  // bInterfaceProtocol
    0x00   // iInterface (No string)
  };
#else  // CDC ACM Serial
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
    0x07,                            // bLength
    0x05,                            // bDescriptorType (Endpoint)
    status_ep.second.info().number,  // bEndpointAddress (IN 2)
    0x03,                            // bmAttributes (Interrupt)
    static_cast<hal::u8>(status_ep.second.info().size),
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
    0x07,                                // bLength
    0x05,                                // bDescriptorType (Endpoint)
    serial_data_ep.first.info().number,  // bEndpointAddress (OUT + 1)
    0x02,                                // bmAttributes (Bulk)
    static_cast<hal::u8>(serial_data_ep.first.info().size),
    0x00,  // wMaxPacketSize 16
    0x00,  // bInterval (Ignored for Bulk)

    // Endpoint Descriptor (Data IN)
    0x07,                                 // bLength
    0x05,                                 // bDescriptorType (Endpoint)
    serial_data_ep.second.info().number,  // bEndpointAddress (IN + 1)
    0x02,                                 // bmAttributes (Bulk)
    static_cast<hal::u8>(serial_data_ep.second.info().size),
    0x00,  // wMaxPacketSize 16
    0x00   // bInterval (Ignored for Bulk)
  };
#endif

  hal::print<64>(uart1,
                 ">>>> status_ep.second.info().number = 0x%02X\n",
                 status_ep.second.info().number);
  hal::print<64>(uart1,
                 ">>>> status_ep.second.info().size = %d\n",
                 status_ep.second.info().size);
  hal::print<64>(uart1,
                 ">>>> serial_data_ep.first.info().number = 0x%02X\n",
                 serial_data_ep.first.info().number);
  hal::print<64>(uart1,
                 ">>>> serial_data_ep.first.info().size = %d\n",
                 serial_data_ep.first.info().size);
  hal::print<64>(uart1,
                 ">>>> serial_data_ep.second.info().number = 0x%02X\n",
                 serial_data_ep.second.info().number);
  hal::print<64>(uart1,
                 ">>>> serial_data_ep.second.info().size = %d\n",
                 serial_data_ep.second.info().size);

  // String Descriptor 0 (Language ID)
  uint8_t const lang_descriptor[] = {
    0x04,  // bLength
    0x03,  // bDescriptorType (String)
    0x09,
    0x04  // wLANGID[0] (0x0409: English-US)
  };

  // String Descriptor 1 (Manufacturer)
  uint8_t const manufacturer_descriptor[] = {
    22,    // bLength
    0x03,  // bDescriptorType (String)
    'l',  0, 'i', 0, 'b', 0, 'h', 0, 'a', 0,
    'l',  0, ' ', 0, 'i', 0, 'n', 0, 'c', 0,
  };

  // String Descriptor 2 (Product)
  uint8_t const product_descriptor[] = {
    36,          // bLength: 34 bytes (2 + (16 chars * 2))
    0x03,        // bDescriptorType: String descriptor
    'l',  0x00,  // Unicode string: "libhal USB device"
    'i',  0x00, 'b',  0x00, 'h',  0x00, 'a',  0x00, 'l',  0x00, ' ',
    0x00, 'U',  0x00, 'S',  0x00, 'B',  0x00, ' ',  0x00, 'd',  0x00,
    'e',  0x00, 'v',  0x00, 'i',  0x00, 'c',  0x00, 'e',  0x00,
  };

  // String Descriptor 3 (Serial Number)
  constexpr uint8_t serial_descriptor[] = {
    14,    // bLength
    0x03,  // bDescriptorType (String)
    '0',  0, '0', 0, '0', 0, '0', 0, '1', 0, '.', 0,
  };

  // I love that you can just do that
  static_assert(sizeof(serial_descriptor) == serial_descriptor[0]);

  // Example notification with DSR and DCD set
  [[maybe_unused]] static uint8_t constexpr serial_state_notification[] = {
    0xA1,        // bmRequestType (Device to Host | Class | Interface)
    0x20,        // bNotification (SERIAL_STATE)
    0x00, 0x00,  // wValue (zero)
    0x00, 0x00,  // wIndex (Interface number)
    0x02, 0x00,  // wLength (2 bytes of data)
    0x03, 0x00   // serialState (DCD | DSR = 0x03)
  };

  std::array<std::span<uint8_t const>, 4> strings = {
    lang_descriptor,
    manufacturer_descriptor,
    product_descriptor,
    serial_descriptor,
  };

  bool serial_data_available = false;

  serial_data_ep.first.on_receive([&serial_data_available](bulk_receive_tag) {
    serial_data_available = true;
  });

  auto handle_serial = [&serial_data_available]() {
    if (serial_data_available) {
      // Drain endpoint of content...
      serial_data_available = false;
      // Only 3 bytes to test only grabbing a few bytes from the endpoint and
      // still having some remaining.
      std::array<hal::u8, 3> buffer{};
      auto data_received = serial_data_ep.first.read(buffer);
      while (not data_received.empty()) {
        uart1.write(data_received);
        data_received = serial_data_ep.first.read(buffer);
      }
    }
  };

  hal::u8 configuration = 0;

  // CDC Class-Specific Request Codes
  constexpr hal::u8 CDC_SET_LINE_CODING = 0x20;
  constexpr hal::u8 CDC_GET_LINE_CODING = 0x21;
  constexpr hal::u8 CDC_SET_CONTROL_LINE_STATE = 0x22;
  constexpr hal::u8 CDC_SEND_BREAK = 0x23;

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

  bool port_connected = false;
  // Example handler for CDC class-specific setup packets
  auto handle_cdc_setup = [&port_connected](hal::u8 bmRequestType,
                                            hal::u8 bRequest,
                                            hal::u16 wValue) -> bool {
    switch (bRequest) {
      case CDC_GET_LINE_CODING:
        if (bmRequestType == 0xA1) {  // Direction: Device to Host
          // Send current line coding
          control_endpoint.write(line_coding);
          hal::print(uart1, "CDC_GET_LINE_CODING\n");
          return true;
        }
        break;

      case CDC_SET_LINE_CODING:
        if (bmRequestType == 0x21) {      // Direction: Host to Device
          control_endpoint.stall(false);  // ensure RX of endpoint is valid
          while (true) {
            if (not host_command_available) {  // nothing received
              continue;
            }

            auto const host_command = control_endpoint.read();
            if (not host_command) {
              continue;
            }

            host_command_available = false;

            hal::print(uart1, "{{ ");
            for (auto const byte : host_command.value()) {
              hal::print<8>(uart1, "0x%" PRIx8 ", ", byte);
            }
            hal::print(uart1, "}}\n");
            std::copy_n(host_command.value().begin(),
                        line_coding.size(),
                        line_coding.begin());
            control_endpoint.write({});
            break;
          }

          port_connected = true;
          hal::print(uart1, "CDC_SET_LINE_CODING+\n");
          return true;
        }
        break;

      case CDC_SET_CONTROL_LINE_STATE:
        if (bmRequestType == 0x21) {  // Direction: Host to Device
          // wValue contains the control signals:
          // Bit 0: DTR state
          // Bit 1: RTS state
          bool const dtr = wValue & 0x01;
          bool const rts = wValue & 0x02;
          // Handle control line state change
          // Most basic implementation just returns success
          control_endpoint.write({});
          hal::print<32>(uart1, "DTR = %d, RTS = %d\n", dtr, rts);
          return true;
        }
        break;

      case CDC_SEND_BREAK:
        if (bmRequestType == 0x21) {  // Direction: Host to Device
          // wValue contains break duration in milliseconds
          // Most basic implementation just returns success
          control_endpoint.write({});
          hal::print(uart1, "SEND_BREAK\n");
          return true;
        }
        break;
    }

    return false;  // Command not handled
  };

  // Wait for the message number to increment
  auto deadline = hal::future_deadline(steady_clock, 1s);
  auto command_count = 0;

  std::array<std::array<hal::experimental::usb_endpoint*, 2>, 2> map = {
    std::array<hal::experimental::usb_endpoint*, 2>{
      &serial_data_ep.first,
      &serial_data_ep.second,
    },
    {
      &status_ep.first,
      &status_ep.second,
    }
  };

  while (true) {
    if (configuration == 1 && port_connected) {
      handle_serial();
      // Send a '.' every second
      if (deadline < steady_clock.uptime()) {
        using namespace std::string_view_literals;
        try {
          serial_data_ep.second.write(hal::as_bytes("."sv));
        } catch (hal::timed_out const&) {
          hal::print(
            uart1, "\n\n\033[48;5;9mEP TIMEOUT! PORT DISCONNECTED!\033[0m\n\n");
          port_connected = false;
          continue;
        }
        hal::print(uart1, ">");
        deadline = hal::future_deadline(steady_clock, 1s);
      }
    }

    if (not host_command_available) {  // nothing received
      continue;
    }

    auto const host_command = control_endpoint.read();
    if (not host_command) {
      host_command_available = false;
      continue;
    }

    host_command_available = false;

    auto const& buffer = host_command.value();
    auto print_buffer = [&buffer]() {
      for (auto const byte : buffer) {
        hal::print<8>(uart1, "0x%" PRIx8 ", ", byte);
      }
    };

    // Extract bmRequestType and bRequest
    hal::u8 const bmRequestType = buffer[0];
    hal::u8 const bRequest = buffer[1];
    std::size_t const wValue = (buffer[3] << 8) | buffer[2];
    std::size_t const wIndex = (buffer[5] << 8) | buffer[4];
    std::size_t const wLength = (buffer[7] << 8) | buffer[6];
    try {
      if (bmRequestType == 0x00 && bRequest == 0x05) {
        control_endpoint.write({});
        control_endpoint.set_address(buffer[2]);
        hal::print<32>(uart1, "ZLP+SET_ADDR[%d]\n", buffer[2]);
      } else if (bmRequestType == 0x00 && bRequest == 0x09) {
        hal::u8 const descriptor_index = buffer[2];
        // SET_CONFIGURATION
        configuration = descriptor_index;
        control_endpoint.write({});
        serial_data_ep.first.reset();
        status_ep.first.reset();
        hal::print<16>(uart1, "SC%" PRIu8 "\n", descriptor_index);
      } else if (bmRequestType == 0x80) {  // Device-to-host
        if (bRequest == 0x06) {            // GET_DESCRIPTOR
          hal::u8 const descriptor_index = buffer[2];
          hal::u8 const descriptor_type = buffer[3];
          switch (descriptor_type) {
            case 0x01: {  // Device Descriptor
              hal::print<16>(uart1, "DD%" PRIu16 "\n", wLength);
              control_endpoint.write(
                std::span(device_descriptor).first(wLength));
              hal::print(uart1, "DD+\n");
              break;
            }
            case 0x02:  // Configuration Descriptor
            {
              hal::print<32>(uart1, "CD%" PRIu16 "\n", wLength);
              control_endpoint.write(
                std::span(config_descriptor).first(wLength));
              hal::print(uart1, "CD+\n");
              break;
            }
            case 0x03: {  // String Descriptor
              hal::print<16>(
                uart1, "S%" PRIu8 ":%" PRIu16 "\n", descriptor_index, wLength);
              auto const str = strings.at(descriptor_index);
              auto const first = std::min(str.size(), wLength);
              auto const payload_span = str.first(first);
              control_endpoint.write(payload_span);
              hal::print(uart1, "S+\n");
              break;
            }
            default:
              hal::print(uart1, "bmRequestType?\n");
              break;
          }
        }
      } else if (hal::bit_extract<hal::bit_mask::from(5, 6)>(bmRequestType) ==
                 0x1) {
        // Class-specific request
        auto const handled = handle_cdc_setup(bmRequestType, bRequest, wValue);
        if (handled) {
          hal::print(uart1, "CDC-CLASS+\n");
        } else {
          hal::print(uart1, "CDC-CLASS!\n");
        }
      } else if (bmRequestType == 0x02) {
        hal::print(uart1, "[EP");

        // Standard USB request codes
        constexpr hal::u8 get_status = 0x00;
        constexpr hal::u8 clear_feature = 0x01;
        constexpr hal::u8 set_feature = 0x03;

        auto const ep_select = wIndex & 0xF;
        auto const direction = hal::bit_extract<hal::bit_mask::from(7)>(wIndex);
        hal::print<8>(uart1, "%" PRIu8, ep_select);
        hal::print<8>(uart1, "]:[%" PRIu8 "]", direction);
        auto& selected_ep = *map.at(ep_select).at(direction);

        switch (bRequest) {
          case get_status:
            control_endpoint.write(
              std::to_array<hal::byte>({ selected_ep.info().stalled, 0 }));
            hal::print<8>(uart1, "STALLED:%d\n", selected_ep.info().stalled);
            break;
          case clear_feature:
            selected_ep.stall(false);
            control_endpoint.write({});
            hal::print(uart1, "CLEAR\n");
            break;
          case set_feature:
            hal::print(uart1, "SET_FEATURE\n");
            break;
          default:
            hal::print<16>(uart1, "DEFAULT:0x%02X\n", bRequest);
            break;
        }
        hal::print(uart1, "\n");
      }

      hal::print<16>(uart1, "COMMAND[%zu]: {", command_count++);
      print_buffer();
      hal::print(uart1, "}\n\n");
    } catch (hal::timed_out const&) {
      hal::print(uart1, "EP write operation timed out!\n");
    }
  }
}
