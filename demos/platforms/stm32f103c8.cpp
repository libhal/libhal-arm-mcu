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
#include <libhal-util/bit_bang_i2c.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

#include <resource_list.hpp>

using setup_command_buffer = std::array<hal::byte, 16>;
std::array<setup_command_buffer, 16> buffer_history{};
std::size_t history_idx = 0;
std::size_t current_idx = 0;

enum class enumeration_state
{
  pre_address,
  post_address,
  sending_device_descriptor,
  post_device_descriptor,
  sending_configure_descriptor,
  post_configure_descriptor,
};
enumeration_state state = enumeration_state::pre_address;

void my_control_handler(std::span<hal::byte> p_data)
{
  // Ignore zero length messages from the host.
  if (p_data.empty()) {
    return;
  }
  auto& buffer = buffer_history[history_idx++ % buffer_history.size()];
  auto const min = std::min(buffer.size(), p_data.size());
  std::copy_n(p_data.begin(), min, buffer.begin());
};

auto get_latest_host_command() -> auto&
{
  decltype(current_idx) setup_command_select = 0;
  if (history_idx == 0) {
    setup_command_select = buffer_history.size() - 1;
  } else {
    setup_command_select = history_idx - 1;
  }

  return buffer_history[setup_command_select % buffer_history.size()];
};

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

  hal::stm32f1::usb usb(steady_clock);
  hal::print(uart1, "USB\n");
  auto control_endpoint = usb.acquire_control_endpoint();
  hal::print(uart1, "ctrl\n");

  using namespace std::chrono_literals;
  control_endpoint.on_request(my_control_handler);
  hal::print(uart1, "on_request\n");

  control_endpoint.connect(true);
  hal::print(uart1, "connect\n");
  control_endpoint.enable_rx();

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
    'a',  0, 'b', 0, 'c', 0, 'd', 0, 'e', 0, 'f', 0,
  };

  // I love that you can just do that
  static_assert(sizeof(serial_descriptor) == serial_descriptor[0]);

  std::array<std::span<uint8_t const>, 4> strings = {
    lang_descriptor,
    manufacturer_descriptor,
    product_descriptor,
    serial_descriptor,
  };

  while (true) {
    // Wait for a new setup message to come in.
    if (current_idx == history_idx) {
      continue;
    }
    current_idx = history_idx;
    auto& buffer = get_latest_host_command();

    // Extract bmRequestType and bRequest
    hal::u8 const bmRequestType = buffer[0];
    hal::u8 const bRequest = buffer[1];

    if (bmRequestType == 0x00 && bRequest == 0x05) {
      control_endpoint.write({});
      control_endpoint.set_address(buffer[2]);
      hal::print<32>(uart1, "ZLP+SET_ADDR[%d]\n", buffer[2]);
    } else if (bmRequestType == 0x00 && bRequest == 0x09) {
      hal::u8 const descriptor_index = buffer[2];
      // SET_CONFIGURATION
      control_endpoint.write({});
      hal::print<16>(uart1, "SC%" PRIu8 "\n", descriptor_index);
    } else if (bmRequestType == 0x80) {  // Device-to-host
      if (bRequest == 0x06) {            // GET_DESCRIPTOR
        std::size_t const wLength = (buffer[7] << 8) | buffer[6];
        hal::u8 const descriptor_index = buffer[2];
        hal::u8 const descriptor_type = buffer[3];
        switch (descriptor_type) {
          case 0x01: {  // Device Descriptor
            hal::print<16>(uart1, "DD%" PRIu16 "\n", wLength);
            control_endpoint.write(std::span(device_descriptor).first(wLength));
            hal::print(uart1, "DD+\n");
            break;
          }
          case 0x02:  // Configuration Descriptor
          {
            state = enumeration_state::sending_configure_descriptor;
            hal::print<16>(uart1, "CD%" PRIu16 "\n", wLength);
            control_endpoint.write(std::span(config_descriptor).first(wLength));
            state = enumeration_state::post_configure_descriptor;
            hal::print(uart1, "CD!\n");
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
            break;
        }
      }
    }

    hal::print<16>(uart1, "ACT[%zu]\n", history_idx);
  }
}
