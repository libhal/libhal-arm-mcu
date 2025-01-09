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

std::array<hal::byte, 16> buffer{};
std::span<hal::byte> usable_data(buffer);
bool act = false;

void my_control_handler(std::span<hal::byte> p_data)
{
  auto const min = std::min(buffer.size(), p_data.size());
  std::copy_n(p_data.begin(), min, buffer.begin());
  usable_data = buffer;
  usable_data = usable_data.first(min);
  act = true;
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

  hal::stm32f1::output_pin signal('A', 0);
  signal.level(true);
  hal::stm32f1::output_pin signal2('A', 15);
  signal.level(true);
  hal::stm32f1::usb usb(steady_clock);
  hal::print(uart1, "USB\n");
  auto control_endpoint = usb.acquire_control_endpoint();
  hal::print(uart1, "ctrl\n");

  using namespace std::chrono_literals;
  control_endpoint.on_request(my_control_handler);
  hal::print(uart1, "on_request\n");

  control_endpoint.connect(true);
  hal::print(uart1, "connect\n");

  // Device Descriptor
  uint8_t const device_descriptor[] = {
    0x12,        // bLength
    0x01,        // bDescriptorType (Device)
    0x00, 0x02,  // bcdUSB (USB 2.0)
    0x00,  // bDeviceClass (Use class information in the Interface Descriptors)
    0x00,  // bDeviceSubClass
    0x00,  // bDeviceProtocol
    16,    // bMaxPacketSize0 (64 bytes)
    0xAD, 0xDE,  // idVendor (0xDEAD)
    0xEF, 0xBE,  // idProduct (0xBEEF)
    0x00, 0x01,  // bcdDevice
    0x01,        // iManufacturer (String Index)
    0x02,        // iProduct (String Index)
    0x03,        // iSerialNumber (String Index)
    0x01         // bNumConfigurations
  };

  // String Descriptor 0 (Language ID)
  [[maybe_unused]] uint8_t const lang_descriptor[] = {
    0x04,  // bLength
    0x03,  // bDescriptorType (String)
    0x09,
    0x04  // wLANGID[0] (0x0409: English-US)
  };

  // String Descriptor 1 (Manufacturer)
  [[maybe_unused]] uint8_t const manufacturer_descriptor[] = {
    0x18,  // bLength
    0x03,  // bDescriptorType (String)
    'l',  0, 'i', 0, 'b', 0, 'h', 0, 'a', 0,
    'l',  0, ' ', 0, 'i', 0, 'n', 0, 'c', 0
  };

  // String Descriptor 2 (Product)
  [[maybe_unused]] uint8_t const product_descriptor[] = {
    0x18,  // bLength
    0x03,  // bDescriptorType (String)
    's',  0, 'u', 0, 'p', 0, 'e', 0, 'r', 0, ' ', 0, 'u', 0, 's', 0, 'b', 0
  };

  // String Descriptor 3 (Serial Number)
  [[maybe_unused]] uint8_t const serial_descriptor[] = {
    0x1A,  // bLength
    0x03,  // bDescriptorType (String)
    '0',  0, '8', 0, '8', 0, '8', 0, '0', 0, '8', 0, '8', 0, '0', 0, '8', 0
  };

  while (true) {
    if (not act) {
      signal2.level(not signal2.level());
      continue;
    }

    if (buffer[0] == 0x00 && buffer[1] == 0x05) {
      control_endpoint.write({});
      control_endpoint.set_address(buffer[2]);
      hal::print(uart1, "ZLP+\n");
      hal::print<16>(uart1, "ADDR (%d)+\n", buffer[2]);
    } else if (buffer[0] == 0x80 && buffer[1] == 0x06 && buffer[2] == 0x00 &&
               buffer[3] == 0x01) {
      hal::print(uart1, "S:DD\n");
      signal.level(not signal.level());
      control_endpoint.write(device_descriptor);
      signal.level(not signal.level());
      hal::print(uart1, "S:DD+\n");
    } else if (buffer[0] == 0x80 && buffer[1] == 0x06 && buffer[2] == 0x00 &&
               buffer[3] == 0x02) {
      hal::print(uart1, "NOICE! Now disconnect!\n");
      control_endpoint.connect(false);
      while (true) {
        continue;
      }
    } else {
      while (true) {
        continue;
      }
    }

    act = false;
  }
}
