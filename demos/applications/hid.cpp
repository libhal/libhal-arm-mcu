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

#include <array>
#include <cstddef>
#include <libhal/steady_clock.hpp>
#include <string_view>

#include <libhal-util/as_bytes.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal-util/usb.hpp>
#include <libhal-util/usb/descriptors.hpp>
#include <libhal/error.hpp>
#include <libhal/pointers.hpp>
#include <libhal/scatter_span.hpp>
#include <libhal/timeout.hpp>
#include <libhal/units.hpp>
#include <libhal/usb.hpp>

#include <libhal-arm-mcu/stm32f1/usb.hpp>

#include <resource_list.hpp>
#include <utility>

hal::optional_ptr<hal::serial> g_console;

class usb_hid_mouse : public hal::usb::interface
{
public:
  usb_hid_mouse(
    hal::strong_ptr<hal::usb::interrupt_in_endpoint> const& p_mouse_data)
    : m_mouse_data(p_mouse_data)
  {
  }

  hal::usize write(hal::scatter_span<hal::byte const> p_buffer)
  {
    if (m_connected) {
      hal::v5::write_and_flush(*m_mouse_data, p_buffer);
      return p_buffer.size();
    }
    return 0;
  }

  bool ready()
  {
    return m_connected;
  }

private:
  static constexpr auto hid_descriptor = std::to_array<hal::u8>({
    0x09,  // bLength: 9
    0x21,  // bDescriptorType: 0x21 (HID)
    0x11,  // bcdHID: 0x0111 (HID spec release number)
    0x01,  // bcdHID: 0x0111
    0x00,  // bCountryCode: Not Supported (0x00)
    0x01,  // bNumDescriptors: (num of subordinate report and physical
           // descriptors)
    0x22,  // bDescriptorType: HID Report (0x22)
    0x2C,  // wDescriptorLength: (num bytes in report descriptor) 44
    0x00   // wDescriptorLength
  });
  hal::usb::interface::descriptor_count driver_write_descriptors(
    descriptor_start p_start,
    hal::usb::endpoint_io& p_endpoint) override
  {
    // As required by the rules of "descriptor_start", the interface and string
    // numbers should be checked and if they are set, then those values must be
    // stored so they are used later by the enumerator. This pushes the cost of
    // remember which interface is what to the usb::interface implementations vs
    // the enumerator.
    if (p_start.interface.has_value()) {
      m_start.interface = p_start.interface;
    }

    // In the event that the interface value is not set, it is always valid to
    // use the 0 index for the interface. Prefer to use 0 since thats simple to
    // work with.
    auto const start = m_start.interface.value_or(0);

    auto const interface_descriptor =
      hal::v5::usb::generate_interface_descriptor({
        .interface_number = start,
        .alternate_setting = 0x00,
        .num_endpoints = 1,
        .interface_class = hal::v5::usb::class_code::hid,
        .interface_subclass = 0x00,      // boot interface
        .interface_protocol = 0x02,      // mouse
        .interface_string_index = 0x00,  // no strings
      });

    // auto const endpoint_descriptor = std::array<hal::u8, 7>{
    //   0x07,                         // bLength: 7
    //   0x05,                         // bDescriptorType: 0x05 (ENDPOINT)
    //   m_mouse_data->info().number,  // bEndpointAddress: 0x82  IN  Endpoint:2
    //   0x03,  // bmAttributes: 0000 0011 = Interrupt-Transfer
    //   0x03,  // wMaxPacketSize: 3 bytes
    //   0x00,  // wMaxPacketSize:
    //   0x01   // bInterval: 1
    // };

    auto const endpoint_descriptor =
      hal::v5::usb::generate_endpoint_descriptor(*m_mouse_data, 1);

    // Combine all of the endpoints together in one big scatter array
    auto const descriptors = hal::make_scatter_array<hal::u8 const>(
      interface_descriptor, hid_descriptor, endpoint_descriptor);

    p_endpoint.write(descriptors);
    // Report back to the enumerator that we have 1 interface available
    return hal::usb::interface::descriptor_count{ .interface = 1, .string = 0 };
  }

  bool driver_write_string_descriptor(hal::u8, hal::usb::endpoint_io&) override
  {
    // NOTE: This interface has NO string descriptors so this API always
    // returns false.
    return false;
  }

  void driver_handle_host_event(hal::v5::usb::host_event p_event) override
  {
    if (p_event == hal::v5::usb::host_event::enumerated) {
      hal::print(*g_console, "{ENU}");
      m_enumerated = true;
      m_mouse_data->reset();
    }
    if (p_event == hal::v5::usb::host_event::reset) {
      hal::print(*g_console, "{RST}");
      m_enumerated = false;
      m_mouse_data->reset();
    }
  }

  bool driver_handle_request(hal::usb::setup_packet const& p_setup,
                             hal::usb::endpoint_io& p_endpoint) override
  {
    hal::print(*g_console, "[R]");

    constexpr hal::u8 get_hid_report = 0x22;
    [[maybe_unused]] constexpr hal::u8 set_idle = 0x21;
    [[maybe_unused]] constexpr hal::u8 clear_feature = 0x01;
    using namespace hal::v5::usb;
    if (p_setup.request() == 0x06) {
      hal::print(*g_console, "{R06}");
      if (p_setup.bm_request_type() == 0x21) {
        return true;
      }
      if (p_setup.bm_request_type() == 0x81) {
        hal::print(*g_console, "{T81}");
        if (p_setup.value_bytes()[0] == get_hid_report or
            p_setup.value_bytes()[1] == get_hid_report) {
          hal::print<32>(*g_console, "{0x%04X}", p_setup.value());
          // To get a request from the host means that we can assume we are
          // connected.
          auto const report_descriptor = std::to_array<hal::u8>({
            0x05,  // usage page
            0x01,  // generic desktop
            0x09,  // usage
            0x02,  // mouse
            0xA1,  // collection
            0x01,  // application
            // 0x85,  // report id (1 byte, global, report ID) might not need
            // this? 0x02,  // 2
            0x09,  // usage
            0x01,  // pointer
            0xA1,  // collection
            0x00,  // physical

            0x05,  // usage page
            0x01,  // generic desktop
            0x09,  // usage
            0x30,  // X
            0x09,  // usage
            0x31,  // Y
            0x15,  // logical min
            0x81,  // -127
            0x25,  // logical max
            0x7F,  // 127
            0x75,  // report size
            0x08,  // 8 bits
            0x95,  // report count
            0x02,  // 2
            0x81,  // input
            0x06,  // data, var, rel

            0x05,  // usage page
            0x09,  // button
            0x19,  // usage min
            0x01,  // button 1
            0x29,  // usage max
            0x03,  //  button 3
            0x15,  // logical min
            0x00,  // 0
            0x25,  // logical max
            0x01,  // 1
            0x95,  // report count
            0x03,  // 3
            0x75,  // report size
            0x01,  // 1
            0x81,  // input
            0x02,  // data, var, abs

            0xC0,  // end collection
            0xC0   // end collection
          });

          hal::print(*g_console, "REP:");
          auto const bytes_written = p_endpoint.write(
            hal::make_scatter_array<hal::u8 const>(report_descriptor));
          hal::print<64>(
            *g_console, "(%d/%d)\n", bytes_written, p_setup.length());
          m_connected = true;
          return true;
        }
      }
    }

    if (p_setup.request() == 0x0A) {
      return true;
    }

    hal::print<64>(*g_console,
                   "SKIP:bm:0x%02X,r:0x%02X,v:0x%04X,i:0x%04X\n",
                   p_setup.bm_request_type(),
                   p_setup.request(),
                   p_setup.value(),
                   p_setup.index());
    return false;
  }

  hal::strong_ptr<hal::usb::interrupt_in_endpoint> m_mouse_data;
  descriptor_start m_start{};
  bool m_connected{};
  bool m_enumerated = false;
};

std::string_view to_string_view(std::errc p_errc);

void application()
{
  using namespace hal::literals;
  using namespace std::literals;
  using namespace std::chrono_literals;

  auto clock = resources::clock();
  auto console = resources::console();
  g_console = console;

  auto allocator = resources::driver_allocator();
  auto control_endpoint = resources::usb_control_endpoint();
  auto serial_host_ep_out = resources::usb_bulk_out_endpoint1();
  auto serial_host_ep_out2 = resources::usb_bulk_out_endpoint2();
  auto serial_host_ep_in = resources::usb_bulk_in_endpoint1();
  auto serial_host_ep_in2 = resources::usb_bulk_in_endpoint2();
  auto status_ep_in = resources::usb_interrupt_in_endpoint1();
  auto status_ep_in2 = resources::usb_interrupt_in_endpoint2();

  hal::print<512 * 2>(*console,
                      "+---------------------+--------+------+\n"
                      "| Endpoint Name       | Number | Size |\n"
                      "+---------------------+--------+------+\n"
                      "| %-19s |  0x%02X  | %4zu |\n"
                      "| %-19s |  0x%02X  | %4zu |\n"
                      "| %-19s |  0x%02X  | %4zu |\n"
                      "| %-19s |  0x%02X  | %4zu |\n"
                      "| %-19s |  0x%02X  | %4zu |\n"
                      "| %-19s |  0x%02X  | %4zu |\n"
                      "+---------------------+--------+------+\n\n",
                      "serial_host_ep_out",
                      serial_host_ep_out->info().number,
                      serial_host_ep_out->info().size,
                      "serial_host_ep_out2",
                      serial_host_ep_out2->info().number,
                      serial_host_ep_out2->info().size,
                      "status_ep_in",
                      status_ep_in->info().number,
                      status_ep_in->info().size,
                      "status_ep_in2",
                      status_ep_in2->info().number,
                      status_ep_in2->info().size,
                      "serial_host_ep_in",
                      serial_host_ep_in->info().number,
                      serial_host_ep_in->info().size,
                      "serial_host_ep_in2",
                      serial_host_ep_in2->info().number,
                      serial_host_ep_in2->info().size);

  auto hid_mouse = hal::make_strong_ptr<usb_hid_mouse>(allocator, status_ep_in);

  hal::usb::inplace_enumerator usb_enumerator(
    control_endpoint,
    {
      .manufacturer = u"Milosoft",
      .product = u"Milosoft MOWS",
      .serial_number = u"0001",
      .vendor_id = 0xDEAD,
      .product_id = 0xBEEF,
      // Takes default for everything else
    },
    hid_mouse);

  auto const mouse_left = std::array<hal::u8, 3>{ 0xC0, 0x00, 0x00 };
  auto const mouse_right = std::array<hal::u8, 3>{ 0x40, 0x00, 0x00 };
  [[maybe_unused]] auto const mouse_left_data =
    hal::make_scatter_array<hal::u8 const>(mouse_left);
  [[maybe_unused]] auto const mouse_right_data =
    hal::make_scatter_array<hal::u8 const>(mouse_right);

  auto const activity_period = 250ms;
  auto dot_deadline = hal::future_deadline(*clock, activity_period);
  auto wait_deadline = hal::future_deadline(*clock, activity_period);

  [[maybe_unused]] bool left = true;
  while (true) {
    try {
      usb_enumerator.process_ctrl_transfer();
      if (usb_enumerator.is_enumerated()) {
        if (clock->uptime() >= dot_deadline) {
          hal::print(*console, "+");
          if (left) {
            auto const size = hid_mouse->write(mouse_left_data);
            if (size == 0) {
              hal::print(*console, "❌");
            } else {
              hal::print(*console, "⬅️");
            }
            left = false;
          } else {
            auto const size = hid_mouse->write(mouse_right_data);
            if (size == 0) {
              hal::print(*console, "❌");
            } else {
              hal::print(*console, "➡️");
            }
            left = true;
          }
          dot_deadline = hal::future_deadline(*clock, activity_period);
        }
      } else {
        if (clock->uptime() >= wait_deadline) {
          hal::print(*console, "-");
          wait_deadline = hal::future_deadline(*clock, activity_period);
        }
      }
    } catch (hal::exception const& p_error) {
      hal::print<64>(
        *console, "\nException Error Code '%d' caught\n", p_error.error_code());
      dot_deadline = hal::future_deadline(*clock, activity_period);
      wait_deadline = hal::future_deadline(*clock, activity_period);
    }
  }
}
