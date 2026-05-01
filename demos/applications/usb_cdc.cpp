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
#include <libhal-util/usb/descriptors.hpp>
#include <libhal/pointers.hpp>
#include <libhal/scatter_span.hpp>
#include <libhal/timeout.hpp>
#include <libhal/units.hpp>
#include <libhal/usb.hpp>

#include <libhal-arm-mcu/stm32f1/usb.hpp>

#include <resource_list.hpp>

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
    return false;
  }
  bool driver_handle_request(hal::usb::setup_packet const& p_setup,
                             hal::usb::endpoint_io& p_endpoint) override
  {
    return false;
  }

  hal::strong_ptr<hal::usb::bulk_out_endpoint> m_serial_data_ep_out;
  hal::strong_ptr<hal::usb::bulk_in_endpoint> m_serial_data_ep_in;
  hal::strong_ptr<hal::usb::interrupt_out_endpoint> m_status_ep_out;
  hal::strong_ptr<hal::usb::interrupt_in_endpoint> m_status_ep_in;
  descriptor_start m_start;
};

void application()
{
  using namespace hal::literals;

  auto clock = resources::clock();
  auto console = resources::console();

  hal::print(*console, "Staring USB CDC application...\n");

  using namespace std::chrono_literals;
  using namespace std::literals;

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
}
