#include "applications/descriptors.hpp"
#include "applications/utils.hpp"
#include "resource_list.hpp"
#include <array>
#include <cinttypes>
#include <cstddef>
#include <libhal-util/as_bytes.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal-util/usb/endpoints.hpp>
#include <libhal/pointers.hpp>
#include <libhal/scatter_span.hpp>
#include <libhal/serial.hpp>
#include <libhal/units.hpp>
#include <libhal/usb.hpp>
#include <memory_resource>
#include <optional>
#include <string_view>

#include "enumerator.hpp"

namespace hal::v5::usb {
namespace constants {
constexpr hal::u8 cdc_set_line_coding = 0x20;
constexpr hal::u8 cdc_get_line_coding = 0x21;
constexpr hal::u8 cdc_set_control_line_state = 0x22;
constexpr hal::u8 cdc_send_break = 0x23;
}  // namespace constants

class cdc_serial : public interface
{
public:
  ~cdc_serial() override = default;

  // NOLINTBEGIN
  cdc_serial(std::u16string_view p_iad_name,
             std::u16string_view p_ctrl_name,
             std::u16string_view p_data_name,
             strong_ptr<bulk_out_endpoint> const& p_ep_data_out,
             strong_ptr<bulk_in_endpoint> const& p_ep_data_in,
             strong_ptr<interrupt_out_endpoint> const& p_ep_status_out,
             strong_ptr<interrupt_in_endpoint> const& p_ep_status_in,
             strong_ptr<hal::serial> const& p_console)
    : m_iad(0x2, 0x2, 0x1)
    , m_ctrl_iface(0x0, 0x2, 0x2, 0x1)
    , m_data_iface(0x0, 0xA, 0x0, 0x0)
    , m_iad_name(p_iad_name)
    , m_ctrl_name(p_ctrl_name)
    , m_data_name(p_data_name)
    , m_ep_data_out(p_ep_data_out)
    , m_ep_data_in(p_ep_data_in)
    , m_ep_status_out(p_ep_status_out)
    , m_ep_status_in(p_ep_status_in)
    , m_console(p_console) {};

  struct iface_desc
  {
    // Full Interface Descriptor size: 9 bytes (type + length in header)
    std::array<byte, static_cast<size_t>(constants::inferface_desc_size)>
      data{};

    // Default values for USB CDC ACM:
    // bLength = 9, bDescriptorType = 4
    constexpr iface_desc()
      : data{ 0x09, 0x04, 0, 0, 0, 0, 0, 0, 0 }
    {
    }

    // Customizable constructor (alt setting, class, subclass, protocol)
    constexpr iface_desc(byte alt_setting,
                         byte iface_class,
                         byte iface_subclass,
                         byte iface_protocol)
      : data{ 0x09,           0x04,           0, alt_setting, 0, iface_class,
              iface_subclass, iface_protocol, 0 }
    {
    }

    // Accessors (now that header is included)
    constexpr byte& bLength()
    {
      return data[0];
    }
    constexpr byte& bDescriptorType()
    {
      return data[1];
    }
    constexpr byte& bInterfaceNumber()
    {
      return data[2];
    }
    constexpr byte& bAlternateSetting()
    {
      return data[3];
    }
    constexpr byte& bNumEndpoints()
    {
      return data[4];
    }
    constexpr byte& bInterfaceClass()
    {
      return data[5];
    }
    constexpr byte& bInterfaceSubClass()
    {
      return data[6];
    }
    constexpr byte& bInterfaceProtocol()
    {
      return data[7];
    }
    constexpr byte& iInterface()
    {
      return data[8];
    }

    // Span conversion operator
    constexpr operator std::span<byte const>() const
    {
      return std::span<byte const>(data.data(), data.size());
    }
  };

  struct iad_desc
  {
    // Full IAD Descriptor size: 8 bytes (type + length in header)
    std::array<byte, static_cast<size_t>(constants::iad_desc_size)> data{};

    // Default values for USB CDC ACM:
    // bLength = 8, bDescriptorType = 0x0B
    constexpr iad_desc()
      : data{ 0x08, 0x0B, 0, 0, 0, 0, 0, 0 }
    {
    }

    // Customizable constructor (class, subclass, protocol)
    constexpr iad_desc(byte function_class,
                       byte function_subclass,
                       byte function_protocol)
      : data{
        0x08, 0x0B, 0, 0, function_class, function_subclass, function_protocol,
        0
      }
    {
    }

    // Accessors
    constexpr byte& bLength()
    {
      return data[0];
    }
    constexpr byte& bDescriptorType()
    {
      return data[1];
    }
    constexpr byte& bFirstInterface()
    {
      return data[2];
    }
    constexpr byte& bInterfaceCount()
    {
      return data[3];
    }
    constexpr byte& bFunctionClass()
    {
      return data[4];
    }
    constexpr byte& bFunctionSubClass()
    {
      return data[5];
    }
    constexpr byte& bFunctionProtocol()
    {
      return data[6];
    }
    constexpr byte& iFunction()
    {
      return data[7];
    }

    // Span conversion operator
    constexpr operator std::span<byte const>() const
    {
      return std::span<byte const>(data.data(), data.size());
    }
  };
  // NOLINTEND

private:
  [[nodiscard]] descriptor_count driver_write_descriptors(
    descriptor_start p_start,
    endpoint_writer const& p_callback) override
  {

    constexpr std::array<byte const, 5> cdc_hdr_func_desc = {
      0x05,  // bLength
      0x24,  // bDescriptorType (CS_INTERFACE)
      0x00,  // bDescriptorSubtype (Header)
      0x10,
      0x01,  // bcdCDC (1.10)
    };

    if (p_start.interface.has_value()) {
      byte counter = p_start.interface.value();
      m_ctrl_iface.bInterfaceNumber() = counter;
      counter += 1;
      m_data_iface.bInterfaceNumber() = counter;
    }

    m_ctrl_iface.bNumEndpoints() = 1;
    m_data_iface.bNumEndpoints() = 2;

    // Set IAD values
    m_iad.bFirstInterface() = m_ctrl_iface.bInterfaceNumber();
    m_iad.bInterfaceCount() = 2;

    auto str_optional = p_start.string;
    if (str_optional.has_value()) {
      m_iad.iFunction() = str_optional.value();
      str_optional.value() += 1;
    }

    // Populate iface string indexes
    if (str_optional.has_value()) {
      m_ctrl_iface.iInterface() = str_optional.value();
      str_optional.value() += 1;
      m_data_iface.iInterface() = str_optional.value();
    }

    // Declare CDC descriptors
    constexpr std::array<byte const, 4> cdc_asm_func_desc = {
      0x04,  // bLength
      0x24,  // bDescriptorType (CS_INTERFACE)
      0x02,  // bDescriptorSubtype (Abstract Control Management)
      0x02,  // bmCapabilities
    };

    // Set bSubordinateInterface0 with the Control Interface's index
    std::array<byte const, 5> cdc_union_func_desc{
      0x05,                             // bLength
      0x24,                             // bDescriptorType (CS_INTERFACE)
      0x06,                             // bDescriptorSubtype (Union)
      0x00,                             // bControlInterface
      m_data_iface.bInterfaceNumber(),  // bSubordinateInterface0
    };

    constexpr std::array<byte const, 5> cdc_call_mgnt_func_desc{
      // CDC Call Management Functional Descriptor
      0x05,  // bLength
      0x24,  // bDescriptorType (CS_INTERFACE)
      0x01,  // bDescriptorSubtype (Call Management)
      0x00,  // bmCapabilities
      0x01,  // bDataInterface
    };

    // Declare endpoint descriptor arrays
    using endpoint_desc =
      std::array<byte const, static_cast<byte>(constants::endpoint_desc_size)>;

    // Control Iface Endpoints
    endpoint_desc ctrl_in_desc{ static_cast<byte>(
                                  constants::endpoint_desc_size),
                                static_cast<byte>(descriptor_type::endpoint),
                                m_ep_status_in->info().number,
                                0x03,
                                static_cast<byte>(m_ep_status_in->info().size),
                                0x00,
                                0x10 };

    // Data Iface Endpoints
    endpoint_desc data_out_desc{
      static_cast<byte>(constants::endpoint_desc_size),
      static_cast<byte>(descriptor_type::endpoint),
      m_ep_data_out->info().number,  // bEndpointAddress (OUT + 1)
      0x02,                          // bmAttributes (Bulk)
      static_cast<hal::u8>(m_ep_data_out->info().size),
      0x00,  // wMaxPacketSize 16
      0x00,  // bInterval (Ignored for Bulk)
    };

    endpoint_desc data_in_desc{
      static_cast<byte>(constants::endpoint_desc_size),
      static_cast<byte>(descriptor_type::endpoint),
      m_ep_data_in->info().number,  // bEndpointAddress (IN + 1)
      0x02,                         // bmAttributes (Bulk)
      static_cast<hal::u8>(m_ep_data_in->info().size),
      0x00,  // wMaxPacketSize 16
      0x00   // bInterval (Ignored for Bulk)
    };

    // Assemble payload

    auto payload = make_scatter_bytes(m_iad,
                                      // Control
                                      m_ctrl_iface,
                                      cdc_hdr_func_desc,
                                      cdc_asm_func_desc,
                                      cdc_union_func_desc,
                                      cdc_call_mgnt_func_desc,
                                      ctrl_in_desc,
                                      // Data
                                      m_data_iface,
                                      data_out_desc,
                                      data_in_desc);

    p_callback(payload);
    return { .interface = 2, .string = 3 };
  }

  [[nodiscard]] bool driver_write_string_descriptor(
    u8 p_index,
    endpoint_writer const& p_callback) override
  {
    std::array<byte, 2> hdr{ 0, static_cast<byte>(descriptor_type::string) };
    std::u16string_view sv;
    if (p_index == m_iad.iFunction()) {
      hdr[0] = p_index;
      sv = m_iad_name;
    } else if (p_index == m_ctrl_iface.iInterface()) {
      hdr[0] = p_index;
      sv = m_ctrl_name;
    } else if (p_index == m_data_iface.iInterface()) {
      hdr[0] = p_index;
      sv = m_data_name;
    } else {
      return false;
    }

    auto sv_bytes = hal::as_bytes(sv);
    auto payload = make_scatter_bytes(hdr, sv_bytes);
    p_callback(payload);
    return true;
  }

  optional_ptr<endpoint> select_endpoint(byte p_index)
  {
    if (p_index == m_ep_data_in->info().number) {
      return m_ep_data_in;
    } else if (p_index == m_ep_data_out->info().number) {
      return m_ep_data_out;
    } else if (p_index == m_ep_status_in->info().number) {
      return m_ep_status_in;
    } else if (p_index == m_ep_status_out->info().number) {
      return m_ep_status_out;
    } else {
      return nullptr;
    }
  }

  bool handle_cdc_requests(setup_packet const& p_setup,
                           endpoint_writer const& p_callback)
  {
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

    switch (p_setup.request) {
      case constants::cdc_get_line_coding: {  // Device to Host
        if (!p_setup.is_device_to_host()) {
          return false;
        }

        auto payload = make_scatter_bytes(line_coding);
        p_callback(payload);
        return true;
      }

      case constants::cdc_set_line_coding: {  // Host to Device
        if (p_setup.is_device_to_host()) {
          return false;
        }

        std::array<byte, 8> payload;
        auto read_buf = make_scatter_bytes(payload);

        // Acquire data, assumed to have read a proper payload, last byte
        // are for amt read, it should never be larger than 8
        p_callback(read_buf);

        hal::print(*m_console, "{{");
        for (auto const byte : payload) {
          hal::print<8>(*m_console, "0x%" PRIx8 ", ", byte);
        }
        hal::print(*m_console, "}}\n");
        std::copy_n(payload.begin(), line_coding.size(), line_coding.begin());
      }

      case constants::cdc_set_control_line_state: {  // Host to Device
        if (p_setup.is_device_to_host()) {
          return false;
        }
        // wValue contains the control signals:
        // Bit 0: DTR state
        // Bit 1: RTS state
        bool const dtr = p_setup.value & 0x01;
        bool const rts = p_setup.value & 0x02;
        // Handle control line state change
        // Most basic implementation just returns success
        p_callback({});
        hal::print<32>(*m_console, "DTR = %d, RTS = %d\n", dtr, rts);
        return true;
      }

      case constants::cdc_send_break: {      // Device to Host
        if (!p_setup.is_device_to_host()) {  // Direction: Host to Device
          return false;
        }
        // wValue contains break duration in milliseconds
        // Most basic implementation just returns success
        p_callback({});
        hal::print(*m_console, "SEND_BREAK\n");
        return true;
      }

      default:
        return false;
    }
    return false;  // Fail safe
  }

  bool driver_handle_request(setup_packet const& p_setup,
                             endpoint_writer const& p_callback) override
  {
    auto pkt_type = determine_standard_request(p_setup);
    if (pkt_type == standard_request_types::invalid) {
      return handle_request(p_setup, p_callback);
    }

    switch (pkt_type) {
      case standard_request_types::get_interface:
      case standard_request_types::set_interface:
      case standard_request_types::set_feature:
        return true;
      case standard_request_types::clear_feature: {
        if (p_setup.get_recipient() == setup_packet::recipient::endpoint) {
          auto ep = select_endpoint(p_setup.index & 0xF);
          if (!ep) {
            return false;
          }
          ep->stall(false);
        }
        return true;
      }

      case standard_request_types::get_status: {
        if (p_setup.get_recipient() == setup_packet::recipient::endpoint) {
          auto const ep_idx = p_setup.index & 0xF;
          optional_ptr<endpoint> ep = select_endpoint(ep_idx);
          if (!ep) {
            return false;
          }
          auto payload = make_scatter_bytes(std::to_array<byte const>(
            { static_cast<byte>(ep->info().stalled), 0 }));
          p_callback(payload);
          return true;
        }

        return false;
      }

      default:
        return false;
    }
  }

  iad_desc m_iad;
  iface_desc m_ctrl_iface;
  iface_desc m_data_iface;

  std::u16string_view m_iad_name;
  std::u16string_view m_ctrl_name;
  std::u16string_view m_data_name;

  strong_ptr<bulk_out_endpoint> m_ep_data_out;
  strong_ptr<bulk_in_endpoint> m_ep_data_in;
  strong_ptr<interrupt_out_endpoint> m_ep_status_out;
  strong_ptr<interrupt_in_endpoint> m_ep_status_in;
  strong_ptr<hal::serial> m_console;
};
}  // namespace hal::v5::usb

namespace hal5 = hal::v5;

void application()
{
  namespace usb = hal5::usb;
  namespace pmr = std::pmr;
  using namespace hal::v5::literals;
  using namespace std::chrono_literals;
  using namespace std::string_view_literals;

  auto const console = resources::console();
  auto const clk = resources::clock();
  auto const pool = resources::driver_allocator();

  auto manu_str = u"libhal"sv;
  auto prod_str = u"my epic cdc device!!"sv;
  auto sn_str = u"42069"sv;

  auto dev = hal::v5::make_strong_ptr<usb::device>(
    pool,
    usb::device::device_arguments{ .p_bcd_usb = 0x2,
                                   .p_device_class =
                                     usb::class_code::use_interface_descriptor,
                                   .p_device_subclass = 0,
                                   .p_device_protocol = 0,
                                   .p_id_vendor = 0x1234,
                                   .p_id_product = 0x5678,
                                   .p_bcd_device = 0x1,
                                   .p_manufacturer = manu_str,
                                   .p_product = prod_str,
                                   .p_serial_number_str = sn_str });
  hal::print(*console, "Device made\n");
  auto conf_str = u"cdc config"sv;
  auto control_endpoint = resources::usb_control_endpoint();
  auto serial_data_ep_out = resources::usb_bulk_out_endpoint1();
  auto serial_data_ep_in = resources::usb_bulk_in_endpoint1();
  auto status_ep_out = resources::usb_interrupt_out_endpoint1();
  auto status_ep_in = resources::usb_interrupt_in_endpoint1();

  hal5::strong_ptr<usb::cdc_serial> usb_console_iface =
    hal5::make_strong_ptr<usb::cdc_serial>(pool,
                                           u"My Serial Device!"sv,
                                           u"ctrl iface"sv,
                                           u"data iface"sv,
                                           serial_data_ep_out,
                                           serial_data_ep_in,
                                           status_ep_out,
                                           status_ep_in,
                                           console);

  hal5::strong_ptr<std::array<usb::configuration, 1>> confs =
    hal::v5::make_strong_ptr<std::array<usb::configuration, 1>>(
      pool,
      std::array<usb::configuration, 1>{
        usb::configuration{ conf_str,
                            usb::configuration::bitmap(false, false),
                            0x32,
                            pool,
                            usb_console_iface } });

  hal::print(*console, "conf array made with interfaces");
  try {
    usb::enumerator<1> en(
      control_endpoint, dev, confs, 0x0409, 1, console, false);
    en.enumerate();
  }  // Catch statements for specific hal exceptions

  catch (hal::no_such_device const& e) {
    hal::print(*console, "no_such_device exception has occurred, terminate\n");
  }

  catch (hal::io_error const& e) {
    hal::print(*console, "io_error exception has occurred, terminate\n");
  }

  catch (hal::resource_unavailable_try_again const& e) {
    hal::print(
      *console,
      "resource_unavailable_try_again exception has occurred, terminate\n");
  }

  catch (hal::device_or_resource_busy const& e) {
    hal::print(*console,
               "device_or_resource_busy exception has occurred, terminate\n");
  }

  catch (hal::timed_out const& e) {
    hal::print(*console, "timed_out exception has occurred, terminate\n");
  }

  catch (hal::operation_not_supported const& e) {
    hal::print(*console,
               "operation_not_supported exception has occurred, terminate\n");
  }

  catch (hal::operation_not_permitted const& e) {
    hal::print(*console,
               "operation_not_permitted exception has occurred, terminate\n");
  }

  catch (hal::argument_out_of_domain const& e) {
    hal::print(*console,
               "argument_out_of_domain exception has occurred, terminate\n");
  }

  catch (hal::message_size const& e) {
    hal::print(*console, "message_size exception has occurred, terminate\n");
  }

  catch (hal::not_connected const& e) {
    hal::print(*console, "not_connected exception has occurred, terminate\n");
  }

  catch (hal::unknown const& e) {
    hal::print(*console, "unknown exception has occurred, terminate\n");
  }

  catch (hal::bad_weak_ptr const& e) {
    hal::print(*console, "bad_weak_ptr exception has occurred, terminate\n");
  }

  catch (hal::out_of_range const& e) {
    hal::print(*console, "out_of_range exception has occurred, terminate\n");
  }

  catch (hal::bad_optional_ptr_access const& e) {
    hal::print(*console,
               "bad_optional_ptr_access exception has occurred, terminate\n");
  }

  // Generic catch for any hal::exception not caught above
  catch (hal::exception const& e) {
    hal::print(*console, "hal::exception has occurred, terminate\n");
  }

  // Generic catch for any std::exception not caught above
  catch (std::exception const& e) {
    hal::print(*console, "std::exception has occurred, terminate\n");
    hal::print(*console, e.what());
    hal::print(*console, "\n");
  }

  catch (...) {
    hal::print(*console, "what?\n");
  }

  hal::print(*console, "Enumerator made\n");

  hal::print(*console, "Enumeration complete\n");
  while (true) {
    continue;
  }
}
