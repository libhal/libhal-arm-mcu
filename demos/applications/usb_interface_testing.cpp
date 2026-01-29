#include <array>
#include <string_view>
#include <tuple>
#include <utility>

#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal-util/usb.hpp>
#include <libhal/pointers.hpp>
#include <libhal/scatter_span.hpp>
#include <libhal/units.hpp>
#include <libhal/usb.hpp>

#include "resource_list.hpp"

namespace hal::usb {
class dummy_interface : public interface
{
public:
  constexpr dummy_interface(std::u16string_view p_name)
    : m_name(p_name) {};

private:
  descriptor_count driver_write_descriptors(
    descriptor_start p_start,
    endpoint_writer const& p_callback) override
  {

    if (p_start.interface.has_value()) {
      m_packed_array[2] = p_start.interface.value();
    }

    if (p_start.string.has_value()) {
      m_packed_array[8] = p_start.string.value();
    }

    auto scatter_arr =
      make_scatter_bytes(std::span<byte const>(m_packed_array));

    p_callback(scatter_arr);
    return { .interface = 1, .string = 1 };
  }

  bool driver_write_string_descriptor(
    u8 p_index,
    endpoint_writer const& p_callback) override
  {
    if (p_index != m_packed_array[8]) {
      return false;
    }
    std::array<byte, 2> hdr_array(
      { static_cast<byte>(m_name.length()),
        static_cast<byte>(descriptor_type::string) });
    auto scatter_payload = make_scatter_bytes(
      hdr_array,
      std::span(reinterpret_cast<byte const*>(m_name.data()), m_name.length()));

    p_callback(scatter_payload);
    return true;
  }

  bool driver_handle_request(setup_packet const& p_setup,
                             endpoint_writer const& p_callback) override
  {
    std::ignore = p_setup;
    std::ignore = p_callback;
    return true;
  }

public:
  std::array<u8, 9> m_packed_array = {
    constants::inferface_desc_size,                 // size
    static_cast<byte>(descriptor_type::interface),  // desc type
    0,                                              // interface_number
    0,                                              // alternate_setting
    0,                                              //  num_endpoints
    0,                                              //  interface_class
    0,                                              // interface_sub_class
    0,                                              // interface_protocol
    0                                               // interface name index
  };

  std::u16string_view m_name;
};

}  // namespace hal::usb

namespace hal5 = hal::v5;

void application()
{
  namespace usb = hal::usb;
  namespace pmr = std::pmr;
  using namespace hal::literals;
  using namespace std::chrono_literals;
  using namespace std::string_view_literals;

  auto const console = resources::console();
  auto const clk = resources::clock();
  auto const pool = resources::driver_allocator();

  auto manu_str = u"libhal"sv;
  auto prod_str = u"my epic device!!"sv;
  auto sn_str = u"2468"sv;

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
  auto iface_str = u"dummy";
  auto conf_str = u"test config";
  auto iface_ptr = hal5::make_strong_ptr<usb::dummy_interface>(
    pool, usb::dummy_interface{ iface_str });

  hal::print(*console, "conf made\n");

  auto confs = hal::v5::make_strong_ptr<std::array<usb::configuration, 1>>(
    pool,
    std::array<usb::configuration, 1>{
      usb::configuration{ conf_str,  // yeet
                          usb::configuration::bitmap(false, false),
                          1,
                          pool,
                          iface_ptr } });

  hal::print(*console, "conf array made\n");

  auto ctrl_ep = resources::usb_control_endpoint();
  hal::u16 const en_lang_str = 0x0409;

  hal::print(*console, "ctrl ep called\n");

  try {
    usb::enumerator<1> en(ctrl_ep,
                          dev,
                          confs,
                          en_lang_str,  // f
                          1,
                          false);

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
  // return;
}
