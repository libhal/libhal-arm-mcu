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

struct line_coding_packet
{
  constexpr static hal::usize baud_rate_offset = 0;
  constexpr static hal::usize stop_bits_offset = 4;
  constexpr static hal::usize parity_offset = 5;
  constexpr static hal::usize data_bits_offset = 6;

  enum class stop_bits_e : hal::u8
  {
    one = 0,
    one_and_half = 1,
    two = 2,
  };

  enum class parity_e : hal::u8
  {
    none = 0,
    odd = 1,
    even = 2,
    mark = 3,
    space = 4,
  };

  constexpr line_coding_packet() = default;

  constexpr line_coding_packet(hal::u32 p_baud_rate,
                               stop_bits_e p_stop_bits,
                               parity_e p_parity,
                               hal::u8 p_data_bits)
  {
    baud_rate(p_baud_rate);
    raw_bytes[stop_bits_offset] = static_cast<hal::u8>(p_stop_bits);
    raw_bytes[parity_offset] = static_cast<hal::u8>(p_parity);
    raw_bytes[data_bits_offset] = p_data_bits;
  }

  constexpr line_coding_packet(std::array<hal::u8, 7> const& p_raw)
    : raw_bytes(p_raw)
  {
  }

  [[nodiscard]] constexpr hal::u32 baud_rate() const
  {
    return static_cast<hal::u32>(raw_bytes[0]) |
           static_cast<hal::u32>(raw_bytes[1]) << 8 |
           static_cast<hal::u32>(raw_bytes[2]) << 16 |
           static_cast<hal::u32>(raw_bytes[3]) << 24;
  }

  constexpr void baud_rate(hal::u32 p_baud_rate)
  {
    raw_bytes[0] = static_cast<hal::u8>(p_baud_rate & 0xFF);
    raw_bytes[1] = static_cast<hal::u8>((p_baud_rate >> 8) & 0xFF);
    raw_bytes[2] = static_cast<hal::u8>((p_baud_rate >> 16) & 0xFF);
    raw_bytes[3] = static_cast<hal::u8>((p_baud_rate >> 24) & 0xFF);
  }

  [[nodiscard]] constexpr stop_bits_e stop_bits() const
  {
    return static_cast<enum stop_bits_e>(raw_bytes[stop_bits_offset]);
  }

  constexpr void stop_bits(stop_bits_e p_stop_bits)
  {
    raw_bytes[stop_bits_offset] = static_cast<hal::u8>(p_stop_bits);
  }

  [[nodiscard]] constexpr parity_e parity() const
  {
    return static_cast<parity_e>(raw_bytes[parity_offset]);
  }

  constexpr void parity(parity_e p_parity)
  {
    raw_bytes[parity_offset] = static_cast<hal::u8>(p_parity);
  }

  [[nodiscard]] constexpr hal::u8 data_bits() const
  {
    return raw_bytes[data_bits_offset];
  }

  constexpr void data_bits(hal::u8 p_data_bits)
  {
    raw_bytes[data_bits_offset] = p_data_bits;
  }

  constexpr bool operator==(line_coding_packet const& p_rhs) const = default;

  std::array<hal::u8, 7> raw_bytes{};
};

struct control_line_state
{
  static constexpr auto dtr_mask = hal::bit_mask::from<0>();
  static constexpr auto rts_mask = hal::bit_mask::from<1>();

  constexpr control_line_state() = default;

  explicit constexpr control_line_state(hal::u16 p_raw)
    : raw(p_raw)
  {
  }

  [[nodiscard]] constexpr bool dtr() const
  {
    return hal::bit_extract<dtr_mask>(raw.get());
  }

  [[nodiscard]] constexpr bool rts() const
  {
    return hal::bit_extract<rts_mask>(raw.get());
  }

  constexpr void dtr(bool p_state)
  {
    raw.insert<dtr_mask>(p_state);
  }

  constexpr void rts(bool p_state)
  {
    raw.insert<rts_mask>(p_state);
  }

  constexpr bool operator==(control_line_state const&) const = default;

  hal::bit_value<hal::u16> raw{ 0 };
};

struct port_state
{
  static constexpr auto port_connected_bit = hal::bit_mask::from<0>();
  static constexpr auto data_available_bit = hal::bit_mask::from<1>();

  constexpr port_state() = default;

  explicit constexpr port_state(hal::u8 p_raw)
    : raw(p_raw)
  {
  }

  [[nodiscard]] constexpr bool port_connected() const
  {
    return hal::bit_extract<port_connected_bit>(raw.get());
  }

  constexpr void port_connected(bool p_connected)
  {
    if (p_connected) {
      raw.set<port_connected_bit>();
    } else {
      raw.clear<port_connected_bit>();
    }
  }

  [[nodiscard]] constexpr bool data_available() const
  {
    return hal::bit_extract<data_available_bit>(raw.get());
  }

  constexpr void data_available(bool p_available)
  {
    raw.insert<data_available_bit>(p_available);
  }

  constexpr bool operator==(port_state const& p_other) const
  {
    return p_other.raw.get() == raw.get();
  }

  hal::bit_value<hal::u8> raw{ 0 };
};

class usb_cdc_serial : public hal::usb::interface
{
public:
  usb_cdc_serial(
    hal::strong_ptr<hal::steady_clock> const& p_clock,
    hal::strong_ptr<hal::usb::bulk_out_endpoint> const& p_serial_host_tx,
    hal::strong_ptr<hal::usb::bulk_in_endpoint> const& p_serial_host_rx,
    hal::strong_ptr<hal::usb::interrupt_in_endpoint> const& p_status_in)
    : m_clock(p_clock)
    , m_serial_rx(p_serial_host_tx)
    , m_serial_tx(p_serial_host_rx)
    , m_status_in(p_status_in)
  {
    m_serial_rx->on_receive(
      [this](hal::usb::bulk_out_endpoint::on_receive_tag) {
        m_state.data_available(true);
      });
  }

  void write(hal::scatter_span<hal::byte const> p_data)
  {
    using namespace std::string_view_literals;
    try {
      if (m_state.port_connected()) {
        hal::v5::write_and_flush(*m_serial_tx, p_data);
      }
    } catch (hal::operation_not_permitted const&) {
      m_state.port_connected(false);
    } catch (hal::timed_out const&) {
      m_state.port_connected(false);
    }
  }

  hal::usize read(hal::scatter_span<hal::byte> p_buffer)
  {
    auto length = 0uz;
    if (m_state.data_available() and m_state.port_connected()) {
      length = m_serial_rx->read(p_buffer);
    }
    if (length == 0) {
      m_state.data_available(false);
    }
    return length;
  }

  [[nodiscard]] bool port_connected() const
  {
    return m_state.port_connected();
  }

  [[nodiscard]] control_line_state get_control_line_state() const
  {
    return m_control_line_state;
  }

  [[nodiscard]] line_coding_packet get_line_coding() const
  {
    return m_line_coding;
  }

  constexpr bool data_available()
  {
    return m_state.data_available();
  }

private:
  hal::usb::interface::descriptor_count driver_write_descriptors(
    descriptor_start p_start,
    hal::usb::endpoint_io& p_endpoint) override
  {
    if (p_start.interface.has_value()) {
      m_start.interface = p_start.interface;
    }
    if (p_start.string.has_value()) {
      m_start.string = p_start.string;
    }
    auto const start = m_start.interface.value_or(0);

    auto const idx1 = static_cast<hal::u8>(start + 0);
    auto const idx2 = static_cast<hal::u8>(start + 1);

    auto const interface_association_descriptor = std::to_array<hal::byte>({
      // Interface Association Descriptor
      0x08,  // bLength
      0x0B,  // bDescriptorType (Interface Association)
      idx1,  // bFirstInterface
      0x02,  // bInterfaceCount
      0x02,  // bFunctionClass (CDC)
      0x02,  // bFunctionSubClass (Abstract Control Model)
      0x01,  // bFunctionProtocol
      0x00,  // iFunction (String Index)
    });

    auto const control_interface = hal::v5::usb::generate_interface_descriptor({
      .interface_number = idx1,
      .alternate_setting = 0x00,
      .num_endpoints = 1,
      .interface_class = hal::v5::usb::class_code::cdc_control,
      .interface_subclass = 0x02,  // Abstract Control Model
      .interface_protocol = 0x01,  // AT Commands V.250
      .interface_string_index = 0x00,
    });

    // Make this static-const so the data is put into .rodata (read-only data)
    // section, to save on stack space.
    static auto const cdc_descriptor = std::to_array<hal::byte>({
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
    });

    auto const cdc_final = std::to_array<hal::byte>({
      // CDC Union Functional Descriptor
      0x05,  // bLength
      0x24,  // bDescriptorType (CS_INTERFACE)
      0x06,  // bDescriptorSubtype (Union)
      idx1,  // bControlInterface
      idx2,  // bSubordinateInterface0

      // CDC Call Management Functional Descriptor
      0x05,  // bLength
      0x24,  // bDescriptorType (CS_INTERFACE)
      0x01,  // bDescriptorSubtype (Call Management)
      0x00,  // bmCapabilities
      idx2,  // bDataInterface
    });

    auto const control_in_endpoint =
      hal::v5::usb::generate_endpoint_descriptor(*m_status_in, 0xff);

    auto const data_interface = hal::v5::usb::generate_interface_descriptor({
      .interface_number = idx2,
      .alternate_setting = 0x00,
      .num_endpoints = 2,
      .interface_class = hal::v5::usb::class_code::cdc_data,
      .interface_subclass = 0x00,  // nothing
      .interface_protocol = 0x00,  // nothing
      .interface_string_index = 0x00,
    });

    auto const data_out =
      hal::v5::usb::generate_endpoint_descriptor(*m_serial_rx, 0);
    auto const data_in =
      hal::v5::usb::generate_endpoint_descriptor(*m_serial_tx, 0);

    auto const descriptor =
      hal::make_scatter_array<hal::u8 const>(interface_association_descriptor,
                                             control_interface,
                                             cdc_descriptor,
                                             cdc_final,
                                             control_in_endpoint,
                                             data_interface,
                                             data_out,
                                             data_in);

    p_endpoint.write(descriptor);
    return hal::usb::interface::descriptor_count{ .interface = 2, .string = 0 };
  }

  bool driver_write_string_descriptor(hal::u8, hal::usb::endpoint_io&) override
  {
    // NOTE: This interface has NO string descriptors so this API always returns
    // false.
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
    constexpr hal::u8 clear_feature = 0x01;

    // To get a request from the host means that we can assume we are connected.
    m_state.port_connected(true);

    switch (p_setup.request()) {
      case clear_feature: {
        auto const ep_addr = static_cast<hal::u8>(p_setup.index());
        if (ep_addr == m_serial_rx->info().number) {
          m_serial_rx->reset();
          m_serial_rx->stall(false);
        } else if (ep_addr == m_serial_tx->info().number) {
          m_serial_tx->reset();
          m_serial_rx->stall(false);
        } else if (ep_addr == m_status_in->info().number) {
          m_status_in->reset();
          m_serial_rx->stall(false);
        } else {
          return false;  // Unknown endpoint
        }
        return true;
      }

      case cdc_get_line_coding: {
        p_endpoint.write(
          hal::make_scatter_array<hal::u8 const>(m_line_coding.raw_bytes));
        return true;
      }

      case cdc_set_line_coding: {
        using namespace std::chrono_literals;
        auto const deadline = hal::future_deadline(*m_clock, 1ms);
        auto bytes_read = 0uz;
        while (deadline > m_clock->uptime()) {
          auto const sub_buffer =
            std::span(m_line_coding.raw_bytes).subspan(bytes_read);

          bytes_read +=
            p_endpoint.read(hal::v5::make_writable_scatter_bytes(sub_buffer));

          if (bytes_read >= m_line_coding.raw_bytes.size()) {
            return true;
          }
        }
        return false;
      }

      case cdc_set_control_line_state: {
        m_control_line_state.raw = p_setup.value();
        return true;
      }

      case cdc_send_break: {  // SEND BREAK (do nothing)
        return true;
      }

      default:
        return false;
    }
  }

  void driver_handle_host_event(hal::v5::usb::host_event p_event) override
  {
    switch (p_event) {
      case hal::v5::usb::host_event::reset:
        m_state.port_connected(false);
        m_serial_rx->reset();
        m_serial_tx->reset();
        m_status_in->reset();
        break;
      case hal::v5::usb::host_event::enumerated:
        m_serial_rx->reset();
        m_serial_tx->reset();
        m_status_in->reset();
        break;
      case hal::v5::usb::host_event::suspend_with_wakeup:
        break;
      case hal::v5::usb::host_event::suspend_without_wakeup:
        m_state.port_connected(false);
        break;
      case hal::v5::usb::host_event::resume:
        m_state.port_connected(true);
        break;
      default:  // The rest...
        break;
    }
  }

  hal::strong_ptr<hal::steady_clock> m_clock;
  hal::strong_ptr<hal::usb::bulk_out_endpoint> m_serial_rx;
  hal::strong_ptr<hal::usb::bulk_in_endpoint> m_serial_tx;
  hal::strong_ptr<hal::usb::interrupt_in_endpoint> m_status_in;
  line_coding_packet m_line_coding{};
  descriptor_start m_start{};
  control_line_state m_control_line_state{};
  port_state m_state{};
};

std::string_view to_string_view(std::errc p_errc);

void application()
{
  using namespace hal::literals;
  using namespace std::literals;
  using namespace std::chrono_literals;

  auto clock = resources::clock();
  auto console = resources::console();

  hal::print(*console,
             R"(
Starting USB CDC virtual serial port application...

This demo does the following:

- Sends 'Hello, World' every 4 seconds
- Echoes received data
- Logs if DTR or RTS have changed state
- Prints '-' on USB RESET
- Prints '+' if enumeration was completed

)");

  auto allocator = resources::driver_allocator();
  auto control_endpoint = resources::usb_control_endpoint();
  auto serial_host_ep_out = resources::usb_bulk_out_endpoint1();
  auto serial_host_ep_in = resources::usb_bulk_in_endpoint1();
  auto status_ep_in = resources::usb_interrupt_in_endpoint1();

  hal::print<512 + 256>(*console,
                        "+--------------------+--------+------+\n"
                        "| Endpoint Name      | Number | Size |\n"
                        "+--------------------+--------+------+\n"
                        "| %-18s |  0x%02X  | %4zu |\n"
                        "| %-18s |  0x%02X  | %4zu |\n"
                        "| %-18s |  0x%02X  | %4zu |\n"
                        "+--------------------+--------+------+\n\n",
                        "status_ep_in",
                        status_ep_in->info().number,
                        status_ep_in->info().size,
                        "serial_host_ep_out",
                        serial_host_ep_out->info().number,
                        serial_host_ep_out->info().size,
                        "serial_host_ep_in",
                        serial_host_ep_in->info().number,
                        serial_host_ep_in->info().size);

  auto virtual_usb_serial = hal::make_strong_ptr<usb_cdc_serial>(
    allocator, clock, serial_host_ep_out, serial_host_ep_in, status_ep_in);

  hal::usb::inplace_enumerator usb_enumerator(
    control_endpoint,
    {
      .manufacturer = u"libhal",
      .product = u"libhal virtual serial",
      .serial_number = u"0001",
      .vendor_id = 0xDEAD,
      .product_id = 0xBEEF,
      // Takes default for everything else
    },
    virtual_usb_serial);

  auto const send_period = 4s;
  auto send_deadline = hal::future_deadline(*clock, send_period);

  auto const activity_period = 250ms;
  auto dot_deadline = hal::future_deadline(*clock, activity_period);

  auto last_control_line_state = virtual_usb_serial->get_control_line_state();
  auto last_line_coding = virtual_usb_serial->get_line_coding();
  bool previously_enumerated = false;

  while (true) {
    try {
      usb_enumerator.process_ctrl_transfer();

      // =======================================================================
      // Log data received from the HOST
      // =======================================================================

      if (virtual_usb_serial->data_available()) {
        std::array<char, 16> buffer{};
        auto length = virtual_usb_serial->read(
          hal::make_scatter_array<hal::u8>(hal::as_writable_bytes(buffer)));

        if (length != 0) {
          hal::print<128>(
            *console, "\n[📨 Received]:(%.*s)\n", length, buffer.data());
        }
      }

      // =======================================================================
      // Check the DTR & RTS states
      // =======================================================================

      if (auto const current_control_line_state =
            virtual_usb_serial->get_control_line_state();
          current_control_line_state != last_control_line_state) {
        if (current_control_line_state.dtr() != last_control_line_state.dtr()) {
          hal::print<32>(
            *console, "\n[📡 DTR]:(%d)\n", current_control_line_state.dtr());
        }
        if (current_control_line_state.rts() != last_control_line_state.rts()) {
          hal::print<32>(
            *console, "\n[📡 RTS]:(%d)\n", current_control_line_state.rts());
        }
        last_control_line_state = current_control_line_state;
      }

      // =======================================================================
      // Check the line coding (baud rate, stop bits, parity, data bits)
      // =======================================================================

      if (auto const current_line_coding =
            virtual_usb_serial->get_line_coding();
          current_line_coding != last_line_coding) {
        hal::print<128>(*console,
                        "\n[⚙️ LINE CODING]:(baud=%lu, stop=%d, parity=%d, "
                        "data_bits=%d)\n",
                        current_line_coding.baud_rate(),
                        static_cast<int>(current_line_coding.stop_bits()),
                        static_cast<int>(current_line_coding.parity()),
                        static_cast<int>(current_line_coding.data_bits()));
        last_line_coding = current_line_coding;
      }

      // =======================================================================
      // Send "Hello, World" to HOST
      // =======================================================================

      if (usb_enumerator.is_enumerated()) {
        if (not previously_enumerated) {
          hal::print(*console, "\n[✅ ENUMERATED]\n");
          previously_enumerated = true;
        }
        if (clock->uptime() >= dot_deadline) {
          hal::print(*console, "+");
          dot_deadline = hal::future_deadline(*clock, activity_period);
        }
        if (clock->uptime() >= send_deadline) {
          virtual_usb_serial->write(hal::make_scatter_array<hal::byte const>(
            hal::as_bytes("Hello, World\n"sv)));
          send_deadline = hal::future_deadline(*clock, send_period);
          hal::print(*console, "\n[📬 SENT]:(Hello, World\\n)\n");
        }
      } else {
        if (previously_enumerated) {
          hal::print(*console, "\n[🚫 ENUMERATION LOST]\n");
          previously_enumerated = true;
        }
        if (clock->uptime() >= dot_deadline) {
          hal::print(*console, "-");
          dot_deadline = hal::future_deadline(*clock, activity_period);
        }
      }

    } catch (hal::exception const& p_error) {
      hal::print<64>(
        *console, "\nException Error Code '%d' caught\n", p_error.error_code());
    }
  }
}
