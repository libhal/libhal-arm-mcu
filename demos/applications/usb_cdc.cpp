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
#include <utility>

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

  constexpr bool operator==(control_line_state const& p_other) const
  {
    return raw.get() == p_other.raw.get();
  }

  hal::bit_value<hal::u16> raw{ 0 };
};

hal::optional_ptr<hal::serial> g_console;

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
        // If we received data, then we know the port is connected
        m_port_connected = true;
      });
  }

  void write(hal::scatter_span<hal::byte const> p_data)
  {
    using namespace std::string_view_literals;
    try {
      if (m_port_connected) {
        hal::v5::write_and_flush(*m_serial_tx, p_data);
      }
    } catch (hal::operation_not_permitted const&) {
      // NOTE: Why are we catching this exception and why does it happen?
      //
      // This exception will be thrown from an IN endpoint if USB peripheral is
      // in the "suspend_without_wakeup" state OR in "reset". In the case of
      // suspend without wakeup, the IN endpoint is not allowed to wake the HOST
      // and thus cannot proceed with the write operation and must throw to exit
      // the code. In the case of "reset", the enumeration status is gone and
      // there is no assumption that a HOST will respond in time, and thus an
      // exception is thrown to exit the impossible to fulfil code (without
      // potentially waiting forever)
      //
      // This is the purpose  of the `port_connected()` check in the if
      // statement above.
      m_port_connected = false;
    } catch (hal::timed_out const&) {
      // NOTE: Why are we catching this exception and why does it happen?
      //
      // For libhal v4, which doesn't utilize coroutine and doesn't mandate a
      // scheduler, there needs to be some way for the serial TX to communicate
      // when its waited for too long to get data sent out. This has to do with
      // the asynchronous and non-deterministic nature in how IN packets work
      // for USB.
      //
      // Because USB (prior to 3.0) is HOST driven and polled, IN endpoints rely
      // on the HOST to send out "IN Token" packets which signal to the device,
      // "If you have data, give it to me now." For bulk endpoints, there is no
      // polling rate, thus no guarantee when an IN Token will show up. This
      // should be rare so long as the USB data lines haven't been disconnected
      // OR the HOST hasn't gone to sleep.
      m_port_connected = false;
    }
  }

  hal::usize read(hal::scatter_span<hal::byte> p_buffer)
  {
    return m_serial_rx->read(p_buffer);
  }

  [[nodiscard]] constexpr bool port_connected() const
  {
    return m_port_connected;
  }

  [[nodiscard]] constexpr control_line_state get_control_line_state() const
  {
    return m_control_line_state;
  }

  [[nodiscard]] constexpr line_coding_packet get_line_coding() const
  {
    return m_line_coding;
  }

private:
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
    if (p_start.string.has_value()) {
      m_start.string = p_start.string;
    }

    // In the event that the interface value is not set, it is always valid to
    // use the 0 index for the interface. Prefer to use 0 since thats simple to
    // work with.
    auto const start = m_start.interface.value_or(0);

    // Rather than putting a static cast and addition everywhere, make a const
    // value with the index 1 and index 2 values. Then use those in the arrays
    // below.
    auto const idx1 = static_cast<hal::u8>(start + 0);
    auto const idx2 = static_cast<hal::u8>(start + 1);
    auto const cdc = std::to_underlying(hal::v5::usb::class_code::cdc_control);

    auto const interface_association_descriptor = std::to_array<hal::byte>({
      // Interface Association Descriptor
      0x08,  // bLength (length of this descriptor)
      0x0B,  // bDescriptorType (Interface Association: allows multi-interfaces)
      idx1,  // bFirstInterface (The index of the first interface in this set)
      0x02,  // bInterfaceCount (Count of interfaces)
      cdc,   // bFunctionClass (Communications Device Class)
      0x02,  // bFunctionSubClass (Abstract Control Model)
      0x01,  // bFunctionProtocol (Protocol: AT Commands (V.250))
      0x00,  // iFunction (String Index, set to 0 meaning no strings)
    });

    auto const control_interface = hal::v5::usb::generate_interface_descriptor({
      .interface_number = idx1,
      .alternate_setting = 0x00,
      .num_endpoints = 1,  // endpoint count
      .interface_class = hal::v5::usb::class_code::cdc_control,
      .interface_subclass = 0x02,      // Abstract Control Model
      .interface_protocol = 0x01,      // AT Commands V.250
      .interface_string_index = 0x00,  // No string associated with this
    });

    // Make this static-const so the data is put into .rodata (read-only data)
    // section, to save on stack space.
    static auto const cdc_header = std::to_array<hal::byte>({
      // CDC Header Functional Descriptor
      0x05,  // bLength
      0x24,  // bDescriptorType (CS_INTERFACE)
      0x00,  // bDescriptorSubtype (Header)
      0x10,  // bcdCDC (1.10) part 1
      0x01,  // bcdCDC (1.10) part 2

      // CDC ACM Functional Descriptor
      0x04,  // bLength
      0x24,  // bDescriptorType (CS_INTERFACE)
      0x02,  // bDescriptorSubtype (Abstract Control Management)
      0x02,  // bmCapabilities
    });

    auto const cdc_footer = std::to_array<hal::byte>({
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

    auto const status_ep_descriptor =
      hal::v5::usb::generate_endpoint_descriptor(*m_status_in, 0xff);

    auto const data_interface = hal::v5::usb::generate_interface_descriptor({
      .interface_number = idx2,
      .alternate_setting = 0x00,
      .num_endpoints = 2,
      .interface_class = hal::v5::usb::class_code::cdc_data,
      .interface_subclass = 0x00,      // nothing
      .interface_protocol = 0x00,      // nothing
      .interface_string_index = 0x00,  // no strings
    });

    auto const data_rx_descriptor =
      hal::v5::usb::generate_endpoint_descriptor(*m_serial_rx, 0);
    auto const data_tx_descriptor =
      hal::v5::usb::generate_endpoint_descriptor(*m_serial_tx, 0);

    // Combine all of the endpoints together in one big scatter array
    auto const descriptor =
      hal::make_scatter_array<hal::u8 const>(interface_association_descriptor,
                                             control_interface,
                                             cdc_header,
                                             cdc_footer,
                                             status_ep_descriptor,
                                             data_interface,
                                             data_rx_descriptor,
                                             data_tx_descriptor);

    p_endpoint.write(descriptor);
    // Report back to the enumerator that we hae 2 interfaces available
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
    m_port_connected = true;

    switch (p_setup.request()) {
      case clear_feature: {
        auto const ep_addr = static_cast<hal::u8>(p_setup.index());
        if (ep_addr == m_serial_rx->info().number) {
          m_serial_rx->reset();
        } else if (ep_addr == m_serial_tx->info().number) {
          m_serial_tx->reset();
        } else if (ep_addr == m_status_in->info().number) {
          m_status_in->reset();
        } else {
          return false;  // Unknown endpoint
        }
        return true;
      }

      case cdc_get_line_coding: {
        // Read contents into the m_line_coding array
        p_endpoint.write(
          hal::make_scatter_array<hal::u8 const>(m_line_coding.raw_bytes));
        return true;
      }

      case cdc_set_line_coding: {
        // It takes time for the HOST to send additional data packets after the
        // setup, so we use a deadline of 1ms of time to retrieve the 7-bytes we
        // need for the line coding.
        using namespace std::chrono_literals;

        auto const deadline = hal::future_deadline(*m_clock, 1ms);
        auto bytes_read = 0uz;
        while (deadline > m_clock->uptime()) {
          // NOTE: Technically, the reading from the control endpoint, when
          // there is any data, should result in the entire payload being
          // delivered for this class specific host command. The line coding
          // payload is 7 bytes and the control endpoint capacity must be at
          // least 8 bytes. Meaning, when the endpoint contains the data after
          // the setup command, it should have all of the bytes and the
          // following read will acquire all bytes.
          //
          // The choice below to subspan the already read bytes is to play it
          // safe and not assume that the read API always drains all of the data
          // from the endpoint.
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

      case cdc_send_break: {
        // SEND BREAK (do nothing currently)
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
        m_port_connected = false;
        m_serial_rx->reset();
        m_serial_tx->reset();
        m_status_in->reset();
        break;
      case hal::v5::usb::host_event::enumerated:
        m_port_connected = true;
        m_serial_rx->reset();
        m_serial_tx->reset();
        m_status_in->reset();
        break;
      case hal::v5::usb::host_event::suspend_with_wakeup:
        break;
      case hal::v5::usb::host_event::suspend_without_wakeup:
        m_port_connected = false;
        break;
      case hal::v5::usb::host_event::resume:
        m_port_connected = true;
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
  bool m_port_connected{};
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

  auto virtual_usb_serial = hal::make_strong_ptr<usb_cdc_serial>(
    allocator, clock, serial_host_ep_out, serial_host_ep_in, status_ep_in);
  auto virtual_usb_serial2 = hal::make_strong_ptr<usb_cdc_serial>(
    allocator, clock, serial_host_ep_out2, serial_host_ep_in2, status_ep_in2);

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
    virtual_usb_serial,
    virtual_usb_serial2);

  auto const send_period = 4s;
  auto send_deadline = hal::future_deadline(*clock, send_period);

  auto const activity_period = 250ms;
  auto dot_deadline = hal::future_deadline(*clock, activity_period);

  auto last_control_line_state = virtual_usb_serial->get_control_line_state();
  auto last_line_coding = virtual_usb_serial->get_line_coding();
  auto last_control_line_state2 = virtual_usb_serial2->get_control_line_state();
  auto last_line_coding2 = virtual_usb_serial2->get_line_coding();
  bool previously_enumerated = false;

  while (true) {
    try {
      usb_enumerator.process_ctrl_transfer();

      // =======================================================================
      // Log data received from the HOST
      // =======================================================================

      {
        std::array<char, 16> buffer{};
        auto length = 0uz;

        // Check serial port 1
        length = virtual_usb_serial->read(
          hal::make_scatter_array<hal::u8>(hal::as_writable_bytes(buffer)));

        if (length != 0) {
          hal::print<128>(
            *console, "\n[📨 Received 1]:(%.*s)\n", length, buffer.data());
        }

        // Check serial port 2
        length = virtual_usb_serial2->read(
          hal::make_scatter_array<hal::u8>(hal::as_writable_bytes(buffer)));

        if (length != 0) {
          hal::print<128>(
            *console, "\n[📨 Received 2]:(%.*s)\n", length, buffer.data());
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
            *console, "\n[📡 DTR 1]:(%d)\n", current_control_line_state.dtr());
        }
        if (current_control_line_state.rts() != last_control_line_state.rts()) {
          hal::print<32>(
            *console, "\n[📡 RTS 1]:(%d)\n", current_control_line_state.rts());
        }
        last_control_line_state = current_control_line_state;
      }

      if (auto const current_control_line_state2 =
            virtual_usb_serial2->get_control_line_state();
          current_control_line_state2 != last_control_line_state2) {
        if (current_control_line_state2.dtr() !=
            last_control_line_state2.dtr()) {
          hal::print<32>(
            *console, "\n[📡 DTR 2]:(%d)\n", current_control_line_state2.dtr());
        }
        if (current_control_line_state2.rts() !=
            last_control_line_state2.rts()) {
          hal::print<32>(
            *console, "\n[📡 RTS 2]:(%d)\n", current_control_line_state2.rts());
        }
        last_control_line_state2 = current_control_line_state2;
      }

      // =======================================================================
      // Check the line coding (baud rate, stop bits, parity, data bits)
      // =======================================================================

      if (auto const current_line_coding =
            virtual_usb_serial->get_line_coding();
          current_line_coding != last_line_coding) {
        hal::print<128>(*console,
                        "\n[⚙️ LINE CODING 1]:(baud=%lu, stop=%d, parity=%d, "
                        "data_bits=%d)\n",
                        current_line_coding.baud_rate(),
                        static_cast<int>(current_line_coding.stop_bits()),
                        static_cast<int>(current_line_coding.parity()),
                        static_cast<int>(current_line_coding.data_bits()));
        last_line_coding = current_line_coding;
      }

      if (auto const current_line_coding2 =
            virtual_usb_serial2->get_line_coding();
          current_line_coding2 != last_line_coding2) {
        hal::print<128>(*console,
                        "\n[⚙️ LINE CODING 2]:(baud=%lu, stop=%d, parity=%d, "
                        "data_bits=%d)\n",
                        current_line_coding2.baud_rate(),
                        static_cast<int>(current_line_coding2.stop_bits()),
                        static_cast<int>(current_line_coding2.parity()),
                        static_cast<int>(current_line_coding2.data_bits()));
        last_line_coding2 = current_line_coding2;
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
          hal::print(*console, "\n[📬 SENT 1]:(Hello, World\\n)\n");

          virtual_usb_serial2->write(hal::make_scatter_array<hal::byte const>(
            hal::as_bytes("Goodbye, World\n"sv)));
          hal::print(*console, "\n[📬 SENT 2]:(Goodbye, World\\n)\n");
          send_deadline = hal::future_deadline(*clock, send_period);
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
