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

#include <array>
#include <chrono>
#include <coroutine>
#include <span>
#include <string_view>
#include <utility>

module arm_mcu_demos;

import hal;
import hal.util;
import hal.usb;

namespace {
struct line_coding_packet
{
  [[maybe_unused]] constexpr static hal::usize baud_rate_offset = 0;
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

  [[nodiscard]] constexpr hal::u32 baud_rate() const
  {
    return static_cast<hal::u32>(raw_bytes[0]) |
           static_cast<hal::u32>(raw_bytes[1]) << 8 |
           static_cast<hal::u32>(raw_bytes[2]) << 16 |
           static_cast<hal::u32>(raw_bytes[3]) << 24;
  }

  [[nodiscard]] constexpr stop_bits_e stop_bits() const
  {
    return static_cast<enum stop_bits_e>(raw_bytes[stop_bits_offset]);
  }

  [[nodiscard]] constexpr parity_e parity() const
  {
    return static_cast<parity_e>(raw_bytes[parity_offset]);
  }

  [[nodiscard]] constexpr hal::u8 data_bits() const
  {
    return raw_bytes[data_bits_offset];
  }

  constexpr bool operator==(line_coding_packet const& p_rhs) const = default;

  std::array<hal::u8, 7> raw_bytes{};
};

struct control_line_state
{
  static constexpr auto dtr_mask = hal::bit_mask::from<0>();
  static constexpr auto rts_mask = hal::bit_mask::from<1>();

  constexpr control_line_state() = default;

  [[nodiscard]] constexpr bool dtr() const
  {
    return hal::bit_extract<dtr_mask>(raw);
  }

  [[nodiscard]] constexpr bool rts() const
  {
    return hal::bit_extract<rts_mask>(raw);
  }

  constexpr bool operator==(control_line_state const&) const = default;

  hal::u16 raw = 0;
};

/**
 * @brief USB CDC-ACM virtual serial port interface.
 *
 * Ported from v1's usb_cdc.cpp `usb_cdc_serial`. The main structural
 * difference from v1 is that there is no `on_receive(callback)` registration
 * API on `out_endpoint` anymore (v5 core replaced callback registration with
 * single-waiter awaitables), so "the host has sent us data, therefore the
 * port must be connected" is now detected directly inside `read()` instead of
 * a callback set up in the constructor.
 *
 * `is_enumerated()` similarly replaces v1's `usb_enumerator.is_enumerated()`
 * check: the enumerator itself lives inside `application()`'s coroutine frame
 * and isn't reachable from `task1()`, but the enumerator already reports
 * `host_event::enumerated`/`host_event::reset` to every interface via
 * `handle_host_event()`, so this interface tracks that state itself.
 */
class usb_cdc_serial : public hal::usb::interface
{
public:
  usb_cdc_serial(hal::ptr<hal::steady_clock> p_clock,
                 hal::ptr<hal::usb::bulk_out_endpoint> p_serial_rx,
                 hal::ptr<hal::usb::bulk_in_endpoint> p_serial_tx,
                 hal::ptr<hal::usb::interrupt_in_endpoint> p_status_in)
    : m_clock(p_clock)
    , m_serial_rx(p_serial_rx)
    , m_serial_tx(p_serial_tx)
    , m_status_in(p_status_in)
  {
  }

  async::future<void> write(async::context& p_context,
                            mem::scatter_span<hal::byte const> p_data)
  {
    if (not m_port_connected) {
      co_return;
    }
    try {
      co_await hal::usb::write_and_flush(p_context, *m_serial_tx, p_data);
    } catch (hal::operation_not_permitted const&) {
      // Thrown by an IN endpoint when the bus is suspended without wakeup
      // permission: the endpoint cannot wake the host and cannot fulfil the
      // write, so treat the port as disconnected until the next successful
      // host interaction.
      m_port_connected = false;
    }
  }

  async::future<hal::usize> read(async::context& p_context,
                                 mem::scatter_span<hal::byte> p_buffer)
  {
    auto const length = co_await m_serial_rx->read(p_context, p_buffer);
    if (length != 0) {
      m_port_connected = true;
    }
    co_return length;
  }

  [[nodiscard]] bool port_connected() const
  {
    return m_port_connected;
  }

  [[nodiscard]] bool is_enumerated() const
  {
    return m_enumerated;
  }

  [[nodiscard]] control_line_state get_control_line_state() const
  {
    return m_control_line_state;
  }

  [[nodiscard]] line_coding_packet get_line_coding() const
  {
    return m_line_coding;
  }

private:
  async::future<hal::usb::interface::descriptor_count> driver_write_descriptors(
    async::context& p_context,
    hal::usb::interface::descriptor_start p_start,
    hal::usb::endpoint_io& p_endpoint) override
  {
    // As required by the rules of "descriptor_start", the interface and
    // string numbers should be checked and if they are set, then those
    // values must be stored so they are used later by the enumerator.
    if (p_start.interface.has_value()) {
      m_start.interface = p_start.interface;
    }
    if (p_start.string.has_value()) {
      m_start.string = p_start.string;
    }

    auto const start = m_start.interface.value_or(0);
    auto const idx1 = static_cast<hal::u8>(start + 0);
    auto const idx2 = static_cast<hal::u8>(start + 1);
    auto const cdc = std::to_underlying(hal::usb::class_code::cdc_control);

    auto const interface_association_descriptor = std::to_array<hal::byte>({
      // Interface Association Descriptor
      0x08,  // bLength
      0x0B,  // bDescriptorType (Interface Association)
      idx1,  // bFirstInterface
      0x02,  // bInterfaceCount
      cdc,   // bFunctionClass (Communications Device Class)
      0x02,  // bFunctionSubClass (Abstract Control Model)
      0x01,  // bFunctionProtocol (AT Commands V.250)
      0x00,  // iFunction
    });

    auto const control_interface = hal::usb::generate_interface_descriptor({
      .interface_number = idx1,
      .alternate_setting = 0x00,
      .num_endpoints = 1,
      .interface_class = hal::usb::class_code::cdc_control,
      .interface_subclass = 0x02,      // Abstract Control Model
      .interface_protocol = 0x01,      // AT Commands V.250
      .interface_string_index = 0x00,  // No string associated with this
    });

    auto const cdc_header = std::to_array<hal::byte>({
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
      hal::usb::generate_endpoint_descriptor(*m_status_in, 0xff);

    auto const data_interface = hal::usb::generate_interface_descriptor({
      .interface_number = idx2,
      .alternate_setting = 0x00,
      .num_endpoints = 2,
      .interface_class = hal::usb::class_code::cdc_data,
      .interface_subclass = 0x00,      // nothing
      .interface_protocol = 0x00,      // nothing
      .interface_string_index = 0x00,  // no strings
    });

    auto const data_rx_descriptor =
      hal::usb::generate_endpoint_descriptor(*m_serial_rx, 0);
    auto const data_tx_descriptor =
      hal::usb::generate_endpoint_descriptor(*m_serial_tx, 0);

    mem::scatter_array<hal::byte const, 8> descriptor(
      interface_association_descriptor,
      control_interface,
      cdc_header,
      cdc_footer,
      status_ep_descriptor,
      data_interface,
      data_rx_descriptor,
      data_tx_descriptor);

    std::ignore = co_await p_endpoint.write(p_context, descriptor);

    // Report back to the enumerator that we have 2 interfaces available.
    co_return hal::usb::interface::descriptor_count{ .interface = 2,
                                                     .string = 0 };
  }

  async::future<bool> driver_write_string_descriptor(
    async::context&,
    hal::u8,
    hal::usb::endpoint_io&) override
  {
    // This interface has NO string descriptors so this API always returns
    // false.
    co_return false;
  }

  async::future<bool> driver_handle_request(
    async::context& p_context,
    hal::usb::setup_packet const& p_setup,
    hal::usb::endpoint_io& p_endpoint) override
  {
    // CDC Class-Specific Request Codes
    constexpr hal::u8 cdc_set_line_coding = 0x20;
    constexpr hal::u8 cdc_get_line_coding = 0x21;
    constexpr hal::u8 cdc_set_control_line_state = 0x22;
    constexpr hal::u8 cdc_send_break = 0x23;
    constexpr hal::u8 clear_feature = 0x01;

    // To get a request from the host means that we can assume we are
    // connected.
    m_port_connected = true;

    switch (p_setup.request()) {
      case clear_feature: {
        auto const ep_addr = static_cast<hal::u8>(p_setup.index());
        if (ep_addr == m_serial_rx->info().number) {
          co_await m_serial_rx->reset(p_context);
        } else if (ep_addr == m_serial_tx->info().number) {
          co_await m_serial_tx->reset(p_context);
        } else if (ep_addr == m_status_in->info().number) {
          co_await m_status_in->reset(p_context);
        } else {
          co_return false;  // Unknown endpoint
        }
        co_return true;
      }

      case cdc_get_line_coding: {
        std::ignore =
          co_await p_endpoint.write(p_context,
                                    mem::scatter_span<hal::byte const>{
                                      std::span(m_line_coding.raw_bytes) });
        co_return true;
      }

      case cdc_set_line_coding: {
        // It takes time for the HOST to send additional data packets after
        // the setup, so use a deadline of 1ms of time to retrieve the 7
        // bytes needed for the line coding.
        using namespace std::chrono_literals;

        auto const deadline =
          co_await hal::future_deadline(p_context, *m_clock, 1ms);
        auto bytes_read = 0uz;
        while (deadline > co_await m_clock->uptime(p_context)) {
          auto const sub_buffer =
            std::span(m_line_coding.raw_bytes).subspan(bytes_read);

          bytes_read += co_await p_endpoint.read(
            p_context, mem::scatter_span<hal::byte>{ sub_buffer });

          if (bytes_read >= m_line_coding.raw_bytes.size()) {
            co_return true;
          }
        }
        co_return false;
      }

      case cdc_set_control_line_state: {
        m_control_line_state.raw = p_setup.value();
        co_return true;
      }

      case cdc_send_break: {
        // SEND BREAK (do nothing currently)
        co_return true;
      }

      default:
        co_return false;
    }
  }

  async::future<void> driver_handle_host_event(
    async::context& p_context,
    hal::usb::host_event p_event) override
  {
    switch (p_event) {
      case hal::usb::host_event::reset:
        m_port_connected = false;
        m_enumerated = false;
        co_await m_serial_rx->reset(p_context);
        co_await m_serial_tx->reset(p_context);
        co_await m_status_in->reset(p_context);
        break;
      case hal::usb::host_event::enumerated:
        m_port_connected = true;
        m_enumerated = true;
        co_await m_serial_rx->reset(p_context);
        co_await m_serial_tx->reset(p_context);
        co_await m_status_in->reset(p_context);
        break;
      case hal::usb::host_event::suspend_with_wakeup:
        break;
      case hal::usb::host_event::suspend_without_wakeup:
        m_port_connected = false;
        break;
      case hal::usb::host_event::resume:
        m_port_connected = true;
        break;
      default:  // The rest...
        break;
    }
  }

  hal::ptr<hal::steady_clock> m_clock;
  hal::ptr<hal::usb::bulk_out_endpoint> m_serial_rx;
  hal::ptr<hal::usb::bulk_in_endpoint> m_serial_tx;
  hal::ptr<hal::usb::interrupt_in_endpoint> m_status_in;
  line_coding_packet m_line_coding{};
  hal::usb::interface::descriptor_start m_start{};
  control_line_state m_control_line_state{};
  bool m_port_connected = false;
  bool m_enumerated = false;
};

// Published by application() once the interface has been constructed;
// task1() polls this to know when it is safe to start using it. See
// resource_list.cppm / main.cpp for why application() and task1() cannot
// share this state through a function parameter: they are two independent
// coroutines driven concurrently off of separate contexts by
// async::run_until_done(), which is required here because a single
// coroutine cannot simultaneously await both the control endpoint's bus
// events (what the enumerator needs) and the bulk endpoint's data (what the
// echo logic needs).
hal::opt_ptr<usb_cdc_serial> g_serial_interface;
}  // namespace

hal::task application(async::context& p_context)
{
  auto allocator = resources::driver_allocator();
  auto clock = resources::clock();

  auto control_endpoint = co_await resources::usb_control_endpoint(p_context);
  auto serial_rx = co_await resources::usb_bulk_out_endpoint1(p_context);
  auto serial_tx = co_await resources::usb_bulk_in_endpoint1(p_context);
  auto status_in = co_await resources::usb_interrupt_in_endpoint1(p_context);

  auto serial_interface = hal::allocate<usb_cdc_serial>(
    allocator, clock, serial_rx, serial_tx, status_in);

  g_serial_interface = serial_interface;

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
    serial_interface);

  co_await usb_enumerator.run(p_context);
}

extern "C++"
{
  hal::task task1(async::context& p_context)
  {
    using namespace std::literals;
    using namespace std::chrono_literals;

    // Wait for application() to finish constructing the shared USB
    // interface. See the comment on g_serial_interface above.
    while (not g_serial_interface) {
      co_await 1ms;
    }

    auto serial_interface = g_serial_interface;
    auto clock = resources::clock();
    auto console = resources::console();

    co_await hal::write(p_context, *console, R"(
Starting USB CDC virtual serial port application...

This demo does the following:

- Sends 'Hello, World' every 4 seconds
- Echoes received data
- Logs if DTR or RTS have changed state
- Prints '-' on USB RESET
- Prints '+' if enumeration was completed

)");

    auto const send_period = 4s;
    auto send_deadline =
      co_await hal::future_deadline(p_context, *clock, send_period);

    auto const activity_period = 250ms;
    auto dot_deadline =
      co_await hal::future_deadline(p_context, *clock, activity_period);

    auto last_control_line_state = serial_interface->get_control_line_state();
    auto last_line_coding = serial_interface->get_line_coding();
    bool previously_enumerated = false;

    while (true) {
      bool exception_caught = false;
      int error_code_to_print = 0;
      try {
        std::array<char, 16> buffer{};
        auto const length = co_await serial_interface->read(
          p_context,
          mem::scatter_span<hal::byte>{ hal::as_writable_bytes(buffer) });

        if (length != 0) {
          co_await hal::print<128>(p_context,
                                   *console,
                                   "\n[Received]:(%.*s)\n",
                                   length,
                                   buffer.data());
        }

        // Control-line state and line coding are reported through one
        // shared print<> instantiation instead of three separate ones (was:
        // print<32, bool> for DTR, print<32, bool> for RTS, print<128, u32,
        // int, int, int> for line coding) -- each distinct print<N, ...>
        // signature compiles to its own coroutine, so collapsing them onto
        // one signature removes duplicate generated code. A full fix (an
        // inlined-buffer print that takes a span instead of a template
        // buffer_size) is future work.
        bool line_state_changed = false;

        if (auto const current_control_line_state =
              serial_interface->get_control_line_state();
            current_control_line_state != last_control_line_state) {
          last_control_line_state = current_control_line_state;
          line_state_changed = true;
        }

        if (auto const current_line_coding =
              serial_interface->get_line_coding();
            current_line_coding != last_line_coding) {
          last_line_coding = current_line_coding;
          line_state_changed = true;
        }

        if (line_state_changed) {
          co_await hal::print<128>(
            p_context,
            *console,
            "\n[LINE STATE]:(dtr=%d, rts=%d, baud=%lu, stop=%d, parity=%d, "
            "data_bits=%d)\n",
            static_cast<int>(last_control_line_state.dtr()),
            static_cast<int>(last_control_line_state.rts()),
            last_line_coding.baud_rate(),
            static_cast<int>(last_line_coding.stop_bits()),
            static_cast<int>(last_line_coding.parity()),
            static_cast<int>(last_line_coding.data_bits()));
        }

        auto const now = co_await clock->uptime(p_context);

        if (serial_interface->is_enumerated()) {
          if (not previously_enumerated) {
            co_await hal::write(p_context, *console, "\n[ENUMERATED]\n");
            previously_enumerated = true;
          }
          if (now >= dot_deadline) {
            co_await hal::write(p_context, *console, "+");
            dot_deadline =
              co_await hal::future_deadline(p_context, *clock, activity_period);
          }
          if (now >= send_deadline) {
            co_await serial_interface->write(
              p_context,
              mem::scatter_span<hal::byte const>{
                hal::as_bytes("Hello, World\n"sv) });
            co_await hal::write(
              p_context, *console, "\n[SENT]:(Hello, World\\n)\n");
            send_deadline =
              co_await hal::future_deadline(p_context, *clock, send_period);
          }
        } else {
          if (previously_enumerated) {
            co_await hal::write(p_context, *console, "\n[ENUMERATION LOST]\n");
            previously_enumerated = false;
          }
          if (now >= dot_deadline) {
            co_await hal::write(p_context, *console, "-");
            dot_deadline =
              co_await hal::future_deadline(p_context, *clock, activity_period);
          }
        }
      } catch (hal::exception const& p_error) {
        exception_caught = true;
        error_code_to_print = static_cast<int>(p_error.error_code());
      }
      if (exception_caught) {
        co_await hal::print<64>(p_context,
                                *console,
                                "\nException Error Code '%d' caught\n",
                                error_code_to_print);
      }
    }
  }
}
