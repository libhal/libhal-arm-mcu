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

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <span>

module hal.arm_mcu.stm32f1;

import hal;
import hal.util;
import hal.arm_mcu.cortex_m;

import :constants;
import :pin;
import :power;
import :clock;

namespace hal::stm32f1 {
namespace {
struct usb_endpoint_register_t
{
  u32 volatile epr;
};

struct usb_reg_t
{
  /// Endpoint registers
  std::array<usb_endpoint_register_t, usb::endpoint_count> ep;
  /// Reserved
  std::array<u32, usb::endpoint_count> reserved;
  /// Control register
  u32 volatile cntr;
  /// Interrupt status register
  u32 volatile istr;
  /// Frame number register
  u32 volatile fnr;
  /// Device address register
  u32 volatile daddr;
  /// Buffer table address register
  u32 volatile btable;
};

static_assert(offsetof(usb_reg_t, cntr) == 0x40);
static_assert(offsetof(usb_reg_t, btable) == 0x50);

// NOLINTNEXTLINE(performance-no-int-to-ptr)
auto* usb_reg = reinterpret_cast<usb_reg_t*>(0x4000'5C00);

usb_reg_t& reg()
{
  return *usb_reg;
}

enum class endpoint_type : u8
{
  bulk = 0b00,
  control = 0b01,
  iso = 0b10,
  interrupt = 0b11,
};

/// Bit masks for the CNTR register
struct control
{
  static constexpr auto force_reset = bit_mask::from<0>();
  static constexpr auto power_down = bit_mask::from<1>();
  static constexpr auto force_suspend = bit_mask::from<3>();
  // NOTE: remote-wakeup RESUME signaling (asserting this bit for ~2ms when an
  // IN endpoint write occurs while the host has suspended the bus with
  // wakeup permission granted) isn't implemented yet, mirroring v1's own
  // un-integrated `resume_if_suspended()`. `remote_wakeup_enable()`/
  // `remote_wakeup_granted()` only track the permission state for now.
  [[maybe_unused]] static constexpr auto resume_request = bit_mask::from<4>();
  static constexpr auto reset_interrupt = bit_mask::from<10>();
  static constexpr auto suspend_mode_interrupt = bit_mask::from<11>();
  static constexpr auto wakeup_interrupt = bit_mask::from<12>();
  static constexpr auto packet_memory_interrupt = bit_mask::from<14>();
  static constexpr auto correct_transfer_interrupt = bit_mask::from<15>();
};

/// Bit masks for the ISTR register
struct interrupt_status
{
  static constexpr auto endpoint_id = bit_mask::from<0, 3>();
  static constexpr auto direction = bit_mask::from<4>();
  static constexpr auto reset_request = bit_mask::from<10>();
  static constexpr auto suspend_mode_request = bit_mask::from<11>();
  static constexpr auto wake_up = bit_mask::from<12>();
  static constexpr auto error = bit_mask::from<13>();
  static constexpr auto packet_memory_over_underrun = bit_mask::from<14>();
  static constexpr auto correct_transfer = bit_mask::from<15>();
};

/// Bit masks for the DADDR register
struct device_address
{
  static constexpr auto address = bit_mask::from<0, 6>();
  static constexpr auto enable_function = bit_mask::from<7>();
};

/// Bit masks for the EPnR registers
struct endpoint
{
  static constexpr auto address = bit_mask::from<0, 3>();
  static constexpr auto status_tx = bit_mask::from<4, 5>();
  static constexpr auto correct_transfer_tx = bit_mask::from<7>();
  static constexpr auto kind = bit_mask::from<8>();
  static constexpr auto type = bit_mask::from<9, 10>();
  static constexpr auto setup_complete = bit_mask::from<11>();
  static constexpr auto status_rx = bit_mask::from<12, 13>();
  static constexpr auto correct_transfer_rx = bit_mask::from<15>();
};

/// Bit masks for the BDT's reception count word
struct block_table
{
  static constexpr auto block_size = bit_mask::from<15>();
  static constexpr auto number_of_blocks = bit_mask::from<10, 14>();
  static constexpr auto count = bit_mask::from<0, 9>();
};

/// Same STAT encoding for both STAT_TX and STAT_RX
enum class stat : u8
{
  disabled = 0b00,
  stall = 0b01,
  nak = 0b10,
  valid = 0b11,
};

// Memory internal to USB used for sending/receiving packets to/from the HOST
constexpr usize packet_buffer_sram_size = 512;
constexpr usize buffer_descriptor_table_start = 0;
// RM0008 p.650: reception byte count address for endpoint N is
// `[USB_BTABLE] + n*16 + 12`, the furthest the BTABLE could ever reach.
constexpr usize buffer_descriptor_table_end = (usb::endpoint_count * 16) + 12;

// TODO(#157): make this no longer fixed
constexpr u16 fixed_endpoint_size = 16;
constexpr auto block_number = fixed_endpoint_size / 2U;
constexpr auto rx_endpoint_count_mask =
  hal::bit_value(0U)
    .clear<block_table::block_size>()
    .insert<block_table::number_of_blocks>(block_number)
    .to<u16>();

std::span<u8> usb_packet_buffer_sram()
{
  // NOLINTNEXTLINE(performance-no-int-to-ptr)
  return { reinterpret_cast<u8*>(0x4000'6000), packet_buffer_sram_size };
}

std::span<u32> usb_packet_buffer_sram_u32()
{
  // NOTE: dividing by sizeof(u16) is not a mistake. The memory is u16
  // addressable by the USB peripheral but u32 accessible/aligned for the
  // processor: each u16 has padding of an additional u16 that goes nowhere.
  // The number of u16 blocks in this region equals the number of u32 blocks.
  // NOLINTNEXTLINE(performance-no-int-to-ptr)
  return { reinterpret_cast<u32*>(0x4000'6000),
           packet_buffer_sram_size / sizeof(u16) };
}

constexpr u16 tx_endpoint_memory_address(usize p_endpoint)
{
  auto const offset = (p_endpoint * 2) * fixed_endpoint_size;
  return static_cast<u16>(buffer_descriptor_table_end + offset);
}

constexpr u16 rx_endpoint_memory_address(usize p_endpoint)
{
  auto const offset = ((p_endpoint * 2) + 1) * fixed_endpoint_size;
  return static_cast<u16>(buffer_descriptor_table_end + offset);
}

struct buffer_descriptor_block
{
  u32 tx_address;
  u32 tx_count;
  u32 rx_address;
  u32 rx_count;

  void setup_ctrl_descriptor()
  {
    tx_address = tx_endpoint_memory_address(0);
    tx_count = 0;
    rx_address = rx_endpoint_memory_address(0);
    rx_count = rx_endpoint_count_mask;
  }

  void setup_in_endpoint_for(u8 p_endpoint)
  {
    tx_address = tx_endpoint_memory_address(p_endpoint);
    tx_count = 0;
  }

  void setup_out_endpoint_for(u8 p_endpoint)
  {
    rx_address = rx_endpoint_memory_address(p_endpoint);
    rx_count = rx_endpoint_count_mask;
  }

  [[nodiscard]] usize bytes_received() const
  {
    return bit_extract<block_table::count>(rx_count);
  }

  std::span<u32> rx_span()
  {
    auto const offset = rx_address / sizeof(u16);
    auto const size = bytes_received() / 2;
    return usb_packet_buffer_sram_u32().subspan(offset, size);
  }

  std::span<u32> tx_span()
  {
    auto const offset = tx_address / sizeof(u16);
    return usb_packet_buffer_sram_u32().subspan(offset,
                                                fixed_endpoint_size / 2);
  }
};

buffer_descriptor_block& endpoint_descriptor_block(usize p_endpoint)
{
  auto* buffer = usb_packet_buffer_sram().data();
  buffer += p_endpoint * sizeof(buffer_descriptor_block);
  return *std::bit_cast<buffer_descriptor_block*>(buffer);
}

constexpr auto endpoint_invariant_mask = hal::bit_value(0U)
                                           .insert<endpoint::address>(0xFUL)
                                           .set<endpoint::correct_transfer_tx>()
                                           .set<endpoint::kind>()
                                           .insert<endpoint::type>(0xFUL)
                                           .set<endpoint::correct_transfer_rx>()
                                           .to<u32>();

template<bit_mask mask>
void set_endpoint_register_toggle(usize p_endpoint, stat p_value)
{
  auto& endpoint_register = reg().ep[p_endpoint].epr;
  auto const reg_value = endpoint_register;
  auto const masked_endpoint_reg = reg_value & endpoint_invariant_mask;
  auto const desired_stat = value(p_value);
  auto const current_stat = bit_extract<mask>(reg_value);
  auto const toggle_mask = current_stat ^ desired_stat;
  endpoint_register = bit_value(masked_endpoint_reg)
                        .template insert<mask>(toggle_mask)
                        .template to<u32>();
}

void set_rx_stat(usize p_endpoint, stat p_stat)
{
  set_endpoint_register_toggle<endpoint::status_rx>(p_endpoint, p_stat);
}

void set_tx_stat(usize p_endpoint, stat p_stat)
{
  set_endpoint_register_toggle<endpoint::status_tx>(p_endpoint, p_stat);
}

void set_endpoint_address_and_type(usize p_endpoint, endpoint_type p_type)
{
  auto& endpoint_register = reg().ep[p_endpoint].epr;
  auto const masked_endpoint_reg = endpoint_register & endpoint_invariant_mask;
  endpoint_register = bit_value(masked_endpoint_reg)
                        .insert<endpoint::address>(p_endpoint)
                        .insert<endpoint::kind>(0U)
                        .insert<endpoint::type>(value(p_type))
                        .to<u32>();
}

template<bit_mask mask>
void clear_correct_transfer_for(u8 p_endpoint)
{
  auto& endpoint_reg = reg().ep[p_endpoint].epr;
  auto const masked_value = endpoint_reg & endpoint_invariant_mask;
  endpoint_reg =
    bit_value(masked_value).template clear<mask>().template to<u32>();
}

[[nodiscard]] bool endpoint_stalled_tx(u8 p_endpoint)
{
  return bit_extract<endpoint::status_tx>(reg().ep[p_endpoint].epr) <= 0b01;
}

[[nodiscard]] bool endpoint_stalled_rx(u8 p_endpoint)
{
  return bit_extract<endpoint::status_rx>(reg().ep[p_endpoint].epr) <= 0b01;
}

/// The hardware sets STAT_TX from VALID to NAK once the host has taken the
/// packet, so a VALID status means the transmission is still pending.
[[nodiscard]] bool transmission_pending(u8 p_endpoint)
{
  return bit_extract<endpoint::status_tx>(reg().ep[p_endpoint].epr) ==
         value(stat::valid);
}

void handle_bus_reset()
{
  reg().istr = 0;
  reg().daddr = 0;  // disable USB function

  std::ranges::fill(usb_packet_buffer_sram(), 0);

  reg().btable = buffer_descriptor_table_start;

  endpoint_descriptor_block(0).setup_ctrl_descriptor();
  set_endpoint_address_and_type(0, endpoint_type::control);
  set_rx_stat(0, stat::nak);
  set_tx_stat(0, stat::nak);

  // Reset endpoints 1..N to disabled
  for (usize i = 1; i < reg().ep.size(); i++) {
    reg().ep[i].epr = static_cast<u32>(i);
    set_rx_stat(i, stat::disabled);
    set_tx_stat(i, stat::disabled);
  }

  bit_modify(reg().cntr)
    .set(control::reset_interrupt)
    .set(control::correct_transfer_interrupt)
    .set(control::wakeup_interrupt)
    .set(control::suspend_mode_interrupt)
    .set(control::packet_memory_interrupt);

  bit_modify(reg().daddr).set(device_address::enable_function);
}

usize read_endpoint_bytes(u8 p_endpoint,
                          std::span<byte> p_buffer,
                          u16& p_bytes_read)
{
  auto const endpoint_stat = static_cast<stat>(
    bit_extract<endpoint::status_rx>(reg().ep[p_endpoint].epr));

  // NAK means a packet has arrived in USB SRAM and hasn't been fully
  // consumed yet; the endpoint automatically returns to VALID once all of it
  // has been read.
  if (endpoint_stat != stat::nak) {
    return 0;
  }

  auto& descriptor = endpoint_descriptor_block(p_endpoint);
  auto const bytes_remaining = descriptor.bytes_received() - p_bytes_read;
  auto const bytes_to_copy = std::min(bytes_remaining, p_buffer.size());
  auto const rx_span = descriptor.rx_span();

  for (usize idx = 0; idx < bytes_to_copy; idx++) {
    auto const offset_index = p_bytes_read + idx;
    auto const word = rx_span[offset_index / 2U];
    auto const shift = (offset_index & 1U) ? 8U : 0U;
    p_buffer[idx] = static_cast<byte>((word >> shift) & 0xFF);
  }

  p_bytes_read = static_cast<u16>(p_bytes_read + bytes_to_copy);

  if (p_bytes_read == descriptor.bytes_received()) {
    p_bytes_read = 0;
    set_rx_stat(p_endpoint, stat::valid);
  }

  return bytes_to_copy;
}
}  // namespace

/// Implementation state shared by every resource acquired from a `usb`
/// manager. The pimpl pointer doubles as the "USB is in use" singleton guard
/// for the ISR, matching the pattern used by `can::impl`.
struct usb::impl
{
  /// Notified by the ISR when an endpoint's IN transfer completes.
  std::array<hal::notifier, usb::endpoint_count> tx_notifier{};
  /// Invoked by the ISR when an OUT endpoint receives data. Endpoint 0 reports
  /// its packets through `bus_event_callback` instead.
  std::array<mem::optional_ptr<hal::usb::receive_callback>, usb::endpoint_count>
    receive_callback{};
  mem::optional_ptr<hal::usb::bus_event_callback> bus_event_callback{};
  bool setup_packet_pending = false;
  bool wake_enable = false;
  // Starts at 1: endpoint 0 is always reserved for the control endpoint.
  u8 endpoints_allocated = 1;
};

namespace {
// USB is a singleton peripheral (one register bank, shared packet memory),
// so its interrupt handlers reach the active manager through a plain static
// pointer, matching `can::impl`'s `s_active_can`.
usb::impl* s_active_usb = nullptr;

void usb_interrupt_handler() noexcept;

extern "C" void usb_lp_can1_rx0_isr()
{
  usb_interrupt_handler();
}

extern "C" void usb_hp_can1_tx_isr()
{
  usb_interrupt_handler();
}

// Callbacks are swapped with the USB interrupts masked so the ISR never
// observes a partially written callback. Interrupts that arrive meanwhile stay
// pending in the NVIC and are serviced on unmasking.
void mask_interrupts()
{
  hal::cortex_m::disable_interrupt(irq::usb_lp_can1_rx0);
  hal::cortex_m::disable_interrupt(irq::usb_hp_can1_tx);
}

void unmask_interrupts()
{
  // Only low priority is serviced; high priority is for isochronous
  // endpoints, which this driver does not yet support.
  hal::cortex_m::enable_interrupt(irq::usb_lp_can1_rx0, usb_lp_can1_rx0_isr);
  hal::cortex_m::enable_interrupt(irq::usb_hp_can1_tx, usb_hp_can1_tx_isr);
}

void notify_bus_event(usb::impl& p_state, hal::usb::bus_event p_event) noexcept
{
  if (p_state.bus_event_callback.has_value()) {
    p_state.bus_event_callback.value()->callback(p_event);
  }
}

void usb_interrupt_handler() noexcept
{
  auto* state = s_active_usb;
  if (state == nullptr) {
    return;
  }

  auto& interrupt_reg = reg().istr;
  auto const interrupt_value = interrupt_reg;
  auto const endpoint_id = static_cast<u8>(
    bit_extract<interrupt_status::endpoint_id>(interrupt_value));
  auto const direction =
    bit_extract<interrupt_status::direction>(interrupt_value);
  bool const transfer_completed =
    bit_extract<interrupt_status::correct_transfer>(interrupt_value);

  if (endpoint_id == 0) {
    state->setup_packet_pending =
      bit_extract<endpoint::setup_complete>(reg().ep[0].epr);
  }

  if (transfer_completed) {
    if (direction == 0) {
      // TX (IN) transfer complete
      clear_correct_transfer_for<endpoint::correct_transfer_tx>(endpoint_id);
      state->tx_notifier[endpoint_id].notify();
    } else {
      // RX (OUT/SETUP) transfer complete
      clear_correct_transfer_for<endpoint::correct_transfer_rx>(endpoint_id);
      if (endpoint_id == 0) {
        notify_bus_event(*state,
                         state->setup_packet_pending
                           ? hal::usb::bus_event::setup_packet
                           : hal::usb::bus_event::data_packet);
      } else if (state->receive_callback[endpoint_id].has_value()) {
        state->receive_callback[endpoint_id].value()->callback();
      }
    }
  }

  if (bit_extract<interrupt_status::reset_request>(interrupt_value)) {
    handle_bus_reset();
    notify_bus_event(*state, hal::usb::bus_event::reset);
  }

  if (bit_extract<interrupt_status::suspend_mode_request>(interrupt_value)) {
    bit_modify(reg().cntr).set<control::force_suspend>();
    notify_bus_event(*state, hal::usb::bus_event::suspend);
    interrupt_reg = ~(1U << interrupt_status::suspend_mode_request.position);
  }

  if (bit_extract<interrupt_status::wake_up>(interrupt_value)) {
    bit_modify(reg().cntr).clear<control::force_suspend>();
    notify_bus_event(*state, hal::usb::bus_event::resume);
    interrupt_reg = ~(1U << interrupt_status::wake_up.position);
  }

  if (bit_extract<interrupt_status::packet_memory_over_underrun>(
        interrupt_value)) {
    interrupt_reg =
      ~(1U << interrupt_status::packet_memory_over_underrun.position);
  }

  // TODO(#202 upstream): do something with the USB error-detected interrupt
  if (bit_extract<interrupt_status::error>(interrupt_value)) {
    interrupt_reg = ~(1U << interrupt_status::error.position);
  }
}

class control_endpoint final : public hal::usb::control_endpoint
{
public:
  explicit control_endpoint(hal::ptr<usb> p_manager)
    : m_manager(p_manager)
  {
    set_rx_stat(0, stat::valid);
    endpoint_descriptor_block(0).tx_count = 0;
  }

  control_endpoint(control_endpoint const&) = delete;
  control_endpoint& operator=(control_endpoint const&) = delete;
  control_endpoint(control_endpoint&&) = delete;
  control_endpoint& operator=(control_endpoint&&) = delete;
  ~control_endpoint() = default;

private:
  [[nodiscard]] hal::usb::endpoint_info driver_info() const override
  {
    return {
      .size = fixed_endpoint_size,
      .number = 0,
      .stalled = false,
    };
  }

  void driver_stall(bool p_should_stall) override
  {
    if (p_should_stall) {
      set_rx_stat(0, stat::stall);
      set_tx_stat(0, stat::stall);
    } else {
      set_rx_stat(0, stat::valid);
      set_tx_stat(0, stat::nak);
    }
  }

  void driver_reset() override
  {
    set_rx_stat(0, stat::valid);
    endpoint_descriptor_block(0).tx_count = 0;
  }

  void driver_connect(bool p_should_connect) override
  {
    bit_modify(reg().daddr)
      .insert<device_address::enable_function>(p_should_connect);
  }

  void driver_set_address(u8 p_address) override
  {
    bit_modify(reg().daddr)
      .set(device_address::enable_function)
      .insert<device_address::address>(p_address);
  }

  void driver_write(mem::scatter_span<byte const> p_data) override
  {
    set_rx_stat(0, stat::stall);
    set_tx_stat(0, stat::nak);
    m_manager->write(0, p_data);
  }

  usize driver_read(mem::scatter_span<byte> p_data_list) override
  {
    usize total = 0;
    for (auto const& data : p_data_list) {
      auto const copied = read_endpoint_bytes(0, data, m_bytes_read);
      if (copied == 0) {
        if (m_bytes_read == 8 && m_manager->setup_packet_pending()) {
          m_manager->clear_setup_packet_flag();
        }
        break;
      }
      total += copied;
    }
    return total;
  }

  void driver_on_bus_event(
    mem::optional_ptr<hal::usb::bus_event_callback> const& p_callback) override
  {
    m_manager->on_bus_event(p_callback);
  }

  void driver_remote_wakeup_enable(bool p_enabled) override
  {
    m_manager->remote_wakeup_enable(p_enabled);
  }

  bool driver_remote_wakeup_granted() override
  {
    return m_manager->remote_wakeup_granted();
  }

  void driver_acknowledge_sleep(bool) override
  {
  }

  hal::usb::lpm_support driver_supports_lpm() override
  {
    return hal::usb::lpm_support().remote_wakeup_supported(true);
  }

  hal::ptr<usb> m_manager;
  u16 m_bytes_read = 0;
};

template<hal::usb::in_endpoint_type Interface>
class in_endpoint final : public Interface
{
public:
  in_endpoint(hal::ptr<usb> p_manager, u8 p_endpoint_number)
    : m_manager(p_manager)
    , m_endpoint_number(p_endpoint_number)
  {
    configure();
  }

  in_endpoint(in_endpoint const&) = delete;
  in_endpoint& operator=(in_endpoint const&) = delete;
  in_endpoint(in_endpoint&&) = delete;
  in_endpoint& operator=(in_endpoint&&) = delete;
  ~in_endpoint() = default;

private:
  static constexpr endpoint_type physical_type =
    std::is_base_of_v<hal::usb::interrupt_in_endpoint, Interface>
      ? endpoint_type::interrupt
      : endpoint_type::bulk;

  void configure()
  {
    endpoint_descriptor_block(m_endpoint_number)
      .setup_in_endpoint_for(m_endpoint_number);
    set_endpoint_address_and_type(m_endpoint_number, physical_type);
    set_rx_stat(m_endpoint_number, stat::nak);
  }

  void driver_write(mem::scatter_span<byte const> p_data) override
  {
    m_manager->write(m_endpoint_number, p_data);
  }

  [[nodiscard]] hal::usb::endpoint_info driver_info() const override
  {
    return {
      .size = fixed_endpoint_size,
      .number = static_cast<u8>(m_endpoint_number | (1U << 7U)),
      .stalled = endpoint_stalled_tx(m_endpoint_number),
    };
  }

  void driver_stall(bool p_should_stall) override
  {
    set_tx_stat(m_endpoint_number, p_should_stall ? stat::stall : stat::nak);
  }

  void driver_reset() override
  {
    configure();
  }

  hal::ptr<usb> m_manager;
  u8 m_endpoint_number;
};

template<hal::usb::out_endpoint_type Interface>
class out_endpoint final : public Interface
{
public:
  out_endpoint(hal::ptr<usb> p_manager, u8 p_endpoint_number)
    : m_manager(p_manager)
    , m_endpoint_number(p_endpoint_number)
  {
    configure();
  }

  out_endpoint(out_endpoint const&) = delete;
  out_endpoint& operator=(out_endpoint const&) = delete;
  out_endpoint(out_endpoint&&) = delete;
  out_endpoint& operator=(out_endpoint&&) = delete;
  ~out_endpoint() = default;

private:
  static constexpr endpoint_type physical_type =
    std::is_base_of_v<hal::usb::interrupt_out_endpoint, Interface>
      ? endpoint_type::interrupt
      : endpoint_type::bulk;

  void configure()
  {
    endpoint_descriptor_block(m_endpoint_number)
      .setup_out_endpoint_for(m_endpoint_number);
    set_endpoint_address_and_type(m_endpoint_number, physical_type);
    set_rx_stat(m_endpoint_number, stat::valid);
  }

  void driver_on_receive(
    mem::optional_ptr<hal::usb::receive_callback> const& p_callback) override
  {
    m_manager->on_receive(m_endpoint_number, p_callback);
  }

  [[nodiscard]] hal::usb::endpoint_info driver_info() const override
  {
    return {
      .size = fixed_endpoint_size,
      .number = m_endpoint_number,
      .stalled = endpoint_stalled_rx(m_endpoint_number),
    };
  }

  void driver_stall(bool p_should_stall) override
  {
    set_rx_stat(m_endpoint_number, p_should_stall ? stat::stall : stat::valid);
  }

  usize driver_read(mem::scatter_span<byte> p_data_list) override
  {
    usize total = 0;
    for (auto const& data : p_data_list) {
      auto const copied =
        read_endpoint_bytes(m_endpoint_number, data, m_bytes_read);
      if (copied == 0) {
        break;
      }
      total += copied;
    }
    return total;
  }

  void driver_reset() override
  {
    configure();
  }

  hal::ptr<usb> m_manager;
  u16 m_bytes_read = 0;
  u8 m_endpoint_number;
};
}  // namespace

hal::ptr<usb> usb::create(hal::allocator p_allocator)
{
  using namespace std::chrono_literals;
  using namespace mp_units::si::unit_symbols;

  // USB and CAN1 physically share the same packet/message memory on the
  // STM32F1, so only one may be powered on at a time.
  if (is_on(peripheral::usb) or is_on(peripheral::can1)) {
    throw hal::device_or_resource_busy(nullptr);
  }

  if (frequency(peripheral::usb) != 48 * MHz) {
    throw hal::operation_not_supported(nullptr);
  }

  // Must be done prior to lifting the power-down bit.
  configure_pin({ .port = 'A', .pin = 11 }, push_pull_alternative_output);
  configure_pin({ .port = 'A', .pin = 12 }, push_pull_alternative_output);

  power_on(peripheral::usb);

  std::ranges::fill(usb_packet_buffer_sram(), 0);

  hal::sleep_for(1ms);

  // Perform reset
  bit_modify(reg().cntr).set(control::power_down).set(control::force_reset);
  hal::sleep_for(1ms);

  // The USB peripheral sets this bit on system reset; clear it to allow the
  // device to power on. RM0008 requires a short settle time before
  // proceeding.
  bit_modify(reg().cntr).clear(control::power_down);
  hal::sleep_for(1ms);

  // Clears everything including force_reset, enabling the USB device.
  reg().cntr = 0;

  handle_bus_reset();

  auto manager = hal::allocate<usb>(p_allocator, private_key{}, p_allocator);

  s_active_usb = &manager->inner();

  hal::cortex_m::initialize_interrupts<irq::max>();
  unmask_interrupts();

  return manager;
}

usb::usb(private_key, hal::allocator p_allocator)
  : pimpl(p_allocator)
{
}

usb::~usb()
{
  configure_pin({ .port = 'A', .pin = 11 }, input_pull_up);
  configure_pin({ .port = 'A', .pin = 12 }, input_pull_up);
  hal::cortex_m::disable_interrupt(irq::usb_lp_can1_rx0);
  hal::cortex_m::disable_interrupt(irq::usb_hp_can1_tx);
  s_active_usb = nullptr;
  power_off(peripheral::usb);
}

namespace {
/// Hand the buffered packet to the host and block until it has been taken.
void transmit(usb::impl& p_state, u8 p_endpoint)
{
  // The transfer complete interrupt stays enabled for the life of the
  // peripheral, so there is nothing to disarm when the operation ends.
  hal::notifier::scope operation(p_state.tx_notifier[p_endpoint],
                                 []() noexcept {});
  set_tx_stat(p_endpoint, stat::valid);
  while (transmission_pending(p_endpoint)) {
    operation.block();
  }
  endpoint_descriptor_block(p_endpoint).tx_count = 0;
}
}  // namespace

void usb::write(u8 p_endpoint, mem::scatter_span<byte const> p_data)
{
  auto& descriptor = endpoint_descriptor_block(p_endpoint);
  auto const tx_span = descriptor.tx_span();

  for (auto const& chunk : p_data) {
    auto remaining = chunk;
    while (not remaining.empty()) {
      if (descriptor.tx_count >= fixed_endpoint_size) {
        transmit(inner(), p_endpoint);
      }

      if (descriptor.tx_count & 0b1) {
        bit_modify(tx_span[descriptor.tx_count / 2U])
          .insert<byte_m<1>>(remaining[0]);
      } else {
        bit_modify(tx_span[descriptor.tx_count / 2U])
          .insert<byte_m<0>>(remaining[0]);
      }
      remaining = remaining.subspan(1);
      descriptor.tx_count++;
    }
  }

  // An explicit empty write (or a call to flush()) sends whatever is
  // currently buffered, i.e. the terminating short packet / ZLP.
  if (p_data.length() == 0) {
    transmit(inner(), p_endpoint);
  }
}

void usb::on_receive(
  u8 p_endpoint,
  mem::optional_ptr<hal::usb::receive_callback> const& p_callback)
{
  mask_interrupts();
  inner().receive_callback[p_endpoint] = p_callback;
  unmask_interrupts();
}

void usb::on_bus_event(
  mem::optional_ptr<hal::usb::bus_event_callback> const& p_callback)
{
  mask_interrupts();
  inner().bus_event_callback = p_callback;
  unmask_interrupts();
}

u8 usb::allocate_endpoint_number()
{
  auto& state = inner();
  if (state.endpoints_allocated >= endpoint_count) {
    throw hal::resource_unavailable_try_again(this);
  }
  return state.endpoints_allocated++;
}

void usb::remote_wakeup_enable(bool p_enabled)
{
  inner().wake_enable = p_enabled;
}

bool usb::remote_wakeup_granted()
{
  return inner().wake_enable;
}

bool usb::setup_packet_pending()
{
  return inner().setup_packet_pending;
}

void usb::clear_setup_packet_flag()
{
  inner().setup_packet_pending = false;
}

hal::ptr<hal::usb::control_endpoint> usb::acquire_control_endpoint()
{
  return hal::allocate<control_endpoint>(memory_resource(), strong_from_this());
}

usb_interrupt_endpoint_pair usb::acquire_interrupt_endpoint()
{
  auto const endpoint_number = allocate_endpoint_number();
  auto out = hal::allocate<out_endpoint<hal::usb::interrupt_out_endpoint>>(
    memory_resource(), strong_from_this(), endpoint_number);
  auto in = hal::allocate<in_endpoint<hal::usb::interrupt_in_endpoint>>(
    memory_resource(), strong_from_this(), endpoint_number);
  return { .out = out, .in = in };
}

usb_bulk_endpoint_pair usb::acquire_bulk_endpoint()
{
  auto const endpoint_number = allocate_endpoint_number();
  auto out = hal::allocate<out_endpoint<hal::usb::bulk_out_endpoint>>(
    memory_resource(), strong_from_this(), endpoint_number);
  auto in = hal::allocate<in_endpoint<hal::usb::bulk_in_endpoint>>(
    memory_resource(), strong_from_this(), endpoint_number);
  return { .out = out, .in = in };
}
}  // namespace hal::stm32f1
