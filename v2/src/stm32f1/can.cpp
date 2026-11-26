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
#include <atomic>
#include <bit>
#include <bitset>
#include <chrono>
#include <mutex>
#include <optional>
#include <span>
#include <utility>

module hal.arm_mcu.stm32f1;

import hal;
import hal.util;
import hal.arm_mcu.cortex_m;

import :constants;
import :pin;
import :power;
import :clock;
import :can;

namespace hal::stm32f1 {
namespace {
struct can_tx_mailbox_t
{
  u32 volatile tir;
  u32 volatile tdtr;
  u32 volatile tdlr;
  u32 volatile tdhr;
};

struct can_fifo_mailbox_t
{
  u32 volatile rir;
  u32 volatile rdtr;
  u32 volatile rdlr;
  u32 volatile rdhr;
};

struct can_filter_register_t
{
  u32 volatile fr1;
  u32 volatile fr2;
};

/// Register map for the bxCAN peripheral (RM0008 section 24.9)
struct can_reg_t
{
  u32 volatile mcr;
  u32 volatile msr;
  u32 volatile tsr;
  std::array<u32 volatile, 2> rfr;
  u32 volatile ier;
  u32 volatile esr;
  u32 volatile btr;
  std::array<u32, 88> reserved0;
  std::array<can_tx_mailbox_t, 3> transmit_mailbox;
  std::array<can_fifo_mailbox_t, 2> fifo_mailbox;
  std::array<u32, 12> reserved1;
  u32 volatile fmr;
  u32 volatile fm1r;
  u32 reserved2;
  u32 volatile fs1r;
  u32 reserved3;
  u32 volatile ffa1r;
  u32 reserved4;
  u32 volatile fa1r;
  std::array<u32, 8> reserved5;
  /// 14 banks on the STM32F103, 28 on connectivity line parts
  std::array<can_filter_register_t, 28> filter_registers;
};
static_assert(offsetof(can_reg_t, transmit_mailbox) == 0x180);
static_assert(offsetof(can_reg_t, fifo_mailbox) == 0x1B0);
static_assert(offsetof(can_reg_t, fmr) == 0x200);
static_assert(offsetof(can_reg_t, fa1r) == 0x21C);
static_assert(offsetof(can_reg_t, filter_registers) == 0x240);

// NOLINTNEXTLINE(performance-no-int-to-ptr)
inline auto* const can1_reg = reinterpret_cast<can_reg_t*>(0x4000'6400);

/// Bit masks for the BTR register
struct bus_timing
{
  static constexpr auto prescaler = bit_mask::from<0, 9>();
  static constexpr auto time_segment1 = bit_mask::from<16, 19>();
  static constexpr auto time_segment2 = bit_mask::from<20, 22>();
  static constexpr auto sync_jump_width = bit_mask::from<24, 25>();
  static constexpr auto loop_back_mode = bit_mask::from<30>();
  static constexpr auto silent_mode = bit_mask::from<31>();
};

/// Bit masks for the MCR register
struct master_control
{
  static constexpr auto initialization_request = bit_mask::from<0>();
  static constexpr auto sleep_mode_request = bit_mask::from<1>();
  static constexpr auto transmit_fifo_priority = bit_mask::from<2>();
  static constexpr auto no_automatic_retransmission = bit_mask::from<4>();
  static constexpr auto automatic_bus_off_management = bit_mask::from<6>();
};

/// Bit masks for the MSR register
struct master_status
{
  static constexpr auto initialization_acknowledge = bit_mask::from<0>();
  static constexpr auto error_interrupt = bit_mask::from<2>();
};

/// Bit masks for the TSR register
struct transmit_status
{
  /// RQCP0, RQCP1 and RQCP2: a mailbox's request completed. Cleared by
  /// writing 1.
  static constexpr u32 requests_completed =
    (1U << 0U) | (1U << 8U) | (1U << 16U);
  /// TME0 to TME2: which mailboxes are empty
  static constexpr auto mailbox_empty = bit_mask::from<26, 28>();
};

/// Bit masks for the IER register
struct interrupt_enable
{
  static constexpr auto transmit_mailbox_empty = bit_mask::from<0>();
  static constexpr auto fifo0_message_pending = bit_mask::from<1>();
  static constexpr auto fifo1_message_pending = bit_mask::from<4>();
  static constexpr auto bus_off = bit_mask::from<10>();
  static constexpr auto error = bit_mask::from<15>();
};

/// Bit mask for the ESR register
struct error_status
{
  static constexpr auto bus_off = bit_mask::from<2>();
};

/// Bit masks shared by the TIxR/RIxR mailbox identifier registers
struct mailbox_identifier
{
  static constexpr auto transmit_request = bit_mask::from<0>();
  static constexpr auto remote_request = bit_mask::from<1>();
  static constexpr auto extended = bit_mask::from<2>();
  static constexpr auto standard_identifier = bit_mask::from<21, 31>();
  static constexpr auto extended_identifier = bit_mask::from<3, 31>();
};

/// Bit mask for the TDTxR/RDTxR frame length registers
struct frame_length
{
  static constexpr auto data_length_code = bit_mask::from<0, 3>();
};

/// Bit masks for the RF0R/RF1R FIFO registers
struct fifo_status
{
  static constexpr auto messages_pending = bit_mask::from<0, 1>();
  static constexpr auto release_output_mailbox = bit_mask::from<5>();
};

/// Bit mask for the FMR filter master register
struct filter_master
{
  static constexpr auto initialization_mode = bit_mask::from<0>();
};

/// The bank that accepts every message while the filter mode is `all`
constexpr u8 accept_all_bank = 13;

/// The longest frame, an extended frame with 8 bytes of data and the most bit
/// stuffing, in bits
constexpr u32 max_frame_bits = 160;

can* s_can = nullptr;

void enter_initialization()
{
  bit_modify(can1_reg->mcr).set<master_control::initialization_request>();
  while (
    not bit_is_set<master_status::initialization_acknowledge>(can1_reg->msr)) {
    continue;
  }
}

void exit_initialization()
{
  bit_modify(can1_reg->mcr).clear<master_control::initialization_request>();
  while (bit_is_set<master_status::initialization_acknowledge>(can1_reg->msr)) {
    continue;
  }
}

/// Program the bit timing. Must run in initialization mode.
/// @return hal::hertz - the baud rate achieved
hal::hertz configure_bit_timing(hal::hertz p_baud_rate, bool p_loop_back)
{
  auto const can_frequency = frequency(peripheral::can1);
  auto const divider = calculate_can_bus_divider(can_frequency, p_baud_rate);
  if (not divider) {
    throw hal::operation_not_supported(nullptr);
  }

  auto const timing = *divider;
  u32 const quanta = timing.sync_segment + timing.propagation_delay +
                     timing.phase_segment1 + timing.phase_segment2;
  bit_modify(can1_reg->btr)
    .insert<bus_timing::prescaler>(timing.clock_divider - 1U)
    .insert<bus_timing::time_segment1>(
      (timing.phase_segment1 + timing.propagation_delay) - 1U)
    .insert<bus_timing::time_segment2>(timing.phase_segment2 - 1U)
    .insert<bus_timing::sync_jump_width>(timing.synchronization_jump_width - 1U)
    .insert<bus_timing::loop_back_mode>(p_loop_back)
    .clear<bus_timing::silent_mode>();

  auto const can_hz = can_frequency.numerical_value_in(mp_units::si::hertz);
  return (can_hz / (timing.clock_divider * quanta)) * mp_units::si::hertz;
}

bool is_bus_off()
{
  return bit_is_set<error_status::bus_off>(can1_reg->esr);
}

void set_filter_initialization(bool p_initialization)
{
  bit_modify(can1_reg->fmr)
    .insert<filter_master::initialization_mode>(p_initialization);
}

/// Set a bank's scale, mode and FIFO. Only allowed in filter initialization
/// mode, which stops reception, so it's left at once.
void configure_bank(u8 p_bank, bool p_32_bit, bool p_list, can_fifo p_fifo)
{
  auto const bank = bit_mask::from(p_bank);
  set_filter_initialization(true);
  bit_modify(can1_reg->fs1r).insert(bank, static_cast<u32>(p_32_bit));
  bit_modify(can1_reg->fm1r).insert(bank, static_cast<u32>(p_list));
  bit_modify(can1_reg->ffa1r).insert(bank, value(p_fifo));
  set_filter_initialization(false);
}

hal::can_message read_fifo(u8 p_fifo)
{
  auto& mailbox = can1_reg->fifo_mailbox[p_fifo];
  auto const id = mailbox.rir;
  auto const frame = mailbox.rdtr;
  auto const low = mailbox.rdlr;
  auto const high = mailbox.rdhr;

  hal::can_message message{};
  message.extended = bit_is_set<mailbox_identifier::extended>(id);
  message.remote_request = bit_is_set<mailbox_identifier::remote_request>(id);
  message.id = message.extended
                 ? bit_extract<mailbox_identifier::extended_identifier>(id)
                 : bit_extract<mailbox_identifier::standard_identifier>(id);
  // A data length code of 9 to 15 still carries 8 bytes
  message.length = static_cast<u8>(
    std::min(bit_extract<frame_length::data_length_code>(frame), 8U));
  for (u32 i = 0; i < 4; i++) {
    message.payload[i] = static_cast<hal::byte>(low >> (i * 8U));
    message.payload[i + 4] = static_cast<hal::byte>(high >> (i * 8U));
  }

  bit_modify(can1_reg->rfr[p_fifo]).set<fifo_status::release_output_mailbox>();
  return message;
}

/// The pins of each pin set: RX first
struct can_pin_pair
{
  pin rx;
  pin tx;
};

can_pin_pair pins_for(can_pins p_pins)
{
  switch (p_pins) {
    case can_pins::pb9_pb8:
      return { .rx = { .port = 'B', .pin = 8 },
               .tx = { .port = 'B', .pin = 9 } };
    case can_pins::pd0_pd1:
      return { .rx = { .port = 'D', .pin = 0 },
               .tx = { .port = 'D', .pin = 1 } };
    case can_pins::pa11_pa12:
    default:
      return { .rx = { .port = 'A', .pin = 11 },
               .tx = { .port = 'A', .pin = 12 } };
  }
}

/// The bank register bits of one filter value, before it's shifted into its
/// slot
template<typename Allowed>
struct filter_format;

/// A 16-bit list entry: STID in bits 5 to 15, RTR and IDE clear, so it
/// accepts standard data frames
template<>
struct filter_format<hal::u16>
{
  static constexpr bool is_32_bit = false;
  static constexpr bool is_list = true;
  static hal::u64 encode(hal::u16 p_id)
  {
    return static_cast<hal::u64>(p_id & 0x7FFU) << 5U;
  }
};

/// A 32-bit list entry: the 29-bit ID in bits 3 to 31, IDE set, RTR clear, so
/// it accepts extended data frames
template<>
struct filter_format<hal::u32>
{
  static constexpr bool is_32_bit = true;
  static constexpr bool is_list = true;
  static hal::u64 encode(hal::u32 p_id)
  {
    return (static_cast<hal::u64>(p_id & 0x1FFF'FFFFU) << 3U) | 0b100U;
  }
};

/// A 16-bit ID in the lower half and its mask in the upper half. The mask
/// also requires IDE to be clear, so only standard frames match, and leaves
/// RTR out, so data and remote frames both do.
template<>
struct filter_format<hal::can_mask>
{
  static constexpr bool is_32_bit = false;
  static constexpr bool is_list = false;
  static hal::u64 encode(hal::can_mask p_mask)
  {
    auto const id = static_cast<hal::u64>(p_mask.id & 0x7FFU) << 5U;
    auto const mask =
      (static_cast<hal::u64>(p_mask.mask & 0x7FFU) << 5U) | 0b1000U;
    return id | (mask << 16U);
  }
};

/// A 32-bit ID in FR1 and its mask in FR2. The mask also requires IDE to be
/// set, so only extended frames match, and leaves RTR out.
template<>
struct filter_format<hal::can_mask_ext>
{
  static constexpr bool is_32_bit = true;
  static constexpr bool is_list = false;
  static hal::u64 encode(hal::can_mask_ext p_mask)
  {
    auto const id =
      (static_cast<hal::u64>(p_mask.id & 0x1FFF'FFFFU) << 3U) | 0b100U;
    auto const mask =
      (static_cast<hal::u64>(p_mask.mask & 0x1FFF'FFFFU) << 3U) | 0b100U;
    return id | (mask << 32U);
  }
};
}  // namespace

can::can(hal::ref<hal::basic_lock> p_lock,
         std::span<hal::can_message> p_buffer,
         can_settings const& p_settings)
  : m_lock(p_lock)
  , m_receive_buffer(p_buffer)
  , m_baud_rate(p_settings.baud_rate)
  , m_pins(p_settings.pins)
{
  if (p_buffer.empty()) {
    throw hal::argument_out_of_domain(this);
  }
  auto const pins = pins_for(m_pins);
  // Claim everything or nothing: check first, so a claim that succeeds is
  // never left behind by one that throws.
  if (is_claimed(peripheral::can1) or is_claimed(peripheral::usb) or
      is_pin_claimed(pins.rx) or is_pin_claimed(pins.tx)) {
    throw hal::device_or_resource_busy(this);
  }
  claim(peripheral::can1);
  claim(peripheral::usb);
  claim_pin(pins.rx);
  claim_pin(pins.tx);

  power_on(peripheral::can1);
  // Start from the reset state, whatever ran before (e.g. a bootloader) left.
  reset_peripheral(peripheral::can1);

  // Leave sleep mode, which the reset state starts in, for initialization
  bit_modify(can1_reg->mcr)
    .clear<master_control::sleep_mode_request>()
    .clear<master_control::no_automatic_retransmission>()
    .clear<master_control::automatic_bus_off_management>()
    .set<master_control::transmit_fifo_priority>();
  enter_initialization();
  try {
    m_baud_rate =
      configure_bit_timing(p_settings.baud_rate, p_settings.loop_back);
  } catch (...) {
    power_off(peripheral::can1);
    release_pin(pins.rx);
    release_pin(pins.tx);
    release(peripheral::usb);
    release(peripheral::can1);
    throw;
  }

  configure_pin(pins.rx, input_pull_up);
  configure_pin(pins.tx, push_pull_alternative_output);
  remap_pins(m_pins);

  // Bank 13 as one 32-bit mask with a mask of 0 accepts every message
  configure_bank(accept_all_bank, true, false, can_fifo::select1);
  can1_reg->filter_registers[accept_all_bank].fr1 = 0;
  can1_reg->filter_registers[accept_all_bank].fr2 = 0;
  apply_bank_activation();

  exit_initialization();

  s_can = this;
  cortex_m::enable_interrupt(irq::can1_rx0, &fifo0_isr);
  cortex_m::enable_interrupt(irq::can1_rx1, &fifo1_isr);
  cortex_m::enable_interrupt(irq::can1_tx, &transmit_isr);
  cortex_m::enable_interrupt(irq::can1_sce, &status_change_isr);
  // The transmit interrupt is only enabled while a send waits for a mailbox
  bit_modify(can1_reg->ier)
    .set<interrupt_enable::fifo0_message_pending>()
    .set<interrupt_enable::fifo1_message_pending>()
    .set<interrupt_enable::bus_off>()
    .set<interrupt_enable::error>();
}

can::~can()
{
  can1_reg->ier = 0;
  cortex_m::disable_interrupt(irq::can1_rx0);
  cortex_m::disable_interrupt(irq::can1_rx1);
  cortex_m::disable_interrupt(irq::can1_tx);
  cortex_m::disable_interrupt(irq::can1_sce);
  s_can = nullptr;
  power_off(peripheral::can1);

  auto const pins = pins_for(m_pins);
  reset_pin(pins.rx);
  reset_pin(pins.tx);
  release_pin(pins.rx);
  release_pin(pins.tx);
  release(peripheral::usb);
  release(peripheral::can1);
}

void can::fifo0_isr()
{
  s_can->receive(0);
}

void can::fifo1_isr()
{
  s_can->receive(1);
}

void can::transmit_isr()
{
  // Clear the completed requests, so the interrupt stops firing, and wake
  // the send waiting for a mailbox.
  can1_reg->tsr = transmit_status::requests_completed;
  s_can->m_transmit_notifier.notify();
}

void can::status_change_isr()
{
  // Writing 1 clears the error interrupt flag. The other flags in the
  // register ignore a 0.
  can1_reg->msr = bit_value(0U).set<master_status::error_interrupt>().get();
  if (is_bus_off()) {
    // A send waiting for a mailbox would wait forever
    s_can->m_transmit_notifier.notify();
    if (auto* const callback = s_can->m_bus_off_callback) {
      callback->callback();
    }
  }
}

void can::receive(u8 p_fifo) noexcept
{
  while (bit_extract<fifo_status::messages_pending>(can1_reg->rfr[p_fifo]) !=
         0) {
    auto const message = read_fifo(p_fifo);
    if (not m_receive_buffer.empty()) {
      auto const count = m_receive_count;
      m_receive_buffer[count % m_receive_buffer.size()] = message;
      // The message is in the buffer before the count says so
      std::atomic_signal_fence(std::memory_order_seq_cst);
      m_receive_count = count + 1U;
    }
    auto* const callback = m_receive_callback;
    if ((callback != nullptr) and m_receive_filter.matches(message)) {
      callback->callback(message);
    }
  }
}

void can::transmit(hal::can_message const& p_message)
{
  std::lock_guard const hold(*m_lock);

  u32 id =
    bit_value(0U)
      .insert<mailbox_identifier::remote_request>(p_message.remote_request)
      .insert<mailbox_identifier::extended>(p_message.extended)
      .get();
  if (p_message.extended) {
    id = bit_value(id)
           .insert<mailbox_identifier::extended_identifier>(p_message.id)
           .get();
  } else {
    id = bit_value(id)
           .insert<mailbox_identifier::standard_identifier>(p_message.id)
           .get();
  }
  u32 low = 0;
  u32 high = 0;
  for (u32 i = 0; i < 4; i++) {
    low = low | (static_cast<u32>(p_message.payload[i]) << (i * 8U));
    high = high | (static_cast<u32>(p_message.payload[i + 4]) << (i * 8U));
  }

  hal::notifier::scope operation(m_transmit_notifier, []() noexcept {
    bit_modify(can1_reg->ier).clear<interrupt_enable::transmit_mailbox_empty>();
  });
  // The time six of the longest frames take, plus 1 ms: the three queued
  // ones, twice over
  using namespace std::chrono_literals;
  auto const baud = m_baud_rate.numerical_value_in(mp_units::si::hertz);
  auto const time_limit =
    std::chrono::nanoseconds(hal::divide<hal::rounding::ceil, hal::u64>(
      { .num = 6ULL * max_frame_bits * 1'000'000'000ULL, .den = baud })) +
    1ms;

  while (true) {
    if (is_bus_off()) {
      throw hal::operation_not_permitted(this);
    }
    auto const empty =
      bit_extract<transmit_status::mailbox_empty>(can1_reg->tsr);
    if (empty != 0) {
      auto& mailbox = can1_reg->transmit_mailbox[std::countr_zero(empty)];
      mailbox.tdtr = bit_value(0U)
                       .insert<frame_length::data_length_code>(
                         std::min<u32>(p_message.length, 8U))
                       .get();
      mailbox.tdlr = low;
      mailbox.tdhr = high;
      // Setting the transmit request last sends the message
      mailbox.tir =
        bit_value(id).set<mailbox_identifier::transmit_request>().get();
      return;
    }
    bit_modify(can1_reg->ier).set<interrupt_enable::transmit_mailbox_empty>();
    if (not operation.block_for(time_limit) and
        (bit_extract<transmit_status::mailbox_empty>(can1_reg->tsr) == 0) and
        not is_bus_off()) {
      throw hal::timed_out(this);
    }
  }
}

void can::set_baud_rate(hal::hertz p_baud_rate)
{
  std::lock_guard const hold(*m_lock);
  auto const loop_back = bit_is_set<bus_timing::loop_back_mode>(can1_reg->btr);
  enter_initialization();
  try {
    m_baud_rate = configure_bit_timing(p_baud_rate, loop_back);
  } catch (...) {
    exit_initialization();
    throw;
  }
  exit_initialization();
}

void can::set_filter_mode(hal::can_message_acceptance p_accept)
{
  std::lock_guard const hold(*m_lock);
  m_acceptance = p_accept;
  apply_bank_activation();
}

void can::recover_bus()
{
  std::lock_guard const hold(*m_lock);
  if (is_bus_off()) {
    // RM0008 section 24.7.6: with automatic bus-off management off, software
    // starts recovery by entering and leaving initialization mode. The
    // controller then rejoins after 128 runs of 11 recessive bits.
    enter_initialization();
    exit_initialization();
  }
}

u8 can::claim_bank()
{
  std::lock_guard const hold(*m_lock);
  for (u8 bank = 0; bank < user_bank_count; bank++) {
    if (not m_claimed_banks[bank]) {
      m_claimed_banks[bank] = true;
      return bank;
    }
  }
  throw hal::device_or_resource_busy(this);
}

void can::release_bank(u8 p_bank) noexcept
{
  m_enabled_banks[p_bank] = false;
  m_claimed_banks[p_bank] = false;
  apply_bank_activation();
}

void can::set_bank_enabled(u8 p_bank, bool p_enabled) noexcept
{
  m_enabled_banks[p_bank] = p_enabled;
  apply_bank_activation();
}

void can::apply_bank_activation() noexcept
{
  u32 active = 0;
  if (m_acceptance == hal::can_message_acceptance::filtered) {
    active = static_cast<u32>(m_enabled_banks.to_ulong());
  } else if (m_acceptance == hal::can_message_acceptance::all) {
    active = 1U << accept_all_bank;
  }
  // No active bank accepts nothing (`none`)
  can1_reg->fa1r = active;
}

// ---- transceiver -----------------------------------------------------------

hal::hertz can::driver_baud_rate()
{
  return m_baud_rate;
}

void can::driver_send(hal::can_message const& p_message)
{
  transmit(p_message);
}

hal::circular_span<hal::can_message const> can::driver_receive_buffer()
{
  return std::span<hal::can_message const>(m_receive_buffer);
}

hal::usize can::driver_receive_count()
{
  auto const count = m_receive_count;
  // Reads of the buffer must happen after the count was read
  std::atomic_signal_fence(std::memory_order_seq_cst);
  return count;
}

// ---- bus manager -----------------------------------------------------------

can::bus_manager::bus_manager(can& p_can)
  : m_can(p_can)
{
}

void can::bus_manager::driver_baud_rate(hal::u32 p_hertz)
{
  m_can->set_baud_rate(p_hertz * mp_units::si::hertz);
}

void can::bus_manager::driver_filter_mode(hal::can_message_acceptance p_accept)
{
  m_can->set_filter_mode(p_accept);
}

void can::bus_manager::driver_on_bus_off(hal::can_bus_off_callback& p_callback)
{
  if (m_can->m_bus_off_callback != nullptr) {
    throw hal::subscription_busy(this);
  }
  m_can->m_bus_off_callback = &p_callback;
}

void can::bus_manager::driver_clear_bus_off() noexcept
{
  m_can->m_bus_off_callback = nullptr;
}

void can::bus_manager::driver_bus_on()
{
  m_can->recover_bus();
}

// ---- interrupt -------------------------------------------------------------

can::interrupt::interrupt(can& p_can)
  : m_can(p_can)
{
}

void can::interrupt::driver_on_receive(hal::can_receive_callback& p_callback,
                                       hal::can_receive_filter p_filter)
{
  if (m_can->m_receive_callback != nullptr) {
    throw hal::subscription_busy(this);
  }
  m_can->m_receive_filter = p_filter;
  // The filter is in place before the interrupt can see the callback
  std::atomic_signal_fence(std::memory_order_seq_cst);
  m_can->m_receive_callback = &p_callback;
}

void can::interrupt::driver_clear_receive() noexcept
{
  m_can->m_receive_callback = nullptr;
}

// ---- filter banks ----------------------------------------------------------

template<typename Allowed, hal::usize Count>
can::filter_bank<Allowed, Count>::filter::filter(filter_bank& p_bank, u8 p_slot)
  : m_bank(&p_bank)
  , m_slot(p_slot)
{
}

template<typename Allowed, hal::usize Count>
void can::filter_bank<Allowed, Count>::filter::driver_allow(
  std::optional<Allowed> p_allowed)
{
  m_bank->allow(m_slot, p_allowed);
}

template<typename Allowed, hal::usize Count>
template<hal::usize... Index>
std::array<typename can::filter_bank<Allowed, Count>::filter, Count>
can::filter_bank<Allowed, Count>::make_filters(std::index_sequence<Index...>)
{
  return { filter(*this, static_cast<u8>(Index))... };
}

template<typename Allowed, hal::usize Count>
can::filter_bank<Allowed, Count>::filter_bank(can& p_can, can_fifo p_fifo)
  : m_can(p_can)
  , m_filters(make_filters(std::make_index_sequence<Count>{}))
  , m_bank(p_can.claim_bank())
{
  using format = filter_format<Allowed>;
  std::lock_guard const hold(*m_can->m_lock);
  // Disabled until a filter allows something
  m_can->set_bank_enabled(m_bank, false);
  configure_bank(m_bank, format::is_32_bit, format::is_list, p_fifo);
}

template<typename Allowed, hal::usize Count>
can::filter_bank<Allowed, Count>::~filter_bank<Allowed, Count>()
{
  std::lock_guard const hold(*m_can->m_lock);
  m_can->release_bank(m_bank);
}

template<typename Allowed, hal::usize Count>
void can::filter_bank<Allowed, Count>::allow(u8 p_slot,
                                             std::optional<Allowed> p_allowed)
{
  constexpr u32 slot_bits = 64 / Count;
  constexpr hal::u64 slot_mask =
    (slot_bits == 64) ? ~hal::u64{ 0 } : ((hal::u64{ 1 } << slot_bits) - 1U);
  std::lock_guard const hold(*m_can->m_lock);

  m_allowing[p_slot] = p_allowed.has_value();
  if (p_allowed) {
    auto const shift = p_slot * slot_bits;
    m_value = (m_value & ~(slot_mask << shift)) |
              (filter_format<Allowed>::encode(*p_allowed) << shift);
  }

  // The bank stays disabled while its registers change
  m_can->set_bank_enabled(m_bank, false);
  if (m_allowing.none()) {
    return;
  }
  // A filter that allows nothing repeats one that does, so it accepts
  // nothing new
  hal::u64 value = m_value;
  u32 allowing_slot = 0;
  while (not m_allowing[allowing_slot]) {
    allowing_slot++;
  }
  auto const copy = (m_value >> (allowing_slot * slot_bits)) & slot_mask;
  for (u32 slot = 0; slot < Count; slot++) {
    if (not m_allowing[slot]) {
      auto const shift = slot * slot_bits;
      value = (value & ~(slot_mask << shift)) | (copy << shift);
    }
  }
  auto& registers = can1_reg->filter_registers[m_bank];
  registers.fr1 = static_cast<u32>(value);
  registers.fr2 = static_cast<u32>(value >> 32U);
  m_can->set_bank_enabled(m_bank, true);
}

template class can::filter_bank<hal::u16, 4>;
template class can::filter_bank<hal::u32, 2>;
template class can::filter_bank<hal::can_mask, 2>;
template class can::filter_bank<hal::can_mask_ext, 1>;
}  // namespace hal::stm32f1
