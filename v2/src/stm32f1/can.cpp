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
#include <bitset>
#include <coroutine>
#include <memory>
#include <optional>

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

/// Register map for the bxCAN peripheral (RM0008 Chapter 24)
struct can_reg_t
{
  u32 volatile mcr;
  u32 volatile msr;
  u32 volatile tsr;
  u32 volatile rf0r;
  u32 volatile rf1r;
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
  // Limited to only 14 on connectivity line devices
  std::array<can_filter_register_t, 28> filter_registers;
};

// NOLINTNEXTLINE(performance-no-int-to-ptr)
auto* can1_reg = reinterpret_cast<can_reg_t*>(0x4000'6400);

/// Bit masks for the BTR register
struct bus_timing
{
  static constexpr auto prescalar = bit_mask::from<0, 9>();
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
  static constexpr auto no_automatic_retransmission = bit_mask::from<4>();
  static constexpr auto automatic_bus_off_management = bit_mask::from<6>();
};

/// Bit masks for the MSR register
struct master_status
{
  static constexpr auto initialization_acknowledge = bit_mask::from<0>();
};

/// Bit masks for the TSR register
struct transmit_status
{
  static constexpr auto transmit_mailbox0_empty = bit_mask::from<26>();
  static constexpr auto transmit_mailbox1_empty = bit_mask::from<27>();
  static constexpr auto transmit_mailbox2_empty = bit_mask::from<28>();
};

/// Bit masks for the IER register
struct interrupt_enable_register
{
  static constexpr auto fifo0_message_pending = bit_mask::from<1>();
  static constexpr auto fifo1_message_pending = bit_mask::from<4>();
  static constexpr auto bus_off = bit_mask::from<10>();
};

/// Bit mask for the ESR register
struct error_status_register
{
  static constexpr auto bus_off = bit_mask::from<2>();
};

/// Bit masks shared by the TIxR/RIxR mailbox identifier registers
struct mailbox_identifier
{
  static constexpr auto transmit_mailbox_request = bit_mask::from<0>();
  static constexpr auto remote_request = bit_mask::from<1>();
  static constexpr auto identifier_type = bit_mask::from<2>();
  static constexpr auto standard_identifier = bit_mask::from<21, 31>();
  static constexpr auto extended_identifier = bit_mask::from<3, 31>();

  static constexpr u32 standard = 0;
  static constexpr u32 extended = 1;
};

/// Bit masks shared by the TDTxR/RDTxR frame-length registers
struct frame_length_and_info
{
  static constexpr auto data_length_code = bit_mask::from<0, 3>();
};

/// Bit masks for the RF0R/RF1R FIFO status registers
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

enum class filter_bank_master_control : u8
{
  active = 0,
  initialization = 1,
};

enum class filter_type : u8
{
  mask = 0,
  list = 1,
};

enum class filter_scale : u8
{
  dual_16_bit_scale = 0,
  single_32_bit_scale = 1,
};

enum class filter_activation : u8
{
  not_active = 0,
  active = 1,
};

/// Bit layout of a single 16-bit sub-bank within CAN_FiRx (dual 16-bit scale)
struct standard_filter_bank
{
  static constexpr auto id = bit_mask::from<5, 15>();
  static constexpr auto rtr = bit_mask::from<4>();
  static constexpr auto id_extension = bit_mask::from<3>();

  static constexpr auto sub_bank1 = bit_mask::from<0, 15>();
  static constexpr auto sub_bank2 = bit_mask::from<16, 31>();
};

/// Bit layout of a 32-bit extended filter bank word (single 32-bit scale)
struct extended_filter_bank
{
  static constexpr auto id = bit_mask::from<3, 31>();
  static constexpr auto id_extension = bit_mask::from<2>();
  static constexpr auto rtr = bit_mask::from<1>();
};

void set_master_mode(bit_mask p_mode, bool p_enable)
{
  bit_modify(can1_reg->mcr).insert(p_mode, p_enable);
}

bool get_master_status(bit_mask p_mode)
{
  return bit_extract(p_mode, can1_reg->msr);
}

void enter_initialization()
{
  set_master_mode(master_control::initialization_request, true);
  while (not get_master_status(master_status::initialization_acknowledge)) {
    continue;
  }
}

void exit_initialization()
{
  set_master_mode(master_control::initialization_request, false);
  while (get_master_status(master_status::initialization_acknowledge)) {
    continue;
  }
}

void configure_baud_rate(u32 p_baud_rate)
{
  auto const can_frequency = frequency(peripheral::can1);
  auto const divider = calculate_can_bus_divider(
    can_frequency,
    static_cast<hal::hertz>(p_baud_rate * mp_units::si::unit_symbols::Hz));

  if (not divider) {
    throw hal::operation_not_supported(nullptr);
  }

  auto const timing = divider.value();
  auto const prescale = timing.clock_divider - 1U;
  auto const sync_jump_width = timing.synchronization_jump_width - 1U;
  auto const phase_segment1 =
    (timing.phase_segment1 + timing.propagation_delay) - 1U;
  auto const phase_segment2 = timing.phase_segment2 - 1U;

  bit_modify(can1_reg->btr)
    .insert<bus_timing::prescalar>(prescale)
    .insert<bus_timing::time_segment1>(phase_segment1)
    .insert<bus_timing::time_segment2>(phase_segment2)
    .insert<bus_timing::sync_jump_width>(sync_jump_width)
    .clear<bus_timing::silent_mode>();
}

void set_filter_bank_mode(filter_bank_master_control p_mode)
{
  bit_modify(can1_reg->fmr)
    .insert<filter_master::initialization_mode>(value(p_mode));
}

void set_filter_type(u8 p_filter, filter_type p_type)
{
  bit_modify(can1_reg->fm1r).insert(bit_mask::from(p_filter), value(p_type));
}

void set_filter_scale(u8 p_filter, filter_scale p_scale)
{
  bit_modify(can1_reg->fs1r).insert(bit_mask::from(p_filter), value(p_scale));
}

void set_filter_fifo_assignment(u8 p_filter, can_fifo p_fifo)
{
  bit_modify(can1_reg->ffa1r).insert(bit_mask::from(p_filter), value(p_fifo));
}

void set_filter_activation_state(u8 p_filter, filter_activation p_state)
{
  bit_modify(can1_reg->fa1r).insert(bit_mask::from(p_filter), value(p_state));
}

u16 standard_id_to_stm_filter(u16 p_id)
{
  return bit_value()
    .insert<standard_filter_bank::id>(p_id)
    .clear(standard_filter_bank::rtr)
    .clear(standard_filter_bank::id_extension)
    .to<u16>();
}

u32 extended_id_to_stm_filter(u32 p_id)
{
  return bit_value()
    .insert<extended_filter_bank::id>(p_id)
    .clear(extended_filter_bank::id_extension)
    .clear(extended_filter_bank::rtr)
    .to<u32>();
}

struct can_data_registers_t
{
  u32 frame = 0;
  u32 id = 0;
  u32 data_a = 0;
  u32 data_b = 0;
};

can_data_registers_t convert_message_to_stm_can(can_message const& p_message)
{
  can_data_registers_t registers;

  registers.frame =
    bit_value(0U)
      .insert<frame_length_and_info::data_length_code>(p_message.length)
      .to<u32>();

  if (p_message.extended) {
    registers.id =
      bit_value(0U)
        .insert<mailbox_identifier::transmit_mailbox_request>(true)
        .insert<mailbox_identifier::remote_request>(p_message.remote_request)
        .insert<mailbox_identifier::identifier_type>(
          mailbox_identifier::extended)
        .insert<mailbox_identifier::extended_identifier>(p_message.id)
        .to<u32>();
  } else {
    registers.id =
      bit_value(0U)
        .insert<mailbox_identifier::transmit_mailbox_request>(true)
        .insert<mailbox_identifier::remote_request>(p_message.remote_request)
        .insert<mailbox_identifier::identifier_type>(
          mailbox_identifier::standard)
        .insert<mailbox_identifier::standard_identifier>(p_message.id)
        .to<u32>();
  }

  u32 data_a = 0;
  data_a |= static_cast<u32>(p_message.payload[0]) << (0 * 8);
  data_a |= static_cast<u32>(p_message.payload[1]) << (1 * 8);
  data_a |= static_cast<u32>(p_message.payload[2]) << (2 * 8);
  data_a |= static_cast<u32>(p_message.payload[3]) << (3 * 8);

  u32 data_b = 0;
  data_b |= static_cast<u32>(p_message.payload[4]) << (0 * 8);
  data_b |= static_cast<u32>(p_message.payload[5]) << (1 * 8);
  data_b |= static_cast<u32>(p_message.payload[6]) << (2 * 8);
  data_b |= static_cast<u32>(p_message.payload[7]) << (3 * 8);

  registers.data_a = data_a;
  registers.data_b = data_b;

  return registers;
}

can_message read_receive_mailbox()
{
  can_message message{};

  auto fifo_select = can_fifo::select1;
  if (bit_extract<fifo_status::messages_pending>(can1_reg->rf0r)) {
    fifo_select = can_fifo::select1;
  } else if (bit_extract<fifo_status::messages_pending>(can1_reg->rf1r)) {
    fifo_select = can_fifo::select2;
  } else {
    // Spurious call: nothing pending on either FIFO.
    return message;
  }

  auto& mailbox = can1_reg->fifo_mailbox[value(fifo_select)];
  auto const frame = mailbox.rdtr;
  auto const id = mailbox.rir;

  message.remote_request = bit_extract<mailbox_identifier::remote_request>(id);
  message.length = static_cast<u8>(
    bit_extract<frame_length_and_info::data_length_code>(frame));
  message.extended = bit_extract<mailbox_identifier::identifier_type>(id) ==
                     mailbox_identifier::extended;

  if (message.extended) {
    message.id = bit_extract<mailbox_identifier::extended_identifier>(id);
  } else {
    message.id = bit_extract<mailbox_identifier::standard_identifier>(id);
  }

  auto const low = mailbox.rdlr;
  auto const high = mailbox.rdhr;

  message.payload[0] = static_cast<hal::byte>(low >> (0 * 8));
  message.payload[1] = static_cast<hal::byte>(low >> (1 * 8));
  message.payload[2] = static_cast<hal::byte>(low >> (2 * 8));
  message.payload[3] = static_cast<hal::byte>(low >> (3 * 8));
  message.payload[4] = static_cast<hal::byte>(high >> (0 * 8));
  message.payload[5] = static_cast<hal::byte>(high >> (1 * 8));
  message.payload[6] = static_cast<hal::byte>(high >> (2 * 8));
  message.payload[7] = static_cast<hal::byte>(high >> (3 * 8));

  if (fifo_select == can_fifo::select1) {
    bit_modify(can1_reg->rf0r).set<fifo_status::release_output_mailbox>();
  } else {
    bit_modify(can1_reg->rf1r).set<fifo_status::release_output_mailbox>();
  }

  return message;
}

bool is_bus_off()
{
  return bit_extract<error_status_register::bus_off>(can1_reg->esr);
}

void recover_from_bus_off()
{
  // RM0008 p.670: bus-off is recovered from by re-entering and then leaving
  // initialization mode.
  enter_initialization();
  exit_initialization();
}

std::span<can_message> allocate_receive_buffer(hal::allocator p_allocator,
                                               hal::usize p_count)
{
  auto* storage = p_allocator.allocate_object<can_message>(p_count);
  std::uninitialized_value_construct_n(storage, p_count);
  return { storage, p_count };
}
}  // namespace

/// Implementation state shared by every resource acquired from a `can`
/// manager. The pimpl pointer doubles as the "CAN1 is in use" singleton
/// guard: only one `can::impl` can exist at a time because `create()` throws
/// if CAN1 is already powered on.
struct can::impl
{
  std::span<can_message> receive_buffer;
  hal::usize receive_cursor = 0;
  std::bitset<28> acquired_banks{};
  async::mutex tx_owner{};
  async::context* bus_off_waiter = nullptr;
};

namespace {
// CAN1 is a process-wide singleton peripheral (there is exactly one bxCAN1
// register bank), so its interrupt handlers reach the active manager through
// a plain static pointer rather than a per-instance callback, matching the
// singleton pattern already used for the DWT and systick peripherals.
can::impl* s_active_can = nullptr;

extern "C" void can1_rx_isr()
{
  auto* state = s_active_can;
  if (state == nullptr) {
    return;
  }

  auto const message = read_receive_mailbox();

  // Guard against a damaged/misbehaving peripheral reporting a length above
  // the 8-byte maximum a CAN frame can hold; discard rather than propagate
  // garbage payload bytes.
  if (message.length > 8) {
    return;
  }

  state->receive_buffer[state->receive_cursor % state->receive_buffer.size()] =
    message;
  state->receive_cursor++;
}

extern "C" void can1_sce_isr()
{
  auto* state = s_active_can;
  if (state == nullptr) {
    return;
  }

  if (is_bus_off() and state->bus_off_waiter != nullptr) {
    state->bus_off_waiter->unblock();
    state->bus_off_waiter = nullptr;
  }
}

class transceiver final : public hal::can_transceiver
{
public:
  explicit transceiver(hal::ptr<can> p_manager)
    : m_manager(p_manager)
  {
  }

  transceiver(transceiver const&) = delete;
  transceiver& operator=(transceiver const&) = delete;
  transceiver(transceiver&&) = delete;
  transceiver& operator=(transceiver&&) = delete;
  ~transceiver() = default;

private:
  async::future<hertz> driver_baud_rate(async::context&) override
  {
    return frequency(peripheral::can1);
  }

  async::future<void> driver_send(async::context& p_context,
                                  can_message const& p_message) override
  {
    return m_manager->send(p_context, p_message);
  }

  circular_span<can_message const> driver_receive_buffer() override
  {
    return m_manager->receive_buffer();
  }

  usize driver_receive_cursor() override
  {
    return m_manager->receive_cursor();
  }

  hal::ptr<can> m_manager;
};

class bus_manager final : public hal::can_bus_manager
{
public:
  explicit bus_manager(hal::ptr<can> p_manager)
    : m_manager(p_manager)
  {
  }

  bus_manager(bus_manager const&) = delete;
  bus_manager& operator=(bus_manager const&) = delete;
  bus_manager(bus_manager&&) = delete;
  bus_manager& operator=(bus_manager&&) = delete;
  ~bus_manager() = default;

private:
  async::future<void> driver_baud_rate(async::context& p_context,
                                       u32 p_hertz) override
  {
    return m_manager->set_baud_rate(p_context, p_hertz);
  }

  async::future<void> driver_filter_mode(async::context&,
                                         can_message_acceptance) override
  {
    // bxCAN has no single register to bypass every filter bank at once; the
    // only accurate way to implement `all`/`none` would be to reserve one of
    // the 28 filter banks as an always-pass/always-block toggle, which would
    // silently shrink the pool available to `acquire_*_filter()`. Until a
    // driver needs that trade-off, this is a deliberate no-op: messages are
    // always filtered per whatever filter banks the caller has acquired,
    // matching this driver's documented default behavior.
    return {};
  }

  async::future<void> driver_on_bus_off(async::context& p_context) override
  {
    return m_manager->wait_for_bus_off(p_context);
  }

  async::future<void> driver_bus_on(async::context& p_context) override
  {
    return m_manager->bus_on(p_context);
  }

  hal::ptr<can> m_manager;
};

/// Claims a free filter bank, puts the filter banks into initialization
/// mode for the duration of `p_configure`, then restores active mode and
/// activates the newly claimed bank.
u8 claim_and_configure_filter_bank(hal::ptr<can> const& p_manager,
                                   filter_scale p_scale,
                                   filter_type p_type,
                                   can_fifo p_fifo,
                                   std::invocable<u8> auto&& p_configure)
{
  auto const index = p_manager->acquire_filter_bank();

  set_filter_bank_mode(filter_bank_master_control::initialization);
  set_filter_activation_state(index, filter_activation::not_active);
  set_filter_scale(index, p_scale);
  set_filter_type(index, p_type);
  set_filter_fifo_assignment(index, p_fifo);

  p_configure(index);

  set_filter_bank_mode(filter_bank_master_control::active);
  set_filter_activation_state(index, filter_activation::active);

  return index;
}

void release_filter_bank(hal::ptr<can> const& p_manager, u8 p_index)
{
  set_filter_activation_state(p_index, filter_activation::not_active);
  p_manager->release_filter_bank(p_index);
}

class id_filter final : public hal::can_id_filter
{
public:
  id_filter(hal::ptr<can> p_manager, u8 p_filter_index, u8 p_word_index)
    : m_manager(p_manager)
    , m_filter_index(p_filter_index)
    , m_word_index(p_word_index)
  {
  }

private:
  async::future<void> driver_allow(async::context&,
                                   std::optional<u16> p_id) override
  {
    auto const id = p_id.value_or(0);
    auto& filter = can1_reg->filter_registers[m_filter_index];
    auto const reg = standard_id_to_stm_filter(id);

    set_filter_activation_state(m_filter_index, filter_activation::not_active);
    switch (m_word_index) {
      case 0:
        bit_modify(filter.fr1).insert<standard_filter_bank::sub_bank1>(reg);
        break;
      case 1:
        bit_modify(filter.fr1).insert<standard_filter_bank::sub_bank2>(reg);
        break;
      case 2:
        bit_modify(filter.fr2).insert<standard_filter_bank::sub_bank1>(reg);
        break;
      default:
        bit_modify(filter.fr2).insert<standard_filter_bank::sub_bank2>(reg);
        break;
    }
    set_filter_activation_state(m_filter_index, filter_activation::active);
    return {};
  }

  hal::ptr<can> m_manager;
  u8 m_filter_index;
  u8 m_word_index;
};

struct id_filter_set
{
  id_filter_set(hal::ptr<can> p_manager, can_fifo p_fifo)
    : manager(p_manager)
    , filters{ { id_filter(p_manager, 0, 0),
                 id_filter(p_manager, 0, 1),
                 id_filter(p_manager, 0, 2),
                 id_filter(p_manager, 0, 3) } }
  {
    auto const disable_reg = standard_id_to_stm_filter(0);
    auto const disable_word =
      (static_cast<u32>(disable_reg) << 16) | disable_reg;

    index = claim_and_configure_filter_bank(
      p_manager,
      filter_scale::dual_16_bit_scale,
      filter_type::list,
      p_fifo,
      [&](u8 p_index) {
        can1_reg->filter_registers[p_index].fr1 = disable_word;
        can1_reg->filter_registers[p_index].fr2 = disable_word;
        for (auto& f : filters) {
          f = id_filter(p_manager, p_index, &f - &filters[0]);
        }
      });
  }

  ~id_filter_set()
  {
    release_filter_bank(manager, index);
  }

  hal::ptr<can> manager;
  std::array<id_filter, 4> filters;
  u8 index = 0;
};

class id_ext_filter final : public hal::can_id_ext_filter
{
public:
  id_ext_filter(hal::ptr<can> p_manager, u8 p_filter_index, bool p_second_word)
    : m_manager(p_manager)
    , m_filter_index(p_filter_index)
    , m_second_word(p_second_word)
  {
  }

private:
  async::future<void> driver_allow(async::context&,
                                   std::optional<u32> p_id) override
  {
    auto const id = p_id.value_or(0);
    auto const reg = extended_id_to_stm_filter(id);
    auto& filter = can1_reg->filter_registers[m_filter_index];

    set_filter_activation_state(m_filter_index, filter_activation::not_active);
    if (m_second_word) {
      filter.fr2 = reg;
    } else {
      filter.fr1 = reg;
    }
    set_filter_activation_state(m_filter_index, filter_activation::active);
    return {};
  }

  hal::ptr<can> m_manager;
  u8 m_filter_index;
  bool m_second_word;
};

struct id_ext_filter_set
{
  id_ext_filter_set(hal::ptr<can> p_manager, can_fifo p_fifo)
    : manager(p_manager)
    , filters{ { id_ext_filter(p_manager, 0, false),
                 id_ext_filter(p_manager, 0, true) } }
  {
    auto const disable_reg = extended_id_to_stm_filter(0);

    index = claim_and_configure_filter_bank(
      p_manager,
      filter_scale::single_32_bit_scale,
      filter_type::list,
      p_fifo,
      [&](u8 p_index) {
        can1_reg->filter_registers[p_index].fr1 = disable_reg;
        can1_reg->filter_registers[p_index].fr2 = disable_reg;
        filters[0] = id_ext_filter(p_manager, p_index, false);
        filters[1] = id_ext_filter(p_manager, p_index, true);
      });
  }

  ~id_ext_filter_set()
  {
    release_filter_bank(manager, index);
  }

  hal::ptr<can> manager;
  std::array<id_ext_filter, 2> filters;
  u8 index = 0;
};

class mask_filter final : public hal::can_mask_filter
{
public:
  mask_filter(hal::ptr<can> p_manager, u8 p_filter_index, bool p_second_word)
    : m_manager(p_manager)
    , m_filter_index(p_filter_index)
    , m_second_word(p_second_word)
  {
  }

private:
  async::future<void> driver_allow(async::context&,
                                   std::optional<can_mask> p_mask) override
  {
    auto const selected = p_mask.value_or(can_mask{ .id = 0, .mask = 0x1FF });
    auto const id_reg = standard_id_to_stm_filter(selected.id);
    auto const mask_reg = standard_id_to_stm_filter(selected.mask);
    auto& filter = can1_reg->filter_registers[m_filter_index];

    set_filter_activation_state(m_filter_index, filter_activation::not_active);
    if (m_second_word) {
      bit_modify(filter.fr2)
        .insert<standard_filter_bank::sub_bank1>(id_reg)
        .insert<standard_filter_bank::sub_bank2>(mask_reg);
    } else {
      bit_modify(filter.fr1)
        .insert<standard_filter_bank::sub_bank1>(id_reg)
        .insert<standard_filter_bank::sub_bank2>(mask_reg);
    }
    set_filter_activation_state(m_filter_index, filter_activation::active);
    return {};
  }

  hal::ptr<can> m_manager;
  u8 m_filter_index;
  bool m_second_word;
};

struct mask_filter_set
{
  mask_filter_set(hal::ptr<can> p_manager, can_fifo p_fifo)
    : manager(p_manager)
    , filters{ { mask_filter(p_manager, 0, false),
                 mask_filter(p_manager, 0, true) } }
  {
    auto const disable_id = standard_id_to_stm_filter(0);
    auto const disable_mask = standard_id_to_stm_filter(0x1FF);
    auto const disable_word =
      (static_cast<u32>(disable_mask) << 16) | disable_id;

    index = claim_and_configure_filter_bank(
      p_manager,
      filter_scale::dual_16_bit_scale,
      filter_type::mask,
      p_fifo,
      [&](u8 p_index) {
        can1_reg->filter_registers[p_index].fr1 = disable_word;
        can1_reg->filter_registers[p_index].fr2 = disable_word;
        filters[0] = mask_filter(p_manager, p_index, false);
        filters[1] = mask_filter(p_manager, p_index, true);
      });
  }

  ~mask_filter_set()
  {
    release_filter_bank(manager, index);
  }

  hal::ptr<can> manager;
  std::array<mask_filter, 2> filters;
  u8 index = 0;
};

class mask_ext_filter final : public hal::can_mask_ext_filter
{
public:
  mask_ext_filter(hal::ptr<can> p_manager, can_fifo p_fifo)
    : m_manager(p_manager)
  {
    auto const disable_id = extended_id_to_stm_filter(0);
    auto const disable_mask = extended_id_to_stm_filter(0x1FFF'FFFF);

    m_filter_index = claim_and_configure_filter_bank(
      p_manager,
      filter_scale::single_32_bit_scale,
      filter_type::mask,
      p_fifo,
      [&](u8 p_index) {
        can1_reg->filter_registers[p_index].fr1 = disable_id;
        can1_reg->filter_registers[p_index].fr2 = disable_mask;
      });
  }

  mask_ext_filter(mask_ext_filter const&) = delete;
  mask_ext_filter& operator=(mask_ext_filter const&) = delete;
  mask_ext_filter(mask_ext_filter&&) = delete;
  mask_ext_filter& operator=(mask_ext_filter&&) = delete;

  ~mask_ext_filter()
  {
    release_filter_bank(m_manager, m_filter_index);
  }

private:
  async::future<void> driver_allow(async::context&,
                                   std::optional<can_mask_ext> p_mask) override
  {
    auto const selected =
      p_mask.value_or(can_mask_ext{ .id = 0, .mask = 0x1FFF'FFFF });
    auto const id_reg = extended_id_to_stm_filter(selected.id);
    auto const mask_reg = extended_id_to_stm_filter(selected.mask);
    auto& filter = can1_reg->filter_registers[m_filter_index];

    set_filter_activation_state(m_filter_index, filter_activation::not_active);
    filter.fr1 = id_reg;
    filter.fr2 = mask_reg;
    set_filter_activation_state(m_filter_index, filter_activation::active);
    return {};
  }

  hal::ptr<can> m_manager;
  u8 m_filter_index = 0;
};
}  // namespace

hal::ptr<can> can::create(hal::allocator p_allocator,
                          u32 p_baud_rate,
                          can_settings const& p_settings)
{
  if (is_on(peripheral::can1)) {
    throw hal::device_or_resource_busy(nullptr);
  }

  return hal::allocate<can>(
    p_allocator, private_key{}, p_allocator, p_baud_rate, p_settings);
}

can::can(private_key,
         hal::allocator p_allocator,
         u32 p_baud_rate,
         can_settings const& p_settings)
  : pimpl(
      p_allocator,
      impl{ .receive_buffer =
              allocate_receive_buffer(p_allocator, p_settings.message_count) })
{
  power_on(peripheral::can1);

  set_master_mode(master_control::sleep_mode_request, false);
  set_master_mode(master_control::no_automatic_retransmission, false);
  set_master_mode(master_control::automatic_bus_off_management, false);

  enter_initialization();

  configure_baud_rate(p_baud_rate);

  switch (p_settings.pins) {
    case can_pins::pa11_pa12:
      configure_pin({ .port = 'A', .pin = 11 }, input_pull_up);
      configure_pin({ .port = 'A', .pin = 12 }, push_pull_alternative_output);
      break;
    case can_pins::pb9_pb8:
      configure_pin({ .port = 'B', .pin = 8 }, input_pull_up);
      configure_pin({ .port = 'B', .pin = 9 }, push_pull_alternative_output);
      break;
    case can_pins::pd0_pd1:
      configure_pin({ .port = 'D', .pin = 0 }, input_pull_up);
      configure_pin({ .port = 'D', .pin = 1 }, push_pull_alternative_output);
      break;
  }

  bit_modify(can1_reg->btr)
    .insert<bus_timing::loop_back_mode>(p_settings.enable_self_test);

  remap_pins(p_settings.pins);

  exit_initialization();

  s_active_can = &inner();

  hal::cortex_m::initialize_interrupts<irq::max>();
  hal::cortex_m::enable_interrupt(irq::can1_rx0, can1_rx_isr);
  hal::cortex_m::enable_interrupt(irq::can1_rx1, can1_rx_isr);
  hal::cortex_m::enable_interrupt(irq::can1_sce, can1_sce_isr);

  bit_modify(can1_reg->ier)
    .set<interrupt_enable_register::fifo0_message_pending>()
    .set<interrupt_enable_register::fifo1_message_pending>()
    .set<interrupt_enable_register::bus_off>();
}

can::~can()
{
  hal::cortex_m::disable_interrupt(irq::can1_rx0);
  hal::cortex_m::disable_interrupt(irq::can1_rx1);
  hal::cortex_m::disable_interrupt(irq::can1_sce);
  s_active_can = nullptr;
  power_off(peripheral::can1);
}

// TODO(#212): TX mailbox availability is polled with a bounded retry loop
// rather than suspending on the transmit-mailbox-empty interrupt. Convert to
// the completion-interrupt async pattern (mirroring #211 for SPI) before
// relying on this under sustained bus load.
async::future<void> can::send(async::context&,
                              hal::can_message const& p_message)
{
  if (is_bus_off()) {
    throw hal::operation_not_permitted(this);
  }

  auto const registers = convert_message_to_stm_can(p_message);
  std::optional<u8> available_mailbox{};
  i8 retries_remaining = 10;

  while (not available_mailbox) {
    if (retries_remaining <= 0) {
      throw hal::resource_unavailable_try_again(this);
    }

    auto const status = can1_reg->tsr;
    if (bit_extract<transmit_status::transmit_mailbox0_empty>(status)) {
      available_mailbox = 0;
    } else if (bit_extract<transmit_status::transmit_mailbox1_empty>(status)) {
      available_mailbox = 1;
    } else if (bit_extract<transmit_status::transmit_mailbox2_empty>(status)) {
      available_mailbox = 2;
    }

    retries_remaining--;
  }

  auto& mailbox = can1_reg->transmit_mailbox[available_mailbox.value()];
  bit_modify(mailbox.tdtr)
    .insert<frame_length_and_info::data_length_code>(p_message.length);
  mailbox.tdlr = registers.data_a;
  mailbox.tdhr = registers.data_b;
  mailbox.tir = registers.id;

  return {};
}

hal::circular_span<hal::can_message const> can::receive_buffer()
{
  return inner().receive_buffer;
}

hal::usize can::receive_cursor()
{
  return inner().receive_cursor;
}

async::future<void> can::set_baud_rate(async::context&, hal::u32 p_hertz)
{
  enter_initialization();
  configure_baud_rate(p_hertz);
  exit_initialization();
  return {};
}

async::future<void> can::wait_for_bus_off(async::context& p_context)
{
  if (is_bus_off()) {
    co_return;
  }

  inner().bus_off_waiter = &p_context;
  co_await p_context.block_by_signal();
  co_return;
}

async::future<void> can::bus_on(async::context&)
{
  if (is_bus_off()) {
    recover_from_bus_off();
  }
  return {};
}

hal::u8 can::acquire_filter_bank()
{
  auto& state = inner();
  for (hal::usize index = 0; index < state.acquired_banks.size(); index++) {
    if (not state.acquired_banks.test(index)) {
      state.acquired_banks.set(index);
      return static_cast<u8>(index);
    }
  }
  throw hal::resource_unavailable_try_again(this);
}

void can::release_filter_bank(hal::u8 p_index)
{
  inner().acquired_banks.reset(p_index);
}

hal::ptr<hal::can_transceiver> can::acquire_transceiver()
{
  return hal::allocate<transceiver>(memory_resource(), strong_from_this());
}

hal::ptr<hal::can_bus_manager> can::acquire_bus_manager()
{
  return hal::allocate<bus_manager>(memory_resource(), strong_from_this());
}

std::array<hal::ptr<hal::can_id_filter>, 4> can::acquire_identifier_filter(
  can_fifo p_fifo)
{
  auto set =
    hal::allocate<id_filter_set>(memory_resource(), strong_from_this(), p_fifo);
  return {
    hal::ptr<hal::can_id_filter>(set, &id_filter_set::filters, 0),
    hal::ptr<hal::can_id_filter>(set, &id_filter_set::filters, 1),
    hal::ptr<hal::can_id_filter>(set, &id_filter_set::filters, 2),
    hal::ptr<hal::can_id_filter>(set, &id_filter_set::filters, 3),
  };
}

std::array<hal::ptr<hal::can_id_ext_filter>, 2>
can::acquire_extended_identifier_filter(can_fifo p_fifo)
{
  auto set = hal::allocate<id_ext_filter_set>(
    memory_resource(), strong_from_this(), p_fifo);
  return {
    hal::ptr<hal::can_id_ext_filter>(set, &id_ext_filter_set::filters, 0),
    hal::ptr<hal::can_id_ext_filter>(set, &id_ext_filter_set::filters, 1),
  };
}

std::array<hal::ptr<hal::can_mask_filter>, 2> can::acquire_mask_filter(
  can_fifo p_fifo)
{
  auto set = hal::allocate<mask_filter_set>(
    memory_resource(), strong_from_this(), p_fifo);
  return {
    hal::ptr<hal::can_mask_filter>(set, &mask_filter_set::filters, 0),
    hal::ptr<hal::can_mask_filter>(set, &mask_filter_set::filters, 1),
  };
}

hal::ptr<hal::can_mask_ext_filter> can::acquire_extended_mask_filter(
  can_fifo p_fifo)
{
  return hal::allocate<mask_ext_filter>(
    memory_resource(), strong_from_this(), p_fifo);
}
}  // namespace hal::stm32f1
