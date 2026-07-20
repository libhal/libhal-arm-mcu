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

#include <libhal-arm-mcu/lpc40/can2.hpp>

#include <libhal-arm-mcu/interrupt.hpp>
#include <libhal-arm-mcu/lpc40/clock.hpp>
#include <libhal-arm-mcu/lpc40/constants.hpp>
#include <libhal-arm-mcu/lpc40/interrupt.hpp>
#include <libhal-arm-mcu/lpc40/pin.hpp>
#include <libhal-arm-mcu/lpc40/power.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/bit_limits.hpp>
#include <libhal-util/can.hpp>
#include <libhal-util/enum.hpp>
#include <libhal-util/static_callable.hpp>
#include <libhal/error.hpp>
#include <libhal/pointers.hpp>

#include "can_reg.hpp"

namespace hal::lpc40 {
namespace {

can_reg_t* get_can_reg(peripheral p_id)
{
  switch (p_id) {
    case peripheral::can1:
      return can_reg1;
    case peripheral::can2:
    default:
      return can_reg2;
  }
}

void enable_acceptance_filter() noexcept
{
  can_acceptance_filter->acceptance_filter =
    value(can_commands::accept_all_messages);
}

can_message receive(can_reg_t* p_reg) noexcept
{
  static constexpr auto id_mask = bit_mask::from<0, 28>();
  can_message message{};

  // Extract all of the information from the message frame
  auto const frame = p_reg->rfs;
  auto const remote_request = bit_extract<can_frame_info::remote_request>(frame);
  auto const length = bit_extract<can_frame_info::length>(frame);
  auto const format = bit_extract<can_frame_info::format>(frame);

  message.remote_request = remote_request;
  message.length = static_cast<u8>(length);
  message.extended = static_cast<bool>(format);

  // Get the frame ID
  message.id = bit_extract<id_mask>(p_reg->rid);

  // Pull the bytes from RDA into the payload array
  message.payload[0] = (p_reg->rda >> (0 * 8)) & 0xFF;
  message.payload[1] = (p_reg->rda >> (1 * 8)) & 0xFF;
  message.payload[2] = (p_reg->rda >> (2 * 8)) & 0xFF;
  message.payload[3] = (p_reg->rda >> (3 * 8)) & 0xFF;

  // Pull the bytes from RDB into the payload array
  message.payload[4] = (p_reg->rdb >> (0 * 8)) & 0xFF;
  message.payload[5] = (p_reg->rdb >> (1 * 8)) & 0xFF;
  message.payload[6] = (p_reg->rdb >> (2 * 8)) & 0xFF;
  message.payload[7] = (p_reg->rdb >> (3 * 8)) & 0xFF;

  // Release the RX buffer and allow another buffer to be read.
  p_reg->cmr = value(can_commands::release_rx_buffer);

  return message;
}

/// Convert message into the LPC40xx can bus registers.
can_lpc_message message_to_registers(can_message const& p_message)
{
  can_lpc_message registers;

  auto const message_frame_info =
    bit_value<std::uint32_t>(0)
      .insert<can_frame_info::length>(p_message.length)
      .insert<can_frame_info::remote_request>(p_message.remote_request)
      .insert<can_frame_info::format>(static_cast<u32>(p_message.extended))
      .to<std::uint32_t>();

  u32 data_a = 0;
  data_a |= static_cast<u32>(p_message.payload[0] << (0UL * 8));
  data_a |= static_cast<u32>(p_message.payload[1] << (1UL * 8));
  data_a |= static_cast<u32>(p_message.payload[2] << (2UL * 8));
  data_a |= static_cast<u32>(p_message.payload[3] << (3UL * 8));

  u32 data_b = 0;
  data_b |= static_cast<u32>(p_message.payload[4] << (0UL * 8));
  data_b |= static_cast<u32>(p_message.payload[5] << (1UL * 8));
  data_b |= static_cast<u32>(p_message.payload[6] << (2UL * 8));
  data_b |= static_cast<u32>(p_message.payload[7] << (3UL * 8));

  registers.frame = message_frame_info;
  registers.id = p_message.id;
  registers.data_a = data_a;
  registers.data_b = data_b;

  return registers;
}

bool is_bus_off(can_reg_t* p_reg) noexcept
{
  return bit_extract<can_buffer_status::bus_status>(p_reg->sr) ==
         can_buffer_status::bus_off;
}
}  // namespace

void can_peripheral_manager_v2::configure_baud_rate(hal::u32 p_baud_rate)
{
  using namespace hal::literals;

  auto* reg = get_can_reg(m_port.id);
  auto const can_frequency = get_frequency(m_port.id);
  auto const target_baud_rate = static_cast<hertz>(p_baud_rate);

  bool const external_oscillator_used = using_external_oscillator();
  bool const baud_rate_above_100khz = target_baud_rate > 100.0_kHz;
  auto const valid_divider =
    hal::calculate_can_bus_divider(can_frequency, target_baud_rate);

  if ((baud_rate_above_100khz && not external_oscillator_used) ||
      not valid_divider) {
    hal::safe_throw(hal::operation_not_supported(this));
  }

  auto const dividers = valid_divider.value();
  auto const prescale = dividers.clock_divider - 1U;
  auto const sync_jump_width = dividers.synchronization_jump_width - 1U;

  auto phase_segment1 =
    (dividers.phase_segment1 + dividers.propagation_delay) - 1U;
  auto phase_segment2 = dividers.phase_segment2 - 1U;

  constexpr auto segment2_bit_limit =
    hal::bit_limits<can_bus_timing::time_segment2.width, std::uint32_t>::max();

  // Check if phase segment 2 does not fit
  if (phase_segment2 > segment2_bit_limit) {
    // Take the extra time quanta and add it to the phase 1 segment
    auto const phase_segment2_remainder = phase_segment2 - segment2_bit_limit;
    phase_segment1 += phase_segment2_remainder;
    // Cap phase segment 2 to the max available in the bit field
    phase_segment2 = segment2_bit_limit;
  }

  std::uint8_t enable_triple_sampling = 0;
  // The bus is sampled 3 times (recommended for low speeds, 100kHz is
  // considered HIGH).
  if (target_baud_rate < 100.0_kHz) {
    enable_triple_sampling = 1U;
  }

  bit_modify(reg->btr)
    .insert<can_bus_timing::sync_jump_width>(sync_jump_width)
    .insert<can_bus_timing::time_segment1>(phase_segment1)
    .insert<can_bus_timing::time_segment2>(phase_segment2)
    .insert<can_bus_timing::prescalar>(prescale)
    .insert<can_bus_timing::sampling>(enable_triple_sampling);

  m_current_baud_rate = p_baud_rate;
}

void can_peripheral_manager_v2::setup(port const& p_port, hal::u32 p_baud_rate)
{
  auto* reg = get_can_reg(p_port.id);

  initialize_interrupts();

  // Power on CAN BUS peripheral
  power_on(p_port.id);

  // Configure pins
  p_port.td.function(p_port.td_function_code);
  p_port.rd.function(p_port.rd_function_code);

  // Enable reset mode in order to write to CAN registers.
  bit_modify(reg->mod).set<can_mode::reset>();

  configure_baud_rate(p_baud_rate);
  enable_acceptance_filter();

  // Disable reset mode, enabling the device
  bit_modify(reg->mod).clear<can_mode::reset>();
}

void can_peripheral_manager_v2::install_interrupt()
{
  auto* reg = get_can_reg(m_port.id);

  // NOTE: CAN1 and CAN2 share the same NVIC interrupt request line on the
  // LPC40xx. This means only a single can_peripheral_manager_v2 instance's
  // interrupt handler will be active for both peripherals at any given time,
  // since static_callable's storage is shared per template instantiation.
  hal::static_callable<can_peripheral_manager_v2, 0, void(void)> isr(
    [this]() {
      auto* isr_reg = get_can_reg(m_port.id);
      // Reading the ICR clears the pending interrupt condition bits it
      // reports. This is required here since more than one interrupt source
      // (receive and error/bus-off) is enabled on the same IRQ line.
      auto const interrupt_status = isr_reg->icr;

      if (bit_extract<can_interrupts::received_message>(interrupt_status)) {
        auto const message = receive(isr_reg);

        if (m_receive_handler) {
          using tag = hal::can_interrupt::on_receive_tag;
          (*m_receive_handler)(tag{}, message);
        }
        m_message_count++;
        m_buffer.push(message);
      }

      if (bit_extract<can_interrupts::error_warning>(interrupt_status) &&
          is_bus_off(isr_reg) && m_bus_off_handler) {
        (*m_bus_off_handler)(hal::can_bus_manager::bus_off_tag{});
      }
    });

  cortex_m::enable_interrupt(m_port.irq_number, isr.get_handler());

  bit_modify(reg->ier)
    .set<can_interrupts::received_message>()
    .set<can_interrupts::error_warning>();
}

can_peripheral_manager_v2::can_peripheral_manager_v2(
  hal::usize p_message_count,
  std::pmr::polymorphic_allocator<> p_allocator,
  hal::u32 p_baud_rate,
  std::uint8_t p_can_port_number)
  : m_buffer(p_allocator, p_message_count)
{
  if (p_can_port_number == 1) {
    m_port = port{
      .td = pin(0, 1),
      .td_function_code = 1,
      .rd = pin(0, 0),
      .rd_function_code = 1,
      .id = peripheral::can1,
      .irq_number = irq::can,
    };
  } else if (p_can_port_number == 2) {
    m_port = port{
      .td = pin(2, 8),
      .td_function_code = 1,
      .rd = pin(2, 7),
      .rd_function_code = 1,
      .id = peripheral::can2,
      .irq_number = irq::can,
    };
  } else {
    hal::safe_throw(hal::operation_not_supported(this));
  }

  setup(m_port, p_baud_rate);
  install_interrupt();
}

can_peripheral_manager_v2::can_peripheral_manager_v2(
  hal::usize p_message_count,
  std::pmr::polymorphic_allocator<> p_allocator,
  hal::u32 p_baud_rate,
  port const& p_port)
  : m_port(p_port)
  , m_buffer(p_allocator, p_message_count)
{
  setup(m_port, p_baud_rate);
  install_interrupt();
}

can_peripheral_manager_v2::~can_peripheral_manager_v2()
{
  auto* reg = get_can_reg(m_port.id);
  // Disable generating an interrupt request by this CAN peripheral, but leave
  // the interrupt enabled. We must NOT disable the interrupt via Arm's NVIC
  // as it could be used by the other CAN peripheral.
  bit_modify(reg->ier)
    .clear<can_interrupts::received_message>()
    .clear<can_interrupts::error_warning>();
}

void can_peripheral_manager_v2::baud_rate(hal::u32 p_hertz)
{
  auto* reg = get_can_reg(m_port.id);

  bit_modify(reg->mod).set<can_mode::reset>();
  configure_baud_rate(p_hertz);
  bit_modify(reg->mod).clear<can_mode::reset>();
}

hal::u32 can_peripheral_manager_v2::baud_rate() const
{
  return m_current_baud_rate;
}

void can_peripheral_manager_v2::send(can_message const& p_message)
{
  auto* reg = get_can_reg(m_port.id);
  auto const registers = message_to_registers(p_message);

  // Wait for one of the buffers to be free so we can transmit a message
  // through it.
  bool sent = false;
  while (not sent) {
    auto const status_register = reg->sr;
    // Check if any buffer is available.
    if (bit_extract<can_buffer_status::bus_status>(status_register) ==
        can_buffer_status::bus_off) {
      hal::safe_throw(hal::operation_not_permitted(this));
    } else if (bit_extract<can_buffer_status::tx1_released>(status_register)) {
      reg->tfi1 = registers.frame;
      reg->tid1 = registers.id;
      reg->tda1 = registers.data_a;
      reg->tdb1 = registers.data_b;
      reg->cmr = value(can_commands::send_tx_buffer1);
      sent = true;
    } else if (bit_extract<can_buffer_status::tx2_released>(status_register)) {
      reg->tfi2 = registers.frame;
      reg->tid2 = registers.id;
      reg->tda2 = registers.data_a;
      reg->tdb2 = registers.data_b;
      reg->cmr = value(can_commands::send_tx_buffer2);
      sent = true;
    } else if (bit_extract<can_buffer_status::tx3_released>(status_register)) {
      reg->tfi3 = registers.frame;
      reg->tid3 = registers.id;
      reg->tda3 = registers.data_a;
      reg->tdb3 = registers.data_b;
      reg->cmr = value(can_commands::send_tx_buffer3);
      sent = true;
    }
  }
}

void can_peripheral_manager_v2::on_receive(
  hal::can_interrupt::optional_receive_handler const& p_callback)
{
  m_receive_handler = p_callback;
}

void can_peripheral_manager_v2::on_bus_off(
  hal::can_bus_manager::optional_bus_off_handler p_callback)
{
  m_bus_off_handler = p_callback;
}

void can_peripheral_manager_v2::bus_on()
{
  auto* reg = get_can_reg(m_port.id);
  // When the device is in "bus-off" mode, the mode::reset bit is set to '1'.
  // To re-enable the device, clear the reset bit.
  bit_modify(reg->mod).clear<can_mode::reset>();
}

hal::u8 can_peripheral_manager_v2::available_filter()
{
  for (std::size_t i = 0; i < m_acquired_banks.size(); i++) {
    if (not m_acquired_banks.test(i)) {
      m_acquired_banks.set(i);
      // NOTE: This is only safe because m_acquired_banks bitset size is less
      // than 256.
      return static_cast<hal::u8>(i);
    }
  }

  hal::safe_throw(hal::resource_unavailable_try_again(this));
}

void can_peripheral_manager_v2::release_filter(hal::u8 p_filter_bank)
{
  m_acquired_banks.reset(p_filter_bank);
}

/**
 * @brief Acquire an `hal::can_transceiver` implementation
 *
 * @return transceiver - object implementing the `hal::can_transceiver`
 * interface for this can peripheral.
 */
hal::v5::strong_ptr<hal::can_transceiver> acquire_can_transceiver(
  std::pmr::polymorphic_allocator<> p_allocator,
  hal::v5::strong_ptr<can_peripheral_manager_v2> const& p_manager)
{
  struct transceiver : public hal::can_transceiver
  {
  public:
    explicit transceiver(
      hal::v5::strong_ptr<can_peripheral_manager_v2> const& p_manager)
      : m_manager(p_manager)
    {
    }

    transceiver(transceiver const&) = delete;
    transceiver& operator=(transceiver const&) = delete;
    transceiver(transceiver&&) = delete;
    transceiver& operator=(transceiver&&) = delete;
    ~transceiver() override = default;

  private:
    u32 driver_baud_rate() override
    {
      return m_manager->baud_rate();
    }

    void driver_send(can_message const& p_message) override
    {
      m_manager->send(p_message);
    }

    std::span<can_message const> driver_receive_buffer() override
    {
      return m_manager->receive_buffer();
    }

    std::size_t driver_receive_cursor() override
    {
      return m_manager->receive_cursor();
    }

    std::optional<std::size_t> driver_receive_count() override
    {
      return m_manager->receive_count();
    }

    hal::v5::strong_ptr<can_peripheral_manager_v2> m_manager;
  };

  return hal::v5::make_strong_ptr<transceiver>(p_allocator, p_manager);
}

/**
 * @brief Acquire an `hal::can_bus_manager` implementation
 *
 * @return bus_manager - object implementing the `hal::can_bus_manager`
 * interface for this can peripheral.
 */
hal::v5::strong_ptr<hal::can_bus_manager> acquire_can_bus_manager(
  std::pmr::polymorphic_allocator<> p_allocator,
  hal::v5::strong_ptr<can_peripheral_manager_v2> const& p_manager)
{
  class bus_manager : public hal::can_bus_manager
  {
  public:
    explicit bus_manager(
      hal::v5::strong_ptr<can_peripheral_manager_v2> const& p_manager)
      : m_manager(p_manager)
    {
    }

    bus_manager(bus_manager const&) = delete;
    bus_manager& operator=(bus_manager const&) = delete;
    bus_manager(bus_manager&&) = delete;
    bus_manager& operator=(bus_manager&&) = delete;
    ~bus_manager() override
    {
      m_manager->on_bus_off(std::nullopt);
    }

  private:
    void driver_baud_rate(hal::u32 p_hertz) override
    {
      m_manager->baud_rate(p_hertz);
    }

    void driver_filter_mode(accept) override
    {
      // This does nothing. The LPC40xx acceptance filter is always
      // configured by this driver to accept every message on the bus, and
      // hardware ID filtering is not implemented.
    }

    void driver_on_bus_off(optional_bus_off_handler p_callback) override
    {
      m_manager->on_bus_off(p_callback);
    }

    void driver_bus_on() override
    {
      m_manager->bus_on();
    }

    hal::v5::strong_ptr<can_peripheral_manager_v2> m_manager;
  };

  return hal::v5::make_strong_ptr<bus_manager>(p_allocator, p_manager);
}

/**
 * @brief Acquire an `hal::can_interrupt` implementation
 *
 * @return interrupt - object implementing the `hal::can_interrupt` interface
 * for this can peripheral.
 */
hal::v5::strong_ptr<hal::can_interrupt> acquire_can_interrupt(
  std::pmr::polymorphic_allocator<> p_allocator,
  hal::v5::strong_ptr<can_peripheral_manager_v2> const& p_manager)
{
  class interrupt : public hal::can_interrupt
  {
  public:
    explicit interrupt(
      hal::v5::strong_ptr<can_peripheral_manager_v2> const& p_manager)
      : m_manager(p_manager)
    {
    }
    interrupt(interrupt const&) = delete;
    interrupt& operator=(interrupt const&) = delete;
    interrupt(interrupt&&) = delete;
    interrupt& operator=(interrupt&&) = delete;
    ~interrupt() override
    {
      m_manager->on_receive(std::nullopt);
    }

  private:
    void driver_on_receive(optional_receive_handler p_callback) override
    {
      m_manager->on_receive(p_callback);
    }
    hal::v5::strong_ptr<can_peripheral_manager_v2> m_manager;
  };

  return hal::v5::make_strong_ptr<interrupt>(p_allocator, p_manager);
}

/**
 * @brief Acquire a set of 4x standard identifier filters
 *
 * NOTE: The LPC40xx driver's acceptance filter is fixed to accept all
 * messages, so `driver_allow()` is a no-op here.
 *
 * @return identifier_filter_set - A set of 4x identifier filters. When
 * destroyed, releases the filter slot it held on to.
 */
std::array<hal::v5::strong_ptr<hal::can_identifier_filter>, 4>
acquire_can_identifier_filter(
  std::pmr::polymorphic_allocator<> p_allocator,
  hal::v5::strong_ptr<can_peripheral_manager_v2> const& p_manager)
{
  struct identifier_filter : public hal::can_identifier_filter
  {
    void driver_allow(std::optional<u16>) override
    {
      // No-op: hardware ID filtering is not implemented for this platform.
    }
  };

  struct filter_set
  {
    explicit filter_set(
      hal::v5::strong_ptr<can_peripheral_manager_v2> const& p_manager)
      : m_manager(p_manager)
      , m_filter_slot(p_manager->available_filter())
    {
    }

    ~filter_set()
    {
      m_manager->release_filter(m_filter_slot);
    }

    hal::v5::strong_ptr<can_peripheral_manager_v2> m_manager;
    hal::u8 m_filter_slot;
    std::array<identifier_filter, 4> filters{};
  };

  auto set = hal::v5::make_strong_ptr<filter_set>(p_allocator, p_manager);

  return {
    hal::v5::strong_ptr<hal::can_identifier_filter>(
      set, &filter_set::filters, 0),
    hal::v5::strong_ptr<hal::can_identifier_filter>(
      set, &filter_set::filters, 1),
    hal::v5::strong_ptr<hal::can_identifier_filter>(
      set, &filter_set::filters, 2),
    hal::v5::strong_ptr<hal::can_identifier_filter>(
      set, &filter_set::filters, 3),
  };
}

/**
 * @brief Acquire a pair of two extended identifier filters
 *
 * NOTE: See `acquire_can_identifier_filter()` for why `driver_allow()` is a
 * no-op here.
 *
 * @return extended_identifier_filter_set - A set of 2x extended identifier
 * filters.
 */
std::array<hal::v5::strong_ptr<hal::can_extended_identifier_filter>, 2>
acquire_can_extended_identifier_filter(
  std::pmr::polymorphic_allocator<> p_allocator,
  hal::v5::strong_ptr<can_peripheral_manager_v2> const& p_manager)
{
  struct extended_id_filter : public hal::can_extended_identifier_filter
  {
    void driver_allow(std::optional<u32>) override
    {
      // No-op: hardware ID filtering is not implemented for this platform.
    }
  };

  struct filter_set
  {
    explicit filter_set(
      hal::v5::strong_ptr<can_peripheral_manager_v2> const& p_manager)
      : m_manager(p_manager)
      , m_filter_slot(p_manager->available_filter())
    {
    }

    ~filter_set()
    {
      m_manager->release_filter(m_filter_slot);
    }

    hal::v5::strong_ptr<can_peripheral_manager_v2> m_manager;
    hal::u8 m_filter_slot;
    std::array<extended_id_filter, 2> filters{};
  };

  auto set = hal::v5::make_strong_ptr<filter_set>(p_allocator, p_manager);

  return {
    hal::v5::strong_ptr<hal::can_extended_identifier_filter>(
      set, &filter_set::filters, 0),
    hal::v5::strong_ptr<hal::can_extended_identifier_filter>(
      set, &filter_set::filters, 1),
  };
}

/**
 * @brief Acquire a pair of mask filters
 *
 * NOTE: See `acquire_can_identifier_filter()` for why `driver_allow()` is a
 * no-op here.
 *
 * @return hal::v5::strong_ptr<can_mask_filter_set> - A set of 2x standard
 * mask filters
 */
std::array<hal::v5::strong_ptr<hal::can_mask_filter>, 2>
acquire_can_mask_filter(
  std::pmr::polymorphic_allocator<> p_allocator,
  hal::v5::strong_ptr<can_peripheral_manager_v2> const& p_manager)
{
  struct mask_filter : public hal::can_mask_filter
  {
    void driver_allow(std::optional<pair>) override
    {
      // No-op: hardware ID filtering is not implemented for this platform.
    }
  };

  struct filter_set
  {
    explicit filter_set(
      hal::v5::strong_ptr<can_peripheral_manager_v2> const& p_manager)
      : m_manager(p_manager)
      , m_filter_slot(p_manager->available_filter())
    {
    }

    ~filter_set()
    {
      m_manager->release_filter(m_filter_slot);
    }

    hal::v5::strong_ptr<can_peripheral_manager_v2> m_manager;
    hal::u8 m_filter_slot;
    std::array<mask_filter, 2> filters{};
  };

  auto set = hal::v5::make_strong_ptr<filter_set>(p_allocator, p_manager);

  return {
    hal::v5::strong_ptr<hal::can_mask_filter>(set, &filter_set::filters, 0),
    hal::v5::strong_ptr<hal::can_mask_filter>(set, &filter_set::filters, 1),
  };
}

/**
 * @brief Acquire an extended mask filter
 *
 * NOTE: See `acquire_can_identifier_filter()` for why `driver_allow()` is a
 * no-op here.
 *
 * @return hal::v5::strong_ptr<hal::can_extended_mask_filter> - An extended
 * mask filter
 */
hal::v5::strong_ptr<hal::can_extended_mask_filter>
acquire_can_extended_mask_filter(
  std::pmr::polymorphic_allocator<> p_allocator,
  hal::v5::strong_ptr<can_peripheral_manager_v2> const& p_manager)
{
  struct extended_mask_filter : public hal::can_extended_mask_filter
  {
    explicit extended_mask_filter(
      hal::v5::strong_ptr<can_peripheral_manager_v2> const& p_manager)
      : m_manager(p_manager)
      , m_filter_slot(p_manager->available_filter())
    {
    }

    void driver_allow(std::optional<pair>) override
    {
      // No-op: hardware ID filtering is not implemented for this platform.
    }

    ~extended_mask_filter() override
    {
      m_manager->release_filter(m_filter_slot);
    }

    hal::v5::strong_ptr<can_peripheral_manager_v2> m_manager;
    hal::u8 m_filter_slot;
  };

  return hal::v5::make_strong_ptr<extended_mask_filter>(p_allocator,
                                                          p_manager);
}
}  // namespace hal::lpc40
