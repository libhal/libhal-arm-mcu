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

#include <bitset>
#include <cstdint>
#include <optional>

#include <libhal-arm-mcu/interrupt.hpp>
#include <libhal-arm-mcu/stm32f1/can2.hpp>
#include <libhal-arm-mcu/stm32f1/clock.hpp>
#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal-arm-mcu/stm32f1/interrupt.hpp>
#include <libhal-arm-mcu/stm32f1/pin.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/bit_limits.hpp>
#include <libhal-util/can.hpp>
#include <libhal-util/enum.hpp>
#include <libhal-util/static_callable.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>
#include <libhal/circular_buffer.hpp>
#include <libhal/error.hpp>
#include <libhal/pointers.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>
#include <nonstd/scope.hpp>

#include "can_reg.hpp"
#include "pin.hpp"
#include "power.hpp"

namespace hal::stm32f1 {
namespace {
/// Enable/Disable controller modes
///
/// @param mode - which mode to enable/disable
/// @param enable_mode - true if you want to enable the mode. False otherwise.
void set_master_mode(bit_mask p_mode, bool p_enable_mode)
{
  bit_modify(can1_reg->MCR).insert(p_mode, p_enable_mode);
}

bool get_master_status(bit_mask p_mode)
{
  return bit_extract(p_mode, can1_reg->MSR);
}

void enter_initialization()
{
  // Enter Initialization mode in order to write to CAN registers.
  set_master_mode(master_control::initialization_request, true);

  // Wait to enter Initialization mode
  while (not get_master_status(master_status::initialization_acknowledge)) {
    continue;
  }
}

void exit_initialization()
{
  // Leave Initialization mode
  set_master_mode(master_control::initialization_request, false);

  // Wait to leave initialization mode
  while (get_master_status(master_status::initialization_acknowledge)) {
    continue;
  }
}

void timed_exit_initialization(hal::steady_clock& p_clock,
                               hal::time_duration p_timeout_time)
{
  // Leave Initialization mode
  set_master_mode(master_control::initialization_request, false);

  auto const deadline = hal::future_deadline(p_clock, p_timeout_time);
  while (deadline > p_clock.uptime()) {
    if (not get_master_status(master_status::initialization_acknowledge)) {
      return;
    }
  }
  hal::safe_throw(hal::timed_out(nullptr));
}

void configure_baud_rate(hal::u32 p_baud_rate)
{
  auto const can_frequency = frequency(peripheral::can1);
  auto const valid_divider =
    calculate_can_bus_divider(can_frequency, static_cast<float>(p_baud_rate));

  if (not valid_divider) {
    hal::safe_throw(hal::operation_not_supported(nullptr));
  }

  auto const divisors = valid_divider.value();

  auto const prescale = divisors.clock_divider - 1U;
  auto const sync_jump_width = divisors.synchronization_jump_width - 1U;

  auto phase_segment1 =
    (divisors.phase_segment1 + divisors.propagation_delay) - 1U;
  auto phase_segment2 = divisors.phase_segment2 - 1U;

  constexpr auto segment2_bit_limit =
    hal::bit_limits<bus_timing::time_segment2.width, std::uint32_t>::max();

  // Check if phase segment 2 does not fit
  if (phase_segment2 > segment2_bit_limit) {
    // Take the extra time quanta and add it to the phase 1 segment
    auto const phase_segment2_remainder = phase_segment2 - segment2_bit_limit;
    phase_segment1 += phase_segment2_remainder;
    // Cap phase segment 2 to the max available in the bit field
    phase_segment2 = segment2_bit_limit;
  }

  bit_modify(can1_reg->BTR)
    .insert<bus_timing::prescalar>(prescale)
    .insert<bus_timing::time_segment1>(phase_segment1)
    .insert<bus_timing::time_segment2>(phase_segment2)
    .insert<bus_timing::sync_jump_width>(sync_jump_width)
    .clear<bus_timing::silent_mode>();
}

void set_filter_bank_mode(filter_bank_master_control p_mode)
{
  bit_modify(can1_reg->FMR)
    .insert<filter_master::initialization_mode>(hal::value(p_mode));
}

void set_filter_type(hal::u8 p_filter, filter_type p_filter_type)
{
  auto const filter_bit_mask = bit_mask::from(p_filter);
  bit_modify(can1_reg->FM1R).insert(filter_bit_mask, hal::value(p_filter_type));
}

void set_filter_scale(hal::u8 p_filter, filter_scale p_scale)
{
  auto const filter_bit_mask = bit_mask::from(p_filter);
  bit_modify(can1_reg->FS1R).insert(filter_bit_mask, hal::value(p_scale));
}

void set_filter_fifo_assignment(hal::u8 p_filter, can_fifo p_fifo)
{
  auto const filter_bit_mask = bit_mask::from(p_filter);
  bit_modify(can1_reg->FFA1R).insert(filter_bit_mask, hal::value(p_fifo));
}

void set_filter_activation_state(hal::u8 p_filter, filter_activation p_state)
{
  auto const filter_bit_mask = bit_mask::from(p_filter);
  bit_modify(can1_reg->FA1R).insert(filter_bit_mask, hal::value(p_state));
}

struct can_data_registers_t
{
  /// TFI register contents
  uint32_t frame = 0;
  /// TID register contents
  uint32_t id = 0;
  /// TDA register contents
  uint32_t data_a = 0;
  /// TDB register contents
  uint32_t data_b = 0;
};

/// Converts desired message to the CANx registers
can_data_registers_t convert_message_to_stm_can(hal::can_message const& message)
{
  can_data_registers_t registers;

  auto frame_info =
    bit_value(0U)
      .insert<frame_length_and_info::data_length_code>(message.length)
      .to<std::uint32_t>();

  uint32_t frame_id = 0;

  if (message.extended) {
    frame_id =
      bit_value(0U)
        .insert<mailbox_identifier::transmit_mailbox_request>(true)
        .insert<mailbox_identifier::remote_request>(message.remote_request)
        .set(mailbox_identifier::identifier_type)
        .insert<mailbox_identifier::extended_identifier>(message.id)
        .to<std::uint32_t>();
  } else {
    frame_id =
      bit_value(0U)
        .insert<mailbox_identifier::transmit_mailbox_request>(true)
        .insert<mailbox_identifier::remote_request>(message.remote_request)
        .clear<mailbox_identifier::identifier_type>()
        .insert<mailbox_identifier::standard_identifier>(message.id)
        .to<std::uint32_t>();
  }

  uint32_t data_a = 0;
  data_a |= message.payload[0] << (0 * 8);
  data_a |= message.payload[1] << (1 * 8);
  data_a |= message.payload[2] << (2 * 8);
  data_a |= message.payload[3] << (3 * 8);

  uint32_t data_b = 0;
  data_b |= message.payload[4] << (0 * 8);
  data_b |= message.payload[5] << (1 * 8);
  data_b |= message.payload[6] << (2 * 8);
  data_b |= message.payload[7] << (3 * 8);

  registers.frame = frame_info;
  registers.id = frame_id;
  registers.data_a = data_a;
  registers.data_b = data_b;

  return registers;
}

bool is_bus_off()
{
  // True = Bus is in sleep mode
  // False = Bus has left sleep mode.
  return bit_extract<master_status::sleep_acknowledge>(can1_reg->MCR);
}

void setup_can(hal::u32 p_baud_rate,
               can_pins p_pins,
               can_self_test p_enable_self_test,
               hal::steady_clock& p_clock,
               hal::time_duration p_timeout_time)
{
  power_on(peripheral::can1);

  set_master_mode(master_control::sleep_mode_request, false);
  set_master_mode(master_control::no_automatic_retransmission, false);
  set_master_mode(master_control::automatic_bus_off_management, false);

  enter_initialization();

  configure_baud_rate(p_baud_rate);

  switch (p_pins) {
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

  bit_modify(can1_reg->BTR)
    .insert<bus_timing::loop_back_mode>(hal::value(p_enable_self_test));

  remap_pins(p_pins);

  timed_exit_initialization(p_clock, p_timeout_time);
}

can_message read_receive_mailbox()
{
  can_message message{};

  auto const fifo0_status = can1_reg->RF0R;
  auto const fifo1_status = can1_reg->RF1R;

  auto fifo_select = can_fifo::select1;

  if (bit_extract<fifo_status::messages_pending>(fifo0_status)) {
    fifo_select = can_fifo::select1;
  } else if (bit_extract<fifo_status::messages_pending>(fifo1_status)) {
    fifo_select = can_fifo::select2;
  } else {
    // Error, tried to receive when there were no pending messages.
    return message;
  }

  uint32_t frame = can1_reg->fifo_mailbox[value(fifo_select)].RDTR;
  uint32_t id = can1_reg->fifo_mailbox[value(fifo_select)].RIR;

  // Extract all of the information from the message frame
  bool const is_remote_request =
    bit_extract<mailbox_identifier::remote_request>(id);
  uint32_t const length =
    bit_extract<frame_length_and_info::data_length_code>(frame);
  uint32_t const format = bit_extract<mailbox_identifier::identifier_type>(id);
  bool const is_extended =
    format == value(mailbox_identifier::id_type::extended);

  message.remote_request = is_remote_request;
  message.length = static_cast<std::uint8_t>(length);
  message.extended = is_extended;

  // Get the frame ID
  if (is_extended) {
    message.id = bit_extract<mailbox_identifier::extended_identifier>(id);
  } else {
    message.id = bit_extract<mailbox_identifier::standard_identifier>(id);
  }

  auto low_read_data = can1_reg->fifo_mailbox[value(fifo_select)].RDLR;
  auto high_read_data = can1_reg->fifo_mailbox[value(fifo_select)].RDHR;

  // Pull the bytes from RDL into the payload array
  message.payload[0] = (low_read_data >> (0 * 8)) & 0xFF;
  message.payload[1] = (low_read_data >> (1 * 8)) & 0xFF;
  message.payload[2] = (low_read_data >> (2 * 8)) & 0xFF;
  message.payload[3] = (low_read_data >> (3 * 8)) & 0xFF;

  // Pull the bytes from RDH into the payload array
  message.payload[4] = (high_read_data >> (0 * 8)) & 0xFF;
  message.payload[5] = (high_read_data >> (1 * 8)) & 0xFF;
  message.payload[6] = (high_read_data >> (2 * 8)) & 0xFF;
  message.payload[7] = (high_read_data >> (3 * 8)) & 0xFF;

  // Release the RX buffer and allow another buffer to be read.
  if (fifo_select == can_fifo::select1) {
    bit_modify(can1_reg->RF0R).set<fifo_status::release_output_mailbox>();
  } else if (fifo_select == can_fifo::select2) {
    bit_modify(can1_reg->RF1R).set<fifo_status::release_output_mailbox>();
  }

  return message;
}

void bus_on()
{
  constexpr auto bus_off_mask = error_status_register::bus_off;
  bool const bus_off = bit_extract<bus_off_mask>(can1_reg->ESR);

  if (not bus_off) {
    return;  // nothing to do here, return
  }

  // RM0008 page 670 states that bus off can be recovered from by entering and
  // Request to enter initialization mode
  enter_initialization();

  // Leave Initialization mode
  exit_initialization();
}

hal::u16 standard_id_to_stm_filter(hal::u16 p_id)
{
  auto const reg = hal::bit_value()
                     .insert<standard_filter_bank::standard_id1>(p_id)
                     .clear(standard_filter_bank::rtr1)
                     .clear(standard_filter_bank::id_extension1)
                     .insert(standard_filter_bank::extended_id1, 0UL)
                     .to<hal::u16>();
  return reg;
}

hal::u32 extended_id_to_stm_filter(hal::u32 p_id)
{
  auto const reg = hal::bit_value()
                     .insert<extended_filter_bank::id>(p_id)
                     .clear(extended_filter_bank::rtr)
                     .clear(extended_filter_bank::id_extension)
                     .clear(extended_filter_bank::reserved)
                     .to<hal::u32>();
  return reg;
}
}  // namespace

can_peripheral_manager_v2::can_peripheral_manager_v2(
  hal::usize p_message_count,
  std::pmr::polymorphic_allocator<> p_allocator,
  hal::u32 p_baud_rate,
  hal::steady_clock& p_clock,
  hal::time_duration p_timeout_time,
  can_pins p_pins,
  can_self_test p_enable_self_test)
  : m_buffer(p_allocator, p_message_count)
  , m_current_baud_rate(p_baud_rate)
{
  setup_can(p_baud_rate, p_pins, p_enable_self_test, p_clock, p_timeout_time);

  initialize_interrupts();

  hal::static_callable<can_peripheral_manager_v2, 0, void(void)> rx_handler(
    [this]() {
      auto const message = read_receive_mailbox();
      // Why is this here? Because there was an stm32f103c8 chip that may have a
      // defect or was damaged in testing. That device was then able to set its
      // length to 9. The actual data in the data registers were garbage data.
      // Even if the device is damaged, its best to throw out those damaged
      // frames then attempt to pass them to a handler that may not able to
      // manage them.
      if (message.length > 8) {
        return;
      }

      if (m_receive_handler) {
        using tag = hal::can_interrupt::on_receive_tag;
        (*m_receive_handler)(tag{}, message);
      }

      m_buffer.push(message);
    });

  // Setup interrupt service routines
  cortex_m::enable_interrupt(irq::can1_rx0, rx_handler.get_handler());
  cortex_m::enable_interrupt(irq::can1_rx1, rx_handler.get_handler());
  bit_modify(can1_reg->IER)
    .set<interrupt_enable_register::fifo0_message_pending>();
  bit_modify(can1_reg->IER)
    .set<interrupt_enable_register::fifo1_message_pending>();
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

void can_peripheral_manager_v2::enable_self_test(bool p_enable)
{
  enter_initialization();
  nonstd::scope_exit on_exit(&exit_initialization);

  if (p_enable) {
    bit_modify(can1_reg->BTR).set<bus_timing::loop_back_mode>();
  } else {
    bit_modify(can1_reg->BTR).clear<bus_timing::loop_back_mode>();
  }
}

void can_peripheral_manager_v2::bus_on()
{
  hal::stm32f1::bus_on();
}

hal::u32 can_peripheral_manager_v2::baud_rate() const
{
  return m_current_baud_rate;
}

void can_peripheral_manager_v2::baud_rate(hal::u32 p_hertz)
{
  enter_initialization();
  // Ensure we have left initialization phase so the peripheral can operate
  // correctly. If an exception is thrown at any point, this will ensure that
  // the can peripheral is taken out of initialization.
  nonstd::scope_exit on_exit(&exit_initialization);
  configure_baud_rate(p_hertz);
  m_current_baud_rate = p_hertz;
}

void can_peripheral_manager_v2::send(can_message const& p_message)
{
  if (is_bus_off()) {
    hal::safe_throw(hal::operation_not_permitted(this));
  }

  can_data_registers_t const registers = convert_message_to_stm_can(p_message);
  std::optional<hal::u8> available_mailbox{};

  while (not available_mailbox) {
    hal::u32 const status_register = can1_reg->TSR;
    // Check if any buffer is available.
    if (bit_extract<transmit_status::transmit_mailbox0_empty>(
          status_register)) {
      available_mailbox = 0;
    } else if (bit_extract<transmit_status::transmit_mailbox1_empty>(
                 status_register)) {
      available_mailbox = 1;
    } else if (bit_extract<transmit_status::transmit_mailbox2_empty>(
                 status_register)) {
      available_mailbox = 2;
    }
  }

  // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
  auto& mailbox = can1_reg->transmit_mailbox[*available_mailbox];
  // The above is removed from lint because the while loop is our check that the
  // optional value has been set.

  bit_modify(mailbox.TDTR)
    .insert<frame_length_and_info::data_length_code>(p_message.length);
  mailbox.TDLR = registers.data_a;
  mailbox.TDHR = registers.data_b;
  mailbox.TIR = registers.id;
}

void can_peripheral_manager_v2::on_receive(
  can_interrupt::optional_receive_handler const& p_callback)
{
  m_receive_handler = p_callback;
}

can_peripheral_manager_v2::~can_peripheral_manager_v2()
{
  hal::cortex_m::disable_interrupt(irq::can1_rx0);
  hal::cortex_m::disable_interrupt(irq::can1_rx1);
  hal::cortex_m::disable_interrupt(irq::can1_sce);
  power_off(peripheral::can1);
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
  struct inner_transceiver : public hal::can_transceiver
  {
  public:
    explicit inner_transceiver(
      hal::v5::strong_ptr<can_peripheral_manager_v2> const& p_manager)
      : m_manager(p_manager)
    {
    }

    inner_transceiver(inner_transceiver const&) = delete;
    inner_transceiver& operator=(inner_transceiver const&) = delete;
    inner_transceiver(inner_transceiver&&) = delete;
    inner_transceiver& operator=(inner_transceiver&&) = delete;
    ~inner_transceiver() override = default;

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

    hal::v5::strong_ptr<can_peripheral_manager_v2> m_manager;
  };

  return hal::v5::make_strong_ptr<inner_transceiver>(p_allocator, p_manager);
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
    bus_manager(hal::v5::strong_ptr<can_peripheral_manager_v2> const& p_manager)
      : m_manager(p_manager)
    {
      hal::static_callable<bus_manager, 0, void(void)> static_callable(
        [this]() {
          if (m_bus_off_handler) {
            (*m_bus_off_handler)(hal::can_bus_manager::bus_off_tag{});
          }
        });

      cortex_m::enable_interrupt(irq::can1_sce, static_callable.get_handler());
    }
    bus_manager(bus_manager const&) = delete;
    bus_manager& operator=(bus_manager const&) = delete;
    bus_manager(bus_manager&&) = delete;
    bus_manager& operator=(bus_manager&&) = delete;
    ~bus_manager() override
    {
      cortex_m::disable_interrupt(irq::can1_sce);
      m_bus_off_handler = std::nullopt;
    }

  private:
    friend class can_peripheral_manager;

    void driver_baud_rate(hal::u32 p_hertz) override
    {
      m_manager->baud_rate(p_hertz);
    }

    void driver_filter_mode(accept) override
    {
      // this does nothing for now. We should consider dropping this in favor of
      // always using a filter to manager message acceptance. A single mask
      // filter with its mask set to all ZEROs would do the trick.
    }

    void driver_on_bus_off(optional_bus_off_handler p_callback) override
    {
      m_bus_off_handler = p_callback;
    }

    void driver_bus_on() override
    {
      m_manager->bus_on();
    }

    hal::v5::strong_ptr<can_peripheral_manager_v2> m_manager;
    can_bus_manager::optional_bus_off_handler m_bus_off_handler{};
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
    interrupt(hal::v5::strong_ptr<can_peripheral_manager_v2> const& p_manager)
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
 * @return identifier_filter_set - A set of 4x identifier filters. When
 * destroyed, releases the filter resource it held on to.
 */
std::array<hal::v5::strong_ptr<hal::can_identifier_filter>, 4>
acquire_can_identifier_filter(
  std::pmr::polymorphic_allocator<> p_allocator,
  hal::v5::strong_ptr<can_peripheral_manager_v2> const& p_manager,
  can_fifo p_fifo)
{
  struct identifier_filter : public hal::can_identifier_filter
  {
    identifier_filter(can_filter_resource p_resource)
      : m_resource(p_resource)
    {
    }

    void driver_allow(std::optional<u16> p_id) override
    {
      auto const id = p_id.value_or(0 /* disable_id.standard */);

      set_filter_activation_state(m_resource.filter_index,
                                  filter_activation::not_active);

      auto& filter = can1_reg->filter_registers[m_resource.filter_index];
      auto const reg = standard_id_to_stm_filter(id);

      switch (m_resource.word_index) {
        case 0:
          hal::bit_modify(filter.FR1)
            .insert<standard_filter_bank::sub_bank1>(reg);
          break;
        case 1:
          hal::bit_modify(filter.FR1)
            .insert<standard_filter_bank::sub_bank2>(reg);
          break;
        case 2:
          hal::bit_modify(filter.FR2)
            .insert<standard_filter_bank::sub_bank1>(reg);
          break;
        case 3:
          hal::bit_modify(filter.FR2)
            .insert<standard_filter_bank::sub_bank2>(reg);
          break;
        default:
          hal::safe_throw(hal::operation_not_supported(this));
          break;
      }

      set_filter_activation_state(m_resource.filter_index,
                                  filter_activation::active);
    }

    can_filter_resource m_resource;
  };

  struct filter_set
  {
    filter_set(hal::v5::strong_ptr<can_peripheral_manager_v2> const& p_manager,
               can_fifo p_fifo)
      : m_manager(p_manager)
      , filters{
        identifier_filter{ { .filter_index = 0, .word_index = 0 } },
        identifier_filter{ { .filter_index = 0, .word_index = 1 } },
        identifier_filter{ { .filter_index = 0, .word_index = 2 } },
        identifier_filter{ { .filter_index = 0, .word_index = 3 } },
      }
    {
      auto const available_filter = p_manager->available_filter();
      for (auto& filter : filters) {
        filter.m_resource.filter_index = available_filter;
      }
      // Required to change filter scale and type
      set_filter_bank_mode(filter_bank_master_control::initialization);

      // On scope exit, whether via a return or an exception, invoke this.
      nonstd::scope_exit on_exit([available_filter]() {
        set_filter_bank_mode(filter_bank_master_control::active);
        set_filter_activation_state(available_filter,
                                    filter_activation::active);
      });

      set_filter_activation_state(available_filter,
                                  filter_activation::not_active);
      set_filter_scale(available_filter, filter_scale::dual_16_bit_scale);
      set_filter_type(available_filter, filter_type::list);
      set_filter_fifo_assignment(available_filter, p_fifo);

      auto const disable_reg =
        standard_id_to_stm_filter(0 /* disable_id.standard*/);
      auto const disable_mask = (disable_reg << 16) | disable_reg;
      can1_reg->filter_registers[available_filter].FR1 = disable_mask;
      can1_reg->filter_registers[available_filter].FR2 = disable_mask;
    }

    ~filter_set()
    {
      auto const index = filters[0].m_resource.filter_index;
      m_manager->release_filter(index);
      set_filter_activation_state(index, filter_activation::not_active);
    }

    hal::v5::strong_ptr<can_peripheral_manager_v2> m_manager;
    std::array<identifier_filter, 4> filters;
  };

  auto set =
    hal::v5::make_strong_ptr<filter_set>(p_allocator, p_manager, p_fifo);

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
 * @return extended_identifier_filter_set - A set of 2x extended identifier
 * filters.
 */
std::array<hal::v5::strong_ptr<hal::can_extended_identifier_filter>, 2>
acquire_can_extended_identifier_filter(
  std::pmr::polymorphic_allocator<> p_allocator,
  hal::v5::strong_ptr<can_peripheral_manager_v2> const& p_manager,
  can_fifo p_fifo)
{
  struct extended_id_filter : public hal::can_extended_identifier_filter
  {
    extended_id_filter(can_filter_resource p_resource)
      : m_resource(p_resource)
    {
    }

    void driver_allow(std::optional<u32> p_id) override
    {
      auto const id = p_id.value_or(/*disable_id.extended*/ 0);
      auto const reg = extended_id_to_stm_filter(id);

      set_filter_activation_state(m_resource.filter_index,
                                  filter_activation::not_active);

      auto& filter = can1_reg->filter_registers[m_resource.filter_index];

      if (m_resource.word_index == 0) {
        filter.FR1 = reg;
      } else {
        filter.FR2 = reg;
      }

      set_filter_activation_state(m_resource.filter_index,
                                  filter_activation::active);
    }

    can_filter_resource m_resource;
  };

  struct filter_set
  {
    filter_set(hal::v5::strong_ptr<can_peripheral_manager_v2> const& p_manager,
               can_fifo p_fifo)
      : m_manager(p_manager)
      , m_filters({
          { can_filter_resource{ .filter_index = 0, .word_index = 0 } },
          { can_filter_resource{ .filter_index = 0, .word_index = 1 } },
        })
    {
      auto const available_filter = p_manager->available_filter();

      for (auto& filter : m_filters) {
        filter.m_resource.filter_index = available_filter;
      }

      // Required to set filter scale and type
      set_filter_bank_mode(filter_bank_master_control::initialization);

      // On scope exit, whether via a return or an exception, invoke this.
      nonstd::scope_exit on_exit([available_filter]() {
        set_filter_bank_mode(filter_bank_master_control::active);
        set_filter_activation_state(available_filter,
                                    filter_activation::active);
      });

      set_filter_activation_state(available_filter,
                                  filter_activation::not_active);
      set_filter_scale(available_filter, filter_scale::single_32_bit_scale);
      set_filter_type(available_filter, filter_type::list);
      set_filter_fifo_assignment(available_filter, p_fifo);

      auto const disable_mask =
        extended_id_to_stm_filter(/*disable_id.extended*/ 0);

      can1_reg->filter_registers[available_filter].FR1 = disable_mask;
      can1_reg->filter_registers[available_filter].FR2 = disable_mask;
    }

    // NOLINTNEXTLINE(bugprone-exception-escape)
    ~filter_set()
    {
      auto const index = m_filters[0].m_resource.filter_index;
      m_manager->release_filter(index);
      set_filter_activation_state(index, filter_activation::not_active);
    }

    hal::v5::strong_ptr<can_peripheral_manager_v2> m_manager;
    std::array<extended_id_filter, 2> m_filters;
  };

  auto set =
    hal::v5::make_strong_ptr<filter_set>(p_allocator, p_manager, p_fifo);

  return {
    hal::v5::strong_ptr<hal::can_extended_identifier_filter>(
      set, &filter_set::m_filters, 0),
    hal::v5::strong_ptr<hal::can_extended_identifier_filter>(
      set, &filter_set::m_filters, 1),
  };
}

/**
 * @brief Acquire a pair of mask filters
 *
 * @param p_manager - Manager for which to extract the filter
 * @param p_fifo - Select the FIFO to store the received message
 * @return hal::v5::strong_ptr<can_mask_filter_set> - A set of 2x standard
 * mask filters
 */
std::array<hal::v5::strong_ptr<hal::can_mask_filter>, 2>
acquire_can_mask_filter(
  std::pmr::polymorphic_allocator<> p_allocator,
  hal::v5::strong_ptr<can_peripheral_manager_v2> const& p_manager,
  can_fifo p_fifo)
{
  struct mask_filter : public hal::can_mask_filter
  {
    mask_filter(can_filter_resource p_resource)
      : m_resource(p_resource)
    {
    }

    void driver_allow(std::optional<pair> p_pair) override
    {
      auto const selected_pair = p_pair.value_or(pair{
        .id = 0,  // disable_id.standard,
        .mask = 0x1FF,
      });

      auto& filter = can1_reg->filter_registers[m_resource.filter_index];

      set_filter_activation_state(m_resource.filter_index,
                                  filter_activation::not_active);

      auto const id_reg = standard_id_to_stm_filter(selected_pair.id);
      auto const mask_reg = standard_id_to_stm_filter(selected_pair.mask);

      if (m_resource.word_index == 0) {
        hal::bit_modify(filter.FR1)
          .insert<standard_filter_bank::sub_bank1>(id_reg)
          .insert<standard_filter_bank::sub_bank2>(mask_reg);
      } else {
        hal::bit_modify(filter.FR2)
          .insert<standard_filter_bank::sub_bank1>(id_reg)
          .insert<standard_filter_bank::sub_bank2>(mask_reg);
      }

      set_filter_activation_state(m_resource.filter_index,
                                  filter_activation::active);
    }

    can_filter_resource m_resource;
  };

  struct mask_filter_set
  {
    mask_filter_set(
      hal::v5::strong_ptr<can_peripheral_manager_v2> const& p_manager,
      hal::u8 p_filter_index,
      can_fifo p_fifo)
      : m_manager(p_manager)
      , filter{
        mask_filter{ { .filter_index = p_filter_index, .word_index = 0 } },
        mask_filter{ { .filter_index = p_filter_index, .word_index = 1 } },
      }
    {
      // Required to change filter scale and type
      set_filter_bank_mode(filter_bank_master_control::initialization);

      // On scope exit, whether via a return or an exception, invoke this.
      nonstd::scope_exit on_exit([p_filter_index]() {
        set_filter_bank_mode(filter_bank_master_control::active);
        set_filter_activation_state(p_filter_index, filter_activation::active);
      });

      set_filter_activation_state(p_filter_index,
                                  filter_activation::not_active);
      set_filter_scale(p_filter_index, filter_scale::dual_16_bit_scale);
      set_filter_type(p_filter_index, filter_type::mask);
      set_filter_fifo_assignment(p_filter_index, p_fifo);

      auto const disable_id_reg =
        standard_id_to_stm_filter(0 /* disable_id.standard*/);
      auto const disable_mask_reg = standard_id_to_stm_filter(0x1FF);
      auto const disable_mask = (disable_mask_reg << 16) | disable_id_reg;

      can1_reg->filter_registers[p_filter_index].FR1 = disable_mask;
      can1_reg->filter_registers[p_filter_index].FR2 = disable_mask;
    }

    // NOLINTNEXTLINE(bugprone-exception-escape)
    ~mask_filter_set()
    {
      auto const index = filter[0].m_resource.filter_index;
      m_manager->release_filter(index);
      set_filter_activation_state(index, filter_activation::not_active);
    }

    hal::v5::strong_ptr<can_peripheral_manager_v2> m_manager;
    std::array<mask_filter, 2> filter;
  };

  auto set = hal::v5::make_strong_ptr<mask_filter_set>(
    p_allocator, p_manager, p_manager->available_filter(), p_fifo);

  return {
    hal::v5::strong_ptr<hal::can_mask_filter>(set, &mask_filter_set::filter, 0),
    hal::v5::strong_ptr<hal::can_mask_filter>(set, &mask_filter_set::filter, 1),
  };
}

/**
 * @brief Acquire an extended mask filter
 *
 * @param p_manager - Manager for which to extract the filter
 * @param p_fifo - Select the FIFO to store the received message
 * @return hal::v5::strong_ptr<hal::can_extended_mask_filter> - An extended
 * mask filter
 */
hal::v5::strong_ptr<hal::can_extended_mask_filter>
acquire_can_extended_mask_filter(
  std::pmr::polymorphic_allocator<> p_allocator,
  hal::v5::strong_ptr<can_peripheral_manager_v2> const& p_manager,
  can_fifo p_fifo)
{
  struct extended_mask_filter : public hal::can_extended_mask_filter
  {
    extended_mask_filter(
      hal::v5::strong_ptr<can_peripheral_manager_v2> const& p_manager,
      can_fifo p_fifo)
      : m_manager(p_manager)
      , m_filter_index(p_manager->available_filter())
    {
      set_filter_bank_mode(filter_bank_master_control::initialization);

      // On scope exit, whether via a return or an exception, invoke this.
      nonstd::scope_exit on_exit([this]() {
        // Deactivate filter initialization mode (clear bit)
        set_filter_bank_mode(filter_bank_master_control::active);
        set_filter_activation_state(m_filter_index, filter_activation::active);
      });

      set_filter_activation_state(m_filter_index,
                                  filter_activation::not_active);
      set_filter_scale(m_filter_index, filter_scale::single_32_bit_scale);
      set_filter_type(m_filter_index, filter_type::mask);
      set_filter_fifo_assignment(m_filter_index, p_fifo);

      auto const id_reg = extended_id_to_stm_filter(/*disable_id.extended*/ 0);
      auto const mask_reg = extended_id_to_stm_filter(0x1FFF'FFFF);
      can1_reg->filter_registers[m_filter_index].FR1 = id_reg;
      can1_reg->filter_registers[m_filter_index].FR2 = mask_reg;
    }

    void driver_allow(std::optional<pair> p_pair) override
    {
      auto const selected_pair = p_pair.value_or(pair{
        .id = 0,  // disable_id.standard,
        .mask = 0x1FFF'FFFF,
      });

      auto const id_reg = extended_id_to_stm_filter(selected_pair.id);
      auto const mask_reg = extended_id_to_stm_filter(selected_pair.mask);

      auto& filter = can1_reg->filter_registers[m_filter_index];

      set_filter_activation_state(m_filter_index,
                                  filter_activation::not_active);
      filter.FR1 = id_reg;
      filter.FR2 = mask_reg;
      set_filter_activation_state(m_filter_index, filter_activation::active);
    }

    ~extended_mask_filter() override
    {
      m_manager->release_filter(m_filter_index);
      set_filter_activation_state(m_filter_index,
                                  filter_activation::not_active);
    }

    hal::v5::strong_ptr<can_peripheral_manager_v2> m_manager;
    hal::u8 m_filter_index;
  };

  return hal::v5::make_strong_ptr<extended_mask_filter>(
    p_allocator, p_manager, p_fifo);
}
}  // namespace hal::stm32f1
