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
#include <optional>
#include <span>
#include <utility>

export module hal.arm_mcu.stm32f1:can;

import hal;
import hal.util;

import :constants;
import :pin;

namespace hal::stm32f1 {
/// The receive FIFO a filter bank sends the messages it accepts to
export enum class can_fifo : hal::u8 {
  /// FIFO 0, which the CAN1_RX0 interrupt empties
  select1 = 0,
  /// FIFO 1, which the CAN1_RX1 interrupt empties
  select2 = 1,
};

/// Settings applied when a `hal::stm32f1::can` is constructed
export struct can_settings
{
  /// The RX/TX pin pair to use
  can_pins pins = can_pins::pa11_pa12;
  /// The bus's bit rate
  hal::hertz baud_rate = 100'000 * mp_units::si::hertz;
  /// Loop sent messages back into the receiver as well as onto the bus
  /// (loop back mode, RM0008 section 24.5.3), for testing without another
  /// node
  bool loop_back = false;
};

/**
 * @brief STM32F1 bxCAN driver (CAN1), and its transceiver
 *
 * Owns CAN1 and keeps it powered for as long as it lives. It is the
 * `hal::can_transceiver`: it sends messages and receives every message the
 * hardware filters accept into the buffer given to the constructor.
 *
 * The other CAN interfaces are separate classes, constructed from a `can`
 * and kept no longer than it: `can::bus_manager`, `can::interrupt` and the
 * filter banks. An application that doesn't construct them doesn't link their
 * code. Each filter bank claims one of the 13 free hardware banks.
 *
 * `send()` places the message in a free transmit mailbox and returns, without
 * waiting for it to go out. If all three mailboxes are full, it sleeps until
 * one frees up.
 *
 * On the STM32F103, CAN1 and USB share their packet memory and can't run at
 * the same time, so the driver also claims the USB peripheral.
 *
 * Sends, baud rate changes and filter changes hold the lock given to the
 * constructor, so several contexts can share the CAN bus. Pass a
 * `hal::no_op_lock` when only one context uses it.
 *
 * ```cpp
 * static hal::no_op_lock single_context;
 * static std::array<hal::can_message, 16> receive_buffer{};
 * static hal::stm32f1::can can1(
 *   single_context, receive_buffer, { .baud_rate = 500 * kHz });
 * static hal::stm32f1::can::bus_manager bus_manager(can1);
 * static hal::stm32f1::can::identifier_filters filters(can1);
 *
 * filters[0].allow(0x111);
 * bus_manager.filter_mode(hal::can_message_acceptance::filtered);
 * can1.send({ .id = 0x111, .length = 1, .payload = { 0xAA } });
 * ```
 */
export class can final : public hal::can_transceiver
{
public:
  class bus_manager;
  class interrupt;
  template<typename Allowed, hal::usize Count>
  class filter_bank;

  /// Four standard (11-bit) IDs. Each accepts standard data frames with
  /// exactly that ID.
  using identifier_filters = filter_bank<hal::u16, 4>;
  /// Two extended (29-bit) IDs. Each accepts extended data frames with exactly
  /// that ID.
  using extended_identifier_filters = filter_bank<hal::u32, 2>;
  /// Two standard ID and mask pairs. Each accepts standard frames, data or
  /// remote, whose ID matches the pair's ID in the bits the mask sets.
  using mask_filters = filter_bank<hal::can_mask, 2>;
  /// One extended ID and mask pair. It accepts extended frames, data or
  /// remote, whose ID matches the pair's ID in the bits the mask sets.
  using extended_mask_filter = filter_bank<hal::can_mask_ext, 1>;

  /**
   * @brief Claim CAN1 and its pins, power it on and join the bus
   *
   * It starts bus-on, accepting every message
   * (`hal::can_message_acceptance::all`) until a bus manager's `filter_mode()`
   * changes that.
   *
   * @param p_lock - held by sends, baud rate changes and filter changes
   * @param p_buffer - the receive buffer. Must outlive the driver, and hold at
   * least one message.
   * @param p_settings - pins, baud rate and test mode
   * @throws hal::argument_out_of_domain - if `p_buffer` is empty
   * @throws hal::device_or_resource_busy - if another driver has claimed CAN1,
   * the USB peripheral (which shares CAN1's memory), or one of the pins
   * @throws hal::operation_not_supported - if CAN1's clock can't make the baud
   * rate
   */
  can(hal::ref<hal::basic_lock> p_lock [[clang::lifetimebound]],
      std::span<hal::can_message> p_buffer [[clang::lifetimebound]],
      can_settings const& p_settings = {});

  can(can const&) = delete;
  can& operator=(can const&) = delete;
  can(can&&) = delete;
  can& operator=(can&&) = delete;

  /// Disable CAN1's interrupts, power it off and release its claims
  ~can();

private:
  hal::hertz driver_baud_rate() override;
  /**
   * @throws hal::operation_not_permitted - if CAN1 is bus-off
   * @throws hal::timed_out - if no transmit mailbox frees up within the time
   * six maximum-length frames take at the baud rate, plus 1 ms. Messages
   * already in the mailboxes keep being sent until another node acknowledges
   * them.
   */
  void driver_send(hal::can_message const& p_message) override;
  hal::circular_span<hal::can_message const> driver_receive_buffer() override;
  hal::usize driver_receive_count() override;

  /// Hardware banks 0 to 12 are for `filter_bank` objects. Bank 13 accepts
  /// every message, and is active while the filter mode is `all`.
  static constexpr hal::usize user_bank_count = 13;

  static void fifo0_isr();
  static void fifo1_isr();
  static void transmit_isr();
  static void status_change_isr();

  void receive(u8 p_fifo) noexcept;
  void transmit(hal::can_message const& p_message);
  void set_baud_rate(hal::hertz p_baud_rate);
  void set_filter_mode(hal::can_message_acceptance p_accept);
  void recover_bus();
  u8 claim_bank();
  void release_bank(u8 p_bank) noexcept;
  void set_bank_enabled(u8 p_bank, bool p_enabled) noexcept;

  hal::ref<hal::basic_lock> m_lock;
  std::span<hal::can_message> m_receive_buffer{};
  hal::usize volatile m_receive_count = 0;
  hal::can_receive_callback* volatile m_receive_callback = nullptr;
  hal::can_receive_filter m_receive_filter{};
  hal::can_bus_off_callback* volatile m_bus_off_callback = nullptr;
  hal::notifier m_transmit_notifier;
  /// Banks claimed through `claim_bank()`. The registers hold the rest of
  /// the state: the baud rate (BTR), the pins (AFIO MAPR), which banks are
  /// enabled (FA1R) and the filter mode (FA1R and FMR).
  std::bitset<user_bank_count> m_claimed_banks{};
};

/**
 * @brief CAN1's bus manager: baud rate, filter mode and bus-off
 *
 * Constructed from a `can`, which must outlive it.
 */
class can::bus_manager final : public hal::can_bus_manager
{
public:
  /// @param p_can - the CAN1 driver to manage
  explicit bus_manager(hal::ref<can> p_can [[clang::lifetimebound]]);
  bus_manager(bus_manager const&) = delete;
  bus_manager& operator=(bus_manager const&) = delete;
  bus_manager(bus_manager&&) = delete;
  bus_manager& operator=(bus_manager&&) = delete;
  ~bus_manager() = default;

private:
  void driver_baud_rate(hal::u32 p_hertz) override;
  void driver_filter_mode(hal::can_message_acceptance p_accept) override;
  void driver_on_bus_off(hal::can_bus_off_callback& p_callback) override;
  void driver_clear_bus_off() noexcept override;
  void driver_bus_on() override;

  hal::ref<can> m_can;
};

/**
 * @brief CAN1's receive interrupt
 *
 * The callback runs from the receive interrupts, after the message is stored
 * in the `can`'s receive buffer, for messages that pass the subscriber's
 * `hal::can_receive_filter`.
 *
 * Constructed from a `can`, which must outlive it.
 */
class can::interrupt final : public hal::can_interrupt
{
public:
  /// @param p_can - the CAN1 driver whose messages to report
  explicit interrupt(hal::ref<can> p_can [[clang::lifetimebound]]);
  interrupt(interrupt const&) = delete;
  interrupt& operator=(interrupt const&) = delete;
  interrupt(interrupt&&) = delete;
  interrupt& operator=(interrupt&&) = delete;
  ~interrupt() = default;

private:
  void driver_on_receive(hal::can_receive_callback& p_callback,
                         hal::can_receive_filter p_filter) override;
  void driver_clear_receive() noexcept override;

  hal::ref<can> m_can;
};

/**
 * @brief One of CAN1's hardware filter banks, split into `Count` filters
 *
 * A message is received if any filter of any enabled bank accepts it. A filter
 * accepts nothing until `allow()` gives it a value, and `allow(std::nullopt)`
 * makes it accept nothing again. While none of a bank's filters allows
 * anything, the bank is disabled.
 *
 * Filters only take effect while the bus manager's filter mode is
 * `hal::can_message_acceptance::filtered`.
 *
 * @tparam Allowed - what each filter accepts: a standard ID (`hal::u16`), an
 * extended ID (`hal::u32`), or an ID and mask pair (`hal::can_mask`,
 * `hal::can_mask_ext`)
 * @tparam Count - how many filters the bank holds
 */
template<typename Allowed, hal::usize Count>
class can::filter_bank
{
public:
  /// One filter of the bank
  class filter final : public hal::can_filter<Allowed>
  {
  public:
    filter(filter const&) = delete;
    filter& operator=(filter const&) = delete;
    filter(filter&&) = delete;
    filter& operator=(filter&&) = delete;
    ~filter() = default;

  private:
    friend class filter_bank;
    filter(filter_bank& p_bank, u8 p_slot);

    void driver_allow(std::optional<Allowed> p_allowed) override;

    filter_bank* m_bank;
    u8 m_slot;
  };

  /**
   * @brief Claim a free filter bank of `p_can` and set it up
   *
   * The bank accepts nothing until one of its filters allows something.
   *
   * @param p_can - the CAN driver whose filter banks to claim from
   * @param p_fifo - the receive FIFO the bank's messages go to
   * @throws hal::device_or_resource_busy - if all 13 filter banks are
   * in use
   */
  filter_bank(hal::ref<can> p_can [[clang::lifetimebound]],
              can_fifo p_fifo = can_fifo::select1);
  filter_bank(filter_bank const&) = delete;
  filter_bank& operator=(filter_bank const&) = delete;
  filter_bank(filter_bank&&) = delete;
  filter_bank& operator=(filter_bank&&) = delete;
  /// Disable the bank and hand it back to the driver
  ~filter_bank();

  /**
   * @brief One of the bank's filters
   *
   * constexpr, so that a `hal::static_ref` can refer to a filter of a static
   * bank: `return bank[0];` from a function returning
   * `hal::static_ref<hal::can_id_filter>`.
   *
   * @param p_index - which filter, from 0 to `Count - 1`
   * @return filter& - the filter
   * @throws hal::argument_out_of_domain - if `p_index` is `Count` or more
   */
  [[nodiscard]] constexpr filter& operator[](hal::usize p_index)
    [[clang::lifetimebound]]
  {
    if (p_index >= Count) {
      throw hal::argument_out_of_domain(this);
    }
    return m_filters[p_index];
  }

private:
  template<hal::usize... Index>
  std::array<filter, Count> make_filters(std::index_sequence<Index...>);
  void allow(u8 p_slot, std::optional<Allowed> p_allowed);

  hal::ref<can> m_can;
  std::array<filter, Count> m_filters;
  /// The bank's two registers, FR2 in the upper half
  hal::u64 m_value = 0;
  u8 m_bank;
  /// Filters that allow something
  std::bitset<Count> m_allowing{};
};
}  // namespace hal::stm32f1
