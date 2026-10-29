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

module;

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <type_traits>

export module hal.arm_mcu.stm32_generic:i2c;

import hal;
import hal.util;
import hal.arm_mcu.cortex_m;

import :dma;

namespace hal::stm32_generic {
/// SPI Control Register 1 (mirrors the field layout, but for i2c cr1)
namespace i2c_cr1 {
/// 0: I2C Peripheral not under reset
/// 1: I2C Peripheral under reset state
constexpr auto software_reset = hal::bit_mask::from<15>();
/// 0: Releases SMBA pin high. Alert Response Address Header followed by
/// NACK.
/// 1: Drives SMBA pin low. Alert Response Address Header followed by ACK
constexpr auto smbus_alert = hal::bit_mask::from<13>();
/// This bit is set and cleared by software, and cleared by hardware when
/// PEC is transferred or by a START or Stop condition or when PE=0.
/// 0: No PEC transfer
/// 1: PEC transfer
constexpr auto pec = hal::bit_mask::from<12>();
/// 0: ACK bit controls the (N)ACK of the current byte being received in
/// the shift register. The PEC bit indicates that current byte in shift
/// register is a PEC.
/// 1: ACK bit controls the (N)ACK of the next byte which will be received
/// in the shift register. The PEC bit indicates that the next byte in the
/// shift register is a PEC
constexpr auto pos = hal::bit_mask::from<11>();
/// This bit is set and cleared by software and cleared by hardware when
/// PE=0
/// 0: no ack return
/// 1: Acknowledge returned after a byte is received (matched address or
/// data)
constexpr auto ack_enable = hal::bit_mask::from<10>();
/// Master mode:
/// 0: no stop generation
/// 1: Stop generation after the current byte transfer or after the
/// current
constexpr auto stop = hal::bit_mask::from<9>();
/// Master mode:
/// 0: no start generation
/// 1: repeated start generation
constexpr auto start = hal::bit_mask::from<8>();
/// 0: Clock stretching enable
/// 1: Clock stretching disable
constexpr auto clock_stretch_disable = hal::bit_mask::from<7>();
/// 0: General call disable Address 0x00 is NACKed
/// 1: General call enable Address 0x00 is ACKed
constexpr auto general_call_enable = hal::bit_mask::from<6>();
/// 0: PEC calculation disable
/// 1: PEC calculation enable
constexpr auto pec_enable = hal::bit_mask::from<5>();
/// 0: ARP disable
/// 1: ARP enable
constexpr auto arp_enable = hal::bit_mask::from<4>();
/// 0: SMBus device
/// 1: SMBus host
constexpr auto smbus_type = hal::bit_mask::from<3>();
/// 0: i2c mode
/// 1: SMBus mode
constexpr auto smbus_mode = hal::bit_mask::from<1>();
/// 0: peripheral disable
/// 1: peripheral enable
constexpr auto peripheral_enable = hal::bit_mask::from<0>();
}  // namespace i2c_cr1

namespace i2c_cr2 {
/// 0: Next DMA EOT is not the last transfer
/// 1: Next DMA EOT is the last transfer
constexpr auto last_dma = hal::bit_mask::from<12>();
/// 0: DMA requests disabled
/// 1: DMA request enabled when TxE=1 or RxNE =1
constexpr auto dma_en = hal::bit_mask::from<11>();
/// 0: TxE = 1 or RxNE = 1 does not generate any interrupt.
/// 1: TxE = 1 or RxNE = 1 generates Event Interrupt
constexpr auto buffer_intr_en = hal::bit_mask::from<10>();
/// 0: Event interrupt disabled
/// 1: Event interrupt enabled
/// This interrupt is generated when:
/// - SB = 1 (Master)
/// - ADDR = 1 (Master/Slave)
/// - ADD10= 1 (Master)
/// - STOPF = 1 (Slave)
/// - BTF = 1 with no TxE or RxNE event
/// - TxE event to 1 if ITBUFEN = 1
/// - RxNE event to 1if ITBUFEN = 1
constexpr auto event_intr_en = hal::bit_mask::from<9>();
/// 0: Error interrupt disabled
/// 1: Error interrupt enabled
/// This interrupt is generated when:
/// - BERR = 1
/// - ARLO = 1
/// - AF = 1
/// - OVR = 1
/// - PECERR = 1
/// - TIMEOUT = 1
/// - SMBALERT = 1
constexpr auto error_intr_en = hal::bit_mask::from<8>();
/// APB clock frequency
/// how many MHz
constexpr auto apb_frequency = hal::bit_mask::from<5, 0>();
}  // namespace i2c_cr2

namespace i2c_sr1 {
constexpr auto smb_alert = hal::bit_mask::from<15>();
/// 0: No timeout error
/// 1: SCL remained LOW for 25 ms (Timeout)
/// or
/// Master cumulative clock low extend time more than 10 ms (Tlow:mext)
/// or
/// Slave cumulative clock low extend time more than 25 ms (Tlow:sext)
/// Cleared by software writing 0, or by hardware when PE=0
constexpr auto timeout_error = hal::bit_mask::from<14>();
/// 0: no PEC error: receiver returns ACK after PEC reception (if ACK=1)
/// 1: PEC error: receiver returns NACK after PEC reception (whatever ACK)
/// Cleared by software writing 0, or by hardware when PE=0
constexpr auto pec_error = hal::bit_mask::from<12>();
/// 0: No overrun/underrun
/// 1: Overrun or underrun
/// Cleared by software writing 0, or by hardware when PE=0
constexpr auto over_under_run = hal::bit_mask::from<11>();
/// 0: No acknowledge failure
/// 1: Acknowledge failure
/// Cleared by software writing 0, or by hardware when PE=0
constexpr auto ack_failure = hal::bit_mask::from<10>();
/// 0: No Arbitration Lost detected
/// 1: Arbitration Lost detected
/// Cleared by software writing 0, or by hardware when PE=0
constexpr auto arbitration_lost = hal::bit_mask::from<9>();
/// 0: No misplaced Start or Stop condition
/// 1: Misplaced Start or Stop condition
/// Cleared by software writing 0, or by hardware when PE=0
constexpr auto bus_error = hal::bit_mask::from<8>();
/// 0: Data register not empty
/// 1: Data register empty
/// Cleared by software writing to the DR register or by hardware after a
/// start or a stop condition or when PE=0
constexpr auto tx_empty = hal::bit_mask::from<7>();
/// 0: Data register empty
/// 1: Data register not empty
/// Cleared by software reading or writing the DR register or by hardware
/// when PE=0.RxNE is not set in case of ARLO event
constexpr auto rx_not_empty = hal::bit_mask::from<6>();
/// used in slave mode
constexpr auto stop_detection = hal::bit_mask::from<4>();
/// 0: No ADD10 event occurred
/// 1: Master has sent first address byte (header)
/// Cleared by software by either a read or write in the DR register or by
/// Cleared by software reading the SR1 register followed by a write in
/// the DR register of the second address byte, or by hardware when PE=0.
constexpr auto addr10 = hal::bit_mask::from<3>();
/// 0: Data byte transfer not done
/// 1: Data byte transfer succeeded
/// Cleared by software by either a read or write in the DR register or by
/// hardware after a start or a stop condition in transmission or when
/// PE=0.
constexpr auto byte_transfered_finish = hal::bit_mask::from<2>();
/// This bit is cleared by software reading SR1 register followed reading
/// SR2, or by hardware when PE=0. This interrupt is generated when:
/// 0: No end of address transmission
/// 1: End of address transmission
constexpr auto addr = hal::bit_mask::from<1>();
/// Cleared by software by reading the SR1 register followed by writing
/// the DR register, or by hardware when peripheral_enable=0
/// 0: No Start condition
/// 1: Start condition generated.
constexpr auto start = hal::bit_mask::from<0>();
}  // namespace i2c_sr1

namespace i2c_sr2 {
/// 0: No General Call
/// 1: General Call Address received when ENGC=1
/// Cleared by hardware after a Stop condition or repeated Start
/// condition, or when PE=0
constexpr auto general_call_addr = hal::bit_mask::from<4>();
/// 0: No communication on the bus
/// 1: Communication ongoing on the bus
/// It indicates a communication in progress on the bus. This information
/// is still updated when the interface is disabled (PE=0).
constexpr auto trans_reciever = hal::bit_mask::from<2>();
/// 0: No communication on the bus
/// 1: Communication ongoing on the bus
/// It indicates a communication in progress on the bus. This information
/// is still updated when the interface is disabled (PE=0).
constexpr auto bus_busy = hal::bit_mask::from<1>();
/// 0: Slave Mode
/// 1: Master Mode
/// Cleared by hardware after detecting a Stop condition on the bus or a
/// loss of arbitration (ARLO=1), or by hardware when PE=0.
constexpr auto master = hal::bit_mask::from<0>();
}  // namespace i2c_sr2

namespace i2c_ccr {
/// 0: Standard mode I2C
/// 1: Fast mode I2C
constexpr auto i2c_mode_sel = hal::bit_mask::from<15>();
/// Fast mode duty cycle
/// 0: SCL low/high = 2
/// 1: SCL low/high = 16/9
constexpr auto fast_mode_duty = hal::bit_mask::from<14>();
/// refer to CH 18.6.8 in the User manual () for equation
constexpr auto ccr = hal::bit_mask::from<11, 0>();
}  // namespace i2c_ccr

namespace i2c_trise {
/// Maximum SCL rise time, in peripheral clock cycles plus one
constexpr auto rise_time = hal::bit_mask::from<5, 0>();
}  // namespace i2c_trise

/// i2c register map (RM0383 CH 18.6.11)
struct i2c_reg_t
{
  /// Control reg 1 (CH 18.6.1)
  hal::u32 volatile cr1;
  /// Control reg 2 (CH 18.6.2)
  hal::u32 volatile cr2;
  /// Own address reg 1 (CH 18.6.3)
  hal::u32 volatile oar1;
  /// Own address reg 2 (CH 18.6.4)
  hal::u32 volatile oar2;
  /// Data reg (CH 18.6.5)
  hal::u32 volatile data_reg;
  /// Status reg 1 (CH 18.6.6)
  hal::u32 volatile sr1;
  /// Status reg 2 (CH 18.6.7)
  hal::u32 volatile sr2;
  /// Clock control reg (CH 18.6.8)
  hal::u32 volatile ccr;
  /// Rise time reg (CH 18.6.9)
  hal::u32 volatile trise;
  /// Noise filter reg (CH 18.6.10)
  hal::u32 volatile filter;
};

/// Clock control and rise time settings for an STM32 I2C bus
export struct i2c_timing
{
  /// CR2 FREQ field: the input clock in whole MHz
  u32 frequency;
  /// CCR divider field
  u32 ccr;
  /// TRISE field: maximum SCL rise time in peripheral clock cycles, plus one
  u32 rise_time;
  /// Fast mode (up to 400 kHz) with a 1:2 SCL high:low duty cycle, instead of
  /// standard mode (up to 100 kHz)
  bool fast_mode;
};

/**
 * @brief Compute the FREQ, CCR and TRISE settings for an i2c clock rate,
 * without touching any hardware
 *
 * The bus never runs faster than `p_clock_rate`. A rate above fast mode's
 * 400 kHz runs at 400 kHz. Otherwise the rate is rounded down to what the
 * clock divider can do.
 *
 * @param p_clock_rate - requested i2c clock rate
 * @param p_frequency - the peripheral's input clock (PCLK1 on the STM32F1),
 * not the bus rate
 * @return i2c_timing - the register settings
 * @throws hal::operation_not_supported - if the clock rate is zero, if the
 * bus can't run as slowly as requested, or if the peripheral's input clock
 * is outside what that mode allows: 2 MHz to 50 MHz, and at least 4 MHz for
 * fast mode (RM0008 section 26.3.3).
 */
export i2c_timing compute_i2c_timing(hal::hertz p_clock_rate,
                                     hal::hertz p_frequency)
{
  using namespace mp_units::si::unit_symbols;
  constexpr hal::hertz standard_mode_max_rate = 100 * kHz;
  constexpr hal::hertz fast_mode_max_rate = 400 * kHz;
  constexpr hal::hertz min_input_clock = 2 * MHz;
  constexpr hal::hertz fast_mode_min_input_clock = 4 * MHz;
  constexpr hal::hertz max_input_clock = 50 * MHz;
  constexpr u32 standard_mode_min_ccr = 4;
  constexpr u32 max_ccr = 0xFFF;

  // A rate fast mode can't reach runs at fast mode's top rate, which is
  // slower than requested and so safe for the devices on the bus.
  auto const rate = std::min(p_clock_rate, fast_mode_max_rate);
  bool const fast_mode = rate > standard_mode_max_rate;
  auto const min_supported_input_clock =
    fast_mode ? fast_mode_min_input_clock : min_input_clock;

  if ((rate == hal::hertz::zero()) or
      (p_frequency < min_supported_input_clock) or
      (p_frequency > max_input_clock)) {
    throw hal::operation_not_supported(nullptr);
  }

  // Standard mode: SCL high and low are each CCR cycles, so one SCL period
  // is 2 * CCR cycles. Fast mode with a 1:2 duty cycle: high is CCR and low
  // is 2 * CCR cycles, so a period is 3 * CCR cycles. Rounding CCR up keeps
  // the bus at or below the requested rate.
  auto const cycles_per_ccr = fast_mode ? 3U : 2U;
  auto const ccr_rate = cycles_per_ccr * rate;
  auto const rounded_ccr = ((p_frequency + ccr_rate) - (1U * Hz)) / ccr_rate;
  auto const ccr = std::max(rounded_ccr.numerical_value_in(mp_units::one),
                            fast_mode ? 1U : standard_mode_min_ccr);

  // A larger divider would be needed to go this slowly.
  if (ccr > max_ccr) {
    throw hal::operation_not_supported(nullptr);
  }

  // FREQ is the input clock in whole MHz. TRISE is the maximum SCL rise time
  // in cycles of that clock, rounded down, plus one (RM0008 section 26.6.9).
  auto const frequency = p_frequency.in(MHz, mp_units::truncated);
  auto const max_rise_time = fast_mode ? (300U * ns) : (1000U * ns);
  auto const rise_cycles = max_rise_time * frequency;

  return {
    .frequency = frequency.numerical_value_in(MHz),
    .ccr = ccr,
    .rise_time =
      rise_cycles.numerical_value_in(mp_units::one, mp_units::truncated) + 1U,
    .fast_mode = fast_mode,
  };
}

/**
 * @brief A generic i2c implementation for all stm32 series MCUs
 *
 * This class is meant to only be used by platform libraries or drivers
 * aiming to be platform libraries for stm32 devices. Application developers
 * should use the driver specific to their platform instead of this class as
 * they handle proper initialization for you.
 *
 * There are two transactions. One polls every byte and needs nothing else.
 * The other moves the data bytes by DMA, sleeping on the calling context's
 * waiter while they move, and polls the rest.
 *
 * Only master mode with 7-bit addressing in hardware is supported; a 10-bit
 * address is sent as described in `hal::i2c::transaction()`.
 */
export class i2c
{
public:
  /// How long a transaction waits for the bus to make progress, for example
  /// while a device stretches the clock, before throwing `hal::timed_out`.
  /// It matches the SMBus clock-low timeout.
  static constexpr hal::u32 stall_timeout_ms = 25;

  /**
   * @brief Construct a new i2c object
   *
   * The constructor doesn't touch the peripheral. Before calling
   * `configure()`, the platform driver must:
   *
   * 1. Power on the i2c peripheral pointed to by `p_i2c`.
   * 2. Configure its SCL and SDA pins as open-drain alternate function
   *    outputs.
   *
   * Before the first DMA transaction, it must also route the peripheral's
   * error interrupt to `handle_error_interrupt()`.
   *
   * Accessing the registers of a peripheral that isn't powered faults.
   *
   * @param p_i2c - i2c peripheral address
   */
  explicit i2c(void* p_i2c)
    : m_i2c(p_i2c)
  {
  }

  i2c(i2c const&) = delete;
  i2c& operator=(i2c const&) = delete;
  i2c(i2c&&) = delete;
  i2c& operator=(i2c&&) = delete;

  /**
   * @brief Leaves the peripheral as it is
   *
   * Like the constructor, the destructor doesn't touch the registers, since
   * the peripheral may already be powered off. The platform specific driver
   * disables the peripheral, for example by resetting it, before powering it
   * off.
   */
  ~i2c() = default;

  /**
   * @brief Configure the i2c peripheral
   *
   * Each stm32 series MCU has its own clock tree, so the caller supplies the
   * peripheral's input clock.
   *
   * The bus never runs faster than `p_settings.clock_rate`. A rate above fast
   * mode's 400 kHz runs at 400 kHz. Otherwise the rate is rounded down to
   * what the clock divider can do.
   *
   * @param p_settings - i2c settings
   * @param p_frequency - the peripheral's input clock (PCLK1 on the
   * STM32F1), not the bus rate
   * @throws hal::operation_not_supported - if the clock rate is zero, if the
   * bus can't run as slowly as requested, or if the peripheral's input clock
   * is outside what that mode allows: 2 MHz to 50 MHz, and at least 4 MHz for
   * fast mode (RM0008 section 26.3.3).
   */
  void configure(hal::i2c_settings const& p_settings, hal::hertz p_frequency)
  {
    auto const timing = compute_i2c_timing(p_settings.clock_rate, p_frequency);

    auto& regs = reg();
    // Reset clears a bus left busy by an interrupted transfer. CCR and TRISE
    // can only be written while the peripheral is disabled, which reset
    // leaves it.
    bit_modify(regs.cr1).set(i2c_cr1::software_reset);
    bit_modify(regs.cr1).clear(i2c_cr1::software_reset);

    // Transactions enable the interrupts and DMA requests they wait on.
    bit_modify(regs.cr2)
      .clear(i2c_cr2::buffer_intr_en)
      .clear(i2c_cr2::event_intr_en)
      .clear(i2c_cr2::error_intr_en)
      .clear(i2c_cr2::dma_en)
      .clear(i2c_cr2::last_dma)
      .insert<i2c_cr2::apb_frequency>(timing.frequency);
    bit_modify(regs.ccr)
      .insert<i2c_ccr::i2c_mode_sel>(timing.fast_mode)
      .clear(i2c_ccr::fast_mode_duty)
      .insert<i2c_ccr::ccr>(timing.ccr);
    bit_modify(regs.trise).insert<i2c_trise::rise_time>(timing.rise_time);
    bit_modify(regs.cr1)
      .clear(i2c_cr1::smbus_mode)
      .set(i2c_cr1::peripheral_enable);
  }

  /**
   * @brief Wake a DMA transaction because the bus reported an error
   *
   * Call this from the peripheral's error interrupt. Only DMA transactions
   * enable it, while they sleep. The error flags stay set until the
   * transaction handles them, so this masks the interrupt before notifying
   * the waiting context.
   */
  void handle_error_interrupt() noexcept
  {
    bit_modify(reg().cr2).clear(i2c_cr2::error_intr_en);
    m_notifier.notify();
  }

  /**
   * @brief Perform a transaction as defined in `hal::i2c::transaction()`,
   * moving every byte by polling
   *
   * Spins on the peripheral's status flags until the transaction finishes.
   * Needs no interrupts and no waiter. If another controller wins
   * arbitration, waits for the bus to be free and starts over.
   *
   * @param p_address - 7-bit address of the target device
   * @param p_data_out - bytes to write, possibly spread across chunks
   * @param p_data_in - buffer for the bytes read, possibly spread across
   * chunks
   * @param p_clock - bounds how long each wait on the bus can take
   * @throws hal::no_such_device - the device didn't acknowledge its address
   * or a byte written to it.
   * @throws hal::io_error - a misplaced start or stop condition was detected
   * on the bus.
   * @throws hal::timed_out - the bus made no progress for
   * `stall_timeout_ms`, for example because a device held SCL low.
   */
  void transaction(hal::byte p_address,
                   mem::scatter_span<hal::byte const> p_data_out,
                   mem::scatter_span<hal::byte> p_data_in,
                   hal::steady_clock& p_clock)
  {
    run(p_address, p_data_out, p_data_in, p_clock, polled_data{ this });
  }

  /**
   * @brief Perform a transaction as defined in `hal::i2c::transaction()`,
   * moving the data bytes by DMA
   *
   * The start, address and stop steps, and the last three bytes of a read,
   * are polled: they take tens of microseconds. The bytes in between move by
   * DMA, one contiguous chunk at a time, and the calling context sleeps on
   * its waiter until each chunk is done, so a waiter provider must be
   * installed. The peripheral's error interrupt must be routed to
   * `handle_error_interrupt()`.
   *
   * @tparam Dma - the platform's DMA channel type
   * @param p_address - 7-bit address of the target device
   * @param p_data_out - bytes to write, possibly spread across chunks
   * @param p_data_in - buffer for the bytes read, possibly spread across
   * chunks
   * @param p_clock - bounds how long each wait on the bus can take
   * @param p_transmit - channel serving the peripheral's transmit DMA
   * request
   * @param p_receive - channel serving the peripheral's receive DMA request
   * @throws hal::no_such_device - the device didn't acknowledge its address
   * or a byte written to it.
   * @throws hal::io_error - a misplaced start or stop condition was detected
   * on the bus, or a DMA transfer failed.
   * @throws hal::timed_out - the bus made no progress for
   * `stall_timeout_ms`, for example because a device held SCL low.
   */
  template<byte_dma_channel Dma>
  void transaction(hal::byte p_address,
                   mem::scatter_span<hal::byte const> p_data_out,
                   mem::scatter_span<hal::byte> p_data_in,
                   hal::steady_clock& p_clock,
                   Dma& p_transmit,
                   Dma& p_receive)
  {
    dma_stop<Dma> const stop{ .bus = this,
                              .transmit = &p_transmit,
                              .receive = &p_receive };
    hal::notifier::scope operation(m_notifier, stop);
    run(p_address,
        p_data_out,
        p_data_in,
        p_clock,
        dma_data<Dma>{ .stop = stop, .operation = &operation });
  }

private:
  /// Thrown internally when another controller wins arbitration. It never
  /// leaves `transaction()`.
  struct arbitration_lost
  {};

  /// Moves the data bytes of a transaction by polling the data register.
  /// The transaction takes how the data bytes move as a template parameter,
  /// so the polled and DMA transactions share the rest of it.
  struct polled_data
  {
    i2c* bus;

    /// Clear ADDR, then send each byte when the data register is empty
    void write(mem::scatter_span<hal::byte const> p_data) const
    {
      bus->clear_addr();
      for (std::span<hal::byte const> chunk : p_data) {
        for (auto const byte : chunk) {
          bus->wait_for(i2c_sr1::tx_empty);
          bus->reg().data_reg = byte;
        }
      }
    }

    /// Clear ADDR, then take each byte when the data register is full
    void read(mem::scatter_span<hal::byte> p_data) const
    {
      bus->clear_addr();
      for (std::span<hal::byte> chunk : p_data) {
        for (auto& byte : chunk) {
          bus->wait_for(i2c_sr1::rx_not_empty);
          byte = bus->received();
        }
      }
    }

    /// Nothing to stop
    void stop() const noexcept
    {
    }
  };

  /// Stops the peripheral's DMA requests and error interrupt, and both
  /// channels, so nothing wakes a transaction that has ended
  template<class Dma>
  struct dma_stop
  {
    i2c* bus;
    Dma* transmit;
    Dma* receive;

    void operator()() const noexcept
    {
      bit_modify(bus->reg().cr2)
        .clear(i2c_cr2::error_intr_en)
        .clear(i2c_cr2::dma_en);
      transmit->stop();
      receive->stop();
    }
  };

  template<class Dma>
  using dma_scope = hal::notifier::scope<dma_stop<Dma>>;

  /// Moves the data bytes of a transaction by DMA
  template<class Dma>
  struct dma_data
  {
    dma_stop<Dma> stop;
    dma_scope<Dma>* operation;

    void write(mem::scatter_span<hal::byte const> p_data) const
    {
      stop.bus->move_by_dma(p_data, *stop.transmit, *operation);
    }

    void read(mem::scatter_span<hal::byte> p_data) const
    {
      stop.bus->move_by_dma(p_data, *stop.receive, *operation);
    }
  };

  static constexpr std::chrono::milliseconds stall_timeout{ stall_timeout_ms };

  /// Run attempts until one isn't lost to another controller
  template<class Data>
  void run(hal::byte p_address,
           mem::scatter_span<hal::byte const> p_data_out,
           mem::scatter_span<hal::byte> p_data_in,
           hal::steady_clock& p_clock,
           Data const& p_data)
  {
    using namespace mp_units::si::unit_symbols;
    // In kHz, so the tick count can't overflow for any realistic clock.
    auto const stall_cycles = p_clock.frequency().in(kHz, mp_units::truncated) *
                              (stall_timeout_ms * ms);
    m_clock = &p_clock;
    m_stall_ticks = stall_cycles.numerical_value_in(mp_units::one) + 1U;
    m_address = p_address;

    auto& regs = reg();
    try {
      while (true) {
        try {
          attempt(p_data_out, p_data_in, p_data);
          return;
        } catch (arbitration_lost const&) {
          // Another controller took the bus. Wait for it to finish, then
          // start over.
          p_data.stop();
          bit_modify(regs.sr1).clear(i2c_sr1::arbitration_lost);
          wait_until_clear(regs.sr2, i2c_sr2::bus_busy);
        }
      }
    } catch (...) {
      release_bus();
      throw;
    }
  }

  template<class Data>
  void attempt(mem::scatter_span<hal::byte const> p_data_out,
               mem::scatter_span<hal::byte> p_data_in,
               Data const& p_data)
  {
    auto& regs = reg();
    std::size_t const read_length = p_data_in.length();
    bit_modify(regs.cr1)
      .set(i2c_cr1::peripheral_enable)
      .set(i2c_cr1::ack_enable)
      .clear(i2c_cr1::pos)
      .clear(i2c_cr1::stop);

    // A transaction with nothing to read still addresses the device, as a
    // write with no data.
    if ((p_data_out.length() != 0) or (read_length == 0)) {
      send_address(hal::i2c_operation::write);
      if (p_data_out.length() == 0) {
        clear_addr();
      } else {
        p_data.write(p_data_out);
        wait_for(i2c_sr1::byte_transfered_finish);
      }

      if (read_length == 0) {
        bit_modify(regs.cr1).set(i2c_cr1::stop);
        wait_until_clear(regs.cr1, i2c_cr1::stop);
        return;
      }
    }

    // A repeated start when bytes were written first.
    send_address(hal::i2c_operation::read);
    read(p_data_in, p_data);
    wait_until_clear(regs.cr1, i2c_cr1::stop);
  }

  /// Generate a start condition and send the address byte
  void send_address(hal::i2c_operation p_operation)
  {
    auto& regs = reg();
    bit_modify(regs.cr1).set(i2c_cr1::start);
    wait_for(i2c_sr1::start);
    regs.data_reg = hal::to_8_bit_address(m_address, p_operation);
    wait_for(i2c_sr1::addr);
  }

  /// Receive the bytes after the address, following the master receiver
  /// sequences in the reference manual (RM0008 section 26.3.3).
  ///
  /// When receiving the last bytes, clearing ADDR, setting STOP and reading
  /// the data register must happen before the next byte finishes, or the
  /// peripheral clocks out an extra byte (STM32F10xxx errata, I2C section).
  /// Those steps run with interrupts masked, so one can't delay them.
  template<class Data>
  void read(mem::scatter_span<hal::byte> p_data_in, Data const& p_data)
  {
    auto& regs = reg();
    std::size_t const length = p_data_in.length();
    // The ending sequences read into here, then the bytes are copied into
    // the buffer, so the steps that must run back to back stay short.
    std::array<hal::byte, 3> ending{};
    auto const ending_length = std::min(length, ending.size());
    auto const body_length = length - ending_length;

    if (length == 1) {
      // NACK the only byte, and stop after it.
      bit_modify(regs.cr1).clear(i2c_cr1::ack_enable);
      {
        hal::cortex_m::interrupt_mask_guard const masked;
        clear_addr();
        bit_modify(regs.cr1).set(i2c_cr1::stop);
      }
      wait_for(i2c_sr1::rx_not_empty);
      ending[0] = received();
    } else if (length == 2) {
      // POS makes the cleared ACK apply to the second byte.
      bit_modify(regs.cr1).clear(i2c_cr1::ack_enable).set(i2c_cr1::pos);
      clear_addr();
      // Both bytes have arrived: the first in the data register and the
      // second in the shift register.
      wait_for(i2c_sr1::byte_transfered_finish);
      {
        hal::cortex_m::interrupt_mask_guard const masked;
        bit_modify(regs.cr1).set(i2c_cr1::stop);
        ending[0] = received();
      }
      ending[1] = received();
      bit_modify(regs.cr1).clear(i2c_cr1::pos);
    } else {
      auto body = p_data_in.sub_scatter_span({ .count = body_length });
      if (body_length == 0) {
        clear_addr();
      } else {
        p_data.read(body);
      }
      // Three bytes left: byte N-2 is in the data register and N-1 in the
      // shift register. NACK the last one and stop after it.
      wait_for(i2c_sr1::byte_transfered_finish);
      bit_modify(regs.cr1).clear(i2c_cr1::ack_enable);
      {
        hal::cortex_m::interrupt_mask_guard const masked;
        ending[0] = received();
        bit_modify(regs.cr1).set(i2c_cr1::stop);
        ending[1] = received();
      }
      wait_for(i2c_sr1::rx_not_empty);
      ending[2] = received();
    }

    // Copy the ending into the last bytes of the buffer.
    std::span<hal::byte const> remaining =
      std::span(ending).first(ending_length);
    auto tail = p_data_in.sub_scatter_span({ .offset = body_length });
    for (std::span<hal::byte> chunk : tail) {
      std::ranges::copy(remaining.first(chunk.size()), chunk.begin());
      remaining = remaining.subspan(chunk.size());
    }
  }

  /// Send, or receive, the bytes of `p_data` by DMA, starting with ADDR
  /// still set. Writes send `hal::byte const` data and reads receive into
  /// `hal::byte` data.
  template<class Dma, class T>
  void move_by_dma(mem::scatter_span<T> p_data,
                   Dma& p_dma,
                   dma_scope<Dma>& p_operation)
  {
    auto& regs = reg();
    bool addr_cleared = false;
    // The first transfer is ready before ADDR is cleared, so it moves as soon
    // as the peripheral is. Between transfers, the peripheral holds SCL low
    // until the next one starts: a write waits for its next byte, and a read
    // keeps its received byte in the data register.
    for (std::span<T> chunk : p_data) {
      while (not chunk.empty()) {
        // One DMA transfer moves at most `max_dma_transfer` bytes.
        auto const run = chunk.first(std::min(chunk.size(), max_dma_transfer));
        if constexpr (std::is_const_v<T>) {
          p_dma.transmit(regs.data_reg, run, m_notifier);
        } else {
          p_dma.receive(regs.data_reg, run, m_notifier);
        }
        if (not addr_cleared) {
          bit_modify(regs.cr2).set(i2c_cr2::dma_en);
          clear_addr();
          addr_cleared = true;
        }
        wait_for_transfer(p_dma, p_operation);
        chunk = chunk.subspan(run.size());
      }
    }
    bit_modify(regs.cr2).clear(i2c_cr2::dma_en);
  }

  /**
   * @brief Sleep until a DMA transfer finishes
   *
   * The error interrupt wakes the context if the device stops acknowledging
   * or the bus fails, which stalls the transfer.
   *
   * @throws hal::io_error - the DMA transfer failed
   * @throws hal::timed_out - the transfer moved no bytes within the stall
   * timeout
   * @throws see `check_errors()`
   */
  template<class Dma>
  void wait_for_transfer(Dma& p_dma, dma_scope<Dma>& p_operation)
  {
    auto& regs = reg();
    while (true) {
      check_errors();
      auto const left = p_dma.remaining();
      if (left == 0) {
        return;
      }
      if (p_dma.failed()) {
        throw hal::io_error(this);
      }
      // handle_error_interrupt() masks it again when it fires.
      bit_modify(regs.cr2).set(i2c_cr2::error_intr_en);
      if (not p_operation.block_for(stall_timeout) and
          (p_dma.remaining() == left)) {
        check_errors();
        throw hal::timed_out(this);
      }
    }
  }

  /**
   * @brief Throw if the peripheral reports an error
   *
   * @throws hal::no_such_device - acknowledge failure
   * @throws hal::io_error - bus error or overrun
   * @throws arbitration_lost - another controller won the bus
   */
  void check_errors()
  {
    auto& regs = reg();
    auto const status = regs.sr1;
    if (bit_extract<i2c_sr1::ack_failure>(status)) {
      bit_modify(regs.sr1).clear(i2c_sr1::ack_failure);
      bit_modify(regs.cr1).set(i2c_cr1::stop);
      throw hal::no_such_device(m_address, this);
    }
    if (bit_extract<i2c_sr1::bus_error>(status) or
        bit_extract<i2c_sr1::over_under_run>(status)) {
      bit_modify(regs.sr1)
        .clear(i2c_sr1::bus_error)
        .clear(i2c_sr1::over_under_run);
      throw hal::io_error(this);
    }
    if (bit_extract<i2c_sr1::arbitration_lost>(status)) {
      throw arbitration_lost{};
    }
  }

  /**
   * @brief Spin until a status register 1 flag is set
   *
   * @throws hal::timed_out - the flag wasn't set within the stall timeout
   * @throws see `check_errors()`
   */
  void wait_for(hal::bit_mask p_flag)
  {
    auto& regs = reg();
    auto const deadline = m_clock->uptime() + m_stall_ticks;
    while (not bit_extract(p_flag, regs.sr1)) {
      check_errors();
      if ((m_clock->uptime() > deadline) and
          not bit_extract(p_flag, regs.sr1)) {
        throw hal::timed_out(this);
      }
    }
  }

  /**
   * @brief Spin until a register bit clears
   *
   * @throws hal::timed_out - the bit didn't clear within the stall timeout
   * @throws see `check_errors()`
   */
  void wait_until_clear(hal::u32 const volatile& p_register,
                        hal::bit_mask p_bit)
  {
    auto const deadline = m_clock->uptime() + m_stall_ticks;
    while (bit_extract(p_bit, p_register)) {
      check_errors();
      if (m_clock->uptime() > deadline) {
        throw hal::timed_out(this);
      }
    }
  }

  /// @return the byte in the data register
  hal::byte received()
  {
    return static_cast<hal::byte>(reg().data_reg);
  }

  /// Clears ADDR, which reading SR1 and then SR2 does
  void clear_addr()
  {
    auto& regs = reg();
    [[maybe_unused]] auto const sr1 = regs.sr1;
    [[maybe_unused]] auto const sr2 = regs.sr2;
  }

  /// Stop driving the bus after a failed transaction
  void release_bus()
  {
    auto& regs = reg();
    if (bit_extract<i2c_sr2::master>(regs.sr2)) {
      bit_modify(regs.cr1).set(i2c_cr1::stop);
    }
    // Disabling the peripheral resets its state machine. The next
    // transaction enables it again.
    bit_modify(regs.cr1).clear(i2c_cr1::peripheral_enable);
  }

  [[nodiscard]] i2c_reg_t& reg()
  {
    return *static_cast<i2c_reg_t*>(m_i2c);
  }

  void* m_i2c;
  hal::notifier m_notifier;
  /// The clock, stall bound and address of the transaction in progress
  hal::steady_clock* m_clock = nullptr;
  hal::u64 m_stall_ticks = 0;
  hal::byte m_address = 0;
};
}  // namespace hal::stm32_generic
