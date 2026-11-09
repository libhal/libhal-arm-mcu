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
#include <limits>
#include <span>

export module hal.arm_mcu.stm32_generic:i2c;

import hal;
import hal.util;
import hal.arm_mcu.cortex_m;

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

/// How long a transaction waits for the bus to make progress, for example
/// while a device stretches the clock, before throwing `hal::timed_out`,
/// unless the application chooses another stall timeout. Matches the SMBus
/// clock-low timeout. A device that stretches the clock for longer needs a
/// longer one. The Sensirion SHT21, for example, can hold SCL low while it
/// measures temperature, which takes up to 85 ms.
export constexpr hal::time_duration default_i2c_stall_timeout =
  std::chrono::milliseconds(25);

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
 * @brief Compute how many polls of the peripheral take at least a stall
 * timeout, without touching any hardware
 *
 * A poll reads a peripheral register, and every APB read takes at least two
 * input clock cycles: the protocol's setup and access phases. The count is
 * rounded up, and polls that take longer only make the wait longer, so the
 * polls never take less than `p_stall_timeout`.
 *
 * @param p_stall_timeout - how long a wait must last before giving up
 * @param p_frequency - the peripheral's input clock (PCLK1 on the STM32F1)
 * @return u32 - the number of polls
 * @throws hal::operation_not_supported - if the stall timeout or the input
 * clock is zero or negative, or the stall timeout takes more polls than 32
 * bits can count: over 171 seconds at 50 MHz.
 */
export u32 compute_stall_polls(hal::time_duration p_stall_timeout,
                               hal::hertz p_frequency)
{
  using namespace mp_units::si::unit_symbols;
  constexpr u64 cycles_per_poll = 2;
  constexpr u64 ns_per_second = 1'000'000'000;
  // polls = timeout (ns) * frequency (Hz) / (2 cycles * 1e9 ns per second),
  // rounded up, in integers
  constexpr u64 ns_hz_per_poll = cycles_per_poll * ns_per_second;
  constexpr u64 max_polls = std::numeric_limits<u32>::max();

  u64 const frequency = p_frequency.numerical_value_in(Hz);
  if ((p_stall_timeout <= hal::time_duration::zero()) or (frequency == 0)) {
    throw hal::operation_not_supported(nullptr);
  }

  // Checking against the longest countable timeout first also keeps the
  // product below from overflowing.
  auto const timeout = static_cast<u64>(p_stall_timeout.count());
  if (timeout > ((max_polls * ns_hz_per_poll) / frequency)) {
    throw hal::operation_not_supported(nullptr);
  }

  return static_cast<u32>(((timeout * frequency) + (ns_hz_per_poll - 1)) /
                          ns_hz_per_poll);
}

/**
 * @brief A generic i2c implementation for all stm32 series MCUs
 *
 * This class is meant to only be used by platform libraries or drivers
 * aiming to be platform libraries for stm32 devices. Application developers
 * should use the driver specific to their platform instead of this class as
 * they handle proper initialization for you.
 *
 * A transaction polls the peripheral's status flags, so it needs no
 * interrupts, no DMA, no waiter and no clock. Each wait gives up after a
 * number of polls that takes at least the stall timeout given to the
 * constructor.
 *
 * Only master mode with 7-bit addressing is supported. After an address of
 * `0b111'10XX`, the peripheral waits for a second address byte (RM0008 section
 * 26.3.3), which this driver doesn't send, so a device that acknowledges such
 * an address causes `hal::timed_out`.
 */
export class i2c
{
public:
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
   * Accessing the registers of a peripheral that isn't powered faults.
   *
   * @param p_i2c - i2c peripheral address
   * @param p_stall_timeout - how long a transaction waits for the bus to make
   * progress before throwing `hal::timed_out`. `configure()` checks it.
   */
  explicit i2c(void* p_i2c,
               hal::time_duration p_stall_timeout = default_i2c_stall_timeout)
    : m_i2c(p_i2c)
    , m_stall_timeout(p_stall_timeout)
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
   * fast mode (RM0008 section 26.3.3). Also if the stall timeout given to the
   * constructor can't be counted (see `compute_stall_polls()`).
   */
  void configure(hal::i2c_settings const& p_settings, hal::hertz p_frequency)
  {
    auto const timing = compute_i2c_timing(p_settings.clock_rate, p_frequency);
    auto const stall_polls = compute_stall_polls(m_stall_timeout, p_frequency);

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
    m_stall_polls = stall_polls;
    bit_modify(regs.cr1)
      .clear(i2c_cr1::smbus_mode)
      .set(i2c_cr1::peripheral_enable);
  }

  /**
   * @brief Perform a transaction as defined in `hal::i2c::transaction()`,
   * moving every byte by polling
   *
   * Spins on the peripheral's status flags until the transaction finishes.
   * Needs no interrupts and no waiter. It never waits for a busy bus and never
   * retries: the caller decides what to do about a bus it can't have.
   *
   * If both `p_data_out` and `p_data_in` are empty, this returns without
   * touching the peripheral.
   *
   * @param p_address - 7-bit address of the target device
   * @param p_data_out - bytes to write, possibly spread across chunks
   * @param p_data_in - buffer for the bytes read, possibly spread across
   * chunks
   * @throws hal::no_such_device - the device didn't acknowledge its address.
   * @throws hal::broken_pipe - the device acknowledged its address, then didn't
   * acknowledge a byte of `p_data_out` or the read address of a
   * write-then-read. Its `progress` is the number of bytes of `p_data_out` the
   * device acknowledged.
   * @throws hal::io_error - a misplaced start or stop condition was detected
   * on the bus.
   * @throws hal::timed_out - the bus made no progress for at least the stall
   * timeout after the start condition went out, for example because a device
   * held SCL low. Its `progress` is the number of bytes known to have been
   * transferred, written ones first, then read ones.
   * @throws hal::resource_unavailable_try_again - the bus was busy when this
   * call started, the start condition didn't go out within the stall timeout,
   * or another controller won arbitration.
   */
  void transaction(hal::byte p_address,
                   mem::scatter_span<hal::byte const> p_data_out,
                   mem::scatter_span<hal::byte> p_data_in)
  {
    if ((p_data_out.length() == 0) and (p_data_in.length() == 0)) {
      return;
    }
    run(p_address, p_data_out, p_data_in, polled_data{ this });
  }

private:
  /// Moves the data bytes of a transaction by polling the data register.
  /// The transaction takes how the data bytes move as a template parameter,
  /// so another way, such as DMA, can reuse the rest of it. Each way adds
  /// the bytes it moves to `m_progress`.
  struct polled_data
  {
    i2c* bus;

    /// Clear ADDR, then send each byte when the data register is empty
    void write(mem::scatter_span<hal::byte const> p_data) const
    {
      bus->clear_addr();
      for (auto const byte : p_data) {
        bus->wait_for(i2c_sr1::tx_empty);
        bus->reg().data_reg = byte;
        bus->m_progress++;
      }
    }

    /// Clear ADDR, then take each byte when the data register is full
    void read(mem::scatter_span<hal::byte> p_data) const
    {
      bus->clear_addr();
      for (auto& byte : p_data) {
        bus->wait_for(i2c_sr1::rx_not_empty);
        byte = static_cast<hal::byte>(bus->reg().data_reg);
        bus->m_progress++;
      }
    }
  };

  /// Run a transaction, and release the bus if it fails
  template<class Data>
  void run(hal::byte p_address,
           mem::scatter_span<hal::byte const> p_data_out,
           mem::scatter_span<hal::byte> p_data_in,
           Data const& p_data)
  {
    auto& regs = reg();
    // Left alone, the peripheral would wait for the bus to be free. Whether to
    // wait is the caller's decision.
    if (bit_extract<i2c_sr2::bus_busy>(regs.sr2)) {
      throw hal::resource_unavailable_try_again(this);
    }

    m_address = p_address;
    m_progress = 0;
    m_in_flight = 0;

    try {
      std::size_t const read_length = p_data_in.length();
      bit_modify(regs.cr1)
        .set(i2c_cr1::peripheral_enable)
        .set(i2c_cr1::ack_enable)
        .clear(i2c_cr1::pos)
        .clear(i2c_cr1::stop);

      if (p_data_out.length() != 0) {
        send_address(hal::i2c_operation::write);
        // Until BTF, the last two bytes written, in the data and shift
        // registers, may not have reached the device.
        m_in_flight = 2;
        p_data.write(p_data_out);
        wait_for(i2c_sr1::byte_transfered_finish);
        m_in_flight = 0;

        if (read_length == 0) {
          bit_modify(regs.cr1).set(i2c_cr1::stop);
          wait_until_clear(regs.cr1, i2c_cr1::stop);
          return;
        }
      }

      // A repeated start when bytes were written first.
      send_address(hal::i2c_operation::read);
      read(p_data_in, p_data);
      m_progress = static_cast<hal::u32>(p_data_out.length() + read_length);
      wait_until_clear(regs.cr1, i2c_cr1::stop);
    } catch (...) {
      // Wait for the stop condition to go out, so that the next transaction
      // doesn't find the bus busy because of this one. CR1 must not be
      // written until then either, and PE must not be cleared before the end
      // of the communication (RM0008 section 26.6.1). A stuck bus can't send
      // a stop condition, so the wait gives up after the stall bound.
      if (bit_extract<i2c_sr2::master>(regs.sr2) and
          not bit_extract<i2c_cr1::stop>(regs.cr1)) {
        bit_modify(regs.cr1).set(i2c_cr1::stop);
      }
      auto polls = m_stall_polls;
      while (bit_extract<i2c_cr1::stop>(regs.cr1) and (polls != 0)) {
        polls--;
      }
      // Disabling the peripheral resets its state machine, and clearing START
      // drops a start condition still waiting for the bus. The next
      // transaction enables the peripheral again.
      bit_modify(regs.cr1)
        .clear(i2c_cr1::start)
        .clear(i2c_cr1::peripheral_enable);
      throw;
    }
  }

  /**
   * @brief Generate a start condition and send the address byte
   *
   * The peripheral sends the transaction's first start condition once the bus
   * is free, so until then the bus belongs to something else. A repeated start
   * comes after bytes this transaction sent, while it holds the bus.
   *
   * @throws hal::resource_unavailable_try_again - the first start condition
   * didn't go out within the stall bound
   * @throws hal::timed_out - a repeated start, or the address, didn't go out
   * within the stall bound
   * @throws see `check_errors()`
   */
  void send_address(hal::i2c_operation p_operation)
  {
    auto& regs = reg();
    bit_modify(regs.cr1).set(i2c_cr1::start);
    auto polls = m_stall_polls;
    while (not bit_extract<i2c_sr1::start>(regs.sr1)) {
      check_errors();
      if ((polls == 0) and (m_progress == 0)) {
        throw hal::resource_unavailable_try_again(this);
      }
      spend_poll(polls);
    }
    regs.data_reg = hal::to_8_bit_address(m_address, p_operation);
    wait_for(i2c_sr1::addr);
  }

  /// Receive the bytes after the address, following the master receiver
  /// sequences in the reference manual (RM0008 section 26.3.3).
  ///
  /// The last steps of each ending run with interrupts masked. The comment
  /// on the first masked block explains why.
  template<class Data>
  void read(mem::scatter_span<hal::byte> p_data_in, Data const& p_data)
  {
    auto& regs = reg();
    std::size_t const length = p_data_in.length();
    // Where each of the last bytes goes, found before the ending sequences
    // so that the steps that must run back to back store through a pointer
    // instead of walking the buffer.
    std::array<hal::byte*, 3> ending{};
    auto const ending_length = std::min(length, ending.size());
    auto const body_length = length - ending_length;
    auto destination = ending.begin();
    for (auto& byte : p_data_in.sub_scatter_span({ .offset = body_length })) {
      *destination++ = &byte;
    }

    if (length == 1) {
      // NACK the only byte, and stop after it.
      bit_modify(regs.cr1).clear(i2c_cr1::ack_enable);
      // The steps in each masked block must finish before the byte on the
      // bus does, or the peripheral may receive an extra byte, read the same
      // byte twice or corrupt the last one. Polling is fast enough, but an
      // interrupt between the steps could delay them past that point, so
      // interrupts are masked for these few register accesses. The STM32F1
      // errata require this. In ES0340 Rev 17 (STM32F103xC/D/E), section
      // 2.9.1 on page 19 sets the deadline, and workaround 2 of section 2.9.2,
      // on pages 19 and 20, masks interrupts for the 2-byte and longer
      // endings.
      {
        hal::cortex_m::interrupt_mask_guard const masked;
        clear_addr();
        bit_modify(regs.cr1).set(i2c_cr1::stop);
      }
      wait_for(i2c_sr1::rx_not_empty);
      *ending[0] = static_cast<hal::byte>(regs.data_reg);
    } else if (length == 2) {
      // POS makes the cleared ACK apply to the second byte.
      bit_modify(regs.cr1).clear(i2c_cr1::ack_enable).set(i2c_cr1::pos);
      clear_addr();
      // Both bytes have arrived: the first in the data register and the
      // second in the shift register.
      wait_for(i2c_sr1::byte_transfered_finish);
      // Masked so an interrupt can't delay reading the first byte until the
      // STOP is on the bus, which corrupts the second byte (ES0340 Rev 17,
      // section 2.9.2, workaround 2, sequence 1). See the comment above the
      // first interrupt_mask_guard in read(), in the 1-byte case.
      {
        hal::cortex_m::interrupt_mask_guard const masked;
        bit_modify(regs.cr1).set(i2c_cr1::stop);
        *ending[0] = static_cast<hal::byte>(regs.data_reg);
      }
      *ending[1] = static_cast<hal::byte>(regs.data_reg);
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
      // Masked so an interrupt can't delay reading bytes N-2 and N-1 until
      // the STOP is on the bus, which corrupts byte N (ES0340 Rev 17, section
      // 2.9.2, workaround 2, sequence 2). See the comment above the first
      // interrupt_mask_guard in read(), in the 1-byte case.
      {
        hal::cortex_m::interrupt_mask_guard const masked;
        *ending[0] = static_cast<hal::byte>(regs.data_reg);
        bit_modify(regs.cr1).set(i2c_cr1::stop);
        *ending[1] = static_cast<hal::byte>(regs.data_reg);
      }
      m_progress += 2;
      wait_for(i2c_sr1::rx_not_empty);
      *ending[2] = static_cast<hal::byte>(regs.data_reg);
    }
  }

  /**
   * @brief Throw if the peripheral reports an error
   *
   * A device refuses a byte once it is in the shift register. That byte, and
   * the next one if it is waiting in the data register, have moved but
   * weren't acknowledged. TxE tells the two cases apart, since it isn't set
   * after a NACK (RM0008 section 26.6.6). It is read before the stop
   * condition, which clears it.
   *
   * @throws hal::no_such_device - a NACK before the device answered: nothing
   * has moved yet
   * @throws hal::broken_pipe - a NACK after the device answered, on a byte or
   * the read address of a write-then-read
   * @throws hal::io_error - bus error or overrun
   * @throws hal::resource_unavailable_try_again - another controller won the
   * bus
   */
  void check_errors()
  {
    auto& regs = reg();
    auto const status = regs.sr1;
    if (bit_extract<i2c_sr1::ack_failure>(status)) {
      bit_modify(regs.sr1).clear(i2c_sr1::ack_failure);
      bit_modify(regs.cr1).set(i2c_cr1::stop);
      if (m_progress == 0) {
        throw hal::no_such_device(m_address, this);
      }
      hal::u32 unacknowledged = 0;
      if (m_in_flight != 0) {
        unacknowledged = bit_extract<i2c_sr1::tx_empty>(status) ? 1U : 2U;
      }
      hal::u32 const acknowledged =
        (m_progress > unacknowledged) ? (m_progress - unacknowledged) : 0U;
      throw hal::broken_pipe(this, acknowledged);
    }
    if (bit_extract<i2c_sr1::bus_error>(status) or
        bit_extract<i2c_sr1::over_under_run>(status)) {
      bit_modify(regs.sr1)
        .clear(i2c_sr1::bus_error)
        .clear(i2c_sr1::over_under_run);
      throw hal::io_error(this);
    }
    if (bit_extract<i2c_sr1::arbitration_lost>(status)) {
      bit_modify(regs.sr1).clear(i2c_sr1::arbitration_lost);
      throw hal::resource_unavailable_try_again(this);
    }
  }

  /**
   * @brief Spin until a status register 1 flag is set
   *
   * @throws hal::timed_out - the flag wasn't set within the stall bound
   * @throws see `check_errors()`
   */
  void wait_for(hal::bit_mask p_flag)
  {
    auto& regs = reg();
    auto polls = m_stall_polls;
    while (not bit_extract(p_flag, regs.sr1)) {
      check_errors();
      spend_poll(polls);
    }
  }

  /**
   * @brief Spin until a register bit clears
   *
   * @throws hal::timed_out - the bit didn't clear within the stall bound
   * @throws see `check_errors()`
   */
  void wait_until_clear(hal::u32 const volatile& p_register,
                        hal::bit_mask p_bit)
  {
    auto polls = m_stall_polls;
    while (bit_extract(p_bit, p_register)) {
      check_errors();
      spend_poll(polls);
    }
  }

  /**
   * @brief Count one poll against a wait's stall bound
   *
   * @param p_polls - the wait's polls left
   * @throws hal::timed_out - the bound is spent
   */
  void spend_poll(hal::u32& p_polls)
  {
    if (p_polls == 0) {
      throw hal::timed_out(this, reported_progress());
    }
    p_polls--;
  }

  /// @return the bytes moved so far, less those that may not have reached
  /// the device yet
  [[nodiscard]] hal::u32 reported_progress() const
  {
    return (m_progress > m_in_flight) ? (m_progress - m_in_flight) : 0;
  }

  /// Clears ADDR, which reading SR1 and then SR2 does
  void clear_addr()
  {
    auto& regs = reg();
    [[maybe_unused]] auto const sr1 = regs.sr1;
    [[maybe_unused]] auto const sr2 = regs.sr2;
  }

  [[nodiscard]] i2c_reg_t& reg()
  {
    return *static_cast<i2c_reg_t*>(m_i2c);
  }

  void* m_i2c;
  /// How long a wait lasts before giving up
  hal::time_duration m_stall_timeout;
  /// Polls each wait makes before giving up, set by `configure()`
  hal::u32 m_stall_polls = 0;
  /// Bytes of the transaction in progress moved through the data register so
  /// far, written ones first, then read ones
  hal::u32 m_progress = 0;
  /// Address of the transaction in progress
  hal::byte m_address = 0;
  /// How many of the bytes in `m_progress` may not have reached the device
  /// yet
  hal::u8 m_in_flight = 0;
};
}  // namespace hal::stm32_generic
