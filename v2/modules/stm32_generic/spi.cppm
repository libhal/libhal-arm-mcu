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

#include <bit>

#include <algorithm>
#include <cstdint>
#include <span>

export module hal.arm_mcu.stm32_generic:spi;

import hal;
import hal.util;

import :scatter_cursor;

namespace hal::stm32_generic {
/// stm32 spi peripheral register map
struct spi_reg_t
{
  /// Offset: 0x000 Control Register 1 (R/W)
  u32 volatile cr1;
  /// Offset: 0x004 Control Register 2 (R/W)
  u32 volatile cr2;
  /// Offset: 0x008 Status Register (R/W)
  u32 volatile sr;
  /// Offset: 0x00C Data Register (R/W)
  u32 volatile dr;
  /// Offset: 0x010 CRC polynomial register (R/ )
  u32 const volatile crcpr;
  /// Offset: 0x014 RX CRC register (R/W)
  u32 volatile rxcrcr;
  /// Offset: 0x018 TX CRC Register (R/W)
  u32 volatile txcrcr;
  /// Offset: 0x01C configuration register (R/W)
  u32 volatile i2scfgr;
  /// Offset: 0x020 prescaler register (R/W)
  u32 volatile i2spr;
};

/// SPI Control Register 1
namespace control_register1 {
/// 0: first clock transistion is the first capture edge
/// 1: second clock transition is the first data capture edge
constexpr auto clock_phase = bit_mask::from<0>();

/// 0: clock to 0 when idle
/// 1: clock to 1 when idle
constexpr auto clock_polarity = bit_mask::from<1>();

/// 0: slave, 1: master
constexpr auto master_selection = bit_mask::from<2>();

/// baudrate control: sets the clock rate to:
/// (peripheral clock frequency)/2**(n+1)
constexpr auto baud_rate_control = bit_mask::from<5, 3>();

/// Peripheral Enable
/// 0: disable, 1: enable
constexpr auto enable = bit_mask::from<6>();

/// Frame Format
/// 0: msb transmitted first
/// 1: lsb tranmitted first
constexpr auto lsb_first = bit_mask::from<7>();

/// internal slave select
constexpr auto internal_slave_select = bit_mask::from<8>();

/// Software slave management
/// 0: disable, 1: enable
constexpr auto software_slave_management = bit_mask::from<9>();

/// Recieve only
/// 0: Full Duplex, 1: Output disable
constexpr auto rx_only = bit_mask::from<10>();

/// Data frame format
/// 0: 8-bits, 1: 16-bit
constexpr auto data_frame_format = bit_mask::from<11>();

/// CRC transfer next
/// 0: No CRC phase, 1: transfer CRC next
constexpr auto crc_transfer_next = bit_mask::from<12>();

/// CRC enable
/// 0: disable, 1: enable
constexpr auto crc_enable = bit_mask::from<13>();

/// Output enable in bidirectional mode
/// 0: output disabled, 1: output enable
constexpr auto bidirectional_output_enable = bit_mask::from<14>();

/// Bidirectional data mode enable
/// 0: full-duplex, 1: half-duplex
constexpr auto bidirectional_mode_enable = bit_mask::from<15>();
}  // namespace control_register1

/// SPI Control Register 2
namespace control_register2 {
/// Rx buffer DMA enable
constexpr auto rx_dma_enable = bit_mask::from<0>();

/// Tx buffer DMA enable
constexpr auto tx_dma_enable = bit_mask::from<1>();

/// Slave select output enable
/// 0: use a GPIO, 1: use the NSS pin
constexpr auto slave_select_output_enable = bit_mask::from<2>();

/// Frame format
/// 0: Motorola mode, 1: TI mode
constexpr auto frame_format = bit_mask::from<4>();

/// Error interupt enable
constexpr auto error_interrupt_enable = bit_mask::from<5>();

/// Rx buffer empty interrupt enable
constexpr auto rx_buffer_empty_interrupt_enable = bit_mask::from<6>();

/// Tx buffer empty interrupt enable
constexpr auto tx_buffer_empty_interrupt_enable = bit_mask::from<7>();
}  // namespace control_register2

/// SPI Status Register
namespace status_register {
/// Recieve buffer not empty
constexpr auto rx_buffer_not_empty = bit_mask::from<0>();

/// Transmit buffer not empty
constexpr auto tx_buffer_empty = bit_mask::from<1>();

/// Busy flag
constexpr auto busy_flag = bit_mask::from<7>();
}  // namespace status_register

bool busy(spi_reg_t& p_reg)
{
  return bit_extract<status_register::busy_flag>(p_reg.sr);
}

bool tx_empty(spi_reg_t& p_reg)
{
  return bit_extract<status_register::tx_buffer_empty>(p_reg.sr);
}

bool rx_not_empty(spi_reg_t& p_reg)
{
  return bit_extract<status_register::rx_buffer_not_empty>(p_reg.sr);
}

/**
 * @brief Select the CR1 baud rate control (prescaler) field that gets a spi
 * bus running at `p_peripheral_clock_speed` as close as possible to, but not
 * above, `p_requested_clock_rate`.
 *
 * A request outside the bus's range selects the nearest rate: faster than
 * the bus can go selects the peripheral clock divided by 2, and slower than
 * it can go (including zero) selects the peripheral clock divided by 256.
 */
std::uint16_t select_baud_rate_control(hal::hertz p_peripheral_clock_speed,
                                       hal::hertz p_requested_clock_rate)
{
  auto const peripheral_clock = static_cast<std::uint32_t>(
    p_peripheral_clock_speed.numerical_value_in(hal::hertz::unit));
  auto const requested = static_cast<std::uint32_t>(
    p_requested_clock_rate.numerical_value_in(hal::hertz::unit));

  constexpr std::uint32_t min_prescaler = 2;
  constexpr std::uint32_t max_prescaler = 256;

  // Zero asks for the slowest rate. Treating it as 1 Hz gets there, since the
  // divider is then clamped to the largest prescaler below.
  auto const nonzero_request = std::max(requested, 1U);
  auto const whole_divider = peripheral_clock / nonzero_request;
  bool const has_remainder = (peripheral_clock % nonzero_request) != 0;
  // Round the divider up, so the bus never runs faster than requested when it
  // can avoid it.
  auto const divider = has_remainder ? whole_divider + 1U : whole_divider;

  // The prescaler is a power of two from 2 (field value 0) to 256 (field
  // value 7). Pick the smallest one that is at least the divider.
  auto const prescaler = std::clamp(divider, min_prescaler, max_prescaler);
  return static_cast<std::uint16_t>(std::bit_width(prescaler - 1U) - 1);
}

/**
 * @brief Compute the spi bus clock rate that `configure()` would achieve for
 * a given peripheral clock and requested clock rate, without touching any
 * hardware state.
 *
 * This is the exact rate, and never rejects a request: one outside the
 * bus's range gets the nearest rate it can do. A result greater than
 * `p_requested_clock_rate` means the bus can't go that slowly, and
 * `configure()` would throw. This is meant for platform specific spi drivers
 * that need to answer `clock_rate()` for a channel that may not currently be
 * the one selected on the bus, and therefore cannot be answered by simply
 * reading back live hardware registers.
 */
export hal::hertz achievable_clock_rate(hal::hertz p_peripheral_clock_speed,
                                        hal::hertz p_requested_clock_rate)
{
  auto const baud_control =
    select_baud_rate_control(p_peripheral_clock_speed, p_requested_clock_rate);
  auto const divider = 1U << (baud_control + 1U);
  auto const peripheral_clock_speed_value = static_cast<std::uint32_t>(
    p_peripheral_clock_speed.numerical_value_in(hal::hertz::unit));

  return (peripheral_clock_speed_value / divider) * hal::hertz::unit;
}

/**
 * @brief A generic spi implementation for all stm32f series MCUs
 *
 * This class is meant to only be used by platform libraries or drivers
 * aiming to be platform libraries for stm32f devices. As an application
 * develop, prefer to use the platform specific drivers instead as they
 * handle all of the initialization for you.
 *
 * This driver is synchronous/blocking. It does not participate in the
 * async::context/coroutine model.
 */
export class spi
{
public:
  /**
   * @brief Construct a new spi object
   *
   * Care must be taken when constructing this object. The constructor does
   * not and cannot power on the spi peripheral. Accessing the peripheral's
   * registers without it be powered on (or provided a clock), will result
   * in a memory fault as the peripheral cannot ACK the CPU when it attempts
   * to access the peripheral's registers. The power management system for
   * each peripheral across the stm32 series of MCUs is different, meaning
   * its not something that can efficiently be manged by this generic
   * driver.
   *
   * After constructing this object, in order to use this driver the
   * program must:
   *
   * 1. Configure pins to be controlled by the spi peripheral pointed to by
   *    the `p_peripheral_address` input parameter.
   * 2. Power on the appropriate spi peripheral
   * 3. Execute the `configure()` function with the appropriate peripheral
   *    frequency.
   *
   * Performing these in this order will properly initialize the spi driver
   * and allow the usage of the `transfer()` API. Failing to execute these
   * steps in this order will result in UB.
   *
   * It is unsafe to use this driver before performing the steps above.
   *
   * @param p_peripheral_address - starting address of the spi peripheral
   */
  spi(void* p_peripheral_address)
    : m_peripheral_address(p_peripheral_address)
  {
  }

  spi(spi& p_other) = delete;
  spi& operator=(spi& p_other) = delete;
  spi(spi&& p_other) noexcept = delete;
  spi& operator=(spi&& p_other) noexcept = delete;

  /**
   * @brief Leaves the peripheral as it is
   *
   * Like the constructor, the destructor doesn't touch the registers, since
   * the peripheral may already be powered off. The platform specific driver
   * disables the peripheral, for example by resetting it, before powering it
   * off.
   */
  ~spi() = default;

  /**
   * @brief Configure the SPI peripheral
   *
   * Because each stm32f series MCU has its own unique clock tree, there is
   * no single, simple and space efficient may to determine the system's
   * spi clock speed. So the caller must supply the operating frequency of
   * the spi peripheral for this to work. This API is meant to be called by
   * platform specific spi drivers that have the necessary context and
   * library apis to retrieve the spi peripheral's clock rate.
   *
   * @param p_settings - spi settings
   * @param p_peripheral_clock_speed - peripheral's operating clock rate
   * @throws hal::operation_not_supported - if the bus can't run as slowly as
   * the requested clock rate. A faster request runs at the bus's fastest
   * rate.
   */
  void configure(hal::spi_settings const& p_settings,
                 hal::hertz p_peripheral_clock_speed)
  {
    auto& regs = reg();

    // Running a device faster than it asked for isn't safe.
    if (achievable_clock_rate(p_peripheral_clock_speed, p_settings.clock_rate) >
        p_settings.clock_rate) {
      throw hal::operation_not_supported(this);
    }

    auto const baud_control =
      select_baud_rate_control(p_peripheral_clock_speed, p_settings.clock_rate);

    // spi mode determines clock polarity (CPOL) and clock phase (CPHA):
    //
    //   m0: CPOL 0, CPHA 0        m2: CPOL 1, CPHA 0
    //   m1: CPOL 0, CPHA 1        m3: CPOL 1, CPHA 1
    bool const clock_polarity = p_settings.bus_mode == hal::spi_mode::m2 ||
                                p_settings.bus_mode == hal::spi_mode::m3;
    bool const clock_phase = p_settings.bus_mode == hal::spi_mode::m1 ||
                             p_settings.bus_mode == hal::spi_mode::m3;

    bit_modify(regs.cr2)
      .clear<control_register2::rx_dma_enable>()
      .clear<control_register2::tx_dma_enable>()
      // We set `slave_select_output_enable` because it is required for
      // master mode to work.
      .set<control_register2::slave_select_output_enable>()
      .clear<control_register2::frame_format>()
      .clear<control_register2::error_interrupt_enable>()
      .clear<control_register2::rx_buffer_empty_interrupt_enable>()
      .clear<control_register2::tx_buffer_empty_interrupt_enable>();

    bit_modify(regs.cr1)
      .clear<control_register1::rx_only>()
      .clear<control_register1::crc_transfer_next>()
      .clear<control_register1::data_frame_format>()
      .clear<control_register1::crc_enable>()
      .clear<control_register1::lsb_first>()
      .clear<control_register1::bidirectional_mode_enable>()
      .clear<control_register1::bidirectional_output_enable>()
      .insert<control_register1::baud_rate_control>(baud_control)
      .insert<control_register1::clock_phase>(clock_phase)
      .insert<control_register1::clock_polarity>(clock_polarity)
      .set<control_register1::internal_slave_select>()  // same as disabled
      .set<control_register1::software_slave_management>()
      .set<control_register1::master_selection>()
      .set<control_register1::enable>();
  }

  /**
   * @brief Perform a transfer operation as defined in `hal::raw_spi_bus`
   *
   * This call blocks until the transfer has completed.
   *
   * @param p_data_out - outgoing data, possibly spread across multiple
   * discontiguous chunks
   * @param p_data_in - incoming data, possibly spread across multiple
   * discontiguous chunks
   * @param p_filler - output filler bytes if the outgoing data runs out
   * before the incoming data.
   */
  void transfer(mem::scatter_span<hal::byte const> p_data_out,
                mem::scatter_span<hal::byte> p_data_in,
                hal::byte p_filler)
  {
    auto& regs = reg();
    std::size_t const max_length =
      std::max(p_data_in.length(), p_data_out.length());

    // NOTE: This is a paranoid check to determine that there is no bus
    // activity before proceeding
    while (busy(regs)) {
      continue;
    }

    // The stm's spi driver needs to be internally told that it is
    // selecting a device before it will emit anything on the pins. This
    // will control the NSS pin if it is selected, otherwise, its just an
    // internal enable signal.
    bit_modify(regs.cr1).clear<control_register1::internal_slave_select>();

    scatter_cursor<hal::byte const> out_cursor(p_data_out);
    scatter_cursor<hal::byte> in_cursor(p_data_in);

    for (std::size_t index = 0; index < max_length; index++) {
      hal::byte byte = p_filler;

      if (not out_cursor.done()) {
        byte = out_cursor.current();
        out_cursor.advance();
      }

      while (not tx_empty(regs)) {
        continue;
      }

      regs.dr = byte;

      while (not rx_not_empty(regs)) {
        continue;
      }

      byte = static_cast<std::uint8_t>(regs.dr);
      if (not in_cursor.done()) {
        in_cursor.current() = byte;
        in_cursor.advance();
      }
    }

    bit_modify(regs.cr1).set<control_register1::internal_slave_select>();

    // Wait for bus activity to cease before leaving the function
    while (busy(regs)) {
      continue;
    }
  }

  /**
   * @brief Perform a transfer operation over contiguous buffers
   *
   * Convenience overload of `transfer(scatter_span<byte const>,
   * scatter_span<byte>, byte)` for callers that already have contiguous
   * `std::span` buffers.
   *
   * @param p_data_out - outgoing data
   * @param p_data_in - incoming data
   * @param p_filler - output filler bytes if the outgoing data runs out
   * before the incoming data.
   */
  void transfer(std::span<hal::byte const> p_data_out,
                std::span<hal::byte> p_data_in,
                hal::byte p_filler)
  {
    transfer(mem::scatter_span<hal::byte const>({ p_data_out }),
             mem::scatter_span<hal::byte>({ p_data_in }),
             p_filler);
  }

private:
  [[nodiscard]] spi_reg_t& reg()
  {
    return *reinterpret_cast<spi_reg_t*>(m_peripheral_address);
  }

  void* m_peripheral_address;
};
}  // namespace hal::stm32_generic
