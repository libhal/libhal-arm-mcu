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

#include <cmath>

#include <cstdint>
#include <span>

export module hal.arm_mcu.stm32_generic:uart;

import hal;
import hal.util;

namespace hal::stm32_generic {
/// Namespace for the status registers (SR) bit masks
namespace status_reg {
/// Indicates if the transmit data register is empty and can be loaded with
/// another byte.
constexpr auto transit_empty = hal::bit_mask::from<7>();
}  // namespace status_reg

/// Namespace for the control registers (CR1, CR3) bit masks and predefined
/// settings constants.
namespace control_reg {
/// When this bit is cleared the USART prescalers and outputs are stopped
/// and the end of the current byte transfer in order to reduce power
/// consumption. (CR1)
constexpr auto usart_enable = hal::bit_mask::from<13>();

/// Enables DMA receiver (CR3)
constexpr auto dma_receiver_enable = hal::bit_mask::from<6>();

/// This bit enables the transmitter. (CR1)
constexpr auto transmitter_enable = hal::bit_mask::from<3>();

/// This bit enables the receiver. (CR1)
constexpr auto receive_enable = hal::bit_mask::from<2>();

/// Enable USART + Enable Receive + Enable Transmitter
constexpr auto control_settings1 = hal::bit_value(0UL)
                                     .set<control_reg::usart_enable>()
                                     .set<control_reg::receive_enable>()
                                     .set<control_reg::transmitter_enable>()
                                     .to<std::uint16_t>();

/// Make sure that DMA is enabled for receive only
constexpr auto control_settings3 = hal::bit_value(0UL)
                                     .set<control_reg::dma_receiver_enable>()
                                     .to<std::uint16_t>();
}  // namespace control_reg

/// Namespace for the baud rate (BRR) registers bit masks
namespace baud_rate_reg {
/// Mantissa of USART DIV
constexpr auto mantissa = hal::bit_mask::from<4, 15>();

/// Fraction of USART DIV
constexpr auto fraction = hal::bit_mask::from<0, 3>();
}  // namespace baud_rate_reg

struct usart_t
{
  std::uint32_t volatile status;
  std::uint32_t volatile data;
  std::uint32_t volatile baud_rate;
  std::uint32_t volatile control1;
  std::uint32_t volatile control2;
  std::uint32_t volatile control3;
  std::uint32_t volatile guard_time_and_prescale;
};

/**
 * @brief A generic uart implementation for all stm32 series MCUs
 *
 * This class is meant to only be used by platform libraries or drivers
 * aiming to be platform libraries for stm32 devices. Application developers
 * should use the driver specific to their platform instead of this class as
 * they handle proper initialization for you.
 *
 * This driver is synchronous/blocking for writes. It does not participate
 * in the async::context/coroutine model.
 *
 * This driver does not manage the receive DMA cursor itself. The receive
 * buffer is exposed via `receive_buffer()`; the platform specific driver is
 * responsible for reading the DMA controller's position and reporting it as
 * the `hal::serial::receive_cursor()` value, since DMA control is external
 * to the USART peripheral itself.
 */
export class uart
{
public:
  /**
   * @brief Construct a new uart object
   *
   * Care must be taken when constructing this object. The constructor does
   * not and cannot power on the uart peripheral. Accessing the peripheral's
   * registers without it be powered on (or provided a clock), will result
   * in a memory fault as the peripheral cannot ACK the CPU when it attempts
   * to access the peripheral's registers. The power management system for
   * each peripheral across the stm32 series of MCUs is different, meaning
   * its not something that can efficiently be manged by this generic
   * driver.
   *
   * @param p_uart - uart peripheral address
   * @param p_receive_buffer - buffer that the uart's receive DMA channel is
   * configured to write into
   */
  uart(void* p_uart, std::span<hal::byte> p_receive_buffer)
    : m_uart(p_uart)
    , m_receive_buffer(p_receive_buffer)
  {
  }

  /**
   * @brief Configures the serial port based on the settings
   *
   * @param p_settings - serial settings
   * @param p_frequency - frequency of the input clock to the peripheral
   */
  void configure(hal::serial::settings const& p_settings,
                 hal::hertz p_frequency)
  {
    auto& uart_reg = reg();
    uart_reg.control1 = control_reg::control_settings1;

    // NOTE: We leave control settings 2 alone as it is for features beyond
    //       basic UART such as USART clock, USART port network (LIN), and
    //       other things.

    uart_reg.control3 = control_reg::control_settings3;
    configure_baud_rate(p_frequency, p_settings);
    configure_format(p_settings);
  }

  /**
   * @brief STM32 Common write function
   *
   * This call blocks until every byte has been written.
   *
   * @param p_data - data to be written to the uart's transmit line
   */
  void write(mem::scatter_span<hal::byte const> p_data)
  {
    auto& uart_reg = reg();

    for (auto const& chunk : p_data) {
      for (auto const& byte : chunk) {
        while (not bit_extract<status_reg::transit_empty>(uart_reg.status)) {
          continue;
        }
        // Load the next byte into the data register
        uart_reg.data = byte;
      }
    }
  }

  /**
   * @brief Returns this driver's receive buffer
   *
   * The receive DMA channel is configured (by the platform specific driver)
   * to write incoming bytes into this buffer in a circular fashion. Use
   * this along with the DMA controller's position (converted by the
   * platform specific driver into a `hal::serial` receive cursor) to
   * determine what new data has arrived.
   *
   * @return hal::circular_span<hal::byte const> - view over the receive
   * buffer supplied at construction.
   */
  [[nodiscard]] hal::circular_span<hal::byte const> receive_buffer() const
  {
    return m_receive_buffer;
  }

  /**
   * @return u32 volatile* - address of the uart's data register. Used by
   * the platform specific driver to configure the receive DMA channel.
   */
  [[nodiscard]] u32 volatile* data_register()
  {
    return &reg().data;
  }

private:
  void configure_baud_rate(hal::hertz p_frequency,
                           hal::serial::settings const& p_settings)
  {
    auto const clock_frequency_value =
      p_frequency.numerical_value_in(hal::hertz::unit);
    auto const baud_rate_value =
      p_settings.baud_rate.numerical_value_in(hal::hertz::unit);
    auto const usart_divider = clock_frequency_value / (16 * baud_rate_value);

    // Truncate off the decimal values
    auto mantissa = static_cast<std::uint16_t>(usart_divider);

    // Subtract the whole number to leave just the decimal
    auto const fraction = static_cast<float>(static_cast<float>(usart_divider) -
                                             static_cast<float>(mantissa));

    auto fractional_int =
      static_cast<std::uint16_t>(std::roundf(fraction * 16));

    if (fractional_int >= 16) {
      mantissa = static_cast<std::uint16_t>(mantissa + 1U);
      fractional_int = 0;
    }

    reg().baud_rate = hal::bit_value()
                        .insert<baud_rate_reg::mantissa>(mantissa)
                        .insert<baud_rate_reg::fraction>(fractional_int)
                        .to<std::uint16_t>();
  }

  void configure_format(hal::serial::settings const& p_settings)
  {
    constexpr auto parity_selection = bit_mask::from<9>();
    constexpr auto parity_control = bit_mask::from<10>();
    constexpr auto word_length = bit_mask::from<12>();
    constexpr auto stop = bit_mask::from<12, 13>();

    bool const parity_enable =
      (p_settings.parity != hal::serial::settings::parity::none);
    bool const parity =
      (p_settings.parity == hal::serial::settings::parity::odd);
    bool const double_stop =
      (p_settings.stop == hal::serial::settings::stop_bits::two);
    std::uint16_t const stop_value = (double_stop) ? 0b10U : 0b00U;

    // Parity codes are: 0 for Even and 1 for Odd, thus the expression above
    // sets the bool to TRUE when odd and zero when something else. This
    // value is ignored if the parity is NONE since parity_enable will be
    // zero.
    auto& uart_reg = reg();
    bit_modify(uart_reg.control1)
      .insert<parity_control>(parity_enable)
      .insert<parity_selection>(parity)
      .insert<word_length>(0U);

    bit_modify(uart_reg.control2).insert<stop>(stop_value);
  }

  [[nodiscard]] usart_t& reg()
  {
    return *reinterpret_cast<usart_t*>(m_uart);
  }

  void* m_uart;
  std::span<hal::byte> m_receive_buffer;
};
}  // namespace hal::stm32_generic
