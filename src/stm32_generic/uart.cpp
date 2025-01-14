#include <cmath>

#include <cstdint>
#include <libhal-arm-mcu/stm32_generic/uart.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/serial.hpp>
#include <libhal/units.hpp>
#include <span>

namespace hal::stm32_generic {

/// Namespace for the status registers (SR) bit masks
struct status_reg  // NOLINT
{
  /// Indicates if the transmit data register is empty and can be loaded with
  /// another byte.
  static constexpr auto transit_empty = hal::bit_mask::from<7>();
};

/// Namespace for the control registers (CR1, CR3) bit masks and predefined
/// settings constants.
struct control_reg  // NOLINT
{
  /// When this bit is cleared the USART prescalers and outputs are stopped
  /// and the end of the current byte transfer in order to reduce power
  /// consumption. (CR1)
  static constexpr auto usart_enable = hal::bit_mask::from<13>();

  /// Enables DMA receiver (CR3)
  static constexpr auto dma_receiver_enable = hal::bit_mask::from<6>();

  /// This bit enables the transmitter. (CR1)
  static constexpr auto transmitter_enable = hal::bit_mask::from<3>();

  /// This bit enables the receiver. (CR1)
  static constexpr auto receive_enable = hal::bit_mask::from<2>();

  /// Enable USART + Enable Receive + Enable Transmitter
  static constexpr auto control_settings1 =
    hal::bit_value(0UL)
      .set<control_reg::usart_enable>()
      .set<control_reg::receive_enable>()
      .set<control_reg::transmitter_enable>()
      .to<std::uint16_t>();

  /// Make sure that DMA is enabled for receive only
  static constexpr auto control_settings3 =
    hal::bit_value(0UL)
      .set<control_reg::dma_receiver_enable>()
      .to<std::uint16_t>();
};

/// Namespace for the baud rate (BRR) registers bit masks
struct baud_rate_reg  // NOLINT
{
  /// Mantissa of USART DIV
  static constexpr auto mantissa = hal::bit_mask::from<4, 15>();

  /// Fraction of USART DIV
  static constexpr auto fraction = hal::bit_mask::from<0, 3>();
};

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

inline usart_t* to_usart(void* p_uart)
{
  return reinterpret_cast<usart_t*>(p_uart);
}

uart::uart(void* p_uart, std::span<hal::byte> p_receive_buffer)
  : m_uart(p_uart)
  , m_receive_buffer(p_receive_buffer)
  , m_read_index(0)
{
}

void uart::configure_baud_rate(hal::hertz p_frequency,
                               serial::settings const& p_settings)
{
  auto const clock_frequency = p_frequency;
  float usart_divider = clock_frequency / (16.0f * p_settings.baud_rate);

  // Truncate off the decimal values
  auto mantissa = static_cast<uint16_t>(usart_divider);

  // Subtract the whole number to leave just the decimal
  auto fraction =
    static_cast<float>(usart_divider - static_cast<float>(mantissa));

  auto fractional_int = static_cast<uint16_t>(std::roundf(fraction * 16));

  if (fractional_int >= 16) {
    mantissa = static_cast<uint16_t>(mantissa + 1U);
    fractional_int = 0;
  }

  to_usart(m_uart)->baud_rate =
    hal::bit_value()
      .insert<baud_rate_reg::mantissa>(mantissa)
      .insert<baud_rate_reg::fraction>(fractional_int)
      .to<std::uint16_t>();
}

void uart::configure_format(serial::settings const& p_settings)
{
  constexpr auto parity_selection = bit_mask::from<9>();
  constexpr auto parity_control = bit_mask::from<10>();
  constexpr auto word_length = bit_mask::from<12>();
  constexpr auto stop = bit_mask::from<12, 13>();

  bool parity_enable = (p_settings.parity != serial::settings::parity::none);
  bool parity = (p_settings.parity == serial::settings::parity::odd);
  bool double_stop = (p_settings.stop == serial::settings::stop_bits::two);
  std::uint16_t stop_value = (double_stop) ? 0b10U : 0b00U;

  // Parity codes are: 0 for Even and 1 for Odd, thus the expression above
  // sets the bool to TRUE when odd and zero when something else. This value
  // is ignored if the parity is NONE since parity_enable will be zero.
  auto& uart_reg = *to_usart(m_uart);
  bit_modify(uart_reg.control1)
    .insert<parity_control>(parity_enable)
    .insert<parity_selection>(parity)
    .insert<word_length>(0U);

  bit_modify(uart_reg.control2).insert<stop>(stop_value);
}

void uart::configure(serial::settings const& p_settings, hertz p_frequency)
{
  auto& uart_reg = *to_usart(m_uart);
  uart_reg.control1 = control_reg::control_settings1;

  // NOTE: We leave control settings 2 alone as it is for features beyond
  //       basic UART such as USART clock, USART port network (LIN), and other
  //       things.

  uart_reg.control3 = control_reg::control_settings3;
  configure_baud_rate(p_frequency, p_settings);
  configure_format(p_settings);
}
// TODO(#86) Add DMA write support to stm32_generic/uart.cpp
serial::write_t uart::uart_write(std::span<hal::byte const> p_data)
{
  auto& uart_reg = *to_usart(m_uart);

  for (auto const& byte : p_data) {
    while (not bit_extract<status_reg::transit_empty>(uart_reg.status)) {
      continue;
    }
    // Load the next byte into the data register
    uart_reg.data = byte;
  }

  return {
    .data = p_data,
  };
}

serial::read_t uart::uart_read(std::span<hal::byte>& p_data,
                               std::uint32_t const& p_dma_cursor_position)
{
  size_t count = 0;

  for (auto& byte : p_data) {
    if (m_read_index == p_dma_cursor_position) {
      break;
    }
    byte = m_receive_buffer[m_read_index++];
    m_read_index = m_read_index % m_receive_buffer.size();
    count++;
  }

  return {
    .data = p_data.first(count),
    .available = 1,
    .capacity = m_receive_buffer.size(),
  };
}
uint32_t volatile* uart::data_register()
{
  return &to_usart(m_uart)->data;
}
void uart::flush(std::uint32_t p_dma_cursor_position)
{
  m_read_index = p_dma_cursor_position;
}

std::uint32_t uart::buffer_size()
{
  return m_receive_buffer.size();
}
}  // namespace hal::stm32_generic
