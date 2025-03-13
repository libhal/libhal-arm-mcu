#include <libhal-arm-mcu/stm32f1/usart.hpp>

#include <cmath>

#include <libhal-arm-mcu/stm32f1/clock.hpp>
#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal-arm-mcu/stm32f1/pin.hpp>
#include <libhal-arm-mcu/stm32f1/uart.hpp>
#include <libhal-util/bit.hpp>
#include <libhal/error.hpp>

#include "pin.hpp"
#include "power.hpp"
#include "usart_reg.hpp"

namespace hal::stm32f1 {
namespace {
inline void configure_baud_rate(usart_t& p_usart,
                                u32 p_frequency,
                                serial::settings const& p_settings)
{
  float const usart_divider =
    static_cast<float>(p_frequency) / (16.0f * p_settings.baud_rate);

  // Truncate off the decimal values
  auto mantissa = static_cast<uint16_t>(usart_divider);
  // Subtract the whole number to leave just the decimal
  auto fraction = usart_divider - static_cast<float>(mantissa);
  auto fractional_int = static_cast<uint16_t>(std::roundf(fraction * 16));

  if (fractional_int >= 16) {
    mantissa = static_cast<uint16_t>(mantissa + 1U);
    fractional_int = 0;
  }

  p_usart.baud_rate = hal::bit_value()
                        .insert<baud_rate_reg::mantissa>(mantissa)
                        .insert<baud_rate_reg::fraction>(fractional_int)
                        .to<std::uint16_t>();
}

inline void configure_format(usart_t& p_usart,
                             serial::settings const& p_settings)
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

  bit_modify(p_usart.control1)
    .insert<parity_control>(parity_enable)
    .insert<parity_selection>(parity)
    .insert<word_length>(0U);

  bit_modify(p_usart.control2).insert<stop>(stop_value);
}
}  // namespace

usart_manager::usart_manager(peripheral p_select)
  : m_reg(peripheral_to_register(p_select))
  , m_id(p_select)
{
  power_on(m_id);
}

usart_manager::~usart_manager()
{
  power_off(m_id);
}

usart_manager::serial usart_manager::acquire_serial(
  std::span<byte> p_buffer,
  hal::serial::settings const& p_settings)
{
  return { *this, p_buffer, p_settings };
}

usart_manager::serial::serial(usart_manager& p_usart_manager,
                              std::span<byte> p_buffer,
                              hal::serial::settings const& p_settings)
  : m_usart_manager(&p_usart_manager)
  , m_buffer(p_buffer)
  , m_dma_channel(0)
{

  if (p_buffer.size() > max_dma_length) {
    hal::safe_throw(hal::operation_not_supported(this));
  }

  switch (m_usart_manager->m_id) {
    case peripheral::usart1:
      m_tx = { .port = 'A', .pin = 9 };
      m_rx = { .port = 'A', .pin = 10 };
      m_dma_channel = 5;
      break;
    case peripheral::usart2:
      m_tx = { .port = 'A', .pin = 2 };
      m_rx = { .port = 'A', .pin = 3 };
      m_dma_channel = 6;
      break;
    case peripheral::usart3:
      m_tx = { .port = 'B', .pin = 10 };
      m_rx = { .port = 'B', .pin = 11 };
      m_dma_channel = 3;
      break;
    default:
      hal::safe_throw(hal::operation_not_supported(this));
  }

  // Power on dma1 which has the usart channels
  // TODO(): DMA1 is shared across multiple peripherals
  if (not is_on(peripheral::dma1)) {
    power_on(peripheral::dma1);
  }

  auto& uart_reg = *to_usart(m_usart_manager->m_reg);

  // Setup RX DMA channel
  auto const data_address = reinterpret_cast<intptr_t>(&uart_reg.data);
  auto const queue_address = reinterpret_cast<intptr_t>(p_buffer.data());
  auto const data_address_int = static_cast<std::uint32_t>(data_address);
  auto const queue_address_int = static_cast<std::uint32_t>(queue_address);

  dma::dma1->channel[m_dma_channel - 1].transfer_amount = p_buffer.size();
  dma::dma1->channel[m_dma_channel - 1].peripheral_address = data_address_int;
  dma::dma1->channel[m_dma_channel - 1].memory_address = queue_address_int;
  dma::dma1->channel[m_dma_channel - 1].configuration = uart_dma_settings1;

  // Setup UART Control Settings 1
  uart_reg.control1 = control_reg::control_settings1;

  // NOTE: We leave control settings 2 alone as it is for features beyond
  //       basic UART such as USART clock, USART port network (LIN), and other
  //       things.

  // Setup UART Control Settings 3
  uart_reg.control3 = control_reg::control_settings3;

  serial::driver_configure(p_settings);

  configure_pin(m_tx, push_pull_alternative_output);
  configure_pin(m_rx, input_pull_up);
}

void usart_manager::serial::driver_configure(
  hal::serial::settings const& p_settings)
{
  auto& uart_reg = *to_usart(m_usart_manager->m_reg);
  auto const uart_freq = static_cast<u32>(frequency(m_usart_manager->m_id));
  configure_baud_rate(uart_reg, uart_freq, p_settings);
  configure_format(uart_reg, p_settings);
}

void usart_manager::serial::driver_write(std::span<hal::byte const> p_data)
{
  auto& uart_reg = *to_usart(m_usart_manager->m_reg);

  for (auto const& byte : p_data) {
    while (not bit_extract<status_reg::transit_empty>(uart_reg.status)) {
      continue;
    }
    // Load the next byte into the data register
    uart_reg.data = byte;
  }
}

std::span<hal::byte const> usart_manager::serial::driver_receive_buffer()
{
  return m_buffer;
}

std::size_t usart_manager::serial::driver_cursor()
{
  return m_buffer.size() -
         dma::dma1->channel[m_dma_channel - 1].transfer_amount;
}

usart_manager::serial::~serial()
{
  reset_pin(m_tx);
  reset_pin(m_rx);
}
}  // namespace hal::stm32f1
