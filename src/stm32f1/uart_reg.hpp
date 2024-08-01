#pragma once

#include <libhal-util/bit.hpp>

namespace hal::stm32f1 {

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

inline auto* usart1 = reinterpret_cast<usart_t*>(0x4001'3800);
inline auto* usart2 = reinterpret_cast<usart_t*>(0x4000'4400);
inline auto* usart3 = reinterpret_cast<usart_t*>(0x4000'4800);
inline auto* uart4 = reinterpret_cast<usart_t*>(0x4000'4C00);
inline auto* uart5 = reinterpret_cast<usart_t*>(0x4000'5000);

}  // namespace hal::stm32f1