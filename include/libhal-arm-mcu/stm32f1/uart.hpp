#pragma once

#include <cstdint>

#include <libhal/initializers.hpp>
#include <libhal/serial.hpp>

#include "constants.hpp"
#include "dma.hpp"

namespace hal::stm32f1 {
class uart final : public hal::serial
{
public:
  /**
   * @brief Construct a new uart object
   *
   * @param p_port - desired port number
   * @param p_buffer - receive buffer size (statically allocated buffer)
   * @param p_settings - initial serial settings
   */
  uart(hal::port_param auto p_port,
       hal::buffer_param auto p_buffer,
       serial::settings const& p_settings = {})
    : uart(p_port(), hal::create_unique_static_buffer(p_buffer), p_settings)
  {
    static_assert(p_buffer() <= max_dma_length,
                  "Buffer size cannot exceed 65,535 bytes,");
    static_assert(1 <= p_port() and p_port() <= 3,
                  "stm32f1 only supports ports from 1 to 3");
  }

  /**
   * @brief Construct a new uart object using runtime values
   *
   * @param p_port - runtime value for p_port
   * @param p_buffer - external buffer to be used as the receive buffer
   * @param p_settings - initial serial settings
   * @throws hal::operation_not_supported - if the port is not supported or the
   * buffer is outside of the maximum dma length.
   */
  uart(hal::runtime,
       std::uint8_t p_port,
       std::span<hal::byte> p_buffer,
       serial::settings const& p_settings = {});

private:
  uart(std::uint8_t p_port,
       std::span<hal::byte> p_receive_buffer,
       serial::settings const& p_settings);

  void driver_configure(settings const& p_settings) override;
  write_t driver_write(std::span<hal::byte const> p_data) override;
  read_t driver_read(std::span<hal::byte> p_data) override;
  void driver_flush() override;

  std::uint32_t dma_cursor_position();

  void* m_uart;
  std::span<hal::byte> m_receive_buffer;
  std::uint16_t m_read_index;
  std::uint8_t m_dma;
  peripheral m_id;
};
}  // namespace hal::stm32f1