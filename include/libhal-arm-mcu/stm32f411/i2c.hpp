
#pragma once

#include <libhal-arm-mcu/stm32_generic/i2c.hpp>
#include <libhal/i2c.hpp>
#include <libhal/initializers.hpp>

#include "constants.hpp"

namespace hal::stm32f411 {
class i2c : public hal::i2c
{
public:
  /**
   * @brief Construct a new i2c object NOTE: does not use internal pull-up
   * resistors
   *
   * @param p_port 1 -3
   * @param p_settings
   * @param p_waiter
   */
  i2c(std::uint8_t p_bus_number,
      i2c::settings const& p_settings = {},
      hal::io_waiter& p_waiter = hal::polling_io_waiter());

  i2c(i2c const& p_other) = delete;
  i2c& operator=(i2c const& p_other) = delete;
  i2c(i2c&& p_other) noexcept = delete;
  i2c& operator=(i2c&& p_other) noexcept = delete;
  virtual ~i2c();

private:
  void driver_configure(settings const& p_settings) override;
  void driver_transaction(
    hal::byte p_address,
    std::span<hal::byte const> p_data_out,
    std::span<hal::byte> p_data_in,
    hal::function_ref<hal::timeout_function> p_timeout) override;
  void setup_interrupt();
  stm32_generic::i2c m_i2c;
  peripheral m_id;
};
}  // namespace hal::stm32f411
