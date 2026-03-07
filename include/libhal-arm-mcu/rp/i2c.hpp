#pragma once

#include "rp.hpp"
#include <libhal/i2c.hpp>
#include <libhal/initializers.hpp>
#include <libhal/units.hpp>

namespace hal::rp::inline v4 {
/*
RP2350 supports a baud rate of up to 1 MHz
*/
struct i2c final : public hal::i2c
{
  i2c(pin_param auto sda,
      pin_param auto scl,
      bus_param auto bus,
      settings const& s = {})
    : i2c(sda(), scl(), bus(), s)
  {
    static_assert(bus() == 0 || bus() == 1, "Invalid bus selected!");
    static_assert(sda() % 4 == 0 || bus() != 0, "SDA pin for I2C0 is invalid!");
    static_assert(scl() % 4 == 1 || bus() != 0, "SCL pin for I2C0 is invalid!");
    static_assert(sda() % 4 == 2 || bus() != 1, "SDA pin for I2C1 is invalid!");
    static_assert(scl() % 4 == 3 || bus() != 1, "SCL pin for I2C1 is invalid!");
  }
  i2c(i2c&&) = delete;
  ~i2c() override;

private:
  i2c(u8 sda, u8 scl, u8 chan, settings const&);
  void driver_configure(settings const&) override;

  /*
  This function does not correctly use the timeout function, and will
  throw it's own timeout exceptions if a transaction takes any longer
  than 10 ms.
  */
  void driver_transaction(hal::byte addr,
                          std::span<hal::byte const> out,
                          std::span<hal::byte> in,
                          hal::function_ref<hal::timeout_function>) override;

  u8 m_sda, m_scl, m_chan;
};

}  // namespace hal::rp::inline v4
