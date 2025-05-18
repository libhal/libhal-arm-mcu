#pragma once

#include "rp.hpp"
#include <libhal/i2c.hpp>
#include <libhal/units.hpp>

namespace hal::rp::inline v1 {
/*
RP2350 supports a baud rate of up to 1 MHz
*/
struct i2c final : public hal::i2c
{

  enum i2c_channel : u8
  {
    i2c0,
    i2c1
  };

  struct i2c_config_data
  {
    u8 const sda_pin;
    u8 const scl_pin;
    i2c_channel const chan;

  protected:
    // clang lint does not know that this constructor is purposefully obstructed
    // to prevent misuse
    constexpr i2c_config_data(u8 sda, u8 scl, i2c_channel c)  // NOLINT
      : sda_pin(sda)
      , scl_pin(scl)
      , chan(c)
    {
    }
  };

  // yes, it is entirely valid to have sda and scl on pins that aren't next to
  // each other because it's all muxed
  template<u8 sda, u8 scl, i2c_channel channel>
  struct i2c_config : public i2c_config_data
  {
    constexpr i2c_config()
      : i2c_config_data(sda, scl, channel)
    {
    }
    static consteval bool check()
    {
      if constexpr (channel == i2c0) {
        static_assert(sda % 4 == 0 && sda < internal::pin_max,
                      "SDA pin for I2C0 is invalid!");
        static_assert(scl % 4 == 1 && scl < internal::pin_max,
                      "SCL pin for I2C0 is invalid!");
      } else if constexpr (channel == i2c1) {
        static_assert(sda % 4 == 2 && sda < internal::pin_max,
                      "SDA pin for I2C1 is invalid!");
        static_assert(scl % 4 == 3 && scl < internal::pin_max,
                      "SCL pin for I2C1 is invalid!");
      }
      return true;
    }
    static_assert(check());
  };

  i2c(i2c_config_data, settings const&);
  ~i2c() override;

private:
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

  i2c_config_data m_config;
};

}  // namespace hal::rp::inline v1
