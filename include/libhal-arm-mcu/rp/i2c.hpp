#pragma once

#include <libhal/i2c.hpp>
#include <libhal/units.hpp>

namespace hal::rp::inline v1 {

// might be better to place in another header file if this gets reused
#ifdef PICO_RP2350A
constexpr u8 pin_max = 30;
#else
constexpr u8 pin_max = 48;
#endif

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
    i2c_channel const chan_pin;

  protected:
    // clang lint does not know that this constructor is purposefully obstructed
    // to prevent misuse
    constexpr i2c_config_data(u8 sda, u8 scl, i2c_channel chan)  // NOLINT
      : sda_pin(sda)
      , scl_pin(scl)
      , chan_pin(chan)
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
        static_assert(sda % 4 == 0 && sda < pin_max,
                      "SDA pin for I2C0 is invalid!");
        static_assert(scl % 4 == 1 && scl < pin_max,
                      "SCL pin for I2C0 is invalid!");
      } else if constexpr (channel == i2c1) {
        static_assert(sda % 4 == 2 && sda < pin_max,
                      "SDA pin for I2C1 is invalid!");
        static_assert(scl % 4 == 3 && scl < pin_max,
                      "SCL pin for I2C1 is invalid!");
      }
      return true;
    }
    static_assert(check());
  };

  i2c(i2c_config_data);
  ~i2c() override;

private:
  void driver_configure(settings const&) override;

  void driver_transaction(hal::byte,
                          std::span<hal::byte const>,
                          std::span<hal::byte>,
                          hal::function_ref<hal::timeout_function>) override;
};

}  // namespace hal::rp::inline v1
