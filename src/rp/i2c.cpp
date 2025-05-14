#include "libhal-arm-mcu/rp/i2c.hpp"

#include <hardware/gpio.h>
#include <hardware/i2c.h>
#include <libhal/error.hpp>
#include <pico/error.h>

// pico macros interfere with ours
#undef i2c0
#undef i2c1

namespace {
auto inst(hal::rp::i2c::i2c_channel c, void* instance)
{
  switch (c) {
    case hal::rp::i2c::i2c0:
      return &i2c0_inst;
    case hal::rp::i2c::i2c1:
      return &i2c1_inst;
    default:
      // realistically should never happen
      hal::safe_throw(hal::io_error(instance));
  }
}
}  // namespace

namespace hal::rp::inline v1 {
i2c::i2c(i2c_config_data data, settings const& options)
  : m_config(data)
{
  gpio_pull_up(data.scl_pin);
  gpio_pull_up(data.sda_pin);
  gpio_set_function(data.scl_pin, gpio_function_t::GPIO_FUNC_I2C);
  gpio_set_function(data.sda_pin, gpio_function_t::GPIO_FUNC_I2C);
  i2c_init(inst(data.chan, this), static_cast<uint>(options.clock_rate));
}

i2c::~i2c()
{
  gpio_deinit(m_config.scl_pin);
  gpio_deinit(m_config.sda_pin);
  i2c_deinit(inst(m_config.chan, this));
}

void i2c::driver_configure(settings const& options)
{
  i2c_set_baudrate(inst(m_config.chan, this),
                   static_cast<uint>(options.clock_rate));
}

void i2c::driver_transaction(hal::byte addr,
                             std::span<hal::byte const> out,
                             std::span<hal::byte> in,
                             hal::function_ref<hal::timeout_function> timeout)
{
  bool is_read = in.size() != 0;
  int write_result = i2c_write_timeout_us(inst(m_config.chan, this),
                                          addr,
                                          out.data(),
                                          out.size_bytes(),
                                          is_read,
                                          5000);
  if (write_result == PICO_ERROR_GENERIC) {
    safe_throw(no_such_device(addr, this));
  }
  if (write_result == PICO_ERROR_TIMEOUT) {
    safe_throw(timed_out(this));
  }
  if (is_read) {
    int read_result = i2c_read_timeout_us(
      inst(m_config.chan, this), addr, in.data(), in.size_bytes(), false, 5000);
    if (read_result == PICO_ERROR_GENERIC) {
      safe_throw(no_such_device(addr, this));
    }
    if (read_result == PICO_ERROR_TIMEOUT) {
      safe_throw(timed_out(this));
    }
  }
  timeout();
}

}  // namespace hal::rp::inline v1
