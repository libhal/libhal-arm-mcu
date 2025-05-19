#include "libhal-arm-mcu/rp/i2c.hpp"

#include <hardware/gpio.h>
#include <hardware/i2c.h>
#include <libhal/error.hpp>
#include <pico/error.h>

// pico macros interfere with ours
#undef i2c0
#undef i2c1

namespace {
auto inst(hal::u8 c, void* instance)
{
  switch (c) {
    case 0:
      return &i2c0_inst;
    case 1:
      return &i2c1_inst;
    default:
      // realistically should never happen
      hal::safe_throw(hal::io_error(instance));
  }
}
}  // namespace

namespace hal::rp::inline v1 {
i2c::i2c(u8 sda, u8 scl, u8 chan, settings const& options)  // NOLINT
  : m_sda(sda)
  , m_scl(scl)
  , m_chan(chan)
{
  i2c_init(inst(chan, this), static_cast<uint>(options.clock_rate));
  gpio_pull_up(sda);
  gpio_pull_up(scl);
  gpio_set_function(sda, gpio_function_t::GPIO_FUNC_I2C);
  gpio_set_function(scl, gpio_function_t::GPIO_FUNC_I2C);
}

i2c::~i2c()
{
  i2c_deinit(inst(m_chan, this));
  gpio_deinit(m_sda);
  gpio_deinit(m_scl);
}

void i2c::driver_configure(settings const& options)
{
  i2c_set_baudrate(inst(m_chan, this), static_cast<uint>(options.clock_rate));
}

void i2c::driver_transaction(hal::byte addr,
                             std::span<hal::byte const> out,
                             std::span<hal::byte> in,
                             hal::function_ref<hal::timeout_function> timeout)
{
  bool is_read = in.size() != 0;
  int write_result = i2c_write_timeout_us(
    inst(m_chan, this), addr, out.data(), out.size_bytes(), is_read, 5000);
  if (write_result == PICO_ERROR_GENERIC) {
    safe_throw(no_such_device(addr, this));
  }
  if (write_result == PICO_ERROR_TIMEOUT) {
    safe_throw(timed_out(this));
  }
  if (is_read) {
    int read_result = i2c_read_timeout_us(
      inst(m_chan, this), addr, in.data(), in.size_bytes(), false, 5000);
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
