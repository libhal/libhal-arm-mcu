#include "libhal-arm-mcu/rp/generic/input_pin.hpp"
#include "libhal-arm-mcu/rp/generic/output_pin.hpp"
#include "libhal/error.hpp"

#include <hardware/gpio.h>
#include <hardware/structs/io_bank0.h>
#include <pico/time.h>

namespace hal::rp::generic {
void v1::sleep_ms(uint32_t ms)
{
  ::sleep_ms(ms);
}

v1::input_pin::input_pin(u8 pin, settings const& options)
  : m_pin(pin)
{
  if (pin >= NUM_BANK0_GPIOS) {
    hal::safe_throw(hal::argument_out_of_domain(this));
  }

  gpio_init(pin);
  gpio_set_function(pin, gpio_function_t::GPIO_FUNC_SIO);
  gpio_set_dir(pin, GPIO_IN);

  driver_configure(options);
}

void v1::input_pin::driver_configure(settings const& p_settings)
{
  switch (p_settings.resistor) {
    case pin_resistor::pull_down:
      gpio_pull_down(m_pin);
      break;
    case pin_resistor::pull_up:
      gpio_pull_up(m_pin);
      break;
    default:
      [[fallthrough]];
    case pin_resistor::none:
      gpio_disable_pulls(m_pin);
      break;
  }
}

bool v1::input_pin::driver_level()
{
  return gpio_get(m_pin);
}

v1::output_pin::output_pin(u8 pin, settings const& options)
  : m_pin(pin)
{
  if (pin >= NUM_BANK0_GPIOS) {
    hal::safe_throw(hal::argument_out_of_domain(this));
  }

  gpio_init(pin);
  gpio_set_function(pin, gpio_function_t::GPIO_FUNC_SIO);
  gpio_set_dir(pin, GPIO_OUT);

  driver_configure(options);
}

void v1::output_pin::driver_configure(settings const& options)
{
  // RP2* series chips don't seem to have any explicit support for
  // open drain mode, so we fail loud rather than silently
  if (options.open_drain) {
    hal::safe_throw(hal::argument_out_of_domain(this));
  }

  switch (options.resistor) {
    case pin_resistor::pull_down:
      gpio_pull_down(m_pin);
      break;
    case pin_resistor::pull_up:
      gpio_pull_up(m_pin);
      break;
    default:
      [[fallthrough]];
    case pin_resistor::none:
      gpio_disable_pulls(m_pin);
      break;
  }
}

void v1::output_pin::driver_level(bool level)
{
  gpio_put(m_pin, level);
}

bool v1::output_pin::driver_level()
{
  return gpio_get(m_pin);
}

}  // namespace hal::rp::generic
