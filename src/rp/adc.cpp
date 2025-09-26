#include "libhal-arm-mcu/rp/adc.hpp"

#include "hardware/adc.h"
#include <hardware/address_mapped.h>
#include <hardware/gpio.h>
#include <hardware/platform_defs.h>
#include <libhal/error.hpp>
#include <libhal/units.hpp>

namespace hal::rp {

inline namespace v4 {
adc::adc(u8 pin)
  : m_pin(pin)
{
  adc_init();
  if (pin < ADC_BASE_PIN || pin >= ADC_BASE_PIN + NUM_ADC_CHANNELS - 1) {
    hal::safe_throw(hal::argument_out_of_domain(this));
  }
  adc_gpio_init(pin);
}

adc::~adc()
{
  gpio_deinit(m_pin);
}

float adc::driver_read()
{
  adc_select_input(m_pin - ADC_BASE_PIN);
  u16 result = adc_read();
  // weirdly enough the sdk doesn't provide a function to read the error bits
  if (adc_hw->cs & ADC_CS_ERR_BITS) {
    hal::safe_throw(hal::io_error(this));
  }

  return float(result) / float((1 << 12) - 1);
}
};  // namespace v4

namespace v5 {

adc16::adc16(u8 pin)
  : m_pin(pin)
{
  adc_init();
  if (pin < ADC_BASE_PIN || pin >= ADC_BASE_PIN + NUM_ADC_CHANNELS - 1) {
    hal::safe_throw(hal::argument_out_of_domain(this));
  }
  adc_gpio_init(pin);
}

adc16::~adc16()
{
  gpio_deinit(m_pin);
}

u16 adc16::driver_read()
{
  adc_select_input(m_pin - ADC_BASE_PIN);
  u16 result = adc_read();
  // weirdly enough the sdk doesn't provide a function to read the error bits
  if (adc_hw->cs & ADC_CS_ERR_BITS) {
    hal::safe_throw(hal::io_error(this));
  }
  // TODO: Use hal::upscale to actually make this 16 bit
  // for some reason the docs reference a function that
  // doesn't exist yet
  return result;
}

}  // namespace v5
}  // namespace hal::rp
