#include <libhal-arm-mcu/rp/adc.hpp>
#include <libhal-arm-mcu/rp/i2c.hpp>
#include <libhal-arm-mcu/rp/input_pin.hpp>
#include <libhal-arm-mcu/rp/interrupt_pin.hpp>
#include <libhal-arm-mcu/rp/output_pin.hpp>
#include <libhal-arm-mcu/rp/pwm.hpp>
#include <libhal-arm-mcu/rp/serial.hpp>
#include <libhal-arm-mcu/rp/spi.hpp>
#include <libhal-arm-mcu/rp/time.hpp>
#include <libhal/initializers.hpp>
#include <resource_list.hpp>

namespace rp = hal::rp;

void do_nothing(bool)
{
}

void initialize_platform(resource_list& resource)
{
  using namespace hal::literals;
  static auto serial = rp::stdio_serial();
  resource.console = &serial;

  static auto led = rp::output_pin(hal::pin<7>, {});
  resource.status_led = &led;

  static auto clock = rp::clock();
  resource.clock = &clock;

  // the hal demos don't use adc16 yet
  // static auto adc = rp::adc(hal::pin<27>);
  // resource.adc = &adc;

  static auto button = rp::input_pin(hal::pin<0>, {});
  resource.input_pin = &button;

  static auto i2c = rp::i2c(hal::pin<6>, hal::pin<7>, hal::bus<1>);
  resource.i2c = &i2c;

  static auto interrupt = rp::interrupt_pin(hal::pin<1>, &do_nothing);
  resource.interrupt_pin = &interrupt;

  static auto slice = rp::pwm_slice(hal::channel<1>);
  resource.pwm_frequency = &slice;
  static auto chan = slice.get_pin(hal::pin<2>);
  resource.pwm_channel = &chan;

  // I wrote spi as spi_channel. Maybe rewrite to support multi-peripheral
  // support later.
  // static auto spi = rp::spi()
}
