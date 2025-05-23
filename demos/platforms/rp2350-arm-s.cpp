#include <libhal-arm-mcu/rp/output_pin.hpp>
#include <libhal-arm-mcu/rp/serial.hpp>
#include <libhal-arm-mcu/rp/time.hpp>
#include <libhal/initializers.hpp>
#include <resource_list.hpp>

namespace rp = hal::rp;

void initialize_platform(resource_list& resource)
{
  using namespace hal::literals;
  static auto serial = rp::stdio_serial();
  resource.console = &serial;

  static auto led = rp::output_pin(hal::pin<7>, {});
  resource.status_led = &led;
  static auto counter = rp::clock();
  resource.clock = &counter;
  
}
