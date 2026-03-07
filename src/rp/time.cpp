#include "libhal-arm-mcu/rp/time.hpp"
#include <libhal/units.hpp>

#include <hardware/platform_defs.h>
#include <hardware/timer.h>

namespace hal::rp::inline v4 {

hertz clock::driver_frequency()
{
  return 1'000'000;
}

u64 clock::driver_uptime()
{
  return time_us_64();
}

hertz core_clock()
{
  return SYS_CLK_HZ;
}

}  // namespace hal::rp::inline v4
