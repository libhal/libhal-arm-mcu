#include "libhal-arm-mcu/rp/time.hpp"
#include <libhal/units.hpp>

#include <hardware/platform_defs.h>
#include <hardware/timer.h>
#include <pico/time.h>

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

void sleep(std::chrono::duration<u64, std::micro> time) noexcept
{
  sleep_us(time.count());
}

}  // namespace hal::rp::inline v4
