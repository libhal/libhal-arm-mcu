#include <libhal-arm-mcu/stm32f1/independent_watchdog.hpp>
#include <libhal-util/bit.hpp>
#include <libhal/error.hpp>
#include <libhal/units.hpp>
#include <memory>

using namespace std::chrono_literals;

namespace hal {
struct independent_watchdog_registers
{
  volatile uint32_t kr;
  volatile uint32_t pr;
  volatile uint32_t rlr;
  volatile uint32_t sr;
};
auto* const iwdg_regs =
  reinterpret_cast<independent_watchdog_registers*>(0x40003000);
uint32_t* const reset_status_register = 
  reinterpret_cast<uint32_t*>(0x40021000 + 0x24);
}  // namespace hal
namespace hal::stm32f1 {

void start_independent_watchdog()
{
  iwdg_regs->kr = 0xCCCC;
}
void stop_independent_watchdog()
{
  iwdg_regs->kr = 0xAAAA;
}
void restart_independent_watchdog()
{
  stop_independent_watchdog();
  start_independent_watchdog();
}

void set_independent_watchdog_countdown_time(hal::time_duration wait_time)
{
  // convert to counts of 40khz clock (pg. 126)
  constexpr hal::time_duration nano_seconds_per_clock_cycle =
    1000000000ns / 400000;
  uint32_t cycle_count = wait_time / nano_seconds_per_clock_cycle;
  //frq divider starts a /4
  cycle_count /= 4;
  if (cycle_count == 0) {
    throw hal::operation_not_supported(nullptr);
  }

  hal::byte frq_divider = 0;
  while (cycle_count > 0xFFF && frq_divider <= 7) {
    cycle_count = cycle_count >> 1;
    frq_divider++;
  }
  if (frq_divider >= 7) {
    throw hal::operation_not_supported(nullptr);
  } else {
    // set values
    iwdg_regs->kr = 0x5555;
    iwdg_regs->pr = frq_divider;
    iwdg_regs->rlr = cycle_count;
  }
}

bool reset_by_independent_watchdog()
{
  return bit_extract(bit_mask::from(30), *reset_status_register);
}

}  // namespace hal::stm32f1
