#include <libhal-arm-mcu/interrupt.hpp>
#include <libhal-arm-mcu/stm32f411/constants.hpp>
#include <libhal-arm-mcu/stm32f411/interrupt.hpp>

namespace hal::stm32f411 {
void initialize_interrupts()
{
  hal::cortex_m::initialize_interrupts<irq::max>();
}
}  // namespace hal::stm32f411
