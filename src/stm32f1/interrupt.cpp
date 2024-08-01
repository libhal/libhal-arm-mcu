#include <libhal-armcortex/interrupt.hpp>
#include <libhal-stm32f1/constants.hpp>

namespace hal::stm32f1 {
void initialize_interrupts()
{
  hal::cortex_m::initialize_interrupts<irq::max>();
}
}  // namespace hal::stm32f1
