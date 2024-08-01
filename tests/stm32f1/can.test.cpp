#include <libhal-stm32f1/can.hpp>

#include "can_reg.hpp"
#include "rcc_reg.hpp"

namespace hal::stm32f1 {
namespace {
bool volatile skip = true;
}
void can_test()
{
  can* my_can = reinterpret_cast<can*>(0x1000'0000);
  if (not skip) {
    my_can->bus_on();
  }
}
}  // namespace hal::stm32f1
