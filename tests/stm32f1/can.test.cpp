#include <libhal-arm-mcu/stm32f1/can.hpp>

#include <boost/ut.hpp>

namespace hal::stm32f1 {
namespace {
bool volatile skip = true;
}
boost::ut::suite can_test = []() {
  // NOLINTNEXTLINE
  can* my_can = reinterpret_cast<can*>(0x1000'0000);

  if (not skip) {
    my_can->bus_on();
  }
};
}  // namespace hal::stm32f1
