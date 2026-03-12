#include <libhal-arm-mcu/stm32f1/can.hpp>

#include <boost/ut.hpp>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

namespace hal::stm32f1 {
namespace {
bool volatile skip = true;
}

boost::ut::suite can_test = []() {
  can* my_can = reinterpret_cast<can*>(0x1000'0000);
  auto* manager = reinterpret_cast<can_peripheral_manager*>(0x2000'0000);
  if (not skip) {
    my_can->bus_on();
    my_can->configure({});
    my_can->send({});
    my_can->on_receive({});
    my_can->enable_self_test(true);

    manager->enable_self_test(true);
    std::array<can_message, 8> buffer;
    [[maybe_unused]] auto transceiver = manager->acquire_transceiver(buffer);
    [[maybe_unused]] auto bus_mgr = manager->acquire_bus_manager();
    [[maybe_unused]] auto interrupt = manager->acquire_interrupt();
    [[maybe_unused]] auto id_filters = manager->acquire_identifier_filter();
    [[maybe_unused]] auto ext_id_filters =
      manager->acquire_extended_identifier_filter();
    [[maybe_unused]] auto mask_filters = manager->acquire_mask_filter();
    [[maybe_unused]] auto ext_mask_filter =
      manager->acquire_extended_mask_filter();
  }
};
}  // namespace hal::stm32f1

#pragma GCC diagnostic pop
