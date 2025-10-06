#include <libhal-arm-mcu/stm32f1/can2.hpp>

#include <boost/ut.hpp>

namespace hal::stm32f1 {
namespace {
bool volatile skip = true;
}

boost::ut::suite can2_test = []() {
  // This code forces the compiler to attempt to link in all of the public APIs
  // from can2.hpp
  if (not skip) {
    std::pmr::polymorphic_allocator<> alloc;
    auto manager = hal::v5::make_strong_ptr<can_peripheral_manager_v2>(
      alloc,
      8,
      alloc,
      100'000,
      *reinterpret_cast<hal::steady_clock*>(0x1000'0000));

    manager->bus_on();
    auto filter = manager->available_filter();
    manager->release_filter(filter);
    manager->enable_self_test(true);
    manager->baud_rate(100'000);
    [[maybe_unused]] auto baud = manager->baud_rate();
    manager->send({});
    manager->on_receive({});
    [[maybe_unused]] auto buffer = manager->receive_buffer();
    [[maybe_unused]] auto cursor = manager->receive_cursor();

    [[maybe_unused]] auto transceiver = acquire_can_transceiver(alloc, manager);
    [[maybe_unused]] auto bus_manager = acquire_can_bus_manager(alloc, manager);
    [[maybe_unused]] auto interrupt = acquire_can_interrupt(alloc, manager);
    [[maybe_unused]] auto id_filters =
      acquire_can_identifier_filter(alloc, manager);
    [[maybe_unused]] auto ext_id_filters =
      acquire_can_extended_identifier_filter(alloc, manager);
    [[maybe_unused]] auto mask_filters =
      acquire_can_mask_filter(alloc, manager);
    [[maybe_unused]] auto ext_mask_filter =
      acquire_can_extended_mask_filter(alloc, manager);
  }
};
}  // namespace hal::stm32f1
