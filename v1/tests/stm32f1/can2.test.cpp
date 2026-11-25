// Copyright 2024 - 2025 Khalil Estell and the libhal contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
