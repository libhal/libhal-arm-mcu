// Copyright 2024 Khalil Estell
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

#include <libhal-armcortex/interrupt.hpp>

#include <libhal-armcortex/system_control.hpp>

#include "helper.hpp"

#include <boost/ut.hpp>
#include <libhal-util/enum.hpp>

namespace hal::cortex_m {

namespace {
// NOLINTNEXTLINE(performance-enum-size): the actual size of these types are u16
enum class my_irq : irq_t
{
  uart0 = 55,
  spi7 = 63,
  max,
};
}  // namespace

boost::ut::suite test_interrupts = []() {
  using namespace boost::ut;

  auto saved_registers = setup_interrupts_for_unit_testing();

  expect(that % hal::value(irq::top_of_stack) == core_interrupts);

  should("initialize_interrupt()") = [&] {
    // Setup
    expect(that % nullptr == get_vector_table().data());
    expect(that % 0 == get_vector_table().size());

    // Exercise
    initialize_interrupts<my_irq::max>();

    auto pointer = reinterpret_cast<intptr_t>(get_vector_table().data());

    // Verify
    expect(that % nullptr != get_vector_table().data());
    expect(that % static_cast<std::size_t>(my_irq::max) ==
           get_vector_table().size());
    expect(that % (pointer + (sizeof(std::uintptr_t) * core_interrupts)) ==
           scb->vtor);

    // Verify: Nothing in the interrupt vector table should have changed
    auto* top_of_stack_expected = reinterpret_cast<void*>(fake_top_of_stack);
    auto* reset_handler_expected = reinterpret_cast<void*>(fake_reset_handler);

    auto ivt_0 = get_vector_table()[-16];
    auto ivt_1 = get_vector_table()[-15];
    auto top_of_stack_actual = reinterpret_cast<void*>(ivt_0);
    auto reset_handler_actual = reinterpret_cast<void*>(ivt_1);

    expect(that % top_of_stack_expected == top_of_stack_actual);
    expect(that % reset_handler_expected == reset_handler_actual);

    for (auto const interrupt_function : get_vector_table().subspan(2)) {
      auto default_interrupt_handler_address =
        reinterpret_cast<void*>(&default_interrupt_handler);
      auto function_address = reinterpret_cast<void*>(interrupt_function);
      expect(that % default_interrupt_handler_address == function_address);
    }
  };

  should("enable_interrupt()") = [&] {
    interrupt_pointer dummy_handler = +[]() {};

    should("enable_interrupt(21)") = [&]() {
      // Setup
      static constexpr std::uint16_t event_number = 21;
      unsigned index = event_number >> 5;
      unsigned bit_position = event_number & 0x1F;

      // Exercise
      enable_interrupt(event_number, dummy_handler);

      // Verify
      expect(verify_vector_enabled(event_number, dummy_handler))
        << "event number " << event_number
        << " did not have the appropriate handler (" << &dummy_handler
        << ") installed";
      std::uint32_t iser = (1U << bit_position) | nvic->iser.at(index);
      interrupt_pointer expected_handler = dummy_handler;
      interrupt_pointer actual_handler = get_vector_table()[event_number];

      expect(that % expected_handler == actual_handler);
      expect(that % (1 << event_number) == iser);
    };

    should("enable_interrupt(my_enum::uart0)") = [&]() {
      // Setup
      static constexpr auto event_number = my_irq::uart0;
      unsigned index = static_cast<irq_t>(event_number) >> 5;
      unsigned bit_position = static_cast<irq_t>(event_number) & 0x1F;

      // Exercise
      enable_interrupt(event_number, dummy_handler);

      // Verify
      expect(verify_vector_enabled(event_number, dummy_handler))
        << "event number " << static_cast<irq_t>(event_number)
        << " did not have the appropriate handler (" << &dummy_handler
        << ") installed";
      expect(that % dummy_handler ==
             get_vector_table()[static_cast<irq_t>(event_number)]);
      std::uint32_t iser = (1U << bit_position) | nvic->iser.at(index);
      expect(that % (1 << bit_position) == iser);
    };

    should("enable_interrupt(-5)") = [&]() {
      // Setup
      static constexpr irq_t event_number = -5;
      auto const old_nvic = *nvic;

      // Exercise
      enable_interrupt(event_number, dummy_handler);

      // Verify
      // Verify: That the dummy handler was added to the IVT (ISER)
      expect(verify_vector_enabled(event_number, dummy_handler))
        << "event number " << event_number
        << " did not have the appropriate handler (" << &dummy_handler
        << ") installed";

      // Verify: ISER[] should not have changed when enable() succeeds but the
      // IRQ is less than 0.
      for (size_t i = 0; i < old_nvic.iser.size(); i++) {
        expect(that % old_nvic.iser.at(i) == nvic->iser.at(i));
      }
    };

    should("enable_interrupt(100) fail") = [&]() {
      // Setup
      // Setup: Re-initialize interrupts which will set each vector to "nop"
      revert_interrupt_vector_table();
      initialize_interrupts<my_irq::max>();

      static constexpr std::uint16_t event_number = 100;
      auto const old_nvic = *nvic;

      // Exercise
      enable_interrupt(event_number, dummy_handler);

      // Verify: that the state of the IVT and enable registers has not changed.
      auto top_of_stack = get_vector_table()[hal::value(irq::top_of_stack)];
      auto reset = get_vector_table()[hal::value(irq::reset)];
      auto non_maskable_interrupt =
        get_vector_table()[hal::value(irq::non_maskable_interrupt)];
      auto hard_fault = get_vector_table()[hal::value(irq::hard_fault)];
      auto memory_management_fault =
        get_vector_table()[hal::value(irq::memory_management_fault)];
      auto bus_fault = get_vector_table()[hal::value(irq::bus_fault)];
      auto usage_fault = get_vector_table()[hal::value(irq::usage_fault)];
      auto reserve7 = get_vector_table()[hal::value(irq::reserve7)];
      auto reserve8 = get_vector_table()[hal::value(irq::reserve8)];
      auto reserve9 = get_vector_table()[hal::value(irq::reserve9)];
      auto reserve10 = get_vector_table()[hal::value(irq::reserve10)];
      auto software_call = get_vector_table()[hal::value(irq::software_call)];
      auto reserve12 = get_vector_table()[hal::value(irq::reserve12)];
      auto reserve13 = get_vector_table()[hal::value(irq::reserve13)];
      auto pend_sv = get_vector_table()[hal::value(irq::pend_sv)];
      auto systick = get_vector_table()[hal::value(irq::systick)];

      expect(that % reinterpret_cast<void*>(&fake_top_of_stack) ==
             reinterpret_cast<void*>(top_of_stack));
      expect(that % reinterpret_cast<void*>(&fake_reset_handler) ==
             reinterpret_cast<void*>(reset));
      expect(that % reinterpret_cast<void*>(&default_interrupt_handler) ==
             reinterpret_cast<void*>(non_maskable_interrupt));
      expect(that % reinterpret_cast<void*>(&hard_fault_handler) ==
             reinterpret_cast<void*>(hard_fault));
      expect(that % reinterpret_cast<void*>(&usage_fault_handler) ==
             reinterpret_cast<void*>(usage_fault));
      expect(that % reinterpret_cast<void*>(&bus_fault_handler) ==
             reinterpret_cast<void*>(bus_fault));
      expect(that % reinterpret_cast<void*>(&memory_management_fault_handler) ==
             reinterpret_cast<void*>(memory_management_fault));

      expect(that % &default_interrupt_handler ==
             reinterpret_cast<void*>(reserve7));
      expect(that % &default_interrupt_handler ==
             reinterpret_cast<void*>(reserve8));
      expect(that % &default_interrupt_handler ==
             reinterpret_cast<void*>(reserve9));
      expect(that % &default_interrupt_handler ==
             reinterpret_cast<void*>(reserve10));
      expect(that % &default_interrupt_handler ==
             reinterpret_cast<void*>(software_call));
      expect(that % &default_interrupt_handler ==
             reinterpret_cast<void*>(reserve12));
      expect(that % &default_interrupt_handler ==
             reinterpret_cast<void*>(reserve13));
      expect(that % &default_interrupt_handler ==
             reinterpret_cast<void*>(pend_sv));
      expect(that % &default_interrupt_handler ==
             reinterpret_cast<void*>(systick));

      for (auto const interrupt_function : get_vector_table()) {
        expect(that % (void*)&default_interrupt_handler ==
               (void*)interrupt_function);
      }

      // Verify: ISER[] should not have changed when enable() fails.
      for (size_t i = 0; i < old_nvic.iser.size(); i++) {
        expect(old_nvic.iser.at(i) == nvic->iser.at(i));
      }
    };
  };

  should("disable_interrupt(event_number)") = [&] {
    should("disable_interrupt(21)") = [&]() {
      // Setup
      static constexpr std::uint16_t event_number = 21;
      unsigned index = event_number >> 5;
      unsigned bit_position = event_number & 0x1F;

      // Exercise
      disable_interrupt(event_number);

      // Verify
      std::uint32_t icer = (1U << bit_position) | nvic->icer.at(index);
      expect(that % (1 << event_number) == icer);
    };

    should("disable_interrupt(my_irq::uart0)") = [&]() {
      // Setup
      static constexpr auto event_number = my_irq::uart0;
      unsigned index = static_cast<irq_t>(event_number) >> 5;
      unsigned bit_position = static_cast<irq_t>(event_number) & 0x1F;

      // Exercise
      disable_interrupt(event_number);

      // Verify
      std::uint32_t icer = (1U << bit_position) | nvic->icer.at(index);
      expect(that % (1 << bit_position) == icer);
    };

    should("disable_interrupt(my_irq::spi7)") = [&]() {
      // Setup
      static constexpr auto event_number = my_irq::uart0;
      unsigned index = static_cast<irq_t>(event_number) >> 5;
      unsigned bit_position = static_cast<irq_t>(event_number) & 0x1F;

      // Exercise
      disable_interrupt(event_number);

      // Verify
      std::uint32_t icer = (1U << bit_position) | nvic->icer.at(index);
      expect(that % (1 << bit_position) == icer);
    };

    should("disable_interrupt(-5)") = [&]() {
      // Setup
      static constexpr std::int16_t event_number = -5;
      auto const old_nvic = *nvic;

      // Exercise
      disable_interrupt(event_number);

      // Verify
      // Verify: icer[] should not have changed when disable() succeeds but the
      // IRQ is less than 0.
      for (size_t i = 0; i < old_nvic.icer.size(); i++) {
        expect(old_nvic.icer.at(i) == nvic->icer.at(i));
      }
    };

    should("disable_interrupt(100) fail") = [&]() {
      // Setup
      // Setup: Re-initialize interrupts which will set each vector to "nop"
      revert_interrupt_vector_table();
      initialize_interrupts<my_irq::max>();
      static constexpr int event_number = 100;
      auto const old_nvic = *nvic;

      // Exercise
      disable_interrupt(event_number);

      // Verify: that the state of the IVT and enable registers has not changed.
      auto top_of_stack = get_vector_table()[hal::value(irq::top_of_stack)];
      auto reset = get_vector_table()[hal::value(irq::reset)];
      auto non_maskable_interrupt =
        get_vector_table()[hal::value(irq::non_maskable_interrupt)];
      auto hard_fault = get_vector_table()[hal::value(irq::hard_fault)];
      auto memory_management_fault =
        get_vector_table()[hal::value(irq::memory_management_fault)];
      auto bus_fault = get_vector_table()[hal::value(irq::bus_fault)];
      auto usage_fault = get_vector_table()[hal::value(irq::usage_fault)];
      auto reserve7 = get_vector_table()[hal::value(irq::reserve7)];
      auto reserve8 = get_vector_table()[hal::value(irq::reserve8)];
      auto reserve9 = get_vector_table()[hal::value(irq::reserve9)];
      auto reserve10 = get_vector_table()[hal::value(irq::reserve10)];
      auto software_call = get_vector_table()[hal::value(irq::software_call)];
      auto reserve12 = get_vector_table()[hal::value(irq::reserve12)];
      auto reserve13 = get_vector_table()[hal::value(irq::reserve13)];
      auto pend_sv = get_vector_table()[hal::value(irq::pend_sv)];
      auto systick = get_vector_table()[hal::value(irq::systick)];

      expect(that % reinterpret_cast<void*>(&fake_top_of_stack) ==
             reinterpret_cast<void*>(top_of_stack));
      expect(that % reinterpret_cast<void*>(&fake_reset_handler) ==
             reinterpret_cast<void*>(reset));
      expect(that % reinterpret_cast<void*>(&default_interrupt_handler) ==
             reinterpret_cast<void*>(non_maskable_interrupt));
      expect(that % reinterpret_cast<void*>(&hard_fault_handler) ==
             reinterpret_cast<void*>(hard_fault));
      expect(that % reinterpret_cast<void*>(&usage_fault_handler) ==
             reinterpret_cast<void*>(usage_fault));
      expect(that % reinterpret_cast<void*>(&bus_fault_handler) ==
             reinterpret_cast<void*>(bus_fault));
      expect(that % reinterpret_cast<void*>(&memory_management_fault_handler) ==
             reinterpret_cast<void*>(memory_management_fault));

      expect(that % &default_interrupt_handler ==
             reinterpret_cast<void*>(reserve7));
      expect(that % &default_interrupt_handler ==
             reinterpret_cast<void*>(reserve8));
      expect(that % &default_interrupt_handler ==
             reinterpret_cast<void*>(reserve9));
      expect(that % &default_interrupt_handler ==
             reinterpret_cast<void*>(reserve10));
      expect(that % &default_interrupt_handler ==
             reinterpret_cast<void*>(software_call));
      expect(that % &default_interrupt_handler ==
             reinterpret_cast<void*>(reserve12));
      expect(that % &default_interrupt_handler ==
             reinterpret_cast<void*>(reserve13));
      expect(that % &default_interrupt_handler ==
             reinterpret_cast<void*>(pend_sv));
      expect(that % &default_interrupt_handler ==
             reinterpret_cast<void*>(systick));

      for (auto const interrupt_function : get_vector_table()) {
        expect(that % (void*)&default_interrupt_handler ==
               (void*)interrupt_function);
      }

      // Verify: ISER[] should not have changed when enable() fails.
      for (size_t i = 0; i < old_nvic.iser.size(); i++) {
        expect(old_nvic.iser.at(i) == nvic->iser.at(i));
      }
    };
  };

  should("get_vector_table()") = [&] {
    // Setup
    expect(that % nullptr != get_vector_table().data());
    expect(that % 0 != get_vector_table().size());

    // Exercise & Verify
    expect(get_vector_table().data() == get_vector_table().data());
    expect(that % get_vector_table().size() == get_vector_table().size());
  };

  should("interrupt_vector_table_initialized()") = [&] {
    expect(interrupt_vector_table_initialized());
  };

  should("disable_all_interrupts() & enable_all_interrupts() works") = [&] {
    // these are empty on host builds as the instruction for these will not work
    // any host that isn't a cortex-m mcu.
    disable_all_interrupts();
    enable_all_interrupts();
  };
};
}  // namespace hal::cortex_m
