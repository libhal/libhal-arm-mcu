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

#pragma once

#include <array>
#include <span>
#include <type_traits>

#include <libhal/error.hpp>

/**
 * @defgroup Enum2 APIs involving enumerations
 *
 */
namespace hal::cortex_m {
/// Used specifically for defining an interrupt vector table of addresses.
using interrupt_pointer = void (*)();
using irq_t = std::int16_t;

/**
 * @ingroup Enum
 * @brief concept for enumeration types
 *
 * @tparam T - enum type
 */
template<typename T>
concept irq_enum =
  std::is_enum_v<T> && std::is_same_v<std::underlying_type_t<T>, irq_t>;

/**
 * @brief IRQ numbers for core processor interrupts
 *
 * All core IRQs are enabled by default.
 */
enum class irq : irq_t
{
  top_of_stack = -16,
  reset = -15,
  non_maskable_interrupt = -14,
  hard_fault = -13,
  memory_management_fault = -12,
  bus_fault = -11,
  usage_fault = -10,
  reserve7 = -9,
  reserve8 = -8,
  reserve9 = -7,
  reserve10 = -6,
  software_call = -5,
  reserve12 = -4,
  reserve13 = -3,
  pend_sv = -2,
  systick = -1,
};

constexpr auto core_interrupts = static_cast<irq_t>(irq::top_of_stack);

/**
 * @brief A default interrupt handler that loops forever
 *
 */
void default_interrupt_handler();

/**
 * @brief Default hard fault handler
 *
 * Used by developers in debug mode to see if they landed in the default hard
 * fault handler.
 *
 */
void hard_fault_handler();

/**
 * @brief Default memory management fault handler
 *
 * Used by developers in debug mode to see if they landed in the default memory
 * management fault handler.
 *
 */
void memory_management_fault_handler();

/**
 * @brief Default bus fault handler
 *
 * Used by developers in debug mode to see if they landed in the default bus
 * fault handler.
 *
 */
void bus_fault_handler();

/**
 * @brief Default usage fault handler
 *
 * Used by developers in debug mode to see if they landed in the default usage
 * fault handler.
 *
 */
void usage_fault_handler();

/**
 * @brief Sets the interrupt vector table back to a form where it can be
 * initialized again().
 *
 * Recommended to never call this function in your application. If it is called,
 * it should be well documented why this choice was necessary.
 *
 * Will clear all pending interrupts and sets the internal state to
 * uninitialized. This call is dangerous and should only be done before any
 * drivers depend on their interrupts to function.
 *
 */
void revert_interrupt_vector_table();

/**
 * @brief Initialize the interrupt vector table
 *
 * Using this function directly is not recommended. Use the templated version of
 * this in drivers and in application code. Only use this if you need to control
 * precisely where the interrupt vector table is located.
 *
 * If the input vector table has the same address as the previous vector table,
 * then this function does nothing.
 *
 * This function does the following:
 *
 * - Sets the default for hard_fault to `hard_fault_handler`
 * - Sets the default for memory_management_fault to
 *   `memory_management_fault_handler`
 * - Sets the default for bus_fault to `bus_fault_handler`
 * - Sets the default for usage_fault to `usage_fault_handler`
 * - Sets the default for everything else to `nop`
 * - Assign a global vector_table span to the the passed vector table.
 * - Relocates the system's interrupt vector table away from the hard coded
 *   vector table in ROM/Flash memory to the table passed in.
 *
 * All default functions contain an infinite loop and it is encouraged to swap
 * these out if you plan to use the interrupts.
 *
 * @param p_vector_table - must have length > 16 to accommodate the core
 * interrupts.
 */
void initialize_interrupts(std::span<interrupt_pointer> p_vector_table);

/**
 * @brief Initializes the interrupt vector table.
 *
 * This template function does the following:
 * - Statically allocates a 512-byte aligned an interrupt vector table the
 *   size of max_possible_irq.
 * - Calls the initialize_interrupts function with the array.
 *
 * Internally, this function checks if it has been called before and will
 * simply return early if so. Making this function safe to call multiple times
 * so long as the max_possible_irq template parameter is the same with each
 * invocation.
 *
 * Calling this function with differing max_possible_irq values will result in
 * multiple statically allocated interrupt vector tables, which will simply
 * waste space in RAM. Only the first call is used as the IVT.
 *
 * @tparam max_possible_irq - the number of interrupts available for this system
 */
template<irq_t max_possible_irq>
void initialize_interrupts()
{
  static_assert(max_possible_irq > 0,
                "Cannot initialize interrupts using a negative number. Please "
                "supply a number above 0.");

  // Statically allocate a buffer of vectors to be used as the new IVT.
  constexpr size_t total_vector_count = max_possible_irq - core_interrupts;

  alignas(512) static std::array<interrupt_pointer, total_vector_count>
    vector_buffer{};

  initialize_interrupts(vector_buffer);
}

/**
 * @brief Initializes the interrupt vector table.
 *
 * Performs the same work as:
 * template<size_t vector_count> initialize_interrupts(),
 * but accepts an enumeration class object.
 *
 * @tparam enum_vector_count - this parameter should always be set to the
 * `hal::platform::irq::max`.
 */
template<irq_enum auto max_possible_irq>
inline void initialize_interrupts()
{
  initialize_interrupts<static_cast<irq_t>(max_possible_irq)>();
}

/**
 * @brief Returns true if the interrupt vector table has been initialized
 *
 * This API should NOT be called by platform peripheral drivers. Those drivers
 * should call `initialize_interrupts<irq::max>()`. That API will do the check
 * for you and will either return early if interrupts are already enabled. There
 * is no reason to call this API directly then call
 * `initialize_interrupts<irq::max>()`. That is unecessary cycles.
 *
 * This API should be used by cortex m drivers like systick, which cannot know
 * the size of the interrupt vector table as it is different for each
 * microcontroller.
 *
 * @return true - interrupt vector table is initialized
 * @return false - interrupt vector table is not initialized
 */
bool interrupt_vector_table_initialized();

/**
 * @brief Disable all interrupts
 *
 */
void disable_all_interrupts();

/**
 * @brief Re-enable all interrupts
 *
 */
void enable_all_interrupts();

/**
 * @brief Get a reference to interrupt vector table object
 *
 * @return const std::span<interrupt_pointer> - interrupt vector table
 */
std::span<interrupt_pointer> const get_vector_table();

/**
 * @brief Enable interrupt and set the service routine handler.
 *
 * If the irq is not valid, meaning it is outside of the range of the interrupt
 * vector table, then nothing happens.
 *
 * Using this with a core cortex-m interrupt will assign its handler.
 *
 * @param p_irq - irq to enable. If the value is beyond the range of the
 * interrupt vector table, then this function does nothing.
 * @param p_handler - the interrupt service routine handler to be executed
 * when the hardware interrupt is fired.
 */
void enable_interrupt(irq_t p_irq, interrupt_pointer p_handler);

/**
 * @brief Enable interrupt and set the service routine handler.
 *
 * Performs the same work as `enable_interrupt` using the `irq_t` type, but
 * allows enum class types to be passed.
 *
 * @param p_irq - enumeration typed irq number
 * @param p_handler - interrupt handler
 */
inline void enable_interrupt(irq_enum auto p_irq, interrupt_pointer p_handler)
{
  enable_interrupt(static_cast<irq_t>(p_irq), p_handler);
}

/**
 * @brief disable interrupt and set the service routine handler to their
 * default.
 *
 * This function does nothing if the vector table has not yet been initialized.
 *
 * @param p_irq - irq to disable, if the value is below 0 or out of range then
 * this function does nothing.
 */
void disable_interrupt(irq_t p_irq);

/**
 * @brief Disable interrupt and set the service routine handler.
 *
 * Performs the same work as `disable_interrupt` using the `irq_t` type, but
 * allows enum class types to be passed.
 *
 * @param p_irq - enumeration typed irq number
 */
inline void disable_interrupt(irq_enum auto p_irq)
{
  disable_interrupt(static_cast<irq_t>(p_irq));
}

/**
 * @brief determine if a particular handler has been put into the interrupt
 * vector table.
 *
 * Generally used by unit testing code.
 *
 * @param p_irq - irq to check
 * @param p_handler - the handler to check against. The address of the handler
 * must match what is in the interrupt vector table.
 * @return true - the handler is equal to the handler in the table
 * @return false - the handler is not at this index in the table or p_irq is not
 * valid.
 */
[[nodiscard]] bool verify_vector_enabled(irq_t p_irq,
                                         interrupt_pointer p_handler);

/**
 * @brief determine if a particular handler has been put into the interrupt
 * vector table.
 *
 * Generally used by unit testing code.
 *
 * @param p_irq - irq to check
 * @param p_handler - the handler to check against
 * @return true - the handler is equal to the handler in the table
 * @return false - the handler is not at this index in the table or p_irq is not
 * valid.
 */
[[nodiscard]] inline bool verify_vector_enabled(irq_enum auto p_irq,
                                                interrupt_pointer p_handler)
{
  return verify_vector_enabled(static_cast<irq_t>(p_irq), p_handler);
}
}  // namespace hal::cortex_m
