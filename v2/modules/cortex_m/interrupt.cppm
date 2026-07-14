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

module;

#include <algorithm>
#include <array>
#include <cstdint>
#include <span>
#include <type_traits>

export module hal.arm_mcu.cortex_m:interrupt;

import hal;
import hal.util;

import :system_control;

/**
 * @defgroup Enum APIs involving enumerations
 *
 */
namespace hal::cortex_m {
/// Used specifically for defining an interrupt vector table of addresses.
export using interrupt_pointer = void (*)();
export using irq_t = i16;

/**
 * @ingroup Enum
 * @brief concept for enumeration types
 *
 * @tparam T - enum type
 */
export template<typename T>
concept irq_enum =
  std::is_enum_v<T> && std::is_same_v<std::underlying_type_t<T>, irq_t>;

// NOLINTBEGIN(performance-enum-size): This enum represents a 16-bit IRQ number
// in ARM Cortex-M.
/**
 * @brief IRQ numbers for core processor interrupts
 *
 * All core IRQs are enabled by default.
 */
export enum class irq : irq_t {
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
// NOLINTEND(performance-enum-size)

export constexpr auto core_interrupts = static_cast<irq_t>(irq::top_of_stack);

/// Structure type to access the Nested Vectored Interrupt Controller (NVIC)
struct nvic_register_t
{
  /// Offset: 0x000 (R/W)  Interrupt Set Enable Register
  std::array<u32 volatile, 8U> iser;
  /// Reserved 0
  std::array<u32, 24U> reserved0;
  /// Offset: 0x080 (R/W)  Interrupt Clear Enable Register
  std::array<u32 volatile, 8U> icer;
  /// Reserved 1
  std::array<u32, 24U> reserved1;
  /// Offset: 0x100 (R/W)  Interrupt Set Pending Register
  std::array<u32 volatile, 8U> ispr;
  /// Reserved 2
  std::array<u32, 24U> reserved2;
  /// Offset: 0x180 (R/W)  Interrupt Clear Pending Register
  std::array<u32 volatile, 8U> icpr;
  /// Reserved 3
  std::array<u32, 24U> reserved3;
  /// Offset: 0x200 (R/W)  Interrupt Active bit Register
  std::array<u32 volatile, 8U> iabr;
  /// Reserved 4
  std::array<u32, 56U> reserved4;
  /// Offset: 0x300 (R/W)  Interrupt Priority Register (8Bit wide)
  std::array<u8 volatile, 240U> ip;
  /// Reserved 5
  std::array<u32, 644U> reserved5;
  /// Offset: 0xE00 ( /W)  Software Trigger Interrupt Register
  u32 volatile stir;
};

/// NVIC address
constexpr auto nvic_address = static_cast<uptr>(0xE000'E100UL);

// NOLINTNEXTLINE(performance-no-int-to-ptr)
auto* nvic = reinterpret_cast<nvic_register_t*>(nvic_address);

/// Pointer to a statically allocated interrupt vector table
std::span<interrupt_pointer> vector_table{};

std::int32_t register_index(irq_t p_irq)
{
  constexpr irq_t register_width = 32;
  return p_irq / register_width;
}

void nvic_enable_irq(irq_t p_irq)
{
  auto* interrupt_enable = &nvic->iser[register_index(p_irq)];
  *interrupt_enable = 1 << p_irq;
}

void nvic_disable_irq(irq_t p_irq)
{
  auto* interrupt_clear = &nvic->icer[register_index(p_irq)];
  *interrupt_clear = 1 << p_irq;
}

/**
 * @brief A default interrupt handler that loops forever
 *
 */
export void default_interrupt_handler()
{
  while (true) {
    continue;
  }
}

/**
 * @brief Default hard fault handler
 *
 * Used by developers in debug mode to see if they landed in the default hard
 * fault handler.
 *
 */
export void hard_fault_handler()
{
  while (true) {
    continue;
  }
}

/**
 * @brief Default memory management fault handler
 *
 * Used by developers in debug mode to see if they landed in the default
 * memory management fault handler.
 *
 */
export void memory_management_fault_handler()
{
  while (true) {
    continue;
  }
}

/**
 * @brief Default bus fault handler
 *
 * Used by developers in debug mode to see if they landed in the default bus
 * fault handler.
 *
 */
export void bus_fault_handler()
{
  while (true) {
    continue;
  }
}

/**
 * @brief Default usage fault handler
 *
 * Used by developers in debug mode to see if they landed in the default
 * usage fault handler.
 *
 */
export void usage_fault_handler()
{
  while (true) {
    continue;
  }
}

/**
 * @brief Disable all interrupts
 *
 */
export void disable_all_interrupts()
{
#if defined(__arm__)
  asm volatile("cpsid i" : : : "memory");
#endif
}

/**
 * @brief Re-enable all interrupts
 *
 */
export void enable_all_interrupts()
{
#if defined(__arm__)
  asm volatile("cpsie i" : : : "memory");
#endif
}

/**
 * @brief Returns true if the interrupt vector table has been initialized
 *
 * This API should NOT be called by platform peripheral drivers. Those
 * drivers should call `initialize_interrupts<irq::max>()`. That API will do
 * the check for you and will either return early if interrupts are already
 * enabled. There is no reason to call this API directly then call
 * `initialize_interrupts<irq::max>()`. That is unecessary cycles.
 *
 * This API should be used by cortex m drivers like systick, which cannot
 * know the size of the interrupt vector table as it is different for each
 * microcontroller.
 *
 * @return true - interrupt vector table is initialized
 * @return false - interrupt vector table is not initialized
 */
export bool interrupt_vector_table_initialized()
{
  auto* vector_base = reinterpret_cast<void*>(&vector_table[core_interrupts]);

  return get_interrupt_vector_table_address() == vector_base;
}

bool is_valid_irq_request(irq_t p_irq)
{
  if (not interrupt_vector_table_initialized()) {
    return false;
  }

  bool within_bounds = hal::value(irq::top_of_stack) <= p_irq &&
                       p_irq <= static_cast<irq_t>(vector_table.size());

  if (not within_bounds) {
    return false;
  }

  return true;
}

/**
 * @brief Get a reference to interrupt vector table object
 *
 * @return const std::span<interrupt_pointer> - interrupt vector table
 */
export std::span<interrupt_pointer> const get_vector_table()
{
  return vector_table;
}

/**
 * @brief Enable interrupt and set the service routine handler.
 *
 * If the irq is not valid, meaning it is outside of the range of the
 * interrupt vector table, then nothing happens.
 *
 * Using this with a core cortex-m interrupt will assign its handler.
 *
 * @param p_irq - irq to enable. If the value is beyond the range of the
 * interrupt vector table, then this function does nothing.
 * @param p_handler - the interrupt service routine handler to be executed
 * when the hardware interrupt is fired.
 */
export void enable_interrupt(irq_t p_irq, interrupt_pointer p_handler)
{
  if (not is_valid_irq_request(p_irq)) {
    return;
  }

  vector_table[p_irq] = p_handler;

  if (p_irq >= 0) {
    nvic_enable_irq(p_irq);
  }
}

/**
 * @brief Enable interrupt and set the service routine handler.
 *
 * Performs the same work as `enable_interrupt` using the `irq_t` type, but
 * allows enum class types to be passed.
 *
 * @param p_irq - enumeration typed irq number
 * @param p_handler - interrupt handler
 */
export inline void enable_interrupt(irq_enum auto p_irq,
                                    interrupt_pointer p_handler)
{
  enable_interrupt(static_cast<irq_t>(p_irq), p_handler);
}

/**
 * @brief disable interrupt and set the service routine handler to their
 * default.
 *
 * This function does nothing if the vector table has not yet been
 * initialized.
 *
 * @param p_irq - irq to disable, if the value is below 0 or out of range
 * then this function does nothing.
 */
export void disable_interrupt(irq_t p_irq)
{
  if (!is_valid_irq_request(p_irq)) {
    return;
  }

  if (p_irq < 0) {
    return;
  }

  nvic_disable_irq(p_irq);
}

/**
 * @brief Disable interrupt and set the service routine handler.
 *
 * Performs the same work as `disable_interrupt` using the `irq_t` type, but
 * allows enum class types to be passed.
 *
 * @param p_irq - enumeration typed irq number
 */
export inline void disable_interrupt(irq_enum auto p_irq)
{
  disable_interrupt(static_cast<irq_t>(p_irq));
}

/**
 * @brief determine if a particular interrupt has been enabled.
 *
 * This is only to determine if that particular entry is enabled, it does
 * not care about what handler is inside.
 *
 * @param p_irq - irq to check.
 * @return true - the interrupt has been enabled.
 * @return false - the interrupt is disabled or is invalid.
 */
export [[nodiscard]] bool is_interrupt_enabled(irq_t p_irq)
{
  if (!is_valid_irq_request(p_irq)) {
    return false;
  }

  if (p_irq < 0) {
    return true;
  }

  u32 enable_register = nvic->iser[register_index(p_irq)];

  return (enable_register & (1 << p_irq)) != 0U;
}

/**
 * @brief determine if a particular interrupt has been enabled.
 *
 * This is only to determine if that particular entry is enabled, it does
 * not care about what handler is inside.
 *
 * @param p_irq - irq to check.
 * @return true - the interrupt has been enabled.
 * @return false - the interrupt is disabled or is invalid.
 */
export [[nodiscard]] inline bool is_interrupt_enabled(irq_enum auto p_irq)
{
  return is_interrupt_enabled(static_cast<irq_t>(p_irq));
}

/**
 * @brief determine if a particular handler has been put into the interrupt
 * vector table.
 *
 * Generally used by unit testing code.
 *
 * @param p_irq - irq to check
 * @param p_handler - the handler to check against. The address of the
 * handler must match what is in the interrupt vector table.
 * @return true - the handler is equal to the handler in the table
 * @return false - the handler is not at this index in the table or p_irq is
 * not valid.
 */
export [[nodiscard]] bool verify_vector_enabled(irq_t p_irq,
                                                interrupt_pointer p_handler)
{
  if (!is_valid_irq_request(p_irq)) {
    return false;
  }

  // Check if the handler match
  auto irq_handler = vector_table[p_irq];
  bool handlers_are_the_same = (irq_handler == p_handler);

  if (not handlers_are_the_same) {
    return false;
  }

  if (p_irq < 0) {
    return true;
  }

  u32 enable_register = nvic->iser[register_index(p_irq)];

  return (enable_register & (1 << p_irq)) != 0U;
}

/**
 * @brief determine if a particular handler has been put into the interrupt
 * vector table.
 *
 * Generally used by unit testing code.
 *
 * @param p_irq - irq to check
 * @param p_handler - the handler to check against
 * @return true - the handler is equal to the handler in the table
 * @return false - the handler is not at this index in the table or p_irq is
 * not valid.
 */
export [[nodiscard]] inline bool verify_vector_enabled(
  irq_enum auto p_irq,
  interrupt_pointer p_handler)
{
  return verify_vector_enabled(static_cast<irq_t>(p_irq), p_handler);
}

namespace {
void setup_default_vector_table(std::span<interrupt_pointer> p_vector_table)
{
  // Move vector span table forward so the negative irq numbers reach the
  // correct array elements.
  p_vector_table = p_vector_table.subspan(-core_interrupts);

  auto* table_reg = get_interrupt_vector_table_address();
  auto* original_stack = reinterpret_cast<interrupt_pointer*>(table_reg)[0];
  auto* original_reset = reinterpret_cast<interrupt_pointer*>(table_reg)[1];

  p_vector_table[hal::value(irq::top_of_stack)] = original_stack;
  p_vector_table[hal::value(irq::reset)] = original_reset;
  p_vector_table[hal::value(irq::non_maskable_interrupt)] =
    default_interrupt_handler;
  p_vector_table[hal::value(irq::hard_fault)] = hard_fault_handler;
  p_vector_table[hal::value(irq::memory_management_fault)] =
    memory_management_fault_handler;
  p_vector_table[hal::value(irq::bus_fault)] = bus_fault_handler;
  p_vector_table[hal::value(irq::usage_fault)] = usage_fault_handler;
  p_vector_table[hal::value(irq::reserve7)] = default_interrupt_handler;
  p_vector_table[hal::value(irq::reserve8)] = default_interrupt_handler;
  p_vector_table[hal::value(irq::reserve9)] = default_interrupt_handler;
  p_vector_table[hal::value(irq::reserve10)] = default_interrupt_handler;
  p_vector_table[hal::value(irq::software_call)] = default_interrupt_handler;
  p_vector_table[hal::value(irq::reserve12)] = default_interrupt_handler;
  p_vector_table[hal::value(irq::reserve13)] = default_interrupt_handler;
  p_vector_table[hal::value(irq::pend_sv)] = default_interrupt_handler;
  p_vector_table[hal::value(irq::systick)] = default_interrupt_handler;

  // Fill the interrupt handler and vector table with a function that does
  // nothing functions.
  std::ranges::fill(p_vector_table, &default_interrupt_handler);
}

constexpr bool is_the_same_vector_buffer(
  std::span<interrupt_pointer> p_vector_table)
{
  p_vector_table = p_vector_table.subspan(-core_interrupts);
  return (p_vector_table.data() == vector_table.data() &&
          p_vector_table.size() == vector_table.size());
}
}  // namespace

/**
 * @brief Sets the interrupt vector table back to a form where it can be
 * initialized again().
 *
 * Recommended to never call this function in your application. If it is
 * called, it should be well documented why this choice was necessary.
 *
 * Will clear all pending interrupts and sets the internal state to
 * uninitialized. This call is dangerous and should only be done before any
 * drivers depend on their interrupts to function.
 *
 */
export void revert_interrupt_vector_table()
{
  disable_all_interrupts();

  // Set all bits in the interrupt clear register to 1s to disable those
  // interrupt vectors.
  for (auto& clear_interrupt : nvic->icer) {
    clear_interrupt = 0xFFFF'FFFF;
  }

  // Reset vector table
  vector_table = std::span<interrupt_pointer>();
}

/**
 * @brief Initialize the interrupt vector table
 *
 * Using this function directly is not recommended. Use the templated
 * version of this in drivers and in application code. Only use this if you
 * need to control precisely where the interrupt vector table is located.
 *
 * If the input vector table has the same address as the previous vector
 * table, then this function does nothing.
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
 * All default functions contain an infinite loop and it is encouraged to
 * swap these out if you plan to use the interrupts.
 *
 * @param p_vector_table - must have length > 16 to accommodate the core
 * interrupts.
 */
export void initialize_interrupts(std::span<interrupt_pointer> p_vector_table)
{
  // If initialize function has already been called before with this same
  // buffer, return early.
  if (is_the_same_vector_buffer(p_vector_table)) {
    return;
  }

  setup_default_vector_table(p_vector_table);

  disable_all_interrupts();

  // Assign the vector within this scope to the global vector_table span so
  // that it can be accessed in other functions. This is valid because the
  // interrupt vector table has static storage duration and will exist
  // throughout the duration of the application.
  vector_table = p_vector_table.subspan(-core_interrupts);

  // Relocate the interrupt vector table the vector buffer. By default this
  // will be set to the address of the start of flash memory for the MCU.
  set_interrupt_vector_table_address(
    reinterpret_cast<void*>(p_vector_table.data()));

  enable_all_interrupts();
}

/**
 * @brief Initializes the interrupt vector table.
 *
 * This template function does the following:
 * - Statically allocates a 512-byte aligned an interrupt vector table the
 *   size of max_possible_irq.
 * - Calls the initialize_interrupts function with the array.
 *
 * Internally, this function checks if it has been called before and will
 * simply return early if so. Making this function safe to call multiple
 * times so long as the max_possible_irq template parameter is the same
 * with each invocation.
 *
 * Calling this function with differing max_possible_irq values will result
 * in multiple statically allocated interrupt vector tables, which will
 * simply waste space in RAM. Only the first call is used as the IVT.
 *
 * @tparam max_possible_irq - the number of interrupts available for this
 * system
 */
export template<irq_t max_possible_irq>
void initialize_interrupts()
{
  static_assert(max_possible_irq > 0,
                "Cannot initialize interrupts using a negative number. Please "
                "supply a number above 0.");

  // Statically allocate a buffer of vectors to be used as the new IVT.
  constexpr hal::usize total_vector_count = max_possible_irq - core_interrupts;

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
export template<irq_enum auto max_possible_irq>
inline void initialize_interrupts()
{
  initialize_interrupts<static_cast<irq_t>(max_possible_irq)>();
}
}  // namespace hal::cortex_m

// Single version for ALL Cortex-M processors
extern "C"
{
  /**
   * @brief Hard fault handler that gracefully handles semihosting breakpoints
   *
   * This handler overrides picolibc's default hard fault handler to detect
   * and skip semihosting BKPT instructions when no debugger is attached.
   * Without this handler, applications linked with semihosting libraries
   * will hang in an infinite loop when executed standalone.
   *
   * The handler checks if a hard fault was caused by a debug event (BKPT
   * instruction). If so, it:
   * 1. Clears the hard fault status
   * 2. Advances the program counter past the BKPT instruction (2 bytes)
   * 3. Sets R0 to -1 to indicate the semihosting operation failed
   * 4. Returns to continue execution
   *
   * If the fault was caused by something other than a BKPT, the handler
   * enters an infinite loop to halt execution, indicating a real hard fault
   * condition.
   *
   * This implementation is based on SEGGER's hard fault handler reference:
   * https://kb.segger.com/Arm_Cortex-M_interrupts
   *
   * Modified to use ARMv6-M (Cortex-M0) compatible instructions, making it
   * compatible with all Cortex-M variants
   * (M0/M0+/M1/M3/M4/M7/M23/M33/M55/M85).
   *
   * @note This handler is wrapped via linker flag -Wl,--wrap=arm_hardfault_isr
   *       to override picolibc's default implementation.
   */
  __attribute__((naked)) inline void __wrap_arm_hardfault_isr(void)  // NOLINT
  {
#if defined(__CORTEX_M)
    __asm volatile("   movs   r0, #4           \n"
                   "   mov    r1, lr           \n"
                   "   tst    r0, r1           \n"
                   "   beq    .use_msp         \n"
                   "   mrs    r0, psp          \n"
                   "   b      .check_debug     \n"
                   ".use_msp:                  \n"
                   "   mrs    r0, msp          \n"
                   ".check_debug:              \n"
                   "   ldr    r1, =0xE000ED2C  \n"
                   "   ldr    r2, [r1]         \n"
                   "   lsls   r2, #1           \n"
                   ".hardfault_loop:           \n"
                   "   bcc    .hardfault_loop  \n"
                   "   ldr    r2, [r1]         \n"
                   "   str    r2, [r1]         \n"
                   "   ldr    r1, [r0, #24]    \n"
                   "   adds   r1, r1, #2       \n"
                   "   str    r1, [r0, #24]    \n"
                   "   movs   r1, #1           \n"
                   "   negs   r1, r1           \n"
                   "   str    r1, [r0, #0]     \n"
                   "   bx     lr               \n"
                   :
                   :
                   : "memory");
#endif
  }
}  // extern "C"
