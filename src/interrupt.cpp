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

#include <libhal-arm-mcu/interrupt.hpp>

#include <algorithm>
#include <cstdint>
#include <span>

#include <libhal-arm-mcu/system_control.hpp>
#include <libhal-util/enum.hpp>

#include "interrupt_reg.hpp"

namespace hal::cortex_m {
namespace {
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
}  // namespace

void default_interrupt_handler()
{
  while (true) {
    continue;
  }
}

void hard_fault_handler()
{
  while (true) {
    continue;
  }
}

void memory_management_fault_handler()
{
  while (true) {
    continue;
  }
}

void bus_fault_handler()
{
  while (true) {
    continue;
  }
}

void usage_fault_handler()
{
  while (true) {
    continue;
  }
}

void disable_all_interrupts()
{
#if defined(__arm__)
  asm volatile("cpsid i" : : : "memory");
#endif
}

void enable_all_interrupts()
{
#if defined(__arm__)
  asm volatile("cpsie i" : : : "memory");
#endif
}

bool interrupt_vector_table_initialized()
{
  auto* vector_base = reinterpret_cast<void*>(&vector_table[core_interrupts]);

  return get_interrupt_vector_table_address() == vector_base;
}

std::span<interrupt_pointer> const get_vector_table()
{
  return vector_table;
}

void enable_interrupt(irq_t p_irq, interrupt_pointer p_handler)
{
  if (not is_valid_irq_request(p_irq)) {
    return;
  }

  vector_table[p_irq] = p_handler;

  if (p_irq >= 0) {
    nvic_enable_irq(p_irq);
  }
}

void disable_interrupt(irq_t p_irq)
{
  if (!is_valid_irq_request(p_irq)) {
    return;
  }

  if (p_irq < 0) {
    return;
  }

  nvic_disable_irq(p_irq);
}

bool is_interrupt_enabled(irq_t p_irq)
{
  if (!is_valid_irq_request(p_irq)) {
    return false;
  }

  if (p_irq < 0) {
    return true;
  }

  uint32_t enable_register = nvic->iser[register_index(p_irq)];

  return (enable_register & (1 << p_irq)) != 0U;
}

bool verify_vector_enabled(irq_t p_irq, interrupt_pointer p_handler)
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

  uint32_t enable_register = nvic->iser[register_index(p_irq)];

  return (enable_register & (1 << p_irq)) != 0U;
}

void revert_interrupt_vector_table()
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

void initialize_interrupts(std::span<interrupt_pointer> p_vector_table)
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
}  // namespace hal::cortex_m

// Single version for ALL Cortex-M processors
extern "C"
{
  /**
   * @brief Hard fault handler that gracefully handles semihosting breakpoints
   *
   * This handler overrides picolibc's default hard fault handler to detect and
   * skip semihosting BKPT instructions when no debugger is attached. Without
   * this handler, applications linked with semihosting libraries will hang in
   * an infinite loop when executed standalone.
   *
   * The handler checks if a hard fault was caused by a debug event (BKPT
   * instruction). If so, it:
   * 1. Clears the hard fault status
   * 2. Advances the program counter past the BKPT instruction (2 bytes)
   * 3. Sets R0 to -1 to indicate the semihosting operation failed
   * 4. Returns to continue execution
   *
   * If the fault was caused by something other than a BKPT, the handler enters
   * an infinite loop to halt execution, indicating a real hard fault condition.
   *
   * This implementation is based on SEGGER's hard fault handler reference:
   * https://kb.segger.com/Arm_Cortex-M_interrupts
   *
   * Modified to use ARMv6-M (Cortex-M0) compatible instructions, making it
   * compatible with all Cortex-M variants (M0/M0+/M1/M3/M4/M7/M23/M33/M55/M85).
   *
   * @note This handler is wrapped via linker flag -Wl,--wrap=arm_hardfault_isr
   *       to override picolibc's default implementation.
   */
  __attribute__((naked)) void __wrap_arm_hardfault_isr(void)  // NOLINT
  {
#if defined(__CORTEX_M)
    __asm volatile("   movs   r0, #4                     \n"
                   "   mov    r1, lr                     \n"
                   "   tst    r0, r1                     \n"
                   "   beq    .use_msp                   \n"
                   "   mrs    r0, psp                    \n"
                   "   b      .check_debug               \n"
                   ".use_msp:                            \n"
                   "   mrs    r0, msp                    \n"
                   ".check_debug:                        \n"
                   "   ldr    r1, =0xE000ED2C            \n"
                   "   ldr    r2, [r1]                   \n"
                   "   lsls   r2, #1                     \n"
                   ".hardfault_loop:                     \n"
                   "   bcc    .hardfault_loop            \n"
                   "   ldr    r2, [r1]                   \n"
                   "   str    r2, [r1]                   \n"
                   "   ldr    r1, [r0, #24]              \n"
                   "   adds   r1, r1, #2                 \n"
                   "   str    r1, [r0, #24]              \n"
                   "   movs   r1, #1                     \n"
                   "   negs   r1, r1                     \n"
                   "   str    r1, [r0, #0]               \n"
                   "   bx     lr                         \n"
                   :
                   :
                   : "memory");
#endif
  }
}  // extern "C"
