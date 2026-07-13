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

#include <array>
#include <cstdint>

export module hal.arm_mcu.cortex_m:system_control;

import hal;

/**
 * @brief libhal drivers for the ARM Cortex-M series of processors
 *
 */
namespace hal::cortex_m {
/// Structure type to access the System Control Block (SCB).
struct scb_registers_t
{
  /// Offset: 0x000 (R/ )  CPUID Base Register
  std::uint32_t const volatile cpuid;
  /// Offset: 0x004 (R/W)  Interrupt Control and State Register
  std::uint32_t volatile icsr;
  /// Offset: 0x008 (R/W)  Vector Table Offset Register
  std::intptr_t volatile vtor;
  /// Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register
  std::uint32_t volatile aircr;
  /// Offset: 0x010 (R/W)  System Control Register
  std::uint32_t volatile scr;
  /// Offset: 0x014 (R/W)  Configuration Control Register
  std::uint32_t volatile ccr;
  /// Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 5)
  std::array<std::uint8_t volatile, 12U> shp;
  /// Offset: 0x024 (R/W)  System Handler Control and State Register
  std::uint32_t volatile shcsr;
  /// Offset: 0x028 (R/W)  Configurable Fault Status Register
  std::uint32_t volatile cfsr;
  /// Offset: 0x02C (R/W)  HardFault Status Register
  std::uint32_t volatile hfsr;
  /// Offset: 0x030 (R/W)  Debug Fault Status Register
  std::uint32_t volatile dfsr;
  /// Offset: 0x034 (R/W)  MemManage Fault Address Register
  std::uint32_t volatile mmfar;
  /// Offset: 0x038 (R/W)  BusFault Address Register
  std::uint32_t volatile bfar;
  /// Offset: 0x03C (R/W)  Auxiliary Fault Status Register
  std::uint32_t volatile afsr;
  /// Offset: 0x040 (R/ )  Processor Feature Register
  std::array<std::uint32_t volatile, 2U> const pfr;
  /// Offset: 0x048 (R/ )  Debug Feature Register
  std::uint32_t const volatile dfr;
  /// Offset: 0x04C (R/ )  Auxiliary Feature Register
  std::uint32_t const volatile adr;
  /// Offset: 0x050 (R/ )  Memory Model Feature Register
  std::array<std::uint32_t volatile, 4U> const mmfr;
  /// Offset: 0x060 (R/ )  Instruction Set Attributes Register
  std::array<std::uint32_t volatile, 5U> const isar;
  /// Reserved 0
  std::array<std::uint32_t, 5U> reserved0;
  /// Offset: 0x088 (R/W)  Coprocessor Access Control Register
  std::uint32_t volatile cpacr;
};

/// System control block address
constexpr auto scb_address = static_cast<uptr>(0xE000'ED00UL);

/// @return auto* - Address of the Cortex M system control block register
auto* scb =
  // NOLINTNEXTLINE(performance-no-int-to-ptr)
  reinterpret_cast<scb_registers_t*>(scb_address);

/**
 * @brief Enable the floating point unit coprocessor
 *
 * WARNING: If the coprocessor does not exist, as it is optional, a UsageFault
 * will occur. Floating point units are only found within Cortex M4 and
 * above processors.
 *
 */
export void initialize_floating_point_unit()
{
  scb->cpacr = scb->cpacr | ((0b11 << 10 * 2) | /* set CP10 Full Access */
                             (0b11 << 11 * 2)); /* set CP11 Full Access */
}

/**
 * @brief Set the address of the systems interrupt vector table
 *
 * The interrupt vector table (IVT) is held in ROM which means that, either
 * the interrupt service routines (ISR) had to be defined at compile time
 * making them immutable at runtime, or that each ISR calls a mutable function
 * pointer which can be changed at runtime.
 *
 * The problem with the first option is that it makes writing and using
 * libraries difficult. Usually requiring updates to the IVT manually by the
 * application designer based on what libraries and drivers the application is
 * using.
 *
 * The second solution has a problem where the additional another layer of
 * indirection increases interrupt latency. A more critical problem of this
 * approach is that many ISRs take advantage of the state of the system when
 * the ISR runs. For example, context switching in an RTOS needs to be able to
 * see the address of where code was when the interrupt occurred and having an
 * additional point of indirection (i.e. calling a function pointer) will
 * change that location from the task to the ISR that called the context
 * switch function. This will usually result in a fault of some sort.
 *
 * Creating an interrupt vector table in RAM and relocating the ISRs there
 * consumes RAM space, but gives great flexibility over the table at runtime.
 *
 * @param p_table_location - address of the interrupt vector table.
 */
export void set_interrupt_vector_table_address(void* p_table_location)
{
  // Relocate the interrupt vector table the vector buffer. By default this
  // will be set to the address of the start of flash memory for the MCU.
  scb->vtor = reinterpret_cast<std::intptr_t>(p_table_location);
}

/**
 * @brief Get the address of the systems interrupt vector table.
 *
 * On reset the VTOR register is set to 0x0000'0000 or nullptr.
 *
 * @return void* - address within VTOR the interrupt vector table relocation
 * register.
 */
export void* get_interrupt_vector_table_address()
{
  // Relocate the interrupt vector table the vector buffer. By default this
  // will be set to the address of the start of flash memory for the MCU.
  return reinterpret_cast<void*>(scb->vtor);  // NOLINT
}

/**
 * @brief Request reset from CPU
 *
 */
export [[noreturn]] void reset()
{
  // Value "0x5FA" must be written to the VECTKEY field [31:16] to confirm
  // that this action is valid, otherwise the processor ignores the write
  // command.
  // Bit 2 is the SYSRESETREQ bit.
  scb->aircr = (0x5FA << 16) | (1 << 2);
  // System reset is asynchronous, so the code needs to wait.
  hal::halt();
}

/**
 * @brief Executes WFI instruction
 *
 * The WFI instruction stops the CPU, reducing power, and wakes up on
 * interrupt.
 *
 */
export void wait_for_interrupt()
{
#if defined(__arm__)
  asm volatile("wfi");
#endif
}

/**
 * @brief Executes WFE instruction
 *
 * The WFE instruction stops the CPU, reducing power, and wakes up on event.
 *
 */
export void wait_for_event()
{
#if defined(__arm__)
  asm volatile("wfe");
#endif
}

/**
 * @brief Check if debugger is connected via CoreDebug->DHCSRd
 *
 * @return true - debugger connected
 * @return false - debugger is not connected
 */
export bool debugger_connected()
{
#if defined(__thumb2__)
  // CoreDebug->DHCSR register (Cortex-M3/M4/M7/etc.)
  std::uint32_t volatile* dhcsr =
    reinterpret_cast<std::uint32_t volatile*>(0xE000EDF0);
  // Bit 0 (C_DEBUGEN) indicates debugger is connected
  return (*dhcsr & 0x00000001) != 0;
#else
  return false;
#endif
}
}  // namespace hal::cortex_m

extern "C"
{
  // The implementation of LLVM calls a calls the breakpoint instruction
  // unconditionally, preventing the program from proceeding past this point
  // without a debugger connected with semihosting enabled. In order to get
  // around this, we replace sys_semihost and check if a debugger is
  // connected. If it is connected we call the breakpoint instruction with the
  // appropriate input value 0xAB. Otherwise, return an error code.
  int sys_semihost([[maybe_unused]] int p_reason, [[maybe_unused]] void* p_arg)
  {
    if (hal::cortex_m::debugger_connected()) {
#if defined(__thumb2__)
      // Let the real semihost call happen (BKPT will work)
      // Need to  the BKPT instruction here
      register int r0 asm("r0") = p_reason;
      register void* r1 asm("r1") = p_arg;
      asm volatile("bkpt 0xAB" : "=r"(r0) : "r"(r0), "r"(r1) : "memory");
      return r0;
#endif
    }
    return -1;  // No debugger, return error
  }

  char* sys_semihost_get_cmdline()
  {
    if (hal::cortex_m::debugger_connected()) {
      // SYS_GET_CMDLINE is semihost operation 0x15
      static char cmdline[256];
      int result = sys_semihost(0x15, cmdline);
      if (result == 0) {
        return cmdline;
      }
    }
    static char empty[] = "";
    return empty;
  }
}
