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

#include <libhal-arm-mcu/system_control.hpp>

#include "system_controller_reg.hpp"

#include <libhal/error.hpp>

namespace hal::cortex_m {
void initialize_floating_point_unit()
{
  scb->cpacr = scb->cpacr | ((0b11 << 10 * 2) | /* set CP10 Full Access */
                             (0b11 << 11 * 2)); /* set CP11 Full Access */
}

void set_interrupt_vector_table_address(void* p_table_location)
{
  // Relocate the interrupt vector table the vector buffer. By default this
  // will be set to the address of the start of flash memory for the MCU.
  scb->vtor = reinterpret_cast<intptr_t>(p_table_location);
}

void* get_interrupt_vector_table_address()
{
  // Relocate the interrupt vector table the vector buffer. By default this
  // will be set to the address of the start of flash memory for the MCU.
  return reinterpret_cast<void*>(scb->vtor);  // NOLINT
}

void reset()
{
  // Value "0x5FA" must be written to the VECTKEY field [31:16] to confirm
  // that this action is valid, otherwise the processor ignores the write
  // command.
  // Bit 2 is the SYSRESETREQ bit.
  scb->aircr = (0x5FA << 16) | (1 << 2);
  // System reset is asynchronous, so the code needs to wait.
  hal::halt();
}

void wait_for_interrupt()
{
#if defined(__arm__)
  asm volatile("wfi");
#endif
}

void wait_for_event()
{
#if defined(__arm__)
  asm volatile("wfe");
#endif
}

bool debugger_connected()
{
#if defined(__thumb2__)
  // CoreDebug->DHCSR register (Cortex-M3/M4/M7/etc.)
  uint32_t volatile* dhcsr = reinterpret_cast<uint32_t volatile*>(0xE000EDF0);
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
  // around this, we replace sys_semihost and check if a debugger is connected.
  // If it is connected we call the breakpoint instruction with the appropriate
  // input value 0xAB. Otherwise, return an error code.
  int sys_semihost([[maybe_unused]] int p_reason, [[maybe_unused]] void* p_arg)
  {
    if (hal::cortex_m::debugger_connected()) {
#if defined(__thumb2__)
      // Let the real semihost call happen (BKPT will work)
      // Need to inline the BKPT instruction here
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
