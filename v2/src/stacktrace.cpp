// Copyright 2026 Khalil Estell and the libhal contributors
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

// Unwinds the stack with the Arm exception tables, as described in "Exception
// Handling ABI for the Arm Architecture" (EHABI, IHI 0038), section 9 and 10.
// Only the unwind instructions are followed. Personality routines and their
// language-specific data are skipped, since nothing is caught or cleaned up.

module;

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

module hal.arm_mcu.cortex_m;

import hal;
import hal.util;

#if defined(__arm__)
namespace hal::cortex_m {
/// One entry of the exception index table (`.ARM.exidx`)
struct exception_index_entry
{
  /// prel31 offset to the start of the function
  hal::u32 function;
  /// 1 if the function can't be unwound, its unwind instructions if bit 31
  /// is set, else a prel31 offset to them in the exception table
  hal::u32 content;
};
}  // namespace hal::cortex_m

// Provided by picolibc's linker script
extern "C"
{
  // NOLINTBEGIN(readability-identifier-naming,bugprone-reserved-identifier)
  extern hal::cortex_m::exception_index_entry const __exidx_start[];
  extern hal::cortex_m::exception_index_entry const __exidx_end[];
  extern hal::byte const __ram;
  /// An absolute symbol: its address is the size of RAM
  extern hal::byte const __ram_size;
  // NOLINTEND(readability-identifier-naming,bugprone-reserved-identifier)
}
#endif

namespace hal::cortex_m {
#if defined(__arm__)
namespace {
constexpr std::size_t stack_pointer = 13;
constexpr std::size_t link_register = 14;
constexpr std::size_t program_counter = 15;

/// r0 to r15 of one frame while the stack is unwound
using register_file = std::array<hal::uptr, 16>;

/**
 * Store r4 to r11, the stack pointer and the link register. The program
 * counter is set to the link register, the return address into the caller,
 * where the caller's unwind instructions apply.
 *
 * Written with ARMv6-M instructions, so it runs on every Cortex-M.
 */
[[gnu::naked, gnu::noinline]] void capture_registers(register_file&) noexcept
{
  asm volatile("str r4, [r0, #16]\n"
               "str r5, [r0, #20]\n"
               "str r6, [r0, #24]\n"
               "str r7, [r0, #28]\n"
               "mov r1, r8\n"
               "str r1, [r0, #32]\n"
               "mov r1, r9\n"
               "str r1, [r0, #36]\n"
               "mov r1, r10\n"
               "str r1, [r0, #40]\n"
               "mov r1, r11\n"
               "str r1, [r0, #44]\n"
               "mov r1, sp\n"
               "str r1, [r0, #52]\n"
               "mov r1, lr\n"
               "str r1, [r0, #56]\n"
               "str r1, [r0, #60]\n"
               "bx lr\n"
               :
               :
               : "r1", "memory");
}

/// The address a prel31 offset points to, relative to the offset itself
hal::uptr prel31_target(hal::u32 const& p_offset)
{
  // Sign extend the 31 bit offset
  auto const offset = static_cast<std::int32_t>(p_offset << 1U) >> 1;
  return std::bit_cast<hal::uptr>(&p_offset) + static_cast<hal::uptr>(offset);
}

/// A word of RAM at an address taken from the stack
hal::uptr read_word(hal::uptr p_address)
{
  // NOLINTNEXTLINE(performance-no-int-to-ptr): saved registers on the stack
  return *reinterpret_cast<hal::uptr const*>(p_address);
}

/// Where words may be read: inside RAM and word aligned
struct readable_memory
{
  hal::uptr begin;
  hal::uptr end;

  [[nodiscard]] bool holds(hal::uptr p_address, hal::uptr p_words = 1) const
  {
    return ((p_address % 4U) == 0) and (begin <= p_address) and
           (p_address <= end) and ((p_words * 4U) <= (end - p_address));
  }
};

/// The unwind instructions of one function: bytes packed into words, most
/// significant byte first
class unwind_instructions
{
public:
  unwind_instructions(hal::u32 const* p_words, hal::u32 p_first, hal::u32 p_end)
    : m_words(p_words)
    , m_next(p_first)
    , m_end(p_end)
  {
  }

  /// The next instruction byte, or "finish" past the last one. Kept out of
  /// line: it's read in several places, and inlining it there costs ROM.
  [[gnu::noinline]] hal::u8 next()
  {
    constexpr hal::u8 finish = 0b1011'0000;
    if (m_next >= m_end) {
      return finish;
    }
    auto const word = m_words[m_next / 4U];
    auto const shift = 24U - ((m_next % 4U) * 8U);
    m_next++;
    return static_cast<hal::u8>(word >> shift);
  }

private:
  hal::u32 const* m_words;
  hal::u32 m_next;
  hal::u32 m_end;
};

/// The unwind instructions of the function containing p_address, if it has
/// any
std::optional<unwind_instructions> instructions_at(hal::uptr p_address)
{
  constexpr hal::u32 cant_unwind = 0b1;
  constexpr auto compact = hal::bit_mask::from<31>();
  constexpr auto personality_index = hal::bit_mask::from<24, 27>();
  constexpr auto compact_extra_words = hal::bit_mask::from<16, 23>();
  constexpr auto generic_extra_words = hal::bit_mask::from<24, 31>();

  auto const table = std::span(__exidx_start, __exidx_end);
  // The entry after the last function starting at or before p_address
  auto const after = std::ranges::upper_bound(
    table, p_address, {}, [](exception_index_entry const& p_entry) {
      return prel31_target(p_entry.function);
    });
  if (after == table.begin()) {
    return std::nullopt;
  }
  auto const& entry = *(after - 1);

  if (entry.content == cant_unwind) {
    return std::nullopt;
  }
  if (hal::bit_is_set<compact>(entry.content)) {
    // Three instructions inside the entry itself, for personality routine 0
    if (hal::bit_extract<personality_index>(entry.content) != 0) {
      return std::nullopt;
    }
    return unwind_instructions(&entry.content, 1, 4);
  }

  // NOLINTNEXTLINE(performance-no-int-to-ptr): into the exception table
  auto const* words =
    reinterpret_cast<hal::u32 const*>(prel31_target(entry.content));
  if (not hal::bit_is_set<compact>(words[0])) {
    // A generic personality routine, such as __gxx_personality_v0. Its
    // address comes first, then a count of extra words and the instructions.
    auto const extra_words = hal::bit_extract<generic_extra_words>(words[1]);
    return unwind_instructions(&words[1], 1, 4U + (extra_words * 4U));
  }
  switch (hal::bit_extract<personality_index>(words[0])) {
    case 0:
      return unwind_instructions(words, 1, 4);
    case 1:
    case 2: {
      auto const extra_words = hal::bit_extract<compact_extra_words>(words[0]);
      return unwind_instructions(words, 2, 4U + (extra_words * 4U));
    }
    default:
      return std::nullopt;
  }
}

/**
 * Undo one function's prologue: restore the registers it saved and the
 * stack pointer of its caller. The program counter becomes the return
 * address.
 *
 * Every pop instruction becomes a mask of the registers it pops, and every
 * floating point pop a number of bytes to skip, so one loop does the work.
 *
 * Returns false if the instructions refuse to unwind, use one that Cortex-M
 * can't have, or read outside p_memory.
 */
bool unwind_frame(unwind_instructions p_instructions,
                  register_file& p_registers,
                  readable_memory p_memory)
{
  constexpr auto low_six = hal::bit_mask::from<0, 5>();
  constexpr auto low_four = hal::bit_mask::from<0, 3>();
  constexpr auto low_three = hal::bit_mask::from<0, 2>();
  constexpr auto top_four = hal::bit_mask::from<4, 7>();
  constexpr auto top_five = hal::bit_mask::from<3, 7>();
  constexpr auto with_link_register = hal::bit_mask::from<3>();
  constexpr hal::u8 finish = 0b1011'0000;
  constexpr hal::u8 pop_low_registers = 0b1011'0001;
  constexpr hal::u8 add_uleb128 = 0b1011'0010;
  constexpr hal::u8 pop_fstmfdx = 0b1011'0011;
  constexpr hal::u8 pop_vpush_high = 0b1100'1000;
  constexpr hal::u8 pop_vpush = 0b1100'1001;

  auto vsp = p_registers[stack_pointer];
  bool pc_restored = false;

  while (true) {
    auto const instruction = p_instructions.next();
    auto const top = hal::bit_extract<top_four>(instruction);
    // Registers to pop, bit n for r[n], lowest first
    hal::u32 pop_mask = 0;
    // Bytes of floating point registers to skip: nothing reads them back
    hal::u32 skip = 0;

    if (instruction < 0b1000'0000) {
      // 00xxxxxx: vsp += (xxxxxx << 2) + 4, and 01xxxxxx: vsp -= the same
      auto const offset = (hal::bit_extract<low_six>(instruction) << 2U) + 4U;
      vsp = (instruction < 0b0100'0000) ? (vsp + offset) : (vsp - offset);
    } else if (top == 0b1000) {
      // 1000iiii iiiiiiii: pop r4 to r15 under a mask. All zero refuses.
      pop_mask =
        (hal::bit_extract<low_four>(instruction) << 8U) | p_instructions.next();
      if (pop_mask == 0) {
        return false;
      }
      pop_mask <<= 4U;
    } else if (top == 0b1001) {
      // 1001nnnn: vsp = r[nnnn], except r13 and r15, which are reserved
      auto const source = hal::bit_extract<low_four>(instruction);
      if ((source == stack_pointer) or (source == program_counter)) {
        return false;
      }
      vsp = p_registers[source];
    } else if (top == 0b1010) {
      // 1010Lnnn: pop r4 to r[4 + nnn], then r14 if L is set
      auto const count = hal::bit_extract<low_three>(instruction) + 1U;
      pop_mask = ((1U << count) - 1U) << 4U;
      if (hal::bit_is_set<with_link_register>(instruction)) {
        pop_mask |= 1U << link_register;
      }
    } else if (instruction == finish) {
      break;
    } else if (instruction == pop_low_registers) {
      // 10110001 0000iiii: pop r0 to r3 under a mask
      pop_mask = p_instructions.next();
      if ((pop_mask == 0) or (pop_mask > 0b1111)) {
        return false;
      }
    } else if (instruction == add_uleb128) {
      // 10110010 uleb128: vsp += 0x204 + (uleb128 << 2)
      hal::uptr value = 0;
      hal::u32 shift = 0;
      hal::u8 byte = 0;
      do {
        byte = p_instructions.next();
        value |= static_cast<hal::uptr>(byte & 0b0111'1111U) << shift;
        shift += 7U;
      } while (((byte & 0b1000'0000U) != 0) and (shift < 32U));
      vsp += 0x204U + (value << 2U);
    } else if ((instruction == pop_fstmfdx) or
               (instruction == pop_vpush_high) or (instruction == pop_vpush)) {
      // sssscccc follows: pop cccc + 1 double registers. The FSTMFDX format
      // stores one more word.
      auto const count = hal::bit_extract<low_four>(p_instructions.next()) + 1U;
      skip = (count * 8U) + ((instruction == pop_fstmfdx) ? 4U : 0U);
    } else if ((hal::bit_extract<top_five>(instruction) == 0b10111) or
               (hal::bit_extract<top_five>(instruction) == 0b11010)) {
      // 10111nnn (FSTMFDX) and 11010nnn (VPUSH): pop d8 to d[8 + nnn]
      auto const count = hal::bit_extract<low_three>(instruction) + 1U;
      skip = (count * 8U) +
             ((hal::bit_extract<top_five>(instruction) == 0b10111) ? 4U : 0U);
    } else {
      // Spare, or Intel Wireless MMX, which no Cortex-M has
      return false;
    }

    vsp += skip;
    for (std::size_t target = 0; target < p_registers.size(); target++) {
      if ((pop_mask & (1U << target)) == 0) {
        continue;
      }
      if (not p_memory.holds(vsp)) {
        return false;
      }
      p_registers[target] = read_word(vsp);
      vsp += 4U;
    }
    // A popped stack pointer takes effect after the whole pop
    if ((pop_mask & (1U << stack_pointer)) != 0) {
      vsp = p_registers[stack_pointer];
    }
    pc_restored = pc_restored or ((pop_mask & (1U << program_counter)) != 0);
  }

  if (not pc_restored) {
    p_registers[program_counter] = p_registers[link_register];
  }
  p_registers[stack_pointer] = vsp;
  return true;
}

/// Whether p_value is an EXC_RETURN value: the link register of an
/// interrupt handler, which returns from the interrupt
constexpr bool is_interrupt_return(hal::uptr p_value)
{
  constexpr auto prefix = hal::bit_mask::from<28, 31>();
  return hal::bit_extract<prefix>(p_value) == 0xF;
}

/// Whether p_value is a return address into Thumb code
constexpr bool is_return_address(hal::uptr p_value)
{
  constexpr auto thumb = hal::bit_mask::from<0>();
  return hal::bit_is_set<thumb>(p_value) and (p_value > 0b1U) and
         (not is_interrupt_return(p_value));
}

/// An address inside the call instruction that returns to p_return_address.
/// Both BL and BLX are at least 2 bytes long.
constexpr hal::uptr call_site(hal::uptr p_return_address)
{
  return (p_return_address & ~hal::uptr{ 0b1 }) - 2U;
}

hal::uptr process_stack_pointer()
{
  hal::uptr value = 0;
  asm volatile("mrs %0, psp" : "=r"(value));
  return value;
}

/**
 * Step from an interrupt handler into the code it interrupted. The handler
 * has been unwound, so the program counter holds EXC_RETURN and the stack
 * pointer is where the hardware saved the interrupted registers, unless they
 * are on the process stack.
 */
bool cross_interrupt_frame(register_file& p_registers, readable_memory p_memory)
{
  constexpr auto on_process_stack = hal::bit_mask::from<2>();
  constexpr auto basic_frame_only = hal::bit_mask::from<4>();
  constexpr auto realigned = hal::bit_mask::from<9>();
  constexpr hal::uptr basic_frame_words = 8;
  // s0 to s15, FPSCR and a reserved word
  constexpr hal::uptr floating_point_words = 18;

  auto const interrupt_return = p_registers[program_counter];
  auto frame = p_registers[stack_pointer];
  if (hal::bit_is_set<on_process_stack>(interrupt_return)) {
    frame = process_stack_pointer();
  }
  if (not p_memory.holds(frame, basic_frame_words)) {
    return false;
  }

  // The frame holds r0, r1, r2, r3, r12, lr, pc and xPSR
  for (std::size_t index = 0; index < 4; index++) {
    p_registers[index] = read_word(frame + (index * 4U));
  }
  p_registers[12] = read_word(frame + 16U);
  p_registers[link_register] = read_word(frame + 20U);
  p_registers[program_counter] = read_word(frame + 24U);
  auto const status = read_word(frame + 28U);

  auto words = basic_frame_words;
  if (not hal::bit_is_set<basic_frame_only>(interrupt_return)) {
    words += floating_point_words;
  }
  auto caller_sp = frame + (words * 4U);
  if (hal::bit_is_set<realigned>(status)) {
    caller_sp += 4U;
  }
  p_registers[stack_pointer] = caller_sp;
  return true;
}
}  // namespace
#endif

[[gnu::noinline]] std::span<hal::uptr> stacktrace(
  std::span<hal::uptr> p_addresses) noexcept
{
#if defined(__arm__)
  register_file registers{};
  capture_registers(registers);

  auto const ram_begin = std::bit_cast<hal::uptr>(&__ram);
  readable_memory const memory{
    .begin = ram_begin,
    .end = ram_begin + std::bit_cast<hal::uptr>(&__ram_size),
  };

  // This function's own frame is unwound first and not recorded
  auto location = call_site(registers[program_counter]);
  // Whether an interrupt stopped the current frame, so its program
  // counter is the interrupted instruction, not a return address
  bool interrupted = false;
  std::size_t count = 0;

  while (count < p_addresses.size()) {
    auto const instructions = instructions_at(location);
    if (not instructions) {
      if (interrupted and is_return_address(registers[link_register])) {
        p_addresses[count++] = call_site(registers[link_register]);
      }
      break;
    }

    auto const previous_sp = registers[stack_pointer];
    if (not interrupted) {
      // The link register is only live in an interrupted function. Any other
      // function made a call, which overwrote it, so its return address is
      // only known if its instructions restore it. A function that never
      // returns and can't throw may not have saved it at all.
      registers[link_register] = 0;
    }
    if (not unwind_frame(*instructions, registers, memory)) {
      break;
    }
    // Every caller's frame is above its callee's, except that an
    // interrupted function may not have pushed anything yet
    if ((not interrupted) and (registers[stack_pointer] <= previous_sp)) {
      break;
    }

    auto const pc = registers[program_counter];
    if (is_interrupt_return(pc)) {
      if (not cross_interrupt_frame(registers, memory)) {
        break;
      }
      interrupted = true;
      location = registers[program_counter] & ~hal::uptr{ 0b1 };
    } else if (is_return_address(pc)) {
      interrupted = false;
      location = call_site(pc);
    } else {
      break;
    }
    p_addresses[count++] = location;
  }

  return p_addresses.first(count);
#else
  return p_addresses.first(0);
#endif
}
}  // namespace hal::cortex_m
