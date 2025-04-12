#pragma once

#include <array>

#include <libhal-util/bit.hpp>
#include <libhal-util/units.hpp>

namespace hal::lpc40 {
struct dac_registers
{
  union
  {
    u32 volatile whole;
    std::array<u8 volatile, 4> parts;
  } conversion_register;
  u32 volatile control;
  u32 volatile count_value;
};
namespace dac_converter_register {
// Holds the value that we want in the register
static constexpr auto value = hal::bit_mask::from<6, 15>();

static constexpr auto bias = hal::bit_mask::from<16>();
}  // namespace dac_converter_register
constexpr std::uintptr_t dac_address = 0x4008'C000;
// NOLINTNEXTLINE(performance-no-int-to-ptr)
inline auto* dac_reg = reinterpret_cast<dac_registers*>(dac_address);

}  // namespace hal::lpc40
