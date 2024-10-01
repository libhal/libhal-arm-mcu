#pragma once

#include <array>

namespace hal::lpc40 {
struct dac_registers
{
  union
  {
    std::uint32_t volatile whole;
    std::array<std::uint8_t volatile, 4> parts;
  } conversion_register;
  std::uint32_t volatile control;
  std::uint32_t volatile count_value;
};
namespace dac_converter_register {
// Holds the value that we want in the register
static constexpr auto value = hal::bit_mask::from<6, 15>();

static constexpr auto bias = hal::bit_mask::from<16>();
}  // namespace dac_converter_register
constexpr std::uintptr_t dac_address = 0x4008'C000;
inline auto* dac_reg = reinterpret_cast<dac_registers*>(dac_address);

}  // namespace hal::lpc40
