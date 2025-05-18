#pragma once

#include <libhal/units.hpp>

namespace hal::rp::internal {

#if defined(PICO_RP2040) || defined(PICO_RP2350A)
constexpr u8 pin_max = 30;
#else
constexpr u8 pin_max = 48;
#endif

}  // namespace hal::rp
