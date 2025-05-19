#pragma once

#include <libhal/initializers.hpp>
#include <libhal/units.hpp>

namespace hal::rp {

namespace internal {

#if defined(PICO_RP2040) || defined(PICO_RP2350A)
constexpr u8 pin_max = 30;
#else
constexpr u8 pin_max = 48;
#endif

enum struct processor_type : uint8_t
{
  rp2040,
  rp2350
};

#if defined(PICO_RP2040)
constexpr inline type t = type::rp2040;
#elif defined(PICO_RP2350A) || defined(PICO_RP2350B) || defined(PICO_RP2350)
constexpr inline processor_type type = processor_type::rp2350;
#else
#error "Unknown Pico model"
#endif

}  // namespace internal
template<typename T>
concept pin_param =
  std::is_same_v<pin_t<T::val>, T> && (T::val < internal::pin_max);
}  // namespace hal::rp
