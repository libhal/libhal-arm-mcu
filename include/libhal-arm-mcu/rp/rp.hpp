#pragma once

#include <libhal/initializers.hpp>
#include <libhal/units.hpp>

namespace hal::rp {

namespace internal {

#if defined(LIBHAL_PLATFORM_RP2040) || defined(LIBHAL_VARIANT_RP2350A)
constexpr u8 pin_max = 30;
#elif defined(LIBHAL_VARIANT_RP2350B)
constexpr u8 pin_max = 48;
#else
#error "Unknown pico variant"
#endif

enum struct processor_type : uint8_t
{
  rp2040,
  rp2350
};

#if defined(LIBHAL_PLATFORM_RP2040)
constexpr inline processor_type type = type::rp2040;
#elif defined(LIBHAL_PLATFORM_RP2350_ARM_S)
constexpr inline processor_type type = processor_type::rp2350;
#else
#error "Unknown Pico model"
#endif

}  // namespace internal
template<typename T>
concept pin_param =
  std::is_same_v<pin_t<T::val>, T> && (T::val < internal::pin_max);
}  // namespace hal::rp
