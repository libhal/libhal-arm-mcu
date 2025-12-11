/**
 * @file terminate_handler.cpp
 * @brief Implementations of the terminate handler that simply halts
 *
 * This provides replacement symbols for the default terminate handler used in
 * both GCC and LLVM. The default implementation prints a message to stdout for
 * the user. The code that performs the message rendering can result in 40kb to
 * 80kB added to ROM. By replacing that symbol we can eliminate all of the text
 * rendering code.
 *
 */

#include <exception>

namespace hal::cortex_m {
std::terminate_handler default_handler = +[]() {  // NOLINT
  while (true) {
    continue;
  }
};

}  // namespace hal::cortex_m

#if defined(__clang__)
// Needs linker argument: -Wl,--wrap=__cxa_terminate_handler
// LLVM's symbol name which is just a global variable
// NOLINTNEXTLINE(readability-identifier-naming,bugprone-reserved-identifier)
std::terminate_handler __wrap___cxa_terminate_handler =
  hal::cortex_m::default_handler;
#elif defined(__GNUC__)
// Needs linker argument: -Wl,--wrap=_ZN10__cxxabiv119__terminate_handlerE
// GCC's symbol name which is within the cxxabiv1 namespace
// NOLINTNEXTLINE(readability-identifier-naming,bugprone-reserved-identifier)
std::terminate_handler __wrap__ZN10__cxxabiv119__terminate_handlerE =
  hal::cortex_m::default_handler;
#endif

extern "C"
{
  // NOLINTNEXTLINE(readability-identifier-naming,bugprone-reserved-identifier)
  void __attribute__((weak)) _exit(int)
  {
    while (true) {
      continue;
    }
  }

  // NOLINTNEXTLINE(readability-identifier-naming,bugprone-reserved-identifier)
  std::terminate_handler __wrap__ZSt13set_terminatePFvvE(
    std::terminate_handler p_handler) noexcept
  {
#if defined(__clang__)
    std::terminate_handler previous = __wrap___cxa_terminate_handler;

    if (p_handler == nullptr) {
      __wrap___cxa_terminate_handler = hal::cortex_m::default_handler;
    } else {
      __wrap___cxa_terminate_handler = p_handler;
    }

    return previous;
#elif defined(__GNUC__)
    std::terminate_handler previous =
      __wrap__ZN10__cxxabiv119__terminate_handlerE;

    if (p_handler == nullptr) {
      __wrap__ZN10__cxxabiv119__terminate_handlerE =
        hal::cortex_m::default_handler;
    }

    return previous;
#endif
  }

  // NOLINTNEXTLINE(readability-identifier-naming,bugprone-reserved-identifier)
  std::terminate_handler __wrap__ZSt13get_terminatev() noexcept
  {
#if defined(__clang__)
    return __wrap___cxa_terminate_handler;
#elif defined(__GNUC__)
    return __wrap__ZN10__cxxabiv119__terminate_handlerE;
#endif
  }
}
