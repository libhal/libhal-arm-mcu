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

// Needs linker argument: -Wl,--wrap=_ZN10__cxxabiv119__terminate_handlerE
// NOLINTNEXTLINE : GCC's symbol name which is within the cxxabiv1 namespace
std::terminate_handler __wrap__ZN10__cxxabiv119__terminate_handlerE =
  +[]() {  // NOLINT
    while (true) {
      continue;
    }
  };

// Needs linker argument: -Wl,--wrap=__cxa_terminate_handler
// NOLINTNEXTLINE : LLVM's symbol name which is just a global variable
std::terminate_handler __wrap___cxa_terminate_handler = []() {
  while (true) {
    continue;
  }
};

extern "C"
{
  void __attribute__((weak)) _exit(int)  // NOLINT
  {
    while (true) {
      continue;
    }
  }
}
