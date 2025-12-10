#include <exception>

/**
 * @brief Implementation of the terminate handler that simply halts
 *
 * This will override the weak symbol used in GCC's exception runtime. The
 * original implementation prints a message to stdout for the user. The code
 * that performs the message rendering can result in 40kb to 80kB added to ROM.
 * By replacing that symbol with one that does nothing besides halt, all of
 * those dependencies on those functions are unnecessary and can be garbage
 * collected.
 *
 * NOTE: If for some reason in the future, this stops working or the linker
 * complains of duplicate symbols, then we should wrap the symbol using.
 * `-Wl,--wrap=__terminate_handler`
 *
 */
// NOLINTNEXTLINE : GCC
std::terminate_handler __wrap__ZN10__cxxabiv119__terminate_handlerE =
  +[]() {  // NOLINT
    while (true) {
      continue;
    }
  };
// NOLINTNEXTLINE : LLVM
std::terminate_handler __wrap___cxa_terminate_handler = []() {
  while (true) {
    continue;
  }
};
