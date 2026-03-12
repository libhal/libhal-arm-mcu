#pragma once

#include <exception>
#include <memory_resource>

namespace hal {
/**
 * @brief Set the global exception allocator function
 * @deprecated Do not use this. This is only here for backwards compatibility.
 *
 */
inline void set_exception_allocator(std::pmr::memory_resource&) noexcept
{
}

/**
 * @brief Get the global exception allocator function
 * @deprecated Do not use this. This is only here for backwards compatibility.
 *
 * @returns the global exception memory allocator implementation
 */
inline std::pmr::memory_resource& get_exception_allocator() noexcept
{
  return *std::pmr::new_delete_resource();
}

using std::get_terminate;
using std::set_terminate;
}  // namespace hal
