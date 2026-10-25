// Copyright 2024 - 2025 Khalil Estell and the libhal contributors
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

module;

#include <cstddef>
#include <span>

export module hal.arm_mcu.stm32_generic:scatter_cursor;

import hal;

namespace hal::stm32_generic {
/**
 * @brief Sequential, one-element-at-a-time cursor over a `scatter_span`
 *
 * Walks the constituent chunks of a `scatter_span<T>` as if it were one
 * contiguous range, skipping over empty chunks. Used to drive a spi or i2c
 * transfer across a caller-supplied set of discontiguous buffers, either one
 * byte at a time or one contiguous run at a time for DMA, without requiring
 * random access into the `scatter_span` (which it does not support).
 *
 * @tparam T - element type of the underlying scatter_span. Use a `const`
 * qualified type for a read-only cursor and an unqualified type for a
 * cursor whose `current()` element can be written to.
 */
template<typename T>
class scatter_cursor
{
public:
  explicit scatter_cursor(mem::scatter_span<T> p_span)
    : m_it(p_span.begin())
    , m_end(p_span.end())
  {
    skip_empty_chunks();
  }

  /// @return true if every element of the scatter_span has been visited
  [[nodiscard]] bool done() const
  {
    return m_it == m_end;
  }

  /// @return reference to the element at the cursor's current position.
  /// Calling this when `done()` is true is undefined behavior.
  [[nodiscard]] T& current()
  {
    return (*m_it)[m_offset];
  }

  /// @return the elements from the cursor's position to the end of its
  /// chunk. Never empty unless `done()` is true.
  [[nodiscard]] std::span<T> contiguous()
  {
    if (done()) {
      return {};
    }
    return (*m_it).subspan(m_offset);
  }

  /// Move the cursor to the next element, advancing past chunk boundaries
  /// (including empty chunks) as needed.
  void advance()
  {
    advance(1);
  }

  /// Move the cursor forward by `p_count` elements, which must not be more
  /// than `contiguous().size()`.
  void advance(std::size_t p_count)
  {
    m_offset += p_count;
    if (m_offset >= (*m_it).size()) {
      ++m_it;
      m_offset = 0;
      skip_empty_chunks();
    }
  }

private:
  void skip_empty_chunks()
  {
    while (m_it != m_end and (*m_it).empty()) {
      ++m_it;
    }
  }

  mem::scatter_span_iterator<T> m_it;
  mem::scatter_span_iterator<T> m_end;
  std::size_t m_offset = 0;
};
}  // namespace hal::stm32_generic
