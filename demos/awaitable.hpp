#pragma once

#include <coroutine>
#include <exception>
#include <stdexcept>
#include <utility>

namespace hal {

/**
 * @brief Zero-allocation awaitable for interrupt-driven operations
 *
 * This class provides an awaitable interface without requiring dynamic
 * memory allocation, making it suitable for interrupt handlers and other
 * performance-critical code.
 */
template<typename T = void>
class interrupt_awaiter
{
private:
  std::coroutine_handle<> m_handle = nullptr;
  std::exception_ptr m_exception = nullptr;
  bool m_completed = false;

  // Storage for result value (not used for void specialization)
  union
  {
    T m_result;
    char m_dummy;  // For empty state
  };
  bool m_has_result = false;

public:
  // Default constructor initializes the dummy value
  interrupt_awaiter() noexcept
    : m_dummy{}
  {
  }

  // Destructor properly cleans up the result if present
  ~interrupt_awaiter()
  {
    if constexpr (!std::is_void_v<T>) {
      if (m_has_result) {
        m_result.~T();
        m_has_result = false;
      }
    }
  }

  // Move constructor
  interrupt_awaiter(interrupt_awaiter&& other) noexcept
    : m_handle(std::exchange(other.m_handle, nullptr))
    , m_exception(std::exchange(other.m_exception, nullptr))
    , m_completed(std::exchange(other.m_completed, false))
  {
    if constexpr (!std::is_void_v<T>) {
      if (other.m_has_result) {
        new (&m_result) T(std::move(other.m_result));
        m_has_result = true;
        other.m_result.~T();
        other.m_has_result = false;
      }
    }
  }

  // Move assignment
  interrupt_awaiter& operator=(interrupt_awaiter&& other) noexcept
  {
    if (this != &other) {
      // Clean up existing result if any
      if constexpr (!std::is_void_v<T>) {
        if (m_has_result) {
          m_result.~T();
          m_has_result = false;
        }
      }

      // Move from other
      m_handle = std::exchange(other.m_handle, nullptr);
      m_exception = std::exchange(other.m_exception, nullptr);
      m_completed = std::exchange(other.m_completed, false);

      if constexpr (!std::is_void_v<T>) {
        if (other.m_has_result) {
          new (&m_result) T(std::move(other.m_result));
          m_has_result = true;
          other.m_result.~T();
          other.m_has_result = false;
        }
      }
    }
    return *this;
  }

  // Delete copy operations
  interrupt_awaiter(interrupt_awaiter const&) = delete;
  interrupt_awaiter& operator=(interrupt_awaiter const&) = delete;

  // Awaiter interface
  [[nodiscard]] bool await_ready() const noexcept
  {
    return m_completed;
  }

  void await_suspend(std::coroutine_handle<> handle) noexcept
  {
    m_handle = handle;
  }

  // Result retrieval, specialized for non-void types
  template<typename U = T>
  U await_resume()
    requires(!std::is_void_v<U>)
  {
    if (m_exception) {
      std::rethrow_exception(m_exception);
    }
    if (m_has_result) {
      return std::move(m_result);
    }
    throw std::runtime_error("No result available");
  }

  // Void specialization
  template<typename U = T>
  void await_resume()
    requires(std::is_void_v<U>)
  {
    if (m_exception) {
      std::rethrow_exception(m_exception);
    }
  }

  // Complete the operation with a result
  template<typename U = T>
  void complete(U&& result) noexcept
    requires(!std::is_void_v<U>)
  {
    // Clean up existing result if any
    if (m_has_result) {
      m_result.~T();
      m_has_result = false;
    }

    // Store new result
    new (&m_result) T(std::forward<U>(result));
    m_has_result = true;
    m_completed = true;

    // Resume the awaiting coroutine if any
    if (m_handle) {
      auto handle = m_handle;
      m_handle = nullptr;
      handle.resume();
    }
  }

  // Void specialization of complete
  template<typename U = T>
  void complete() noexcept
    requires(std::is_void_v<U>)
  {
    m_completed = true;
    if (m_handle) {
      auto handle = m_handle;
      m_handle = nullptr;
      handle.resume();
    }
  }

  // Complete with exception
  void complete_with_exception(std::exception_ptr p_exception) noexcept
  {
    // NOLINTNEXTLINE(performance-unnecessary-value-param)
    m_exception = p_exception;
    m_completed = true;
    if (m_handle) {
      auto handle = m_handle;
      m_handle = nullptr;
      handle.resume();
    }
  }

  // Reset the awaiter for reuse
  void reset() noexcept
  {
    if constexpr (!std::is_void_v<T>) {
      if (m_has_result) {
        m_result.~T();
        m_has_result = false;
      }
    }

    m_completed = false;
    m_exception = nullptr;
    m_handle = nullptr;
  }

  // Check if the operation is complete
  [[nodiscard]] bool is_complete() const noexcept
  {
    return m_completed;
  }
};

// Specialization for void to avoid union complexity
template<>
class interrupt_awaiter<void>
{
private:
  std::coroutine_handle<> m_handle = nullptr;
  std::exception_ptr m_exception = nullptr;
  bool m_completed = false;

public:
  interrupt_awaiter() noexcept = default;

  // Awaiter interface
  [[nodiscard]] bool await_ready() const noexcept
  {
    return m_completed;
  }

  void await_suspend(std::coroutine_handle<> handle) noexcept
  {
    m_handle = handle;
  }

  void await_resume()
  {
    if (m_exception) {
      std::rethrow_exception(m_exception);
    }
  }

  // Complete the operation
  void complete() noexcept
  {
    m_completed = true;
    if (m_handle) {
      auto handle = m_handle;
      m_handle = nullptr;
      handle.resume();
    }
  }

  // Complete with exception
  void complete_with_exception(std::exception_ptr exception) noexcept
  {
    m_exception = exception;  // NOLINT(performance-unnecessary-value-param)
    m_completed = true;
    if (m_handle) {
      auto handle = m_handle;
      m_handle = nullptr;
      handle.resume();
    }
  }

  // Reset the awaiter for reuse
  void reset() noexcept
  {
    m_completed = false;
    m_exception = nullptr;
    m_handle = nullptr;
  }

  // Check if the operation is complete
  [[nodiscard]] bool is_complete() const noexcept
  {
    return m_completed;
  }
};

}  // namespace hal
