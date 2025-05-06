#pragma once

#include <coroutine>
#include <exception>
#include <memory_resource>
#include <type_traits>
#include <utility>

namespace hal::v5 {
namespace detail {
// Helper type for void
struct void_placeholder
{};

// Type selection for void vs non-void
template<typename T>
using void_to_placeholder_t =
  std::conditional_t<std::is_void_v<T>, void_placeholder, T>;
}  // namespace detail

// Forward declaration
template<typename T>
class task;

// Primary promise type
template<typename T>
class hal_promise_type
{
private:
  using value_type = detail::void_to_placeholder_t<T>;

  std::pmr::memory_resource* m_allocator;
  std::exception_ptr m_exception = nullptr;
  std::coroutine_handle<> m_continuation = nullptr;
  value_type m_result{};

public:
  // Constructor with explicit allocator
  explicit hal_promise_type(std::pmr::memory_resource* p_allocator)
    : m_allocator(p_allocator)
  {
  }

  // Create the task object
  task<T> get_return_object() noexcept;

  // Start executing immediately
  std::suspend_never initial_suspend() noexcept
  {
    return {};
  }

  // Suspend at end
  std::suspend_always final_suspend() noexcept
  {
    return {};
  }

  // Store non-void return value
  template<typename U>
  void return_value(U&& value)
    requires(!std::is_void_v<T>)
  {
    m_result = std::forward<U>(value);
  }

  // Handle exceptions
  void unhandled_exception() noexcept
  {
    m_exception = std::current_exception();
  }

  // Get result value or propagate exception
  T& result()
    requires(!std::is_void_v<T>)
  {
    if (m_exception) {
      std::rethrow_exception(m_exception);
    }
    return m_result;
  }

  // Void specialization
  void result()
    requires(std::is_void_v<T>)
  {
    if (m_exception) {
      std::rethrow_exception(m_exception);
    }
  }

  // Store continuation for resumption
  void set_continuation(std::coroutine_handle<> p_continuation) noexcept
  {
    m_continuation = p_continuation;
  }

  // Resume stored continuation
  void resume_continuation() noexcept
  {
    if (m_continuation) {
      auto handle = m_continuation;
      m_continuation = nullptr;
      handle.resume();
    }
  }

  // Get allocator for child coroutines
  [[nodiscard]] std::pmr::memory_resource* get_allocator() const noexcept
  {
    return m_allocator;
  }

  // Custom allocation using our allocator
  static void* operator new(std::size_t size,
                            std::pmr::memory_resource* p_allocator)
  {
    return std::pmr::polymorphic_allocator<>(p_allocator).allocate(size);
  }

  // Custom deletion
  static void operator delete(void*)
  {
    // Deallocation is handled in destroy()
  }
};

// Task class template
template<typename T>
class task
{
private:
  std::coroutine_handle<hal_promise_type<T>> m_handle;

  explicit task(std::coroutine_handle<hal_promise_type<T>> p_handle) noexcept
    : m_handle(p_handle)
  {
  }

  template<typename U>
  friend class hal_promise_type;

public:
  // Coroutine promise type
  using promise_type = hal_promise_type<T>;

  // Default constructor for empty task
  task() noexcept = default;

  // Move constructor
  task(task&& other) noexcept
    : m_handle(std::exchange(other.m_handle, {}))
  {
  }

  // Move assignment
  task& operator=(task&& other) noexcept
  {
    if (this != &other) {
      if (m_handle) {
        m_handle.destroy();
      }
      m_handle = std::exchange(other.m_handle, {});
    }
    return *this;
  }

  // Destructor - clean up coroutine
  ~task()
  {
    if (m_handle) {
      m_handle.destroy();
    }
  }

  // Awaiter for when this task is awaited
  struct basic_awaiter
  {
    std::coroutine_handle<hal_promise_type<T>> m_handle;

    // For allocator propagation
    std::pmr::memory_resource* m_allocator;

    // Constructor with allocator
    basic_awaiter(std::coroutine_handle<hal_promise_type<T>> p_handle,
                  std::pmr::memory_resource* p_allocator) noexcept
      : m_handle(p_handle)
      , m_allocator(p_allocator)
    {
    }

    [[nodiscard]] bool await_ready() const noexcept
    {
      return !m_handle || m_handle.done();
    }

    // Generic await_suspend - captures continuation for resumption
    template<typename Promise>
    void await_suspend(std::coroutine_handle<Promise> continuation) noexcept
    {
      m_handle.promise().set_continuation(continuation);
    }

    // Specialized await_suspend for hal_promise_type - propagates allocator
    template<typename V>
    void await_suspend(
      std::coroutine_handle<hal_promise_type<V>> continuation) noexcept
    {
      // Store continuation for resumption
      m_allocator = continuation.promise().get_allocator();
      m_handle.promise().set_continuation(continuation);
    }

    T await_resume()
    {
      if constexpr (std::is_void_v<T>) {
        m_handle.promise().result();
      } else {
        return m_handle.promise().result();
      }
    }
  };

  using awaiter = basic_awaiter;

  // This is the key function that propagates the allocator during co_await
  // When a task is awaited, this returns an awaiter that captures the caller's
  // allocator
  auto operator co_await() const noexcept
  {
    // No allocator available in this context
    return awaiter{ m_handle, m_handle.promise().get_allocator() };
  }

  // Run synchronously and return result
  T get_result()
  {
    if (!m_handle) {
      throw std::runtime_error("Task has no associated coroutine");
    }

    while (!m_handle.done()) {
      // Allow processing to continue
    }

    if constexpr (std::is_void_v<T>) {
      m_handle.promise().result();
      return;
    } else {
      return m_handle.promise().result();
    }
  }
};

// Implement get_return_object now that task is defined
template<typename T>
task<T> hal_promise_type<T>::get_return_object() noexcept
{
  return task<T>{ std::coroutine_handle<hal_promise_type<T>>::from_promise(
    *this) };
}

// Utility function to run a task synchronously
template<typename T>
T sync_wait(task<T> p_task)
{
  return p_task.get_result();
}
}  // namespace hal::v5
