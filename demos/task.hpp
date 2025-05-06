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

// Forward declaration for task
template<typename T>
class task;

// Primary promise type
template<typename T>
class hal_promise_type
{
private:
  using value_type = detail::void_to_placeholder_t<T>;

  std::pmr::polymorphic_allocator<> m_allocator;
  std::exception_ptr m_exception = nullptr;
  std::coroutine_handle<> m_continuation = nullptr;
  value_type m_result{};

public:
  // Constructor that captures allocator
  explicit hal_promise_type(std::pmr::polymorphic_allocator<> p_allocator)
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
  void return_value(T&& value)
    requires(!std::is_void_v<T>)
  {
    m_result = std::forward<T>(value);
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
  [[nodiscard]] std::pmr::polymorphic_allocator<> get_allocator() const noexcept
  {
    return m_allocator;
  }

  // Custom allocation using our allocator
  static void* operator new(std::size_t size,
                            std::pmr::polymorphic_allocator<> p_allocator)
  {
    return p_allocator.allocate(size);
  }

  // Custom deletion - does nothing as deallocation is handled by destroy()
  static void operator delete(void*)
  {
    // Deallocation is handled elsewhere
  }
};

// Specialization of promise type for void
template<>
class hal_promise_type<void>
{
private:
  std::pmr::polymorphic_allocator<> m_allocator;
  std::exception_ptr m_exception = nullptr;
  std::coroutine_handle<> m_continuation = nullptr;

public:
  explicit hal_promise_type(std::pmr::polymorphic_allocator<> p_allocator)
    : m_allocator(p_allocator)
  {
  }

  task<void> get_return_object() noexcept;

  std::suspend_never initial_suspend() noexcept
  {
    return {};
  }
  std::suspend_always final_suspend() noexcept
  {
    return {};
  }

  void unhandled_exception() noexcept
  {
    m_exception = std::current_exception();
  }

  void result()
  {
    if (m_exception) {
      std::rethrow_exception(m_exception);
    }
  }

  void set_continuation(std::coroutine_handle<> p_continuation) noexcept
  {
    m_continuation = p_continuation;
  }

  void resume_continuation() noexcept
  {
    if (m_continuation) {
      auto handle = m_continuation;
      m_continuation = nullptr;
      handle.resume();
    }
  }

  [[nodiscard]] std::pmr::polymorphic_allocator<> get_allocator() const noexcept
  {
    return m_allocator;
  }

  static void* operator new(std::size_t size,
                            std::pmr::polymorphic_allocator<> p_allocator)
  {
    return p_allocator.allocate(size);
  }

  static void operator delete(void*)
  {
    // Deallocation is handled elsewhere
  }
};

// Primary task class
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
  struct awaiter
  {
    std::coroutine_handle<promise_type> m_handle;

    [[nodiscard]] bool await_ready() const noexcept
    {
      return !m_handle || m_handle.done();
    }

    // Generic await_suspend
    template<typename PromiseType>
    void await_suspend(std::coroutine_handle<PromiseType> continuation) noexcept
    {
      m_handle.promise().set_continuation(continuation);
    }

    // Specialized await_suspend that propagates allocator
    template<typename V>
    void await_suspend(std::coroutine_handle<typename task<V>::promise_type>
                         p_continuation) noexcept
    {
      // Store continuation for resumption
      m_handle.promise().set_continuation(p_continuation);

      // No need to propagate allocator as it was passed at creation time
      // But we could copy it here if needed:
      // auto allocator = continuation.promise().get_allocator();
      // Could store in m_handle.promise() if needed
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

  // Allow co_awaiting this task
  awaiter operator co_await() const noexcept
  {
    return awaiter{ m_handle };
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

// Implement get_return_object() now that task is defined
template<typename T>
task<T> hal_promise_type<T>::get_return_object() noexcept
{
  return task<T>{ std::coroutine_handle<hal_promise_type<T>>::from_promise(
    *this) };
}

#if 0
template<>
task<void> promise_type<void>::get_return_object() noexcept
{
  return task<void>{ std::coroutine_handle<promise_type<void>>::from_promise(
    *this) };
}
#endif

// Utility to run a task synchronously
template<typename T>
T sync_wait(task<T> p_task)
{
  return p_task.get_result();
}

// Helper to create a task with explicit allocator
template<typename Func>
auto make_task(Func&& func, std::pmr::polymorphic_allocator<> p_allocator)
{
  // Pass the allocator to the coroutine frame being created
  // The coroutine's promise constructor will receive this allocator
  co_await std::suspend_never{};

  if constexpr (std::is_invocable_v<Func>) {
    co_return func();
  } else {
    co_return func(p_allocator);
  }
}
}  // namespace hal::v5
