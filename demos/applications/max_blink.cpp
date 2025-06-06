#include <algorithm>
#include <array>
#include <coroutine>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <memory_resource>
#include <numeric>
#include <span>
#include <stdexcept>
#include <string>
#include <syncstream>
#include <utility>
#include <variant>

namespace hal {
using byte = std::uint8_t;
using usize = std::size_t;

class async_context
{
public:
  explicit async_context(std::pmr::memory_resource& p_resource)
    : m_resource(&p_resource)
  {
  }

  async_context() = default;

  [[nodiscard]] constexpr auto* resource()
  {
    return m_resource;
  }

  constexpr auto last_allocation_size()
  {
    return m_last_allocation_size;
  }

  constexpr void last_allocation_size(usize p_last_allocation_size)
  {
    m_last_allocation_size = p_last_allocation_size;
  }

  constexpr std::coroutine_handle<> active_handle()
  {
    return m_active_handle;
  }

  constexpr void active_handle(std::coroutine_handle<> p_active_handle)
  {
    m_active_handle = p_active_handle;
  }

private:
  std::pmr::memory_resource* m_resource = nullptr;
  usize m_last_allocation_size = 0;
  std::coroutine_handle<> m_active_handle = std::noop_coroutine();
};

class task_promise_base
{
public:
  // For regular functions
  template<typename... Args>
  static constexpr void* operator new(std::size_t p_size,
                                      async_context& p_context,
                                      Args&&...)
  {
    p_context.last_allocation_size(p_size);
    return p_context.resource()->allocate(p_size);
  }

  // For member functions - handles the implicit 'this' parameter
  template<typename Class, typename... Args>
  static constexpr void* operator new(std::size_t p_size,
                                      Class&,  // The 'this' object
                                      async_context& p_context,
                                      Args&&...)
  {
    p_context.last_allocation_size(p_size);
    return p_context.resource()->allocate(p_size);
  }
  // Add regular delete operators for normal coroutine destruction
  static constexpr void operator delete(void*) noexcept
  {
  }

  static constexpr void operator delete(void*, std::size_t) noexcept
  {
  }

  // Constructor for regular functions
  task_promise_base(async_context& p_context)
  {
    m_context = &p_context;
    m_frame_size = p_context.last_allocation_size();
  }

  // Constructor for member functions (handles 'this' parameter)
  template<typename Class>
  task_promise_base(Class&, async_context& p_context)
  {
    m_context = &p_context;
    m_frame_size = p_context.last_allocation_size();
  }

  // Generic constructor for additional parameters
  template<typename... Args>
  task_promise_base(async_context& p_context, Args&&...)
  {
    m_context = &p_context;
    m_frame_size = p_context.last_allocation_size();
  }

  // Constructor for member functions with additional parameters
  template<typename Class, typename... Args>
  task_promise_base(Class&, async_context& p_context, Args&&...)
  {
    m_context = &p_context;
    m_frame_size = p_context.last_allocation_size();
  }

  constexpr std::suspend_always initial_suspend() noexcept
  {
    return {};
  }

  template<typename U>
  constexpr U&& await_transform(U&& awaitable) noexcept
  {
    return static_cast<U&&>(awaitable);
  }

  void unhandled_exception() noexcept
  {
    m_error = std::current_exception();
  }
  constexpr auto& context()
  {
    return *m_context;
  }
  constexpr void context(async_context& p_context)
  {
    m_context = &p_context;
  }
  constexpr auto continuation()
  {
    return m_continuation;
  }
  constexpr void continuation(std::coroutine_handle<> p_continuation)
  {
    m_continuation = p_continuation;
  }

  [[nodiscard]] constexpr auto frame_size() const
  {
    return m_frame_size;
  }
  constexpr std::coroutine_handle<> pop_active_coroutine()
  {
    m_context->active_handle(m_continuation);
    return m_continuation;
  }

protected:
  // Storage for the coroutine result/error
  std::coroutine_handle<> m_continuation{};
  async_context* m_context{};
  // NOLINTNEXTLINE(bugprone-throw-keyword-missing)
  std::exception_ptr m_error{};
  usize m_frame_size = 0;
};

template<typename T>
class async;

// Helper type for void
struct void_placeholder
{};

// Type selection for void vs non-void
template<typename T>
using void_to_placeholder_t =
  std::conditional_t<std::is_void_v<T>, void_placeholder, T>;

template<typename T>
class task_promise_type : public task_promise_base
{
public:
  using task_promise_base::task_promise_base;  // Inherit constructors
  using task_promise_base::operator new;

  // Add regular delete operators for normal coroutine destruction
  static constexpr void operator delete(void*) noexcept
  {
  }

  static constexpr void operator delete(void*, std::size_t) noexcept
  {
  }

  struct final_awaiter
  {
    constexpr bool await_ready() noexcept
    {
      return false;
    }

    template<typename U>
    std::coroutine_handle<> await_suspend(
      std::coroutine_handle<task_promise_type<U>> p_self) noexcept
    {
      // The coroutine is now suspended at the final-suspend point.
      // Lookup its continuation in the promise and resume it symmetrically.
      //
      // Rather than return control back to the application, we continue the
      // caller function allowing it to yield when it reaches another suspend
      // point. The idea is that prior to this being called, we were executing
      // code and thus, when we resume the caller, we are still running code.
      // Lets continue to run as much code until we reach an actual suspend
      // point.
      return p_self.promise().pop_active_coroutine();
    }

    void await_resume() noexcept
    {
    }
  };

  final_awaiter final_suspend() noexcept
  {
    return {};
  }

  constexpr async<T> get_return_object() noexcept;

  // For non-void return type
  template<typename U = T>
  void return_value(U&& p_value) noexcept
    requires(not std::is_void_v<T>)
  {
    m_value = std::forward<U>(p_value);
  }

  auto result()
  {
    return m_value;
  }

  void_to_placeholder_t<T> m_value{};
};

template<>
class task_promise_type<void> : public task_promise_base
{
public:
  using task_promise_base::task_promise_base;  // Inherit constructors
  using task_promise_base::operator new;
  // using task_promise_base::operator delete;

  task_promise_type();

  constexpr void return_void() noexcept
  {
  }

  constexpr async<void> get_return_object() noexcept;

  // Add regular delete operators for normal coroutine destruction
  static constexpr void operator delete(void*) noexcept
  {
  }

  static constexpr void operator delete(void*, std::size_t) noexcept
  {
  }

  struct final_awaiter
  {
    constexpr bool await_ready() noexcept
    {
      return false;
    }

    template<typename U>
    std::coroutine_handle<> await_suspend(
      std::coroutine_handle<task_promise_type<U>> p_self) noexcept
    {
      // The coroutine is now suspended at the final-suspend point.
      // Lookup its continuation in the promise and resume it symmetrically.
      //
      // Rather than return control back to the application, we continue the
      // caller function allowing it to yield when it reaches another suspend
      // point. The idea is that prior to this being called, we were executing
      // code and thus, when we resume the caller, we are still running code.
      // Lets continue to run as much code until we reach an actual suspend
      // point.
      return p_self.promise().pop_active_coroutine();
    }

    void await_resume() noexcept
    {
    }
  };

  final_awaiter final_suspend() noexcept
  {
    return {};
  }
};

template<typename T = void>
class async
{
public:
  using promise_type = task_promise_type<T>;
  friend promise_type;

  void resume()
  {
    auto active = m_handle.promise().context().active_handle();
    active.resume();
  }

  // Run synchronously and return result
  T sync_result()
  {
    if (not m_handle) {
      return T{};
    }
    auto& context = m_handle.promise().context();

    while (not m_handle.done()) {
      auto active = context.active_handle();
      active.resume();
    }

    if constexpr (not std::is_void_v<T>) {
      return m_handle.promise().result();
    }
  }

  // Awaiter for when this task is awaited
  struct awaiter
  {
    std::coroutine_handle<promise_type> m_handle;

    explicit awaiter(std::coroutine_handle<promise_type> p_handle) noexcept
      : m_handle(p_handle)
    {
    }

    [[nodiscard]] constexpr bool await_ready() const noexcept
    {
      return not m_handle;
    }

    // Generic await_suspend for any promise type
    template<typename Promise>
    std::coroutine_handle<> await_suspend(
      std::coroutine_handle<Promise> p_continuation) noexcept
    {
      m_handle.promise().continuation(p_continuation);
      return m_handle;
    }

    T await_resume()
    {
      if constexpr (not std::is_void_v<T>) {
        if (m_handle) {
          return m_handle.promise().result();
        }
      }
    }
  };

  [[nodiscard]] constexpr awaiter operator co_await() const noexcept
  {
    return awaiter{ m_handle };
  }

  async() noexcept = default;
  async(async&& p_other) noexcept
    : m_handle(std::exchange(p_other.m_handle, {}))
  {
  }

  ~async()
  {
    if (m_handle) {
      void* const address = m_handle.address();
      auto* const allocator = m_handle.promise().context().resource();
      auto const frame_size = m_handle.promise().frame_size();
      m_handle.destroy();
      allocator->deallocate(address, frame_size);
    }
  }

  async& operator=(async&& p_other) noexcept
  {
    if (this != &p_other) {
      if (m_handle) {
        m_handle.destroy();
      }
      m_handle = std::exchange(p_other.m_handle, {});
    }
    return *this;
  }

  auto handle()
  {
    return m_handle;
  }

private:
  explicit async(std::coroutine_handle<promise_type> p_handle)
    : m_handle(p_handle)
  {
    m_handle.promise().continuation(std::noop_coroutine());
    m_handle.promise().context().active_handle(m_handle);
  }

  std::coroutine_handle<promise_type> m_handle;
};

template<typename T>
constexpr async<T> task_promise_type<T>::get_return_object() noexcept
{
  return async<T>{ std::coroutine_handle<task_promise_type<T>>::from_promise(
    *this) };
}

constexpr async<void> task_promise_type<void>::get_return_object() noexcept
{
  return async<void>{
    std::coroutine_handle<task_promise_type<void>>::from_promise(*this)
  };
}

class coroutine_stack_memory_resource : public std::pmr::memory_resource
{
public:
  constexpr coroutine_stack_memory_resource(std::span<hal::byte> p_memory)
    : m_memory(p_memory)
  {
    if (p_memory.data() == nullptr || p_memory.size() < 32) {
      throw std::runtime_error(
        "Coroutine stack memory invalid! Must be non-null and size > 32.");
    }
  }

private:
  void* do_allocate(std::size_t p_bytes, std::size_t) override
  {
    auto* const new_stack_pointer = &m_memory[m_stack_pointer];
    m_stack_pointer += p_bytes;
    return new_stack_pointer;
  }
  void do_deallocate(void*, std::size_t p_bytes, std::size_t) override
  {
    m_stack_pointer -= p_bytes;
  }

  [[nodiscard]] bool do_is_equal(
    std::pmr::memory_resource const& other) const noexcept override
  {
    return this == &other;
  }

  std::span<hal::byte> m_memory;
  hal::usize m_stack_pointer = 0;
};
}  // namespace hal

#include <libhal-util/bit.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

#include <resource_list.hpp>

/**
 * @brief Digital output pin hardware abstraction.
 *
 * Use this to drive a pin HIGH or LOW in order to send a control signal or turn
 * off or on an LED.
 *
 * Implementations of this interface can be backed by external devices such as
 * I/O expanders or other micro-controllers.
 *
 */
class async_output_pin
{
public:
  /// Generic settings for output pins
  struct settings
  {
    /// Pull resistor for the pin. This generally only helpful when open
    /// drain is enabled.
    hal::pin_resistor resistor = hal::pin_resistor::none;

    /// Starting level of the output pin. HIGH voltage defined as true and LOW
    /// voltage defined as false.
    bool open_drain = false;

    /**
     * @brief Enables default comparison
     *
     */
    bool operator<=>(settings const&) const = default;
  };

  auto configure(hal::async_context& p_context, settings const& p_settings)
  {
    return driver_configure(p_context, p_settings);
  }

  auto level(hal::async_context& p_context, bool p_high)
  {
    return driver_level(p_context, p_high);
  }

  [[nodiscard]] auto level(hal::async_context& p_context)
  {
    return driver_level(p_context);
  }

private:
  virtual hal::async<void> driver_configure(hal::async_context& p_context,
                                            settings const& p_settings) = 0;
  virtual hal::async<void> driver_level(hal::async_context& p_context,
                                        bool p_high) = 0;
  virtual hal::async<bool> driver_level(hal::async_context& p_context) = 0;
};

struct gpio_t
{
  hal::u32 volatile crl;
  hal::u32 volatile crh;
  hal::u32 volatile idr;
  hal::u32 volatile odr;
  hal::u32 volatile bsrr;
  hal::u32 volatile brr;
  hal::u32 volatile lckr;
};
inline constexpr intptr_t ahb_base = 0x20080000UL;

inline static std::array<gpio_t*, 8> gpio_reg_map{
  reinterpret_cast<gpio_t*>(0x4001'0800),  // 'A'
  reinterpret_cast<gpio_t*>(0x4001'0c00),  // 'B'
  reinterpret_cast<gpio_t*>(0x4001'1000),  // 'C'
  reinterpret_cast<gpio_t*>(0x4001'1400),  // 'D'
  reinterpret_cast<gpio_t*>(0x4001'1800),  // 'E'
  reinterpret_cast<gpio_t*>(0x4001'1c00),  // 'F'
  reinterpret_cast<gpio_t*>(0x4001'2000),  // 'G'
};

inline static gpio_t& gpio_reg(hal::u8 p_port)
{
  auto const offset = p_port - 'A';
  return *gpio_reg_map[offset];
}

// Add a volatile operation to prevent optimization
[[maybe_unused]] uint32_t volatile dummy = 0;

class coro_sync_pin : public async_output_pin
{
private:
  hal::async<void> driver_configure(hal::async_context&,
                                    settings const&) override
  {
    co_return;
  }

  hal::async<void> driver_level(hal::async_context&, bool p_high) override
  {
    dummy = gpio_reg(m_port).odr;  // Force a read

    if (p_high) {
      // The first 16 bits of the register set the output state
      gpio_reg(m_port).bsrr = 1 << m_pin;
    } else {
      // The last 16 bits of the register reset the output state
      gpio_reg(m_port).bsrr = 1 << (16 + m_pin);
    }
    // co_return;
    return hal::async<void>{};  // sync return
  }
  hal::async<bool> driver_level(hal::async_context&) override
  {
    auto const pin_value =
      hal::bit_extract(hal::bit_mask::from(m_pin), gpio_reg(m_port).idr);
    co_return static_cast<bool>(pin_value);
  }

  hal::u8 m_port = 'B';
  hal::u8 m_pin = 13;
};
class coro_async_pin : public async_output_pin
{
private:
  hal::async<void> driver_configure(hal::async_context&,
                                    settings const&) override
  {
    co_return;
  }

  hal::async<void> driver_level(hal::async_context&, bool p_high) override
  {
    if (p_high) {
      // The first 16 bits of the register set the output state
      gpio_reg(m_port).bsrr = 1 << m_pin;
    } else {
      // The last 16 bits of the register reset the output state
      gpio_reg(m_port).bsrr = 1 << (16 + m_pin);
    }
    co_return;
  }
  hal::async<bool> driver_level(hal::async_context&) override
  {
    auto const pin_value =
      hal::bit_extract(hal::bit_mask::from(m_pin), gpio_reg(m_port).idr);
    co_return static_cast<bool>(pin_value);
  }

  hal::u8 m_port = 'B';
  hal::u8 m_pin = 13;
};

std::array<hal::byte, 512> coroutine_stack;
hal::coroutine_stack_memory_resource coroutine_resource(coroutine_stack);
hal::async_context ctx(coroutine_resource);

void set_pin(hal::u8 p_port, hal::u8 p_pin, bool p_state)
{
  if (p_state) {
    // The first 16 bits of the register set the output state
    gpio_reg(p_port).bsrr = 1 << p_pin;
  } else {
    // The last 16 bits of the register reset the output state
    gpio_reg(p_port).bsrr = 1 << (16 + p_pin);
  }
}

coro_sync_pin coro_pin_obj;
coro_async_pin coro_pin_async_obj;

bool volatile switcher = false;

async_output_pin* get_coro_pin()
{
  if (not switcher) {
    return &coro_pin_obj;
  }
  return &coro_pin_async_obj;
}
async_output_pin* get_async_coro_pin()
{
  if (not switcher) {
    return &coro_pin_async_obj;
  }
  return &coro_pin_obj;
}

struct c_output_pin_vtable
{
  decltype(set_pin)* set_pin_funct = nullptr;
};

c_output_pin_vtable get_c_output_pin_vtable()
{
  if (not switcher) {
    return { .set_pin_funct = &set_pin };
  }
  return { nullptr };  // Will crash
}

struct c_output_pin_object
{
  void (*set_pin_funct)(c_output_pin_object*, bool p_state) = nullptr;
  hal::u8 m_port = 0;
  hal::u8 m_pin = 0;
};

static void my_set_pin_funct(c_output_pin_object* p_self, bool p_state)
{
  if (p_state) {
    // The first 16 bits of the register set the output state
    gpio_reg(p_self->m_port).bsrr = 1 << p_self->m_pin;
  } else {
    // The last 16 bits of the register reset the output state
    gpio_reg(p_self->m_port).bsrr = 1 << (16 + p_self->m_pin);
  }
}

c_output_pin_object get_c_output_pin_object(hal::u8 p_port, hal::u8 p_pin)
{
  if (not switcher) {
    return {
      .set_pin_funct = &my_set_pin_funct,
      .m_port = p_port,
      .m_pin = p_pin,
    };
  }
  return {};  // Will crash
}

hal::async<void> sync_coro_set_pin(hal::async_context&,
                                   hal::u8 p_port,
                                   hal::u8 p_pin,
                                   bool p_state)
{
  if (p_state) {
    // The first 16 bits of the register set the output state
    gpio_reg(p_port).bsrr = 1 << p_pin;
  } else {
    // The last 16 bits of the register reset the output state
    gpio_reg(p_port).bsrr = 1 << (16 + p_pin);
  }
  return {};
}

hal::async<void> async_coro_set_pin(hal::async_context&,
                                    hal::u8 p_port,
                                    hal::u8 p_pin,
                                    bool p_state)
{
  if (p_state) {
    // The first 16 bits of the register set the output state
    gpio_reg(p_port).bsrr = 1 << p_pin;
  } else {
    // The last 16 bits of the register reset the output state
    gpio_reg(p_port).bsrr = 1 << (16 + p_pin);
  }
  co_return;
}

void application(resource_list& p_map)
{
  auto& clock = *p_map.clock.value();
  auto c_vtable = get_c_output_pin_vtable();
  auto c_output_pin_obj = get_c_output_pin_object('B', 13);
  auto& led = *p_map.status_led.value();
  async_output_pin* coro_pin = get_coro_pin();
  async_output_pin* coro_pin_async = get_async_coro_pin();

  while (true) {
    using namespace std::chrono_literals;
    constexpr auto delay_time = 500us;
    for (int i = 0; i < 900; i++) {
      set_pin('B', 13, false);
      set_pin('B', 13, true);
    }
    hal::delay(clock, delay_time);
    for (int i = 0; i < 800; i++) {
      sync_coro_set_pin(ctx, 'B', 13, false).sync_result();
      sync_coro_set_pin(ctx, 'B', 13, true).sync_result();
    }
    hal::delay(clock, delay_time);
    for (int i = 0; i < 700; i++) {
      c_vtable.set_pin_funct('B', 13, false);
      c_vtable.set_pin_funct('B', 13, true);
    }
    hal::delay(clock, delay_time);
    for (int i = 0; i < 600; i++) {
      c_output_pin_obj.set_pin_funct(&c_output_pin_obj, false);
      c_output_pin_obj.set_pin_funct(&c_output_pin_obj, true);
    }
    hal::delay(clock, delay_time);
    for (int i = 0; i < 500; i++) {
      led.level(false);
      led.level(true);
    }
    hal::delay(clock, delay_time);
    for (int i = 0; i < 400; i++) {
      coro_pin->level(ctx, false).sync_result();
      coro_pin->level(ctx, true).sync_result();
    }
    hal::delay(clock, delay_time);
    for (int i = 0; i < 300; i++) {
      async_coro_set_pin(ctx, 'B', 13, false).sync_result();
      async_coro_set_pin(ctx, 'B', 13, true).sync_result();
    }
    hal::delay(clock, delay_time);
    for (int i = 0; i < 200; i++) {
      coro_pin_async->level(ctx, false).sync_result();
      coro_pin_async->level(ctx, true).sync_result();
    }
    hal::delay(clock, delay_time);
  }
}
