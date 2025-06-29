#include <array>
#include <coroutine>
#include <cstddef>
#include <cstdint>
#include <cstdio>

#include <libhal-util/bit.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/experimental/coroutine.hpp>
#include <libhal/pointers.hpp>

#include <memory_resource>
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

  auto configure(hal::v5::async_context& p_context, settings const& p_settings)
  {
    return driver_configure(p_context, p_settings);
  }

  auto level(hal::v5::async_context& p_context, bool p_high)
  {
    return driver_level(p_context, p_high);
  }

  [[nodiscard]] auto level(hal::v5::async_context& p_context)
  {
    return driver_level(p_context);
  }

private:
  virtual hal::v5::async<void> driver_configure(
    hal::v5::async_context& p_context,
    settings const& p_settings) = 0;
  virtual hal::v5::async<void> driver_level(hal::v5::async_context& p_context,
                                            bool p_high) = 0;
  virtual hal::v5::async<bool> driver_level(
    hal::v5::async_context& p_context) = 0;
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
  hal::v5::async<void> driver_configure(hal::v5::async_context&,
                                        settings const&) override
  {
    co_return;
  }

  hal::v5::async<void> driver_level(hal::v5::async_context&,
                                    bool p_high) override
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
    return hal::v5::async<void>{};  // sync return
  }
  hal::v5::async<bool> driver_level(hal::v5::async_context&) override
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
  hal::v5::async<void> driver_configure(hal::v5::async_context&,
                                        settings const&) override
  {
    co_return;
  }

  hal::v5::async<void> driver_level(hal::v5::async_context&,
                                    bool p_high) override
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
  hal::v5::async<bool> driver_level(hal::v5::async_context&) override
  {
    auto const pin_value =
      hal::bit_extract(hal::bit_mask::from(m_pin), gpio_reg(m_port).idr);
    co_return static_cast<bool>(pin_value);
  }

  hal::u8 m_port = 'B';
  hal::u8 m_pin = 13;
};

class pre_alloc_coro_async_pin : public async_output_pin
{
public:
  pre_alloc_coro_async_pin(std::pmr::memory_resource* m_resource)
    : m_runtime(*m_resource, 256, [](auto&&...) {})
    , m_self_context(m_runtime.lease(0))
    , m_level_write_coroutine(this->driver_level_impl(*m_self_context))
  {
  }

private:
  hal::v5::async<void> driver_configure(hal::v5::async_context&,
                                        settings const&) override
  {
    co_return;
  }

  hal::v5::async<void> driver_level_impl(hal::v5::async_context&)
  {
    while (true) {
      if (m_pin_state) {
        // The first 16 bits of the register set the output state
        gpio_reg(m_port).bsrr = 1 << m_pin;
      } else {
        // The last 16 bits of the register reset the output state
        gpio_reg(m_port).bsrr = 1 << (16 + m_pin);
      }
      // m_active_context->active_handle(nullptr);
      // co_return;
      co_await hal::v5::async_promise_type<void>::final_awaiter{};
    }
  }

  hal::v5::async<void> driver_level(hal::v5::async_context& p_ctx,
                                    bool p_high) override
  {
    m_pin_state = p_high;
    // p_ctx.active_handle(m_level_write_coroutine.handle());
    m_level_write_coroutine.set_context(p_ctx);
    return std::move(m_level_write_coroutine);
  }
  hal::v5::async<bool> driver_level(hal::v5::async_context&) override
  {
    auto const pin_value =
      hal::bit_extract(hal::bit_mask::from(m_pin), gpio_reg(m_port).idr);
    co_return static_cast<bool>(pin_value);
  }

  std::coroutine_handle<> m_previous_handler = nullptr;
  hal::v5::async_runtime<1> m_runtime;
  hal::v5::context_lease m_self_context;
  hal::v5::async<void> m_level_write_coroutine;
  hal::u8 m_port = 'B';
  hal::u8 m_pin = 13;
  bool m_pin_state = false;
};

class coro_forever_async_pin : public async_output_pin
{
private:
  hal::v5::async<void> driver_configure(hal::v5::async_context&,
                                        settings const&) override
  {
    co_return;
  }

  hal::v5::async<void> driver_level(hal::v5::async_context&,
                                    bool p_high) override
  {
    while (true) {
      if (p_high) {
        // The first 16 bits of the register set the output state
        gpio_reg(m_port).bsrr = 1 << m_pin;
      } else {
        // The last 16 bits of the register reset the output state
        gpio_reg(m_port).bsrr = 1 << (16 + m_pin);
      }
      co_await std::suspend_always{};
    }
    co_return;
  }
  hal::v5::async<bool> driver_level(hal::v5::async_context&) override
  {
    auto const pin_value =
      hal::bit_extract(hal::bit_mask::from(m_pin), gpio_reg(m_port).idr);
    co_return static_cast<bool>(pin_value);
  }

  hal::u8 m_port = 'B';
  hal::u8 m_pin = 13;
};

class pseudo_coro_output_pin
{
public:
  auto level(hal::v5::async_context& p_context, bool p_high)
  {
    return driver_level(p_context, p_high);
  }

private:
  virtual void driver_level(hal::v5::async_context& p_context, bool p_high) = 0;
};

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
coro_forever_async_pin coro_pin_forever_async_obj;

bool volatile switcher = false;

async_output_pin* get_pre_alloc_coro_async_pin(
  std::pmr::memory_resource* m_resource)
{
  if (not switcher) {
    static pre_alloc_coro_async_pin pre_alloc_coro_async_pin_obj(m_resource);
    return &pre_alloc_coro_async_pin_obj;
  }
  return &coro_pin_async_obj;
}

async_output_pin* get_sync_coro_pin()
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

hal::v5::async<void> sync_coro_set_pin(hal::v5::async_context&,
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

hal::v5::async<void> async_coro_set_pin(hal::v5::async_context&,
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

hal::v5::async<void> coro_pin_set_level(hal::v5::async_context& p_ctx,
                                        async_output_pin& p_pin,
                                        bool p_level)
{
  co_await p_pin.level(p_ctx, p_level);
}

hal::v5::async<void> coro_await_task_loop_deeper(hal::v5::async_context& p_ctx,
                                                 async_output_pin& p_pin,
                                                 int p_cycle_count)
{
  for (int i = 0; i < p_cycle_count; i++) {
    co_await coro_pin_set_level(p_ctx, p_pin, false);
    co_await coro_pin_set_level(p_ctx, p_pin, true);
  }
  co_return;
}

hal::v5::async<void> coro_await_task_loop(hal::v5::async_context& p_ctx,
                                          async_output_pin& p_pin,
                                          int p_cycle_count)
{
  for (int i = 0; i < p_cycle_count; i++) {
    co_await p_pin.level(p_ctx, false);
    co_await p_pin.level(p_ctx, true);
  }
  co_return;
}

async_output_pin* get_forever_coro_pin()
{
  if (not switcher) {
    return &coro_pin_forever_async_obj;
  }
  return &coro_pin_async_obj;
}

hal::u32 fast_uptime()
{
  return *std::bit_cast<hal::u32*>(0xE0001004);
}

hal::u64 benchmark_output_pin_virtual(hal::output_pin& p_pin)
{
  auto const start = fast_uptime();
  p_pin.level(false);
  auto const end = fast_uptime();
  p_pin.level(true);

  return end - start;
}

hal::u64 measure_timing_overhead(hal::steady_clock& p_clock)
{
  // Measure the exact same pattern you use in your real benchmark
  auto start = p_clock.uptime();

  // Do absolutely nothing (but prevent optimization)
  asm volatile("" ::: "memory");

  auto end = p_clock.uptime();

  return end - start;
}

hal::u64 volatile creation_overhead = 0;
hal::u64 volatile creation_and_resume_overhead = 0;
hal::u64 volatile uptime_overhead = 0;
hal::u64 volatile virtual_overhead = 0;
hal::u64 volatile full_setup_time = 0;
hal::u64 volatile fast_uptime_overhead = 0;

hal::u64 fast_measure_timing_overhead()
{
  // Measure the exact same pattern you use in your real benchmark
  auto start = fast_uptime();

  // Do absolutely nothing (but prevent optimization)
  asm volatile("" ::: "memory");

  auto end = fast_uptime();

  return end - start;
}

#if 0
// Converts output_pin into a synchonous API, will block until finished
class sync_ouput_pin
{
public:
  sync_ouput_pin(hal::v5::strong_ptr<async_output_pin> p_pin)
    : m_pin(std::move(p_pin))
  {
  }

  void level(bool p_value)
  {
    hal::async_thread_manager manager(std::pmr::new_delete_resource(), );
    hal::async_thread_manager manager;
  }
  bool level()
  {
  }
  void configure(settings const& p_settings)
  {
  }

private:
  hal::v5::strong_ptr<async_output_pin> m_pin;
};
#endif

void application(resource_list& p_map)
{
  std::array<hal::byte, 1024 * 2> coroutine_stack;
  std::pmr::monotonic_buffer_resource coroutine_resource(
    coroutine_stack.data(),
    coroutine_stack.size(),
    std::pmr::null_memory_resource());

  hal::v5::async_runtime<2> manager(
    coroutine_resource, sizeof(coroutine_stack) / 2, [](auto const&...) {});
  auto ctx = manager.lease(0);
  auto ctx1 = manager.lease(1);

  auto& clock = *p_map.clock.value();
  auto c_vtable = get_c_output_pin_vtable();
  auto c_output_pin_obj = get_c_output_pin_object('B', 13);
  auto& led = *p_map.status_led.value();
  async_output_pin* sync_coro_pin = get_sync_coro_pin();
  async_output_pin* coro_pin_async = get_async_coro_pin();
  [[maybe_unused]] async_output_pin* coro_forever = get_forever_coro_pin();

  std::array<hal::byte, 256> pre_alloc_buffer{};
  std::pmr::monotonic_buffer_resource pre_alloc_buffer_resource(
    pre_alloc_buffer.data(),
    pre_alloc_buffer.size(),
    std::pmr::null_memory_resource());
  [[maybe_unused]] async_output_pin* coro_pre =
    get_pre_alloc_coro_async_pin(&pre_alloc_buffer_resource);

  while (true) {
    using namespace std::chrono_literals;
    constexpr auto delay_time = 500us;
    for (int i = 0; i < 900; i++) {
      set_pin('B', 13, false);
      set_pin('B', 13, true);
    }
    hal::delay(clock, delay_time);
    for (int i = 0; i < 800; i++) {
      sync_coro_set_pin(*ctx, 'B', 13, false).sync_wait();
      sync_coro_set_pin(*ctx, 'B', 13, true).sync_wait();
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

    virtual_overhead = benchmark_output_pin_virtual(led);

    hal::delay(clock, delay_time);
    for (int i = 0; i < 400; i++) {
      sync_coro_pin->level(*ctx, false).sync_wait();
      sync_coro_pin->level(*ctx, true).sync_wait();
    }

    hal::delay(clock, delay_time);
    for (int i = 0; i < 300; i++) {
      async_coro_set_pin(*ctx, 'B', 13, false).sync_wait();
      async_coro_set_pin(*ctx, 'B', 13, true).sync_wait();
    }

    hal::delay(clock, delay_time);
    for (int i = 0; i < 200; i++) {
      coro_pin_async->level(*ctx, false).sync_wait();
      coro_pin_async->level(*ctx, true).sync_wait();
    }

    hal::delay(clock, delay_time);
#if 0
    for (int i = 0; i < 175; i++) {
      coro_pre->level(*ctx, false).sync_wait();
      coro_pre->level(*ctx, true).sync_wait();
    }
    hal::delay(clock, delay_time);
    // coro_await_task_loop(*ctx, *coro_pre, 175).sync_wait();

    hal::delay(clock, delay_time);
    coro_await_task_loop(*ctx, *coro_pin_async, 150).sync_wait();

    hal::delay(clock, delay_time);
    coro_await_task_loop_deeper(*ctx, *coro_pin_async, 130).sync_wait();

    hal::delay(clock, delay_time);
    coro_await_task_loop_deeper(*ctx, *sync_coro_pin, 120).sync_wait();

    hal::delay(clock, delay_time);
    coro_await_task_loop(*ctx, *sync_coro_pin, 100).sync_wait();

    fast_uptime_overhead = fast_measure_timing_overhead();

    hal::delay(clock, delay_time);
    auto const creation_overhead_start = fast_uptime();
    auto set_high = coro_forever->level(*ctx, true);
    auto const creation_overhead_end = fast_uptime();
    creation_overhead = creation_overhead_end - creation_overhead_start;

    auto const creation_and_resume_overhead_start = fast_uptime();
    auto set_low = coro_forever->level(*ctx1, false);
    set_low.resume();
    auto const creation_and_resume_overhead_end = fast_uptime();
    creation_and_resume_overhead =
      creation_and_resume_overhead_end - creation_and_resume_overhead_start;
    for (int i = 0; i < 75; i++) {
      set_low.resume();
      set_high.resume();
    }

    hal::delay(clock, delay_time);
    std::array<hal::v5::async<void>*, 2> list{
      &set_low,
      &set_high,
    };
    for (int i = 0; i < 50; i++) {  // round robin
      for (auto task : list) {
        if (task->handle() && not task->handle().done()) {
          task->resume();
        }
      }
    }

    hal::delay(clock, delay_time * 15);
#endif
  }
}

#if 0
void my_app()
{
  std::array<hal::byte, 1024> coroutine_stack{};
  hal::v5::async_thread_manager manager(
    coroutine_resource, sizeof(coroutine_stack), [](auto const&...) {});
  hal::v5::async_context ctx = manager.entire_context();
}
#endif
