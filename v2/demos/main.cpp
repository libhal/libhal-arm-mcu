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

#include <cstdio>

#include <chrono>
#include <exception>
#include <new>

import hal;
import hal.util;
import hal.arm_mcu.cortex_m;
import async_context;
import arm_mcu_demos;

async::inplace_context<64> application_context{};
async::inplace_context<64> task1_context{};

struct resumer : public hal::timed_callback
{
  void callback() override
  {
    fired = true;
  }

  bool volatile fired = false;
};

resumer s_resumer{};

/**
 * @brief Adapts a hal::ptr<hal::steady_clock> to satisfy async::clock.
 *
 * async::run_until_done() needs a Clock whose now() is a plain synchronous
 * call, but hal::steady_clock's uptime()/frequency() are async APIs (they
 * return async::future<T> to accommodate steady clocks that need real I/O).
 * Concrete drivers like hal::cortex_m::dwt_counter never actually suspend —
 * their driver_uptime()/driver_frequency() just return an already-resolved
 * future — so now() can call them with a throwaway context and read the
 * value out immediately. This does not work for a steady_clock whose driver
 * genuinely needs to suspend to get the time.
 */
class steady_clock_adapter
{
public:
  using duration = std::chrono::duration<std::int64_t, std::nano>;

  struct time_point
  {
    constexpr time_point() = default;

    [[nodiscard]] static constexpr time_point max()
    {
      return time_point(duration::max());
    }

    friend constexpr duration operator-(time_point p_lhs, time_point p_rhs)
    {
      return p_lhs.m_since_epoch - p_rhs.m_since_epoch;
    }

    friend constexpr time_point operator+(time_point p_lhs, duration p_rhs)
    {
      return time_point(p_lhs.m_since_epoch + p_rhs);
    }

    friend constexpr auto operator<=>(time_point const&,
                                      time_point const&) = default;
    friend constexpr bool operator==(time_point const&,
                                     time_point const&) = default;

    friend class steady_clock_adapter;

    constexpr explicit time_point(duration p_since_epoch)
      : m_since_epoch(p_since_epoch)
    {
    }

    duration m_since_epoch{};
  };

  steady_clock_adapter(hal::ptr<hal::steady_clock> p_clock)
    : m_clock(p_clock)
  {
    async::context scratch;
    auto frequency_future = m_clock->frequency(scratch);
    auto const frequency_hz =
      frequency_future.value().numerical_value_in(hal::hertz::unit);
    m_nanoseconds_per_tick = 1'000'000'000.0f / frequency_hz;
  }

  [[nodiscard]] time_point now() const
  {
    async::context scratch;
    auto ticks_future = m_clock->uptime(scratch);
    auto const ticks = ticks_future.value();
    auto const nanoseconds = static_cast<std::int64_t>(
      static_cast<float>(ticks) * m_nanoseconds_per_tick);
    return time_point(duration(nanoseconds));
  }

private:
  hal::ptr<hal::steady_clock> m_clock;
  float m_nanoseconds_per_tick;
};

static_assert(async::clock<steady_clock_adapter>);

int main()
{
  initialize_platform();

  hal::ptr<resumer> waker(mem::unsafe_assume_static_tag{}, s_resumer);
  auto timer = resources::timer();
  auto clock = resources::clock();
  steady_clock_adapter clk(clock);

  auto app_future = application(application_context);
  auto task1_future = task1(task1_context);

  // TODO(#218): This will not work and doesn't do what we want. We want this.
  // to exit the sleep function if an interrupt has fired off as there might
  // be work to be done.
  auto sleep_function =
    [&timer, &waker](steady_clock_adapter::time_point p_future_time) {
      s_resumer.fired = false;

      timer->schedule(
        waker, p_future_time.m_since_epoch, hal::timer_mode::one_shot);

      while (!s_resumer.fired) {
        if (not hal::cortex_m::debugger_connected()) {
          continue;  // loop if the debugger is connected
        }

        // Disable interrupts to sleep safely without losing interrupts.
        hal::cortex_m::disable_all_interrupts();

        if (s_resumer.fired) {
          hal::cortex_m::enable_all_interrupts();
          break;
        }

        hal::cortex_m::wait_for_interrupt();
        hal::cortex_m::enable_all_interrupts();
      }
    };

  async::run_until_done(
    clk, sleep_function, application_context, task1_context);

  std::terminate();
}

// Override global new operator
void* operator new(std::size_t)
{
  std::terminate();
}

// Override global new[] operator
void* operator new[](std::size_t)
{
  std::terminate();
}

void* operator new(unsigned int, std::align_val_t)
{
  std::terminate();
}

// Override global delete operator
void operator delete(void*) noexcept
{
}

// Override global delete[] operator
void operator delete[](void*) noexcept
{
}

// Optional: Override sized delete operators (C++14 and later)
void operator delete(void*, std::size_t) noexcept
{
}

void operator delete[](void*, std::size_t) noexcept
{
}

void operator delete[](void*, std::align_val_t) noexcept
{
}

void operator delete(void*, std::align_val_t) noexcept
{
}

extern "C++"
{
  [[gnu::weak]] hal::task task1(async::context&)
  {
    return {};
  }
}
