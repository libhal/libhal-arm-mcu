# Toggle measurement

## Release

## Direct

750.1666666674586 nanoseconds

0.39x or (2.5x faster than virtual)

ΔT	0.00009069019460650196	s
Nfalling	121
Nrising	121
fmin	1329787.2341833622	Hz
fmax	1336898.3950334257	Hz
fmean	1333037.1028646226	Hz
Tstd	2.001400074032293e-9	s
fbaud	2941176.4740801915	Hz


## Virtual API

1.887999999780732 microseconds

1x

ΔT	0.00008812399999991172	s
Nfalling	47
Nrising	47
fmin	528541.2261526836	Hz
fmax	529661.0170106662	Hz
fmean	528832.8888097224	Hz
Tstd	1.7758443943881256e-9	s
fbaud	1336898.3958461941	Hz


## Coro API

7.782285714288912 microseconds

4x

ΔT	0.00011275200000016028	s
Nfalling	15
Nrising	15
fmin	128402.67077734385	Hz
fmax	128534.70437164168	Hz
fmean	128496.95278649539	Hz
Tstd	2.584824645443775e-9	s
fbaud	263435.1949444235	Hz


## MinSizeRel

### Coroutine

### Virtual

### Direct

# First impressions of Coroutines on an STM32F103c8

CPU clock set to max @ 64MHz using the internal 8MHz oscillator.

This is a pin toggling test against my drivers. I wanted to see how long
the response time is between the following in gcc 13.2 (LTO enabled):

- direct function call (noinline)
- virtual function call
- virtual coroutine function call (sync return finished coroutine)
- virtual coroutine function call (co_return)

A requirement was to eliminate global heap usage so I pass a context object, which contains a `std::pmr::memory_resource`, to each of my coroutines. My coroutines use this memory resource to allocate my frame. The memory resource is very basic:

```C++
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
    std::pmr::memory_resource const& other) const noexcept override;

  std::span<hal::byte> m_memory;
  hal::usize m_stack_pointer = 0;
};
```

Its a simple stack allocator. Here is the gist to what I have so far.
https://gist.github.com/kammce/38ba005273300236232e5720db50951e

Just wanted to show some results I got from running a sequence of toggle tests:

```C++
while (true) {
  using namespace std::chrono_literals;
  for (int i = 0; i < 1000; i++) {
    set_pin('B', 13, false);
    set_pin('B', 13, true);
  }
  hal::delay(clock, 500us);
  for (int i = 0; i < 500; i++) {
    led.level(false);
    led.level(true);
  }
  hal::delay(clock, 500us);
  for (int i = 0; i < 400; i++) {
    coro_pin->level(ctx, false).sync_result();
    coro_pin->level(ctx, true).sync_result();
  }
  hal::delay(clock, 500us);
  for (int i = 0; i < 200; i++) {
    coro_pin_async->level(ctx, false).sync_result();
    coro_pin_async->level(ctx, true).sync_result();
  }
  hal::delay(clock, 500us);
}
```

The average period for each is the following:

- MinSizeRel (microseconds) `-Os`
  - 1us for direct
  - 2.5us for virtual
  - 4.5us for coro sync
  - 14.5us for coro async
- Release (nanoseconds) `-O3`
  - 640ns for direct
  - 1954 for virtual
  - 375 for coro sync
  - 8595 for coro async

Note that the coro objects are within the same translation unit so thats maybe
why they got optimized further than the direct call. Especiallys ince the
direct call had a `[[gnu::noinline]]`.

I just thought I show some numbers. I wonder if anyone else has tried this out
and has any numbers on their end.