# libhal v5

libhal v5 addresses major design concerns from v4. Interfaces stay synchronous, while concurrency becomes a property of the runtime: drivers block through a pluggable waiter seam that a platform, RTOS, or fiber runtime implements.

## Driver migration status

The drivers are being ported to libhal 5.0.0-alpha (synchronous, no
`async_context`, no `strong_ptr`) one at a time. Drivers that aren't ported
yet are left out of the build: their lines in `CMakeLists.txt`,
`modules/cortex_m/cortex_m.cppm`, `modules/stm32f1/stm32f1.cppm` and
`modules/arm_mcu.cppm` are commented out, along with the demos that use them.

| Ported | Not yet ported |
| --- | --- |
| Cortex-M: interrupts, startup, system control, DWT counter (steady clock), SysTick timer, WFI waiter | `stm32_generic`: pwm, timer |
| `stm32_generic`: uart, spi, i2c | STM32F1: can, usb |
| STM32F1: constants, power, clock, pin, gpio, dma (register map), usart, independent_watchdog, adc, spi, i2c | Demos: can, usb_cdc_raw, usb |
| Demos: adc, blank, blinker, gpio, i2c, spi, uart | |

## Changes from v4

### Foundational

- **Modules First**: Migrate from headers to C++20 modules for all code and libraries
- **Waiter Seam**: All libhal interfaces are synchronous. Peripheral drivers that wait on interrupts or DMA block through a `hal::notifier`, which wakes the calling context's `hal::blocking_waiter`. Platforms install a default waiter provider (e.g. WFI on Cortex-M, std-based on hosts) and runtimes such as an RTOS or fibers replace it, so blocking calls idle the CPU or yield to other contexts without the interfaces changing. Composite drivers only call `hal::sleep_for` and the blocking APIs of the interfaces they hold.
- **Callbacks for Events**: Interfaces expose non-blocking access to state and data plus a callback for "state changed" (e.g. `hal::edge_triggered_interrupt`, `hal::serial_interrupt`, `hal::can_interrupt`). Blocking waits and polling loops are built on top of those callbacks outside of the interfaces.
- **No Timeout Parameters**: Transfer operations are bounded by the hardware; their fault timeouts are a property of each implementation, set at construction, rather than a parameter of the interface.
- **Labelled ABI**: Namespace becomes `hal::inline v5` to label ABIs and support future backwards compatibility
- **Strongly Typed Units**: Migrate to mp-units library. Single precision float for most units, unsigned 32-bit integer for frequency
- **Factor Out Dependencies**: Extract generic libraries into standalone components:
  - scatter_span
- **Tagged Callbacks**: All interface callbacks include unique first parameter tag type (e.g., `struct my_tag{};`) for exception disambiguation
- **Scatter Span Usage**: All span-accepting APIs must accept scatter_span

### Policies

- Drivers are returned and stored **by value**. Simple drivers are constructed
  directly (`hal::stm32f1::output_pin led({ .port = 'C', .pin = 13 });`).
  Managers are constructed directly too, and hand out resources by value from
  `acquire_*()`, which is `[[clang::lifetimebound]]` so a resource can't
  outlive its manager. No `create()` factories, no `hal::pimpl`.
- The application decides where a driver lives: on the stack, as a static, or
  on the heap with `hal::make_owner<T>(allocator, ...)`.

### Interfaces

- Replace `hal::io_waiter` with the `hal::waiter`/`hal::notifier` seam
- `hal::zero_copy_serial` becomes default serial implementation

### Open Questions

1. Where should container libraries like `circular_buffer` and
   `allocated_buffer` go?

## Changes to v4 After Split

- Move v5 motor & servo interfaces into v4
- Move USB interfaces to v4
- Potential ABI break: Change v4 symbols to `hal::inline v4`
- Deprecate the following interfaces:
  - `hal::can`: too much responsibility
  - `hal::pwm`: too much responsibility
  - `hal::interrupt_pin`: Replaced by `hal::edge_triggered_interrupt`
  - (more to be determined)
- Move the following v5 interfaces into v4:
  - `hal::spi_channel`
  - `adc16`, `adc24`, `adc32`
  - `dac8`, `dac16`
  - `pwm_group_frequency`
  - `pwm_duty_cycle16`
  - Extended CAN interfaces (not `hal::can`)
  - USB interfaces
- `hal::v5::strong_ptr` will be brought into the `hal` namespace, eliminating the need to type `v5::` everywhere.
-
