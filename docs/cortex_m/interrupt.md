# Interrupt Controller

## NVIC (Nested Vector Interrupt Controller)

Defined in namespace `hal::cortex_m`

*#include <libhal-arm-mcu/interrupt.hpp>*

```{doxygenenum} hal::cortex_m::irq
```

```{doxygenfunction} hal::cortex_m::default_interrupt_handler
```

```{doxygenfunction} hal::cortex_m::hard_fault_handler
```

```{doxygenfunction} hal::cortex_m::memory_management_fault_handler
```

```{doxygenfunction} hal::cortex_m::bus_fault_handler
```

```{doxygenfunction} hal::cortex_m::usage_fault_handler
```

```{doxygenfunction} hal::cortex_m::revert_interrupt_vector_table
```

```{doxygenfunction} hal::cortex_m::initialize_interrupts(std::span<interrupt_pointer>)
```

```{doxygenfunction} hal::cortex_m::interrupt_vector_table_initialized
```

```{doxygenfunction} hal::cortex_m::disable_all_interrupts
```

```{doxygenfunction} hal::cortex_m::enable_all_interrupts
```

```{doxygenfunction} hal::cortex_m::get_vector_table
```

```{doxygenfunction} hal::cortex_m::enable_interrupt(irq_t, interrupt_pointer)
```

```{doxygenfunction} hal::cortex_m::disable_interrupt(irq_t)
```

```{doxygenfunction} hal::cortex_m::is_interrupt_enabled(irq_t)
```

```{doxygenfunction} hal::cortex_m::verify_vector_enabled(irq_t, interrupt_pointer)
```