#include <libhal-arm-mcu/interrupt.hpp>
#include <libhal-arm-mcu/stm32_generic/timer.hpp>
#include <libhal-util/bit.hpp>
#include <libhal/error.hpp>
#include <limits>

#include "timer.hpp"

namespace hal::stm32_generic {

namespace {

void setup(timer_reg_t* p_reg)
{
  static constexpr auto auto_reload_preload_enable = hal::bit_mask::from(7);
  static constexpr auto one_pulse_mode = hal::bit_mask::from(3);
  static constexpr auto update_request_source = hal::bit_mask::from(2);
  static constexpr auto update_interrupt_enable = hal::bit_mask::from(0);

  bit_modify(p_reg->control_register)
    .set(auto_reload_preload_enable)
    .set(one_pulse_mode)
    .set(update_request_source);

  bit_modify(p_reg->interrupt_enable_register).set(update_interrupt_enable);
}

}  // namespace

timer::timer(hal::unsafe)
{
}

bool timer::is_running()
{
  constexpr auto counter_enable = hal::bit_mask::from(0);
  auto reg = get_timer_reg(m_reg);

  return (bit_extract<counter_enable>(reg->control_register));
}

void timer::cancel()
{
  constexpr auto counter_enable = hal::bit_mask::from(0);
  auto reg = get_timer_reg(m_reg);

  bit_modify(reg->control_register).clear(counter_enable);
}

void timer::schedule(hal::time_duration p_delay, u32 p_timer_clock_frequency)
{
  timer::cancel();

  constexpr auto counter_enable = hal::bit_mask::from(0);
  constexpr auto update_generation = hal::bit_mask::from(0);
  constexpr auto prescaler = hal::bit_mask::from(0, 15);
  constexpr auto auto_reload_value = hal::bit_mask::from(0, 15);
  constexpr auto counter = hal::bit_mask::from(0, 15);

  auto reg = get_timer_reg(m_reg);

  constexpr u32 scale_factor = decltype(p_delay)::period::den;
  constexpr u16 prescaler_max_value = std::numeric_limits<hal::u16>::max();
  constexpr u16 timer_max_ticks = std::numeric_limits<hal::u16>::max();
  constexpr u16 timer_reset_value = 0;

  u32 const time_per_tick_ns = scale_factor / p_timer_clock_frequency;
  // Check if CPU Frequency too fast
  if (time_per_tick_ns == 0) {
    safe_throw(hal::argument_out_of_domain(this));
  }
  // Use 64-bit to prevent immediate overflow. If still too big after division,
  // exception thrown.
  u64 const ticks_required = p_delay.count() / time_per_tick_ns;
  u64 const prescaler_value = ticks_required / timer_max_ticks;
  if (prescaler_value > prescaler_max_value) {
    safe_throw(hal::argument_out_of_domain(this));
  }

  u16 prescaler_ticks_required;
  // When the delay amount is shorter than one clock cycle
  if (ticks_required == 0) {
    prescaler_ticks_required = 1;
  } else {
    u32 const prescaler_frequency =
      p_timer_clock_frequency / (static_cast<u16>(prescaler_value) + 1);
    u32 const prescaler_time_per_tick_ns = scale_factor / prescaler_frequency;
    prescaler_ticks_required = p_delay.count() / prescaler_time_per_tick_ns;
  }

  bit_modify(reg->prescale_register)
    .insert<prescaler>(static_cast<u16>(prescaler_value));
  bit_modify(reg->auto_reload_register)
    .insert<auto_reload_value>(prescaler_ticks_required);
  bit_modify(reg->counter_register).insert<counter>(timer_reset_value);
  bit_modify(reg->event_generator_register).set(update_generation);
  bit_modify(reg->control_register).set(counter_enable);
}

void timer::initialize(hal::unsafe,
                       void* p_peripheral_address,
                       void (*initialize_interrupts_function)(),
                       cortex_m::irq_t p_irq,
                       cortex_m::interrupt_pointer p_handler)
{
  m_reg = p_peripheral_address;
  initialize_interrupts_function();
  if (hal::cortex_m::is_interrupt_enabled(p_irq)) {
    hal::safe_throw(hal::device_or_resource_busy(this));
  }
  cortex_m::enable_interrupt(p_irq, p_handler);
  timer_reg_t* reg = get_timer_reg(m_reg);
  setup(reg);
}
}  // namespace hal::stm32_generic
