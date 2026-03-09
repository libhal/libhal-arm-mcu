#include "libhal-arm-mcu/rp/pwm.hpp"

#include <hardware/gpio.h>
#include <hardware/pwm.h>
#include <libhal-util/bit.hpp>
#include <libhal/error.hpp>
#include <libhal/units.hpp>
#include <limits>

/*
TODO: Specify SYS_CLK_HZ in a board header so that pico actually knows what
frequency it should run at

*/

namespace hal::rp {
inline namespace v4 {
pwm_pin::pwm_pin(hal::unsafe, u8 pin)
  : m_pin(pin)
{
  gpio_set_function(m_pin, GPIO_FUNC_PWM);
  auto config = pwm_get_default_config();
  pwm_init(pwm_gpio_to_slice_num(m_pin), &config, false);
}

pwm_pin::~pwm_pin()
{
  gpio_deinit(m_pin);
}

void pwm_pin::driver_duty_cycle(float duty)
{
  uint channel = pwm_gpio_to_channel(m_pin);
  uint slice = pwm_gpio_to_slice_num(m_pin);
  if (duty == 0) {
    pwm_set_chan_level(slice, channel, 0);
    return;
  }

  float percentage = float(duty) / std::numeric_limits<u16>::max();
  u16 top = pwm_hw->slice[slice].top;
  pwm_set_chan_level(slice, channel, u16(float(top) * percentage));
  pwm_set_enabled(slice, true);
}

void pwm_pin::driver_frequency(float f)
{
  if (f > SYS_CLK_HZ) {
    hal::safe_throw(argument_out_of_domain(this));
  }

  auto frequency = static_cast<float>(f);
  auto clock = static_cast<float>(SYS_CLK_HZ);
  // We try to adjust clock divider to maximize counter resolution
  float wrap_val = std::numeric_limits<uint16_t>::max();
  float clock_div = clock / frequency / wrap_val;
  // cap the clock divider at 256
  if (clock_div > 256) {
    clock_div = 256;
    wrap_val = clock / frequency / 256;
  }
  // maintain a minimum clock divider of 1
  if (clock_div < 1.0) {
    clock_div = 1;
    wrap_val = clock / frequency;
  }

  // frequency too low
  if (wrap_val > std::numeric_limits<u16>::max()) {
    hal::safe_throw(argument_out_of_domain(this));
  }

  uint slice = pwm_gpio_to_slice_num(m_pin);
  pwm_set_wrap(slice, static_cast<u16>(wrap_val));
  pwm_set_clkdiv(slice, clock_div);
}
}  // namespace v4

namespace v5 {

pwm_slice_runtime::pwm_slice_runtime(u8 num, configuration const& cfg)
  : m_number(num)
  , m_phase_correct(cfg.phase_correct)
{
  if (num >= NUM_PWM_SLICES) {
    hal::safe_throw(argument_out_of_domain(this));
  }
  auto config = pwm_get_default_config();
  pwm_init(num, &config, false);
}

pwm_slice_runtime::~pwm_slice_runtime()
{
  pwm_set_enabled(m_number, false);
}

void pwm_slice_runtime::driver_frequency(u32 f)
{
  // frequency too high
  if (f > SYS_CLK_HZ) {
    hal::safe_throw(argument_out_of_domain(this));
  }

  auto frequency = static_cast<float>(f);
  // phase correct mode halfs output frequency, so we need to double it to
  // compensate
  if (m_phase_correct)
    frequency *= 2;
  auto clock = static_cast<float>(SYS_CLK_HZ);
  // We try to adjust clock divider to maximize counter resolution
  float wrap_val = std::numeric_limits<uint16_t>::max();
  float clock_div = clock / frequency / wrap_val;
  // cap the clock divider at 256
  if (clock_div > 256) {
    clock_div = 256;
    wrap_val = clock / frequency / 256;
  }
  // maintain a minimum clock divider of 1
  if (clock_div < 1.0) {
    clock_div = 1;
    wrap_val = clock / frequency;
  }

  // frequency too low
  if (wrap_val > std::numeric_limits<u16>::max()) {
    hal::safe_throw(argument_out_of_domain(this));
  }

  pwm_set_wrap(m_number, static_cast<u16>(wrap_val));
  pwm_set_clkdiv(m_number, clock_div);
}

pwm_pin pwm_slice_runtime::get_pin_raw(u8 pin, pwm_pin_configuration const& c)
{
  return pwm_pin(pin, c, {});
}

pwm_pin::pwm_pin(u8 pin, pwm_pin_configuration const& c, hal::unsafe)
  : m_pin(pin)
  , m_slice(pwm_gpio_to_slice_num(pin))
  , m_autostart(c.autostart)
{
  gpio_set_function(m_pin, GPIO_FUNC_PWM);
  driver_duty_cycle(c.duty_cycle);
}

pwm_pin::~pwm_pin()
{
  gpio_deinit(m_pin);
}

void pwm_pin::driver_duty_cycle(u16 duty)
{
  auto channel = pwm_gpio_to_channel(m_pin);
  if (duty == 0) {
    pwm_set_chan_level(m_slice, channel, 0);
    return;
  }

  float percentage = float(duty) / std::numeric_limits<u16>::max();
  u16 top = pwm_hw->slice[m_slice].top;
  pwm_set_chan_level(m_slice, channel, u16(float(top) * percentage));
  if (m_autostart) {
    pwm_set_enabled(m_slice, true);
  }
}

void enable_all_pwm(bool start)
{
  // Literally no idea if this is required, but let's be cautious and only
  // enable slices that exist
  uint32_t enable_mask;
  if constexpr (rp::internal::type == rp::internal::processor_type::rp2040) {
    enable_mask = 0b1111'1111;
  } else if constexpr (rp::internal::type ==
                       rp::internal::processor_type::rp2350) {
    enable_mask = 0b1111'1111'1111;
  }

  if (start) {
    pwm_set_mask_enabled(enable_mask);
  } else {
    pwm_set_mask_enabled(0x0);
  }
}

void pwm_slice_runtime::enable(bool enable)
{
  pwm_set_enabled(m_number, enable);
}

u32 pwm_pin::driver_frequency()
{
  uint8_t num =
    hal::bit_extract<{ .position = PWM_CH0_DIV_INT_LSB, .width = 8 }>(
      pwm_hw->slice[m_slice].div);
  uint8_t den =
    hal::bit_extract<{ .position = PWM_CH0_DIV_FRAC_LSB, .width = 4 }>(
      pwm_hw->slice[m_slice].div);

  float divider = float(num) / float(den);
  u16 top = pwm_hw->slice[m_slice].top;

  return static_cast<u32>(static_cast<float>(top) * divider);
}

}  // namespace v5
}  // namespace hal::rp
