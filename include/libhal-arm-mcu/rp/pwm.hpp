#pragma once

#include "rp.hpp"
#include <libhal/error.hpp>
#include <libhal/initializers.hpp>
#include <libhal/pwm.hpp>
#include <libhal/units.hpp>

namespace hal::rp::inline v1 {

enum struct pwm_ch : u8
{
  a,
  b
};

struct pwm_pin;

// See https://gcc.gnu.org/bugzilla/show_bug.cgi?id=88165

struct pwm_pin_configuration
{
  u16 duty_cycle = 0;
  bool autostart = true;
};

/*
Somewhat unusually, the pico contains 12 pwm
"groups" with 2 input pins each. These "groups"
are called slices, and are actually very independent
of one other. This slightly complicates synchronization,
but also allows for more independence between slices.
*/
struct pwm_slice_runtime : hal::pwm_group_manager
{
  static constexpr u8 max_slices()
  {
    using enum hal::rp::internal::processor_type;
    if (hal::rp::internal::type == rp2040) {
      return 8;
    } else if (hal::rp::internal::type == rp2350) {
      return 12;
    }
    return 0;
  }

  static constexpr u8 get_slice_number(pin_param auto pin)
  {
    if (pin() < 32) {
      return (pin() / 2) % 8;
    } else {
      return pin() / 2 - 8;
    }
  }

  pwm_slice_runtime(channel_param auto slice)
    : pwm_slice_runtime(slice())
  {
    static_assert(slice() < max_slices(), "Invalid PWM slice!");
  }

  using configuration = pwm_pin_configuration;

  // copied from pico sdk
  static constexpr u8 pin_from_slice(u8 gpio)
  {
    if ((gpio) < 32) {
      return ((gpio) >> 1u) & 7u;
    } else {
      return 8u + (((gpio) >> 1u) & 3u);
    }
  }

protected:
  pwm_pin get_pin_raw(u8 pin, configuration const&);
  pwm_slice_runtime(u8 slice_num);
  /*
  At frequencies above ~2288 Hz, the wrap counter
  begins to lose resolution, reducing the duty cycle resolution
  down from its theoretical 16 bits of resolution.
  */
  void driver_frequency(u32 p_frequency) final;
  u8 m_number;
};

/*
The pico has a thing called phase correct mode where
its phase is centered. No idea if it's useful, may add
if somebody needs it.
*/
struct pwm_pin final : hal::pwm16_channel
{

  ~pwm_pin() override;
  // A false disables the timer.
  void enable(bool = true);

  friend pwm_slice_runtime;

private:
  u32 driver_frequency() override;

  /*
  When set to a 0% duty cycle, it disables the timer
  completely.
  */
  void driver_duty_cycle(u16 p_duty_cycle) override;

  pwm_pin(u8 pin, pwm_slice_runtime::configuration const&);

  u8 m_pin;
  u8 m_slice;
  bool m_autostart;
};

/* This globally starts all of the timers at once, which may be useful for
aligning their phases. Set argument to false to stop all of them at once */
void enable_all_pwm(bool start = true);

template<u64 chan>
struct pwm_slice : pwm_slice_runtime
{

  pwm_slice(channel_param auto ch)
    : pwm_slice_runtime(ch())
  {
    using enum internal::processor_type;
    static_assert(internal::type == rp2040 || internal::type == rp2350,
                  "Update PWM channels for new RP!");
    static_assert(ch() < 8 || internal::pin_max != 30,
                  "PWM channel is invalid!");
    static_assert(ch() < 11 || internal::pin_max != 48,
                  "PWM channel is invalid!");
  }

  pwm_pin get_pin(pin_param auto pin,
                  pwm_slice_runtime::configuration const& config = {})
  {
    static_assert(get_slice_number(pin) == chan, "Slice pin is incorrect!");
    return get_pin_raw(pin(), config);
  }
};
template<channel_param p>
pwm_slice(p pin) -> pwm_slice<p::val>;

}  // namespace hal::rp::inline v1
