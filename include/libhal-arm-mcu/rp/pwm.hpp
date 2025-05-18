#pragma once

#include "rp.hpp"
#include <libhal/pwm.hpp>

namespace hal::rp::inline v1 {

enum struct pwm_ch : u8
{
  a,
  b
};

struct pwm_pin;

/*
Somewhat unusually, the pico contains 12 pwm
"groups" with 2 input pins each. These "groups"
are called slices, and are actually very independent
of one other. This slightly complicates synchronization,
but also allows for more independence between slices.
*/
struct pwm_slice final : hal::pwm_group_manager
{
  pwm_slice(u8 slice_num);

  struct configuration
  {
    u8 pin;
    u16 duty_cycle = 0;
    bool autostart = true;
  };
  pwm_pin get_pin(configuration);

private:
  /*
  At frequencies above ~2288 Hz, the wrap counter
  begins to lose resolution, reducing the duty cycle resolution
  down from its theoretical 16 bits of resolution.
  */
  void driver_frequency(u32 p_frequency) override;
  u8 m_number;
};

/*
The pico has a thing called phase correct mode where
its phase is centered. No idea if it's useful, may add
if somebody needs it.
*/
struct pwm_pin final : hal::pwm16_channel
{
  // A false disables the timer.
  void enable(bool = true);

  friend pwm_slice;

private:
  u32 driver_frequency() override;

  /*
  When set to a 0% duty cycle, it disables the timer
  completely.
  */
  void driver_duty_cycle(u16 p_duty_cycle) override;

  pwm_pin(u8 slice, pwm_slice::configuration);

  u8 m_pin;
  u8 m_slice;
  bool m_autostart;
};

/* This globally starts all of the timers at once, which may be useful for
aligning their phases. Set argument to false to stop all of them at once */
void enable_all_pwm(bool start = true);

struct pwm_pin_config_data
{
  u8 const pin;
  pwm_ch const channel;
  u8 const slice_num;

protected:
  constexpr pwm_pin_config_data(u8 p, pwm_ch c, u8 slice)
    : pin(p)
    , channel(c)
    , slice_num(slice)
  {
  }
};

constexpr pwm_ch get_channel(u8 pin_num)
{
  if (pin_num % 2 == 0) {
    return pwm_ch::a;
  } else {
    return pwm_ch::b;
  }
}

constexpr u8 get_slice_number(u8 pin_num)
{
  // this snippet of code was copied from Pico SDK
  if ((pin_num) < 32) {
    return ((pin_num) >> 1u) & 7u;
  } else {
    return 8u + (((pin_num) >> 1u) & 3u);
  }
}
/* Do not allocate two pwm's of the same slice. */
template<u8 pin_num>
struct pwm_pin_config : pwm_pin_config_data
{
  consteval pwm_pin_config()
    : pwm_pin_config_data(pin_num,
                          get_channel(pin_num),
                          get_slice_number(pin_num))
  {
  }
  static_assert(pin_num < internal::pin_max, "Pin number is invalid!");
};

}  // namespace hal::rp::inline v1
