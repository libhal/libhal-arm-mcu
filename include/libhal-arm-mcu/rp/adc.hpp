#pragma once

#include "rp.hpp"
#include <libhal/adc.hpp>

namespace hal::rp {

inline namespace v4 {
struct adc final : public hal::adc
{
  adc(pin_param auto pin)
    : adc(pin())
  {
    static_assert(internal::pin_max != 30 || (pin() >= 26 && pin() < 30),
                  "ADC pin is invalid!");
    static_assert(internal::pin_max != 48 || (pin() >= 40 && pin() < 48),
                  "ADC pin is invalid!");
  }
  adc(adc&&) = delete;
  ~adc() override;

private:
  adc(u8 pin);
  /*Because the rp chips only have one ADC that's muxed
  across different pins, we just initialize and mux the ADC
  every time we want to read. */
  float driver_read() override;
  u8 m_pin;
};
}  // namespace v4

namespace v5 {
// The ADC is 12 bit
struct adc16 final : public hal::adc16
{
  adc16(pin_param auto pin)
    : adc16(pin())
  {
    static_assert(internal::pin_max != 30 || (pin() >= 26 && pin() < 30),
                  "ADC pin is invalid!");
    static_assert(internal::pin_max != 48 || (pin() >= 40 && pin() < 48),
                  "ADC pin is invalid!");
  }
  ~adc16() override;

private:
  adc16(u8 gpio);
  /*Because the rp chips only have one ADC that's muxed
  across different pins, we just initialize and mux the ADC
  every time we want to read. */
  u16 driver_read() override;
  u8 m_pin;
};
}  // namespace v5
}  // namespace hal::rp
