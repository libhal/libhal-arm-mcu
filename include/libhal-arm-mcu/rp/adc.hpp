#pragma once

#include "libhal-arm-mcu/rp/rp.hpp"
#include <libhal/adc.hpp>

namespace hal::rp::inline v1 {

// The ADC is 12 bit
struct adc final : public hal::adc16
{
  adc(pin_param auto pin)
    : adc(pin())
  {
    static_assert(internal::pin_max != 30 || (pin() >= 26 && pin() < 30),
                  "ADC pin is invalid!");
    static_assert(internal::pin_max != 48 || (pin() >= 40 && pin() < 48),
                  "ADC pin is invalid!");
  }
  ~adc() override;

private:
  adc(u8 gpio);
  /*Because the rp chips only have one ADC that's muxed
  across different pins, we just initialize and mux the ADC
  every time we want to read. */
  u16 driver_read() override;
  u8 m_pin;
};
}  // namespace hal::rp::inline v1
