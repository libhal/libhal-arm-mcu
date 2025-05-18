#pragma once

#include <libhal/adc.hpp>

namespace hal::rp::inline v1 {

// The ADC is 12 bit
struct adc final : public hal::adc16
{
  adc(u8 gpio);

private:
  /*Because the rp chips only have one ADC that's muxed
  across different pins, we just initialize and mux the ADC
  every time we want to read. */
  u16 driver_read() override;
  u8 m_pin;
};
}  // namespace hal::rp::inline v1
