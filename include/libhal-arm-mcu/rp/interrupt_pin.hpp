#pragma once

#include "rp.hpp"
#include <libhal/interrupt_pin.hpp>
#include <libhal/units.hpp>

namespace hal::rp::inline v4 {
/*
Interrupt pin uses hidden globals to implement interrupts because
the interrupt callback function doesn't quite match the hal::handler type.
Because of that, it is unsafe to construct multiple interrupts across cores
or in an interrupt. Not that you'd do that anyways.
*/
struct interrupt_pin final : public hal::interrupt_pin
{
  interrupt_pin(pin_param auto pin,
                hal::callback<handler> callback,
                settings const& options = {})
    : interrupt_pin(pin(), callback, options)
  {
  }
  interrupt_pin(interrupt_pin&&) = delete;
  ~interrupt_pin() override;

private:
  interrupt_pin(u8 pin, hal::callback<handler>, settings const&);
  void driver_configure(settings const&) override;
  void driver_on_trigger(hal::callback<handler>) override;
  u8 m_pin;
};
}  // namespace hal::rp::inline v4
