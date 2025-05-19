#pragma once

#include "rp.hpp"
#include <libhal/input_pin.hpp>

namespace hal::rp::inline v1 {
struct input_pin final : public hal::input_pin
{

  input_pin(pin_param auto pin, settings const&)
    : input_pin(pin())
  {
  }

private:
  input_pin(u8, settings const&);
  void driver_configure(settings const&) override;
  bool driver_level() override;

  u8 m_pin{};
};
}  // namespace hal::rp::inline v1
