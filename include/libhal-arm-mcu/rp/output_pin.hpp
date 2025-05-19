#pragma once

#include "libhal-arm-mcu/rp/rp.hpp"
#include <libhal/output_pin.hpp>

namespace hal::rp::inline v1 {
struct output_pin final : public hal::output_pin
{

  output_pin(pin_param auto pin, settings const& options)
    : output_pin(pin(), options)
  {
  }

private:
  output_pin(u8, settings const&);

  void driver_configure(settings const&) override;
  bool driver_level() override;
  void driver_level(bool) override;

  u8 m_pin{};
};
}  // namespace hal::rp::inline v1
