#pragma once

#include <cstdint>
#include <libhal/output_pin.hpp>


namespace hal::rp::generic::inline v1 {
struct output_pin final : public hal::output_pin
{
  output_pin(u8, settings const&);
  
private:
  void driver_configure(settings const&) override;
  bool driver_level() override;
  void driver_level(bool) override;

  u8 m_pin{};
};

// temporary remove later
void sleep_ms(uint32_t);
} // namespace hal::rp::generic::inline v1


