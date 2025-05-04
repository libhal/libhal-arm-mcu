#pragma once

#include <libhal/output_pin.hpp>

namespace hal::rp::generic {
// Clang-tidy is wrong about the namespace nesting thing because the v1
// namespace is inline
inline int fake = 0;
inline namespace v1 {
struct output_pin final : public hal::output_pin
{
  output_pin(u8, settings const&);
  
private:
  void driver_configure(settings const&) override;
  bool driver_level() override;
  void driver_level(bool) override;

  u8 m_pin{};
};
}  // namespace v1

}  // namespace hal::rp::generic
