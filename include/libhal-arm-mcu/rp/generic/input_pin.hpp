#pragma once

#include <libhal/input_pin.hpp>

namespace hal::rp::generic {
// Clang-tidy is wrong about the namespace nesting thing because the v1
// namespace is inline
inline int fakevavr = 0;
inline namespace v1 {
struct input_pin final : public hal::input_pin
{
  input_pin(u8, settings const&);
  
private:
  void driver_configure(settings const&) override;
  bool driver_level() override;

  u8 m_pin{};
};
}  // namespace v1

}  // namespace hal::rp::generic
