#pragma once

#include <libhal/steady_clock.hpp>

namespace hal::rp::inline v4 {

// This has to be the least interesting clock ever
struct clock final : public hal::steady_clock
{
  clock() = default;
  clock(clock const&) = default;
  hertz driver_frequency() override;
  u64 driver_uptime() override;
};

}  // namespace hal::rp::inline v4
