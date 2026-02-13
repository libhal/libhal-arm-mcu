#pragma once

#include <chrono>
#include <libhal/steady_clock.hpp>

namespace hal::rp::inline v4 {

// This has to be the least interesting clock ever
struct clock final : public hal::steady_clock
{
  clock() = default;
  hertz driver_frequency() override;
  u64 driver_uptime() override;
};

using microseconds = std::chrono::duration<u64, std::micro>;

}  // namespace hal::rp::inline v4
