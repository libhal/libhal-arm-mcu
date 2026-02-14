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

// returns configured clock speed for use with dwt_counter
hertz core_clock();

using microseconds = std::chrono::duration<u32, std::micro>;

}  // namespace hal::rp::inline v4
