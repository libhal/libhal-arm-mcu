// Copyright 2026 Khalil Estell and the libhal contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

module;

#include <cinttypes>
#include <chrono>
#include <coroutine>

module arm_mcu_demos;

import hal;
import hal.util;

hal::task application(async::context& p_ctx)
{
  auto clock = resources::clock();
  auto console = resources::console();
  auto adc = resources::adc();

  co_await hal::write(p_ctx, *console, "ADC Application Starting...\n");

  while (true) {
    using namespace std::chrono_literals;

    auto const sample = co_await adc->read(p_ctx);
    auto const percent =
      static_cast<float>(sample) / static_cast<float>(0xFFFF) * 100.0f;
    auto const uptime = co_await clock->uptime(p_ctx);

    co_await hal::print<128>(p_ctx,
                             *console,
                             "%" PRId32 "%%: %" PRIu64 " ticks\n",
                             static_cast<std::int32_t>(percent),
                             uptime);

    co_await 100ms;
  }
}
