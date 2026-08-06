// Copyright 2024 - 2025 Khalil Estell and the libhal contributors
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

#include <chrono>
#include <coroutine>

module arm_mcu_demos;

import hal;

hal::task application(async::context& p_ctx)
{
  auto led = resources::status_led();

  while (true) {
    using namespace std::chrono_literals;
    co_await led->level(p_ctx, false);
    co_await 500ms;

    co_await led->level(p_ctx, true);
    co_await 2s;
  }
}
