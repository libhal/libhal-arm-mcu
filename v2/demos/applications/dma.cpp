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

// Times copies of a few sizes with memcpy and with the platform's copier,
// which uses DMA, and prints both in steady clock ticks.

module;

#include <array>
#include <chrono>
#include <cinttypes>
#include <cstdint>
#include <cstring>
#include <span>

module arm_mcu_demos;

import hal;
import hal.util;

namespace {
// Static, so DMA can reach them and they don't fill the stack.
// alignas(4) lets DMA move words instead of bytes.
alignas(4) std::array<hal::byte, 4096> source{};
alignas(4) std::array<hal::byte, 4096> destination{};

bool copied(std::size_t p_length)
{
  return std::memcmp(source.data(), destination.data(), p_length) == 0;
}
}  // namespace

void application()
{
  using namespace std::chrono_literals;

  auto console = resources::console();
  auto clock = resources::clock();
  auto copier = resources::copier();

  for (std::size_t i = 0; i < source.size(); i++) {
    source[i] = static_cast<hal::byte>(i * 7U);
  }

  auto const hertz = static_cast<std::uint32_t>(
    clock->frequency().numerical_value_in(hal::hertz::unit));
  hal::print<64>(
    *console, "DMA copy benchmark (clock: %" PRIu32 " Hz)\n", hertz);

  while (true) {
    for (std::size_t const length : { 16U, 256U, 4096U }) {
      destination.fill(0);
      auto start = clock->uptime();
      std::memcpy(destination.data(), source.data(), length);
      auto const memcpy_ticks = clock->uptime() - start;
      bool const memcpy_ok = copied(length);

      destination.fill(0);
      start = clock->uptime();
      copier->copy(std::span(source).first(length),
                   std::span(destination).first(length));
      auto const dma_ticks = clock->uptime() - start;
      bool const dma_ok = copied(length);

      hal::print<96>(*console,
                     "%4" PRIu32 " bytes: memcpy %6" PRIu32 " ticks%s, "
                     "dma %6" PRIu32 " ticks%s\n",
                     static_cast<std::uint32_t>(length),
                     static_cast<std::uint32_t>(memcpy_ticks),
                     memcpy_ok ? "" : " (MISMATCH)",
                     static_cast<std::uint32_t>(dma_ticks),
                     dma_ok ? "" : " (MISMATCH)");
    }
    hal::write(*console, "\n");
    hal::sleep_for(1s);
  }
}
