// Copyright 2024 Khalil Estell
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

#include <exception>

#include <libhal-arm-mcu/dwt_counter.hpp>

// Demonstrate function that throws
void foo()
{
  // volatile integer used to keep
  static int volatile a = 0;
  a = a + 1;
  throw 5;
}

bool volatile run = false;

int main()
{
  std::uint64_t uptime = 0;

  if (run) {
    try {
      hal::cortex_m::dwt_counter counter(1'000'000.0f);
      uptime = counter.uptime();
    } catch (...) {
      std::terminate();
    }

    try {
      // Test that exceptions work for the embedded target
      foo();
    } catch (...) {
      uptime += 1;
    }
  }

  return static_cast<int>(uptime);
}
