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

export module arm_mcu_demos;

import hal;
import hal.arm_mcu;

export namespace resources {
// Each function returns a driver that the platform file keeps in static
// storage. static_ref proves that at compile time: a non-static object won't
// convert to it.
hal::static_ref<hal::steady_clock> clock();
hal::static_ref<hal::output_pin> status_led();
hal::static_ref<hal::input_pin> input_pin();

// Left out until their drivers are ported (see ../CMakeLists.txt):
// console(), adc(), can_transceiver(), can_bus_manager(),
// can_identifier_filter() and the usb_*() endpoints.

inline void reset()
{
  hal::cortex_m::reset();
}
}  // namespace resources

// Application function is implemented by one of the .cpp files.
export void initialize_platform();
export void application();
