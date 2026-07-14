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

#pragma once
#include <libhal/units.hpp>

namespace hal::stm32f1 {

class independent_watchdog
{
public:
  /**
   * @brief start the watchdog countdown
   */
  void start();
  /**
   * @brief resets the watchdog countdown
   */
  void reset();
  /**
   * @brief configures the watchdog countdown frequency and counter to specified
   * time
   *
   * @param p_wait_time coundown time till the watchdog resets
   *
   * the actual wait time may be 66%-133% of the specified time due to clock
   * variance (sec. 7.2.5, pg. 96)
   */
  void set_countdown_time(hal::time_duration p_wait_time);

  /**
   * @brief checks if watchdog reset flag is set
   */
  bool check_flag();
  /**
   * @brief clears reset flags
   */
  void clear_flag();
};

}  // namespace hal::stm32f1
