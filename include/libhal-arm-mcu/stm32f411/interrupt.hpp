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

#pragma once

namespace hal::stm32f411 {
/**
 * @brief Initialize interrupts for the stm32f411 series processors
 *
 * Only initializes after the first call. Does nothing afterwards. Can be
 * called multiple times without issue.
 *
 */
void initialize_interrupts();
}  // namespace hal::stm32f411
