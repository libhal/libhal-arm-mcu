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

#include <bitset>
#include <cstddef>

export module hal.arm_mcu.stm32f1:claim;

import hal;

namespace hal::stm32f1 {
/**
 * @brief Throw if a resource that drivers claim is already claimed
 *
 * Use this on its own when a driver claims more than one resource: check each
 * one first, so the driver throws before it claims anything and never has to
 * release a claim it already made.
 *
 * @param p_claimed - one bit per resource, 1 while a driver has claimed it
 * @param p_index - the resource's bit. An index past the end isn't claimed.
 * @param p_instance - the object to report in the exception
 * @throws hal::device_or_resource_busy - if the resource is claimed
 */
template<std::size_t N>
void throw_if_claimed(std::bitset<N> const& p_claimed,
                      std::size_t p_index,
                      void const* p_instance)
{
  if ((p_index < N) and p_claimed[p_index]) {
    throw hal::device_or_resource_busy(p_instance);
  }
}

/**
 * @brief Claim a resource for one driver, or throw if it's already claimed
 *
 * @pre `p_index` is less than `N`.
 *
 * @param p_claimed - one bit per resource, 1 while a driver has claimed it
 * @param p_index - the resource's bit
 * @param p_instance - the object to report in the exception
 * @throws hal::device_or_resource_busy - if the resource is claimed
 */
template<std::size_t N>
void claim_or_throw(std::bitset<N>& p_claimed,
                    std::size_t p_index,
                    void const* p_instance)
{
  throw_if_claimed(p_claimed, p_index, p_instance);
  p_claimed[p_index] = true;
}
}  // namespace hal::stm32f1
