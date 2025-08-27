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
#include <libhal/initializers.hpp>
#include <libhal/rotation_sensor.hpp>

namespace hal::stm32_generic {

/**
 * @brief These are the 2 channels from the same timer that a user must use.
 * They must be channel 1 and channel 2 of any timer. If channel 1 and channel 2
 * are flipped, the order of these parameters are important otherwise the
 * counter will work in the opposite direction.
 */
struct encoder_channels
{
  u8 channel_a;
  u8 channel_b;
};
class quadrature_encoder : public hal::rotation_sensor
{

public:
  quadrature_encoder(hal::unsafe,
                     encoder_channels channels,
                     void* p_reg,
                     u32 p_pulses_per_rotation);
  quadrature_encoder(hal::unsafe);

  void initialize(hal::unsafe,
                  encoder_channels channels,
                  void* p_reg,
                  u32 p_pulses_per_rotation);

private:
  // should be able to take any timer pin, but the timer should be the same pin.
  // when this encoder is acquired by gptimer or advanced timer, it will ensure
  // that the channels are part of the same timer.
  read_t driver_read() override;
  void* m_reg = nullptr;
  float m_pulses_per_rotation;
};

}  // namespace hal::stm32_generic
