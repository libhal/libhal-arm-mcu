#pragma once
#include <libhal/initializers.hpp>
#include <libhal/rotation_sensor.hpp>

namespace hal::stm32_generic {
struct encoder_channels
{
  u8 channel_a;
  u8 channel_b;
};
class quadrature_encoder : public hal::rotation_sensor
{

public:
  quadrature_encoder(hal::unsafe, encoder_channels channels, void* p_reg);
  quadrature_encoder(hal::unsafe);

  void initialize(hal::unsafe, encoder_channels channels, void* p_reg);

private:
  // should be able to take any timer pin, but the timer should be the same pin.
  // when this encoder is acquired by gptimer or advanced timer, it will ensure
  // that the channels are part of the same timer.
  read_t driver_read() override;
  void* m_reg = nullptr;
};

}  // namespace hal::stm32_generic
