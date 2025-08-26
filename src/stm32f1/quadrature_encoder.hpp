#pragma once

#include <libhal-arm-mcu/stm32f1/timer.hpp>
#include <libhal/pointers.hpp>
#include <libhal/rotation_sensor.hpp>

namespace hal::stm32f1 {
/**
 * @brief This class implements the `hal::rotation_sensor` interface
 *
 * It gets an input from a quadrature encoder motor and measures the amount turn
 * using 2 channels as input.
 *
 * Each Quadrature Encoder must use channel 1 and 2 from the same timer.
 */
class quadrature_encoder : public hal::rotation_sensor
{
protected:
  struct Key;

public:
  friend class hal::stm32f1::advanced_timer_manager;
  friend class hal::stm32f1::general_purpose_timer_manager;
  // this makes the constructor public, but due to the private Key member, only
  // the timer_manager classes are allowed in call make_strong_ptr
  quadrature_encoder([[maybe_unused]] Key key,
                     hal::stm32f1::timer_pins p_pin1,
                     hal::stm32f1::timer_pins p_pin2,
                     hal::stm32f1::peripheral p_select,
                     void* p_reg,
                     timer_manager_data* p_manager_data_ptr,
                     float p_pulses_per_rotation);
  quadrature_encoder(quadrature_encoder const& p_other) = delete;
  quadrature_encoder& operator=(quadrature_encoder const& p_other) = delete;
  quadrature_encoder(quadrature_encoder&& p_other) noexcept;
  quadrature_encoder& operator=(quadrature_encoder&& p_other) noexcept;
  ~quadrature_encoder() override;

protected:
  struct Key
  {
    explicit Key(int)
    {
    }
  };

private:
  read_t driver_read() override;
  hal::stm32_generic::quadrature_encoder m_encoder;
  timer_manager_data* m_manager_data_ptr;
};
}  // namespace hal::stm32f1
