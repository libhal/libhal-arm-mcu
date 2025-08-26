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

#include <atomic>
#include <memory_resource>
#include <optional>
#include <type_traits>

#include <libhal-arm-mcu/interrupt.hpp>
#include <libhal-arm-mcu/stm32_generic/pwm.hpp>
#include <libhal-arm-mcu/stm32_generic/quadrature_encoder.hpp>
#include <libhal-arm-mcu/stm32_generic/timer.hpp>
#include <libhal-arm-mcu/stm32f1/clock.hpp>
#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/enum.hpp>
#include <libhal/pointers.hpp>
#include <libhal/pwm.hpp>
#include <libhal/rotation_sensor.hpp>
#include <libhal/timer.hpp>
#include <libhal/units.hpp>

namespace hal::stm32f1 {
/**
 * @brief Default pins for the various timer peripherals.
 *
 * These pins can perform any timer operation.
 */
enum class timer_pins : u8
{
  pa0 = 0,
  pa1 = 1,
  pa2 = 2,
  pa3 = 3,
  pa6 = 4,
  pa7 = 5,
  pb0 = 6,
  pb1 = 7,
  pa8 = 8,
  pa9 = 9,
  pa10 = 10,
  pa11 = 11,
  pb6 = 12,
  pb7 = 13,
  pb8 = 14,
  pb9 = 15,
  pc6 = 16,
  pc7 = 17,
  pc8 = 18,
  pc9 = 19,
  pb14 = 20,
  pb15 = 21,
};

/**
 * @brief pins that are connected to the timer1 peripheral
 */
enum class timer1_pin : u8
{
  pa8 = hal::value(timer_pins::pa8),
  pa9 = hal::value(timer_pins::pa9),
  pa10 = hal::value(timer_pins::pa10),
  pa11 = hal::value(timer_pins::pa11),
};

/**
 * @brief pins that are connected to the timer 2 and 5 and 9 peripheral
 */
enum class timer2_pin : u8
{
  pa0 = hal::value(timer_pins::pa0),
  pa1 = hal::value(timer_pins::pa1),
  pa2 = hal::value(timer_pins::pa2),
  pa3 = hal::value(timer_pins::pa3),
};

/**
 * @brief pins that are connected to the timer3 peripheral
 */
enum class timer3_pin : u8
{
  pa6 = hal::value(timer_pins::pa6),
  pa7 = hal::value(timer_pins::pa7),
  pb0 = hal::value(timer_pins::pb0),
  pb1 = hal::value(timer_pins::pb1),
};

/**
 * @brief pins that are connected to the timer4 peripheral
 */
enum class timer4_pin : u8
{
  pb6 = hal::value(timer_pins::pb6),
  pb7 = hal::value(timer_pins::pb7),
  pb8 = hal::value(timer_pins::pb8),
  pb9 = hal::value(timer_pins::pb9),
};

/**
 * @brief pins that are connected to the timer 2, 5 and 9 peripheral
 */
enum class timer5_pin : u8
{
  pa0 = hal::value(timer_pins::pa0),
  pa1 = hal::value(timer_pins::pa1),
  pa2 = hal::value(timer_pins::pa2),
  pa3 = hal::value(timer_pins::pa3),
};

/**
 * @brief pins that are connected to the timer8 peripheral
 */
enum class timer8_pin : u8
{
  pc6 = hal::value(timer_pins::pc6),
  pc7 = hal::value(timer_pins::pc7),
  pc8 = hal::value(timer_pins::pc8),
  pc9 = hal::value(timer_pins::pc9),
};

/**
 * @brief pins that are connected to the timers 2, 5, 9 peripherals
 */
enum class timer9_pin : u8
{
  pa2 = hal::value(timer_pins::pa2),
  pa3 = hal::value(timer_pins::pa3),
};

/**
 * @brief pins that are connected to the timers 4 and 10 peripherals
 */
enum class timer10_pin : u8
{
  pb8 = hal::value(timer_pins::pb8),
};

/**
 * @brief pins that are connected to the timers 4 and 11 peripherals
 */
enum class timer11_pin : u8  // won't work
{
  pb9 = hal::value(timer_pins::pb9),
};

/**
 * @brief pins that are connected to the 12 peripheral
 */
enum class timer12_pin : u8
{
  pb14 = hal::value(timer_pins::pb14),
  pb15 = hal::value(timer_pins::pb15),
};

/**
 * @brief pins that are connected to the 13 and 3 peripheral
 */
enum class timer13_pin : u8
{
  pa6 = hal::value(timer_pins::pa6),
};

/**
 * @brief pins that are connected to the 14 and 3 peripheral.
 *
 */
enum class timer14_pin : u8
{
  pa7 = hal::value(timer_pins::pa7),
};

/**
 * @brief Gets the pwm timer type at compile time.
 */
template<peripheral id>
consteval auto get_pwm_timer_type()
{
  if constexpr (id == peripheral::timer1) {
    return std::type_identity<timer1_pin>();
  } else if constexpr (id == peripheral::timer2) {
    return std::type_identity<timer2_pin>();
  } else if constexpr (id == peripheral::timer3) {
    return std::type_identity<timer3_pin>();
  } else if constexpr (id == peripheral::timer4) {
    return std::type_identity<timer4_pin>();
  } else if constexpr (id == peripheral::timer5) {
    return std::type_identity<timer5_pin>();
  } else if constexpr (id == peripheral::timer8) {
    return std::type_identity<timer8_pin>();
  } else if constexpr (id == peripheral::timer9) {
    return std::type_identity<timer9_pin>();
  } else if constexpr (id == peripheral::timer10) {
    return std::type_identity<timer10_pin>();
  } else if constexpr (id == peripheral::timer11) {
    return std::type_identity<timer11_pin>();
  } else if constexpr (id == peripheral::timer12) {
    return std::type_identity<timer12_pin>();
  } else if constexpr (id == peripheral::timer13) {
    return std::type_identity<timer13_pin>();
  } else if constexpr (id == peripheral::timer14) {
    return std::type_identity<timer14_pin>();
  } else {
    return std::type_identity<void>();
  }
}

/**
 * @brief Used to store the state of the timer assigned to the manager.
 */
struct timer_manager_data
{
public:
  enum class usage : u8
  {
    uninitialized,
    old_pwm,
    pwm_generator,
    callback_timer,
    quadrature_encoder
  };
  /**
   * @brief Tracks how many resources are currently tied to the timer
   *
   * @return auto
   */
  auto resource_count() const
  {
    return m_resource_count.load();
  }

  /// Tracks how many resources are currently tied to the

  /**
   * @brief Indicates what kind of usage the timer is currently configured for.
   *
   * @return auto
   */
  auto current_usage() const
  {
    return m_usage;
  }

private:
  friend class timer;
  friend class pwm_group_frequency;
  friend class pwm16_channel;
  friend class pwm;
  friend class advanced_timer_manager;
  friend class general_purpose_timer_manager;
  friend class quadrature_encoder;

  /// Indicates which timer is assigned to this manager.
  peripheral m_id;
  /// Tracks how many resources are currently tied to the
  std::atomic<u8> m_resource_count{ 0 };
  /// Indicates what kind of usage the timer is currently configured for.
  usage m_usage = usage::uninitialized;

  timer_manager_data(peripheral p_id)
    : m_id(p_id)
  {
  }
};

class advanced_timer_manager;

class general_purpose_timer_manager;

/**
 * @brief This class implements the `hal::timer` interface.
 *
 * Can schedule function callbacks to occur via interrupt after a designated
 * amount of time.
 *
 */
class timer final : public hal::timer
{
public:
  friend class hal::stm32f1::advanced_timer_manager;
  friend class hal::stm32f1::general_purpose_timer_manager;

  timer(timer const& p_other) = delete;
  timer& operator=(timer const& p_other) = delete;
  timer(timer&& p_other) noexcept = delete;
  timer& operator=(timer&& p_other) noexcept = delete;

  ~timer() override;

private:
  /**
   * @brief Stores the IRQ number as well as handler object, which are needed
   * to enable the interrupt.
   *
   */
  struct interrupt_params
  {
    cortex_m::irq_t irq;
    cortex_m::interrupt_pointer handler;
  };

  /**
   * @brief This constructor is private because the only way one should be
   * able to access this timer is through one of the timer manager classes.
   *
   * @param p_reg - The address of the selected timer.
   * @param p_manager_data - Pointer to access the managers state information.
   */
  timer(void* p_reg, timer_manager_data* p_manager_data_ptr);

  bool driver_is_running() override;
  void driver_cancel() override;
  void driver_schedule(hal::callback<void(void)> p_callback,
                       hal::time_duration p_delay) override;

  /**
   * @brief Creates a static callable object and returns the necessary data to
   * enable the interrupt.
   *
   * @return interrupt_params - the data necessary to enable the interrupt in
   * the stm32_generic timer class.
   */
  interrupt_params setup_interrupt();
  /**
   * @brief Resets and returns the program from the interrupt.
   *
   */
  void handle_interrupt();
  /**
   * @brief The function that is used as the handler for the ISR.
   *
   */
  void interrupt();

  /// The stm32_generic timer object which is used to actually control the
  /// timer.
  hal::stm32_generic::timer m_timer;
  /// Pointer to access the managers state information.
  timer_manager_data* m_manager_data_ptr;
  /// The function to be used inside the ISR handler for the callback.
  std::optional<hal::callback<void(void)>> m_callback;
};

/**
 * @brief This class sets the frequency for a group of pwm pins
 *
 * Since all the pwm channels for each timer peripheral have to share the same
 * frequency, this class is used to set that frequency for the whole group of
 * channels.
 */
class pwm_group_frequency : public hal::pwm_group_manager
{
public:
  friend class hal::stm32f1::advanced_timer_manager;
  friend class hal::stm32f1::general_purpose_timer_manager;

  pwm_group_frequency(pwm_group_frequency const& p_other) = delete;
  pwm_group_frequency& operator=(pwm_group_frequency const& p_other) = delete;
  pwm_group_frequency(pwm_group_frequency&& p_other) noexcept;
  pwm_group_frequency& operator=(pwm_group_frequency&& p_other) noexcept;
  ~pwm_group_frequency() override;

private:
  /**
   * @brief The constructor is private because the only way one should be
   * able to access pwm_group_frequency is through one of the timer manager
   * classes.
   *
   * @param p_reg is a void pointer that points to the beginning of a timer
   * peripheral
   * @param p_manager_data_ptr is a pointer to access the managers state
   * information.
   */
  pwm_group_frequency(void* p_reg, timer_manager_data* p_manager_data_ptr);

  void driver_frequency(u32 p_hertz) override;

  /// The stm32_generic pwm_group_frequency object which is used to actually
  /// control the timer.
  hal::stm32_generic::pwm_group_frequency m_pwm_frequency;
  /// Pointer to access the managers state information.
  timer_manager_data* m_manager_data_ptr;
};

/**
 * @brief This class is used to control a pwm channel
 *
 * This is to be used in conjunction with pwm_group_frequency. You create your
 * pwm channel(s) with this class, where each channel can set its own unique
 * duty cycle. Then you set the frequency to be used by all the configured
 * channels for the specific timer peripheral through the pwm_group_frequency.
 */
class pwm16_channel : public hal::pwm16_channel
{
public:
  friend class hal::stm32f1::advanced_timer_manager;
  friend class hal::stm32f1::general_purpose_timer_manager;

  pwm16_channel(pwm16_channel const& p_other) = delete;
  pwm16_channel& operator=(pwm16_channel const& p_other) = delete;
  pwm16_channel(pwm16_channel&& p_other) noexcept;
  pwm16_channel& operator=(pwm16_channel&& p_other) noexcept;
  ~pwm16_channel() override;

private:
  /**
   * @brief The constructor is private because the only way one should be
   * able to access pwm16_channel is through one of the timer manager classes.
   *
   * @param p_reg is a void pointer that points to the beginning of a timer
   * peripheral
   * @param p_manager_data_ptr is a pointer to access the managers state
   * information.
   * @param p_is_advanced whether the current peripheral is advanced or not
   * @param p_pin is the pin to be used for the pwm channel.
   */
  pwm16_channel(void* p_reg,
                timer_manager_data* p_manager_data_ptr,
                bool p_is_advanced,
                timer_pins p_pin);

  u32 driver_frequency() override;
  void driver_duty_cycle(u16 p_duty_cycle) override;

  /// The stm32_generic pwm object which is used to actually control the timer.
  hal::stm32_generic::pwm m_pwm;
  /// The pin being used for the channel.
  timer_pins m_pin;
  /// Pointer to access the managers state information.
  timer_manager_data* m_manager_data_ptr;
};

/**
 * @brief This class implements the `hal::pwm` interface
 *
 * Can create pwm objects which can output pwm signals on a chosen pin
 *
 * @deprecated Please use pwm16_channel and/or pwm_group_frequency. This class
 * implements the `hal::pwm` interface which will be deprecated in libhal 5.
 */
class pwm : public hal::pwm
{
public:
  friend class hal::stm32f1::advanced_timer_manager;
  friend class hal::stm32f1::general_purpose_timer_manager;

  pwm(pwm const& p_other) = delete;
  pwm& operator=(pwm const& p_other) = delete;
  pwm(pwm&& p_other) noexcept;
  pwm& operator=(pwm&& p_other) noexcept;
  ~pwm() override;

private:
  /**
   * @brief The constructor is private because the only way one should be
   * able to access pwm is through one of the timer manager classes.
   *
   * @param p_reg is a void pointer that points to the beginning of a timer
   * peripheral
   * @param p_manager_data_ptr is a pointer to access the managers state
   * information.
   * @param p_is_advanced whether the current peripheral is advanced or not
   * @param p_pin is the pin to be used for the pwm channel.
   */
  pwm(void* p_reg,
      timer_manager_data* p_manager_data_ptr,
      bool p_is_advanced,
      timer_pins p_pin);

  void driver_frequency(hertz p_frequency) override;
  void driver_duty_cycle(float p_duty_cycle) override;

  /// The stm32_generic pwm object which is used to actually control the
  /// channel.
  hal::stm32_generic::pwm m_pwm;
  /// The stm32_generic group frequency object which is used to actually control
  /// the frequency for the group of channels.
  hal::stm32_generic::pwm_group_frequency m_pwm_frequency;
  /// The pin being used for the channel.
  timer_pins m_pin;
  /// Pointer to access the managers state information.
  timer_manager_data* m_manager_data_ptr;
};

/**
 * @brief This class is used to do any timer operations for timers 1 and 8.
 *
 * This is the non-templated version that implements all the core functionality.
 * It is designed to be inherited by the templated version, helping to reduce
 * code bloat and duplication caused by template instantiations.
 *
 * These timers can be used to do PWM generation, as well as other advanced
 * timer specific tasks
 */
class advanced_timer_manager
{
public:
  advanced_timer_manager(advanced_timer_manager const& p_other) = delete;
  advanced_timer_manager& operator=(advanced_timer_manager const& p_other) =
    delete;
  advanced_timer_manager(advanced_timer_manager&& p_other) noexcept = delete;
  advanced_timer_manager& operator=(advanced_timer_manager&& p_other) noexcept =
    delete;

  ~advanced_timer_manager();

  /**
   * @brief Creates a timer object to be used for scheduling function callbacks
   * via interrupts.
   *
   * @return hal::stm32f1::timer - the timer object
   *
   * @throws hal::device_or_resource_busy - if timer is already being used with
   * a conflicting resource
   */
  [[nodiscard]] hal::stm32f1::timer acquire_timer();
  /**
   * @brief Creates a pwm object to be used for setting the frequency of the
   * group of pwm channels for the acquired timer.
   *
   * @return hal::stm32f1::pwm_group_frequency - the pwm frequency object.
   *
   * @throws hal::device_or_resource_busy - if timer is already being used with
   * a conflicting resource
   */
  [[nodiscard]] hal::stm32f1::pwm_group_frequency acquire_pwm_group_frequency();

protected:
  /**
   * @brief The constructor is protected because it gets inherited and used by
   * the templated advanced_timer class.
   *
   * @param p_id the id of the timer that will be used by the manager.
   */
  advanced_timer_manager(peripheral p_id);

  /**
   * @brief Creates a pwm channel to be used for generating pulse-widths through
   * configuring of its duty-cycle.
   *
   * @return hal::stm32f1::pwm16_channel - the pwm channel object.
   *
   * @throws hal::device_or_resource_busy - if timer is already being used with
   * a conflicting resource
   */
  [[nodiscard]] hal::stm32f1::pwm16_channel acquire_pwm16_channel(
    timer_pins p_pin);

  /**
   * @brief Creates a pwm object to be used for generating pulse-widths through
   * setting of frequency and duty-cycle.
   * @deprecated Please use pwm16_channel and pwm_group_frequency. This class
   * implements the `hal::pwm` interface which will be deprecated in libhal 5.
   *
   * @return hal::stm32f1::pwm - the pwm object.
   *
   * @throws hal::device_or_resource_busy - if timer is already being used with
   * a conflicting resource
   */
  [[nodiscard]] hal::stm32f1::pwm acquire_pwm(timer_pins p_pin);

  /**
   * @brief Creates a quadrature encoder object used to measure the rotation of
   * a motor that gives out quadrature encoder feedback.
   *
   * @param p_allocator Provide the resource that this can be allocated to.
   * @param p_pin1 First timer pin corresponding to channel 1
   * @param p_pin2 Second timer pin corresponding to channel 2
   * @param p_pulses_per_rotation Cycles per revolution for the quadrature
   encoder.

   * @return hal::v5::strong_ptr<hal::rotation_sensor>
   *
   * @throws hal::device_or_resource_busy - if timer is already being used with
   * a conflicting resource.
   * @throws hal::operation_not_permitted error - if any of the channels are not
   * 1 and 2.
   */
  [[nodiscard]] hal::v5::strong_ptr<hal::rotation_sensor>
  acquire_quadrature_encoder(std::pmr::polymorphic_allocator<> p_allocator,
                             timer_pins p_pin1,
                             timer_pins p_pin2,
                             float p_pulses_per_rotation);

private:
  template<typename... T>
  static hal::v5::strong_ptr<hal::rotation_sensor> create(
    std::pmr::polymorphic_allocator<> p_allocator,
    T&&... args);
  /// Stores the state information about the timer thats assigned to this
  /// manager.
  timer_manager_data m_manager_data;
};

/**
 * @brief This class is used to do any timer operations for timers 1 and 8.
 *
 * This templated version inherits the core functionality and minimizes code
 * bloat by only instantiating templates for the required functions. Since they
 * are also inlined, it allows for additional compiler optimizations.
 *
 * These timers can be used to do pwm generation, as well as other advanced
 * timer specific tasks
 */
template<peripheral select>
class advanced_timer final : public advanced_timer_manager
{
public:
  /**
   * @brief These are the 2 pins that correspond to channel a and b. It is
   * important for the pins to be in the correct order otherwise the counter
   * will work in the opposite direction.
   */
  struct encoder_pins
  {
    hal::stm32f1::timer_pins channel_a_pin;
    hal::stm32f1::timer_pins channel_b_pin;
  };
  static_assert(
    select == peripheral::timer1 or select == peripheral::timer8,
    "Only timer 1 or 8 is allowed as advanced timers for this driver.");
  using pin_type = decltype(get_pwm_timer_type<select>())::type;
  /**
   * @brief Construct a advanced_timer_manager object using the passed template
   * argument.
   *
   */
  advanced_timer()
    : advanced_timer_manager(select)
  {
  }

  /**
   * @brief Creates a pwm channel to be used for generating pulse-widths through
   * configuring of its duty-cycle.
   *
   * @return hal::stm32f1::pwm16_channel - the pwm channel object.
   *
   * @throws hal::device_or_resource_busy - if timer is already being used with
   * a conflicting resource
   */
  [[nodiscard]] hal::stm32f1::pwm16_channel acquire_pwm16_channel(
    pin_type p_pin)
  {
    return advanced_timer_manager::acquire_pwm16_channel(
      static_cast<timer_pins>(p_pin));
  }

  /**
   * @brief Creates a pwm object to be used for generating pulse-widths through
   * setting of frequency and duty-cycle.
   * @deprecated Please use pwm16_channel and pwm_group_frequency. This class
   * implements the `hal::pwm` interface which will be deprecated in libhal 5.
   *
   * @return hal::stm32f1::pwm - the pwm object.
   *
   * @throws hal::device_or_resource_busy - if timer is already being used with
   * a conflicting resource
   */
  [[nodiscard]] hal::stm32f1::pwm acquire_pwm(pin_type p_pin)
  {
    return advanced_timer_manager::acquire_pwm(static_cast<timer_pins>(p_pin));
  }
  /**
   * @brief Creates a quadrature encoder object used to measure the rotation of
   * a motor that gives out quadrature encoder feedback.
   *
   * @param p_allocator Provide the resource that this can be allocated to.
   * @param p_encoder_pins The pins must be part of the same timer and they must
   * correspond to channel 1 and 2 respectively. Any channel other than channel
   * 1 and 2 will throw an hal::operation_not_permitted error. If the channels
   * are flipped, it will result degrees counted in the wrong direction.
   * @param p_pulses_per_rotation Cycles per revolution for the quadrature
   encoder.

   * @return hal::v5::strong_ptr<hal::rotation_sensor>
   *
   * @throws hal::device_or_resource_busy - if timer is already being used with
   * a conflicting resource.
   * @throws hal::operation_not_permitted error - if any of the channels are not
   * 1 and 2.
   */
  [[nodiscard]] hal::v5::strong_ptr<hal::rotation_sensor>
  acquire_quadrature_encoder(std::pmr::polymorphic_allocator<> p_allocator,
                             encoder_pins p_encoder_pins,
                             float p_pulses_per_rotation)
  {
    return advanced_timer_manager::acquire_quadrature_encoder(
      p_allocator,
      static_cast<timer_pins>(p_encoder_pins.channel_a_pin),
      static_cast<timer_pins>(p_encoder_pins.channel_b_pin),
      p_pulses_per_rotation);
  }

private:
  friend hal::v5::strong_ptr<hal::pwm16_channel> acquire_pwm16_channel(
    std::pmr::polymorphic_allocator<> p_allocator,
    advanced_timer_manager m_manager);

  friend hal::v5::strong_ptr<hal::pwm_group_manager> acquire_pwm_group_manager(
    std::pmr::polymorphic_allocator<> p_allocator,
    advanced_timer_manager m_manager);

  friend hal::v5::strong_ptr<hal::rotation_sensor> acquire_quadrature_encoder(
    std::pmr::polymorphic_allocator<> p_allocator,
    advanced_timer_manager m_manager);
};

/**
 * @brief This class is used to do any timer operations for the general purpose
 * timers: 2, 3, 4, 5, 9, 10, 11, 12, 13, 14.
 *
 * This is the non-templated version that implements all the core functionality.
 * It is designed to be inherited by the templated version, helping to reduce
 * code bloat and duplication caused by template instantiations.
 *
 * These timers can be used to do pwm generation, as well as other general
 * purpose timer tasks
 */
class general_purpose_timer_manager
{
public:
  general_purpose_timer_manager(general_purpose_timer_manager const& p_other) =
    delete;
  general_purpose_timer_manager& operator=(
    general_purpose_timer_manager const& p_other) = delete;
  general_purpose_timer_manager(
    general_purpose_timer_manager&& p_other) noexcept = delete;
  general_purpose_timer_manager& operator=(
    general_purpose_timer_manager&& p_other) noexcept = delete;

  ~general_purpose_timer_manager();

  /**
   * @brief Creates a timer object to be used for scheduling function callbacks
   * via interrupts.
   *
   * @return hal::stm32f1::timer - the timer object
   *
   * @throws hal::device_or_resource_busy - if timer is already being used with
   * a conflicting resource
   */
  [[nodiscard]] hal::stm32f1::timer acquire_timer();

  /**
   * @brief Creates a pwm object to be used for setting the frequency of the
   * group of pwm channels for the acquired timer.
   *
   * @return hal::stm32f1::pwm_group_frequency - the pwm frequency object.
   *
   * @throws hal::device_or_resource_busy - if timer is already being used with
   * a conflicting resource
   */
  [[nodiscard]] hal::stm32f1::pwm_group_frequency acquire_pwm_group_frequency();

protected:
  /**
   * @brief The constructor is protected because it gets inherited and used by
   * the templated general_purpose_timer class.
   *
   * @param p_id the id of the timer that will be used by the manager.
   */
  general_purpose_timer_manager(peripheral p_id);

  /**
   * @brief Creates a pwm object to be used for generating pulse-widths through
   * setting of frequency and duty-cycle.
   * @deprecated Please use pwm16_channel and pwm_group_frequency. This class
   * implements the `hal::pwm` interface which will be deprecated in libhal 5.
   *
   * @return hal::stm32f1::pwm - the pwm object.
   *
   * @throws hal::device_or_resource_busy - if timer is already being used with
   * a conflicting resource
   */
  [[nodiscard]] hal::stm32f1::pwm acquire_pwm(timer_pins p_pin);

  /**
   * @brief Creates a pwm channel to be used for generating pulse-widths through
   * configuring of its duty-cycle.
   *
   * @return hal::stm32f1::pwm16_channel - the pwm channel object.
   *
   * @throws hal::device_or_resource_busy - if timer is already being used with
   * a conflicting resource
   */
  [[nodiscard]] hal::stm32f1::pwm16_channel acquire_pwm16_channel(
    timer_pins p_pin);

  /**
   * @brief Creates a quadrature encoder object used to measure the rotation of
   * a motor that gives out quadrature encoder feedback.
   *
   * @param p_allocator Provide the resource that this can be allocated to.
   * @param p_pin1 First timer pin corresponding to channel 1
   * @param p_pin2 Second timer pin corresponding to channel 2
   * @param p_pulses_per_rotation Cycles per revolution for the quadrature
   encoder.

   * @return hal::v5::strong_ptr<hal::rotation_sensor>
   *
   * @throws hal::device_or_resource_busy - if timer is already being used with
   * a conflicting resource.
   * @throws hal::operation_not_permitted error - if any of the channels are not
   * 1 and 2.
   */
  [[nodiscard]] hal::v5::strong_ptr<hal::rotation_sensor>
  acquire_quadrature_encoder(std::pmr::polymorphic_allocator<> p_allocator,
                             timer_pins p_pin1,
                             timer_pins p_pin2,
                             float p_pulses_per_rotation);

private:
  template<typename... T>
  static hal::v5::strong_ptr<hal::rotation_sensor> create(
    std::pmr::polymorphic_allocator<> p_allocator,
    T&&... args);
  /// Stores the state information about the timer thats assigned to this
  /// manager.
  timer_manager_data m_manager_data;
};

/**
 * @brief This class is used to do any timer operations for the general purpose
 * timers: 2, 3, 4, 5, 9, 10, 11, 12, 13, 14.
 *
 * This templated version inherits the core functionality and minimizes code
 * bloat by only instantiating templates for the required functions. Since they
 * are also inlined, it allows for additional compiler optimizations.
 *
 * These timers can be used to do pwm generation, as well as other general
 * purpose timer tasks
 */
template<peripheral select>
class general_purpose_timer : public general_purpose_timer_manager
{
public:
  /**
   * @brief These are the 2 pins that correspond to channel a and b. It is
   * important for the pins to be in the correct order otherwise the counter
   * will work in the opposite direction.
   */
  struct encoder_pins
  {
    hal::stm32f1::timer_pins channel_a_pin;
    hal::stm32f1::timer_pins channel_b_pin;
  };

  static_assert(
    select == peripheral::timer2 or select == peripheral::timer3 or
      select == peripheral::timer4 or select == peripheral::timer5 or
      select == peripheral::timer9 or select == peripheral::timer10 or
      select == peripheral::timer11 or select == peripheral::timer12 or
      select == peripheral::timer13 or select == peripheral::timer14,
    "Only timers 2, 3, 4, 5, 9, 10, 11, 12, 13, and 14 are allowed as general "
    "purpose timers for this driver.");
  using pin_type = decltype(get_pwm_timer_type<select>())::type;

  /**
   * @brief Construct a general_purpose_timer_manager object using the passed
   * template argument.
   *
   */
  general_purpose_timer()
    : general_purpose_timer_manager(select)
  {
  }

  /**
   * @brief Creates a pwm channel to be used for generating pulse-widths through
   * configuring of its duty-cycle.
   *
   * @return hal::stm32f1::pwm16_channel - the pwm channel object.
   *
   * @throws hal::device_or_resource_busy - if timer is already being used with
   * a conflicting resource
   */
  [[nodiscard]] hal::stm32f1::pwm16_channel acquire_pwm16_channel(
    pin_type p_pin)
  {
    return general_purpose_timer_manager::acquire_pwm16_channel(
      static_cast<timer_pins>(p_pin));
  }

  /**
   * @brief Creates a pwm object to be used for generating pulse-widths through
   * setting of frequency and duty-cycle.
   * @deprecated Please use pwm16_channel and pwm_group_frequency. This class
   * implements the `hal::pwm` interface which will be deprecated in libhal 5.
   *
   * @return hal::stm32f1::pwm - the pwm object.
   *
   * @throws hal::device_or_resource_busy - if timer is already being used with
   * a conflicting resource
   */
  [[nodiscard]] hal::stm32f1::pwm acquire_pwm(pin_type p_pin)
  {
    return general_purpose_timer_manager::acquire_pwm(
      static_cast<timer_pins>(p_pin));
  }

  /**
   * @brief Creates a quadrature encoder object used to measure the rotation of
   * a motor that gives out quadrature encoder feedback.
   *

   * @param p_allocator Provide the resource that this can be allocated to.
   * @param p_encoder_pins The pins must be part of the same timer and they must
   * correspond to channel 1 and 2 respectively. Any channel other than channel
   * 1 and 2 will throw an hal::operation_not_permitted error. If the channels
   * are flipped, it will result degrees counted in the wrong direction.
   * @param p_pulses_per_rotation Cycles per revolution for the quadrature
   encoder.

   * @return hal::v5::strong_ptr<hal::rotation_sensor>
   *
   * @throws hal::device_or_resource_busy - if timer is already being used with
   * a conflicting resource.
   * @throws hal::operation_not_permitted error - if any of the channels are not
   * 1 and 2.
   */
  [[nodiscard]] hal::v5::strong_ptr<hal::rotation_sensor>
  acquire_quadrature_encoder(std::pmr::polymorphic_allocator<> p_allocator,
                             encoder_pins p_encoder_pins,
                             float p_pulses_per_rotation)
  {
    return general_purpose_timer_manager::acquire_quadrature_encoder(
      p_allocator,
      p_encoder_pins.channel_a_pin,
      p_encoder_pins.channel_b_pin,
      p_pulses_per_rotation);
  }

private:
  friend hal::v5::strong_ptr<hal::pwm16_channel> acquire_pwm16_channel(
    std::pmr::polymorphic_allocator<> p_allocator,
    general_purpose_timer_manager m_manager);

  friend hal::v5::strong_ptr<hal::pwm_group_manager> acquire_pwm_group_manager(
    std::pmr::polymorphic_allocator<> p_allocator,
    general_purpose_timer_manager m_manager);

  friend hal::v5::strong_ptr<hal::rotation_sensor> acquire_quadrature_encoder(
    std::pmr::polymorphic_allocator<> p_allocator,
    general_purpose_timer_manager m_manager);
};
}  // namespace hal::stm32f1
