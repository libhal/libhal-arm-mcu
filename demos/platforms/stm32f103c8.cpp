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

#include <libhal-arm-mcu/dwt_counter.hpp>
#include <libhal-arm-mcu/startup.hpp>
#include <libhal-arm-mcu/stm32f1/adc.hpp>
#include <libhal-arm-mcu/stm32f1/can2.hpp>
#include <libhal-arm-mcu/stm32f1/clock.hpp>
#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal-arm-mcu/stm32f1/gpio.hpp>
#include <libhal-arm-mcu/stm32f1/independent_watchdog.hpp>
#include <libhal-arm-mcu/stm32f1/input_pin.hpp>
#include <libhal-arm-mcu/stm32f1/output_pin.hpp>
#include <libhal-arm-mcu/stm32f1/pin.hpp>
#include <libhal-arm-mcu/stm32f1/spi.hpp>
#include <libhal-arm-mcu/stm32f1/timer.hpp>
#include <libhal-arm-mcu/stm32f1/uart.hpp>
#include <libhal-arm-mcu/stm32f1/usart.hpp>
#include <libhal-arm-mcu/stm32f1/usb.hpp>
#include <libhal-arm-mcu/system_control.hpp>
#include <libhal-exceptions/control.hpp>
#include <libhal-util/atomic_spin_lock.hpp>
#include <libhal-util/bit_bang_i2c.hpp>
#include <libhal-util/bit_bang_spi.hpp>
#include <libhal-util/inert_drivers/inert_adc.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/pointers.hpp>
#include <libhal/pwm.hpp>
#include <libhal/units.hpp>

#include <libhal/usb.hpp>
#include <memory_resource>
#include <resource_list.hpp>

namespace resources {
using namespace hal::literals;
using st_peripheral = hal::stm32f1::peripheral;

std::array<hal::byte, 1024> driver_memory{};
std::pmr::monotonic_buffer_resource resource(driver_memory.data(),
                                             driver_memory.size(),
                                             std::pmr::null_memory_resource());

std::pmr::memory_resource* driver_allocator()
{
  return &resource;
}

hal::v5::optional_ptr<hal::stm32f1::gpio<st_peripheral::gpio_a>> gpio_a_ptr;
hal::v5::optional_ptr<hal::stm32f1::gpio<st_peripheral::gpio_b>> gpio_b_ptr;
hal::v5::optional_ptr<hal::stm32f1::gpio<st_peripheral::gpio_c>> gpio_c_ptr;

hal::v5::strong_ptr<hal::stm32f1::gpio<st_peripheral::gpio_a>> gpio_a()
{
  if (not gpio_a_ptr) {
    gpio_a_ptr =
      hal::v5::make_strong_ptr<hal::stm32f1::gpio<st_peripheral::gpio_a>>(
        driver_allocator());
  }
  return gpio_a_ptr;
}

hal::v5::strong_ptr<hal::stm32f1::gpio<st_peripheral::gpio_b>> gpio_b()
{
  if (not gpio_b_ptr) {
    gpio_b_ptr =
      hal::v5::make_strong_ptr<hal::stm32f1::gpio<st_peripheral::gpio_b>>(
        driver_allocator());
  }
  return gpio_b_ptr;
}

hal::v5::strong_ptr<hal::stm32f1::gpio<st_peripheral::gpio_c>> gpio_c()
{
  if (not gpio_c_ptr) {
    gpio_c_ptr =
      hal::v5::make_strong_ptr<hal::stm32f1::gpio<st_peripheral::gpio_c>>(
        driver_allocator());
  }
  return gpio_c_ptr;
}

hal::v5::optional_ptr<hal::cortex_m::dwt_counter> clock_ptr;
hal::v5::strong_ptr<hal::steady_clock> clock()
{
  if (not clock_ptr) {
    auto cpu_frequency = hal::stm32f1::frequency(hal::stm32f1::peripheral::cpu);
    clock_ptr = hal::v5::make_strong_ptr<hal::cortex_m::dwt_counter>(
      driver_allocator(), cpu_frequency);
  }
  return clock_ptr;
}

hal::v5::optional_ptr<hal::stm32f1::usb> usb_ptr;
auto usb()
{
  using namespace std::chrono_literals;
  if (not usb_ptr) {
    usb_ptr = hal::v5::make_strong_ptr<hal::stm32f1::usb>(
      driver_allocator(), clock(), 100ms);
  }
  return usb_ptr;
}

hal::v5::optional_ptr<hal::v5::usb::control_endpoint> control_ep_ptr;
hal::v5::strong_ptr<hal::v5::usb::control_endpoint> usb_control_endpoint()
{
  if (not control_ep_ptr) {
    control_ep_ptr =
      hal::acquire_usb_control_endpoint(driver_allocator(), usb());
  }
  return control_ep_ptr;
}

hal::v5::optional_ptr<hal::v5::usb::interrupt_in_endpoint> interrupt_in_ep1_ptr;
hal::v5::optional_ptr<hal::v5::usb::interrupt_out_endpoint>
  interrupt_out_ep1_ptr;
hal::v5::strong_ptr<hal::v5::usb::interrupt_in_endpoint>
usb_interrupt_in_endpoint1()
{
  if (not interrupt_in_ep1_ptr) {
    auto ep = hal::acquire_usb_interrupt_endpoint(driver_allocator(), usb());
    interrupt_in_ep1_ptr = ep.in;
    interrupt_out_ep1_ptr = ep.out;
  }
  return interrupt_in_ep1_ptr;
}
hal::v5::strong_ptr<hal::v5::usb::interrupt_out_endpoint>
usb_interrupt_out_endpoint1()
{
  if (not interrupt_out_ep1_ptr) {
    auto ep = hal::acquire_usb_interrupt_endpoint(driver_allocator(), usb());
    interrupt_in_ep1_ptr = ep.in;
    interrupt_out_ep1_ptr = ep.out;
  }
  return interrupt_out_ep1_ptr;
}

hal::v5::optional_ptr<hal::v5::usb::interrupt_in_endpoint> interrupt_in_ep2_ptr;
hal::v5::optional_ptr<hal::v5::usb::interrupt_out_endpoint>
  interrupt_out_ep2_ptr;
hal::v5::strong_ptr<hal::v5::usb::interrupt_in_endpoint>
usb_interrupt_in_endpoint2()
{
  if (not interrupt_in_ep2_ptr) {
    auto ep = hal::acquire_usb_interrupt_endpoint(driver_allocator(), usb());
    interrupt_in_ep2_ptr = ep.in;
    interrupt_out_ep2_ptr = ep.out;
  }
  return interrupt_in_ep2_ptr;
}

hal::v5::strong_ptr<hal::v5::usb::interrupt_out_endpoint>
usb_interrupt_out_endpoint2()
{
  if (not interrupt_out_ep2_ptr) {
    auto ep = hal::acquire_usb_interrupt_endpoint(driver_allocator(), usb());
    interrupt_in_ep2_ptr = ep.in;
    interrupt_out_ep2_ptr = ep.out;
  }
  return interrupt_out_ep2_ptr;
}

hal::v5::optional_ptr<hal::v5::usb::bulk_in_endpoint> bulk_in_ep1_ptr;
hal::v5::optional_ptr<hal::v5::usb::bulk_out_endpoint> bulk_out_ep1_ptr;
hal::v5::strong_ptr<hal::v5::usb::bulk_in_endpoint> usb_bulk_in_endpoint1()
{
  if (not bulk_in_ep1_ptr) {
    auto ep = hal::acquire_usb_bulk_endpoint(driver_allocator(), usb());
    bulk_in_ep1_ptr = ep.in;
    bulk_out_ep1_ptr = ep.out;
  }
  return bulk_in_ep1_ptr;
}

hal::v5::strong_ptr<hal::v5::usb::bulk_out_endpoint> usb_bulk_out_endpoint1()
{
  if (not bulk_out_ep1_ptr) {
    auto ep = hal::acquire_usb_bulk_endpoint(driver_allocator(), usb());
    bulk_in_ep1_ptr = ep.in;
    bulk_out_ep1_ptr = ep.out;
  }
  return bulk_out_ep1_ptr;
}

hal::v5::optional_ptr<hal::v5::usb::bulk_in_endpoint> bulk_in_ep2_ptr;
hal::v5::optional_ptr<hal::v5::usb::bulk_out_endpoint> bulk_out_ep2_ptr;
hal::v5::strong_ptr<hal::v5::usb::bulk_in_endpoint> usb_bulk_in_endpoint2()
{
  if (not bulk_in_ep2_ptr) {
    auto ep = hal::acquire_usb_bulk_endpoint(driver_allocator(), usb());
    bulk_in_ep2_ptr = ep.in;
    bulk_out_ep2_ptr = ep.out;
  }
  return bulk_in_ep2_ptr;
}

hal::v5::strong_ptr<hal::v5::usb::bulk_out_endpoint> usb_bulk_out_endpoint2()
{
  if (not bulk_out_ep2_ptr) {
    auto ep = hal::acquire_usb_bulk_endpoint(driver_allocator(), usb());
    bulk_in_ep2_ptr = ep.in;
    bulk_out_ep2_ptr = ep.out;
  }
  return bulk_out_ep2_ptr;
}

hal::v5::strong_ptr<hal::serial> console()
{
  return hal::v5::make_strong_ptr<hal::stm32f1::uart>(
    driver_allocator(), hal::port<1>, hal::buffer<128>);
}

hal::v5::optional_ptr<hal::output_pin> led_ptr;
hal::v5::strong_ptr<hal::output_pin> status_led()
{
  if (not led_ptr) {
    led_ptr = hal::acquire_output_pin(driver_allocator(), gpio_c(), 13);
  }
  return led_ptr;
}

hal::v5::strong_ptr<hal::adc> adc()
{
  static hal::atomic_spin_lock adc_lock;
  static hal::stm32f1::adc<st_peripheral::adc1> adc(adc_lock);
  return hal::acquire_adc(driver_allocator(), adc, hal::stm32f1::adc_pins::pb0);
}

hal::v5::strong_ptr<hal::i2c> i2c()
{
  // TODO(#167): Use a version of bit_bang_i2c that accepts strong_ptr's
  static auto sda_output_pin =
    hal::acquire_output_pin(driver_allocator(), gpio_b(), 7);
  static auto scl_output_pin =
    hal::acquire_output_pin(driver_allocator(), gpio_b(), 6);
  auto clock = resources::clock();
  return hal::v5::make_strong_ptr<hal::bit_bang_i2c>(
    driver_allocator(),
    hal::bit_bang_i2c::pins{
      .sda = &(*sda_output_pin),
      .scl = &(*scl_output_pin),
    },
    *clock);
}

hal::v5::strong_ptr<hal::spi> spi()
{
  return hal::v5::make_strong_ptr<hal::stm32f1::spi>(driver_allocator(),
                                                     hal::bus<1>,
                                                     hal::spi::settings{
                                                       .clock_rate = 250.0_kHz,
                                                       .clock_polarity = false,
                                                       .clock_phase = false,
                                                     });
}

hal::v5::strong_ptr<hal::output_pin> spi_chip_select()
{
  return hal::acquire_output_pin(driver_allocator(), gpio_a(), 4);
}

hal::v5::strong_ptr<hal::input_pin> input_pin()
{
  return hal::acquire_input_pin(driver_allocator(), gpio_b(), 4);
}

auto& timer1()
{
  static hal::stm32f1::advanced_timer<st_peripheral::timer1> timer1{};
  return timer1;
}

hal::v5::strong_ptr<hal::timer> timed_interrupt()
{
#if 0
  static hal::stm32f1::general_purpose_timer<st_peripheral::timer2> timer2;
  auto timer_callback_timer = timer2.acquire_timer();
  return hal::v5::make_strong_ptr<decltype(timer_callback_timer)>(
    driver_allocator(), std::move(timer_callback_timer));
#endif
  throw hal::operation_not_supported(nullptr);
}

hal::v5::strong_ptr<hal::pwm> pwm()
{
  static auto timer_old_pwm =
    timer1().acquire_pwm(hal::stm32f1::timer1_pin::pa8);
  return hal::v5::make_strong_ptr<decltype(timer_old_pwm)>(
    driver_allocator(), std::move(timer_old_pwm));
}

hal::v5::strong_ptr<hal::pwm16_channel> pwm_channel()
{
  auto timer_pwm_channel =
    timer1().acquire_pwm16_channel(hal::stm32f1::timer1_pin::pa8);
  return hal::v5::make_strong_ptr<decltype(timer_pwm_channel)>(
    driver_allocator(), std::move(timer_pwm_channel));
}

hal::v5::strong_ptr<hal::pwm_group_manager> pwm_frequency()
{
  auto timer_pwm_frequency = timer1().acquire_pwm_group_frequency();
  return hal::v5::make_strong_ptr<decltype(timer_pwm_frequency)>(
    driver_allocator(), std::move(timer_pwm_frequency));
}

hal::v5::optional_ptr<hal::stm32f1::can_peripheral_manager_v2> can_manager;

void initialize_can()
{
  if (not can_manager) {
    auto clock_ref = clock();
    can_manager =
      hal::v5::make_strong_ptr<hal::stm32f1::can_peripheral_manager_v2>(
        driver_allocator(),
        32,
        driver_allocator(),
        100'000,
        *clock_ref,
        std::chrono::milliseconds(1),
        hal::stm32f1::can_pins::pb9_pb8);
  }
}

hal::v5::strong_ptr<hal::can_transceiver> can_transceiver()
{
  initialize_can();
  return hal::acquire_can_transceiver(driver_allocator(), can_manager);
}

hal::v5::strong_ptr<hal::can_bus_manager> can_bus_manager()
{
  initialize_can();
  return hal::acquire_can_bus_manager(driver_allocator(), can_manager);
}

hal::v5::strong_ptr<hal::can_identifier_filter> can_identifier_filter()
{
  initialize_can();
  return hal::acquire_can_identifier_filter(driver_allocator(), can_manager)[0];
}

hal::v5::strong_ptr<hal::can_interrupt> can_interrupt()
{
  initialize_can();
  return hal::acquire_can_interrupt(driver_allocator(), can_manager);
}

hal::v5::strong_ptr<hal::interrupt_pin> interrupt_pin()
{
  throw hal::operation_not_supported(nullptr);
}

hal::v5::strong_ptr<hal::stream_dac_u8> stream_dac()
{
  throw hal::operation_not_supported(nullptr);
}

hal::v5::strong_ptr<hal::dac> dac()
{
  throw hal::operation_not_supported(nullptr);
}

// Watchdog implementation using global function pattern from original
class stm32f103c8_watchdog : public custom::watchdog
{
public:
  void start() override
  {
    m_stm_watchdog.start();
  }

  void reset() override
  {
    m_stm_watchdog.reset();
  }

  void set_countdown_time(hal::time_duration p_wait_time) override
  {
    m_stm_watchdog.set_countdown_time(p_wait_time);
  }

  bool check_flag() override
  {
    return m_stm_watchdog.check_flag();
  }

  void clear_flag() override
  {
    m_stm_watchdog.clear_flag();
  }

private:
  hal::stm32f1::independent_watchdog m_stm_watchdog{};
};

hal::v5::strong_ptr<custom::watchdog> watchdog()
{
  return hal::v5::make_strong_ptr<stm32f103c8_watchdog>(driver_allocator());
}

[[noreturn]] void terminate_handler() noexcept
{
  if (not led_ptr && not clock_ptr) {
    // spin here until debugger is connected
    while (true) {
      continue;
    }
  }

  // Otherwise, blink the led in a pattern
  auto status_led = resources::status_led();
  auto clock = resources::clock();

  while (true) {
    using namespace std::chrono_literals;
    status_led->level(false);
    hal::delay(*clock, 100ms);
    status_led->level(true);
    hal::delay(*clock, 100ms);
    status_led->level(false);
    hal::delay(*clock, 100ms);
    status_led->level(true);
    hal::delay(*clock, 1000ms);
  }
}

}  // namespace resources

namespace hal::stm32f1 {
class manager
{
public:
  manager(hal::v5::strong_ptr_only_token,
          std::pmr::polymorphic_allocator<> p_allocator)
    : m_allocator(p_allocator)
  {
  }

  manager(manager&) = delete;
  auto& operator=(manager const&) = delete;
  manager(manager&&) = delete;
  auto& operator=(manager&&) = delete;

  void reconfigure_clocks(hal::stm32f1::clock_tree const& p_clock_configuration)
  {
    hal::stm32f1::configure_clocks(p_clock_configuration);
  }

  template<peripheral Peripheral>
  hal::v5::strong_ptr<hal::stm32f1::gpio<Peripheral>> gpio()
  {
    return hal::v5::make_strong_ptr<hal::stm32f1::gpio<Peripheral>>(
      m_allocator);
  }

  hal::v5::strong_ptr<hal::stm32f1::usb> usb(
    hal::v5::strong_ptr<hal::steady_clock> const& p_clock,
    hal::time_duration p_write_timeout = std::chrono::milliseconds(100))
  {
    return hal::v5::make_strong_ptr<hal::stm32f1::usb>(
      m_allocator, p_clock, p_write_timeout);
  }

  template<peripheral Peripheral>
  hal::v5::strong_ptr<advanced_timer<Peripheral>> advanced_timer()
  {
    static_assert(Peripheral == peripheral::timer1 or
                    Peripheral == peripheral::timer8,
                  "Only timer1 and timer8 are advanced timers");
    return hal::v5::make_strong_ptr<hal::stm32f1::advanced_timer<Peripheral>>(
      m_allocator);
  }

  template<peripheral Peripheral>
  hal::v5::strong_ptr<general_purpose_timer<Peripheral>> general_purpose_timer()
  {
    static_assert(
      Peripheral == peripheral::timer2 or Peripheral == peripheral::timer3 or
        Peripheral == peripheral::timer4 or Peripheral == peripheral::timer5 or
        Peripheral == peripheral::timer9 or Peripheral == peripheral::timer10 or
        Peripheral == peripheral::timer11 or
        Peripheral == peripheral::timer12 or
        Peripheral == peripheral::timer13 or Peripheral == peripheral::timer14,
      "Only timer2-5 and timer9-14 are general purpose timers");
    return hal::v5::make_strong_ptr<
      hal::stm32f1::general_purpose_timer<Peripheral>>(m_allocator);
  }

  template<peripheral Peripheral>
  hal::v5::strong_ptr<hal::stm32f1::adc<Peripheral>> adc(
    hal::basic_lock& p_lock)
  {
    static_assert(Peripheral == peripheral::adc1 or
                    Peripheral == peripheral::adc2 or
                    Peripheral == peripheral::adc3,
                  "Only adc1, adc2, and adc3 are valid ADC peripherals");
    return hal::v5::make_strong_ptr<hal::stm32f1::adc<Peripheral>>(m_allocator,
                                                                   p_lock);
  }

  template<peripheral Peripheral>
  hal::v5::strong_ptr<hal::stm32f1::usart<Peripheral>> usart()
  {
    static_assert(
      Peripheral == peripheral::usart1 or Peripheral == peripheral::usart2 or
        Peripheral == peripheral::usart3,
      "Only usart1, usart2, and usart3 are valid USART peripherals");
    return hal::v5::make_strong_ptr<hal::stm32f1::usart<Peripheral>>(
      m_allocator);
  }

  hal::v5::strong_ptr<hal::stm32f1::can_peripheral_manager_v2> can(
    hal::usize p_message_count,
    hal::u32 p_baud_rate,
    hal::steady_clock& p_clock,
    hal::time_duration p_timeout_time = std::chrono::milliseconds(1),
    hal::stm32f1::can_pins p_pins = hal::stm32f1::can_pins::pa11_pa12,
    hal::stm32f1::can_self_test p_enable_self_test =
      hal::stm32f1::can_self_test::off)
  {
    return hal::v5::make_strong_ptr<hal::stm32f1::can_peripheral_manager_v2>(
      m_allocator,
      p_message_count,
      m_allocator,
      p_baud_rate,
      p_clock,
      p_timeout_time,
      p_pins,
      p_enable_self_test);
  }

  template<std::uint8_t BusNumber>
  hal::v5::strong_ptr<hal::stm32f1::spi> spi(
    hal::spi::settings const& p_settings = {})
  {
    static_assert(BusNumber >= 1 and BusNumber <= 3,
                  "STM32F1 only supports SPI bus numbers 1-3");
    return hal::v5::make_strong_ptr<hal::stm32f1::spi>(
      m_allocator, hal::bus<BusNumber>, p_settings);
  }

  hal::v5::strong_ptr<hal::cortex_m::dwt_counter> dwt_counter()
  {
    auto cpu_frequency = hal::stm32f1::frequency(hal::stm32f1::peripheral::cpu);
    return hal::v5::make_strong_ptr<hal::cortex_m::dwt_counter>(m_allocator,
                                                                cpu_frequency);
  }

private:
  friend hal::v5::strong_ptr<manager> init(
    std::pmr::memory_resource* p_allocator,
    std::optional<hal::stm32f1::clock_tree> p_clock_configuration);
  std::pmr::polymorphic_allocator<> m_allocator;
};

hal::v5::strong_ptr<manager> init(
  std::pmr::memory_resource* p_allocator,
  std::optional<hal::stm32f1::clock_tree> p_clock_configuration = std::nullopt)
{
  if (p_clock_configuration) {
    hal::stm32f1::configure_clocks(*p_clock_configuration);
  }
  return hal::v5::make_strong_ptr<manager>(p_allocator, p_allocator);
}
}  // namespace hal::stm32f1

template<typename T>
hal::v5::strong_ptr<T> track_lifetime(
  std::pmr::polymorphic_allocator<> p_allocator,
  T&& p_obj)
{
  return hal::v5::make_strong_ptr<T>(p_allocator, std::forward<T>(p_obj));
}

void initialize_platform2()
{
  using namespace hal::literals;
  using st_peripherals = hal::stm32f1::peripheral;
  hal::set_terminate(resources::terminate_handler);
  // Set the MCU to the maximum clock speed
  auto stm32f1 = hal::stm32f1::init(resources::driver_allocator());
  auto steady_clock = stm32f1->dwt_counter();
  auto gpio_a = stm32f1->gpio<st_peripherals::gpio_a>();
  auto gpio_b = stm32f1->gpio<st_peripherals::gpio_b>();
  auto usb = stm32f1->usb(steady_clock);
#if 1
  // Claude: Desired initialization code
  auto led = gpio_a->output_pin(13);
  auto sda_output_pin = gpio_b->output_pin(7);
  auto scl_output_pin = gpio_b->output_pin(6);
  auto control_endpoint = usb->control_endpoint();
  auto serial_data_ep = usb->bulk_endpoint();
  auto status_ep = usb->interrupt_endpoint();
#else
  // Claude: Necessary code due to the current APIs
  auto led = track_lifetime(resources::driver_allocator(),
                            gpio_a->acquire_output_pin(13));
  auto sda_output_pin = track_lifetime(resources::driver_allocator(),
                                       gpio_b->acquire_output_pin(7));
  auto scl_output_pin = track_lifetime(resources::driver_allocator(),
                                       gpio_b->acquire_output_pin(6));
  auto ctrl_ep =
    hal::acquire_usb_control_endpoint(resources::driver_allocator(), usb);
  auto serial_data_ep =
    hal::acquire_usb_bulk_endpoint(resources::driver_allocator(), usb);
  auto status_ep =
    hal::acquire_usb_interrupt_endpoint(resources::driver_allocator(), usb);
#endif
}

void initialize_platform()
{
  using namespace hal::literals;
  hal::set_terminate(resources::terminate_handler);
  // Set the MCU to the maximum clock speed

  hal::stm32f1::configure_clocks(hal::stm32f1::clock_tree{
    .high_speed_external = 8.0_MHz,
    .pll = {
      .enable = true,
      .source = hal::stm32f1::pll_source::high_speed_external,
      .multiply = hal::stm32f1::pll_multiply::multiply_by_9,
      .usb = {
        .divider = hal::stm32f1::usb_divider::divide_by_1_point_5,
      }
    },
    .system_clock = hal::stm32f1::system_clock_select::pll,
    .ahb = {
      .divider = hal::stm32f1::ahb_divider::divide_by_1,
      .apb1 = {
        .divider = hal::stm32f1::apb_divider::divide_by_2,
      },
      .apb2 = {
        .divider = hal::stm32f1::apb_divider::divide_by_1,
        .adc = {
          .divider = hal::stm32f1::adc_divider::divide_by_6,
        }
      },
    },
  });
  hal::stm32f1::activate_mco_pa8(
    hal::stm32f1::mco_source::pll_clock_divided_by_2);

  hal::stm32f1::release_jtag_pins();
}
