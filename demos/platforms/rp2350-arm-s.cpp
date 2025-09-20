#include <libhal-arm-mcu/rp/adc.hpp>
#include <libhal-arm-mcu/rp/i2c.hpp>
#include <libhal-arm-mcu/rp/input_pin.hpp>
#include <libhal-arm-mcu/rp/interrupt_pin.hpp>
#include <libhal-arm-mcu/rp/output_pin.hpp>
#include <libhal-arm-mcu/rp/pwm.hpp>
#include <libhal-arm-mcu/rp/serial.hpp>
#include <libhal-arm-mcu/rp/spi.hpp>
#include <libhal-arm-mcu/rp/time.hpp>
#include <libhal/initializers.hpp>
#include <libhal/pointers.hpp>
#include <libhal/spi.hpp>
#include <memory_resource>
#include <resource_list.hpp>

namespace rp = hal::rp;

namespace resources {
namespace h5 = hal::v5;
static std::array<hal::byte, 1024> buffer = {};
std::pmr::monotonic_buffer_resource memory(buffer.data(),
                                           buffer.size(),
                                           std::pmr::null_memory_resource());

std::pmr::polymorphic_allocator<> driver_allocator()
{
  return &memory;
}
template<typename T>
using opt = h5::optional_ptr<T>;

template<typename T>
using sptr = h5::strong_ptr<T>;

opt<rp::clock> clk;
sptr<hal::steady_clock> clock()
{
  if (not clk) {
    clk = h5::make_strong_ptr<rp::clock>(&memory);
  }
  return clk;
}

opt<rp::stdio_serial> out;
sptr<hal::serial> console()
{
  if (not out) {
    out = h5::make_strong_ptr<rp::stdio_serial>(&memory);
  }
  return out;
}

opt<rp::output_pin> led;
sptr<hal::output_pin> status_led()
{
  if (not led) {
    // Actually for adafruit board. Change once micromods arrive.
    led = h5::make_strong_ptr<rp::output_pin>(&memory, hal::pin<7>);
  }
  return led;
}

opt<rp::adc> adc0;
sptr<hal::adc> adc()
{
  if (not adc0) {
    adc0 = h5::make_strong_ptr<rp::adc>(&memory, hal::pin<26>);
  }
  return adc0;
}

opt<rp::input_pin> g0;
sptr<hal::input_pin> input_pin()
{
  if (not g0) {
    g0 = h5::make_strong_ptr<rp::input_pin>(&memory, hal::pin<0>);
  }
  return g0;
}

opt<rp::i2c> i2c0;
sptr<hal::i2c> i2c()
{
  if (not i2c0) {
    i2c0 = h5::make_strong_ptr<rp::i2c>(
      &memory, hal::pin<2>, hal::pin<3>, hal::bus<1>);
  }
  return i2c0;
}

opt<rp::interrupt_pin> g1;
hal::callback<hal::interrupt_pin::handler> do_nothing = [](bool) {};
sptr<hal::interrupt_pin> interrupt_pin()
{
  if (not g1) {
    g1 =
      h5::make_strong_ptr<rp::interrupt_pin>(&memory, hal::pin<1>, do_nothing);
  }
  return g1;
}

// TODO: add timer
sptr<hal::timer> timed_interrupt();

opt<rp::pwm_slice_runtime> slice;
opt<rp::pwm_pin> pwm_pin;

sptr<hal::v5::pwm16_channel> pwm_channel()
{
  (void)pwm_frequency();  // Need to initalize this first
  if (not pwm_pin) {
    pwm_pin = h5::make_strong_ptr<rp::pwm_pin>(
      &memory, 5, hal::rp::pwm_pin_configuration{}, hal::unsafe{});
  }
  return pwm_pin;
}

sptr<hal::v5::pwm_group_manager> pwm_frequency()
{
  if (not slice) {
    slice = h5::make_strong_ptr<rp::pwm_slice<2>>(&memory, hal::channel<2>);
  }
  return slice;
}

opt<rp::spi> spi0;
sptr<hal::spi_channel> spi()
{
  if (not spi0) {
    spi0 = h5::make_strong_ptr<rp::spi>(
      &memory, hal::pin<23>, hal::pin<20>, hal::pin<22>, hal::pin<25>);
  }
  return spi0;
}

}  // namespace resources
