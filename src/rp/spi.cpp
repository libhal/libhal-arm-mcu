#include "libhal-arm-mcu/rp/spi.hpp"

#include <algorithm>
#include <hardware/gpio.h>
#include <hardware/spi.h>
#include <libhal/error.hpp>

namespace {
auto spi_bus(hal::u8 busnum)
{
  switch (busnum) {
    case 0:
      return spi0;
    case 1:
      return spi1;
    default:
      hal::safe_throw(hal::io_error(nullptr));
  }
}
}  // namespace
namespace hal::rp::inline v1 {
// TODO shuddup clang-tidy I'll fix it later
spi::spi(u8 bus, u8 tx, u8 rx, u8 sck, u8 cs, spi::settings const& s)  // NOLINT
  : m_bus(bus)
  , m_tx(tx)
  , m_rx(rx)
  , m_sck(sck)
  , m_cs(cs)
{
  gpio_set_function(tx, GPIO_FUNC_SPI);
  gpio_set_function(rx, GPIO_FUNC_SPI);
  gpio_set_function(sck, GPIO_FUNC_SPI);
  gpio_init(cs);
  gpio_set_dir(cs, GPIO_OUT);
  gpio_put(cs, true);
  driver_configure(s);
}

void spi::driver_configure(spi::settings const& s)
{
  spi_cpol_t polarity = SPI_CPOL_0;
  spi_cpha_t phase = SPI_CPHA_0;
  switch (s.bus_mode) {
    case spi_channel::mode::m0:
      polarity = SPI_CPOL_0;
      phase = SPI_CPHA_0;
      break;
    case spi_channel::mode::m1:
      polarity = SPI_CPOL_0;
      phase = SPI_CPHA_1;
      break;
    case spi_channel::mode::m2:
      polarity = SPI_CPOL_1;
      phase = SPI_CPHA_0;
      break;
    case spi_channel::mode::m3:
      polarity = SPI_CPOL_1;
      phase = SPI_CPHA_1;
      break;
    default:
      break;
  }
  spi_init(spi_bus(m_bus), s.clock_rate);
  spi_set_format(spi_bus(m_bus), 8, polarity, phase, SPI_MSB_FIRST);
}

u32 spi::driver_clock_rate()
{
  return spi_get_baudrate(spi_bus(m_bus));
}

void spi::driver_chip_select(bool sel)
{
  // The examples in
  // https://github.com/raspberrypi/pico-examples/tree/master/spi seem to add a
  // mysterious nop sequence before and after the assertion. Apparently the
  // engineer who wrote this 15 years ago doesn't remember
  // (https://forums.raspberrypi.com/viewtopic.php?t=332078) so its necessity is
  // really unclear. In any case, there's a nonzero chance just the overhead of
  // calling a function will mask this bug. If SPI seems a bit flaky, then
  // consider adding asm nops here.
  gpio_put(m_cs, !sel);
}

void spi::driver_transfer(std::span<byte const> out,
                          std::span<byte> in,
                          byte filler)
{
  auto out_size = out.size_bytes();
  auto in_size = in.size_bytes();
  auto size = std::min(out_size, in_size);
  spi_write_read_blocking(spi_bus(m_bus), out.data(), in.data(), size);
  if (out_size > in_size) {
    spi_write_blocking(
      spi_bus(m_bus), out.data() + in_size, out_size - in_size);
  } else if (in_size > out_size) {
    spi_read_blocking(
      spi_bus(m_bus), filler, in.data() + out_size, in_size - out_size);
  }
}

spi::~spi()
{
  spi_deinit(spi_bus(m_bus));
  gpio_deinit(m_tx);
  gpio_deinit(m_rx);
  gpio_deinit(m_sck);
  gpio_deinit(m_cs);
}

}  // namespace hal::rp::inline v1
