#include "libhal-arm-mcu/rp/serial.hpp"

#include <hardware/gpio.h>
#include <hardware/structs/io_bank0.h>
#include <hardware/uart.h>
#include <libhal/error.hpp>
#include <libhal/units.hpp>
#include <pico/stdio.h>
#include <pico/time.h>

namespace {
auto get_uart(hal::u8 bus)
{
  switch (bus) {
    case 0:
      return uart0;
    case 1:
      return uart1;
    default:
      hal::safe_throw(hal::io_error(nullptr));
  }
}
}  // namespace

namespace hal::rp::inline v4 {
stdio_serial::stdio_serial()
{
  stdio_init_all();
}

void stdio_serial::driver_configure(settings const&)
{
}

void stdio_serial::driver_flush()
{
  stdio_flush();
}

serial::write_t stdio_serial::driver_write(std::span<byte const> in)
{
  int write_length = stdio_put_string(reinterpret_cast<char const*>(in.data()),
                                      static_cast<int>(in.size_bytes()),
                                      false,
                                      false);
  return write_t{ in.subspan(0, write_length) };
}

serial::read_t stdio_serial::driver_read(std::span<byte> output)
{
  // time in microseconds
  auto now = get_absolute_time();
  // 500 microseconds is 0.5 ms, which is probably reasonable
  int len = stdio_get_until(reinterpret_cast<char*>(output.data()),
                            static_cast<int>(output.size_bytes()),
                            now + 500);
  return read_t{ .data = output.subspan(0, len),
                 .available = 0,
                 .capacity = 1 };
}

uart::uart(u8 bus, u8 tx, u8 rx, settings const& options)  // NOLINT
  : m_tx(tx)
  , m_rx(rx)
  , m_bus(bus)
{

  auto func = (bus == 0) ? GPIO_FUNC_UART : GPIO_FUNC_UART_AUX;

  driver_configure(options);
  gpio_set_function(tx, func);
  gpio_set_function(rx, func);
}

uart::~uart()
{
  gpio_deinit(m_tx);
  gpio_deinit(m_rx);
  uart_deinit(get_uart(m_bus));
}

void uart::driver_configure(settings const& options)
{
  uint stop = 1;
  switch (options.stop) {
    case serial::settings::stop_bits::one:
      stop = 1;
      break;
    case serial::settings::stop_bits::two:
      stop = 2;
      break;
  }
  uart_parity_t parity = UART_PARITY_NONE;
  switch (options.parity) {
    case serial::settings::parity::none:
      parity = UART_PARITY_NONE;
      break;
    case serial::settings::parity::odd:
      parity = UART_PARITY_ODD;
      break;
    case serial::settings::parity::even:
      parity = UART_PARITY_EVEN;
      break;
    case serial::settings::parity::forced1:
    case serial::settings::parity::forced0:
      hal::safe_throw(operation_not_supported(this));
  }

  uart_init(get_uart(m_bus), uint(options.baud_rate));
  uart_set_format(get_uart(m_bus), 8, stop, parity);
}

serial::write_t uart::driver_write(std::span<byte const> in)
{
  uart_write_blocking(get_uart(m_bus), in.data(), in.size_bytes());
  // always writes everything
  return { in };
}

serial::read_t uart::driver_read(std::span<byte> out)
{
  uart_read_blocking(get_uart(m_bus), out.data(), out.size_bytes());
  return { .data = out,
           .available = uart_is_readable(get_uart(m_bus)),
           .capacity = 32 };
}

}  // namespace hal::rp::inline v4
