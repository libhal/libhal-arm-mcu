#include "libhal-arm-mcu/rp/serial.hpp"

#include <pico/stdio.h>
#include <pico/time.h>

namespace hal::rp {
v1::stdio_serial::stdio_serial()
{
  stdio_init_all();
}

void v1::stdio_serial::driver_configure(settings const&)
{
}

void v1::stdio_serial::driver_flush()
{
  stdio_flush();
}

serial::write_t v1::stdio_serial::driver_write(std::span<byte const> in)
{
  int write_length = stdio_put_string(reinterpret_cast<char const*>(in.data()),
                                      static_cast<int>(in.size_bytes()),
                                      false,
                                      false);
  return write_t{ in.subspan(0, write_length) };
}

serial::read_t v1::stdio_serial::driver_read(std::span<byte> output)
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

}  // namespace hal::rp::generic
