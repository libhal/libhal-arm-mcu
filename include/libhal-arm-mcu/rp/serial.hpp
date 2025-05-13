#pragma once

#include <libhal/serial.hpp>

namespace hal::rp::generic {
inline int fake_serial = 0;
inline namespace v1 {
/*
The RP series chips use a native ROM USB bootloader,
meaning most setups use USB directly, necessitating
using TinyUSB serial instead of a serial-USB converter
This means that DTR has no effect, and also web serial behaves
very weirdly

The stdio serial actually wraps stdio, and may or may not
buffer or be asynchronous. Do not depend on the asynchronality
of this class as it is not guaranteed.
*/
struct stdio_serial final : public hal::serial
{
  stdio_serial();
  ~stdio_serial() override = default;

private:
  // This function is a sham that does nothing
  void driver_configure(settings const&) override;
  write_t driver_write(std::span<byte const>) override;
  // The stdio interface doesn't actually provide us with how many
  // bytes are left. It's not too hard to implement an actual buffer
  // system, but honestly most people aren't going to use this anyways
  read_t driver_read(std::span<byte>) override;
  void driver_flush() override;
};

}  // namespace v1
}  // namespace hal::rp::generic
