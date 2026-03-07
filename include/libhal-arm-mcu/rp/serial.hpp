#pragma once

#include "rp.hpp"
#include <libhal/serial.hpp>

namespace hal::rp::inline v4 {
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
  stdio_serial(stdio_serial&&) = delete;

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

// RP chips support CTS and RTS, but libhal has no support for them currently
// This is not really intended for debug purposes, and as such is blocking
// and has no internal buffering.
struct uart final : public hal::serial
{

  uart(bus_param auto bus,
       pin_param auto tx,
       pin_param auto rx,
       settings const& options = {})
    : uart(bus(), tx(), rx(), options)
  {
    static_assert(bus() == 0 || bus() == 1, "Invalid UART bus selected!");
    static_assert(tx() % 4 == 0, "UART TX pin is invalid!");
    static_assert(rx() % 4 == 1, "UART TX pin is invalid!");
    static_assert(((tx() + 4) / 8) % 2 == bus(),
                  "UART TX pin and bus do not match!");
    static_assert(((rx() + 4) / 8) % 2 == bus(),
                  "UART RX pin and bus do not match!");
  }
  uart(uart&&) = delete;
  ~uart() override;

private:
  uart(u8 bus, u8 tx, u8 rx, settings const&);
  void driver_configure(settings const&) override;
  write_t driver_write(std::span<byte const>) override;
  // available bytes is always read as 0 or 1, since there's no actual way
  // to read number of bytes in the fifo
  read_t driver_read(std::span<byte>) override;
  void driver_flush() override;
  u8 m_tx, m_rx, m_bus;
};

}  // namespace hal::rp::inline v4
