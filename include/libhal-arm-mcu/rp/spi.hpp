#pragma once

#include "libhal-arm-mcu/rp/rp.hpp"
#include <libhal/spi.hpp>

namespace hal::rp::inline v1 {
/*
RP chips suppport 16 bit transfers. It may be worthwhile to add an option
to transfer 16 bits as a time.
*/
struct spi final : public hal::spi_channel
{
  // Yes the spi pins can be completely seperate pins
  spi(pin_param auto copi,
      pin_param auto cipo,
      pin_param auto sck,
      pin_param auto cs,
      spi::settings const& options)
    : spi(bus_from_tx_pin(copi()), copi(), cipo(), sck(), cs(), options)
  {
    // CS is a normal chip select
    static_assert(cipo() % 4 == 0, "SPI RX pin is invalid");
    static_assert(cs() % 4 == 1, "SPI CS pin is invalid");
    static_assert(sck() % 4 == 2, "SPI SCK pin is invalid");
  }
  ~spi() override;

private:
  spi(u8 bus, u8 copi, u8 cipo, u8 sck, u8 cs, spi::settings const&);
  void driver_configure(settings const&) override;
  u32 driver_clock_rate() override;
  void driver_chip_select(bool p_select) override;

  void driver_transfer(std::span<byte const> out,
                       std::span<byte> in,
                       byte) override;

  u8 m_bus, m_tx, m_rx, m_sck, m_cs;

  constexpr u8 bus_from_tx_pin(u8);
};

}  // namespace hal::rp::inline v1
