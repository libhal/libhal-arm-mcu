#pragma once

#include <libhal/spi.hpp>

namespace hal::rp::inline v1 {
/*
RP chips suppport 16 bit transfers. It may be worthwhile to add an option
to transfer 16 bits as a time.
*/
struct spi final : public hal::spi_channel
{
  // todo constrain later
  spi(u8 bus, u8 tx, u8 rx, u8 sck, u8 cs, spi::settings const&);
  ~spi() override;

private:
  void driver_configure(settings const&) override;
  u32 driver_clock_rate() override;
  void driver_chip_select(bool p_select) override;

  void driver_transfer(std::span<byte const> out,
                       std::span<byte> in,
                       byte) override;

  u8 m_bus, m_tx, m_rx, m_sck, m_cs;
};

}  // namespace hal::rp::inline v1
