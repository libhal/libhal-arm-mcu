#pragma once

#include <libhal/io_waiter.hpp>
#include <libhal/stream_dac.hpp>

namespace hal::lpc40 {
/**
 * @brief 16-bit stream dac implementation
 *
 * Operated using DMA. The DMA request number is unique to the dac and thus does
 * not conflict with any other driver.
 *
 * The lpc40 dac only supports up to 10-bit resolution so the 6 least
 * significant bits will be ignored.
 *
 * Note that there is only a single dac on the lpc40 series and thus only one
 * dac driver should be used in an application.
 */
class stream_dac_u16 final : public hal::stream_dac_u16
{
public:
  stream_dac_u16(hal::io_waiter& p_waiter = hal::polling_io_waiter());
  stream_dac_u16(stream_dac_u16 const& p_other) = delete;
  stream_dac_u16& operator=(stream_dac_u16 const& p_other) = delete;
  stream_dac_u16(stream_dac_u16&& p_other) noexcept = delete;
  stream_dac_u16& operator=(stream_dac_u16&& p_other) noexcept = delete;
  ~stream_dac_u16() override = default;

private:
  void driver_write(hal::stream_dac_u16::samples const& p_samples) override;

  hal::io_waiter* m_waiter;
};

/**
 * @brief 8-bit stream dac implementation
 *
 * Operated using DMA. The DMA request number is unique to the dac and thus does
 * not conflict with any other driver.
 *
 * The lpc40 supports a right-aligned 10-bit resolution dac making it easy to
 * DMA byte data to it by choosing the 2nd byte in the register. No additional
 * preprocessing is necessary by the driver to DMA data from a byte stream to
 * the DAC.
 *
 * Note that there is only a single dac on the lpc40 series and thus only one
 * dac driver should be used in an application.
 */
class stream_dac_u8 final : public hal::stream_dac_u8
{
public:
  stream_dac_u8(hal::io_waiter& p_waiter = hal::polling_io_waiter());
  stream_dac_u8(stream_dac_u8 const& p_other) = delete;
  stream_dac_u8& operator=(stream_dac_u8 const& p_other) = delete;
  stream_dac_u8(stream_dac_u8&& p_other) noexcept = delete;
  stream_dac_u8& operator=(stream_dac_u8&& p_other) noexcept = delete;
  ~stream_dac_u8() override = default;

private:
  void driver_write(hal::stream_dac_u8::samples const& p_samples) override;

  hal::io_waiter* m_waiter;
};
}  // namespace hal::lpc40
