#pragma once

#include "rp.hpp"
#include "time.hpp"
#include <libhal/adc.hpp>
#include <libhal/units.hpp>
#include <span>

namespace hal::rp {

inline namespace v4 {
struct adc final : public hal::adc
{
  adc(pin_param auto pin)
    : adc(pin())
  {
    static_assert(internal::pin_max != 30 || (pin() >= 26 && pin() < 30),
                  "ADC pin is invalid!");
    static_assert(internal::pin_max != 48 || (pin() >= 40 && pin() < 48),
                  "ADC pin is invalid!");
  }
  adc(adc&&) = delete;
  ~adc() override;

private:
  adc(u8 pin);
  /*Because the rp chips only have one ADC that's muxed
  across different pins, we just initialize and mux the ADC
  every time we want to read. */
  float driver_read() override;
  u8 m_pin;
};
}  // namespace v4

namespace v5 {
// The ADC is 12 bit
struct adc16 final : public hal::adc16
{
  adc16(pin_param auto pin)
    : adc16(pin())
  {
    static_assert(internal::pin_max != 30 || (pin() >= 26 && pin() < 30),
                  "ADC pin is invalid!");
    static_assert(internal::pin_max != 48 || (pin() >= 40 && pin() < 48),
                  "ADC pin is invalid!");
  }
  ~adc16() override;

private:
  adc16(u8 gpio);
  /*Because the rp chips only have one ADC that's muxed
  across different pins, we just initialize and mux the ADC
  every time we want to read. */
  u16 driver_read() override;
  u8 m_pin;
};
}  // namespace v5

namespace nonstandard {
struct adc16_pack
{
  struct read_session;

  template<pin_param... Pins>
  adc16_pack(Pins... ps)
    : adc16_pack((to_mask(ps) | ...))
  {
  }
  adc16_pack(adc16_pack const&) = delete;
  adc16_pack(adc16_pack&&) = delete;

  void read_many_now(std::span<u16>);
  // Uses up 1 DMA channel
  read_session async();

private:
  adc16_pack(u8 pin_mask);
  u8 to_mask(pin_param auto pin)
  {
    if constexpr (internal::pin_max == 30) {
      static_assert(pin() >= 26 && pin() < 30);
      return 1 << (pin() - 26);
    } else if constexpr (internal::pin_max == 48) {
      static_assert(pin() >= 40 && pin() < 48);
      return 1 << (pin() - 40);
    }
  }
  u8 m_read_size, m_first_pin;
};

struct adc16_pack::read_session
{
  struct promise;
  promise read(std::span<u16>);
  ~read_session();
  read_session(read_session const&) = delete;
  read_session(read_session&&) = delete;

  struct promise
  {
    microseconds poll();
    promise(promise const&) = default;

  private:
    friend read_session;
    promise(u8 dma, u8 first_pin)  // NOLINT
      : m_dma(dma)
      , m_first_pin(first_pin)
    {
    }
    u8 m_dma, m_first_pin;
  };

private:
  friend adc16_pack;
  read_session(u8 dma, u8 read_size, u8 first_pin)  // NOLINT
    : m_dma(dma)
    , m_read_size(read_size)
    , m_first_pin(first_pin)
  {
  }
  u8 m_dma, m_read_size, m_first_pin;
};

}  // namespace nonstandard

}  // namespace hal::rp
