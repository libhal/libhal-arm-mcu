#include <concepts>

#include <libhal-lpc40/clock.hpp>
#include <libhal-lpc40/dma.hpp>
#include <libhal-lpc40/pin.hpp>
#include <libhal-lpc40/stream_dac.hpp>
#include <libhal/io_waiter.hpp>
#include <libhal/stream_dac.hpp>

#include "dac_reg.hpp"

namespace hal::lpc40 {

namespace {
void setup_stream_dac()
{
  hal::lpc40::pin(0, 26).function(0b010).dac(true);

  // Double buffering enabled (1)
  // Count enable (2)
  // DMA enable (3)
  dac_reg->control = (1 << 1) | (1 << 2) | (1 << 3);
}

template<std::unsigned_integral T>
void dac_dma_write(hal::io_waiter& p_waiter,
                   typename hal::stream_dac<T>::samples const& p_samples)
{
  // Setup sampling frequency
  auto const input_clock =
    hal::lpc40::get_frequency(hal::lpc40::peripheral::dac);
  auto const clock_count_value = input_clock / p_samples.sample_rate;
  dac_reg->count_value = clock_count_value;

  auto data_remaining = p_samples.data;

  while (not data_remaining.empty()) {
    bool finished = false;
    auto const transfer_amount =
      std::min(data_remaining.size(), dma_max_transfer_size);

    if constexpr (std::is_same_v<std::uint8_t, T>) {
      hal::lpc40::setup_dma_transfer(
        dma{
          .source = data_remaining.data(),
          .destination = &dac_reg->conversion_register.parts[1],
          .length = transfer_amount,
          .source_increment = true,
          .destination_increment = false,
          .transfer_type = dma_transfer_type::memory_to_peripheral,
          .source_transfer_width = dma_transfer_width::bit_8,
          .destination_transfer_width = dma_transfer_width::bit_8,
          .source_peripheral = dma_peripheral::memory_or_timer0_match0,
          .destination_peripheral = dma_peripheral::dac,
        },
        [&p_waiter, &finished]() {
          finished = true;
          p_waiter.resume();
        });
    } else if (std::is_same_v<std::uint16_t, T>) {
      hal::lpc40::setup_dma_transfer(
        dma{
          .source = data_remaining.data(),
          .destination = &dac_reg->conversion_register.whole,
          .length = transfer_amount,
          .source_increment = true,
          .destination_increment = false,
          .transfer_type = dma_transfer_type::memory_to_peripheral,
          .source_transfer_width = dma_transfer_width::bit_16,
          .destination_transfer_width = dma_transfer_width::bit_16,
          .source_peripheral = dma_peripheral::memory_or_timer0_match0,
          .destination_peripheral = dma_peripheral::dac,
        },
        [&p_waiter, &finished]() {
          finished = true;
          p_waiter.resume();
        });
    }

    while (not finished) {
      p_waiter.wait();
    }

    // Move data forward by the amount of data that was transferred
    data_remaining = data_remaining.subspan(transfer_amount);
  }
}
}  // namespace

stream_dac_u8::stream_dac_u8(hal::io_waiter& p_waiter)
  : m_waiter(&p_waiter)
{
  setup_stream_dac();
}

void stream_dac_u8::driver_write(hal::stream_dac_u8::samples const& p_samples)
{
  dac_dma_write<std::uint8_t>(*m_waiter, p_samples);
}

stream_dac_u16::stream_dac_u16(hal::io_waiter& p_waiter)
  : m_waiter(&p_waiter)
{
  setup_stream_dac();
}

void stream_dac_u16::driver_write(hal::stream_dac_u16::samples const& p_samples)
{
  dac_dma_write<std::uint16_t>(*m_waiter, p_samples);
}
}  // namespace hal::lpc40
