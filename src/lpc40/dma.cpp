#include <cstdint>

#include <array>
#include <bit>
#include <mutex>

#include <libhal-arm-mcu/lpc40/constants.hpp>
#include <libhal-arm-mcu/lpc40/dma.hpp>
#include <libhal-arm-mcu/lpc40/interrupt.hpp>
#include <libhal-arm-mcu/lpc40/power.hpp>
#include <libhal-armcortex/interrupt.hpp>
#include <libhal-util/atomic_spin_lock.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/enum.hpp>
#include <libhal/functional.hpp>
#include <libhal/lock.hpp>
#include <libhal/units.hpp>

namespace hal::lpc40 {
namespace {
struct dma_channel
{
  std::uint32_t volatile source_address;
  std::uint32_t volatile destination_address;
  std::uint32_t volatile linked_list_item;
  std::uint32_t volatile control;
  std::uint32_t volatile config;
};

struct gpdma
{
  std::uint32_t volatile interrupt_status;
  std::uint32_t volatile interrupt_terminal_count_status;
  std::uint32_t volatile interrupt_terminal_count_clear;
  std::uint32_t volatile interrupt_error_status;
  std::uint32_t volatile interrupt_error_clear;
  std::uint32_t volatile raw_interrupt_terminal_count_status;
  std::uint32_t volatile raw_interrupt_error_status;
  std::uint32_t volatile enabled_channels;
  std::uint32_t volatile SOFTBREQ;
  std::uint32_t volatile SOFTSREQ;
  std::uint32_t volatile SOFTLBREQ;
  std::uint32_t volatile SOFTLSREQ;
  std::uint32_t volatile config;
  std::uint32_t volatile sync;
};

namespace dma_control {
constexpr auto transfer_size = hal::bit_mask::from<0, 11>();
constexpr auto source_burst_size = hal::bit_mask::from<12, 14>();
constexpr auto destination_burst_size = hal::bit_mask::from<15, 17>();
constexpr auto source_transfer_width = hal::bit_mask::from<18, 20>();
constexpr auto destination_transfer_width = hal::bit_mask::from<21, 23>();
constexpr auto source_increment = hal::bit_mask::from<26>();
constexpr auto destination_increment = hal::bit_mask::from<27>();
constexpr auto enable_terminal_count_interrupt = hal::bit_mask::from<31>();
}  // namespace dma_control

namespace dma_config {
// config masks
constexpr auto enable = hal::bit_mask::from<0>();
constexpr auto source_peripheral = hal::bit_mask::from<5, 1>();
constexpr auto destination_peripheral = hal::bit_mask::from<10, 6>();
constexpr auto transfer_type = hal::bit_mask::from<13, 11>();
constexpr auto terminal_count_interrupt_mask = hal::bit_mask::from<15>();
}  // namespace dma_config

// NOLINTNEXTLINE(performance-no-int-to-ptr)
auto* dma_reg = reinterpret_cast<gpdma*>(dma_reg_address);

constexpr inline std::uintptr_t dma_channel_offset(unsigned p_channel)
{
  return 0x100 + (p_channel * 0x20);
}

std::array<dma_channel*, dma_channel_count> dma_channel_reg{
  // NOLINTNEXTLINE(performance-no-int-to-ptr)
  reinterpret_cast<dma_channel*>(dma_reg_address + dma_channel_offset(0)),
  // NOLINTNEXTLINE(performance-no-int-to-ptr)
  reinterpret_cast<dma_channel*>(dma_reg_address + dma_channel_offset(1)),
  // NOLINTNEXTLINE(performance-no-int-to-ptr)
  reinterpret_cast<dma_channel*>(dma_reg_address + dma_channel_offset(2)),
  // NOLINTNEXTLINE(performance-no-int-to-ptr)
  reinterpret_cast<dma_channel*>(dma_reg_address + dma_channel_offset(3)),
  // NOLINTNEXTLINE(performance-no-int-to-ptr)
  reinterpret_cast<dma_channel*>(dma_reg_address + dma_channel_offset(4)),
  // NOLINTNEXTLINE(performance-no-int-to-ptr)
  reinterpret_cast<dma_channel*>(dma_reg_address + dma_channel_offset(5)),
  // NOLINTNEXTLINE(performance-no-int-to-ptr)
  reinterpret_cast<dma_channel*>(dma_reg_address + dma_channel_offset(6)),
  // NOLINTNEXTLINE(performance-no-int-to-ptr)
  reinterpret_cast<dma_channel*>(dma_reg_address + dma_channel_offset(7)),
};

std::array<hal::callback<void(void)>, dma_channel_count> dma_callbacks{
  hal::cortex_m::default_interrupt_handler,
  hal::cortex_m::default_interrupt_handler,
  hal::cortex_m::default_interrupt_handler,
  hal::cortex_m::default_interrupt_handler,
  hal::cortex_m::default_interrupt_handler,
  hal::cortex_m::default_interrupt_handler,
  hal::cortex_m::default_interrupt_handler,
  hal::cortex_m::default_interrupt_handler,
};

void handle_dma_interrupt() noexcept
{
  // The zero count from the LSB tells you where the least significant 1 is
  // located. This allows the handled DMA interrupt callback to start at 0 and
  // end at the last bit.
  auto const status = std::countr_zero(dma_reg->interrupt_status);
  auto const clear_mask = 1 << status;

  dma_reg->interrupt_terminal_count_clear = clear_mask;
  dma_reg->interrupt_error_clear = clear_mask;

  // Call this channel's callback
  dma_callbacks[status]();
}

void initialize_dma()
{
  if (is_on(peripheral::gpdma)) {
    return;
  }

  power_on(peripheral::gpdma);
  initialize_interrupts();
  hal::cortex_m::enable_interrupt(irq::dma, handle_dma_interrupt);
  // Enable DMA & use default AHB endianness
  dma_reg->config = 1;
}

hal::atomic_spin_lock dma_spin_lock;
hal::basic_lock* dma_lock = &dma_spin_lock;
}  // namespace

void set_dma_lock(hal::basic_lock& p_lock)
{
  dma_lock = &p_lock;
}

void setup_dma_transfer(
  dma const& p_configuration,
  hal::callback<void(void)> p_interrupt_callback)  // NOLINT
{
  auto const config_value =
    hal::bit_value()
      .insert<dma_config::source_peripheral>(
        hal::value(p_configuration.source_peripheral))
      .insert<dma_config::destination_peripheral>(
        hal::value(p_configuration.destination_peripheral))
      .set<dma_config::enable>()
      .insert<dma_config::transfer_type>(
        hal::value(p_configuration.transfer_type))
      .set<dma_config::terminal_count_interrupt_mask>()
      .to<std::uint32_t>();

  auto const control_value =
    hal::bit_value()
      .insert<dma_control::transfer_size>(p_configuration.length)
      .insert<dma_control::source_burst_size>(
        hal::value(p_configuration.source_burst_size))
      .insert<dma_control::destination_burst_size>(
        hal::value(p_configuration.destination_transfer_width))
      .insert<dma_control::source_transfer_width>(
        hal::value(p_configuration.source_transfer_width))
      .insert<dma_control::destination_transfer_width>(
        hal::value(p_configuration.destination_transfer_width))
      .insert<dma_control::source_increment>(p_configuration.source_increment)
      .insert<dma_control::destination_increment>(
        p_configuration.destination_increment)
      .set<dma_control::enable_terminal_count_interrupt>()
      .to<std::uint32_t>();

  std::lock_guard take_dma_lock(*dma_lock);

  initialize_dma();

  // Busy wait until a channel is available
  while (true) {
    // Count the number of 1s until you reach a zero. That zero will be the
    // available channel. If that zero is 8, then all 8 channels are currently
    // in use and cannot service this request.
    auto const available_channel = std::countr_one(dma_reg->enabled_channels);

    if (available_channel < 8) {
      // Copy callback to the callbacks
      dma_callbacks[available_channel] = p_interrupt_callback;

      // Clear previous interrupts on this channel, if there were any
      dma_reg->interrupt_terminal_count_clear = 1 << available_channel;
      dma_reg->interrupt_error_clear = 1 << available_channel;

      auto* dma_channel = dma_channel_reg[available_channel];

      dma_channel->source_address =
        reinterpret_cast<std::uintptr_t>(p_configuration.source);
      dma_channel->destination_address =
        reinterpret_cast<std::uintptr_t>(p_configuration.destination);
      dma_channel->control = control_value;
      // This will start the dma transfer
      dma_channel->config = config_value;
      break;
    }
  }
}
}  // namespace hal::lpc40
