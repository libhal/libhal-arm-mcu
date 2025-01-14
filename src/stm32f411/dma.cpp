// Copyright 2024 Khalil Estell
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <libhal/units.hpp>
#include <mutex>
#include <span>

#include <libhal-arm-mcu/stm32f411/constants.hpp>
#include <libhal-arm-mcu/stm32f411/dma.hpp>
#include <libhal-arm-mcu/stm32f411/interrupt.hpp>
#include <libhal-util/atomic_spin_lock.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/enum.hpp>
#include <libhal/error.hpp>

#include "dma.hpp"
#include "power.hpp"

namespace hal::stm32f411 {
namespace {
std::array<hal::callback<void()>, 16> dma_callbacks{};

template<std::size_t dma_index>
void handle_dma_interrupt() noexcept
{
  dma_callbacks[dma_index]();
  /// DMA transfer masks are all on bits 5,11,21 and 27
  constexpr std::array<bit_mask, 4> const transfer_complete_mask = {
    bit_mask::from<5>(),
    bit_mask::from<11>(),
    bit_mask::from<21>(),
    bit_mask::from<27>(),
  };

  dma_config_t* dma_reg;
  if constexpr (dma_index / 8) {
    dma_reg = get_dma_reg(peripheral::dma1);
  } else {
    dma_reg = get_dma_reg(peripheral::dma2);
  }

  constexpr auto stream_index = dma_index % 8;
  constexpr auto mask_index = stream_index % 4;
  constexpr auto complete_mask = transfer_complete_mask[mask_index];

  if constexpr (stream_index < 4) {
    bit_modify(dma_reg->interupt_status_clear_low).clear(complete_mask);
  } else {
    bit_modify(dma_reg->interupt_status_clear_high).clear(complete_mask);
  }
}

void initialize_dma(peripheral p_dma)
{
  power dma_power(p_dma);
  if (dma_power.is_on()) {
    return;
  }

  dma_power.on();

  initialize_interrupts();

  hal::cortex_m::enable_interrupt(irq::dma1_channel0, handle_dma_interrupt<0>);
  hal::cortex_m::enable_interrupt(irq::dma1_channel1, handle_dma_interrupt<1>);
  hal::cortex_m::enable_interrupt(irq::dma1_channel2, handle_dma_interrupt<2>);
  hal::cortex_m::enable_interrupt(irq::dma1_channel3, handle_dma_interrupt<3>);
  hal::cortex_m::enable_interrupt(irq::dma1_channel4, handle_dma_interrupt<4>);
  hal::cortex_m::enable_interrupt(irq::dma1_channel5, handle_dma_interrupt<5>);
  hal::cortex_m::enable_interrupt(irq::dma1_channel6, handle_dma_interrupt<6>);
  hal::cortex_m::enable_interrupt(irq::dma1_channel7, handle_dma_interrupt<7>);
  hal::cortex_m::enable_interrupt(irq::dma2_channel0, handle_dma_interrupt<8>);
  hal::cortex_m::enable_interrupt(irq::dma2_channel1, handle_dma_interrupt<9>);
  hal::cortex_m::enable_interrupt(irq::dma2_channel2, handle_dma_interrupt<10>);
  hal::cortex_m::enable_interrupt(irq::dma2_channel3, handle_dma_interrupt<11>);
  hal::cortex_m::enable_interrupt(irq::dma2_channel4, handle_dma_interrupt<12>);
  hal::cortex_m::enable_interrupt(irq::dma2_channel5, handle_dma_interrupt<13>);
  hal::cortex_m::enable_interrupt(irq::dma2_channel6, handle_dma_interrupt<14>);
  hal::cortex_m::enable_interrupt(irq::dma2_channel7, handle_dma_interrupt<15>);
}

hal::atomic_spin_lock dma_spin_lock;
hal::basic_lock* dma_lock = &dma_spin_lock;

// This is here for cost sizing
[[maybe_unused]] constexpr auto dma_memory_usage_size =
  sizeof(dma_callbacks) + sizeof(dma_spin_lock) + sizeof(dma_lock);  // NOLINT
}  // namespace

void set_dma_lock(hal::basic_lock& p_lock)
{
  dma_lock = &p_lock;
}

dma_channel_stream_t set_available_stream(
  peripheral p_dma,
  std::span<dma_channel_stream_t> const& p_possible_streams)
{
  auto dma_addr = get_dma_reg(p_dma);
  while (true) {
    for (auto stream_candidate : p_possible_streams) {
      if (bit_extract<dma_stream_config::channel_select>(
            dma_addr->stream[stream_candidate.stream].configure) ==
          stream_candidate.channel) {
        return stream_candidate;
      } else if (bit_extract<dma_stream_config::stream_enable>(
                   dma_addr->stream[stream_candidate.stream].configure) == 0) {
        bit_modify(dma_addr->stream[stream_candidate.stream].configure)
          .insert(dma_stream_config::channel_select, stream_candidate.channel);
        return stream_candidate;
      }
    }
  }
}

dma_channel_stream_t setup_dma_transfer(
  peripheral p_dma,
  std::span<dma_channel_stream_t> const& p_possible_streams,
  dma_settings_t const& p_configuration,
  hal::callback<void()> p_interrupt_callback)
{
  auto dma_addr = get_dma_reg(p_dma);

  std::lock_guard take_dma_lock(*dma_lock);
  initialize_dma(p_dma);

  auto selected_config = set_available_stream(p_dma, p_possible_streams);
  auto& stream = dma_addr->stream[selected_config.stream];
  switch (p_configuration.transfer_type) {
    case dma_transfer_type::peripheral_to_memory:
      stream.peripheral_address =
        std::bit_cast<std::uintptr_t>(p_configuration.source);
      stream.memory_0_address =
        std::bit_cast<std::uintptr_t>(p_configuration.destination);
      break;
    case dma_transfer_type::memory_to_peripheral:
      stream.peripheral_address =
        std::bit_cast<std::uintptr_t>(p_configuration.destination);
      stream.memory_0_address =
        std::bit_cast<std::uintptr_t>(p_configuration.source);
      break;
    default:
      stream.peripheral_address =
        std::bit_cast<std::uintptr_t>(p_configuration.source);
      stream.memory_0_address =
        std::bit_cast<std::uintptr_t>(p_configuration.destination);
  }

  stream.transfer_count = p_configuration.transfer_length;

  bit_modify(dma_addr->stream[selected_config.stream].configure)
    .insert(dma_stream_config::peripheral_flow_controller,
            value(p_configuration.flow_controller))
    .insert(dma_stream_config::data_transfer_direction,
            value(p_configuration.transfer_type))
    .insert(dma_stream_config::circular_mode_enable,
            p_configuration.circular_mode)
    .insert(dma_stream_config::peripheral_increment_mode,
            p_configuration.peripheral_address_increment)
    .insert(dma_stream_config::memory_increment_mode,
            p_configuration.memory_address_increment)
    .insert(dma_stream_config::peripheral_data_size,
            value(p_configuration.peripheral_data_size))
    .insert(dma_stream_config::memory_data_size,
            value(p_configuration.memory_data_size))
    .insert(dma_stream_config::priority_level,
            value(p_configuration.priority_level))
    .insert(dma_stream_config::double_buffer_mode,
            p_configuration.double_buffer_mode)
    .insert(dma_stream_config::current_target, p_configuration.current_target)
    .insert(dma_stream_config::peripheral_burst_transfer,
            value(p_configuration.peripheral_burst_size))
    .insert(dma_stream_config::memory_burst_transfer,
            value(p_configuration.memory_burst_size))
    .insert(dma_stream_config::channel_select, selected_config.channel);

  bit_modify(dma_addr->stream[selected_config.stream].fifo_control)
    .clear(dma_fifo_config::direct_mode_disable);

  uint8_t callback_entry = 0;
  if (p_dma == peripheral::dma1) {
    callback_entry = selected_config.stream;
  } else {
    callback_entry = selected_config.stream + 8;
  }

  dma_callbacks[callback_entry] = std::move(p_interrupt_callback);

  constexpr std::array<bit_mask, 4> const dma_intr_mask = {
    bit_mask::from<5, 0>(),
    bit_mask::from<11, 6>(),
    bit_mask::from<21, 16>(),
    bit_mask::from<27, 22>(),
  };

  auto complete_mask = dma_intr_mask[selected_config.stream % 4];
  constexpr uint8_t clear_data = 0x1D;
  if (selected_config.stream < 4) {
    bit_modify(dma_addr->interupt_status_clear_low)
      .insert(complete_mask, clear_data);
  } else {
    bit_modify(dma_addr->interupt_status_clear_high)
      .insert(complete_mask, clear_data);
  }

  bit_modify(dma_addr->stream[selected_config.stream].configure)
    .set(dma_stream_config::stream_enable);
  return selected_config;
};

void set_dma_memory_transfer(peripheral p_dma,
                             std::span<dma_channel_stream_t> p_stream_channel,
                             std::span<hal::byte> p_source,
                             std::span<byte> p_destination)
{
  dma_settings_t dma_setting = {
    .source = &p_source,
    .destination = &p_destination,

    .transfer_length = p_source.size(),
    .flow_controller = dma_flow_controller::dma_controls_flow,
    .transfer_type = dma_transfer_type::memory_to_memory,
    .circular_mode = false,
    .peripheral_address_increment = true,
    .memory_address_increment = true,
    .peripheral_data_size = dma_transfer_size::byte,
    .memory_data_size = dma_transfer_size::byte,
    .priority_level = dma_priority_level::high,
    .double_buffer_mode = false,
    .current_target = false,
    .peripheral_burst_size = dma_burst_size::single_transfer,
    .memory_burst_size = dma_burst_size::single_transfer
  };
  setup_dma_transfer(p_dma, p_stream_channel, dma_setting, []() {});
}
}  // namespace hal::stm32f411
