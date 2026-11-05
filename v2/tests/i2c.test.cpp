// Copyright 2026 Khalil Estell and the libhal contributors
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

// compute_i2c_timing() and compute_stall_polls() are pure arithmetic, so they
// run on the host. Transactions run against plain memory standing in for the
// registers: status register 1 keeps the flags a test sets, so a wait for a set
// flag passes at once and a wait for any other flag finds the NACK.

#include <array>
#include <chrono>

#include <boost/ut.hpp>

import hal;
import hal.arm_mcu;

namespace {
constexpr hal::u32 sr1_start = 0x0001;
constexpr hal::u32 sr1_addr = 0x0002;
constexpr hal::u32 sr1_byte_finished = 0x0004;
constexpr hal::u32 sr1_tx_empty = 0x0080;
constexpr hal::u32 sr1_ack_failure = 0x0400;
constexpr hal::u32 cr1_stop = 0x0200;
constexpr hal::byte address = 0x50;
constexpr hal::byte ten_bit_address = 0b111'1011;

/// Plain memory laid out like the peripheral's registers, and a bus that uses
/// it, with status register 1 set to `p_sr1` once the bus is configured
struct fake_bus
{
  explicit fake_bus(hal::u32 p_sr1)
    : bus(registers.data())
  {
    using namespace mp_units::si::unit_symbols;
    bus.configure(hal::i2c_settings{}, 8 * MHz);
    sr1() = p_sr1;
  }

  hal::u32& cr1()
  {
    return registers[0];
  }

  hal::u32& data_reg()
  {
    return registers[4];
  }

  hal::u32& sr1()
  {
    return registers[5];
  }

  std::array<hal::u32, 16> registers{};
  hal::stm32_generic::i2c bus;
};

void i2c_transaction_test()
{
  using namespace boost::ut;

  static constexpr std::array<hal::byte, 3> three_bytes{ 0x11, 0x22, 0x33 };
  static constexpr std::array<hal::byte, 1> one_byte{ 0x11 };

  "an empty transaction leaves the peripheral untouched"_test = []() {
    // Setup
    fake_bus fake(0);
    auto const cr1 = fake.cr1();

    // Exercise
    fake.bus.transaction(address, {}, {});

    // Verify
    expect(that % cr1 == fake.cr1());
    expect(that % 0 == fake.data_reg());
  };

  "a NACK on the address throws no_such_device, and stops"_test = []() {
    // Setup
    fake_bus fake(sr1_start | sr1_ack_failure);
    hal::u32 thrown_address = 0;

    // Exercise
    try {
      fake.bus.transaction(address, { three_bytes }, {});
    } catch (hal::no_such_device const& p_error) {
      thrown_address = p_error.address;
    }

    // Verify
    expect(that % hal::u32{ address } == thrown_address);
    expect(that % 0 == (fake.sr1() & sr1_ack_failure));
    expect(that % cr1_stop == (fake.cr1() & cr1_stop));
  };

  "a NACK on a written byte throws broken_pipe, and stops"_test = []() {
    // Setup
    // The data register is empty, so the third byte is the one refused.
    fake_bus fake(sr1_start | sr1_addr | sr1_tx_empty | sr1_ack_failure);
    hal::u32 progress = 0;

    // Exercise
    try {
      fake.bus.transaction(address, { three_bytes }, {});
    } catch (hal::broken_pipe const& p_error) {
      progress = p_error.progress;
    }

    // Verify
    expect(that % 2 == progress);
    expect(that % cr1_stop == (fake.cr1() & cr1_stop));
  };

  "a NACK on the first written byte is still broken_pipe"_test = []() {
    // Setup
    fake_bus fake(sr1_start | sr1_addr | sr1_tx_empty | sr1_ack_failure);

    // Exercise + Verify
    expect(throws<hal::broken_pipe>(
      [&]() { fake.bus.transaction(address, { one_byte }, {}); }));
  };

  "a NACK on a 10-bit address's low byte throws no_such_device"_test = []() {
    // Setup
    fake_bus fake(sr1_start | sr1_addr | sr1_tx_empty | sr1_ack_failure);

    // Exercise + Verify
    expect(throws<hal::no_such_device>(
      [&]() { fake.bus.transaction(ten_bit_address, { one_byte }, {}); }));
  };

  "a NACK after a 10-bit address's low byte throws broken_pipe"_test = []() {
    // Setup
    fake_bus fake(sr1_start | sr1_addr | sr1_tx_empty | sr1_ack_failure);
    hal::u32 progress = 0;

    // Exercise
    try {
      fake.bus.transaction(ten_bit_address, { three_bytes }, {});
    } catch (hal::broken_pipe const& p_error) {
      progress = p_error.progress;
    }

    // Verify
    expect(that % 2 == progress);
  };

  "a NACK after a whole write counts every written byte"_test = []() {
    // Setup
    fake_bus fake(sr1_start | sr1_addr | sr1_byte_finished | sr1_tx_empty |
                  sr1_ack_failure);
    std::array<hal::byte, 5> buffer{};
    hal::u32 progress = 0;

    // Exercise
    try {
      fake.bus.transaction(address, { three_bytes }, { buffer });
    } catch (hal::broken_pipe const& p_error) {
      progress = p_error.progress;
    }

    // Verify
    expect(that % 3 == progress);
  };
}

void i2c_test()
{
  using namespace boost::ut;
  using namespace mp_units::si::unit_symbols;
  using namespace std::chrono_literals;
  using hal::stm32_generic::compute_i2c_timing;
  using hal::stm32_generic::compute_stall_polls;

  "standard mode divides the clock into equal high and low halves"_test = []() {
    // Exercise
    // 36 MHz / (2 * 100 kHz) = 180
    auto const timing = compute_i2c_timing(100 * kHz, 36 * MHz);

    // Verify
    expect(that % 36 == timing.frequency);
    expect(not timing.fast_mode);
    expect(that % 180 == timing.ccr);
    // 1000 ns at 36 MHz is 36 cycles, plus one.
    expect(that % 37 == timing.rise_time);
  };

  "the stall timeout is counted in polls of at least two cycles"_test = []() {
    // Exercise
    // 100 ms at 36 MHz is 3,600,000 cycles, and a poll takes at least two.
    auto const polls = compute_stall_polls(100ms, 36 * MHz);
    // 25 ms at 8 MHz is 200,000 cycles.
    auto const slow_polls = compute_stall_polls(25ms, 8 * MHz);

    // Verify
    expect(that % 1'800'000 == polls);
    expect(that % 100'000 == slow_polls);
  };

  "the stall poll count rounds up, so a wait is never shorter"_test = []() {
    // Exercise
    // 1 ns at 8 MHz is 0.008 cycles: less than one poll.
    auto const tiny_polls = compute_stall_polls(1ns, 8 * MHz);
    // 1,001 ns at 8 MHz is 8.008 cycles: 4.004 polls.
    auto const odd_polls = compute_stall_polls(1'001ns, 8 * MHz);

    // Verify
    expect(that % 1 == tiny_polls);
    expect(that % 5 == odd_polls);
  };

  "a stall timeout that can't be counted is rejected"_test = []() {
    // Exercise + Verify
    expect(throws<hal::operation_not_supported>(
      []() { static_cast<void>(compute_stall_polls(0ms, 36 * MHz)); }));
    expect(throws<hal::operation_not_supported>(
      []() { static_cast<void>(compute_stall_polls(-1ms, 36 * MHz)); }));
    expect(throws<hal::operation_not_supported>(
      []() { static_cast<void>(compute_stall_polls(100ms, 0 * Hz)); }));
    // 4,294,967,295 polls at 50 MHz take about 171.8 seconds.
    expect(throws<hal::operation_not_supported>(
      []() { static_cast<void>(compute_stall_polls(172s, 50 * MHz)); }));
    expect(that % 4'275'000'000U == compute_stall_polls(171s, 50 * MHz));
  };

  "fast mode uses a 1:2 high:low duty cycle"_test = []() {
    // Exercise
    // 36 MHz / (3 * 400 kHz) = 30
    auto const timing = compute_i2c_timing(400 * kHz, 36 * MHz);

    // Verify
    expect(timing.fast_mode);
    expect(that % 30 == timing.ccr);
    // 300 ns at 36 MHz is 10.8 cycles, rounded down, plus one.
    expect(that % 11 == timing.rise_time);
  };

  "the divider rounds up, so the bus never runs faster than asked"_test = []() {
    // Exercise
    // 8 MHz / (2 * 90 kHz) = 44.4, which would run at 90.9 kHz.
    auto const timing = compute_i2c_timing(90 * kHz, 8 * MHz);

    // Verify
    expect(that % 45 == timing.ccr);
  };

  "a rate above fast mode runs at 400 kHz"_test = []() {
    // Exercise
    auto const timing = compute_i2c_timing(1 * MHz, 36 * MHz);

    // Verify
    expect(timing.fast_mode);
    expect(that % 30 == timing.ccr);
  };

  "unachievable settings are rejected"_test = []() {
    // Exercise + Verify
    // Zero has no divider.
    expect(throws<hal::operation_not_supported>(
      []() { static_cast<void>(compute_i2c_timing(0 * Hz, 36 * MHz)); }));
    // 36 MHz / (2 * 4 kHz) = 4500, above the largest divider of 4095.
    expect(throws<hal::operation_not_supported>(
      []() { static_cast<void>(compute_i2c_timing(4 * kHz, 36 * MHz)); }));
    // Fast mode needs an input clock of at least 4 MHz.
    expect(throws<hal::operation_not_supported>(
      []() { static_cast<void>(compute_i2c_timing(400 * kHz, 3 * MHz)); }));
    // The peripheral needs an input clock of at least 2 MHz.
    expect(throws<hal::operation_not_supported>(
      []() { static_cast<void>(compute_i2c_timing(100 * kHz, 1 * MHz)); }));
  };
}
}  // namespace

int main()
{
  i2c_test();
  i2c_transaction_test();
}
