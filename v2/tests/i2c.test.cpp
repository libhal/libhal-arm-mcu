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

// compute_i2c_timing() and the stall poll count are pure arithmetic, so they
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
constexpr hal::u32 sr1_arbitration_lost = 0x0200;
constexpr hal::u32 sr1_ack_failure = 0x0400;
constexpr hal::u32 sr2_busy = 0x0002;
constexpr hal::u32 cr1_enable = 0x0001;
constexpr hal::u32 cr1_start = 0x0100;
constexpr hal::u32 cr1_stop = 0x0200;
constexpr hal::byte address = 0x50;

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

  hal::u32& sr2()
  {
    return registers[6];
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

  "a NACK on the first written byte is broken_pipe with no progress"_test =
    []() {
      // Setup
      fake_bus fake(sr1_start | sr1_addr | sr1_tx_empty | sr1_ack_failure);
      hal::u32 progress = 1;

      // Exercise
      try {
        fake.bus.transaction(address, { one_byte }, {});
      } catch (hal::broken_pipe const& p_error) {
        progress = p_error.progress;
      }

      // Verify
      expect(that % 0 == progress);
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

  "a busy bus throws try_again at once, without touching it"_test = []() {
    // Setup
    fake_bus fake(0);
    fake.sr2() = sr2_busy;
    auto const cr1 = fake.cr1();

    // Exercise + Verify
    expect(throws<hal::resource_unavailable_try_again>(
      [&]() { fake.bus.transaction(address, { three_bytes }, {}); }));
    expect(that % cr1 == fake.cr1());
  };

  "a lost arbitration throws try_again, without a retry"_test = []() {
    // Setup
    fake_bus fake(sr1_start | sr1_arbitration_lost);

    // Exercise + Verify
    // A retry would find the flag cleared and ADDR never set, and time out.
    expect(throws<hal::resource_unavailable_try_again>(
      [&]() { fake.bus.transaction(address, { three_bytes }, {}); }));
    expect(that % 0 == (fake.sr1() & sr1_arbitration_lost));
    expect(that % 0 == (fake.cr1() & cr1_enable));
  };

  "a start condition that never goes out throws try_again"_test = []() {
    // Setup
    fake_bus fake(0);

    // Exercise + Verify
    expect(throws<hal::resource_unavailable_try_again>(
      [&]() { fake.bus.transaction(address, { three_bytes }, {}); }));
    // It must not go out later, once the bus is free.
    expect(that % 0 == (fake.cr1() & cr1_start));
  };

  "a stall after the start condition throws timed_out"_test = []() {
    // Setup
    fake_bus fake(sr1_start);

    // Exercise + Verify
    expect(throws<hal::timed_out>(
      [&]() { fake.bus.transaction(address, { three_bytes }, {}); }));
  };

  "a stall while writing counts only bytes known to have arrived"_test = []() {
    // Setup
    fake_bus fake(sr1_start | sr1_addr | sr1_tx_empty);
    hal::u32 progress = 0;

    // Exercise
    try {
      fake.bus.transaction(address, { three_bytes }, {});
    } catch (hal::timed_out const& p_error) {
      progress = p_error.progress;
    }

    // Verify
    // All three moved, but the last two may not have reached the device.
    expect(that % 1 == progress);
  };

  "a stop condition that can't go out doesn't hang a failure"_test = []() {
    // Setup
    // Nothing clears STOP here, as on a bus with a line stuck low.
    fake_bus fake(sr1_start | sr1_ack_failure);

    // Exercise + Verify
    expect(throws<hal::no_such_device>(
      [&]() { fake.bus.transaction(address, { three_bytes }, {}); }));
    expect(that % 0 == (fake.cr1() & cr1_enable));
  };
}

void i2c_test()
{
  using namespace boost::ut;
  using namespace mp_units::si::unit_symbols;
  using namespace std::chrono_literals;
  using hal::stm32_generic::compute_i2c_timing;

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

  "a poll of a peripheral on APB takes at least two cycles"_test = []() {
    // Exercise
    // 100 ms at 36 MHz is 3,600,000 cycles.
    auto const polls = hal::cortex_m::apb_polls_for(36 * MHz, 100ms);
    // 25 ms at 8 MHz is 200,000 cycles.
    auto const slow_polls = hal::cortex_m::apb_polls_for(8 * MHz, 25ms);

    // Verify
    expect(that % 1'800'000 == polls);
    expect(that % 100'000 == slow_polls);
  };

  "configure rejects a stall timeout that isn't positive"_test = []() {
    // Setup
    std::array<hal::u32, 16> registers{};
    auto const configure_with = [&](hal::time_duration p_stall_timeout) {
      hal::stm32_generic::i2c bus(registers.data(), p_stall_timeout);
      bus.configure(hal::i2c_settings{}, 50 * MHz);
    };

    // Exercise + Verify
    expect(
      throws<hal::operation_not_supported>([&]() { configure_with(0ms); }));
    expect(
      throws<hal::operation_not_supported>([&]() { configure_with(-1ms); }));
    expect(nothrow([&]() { configure_with(1ns); }));
    // An hour at 50 MHz is 90,000,000,000 polls, past 32 bits.
    expect(nothrow([&]() { configure_with(1h); }));
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
