// Copyright 2024 - 2025 Khalil Estell and the libhal contributors
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

#include <cmath>
#include <cstdint>

#include <libhal-arm-mcu/stm32_generic/i2c.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/enum.hpp>
#include <libhal-util/i2c.hpp>
#include <libhal/error.hpp>
#include <libhal/io_waiter.hpp>
#include <libhal/units.hpp>

namespace hal::stm32_generic {

struct i2c_cr1
{
  /// 0: I2C Peripheral not under reset
  /// 1: I2C Peripheral under reset state
  static constexpr auto software_reset = hal::bit_mask::from<15>();
  /// 0: Releases SMBA pin high. Alert Response Address Header followed by NACK.
  /// 1: Drives SMBA pin low. Alert Response Address Header followed by ACK
  static constexpr auto smbus_alert = hal::bit_mask::from<13>();
  /// This bit is set and cleared by software, and cleared by hardware when PEC
  /// is transferred or by a START or Stop condition or when PE=0.
  /// 0: No PEC transfer
  /// 1: PEC transfer
  static constexpr auto pec = hal::bit_mask::from<12>();
  /// 0: ACK bit controls the (N)ACK of the current byte being received in the
  /// shift register. The PEC bit indicates that current byte in shift register
  /// is a PEC.

  /// 1: ACK bit controls the (N)ACK of the next byte which will be received in
  /// the shift register. The PEC bit indicates that the next byte in the shift
  /// register is a PEC
  static constexpr auto pos = hal::bit_mask::from<11>();
  /// This bit is set and cleared by software and cleared by hardware when PE=0
  /// 0: no ack return
  /// 1: Acknowledge returned after a byte is received (matched address or data)
  static constexpr auto ack_enable = hal::bit_mask::from<10>();
  /// Master mode:
  /// 0: no stop generation
  /// 1: Stop generation after the current byte transfer or after the current
  static constexpr auto stop = hal::bit_mask::from<9>();
  /// Master mode:
  /// 0: no start generation
  /// 1: repeated start generation
  static constexpr auto start = hal::bit_mask::from<8>();
  /// 0: Clock stretching enable
  /// 1: Clock stretching disable
  static constexpr auto clock_stretch_disable = hal::bit_mask::from<7>();
  /// 0: General call disable Address 0x00 is NACKed
  /// 1: General call enable Address 0x00 is ACKed
  static constexpr auto general_call_enable = hal::bit_mask::from<6>();
  /// 0: PEC calculation disable
  /// 1: PEC calculation enable
  static constexpr auto pec_enable = hal::bit_mask::from<5>();
  /// 0: ARP disable
  /// 1: ARP enable
  static constexpr auto arp_enable = hal::bit_mask::from<4>();
  /// 0: SMBus device
  /// 1: SMBus host
  static constexpr auto smbus_type = hal::bit_mask::from<3>();
  /// 0: i2c mode
  /// 1: SMBus mode
  static constexpr auto smbus_mode = hal::bit_mask::from<1>();
  /// 0: peripheral disable
  /// 1: peripheral enable
  static constexpr auto peripheral_enable = hal::bit_mask::from<0>();
};

struct i2c_cr2
{
  /// 0: Next DMA EOT is not the last transfer
  /// 1: Next DMA EOT is the last transfer
  static constexpr auto last_dma = hal::bit_mask::from<12>();
  /// 0: DMA requests disabled
  /// 1: DMA request enabled when TxE=1 or RxNE =1
  static constexpr auto dma_en = hal::bit_mask::from<11>();
  /// 0: TxE = 1 or RxNE = 1 does not generate any interrupt.
  /// 1: TxE = 1 or RxNE = 1 generates Event Interrupt
  static constexpr auto buffer_intr_en = hal::bit_mask::from<10>();
  /// 0: Event interrupt disabled
  /// 1: Event interrupt enabled
  /// This interrupt is generated when:
  /// - SB = 1 (Master)
  /// - ADDR = 1 (Master/Slave)
  /// - ADD10= 1 (Master)
  /// - STOPF = 1 (Slave)
  /// - BTF = 1 with no TxE or RxNE event
  /// - TxE event to 1 if ITBUFEN = 1
  /// - RxNE event to 1if ITBUFEN = 1
  static constexpr auto event_intr_en = hal::bit_mask::from<9>();
  /// 0: Error interrupt disabled
  /// 1: Error interrupt enabled
  /// This interrupt is generated when:
  /// - BERR = 1
  /// - ARLO = 1
  /// - AF = 1
  /// - OVR = 1
  /// - PECERR = 1
  /// - TIMEOUT = 1
  /// - SMBALERT = 1
  static constexpr auto error_intr_en = hal::bit_mask::from<8>();
  /// APB clock frequency
  /// how many MHz
  static constexpr auto apb_frequency = hal::bit_mask::from<5, 0>();
};
struct i2c_sr1
{
  static constexpr auto smb_alert = hal::bit_mask::from<15>();
  /// 0: No timeout error
  /// 1: SCL remained LOW for 25 ms (Timeout)
  /// or
  /// Master cumulative clock low extend time more than 10 ms (Tlow:mext)
  /// or
  /// Slave cumulative clock low extend time more than 25 ms (Tlow:sext)
  /// Cleared by software writing 0, or by hardware when PE=0
  static constexpr auto timeout_error = hal::bit_mask::from<14>();
  /// 0: no PEC error: receiver returns ACK after PEC reception (if ACK=1)
  /// 1: PEC error: receiver returns NACK after PEC reception (whatever ACK)
  /// Cleared by software writing 0, or by hardware when PE=0
  static constexpr auto pec_error = hal::bit_mask::from<12>();
  /// 0: No overrun/underrun
  /// 1: Overrun or underrun
  /// Cleared by software writing 0, or by hardware when PE=0
  static constexpr auto over_under_run = hal::bit_mask::from<11>();
  /// 0: No acknowledge failure
  /// 1: Acknowledge failure
  /// Cleared by software writing 0, or by hardware when PE=0
  static constexpr auto ack_failure = hal::bit_mask::from<10>();
  /// 0: No Arbitration Lost detected
  /// 1: Arbitration Lost detected
  /// Cleared by software writing 0, or by hardware when PE=0
  static constexpr auto arbitration_lost = hal::bit_mask::from<9>();
  /// 0: No misplaced Start or Stop condition
  /// 1: Misplaced Start or Stop condition
  /// Cleared by software writing 0, or by hardware when PE=0
  static constexpr auto bus_error = hal::bit_mask::from<8>();
  /// 0: Data register not empty
  /// 1: Data register empty
  /// Cleared by software writing to the DR register or by hardware after a
  /// start or a stop condition or when PE=0
  static constexpr auto tx_empty = hal::bit_mask::from<7>();
  /// 0: Data register empty
  /// 1: Data register not empty
  /// Cleared by software reading or writing the DR register or by hardware when
  /// PE=0.RxNE is not set in case of ARLO event
  static constexpr auto rx_not_empty = hal::bit_mask::from<6>();
  /// used in slave mode
  static constexpr auto stop_detection = hal::bit_mask::from<4>();
  /// 0: No ADD10 event occurred
  /// 1: Master has sent first address byte (header)
  /// Cleared by software by either a read or write in the DR register or by
  /// Cleared by software reading the SR1 register followed by a write in the DR
  /// register of the second address byte, or by hardware when PE=0.
  static constexpr auto addr10 = hal::bit_mask::from<3>();
  /// 0: Data byte transfer not done
  /// 1: Data byte transfer succeeded
  /// Cleared by software by either a read or write in the DR register or by
  /// hardware after a start or a stop condition in transmission or when PE=0.
  static constexpr auto byte_transfered_finish = hal::bit_mask::from<2>();
  /// This bit is cleared by software reading SR1 register followed reading SR2,
  /// or by hardware when PE=0. This interrupt is generated when:
  /// 0: No end of address transmission
  /// 1: End of address transmission
  static constexpr auto addr = hal::bit_mask::from<1>();
  /// Cleared by software by reading the SR1 register followed by writing the DR
  /// register, or by hardware when peripheral_enable=0
  /// 0: No Start condition
  /// 1: Start condition generated.
  static constexpr auto start = hal::bit_mask::from<0>();
};
struct i2c_sr2
{

  /// 0: No General Call
  /// 1: General Call Address received when ENGC=1
  /// Cleared by hardware after a Stop condition or repeated Start condition, or
  /// when PE=0
  static constexpr auto general_call_addr = hal::bit_mask::from<4>();
  /// 0: No communication on the bus
  /// 1: Communication ongoing on the bus
  /// It indicates a communication in progress on the bus. This information is
  /// still updated when the interface is disabled (PE=0).
  static constexpr auto trans_reciever = hal::bit_mask::from<2>();
  /// 0: No communication on the bus
  /// 1: Communication ongoing on the bus
  /// It indicates a communication in progress on the bus. This information is
  /// still updated when the interface is disabled (PE=0).
  static constexpr auto bus_busy = hal::bit_mask::from<1>();
  /// 0: Slave Mode
  /// 1: Master Mode
  /// Cleared by hardware after detecting a Stop condition on the bus or a loss
  /// of arbitration (ARLO=1), or by hardware when PE=0.
  static constexpr auto master = hal::bit_mask::from<0>();
};
struct i2c_ccr
{
  /// 0: Slow mode mode I2C
  /// 1: Fast mode mode I2C
  static constexpr auto i2c_mode_sel = hal::bit_mask::from<14>();
  /// 0: Fm mode tlow/thigh = 2
  /// 1: Fm mode tlow/thigh = 16/9 (see CCR)
  static constexpr auto duty_cycle = hal::bit_mask::from<14>();
  /// refer to CH 18.6.8 in the User manual () for equation
  static constexpr auto ccr = hal::bit_mask::from<11, 0>();
};
struct i2c_filter
{
  /// 0 : analog filter enable
  /// 1 : analog filter disable
  static constexpr auto analog = hal::bit_mask::from<4>();

  /// Digital noise filter enabled and filtering capability up to n * TPCLK1
  static constexpr auto digital_filter = hal::bit_mask::from<3, 0>();
};

/// i2c register map (RM0383 CH 18.6.11)
struct i2c_reg_t
{
  /// Control reg 1 (CH 18.6.1)
  hal::u32 volatile cr1;
  /// Control reg 2 (CH 18.6.2)
  hal::u32 volatile cr2;
  /// Own address reg 1 (CH 18.6.3)
  hal::u32 volatile oar1;
  /// Own address reg 2 (CH 18.6.4)
  hal::u32 volatile oar2;
  /// Data reg (CH 18.6.5)
  hal::u32 volatile data_reg;
  /// Status reg 1 (CH 18.6.6)
  hal::u32 volatile sr1;
  /// Status reg 2 (CH 18.6.7)
  hal::u32 volatile sr2;
  /// Clock control reg (CH 18.6.8)
  hal::u32 volatile ccr;
  /// Rise time reg (CH 18.6.9)
  hal::u32 volatile trise;
  /// Noise filter reg (CH 18.6.10)
  hal::u32 volatile filter;
};
namespace {
inline i2c_reg_t* get_i2c_reg(void* p_i2c)
{
  return reinterpret_cast<i2c_reg_t*>(p_i2c);
}

/**
 * @brief Reads the status register
 *
 * just trying to make the code more readable
 *
 * @param p_i2c_reg Peripheral register pointers
 */
inline void read_status_reg2(i2c_reg_t const* p_i2c_reg)
{
  p_i2c_reg->sr2;
}
}  // namespace

i2c::i2c(void* p_i2c)
{
  m_i2c = p_i2c;
}

i2c::i2c()
{
  m_i2c = nullptr;
}

i2c::~i2c()
{
  auto i2c_reg = get_i2c_reg(m_i2c);
  bit_modify(i2c_reg->cr1).clear(i2c_cr1::peripheral_enable);
  bit_modify(i2c_reg->cr1).set(i2c_cr1::software_reset);
  bit_modify(i2c_reg->cr1).clear(i2c_cr1::software_reset);
}

void i2c::configure(hal::i2c::settings const& p_settings, hertz p_frequency)
{
  constexpr auto slow_mode_max_speed = 100_Hz;
  auto const freq = static_cast<u32>(p_frequency);

  if (2_MHz > freq || freq > 50_MHz) {
    safe_throw(hal::argument_out_of_domain(this));
  }
  auto i2c_reg = get_i2c_reg(m_i2c);
  bit_modify(i2c_reg->cr1).set(i2c_cr1::software_reset);
  bit_modify(i2c_reg->cr1).clear(i2c_cr1::software_reset);

  bit_modify(i2c_reg->filter)
    .set(i2c_filter::digital_filter)
    .clear(i2c_filter::analog);

  /// I2C communication speed, fahb / (2 * ccr). The real frequency may
  /// differ due to the analog noise filter input delay.
  /// CH 18.6.8 in RM0383 (stm32f411 user manual)

  u16 const ccr_value =
    freq / (2 * static_cast<hal::u32>(p_settings.clock_rate));
  if (p_settings.clock_rate > slow_mode_max_speed) {
    if (ccr_value > 4) {
      bit_modify(i2c_reg->ccr)
        .set(i2c_ccr::i2c_mode_sel)
        .insert<i2c_ccr::ccr>(ccr_value);
    } else {
      safe_throw(hal::argument_out_of_domain(this));
    }
  } else {
    bit_modify(i2c_reg->ccr)
      .clear(i2c_ccr::i2c_mode_sel)
      .insert<i2c_ccr::ccr>(ccr_value);
  }

  auto const ahb_freq = static_cast<hal::u8>(p_frequency / 1_MHz);
  bit_modify(i2c_reg->cr2)
    .set(i2c_cr2::buffer_intr_en)
    .set(i2c_cr2::event_intr_en)
    .set(i2c_cr2::error_intr_en)
    .insert<i2c_cr2::apb_frequency>(ahb_freq);
  bit_modify(i2c_reg->cr1)
    .set(i2c_cr1::peripheral_enable)
    .set(i2c_cr1::ack_enable)
    .clear(i2c_cr1::smbus_mode);
}

void i2c::handle_i2c_event() noexcept
{
  auto i2c_reg = get_i2c_reg(m_i2c);
  auto& status = i2c_reg->sr1;
  auto& data = i2c_reg->data_reg;

  /// @warning SOMETIMES STM32 I2C peripheral enable will just self reset, we
  /// need it to not
  bit_modify(i2c_reg->cr1).set(i2c_cr1::peripheral_enable);
  if (bit_extract<i2c_sr1::start>(status)) {
    if (!m_data_out.empty()) {
      data = to_8_bit_address(m_address, i2c_operation::write);
      m_state = transmission_state::transmitter;
    } else {
      data = to_8_bit_address(m_address, i2c_operation::read);
      m_state = transmission_state::reciever;
    }
    return;
  }

  if (bit_extract<i2c_sr1::addr>(status)) {
    if (m_state == transmission_state::reciever) {
      switch (m_data_in.size()) {
        case 1: {
          bit_modify(i2c_reg->cr1).clear(i2c_cr1::ack_enable);
          i2c_reg->sr2;
          break;
        }
        case 2: {
          bit_modify(i2c_reg->cr1).clear(i2c_cr1::ack_enable).set(i2c_cr1::pos);
          read_status_reg2(i2c_reg);
          m_data_in[1] = data;
          m_data_in[0] = data;
          break;
        }
        default: {
          read_status_reg2(i2c_reg);
        }
      }
    } else {
      read_status_reg2(i2c_reg);
    }
    return;
  }

  if (bit_extract<i2c_sr1::tx_empty>(status)) {
    if (!m_data_out.empty()) {
      data = m_data_out[0];
      m_data_out = m_data_out.subspan(1);
    } else {
      if (!m_data_in.empty()) {
        bit_modify(i2c_reg->cr1).set(i2c_cr1::start);
      } else {
        bit_modify(i2c_reg->cr1).clear(i2c_cr1::ack_enable);
        bit_modify(i2c_reg->cr1).set(i2c_cr1::stop);
        m_state = transmission_state::free;
      }
    }
    return;
  }

  if (bit_extract<i2c_sr1::rx_not_empty>(status)) {
    switch (m_data_in.size()) {
      case 2: {
        bit_modify(i2c_reg->cr1).clear(i2c_cr1::ack_enable);
        m_data_in[0] = data;
        bit_modify(i2c_reg->cr1).set(i2c_cr1::stop);
        m_data_in[1] = data;
        m_data_in[2] = data;
        m_state = transmission_state::free;

        break;
      }

      case 1: {
        m_data_in[0] = data;
        bit_modify(i2c_reg->cr1).set(i2c_cr1::stop);
        m_state = transmission_state::free;

        break;
      }

      case 0: {
        bit_modify(i2c_reg->cr1).set(i2c_cr1::stop).clear(i2c_cr1::ack_enable);
        [[maybe_unused]] auto a = data;
        break;
      }

      default: {
        m_data_in[0] = data;
        m_data_in = m_data_in.subspan(1);
      }
    }
  }
}

void i2c::handle_i2c_error() noexcept
{
  auto i2c_reg = get_i2c_reg(m_i2c);
  auto& status = i2c_reg->sr1;

  if (bit_extract<i2c_sr1::arbitration_lost>(status)) {
    m_status = error_state::arbitration_lost;
  }

  if (bit_extract<i2c_sr1::timeout_error>(status)) {
    /// we don't support stm32's timeout function, but have an external io
    /// waiter
    bit_modify(i2c_reg->sr1).clear(i2c_sr1::timeout_error);
  }

  if (bit_extract<i2c_sr1::ack_failure>(status)) {
    m_status = error_state::no_such_device;
    bit_modify(i2c_reg->sr1).clear<i2c_sr1::ack_failure>();
  }

  if (bit_extract<i2c_sr1::bus_error>(status)) {
    m_status = error_state::io_error;
    bit_modify(i2c_reg->sr1).clear<i2c_sr1::bus_error>();
  }
}
void i2c::transaction(hal::byte p_address,
                      std::span<hal::byte const> p_data_out,
                      std::span<hal::byte> p_data_in,
                      hal::function_ref<hal::timeout_function> p_timeout)
{
  m_status = error_state::no_error;
  m_address = p_address;
  m_data_out = p_data_out;
  m_data_in = p_data_in;
  m_state = transmission_state::transmitter;
  auto i2c_reg = get_i2c_reg(m_i2c);
  bit_modify(i2c_reg->cr1).set(i2c_cr1::ack_enable);
  bit_modify(i2c_reg->cr1).set(i2c_cr1::peripheral_enable);
  bit_modify(i2c_reg->cr1).clear(i2c_cr1::stop);
  bit_modify(i2c_reg->cr1).set(i2c_cr1::start);
  while (m_state != transmission_state::free) {
    try {
      p_timeout();
      handle_i2c_error();
      switch (m_status) {
        case error_state::no_error: {
          break;
        }
        case error_state::no_such_device: {
          safe_throw(hal::no_such_device(m_address, this));
          break;
        }
        case error_state::io_error: {
          safe_throw(hal::io_error(this));
          break;
        }
        case error_state::arbitration_lost: {
          while (bit_extract<i2c_sr2::bus_busy>(i2c_reg->sr2)) {
            continue;
          }
          bit_modify(i2c_reg->cr1)
            .set(i2c_cr1::peripheral_enable)
            .set(i2c_cr1::ack_enable)
            .clear(i2c_cr1::stop);
          bit_modify(i2c_reg->cr1).set(i2c_cr1::start);
          break;
        }
      }

    } catch (...) {
      // The expected exception is hal::timed_out, but it could be something
      // else. Let rethrow the exception so the caller handle it.
      // A better option here would be to use a std::scope_failure handler.
      bit_value(i2c_reg->cr1).clear(i2c_cr1::peripheral_enable);
      throw;
    }
  };
}
}  // namespace hal::stm32_generic
