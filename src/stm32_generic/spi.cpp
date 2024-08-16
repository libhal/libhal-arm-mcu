#include <bit>

#include <libhal-arm-mcu/stm32_generic/spi.hpp>
#include <libhal-util/bit.hpp>
#include <libhal/error.hpp>

namespace hal::stm32_generic {

namespace {
struct spi_reg_t
{
  /*!< Offset: 0x000 Control Register 1 (R/W) */
  uint32_t volatile cr1;
  /*!< Offset: 0x004 Control Register 2 (R/W) */
  uint32_t volatile cr2;
  /*!< Offset: 0x008 Status Register (R/W) */
  uint32_t volatile sr;
  /*!< Offset: 0x00C Data Register (R/W) */
  uint32_t volatile dr;
  /*!< Offset: 0x010 CRC polynomial register (R/ ) */
  uint32_t const volatile crcpr;
  /*!< Offset: 0x014 RX CRC register (R/W) */
  uint32_t volatile rxcrcr;
  /*!< Offset: 0x018 TX CRC Register (R/W) */
  uint32_t volatile txcrcr;
  /*!< Offset: 0x01C configuration register (R/W) */
  uint32_t volatile i2scfgr;
  /*!< Offset: 0x020 prescaler register (R/W) */
  uint32_t volatile i2spr;
};

/// SPI Control Register 1
struct control_register1
{
  /// 0: first clock transistion is the first capture edge
  /// 1: second clock transition is the first data capture edge
  static constexpr auto clock_phase = bit_mask::from<0>();

  /// 0: clock to 0 when idle
  /// 1: clock to 1 when idle
  static constexpr auto clock_polarity = bit_mask::from<1>();

  /// 0: slave, 1: master
  static constexpr auto master_selection = bit_mask::from<2>();

  /// baudrate control: sets the clock rate to:
  /// (peripheral clock frequency)/2**(n+1)
  static constexpr auto baud_rate_control = bit_mask::from<5, 3>();

  /// Peripheral Enable
  /// 0: disable, 1: enable
  static constexpr auto enable = bit_mask::from<6>();

  /// Frame Format
  /// 0: msb transmitted first
  /// 1: lsb tranmitted first
  static constexpr auto frame_format = bit_mask::from<7>();

  /// internal slave select
  static constexpr auto internal_slave_select = bit_mask::from<8>();

  /// Software slave management
  /// 0: disable, 1: enable
  static constexpr auto software_slave_management = bit_mask::from<9>();

  /// Recieve only
  /// 0: Full Duplex, 1: Output disable
  [[maybe_unused]] static constexpr auto rx_only = bit_mask::from<10>();

  /// Data frame format
  /// 0: 8-bits, 1: 16-bit
  [[maybe_unused]] static constexpr auto data_frame_format =
    bit_mask::from<11>();

  /// CRC transfer next
  /// 0: No CRC phase, 1: transfer CRC next
  [[maybe_unused]] static constexpr auto crc_transfer_next =
    bit_mask::from<12>();

  /// CRC enable
  /// 0: disable, 1: enable
  [[maybe_unused]] static constexpr auto crc_enable = bit_mask::from<13>();

  /// Output enable in bidirectional mode
  /// 0: output disabled, 1: output enable
  static constexpr auto bidirectional_output_enable = bit_mask::from<14>();

  /// Bidirectional data mode enable
  /// 0: full-duplex, 1: half-duplex
  [[maybe_unused]] static constexpr auto bidirectional_mode_enable =
    bit_mask::from<15>();
};

/// SPI Control Register 2
struct control_register2
{
  /// Rx buffer DMA enable
  [[maybe_unused]] static constexpr auto rx_dma_enable = bit_mask::from<0>();

  /// Tx buffer DMA enable
  [[maybe_unused]] static constexpr auto tx_dma_enable = bit_mask::from<1>();

  /// Slave select output enable
  /// 0: use a GPIO, 1: use the NSS pin
  static constexpr auto slave_select_output_enable = bit_mask::from<2>();

  /// Frame format
  /// 0: Motorola mode, 1: TI mode
  static constexpr auto frame_format = bit_mask::from<4>();

  /// Error interupt enable
  [[maybe_unused]] static constexpr auto error_interrupt_enable =
    bit_mask::from<5>();

  /// Rx buffer empty interrupt enable
  [[maybe_unused]] static constexpr auto rx_buffer_empty_interrupt_enable =
    bit_mask::from<6>();

  /// Tx buffer empty interrupt enable
  [[maybe_unused]] static constexpr auto tx_buffer_empty_interrupt_enable =
    bit_mask::from<7>();
};

/// SPI Status Register
struct status_register
{
  /// Recieve buffer not empty
  static constexpr auto rx_buffer_not_empty = bit_mask::from<0>();

  /// Transmit buffer not empty
  static constexpr auto tx_buffer_empty = bit_mask::from<1>();

  /// Channel side (i2s only)
  /// 0: left has been transmitted/recieved
  /// 1: right has been transmitted/recieved
  [[maybe_unused]] static constexpr auto i2s_channel_side = bit_mask::from<2>();

  /// Underrun flag
  [[maybe_unused]] static constexpr auto underrun_flag = bit_mask::from<3>();

  /// CRC error flag
  [[maybe_unused]] static constexpr auto crc_error_flag = bit_mask::from<4>();

  /// Mode fault flag
  [[maybe_unused]] static constexpr auto mode_fault_flag = bit_mask::from<5>();

  /// Overrun flag
  [[maybe_unused]] static constexpr auto overrun_flag = bit_mask::from<6>();

  /// Busy flag
  static constexpr auto busy_flag = bit_mask::from<7>();

  /// frame format error flag
  [[maybe_unused]] static constexpr auto frame_format_error_flag =
    bit_mask::from<8>();
};

/**
 * @brief Convert a void* to an spi_reg_t for use in the driver.
 *
 * @param p_address - the address of the peripheral. If the address is outside
 * of valid memory, then the driver will trigger an ARM Cortex Memory Fault
 * Exception. If the address points to valid memory that is not an spi
 * peripheral, then using the result of this function is UB.
 * @return spi_reg_t& - reference to an spi register map pointed to by p_address
 */
spi_reg_t& to_reg(void* p_address)
{
  return *reinterpret_cast<spi_reg_t*>(p_address);
}

inline bool busy(spi_reg_t& p_reg)
{
  return bit_extract<status_register::busy_flag>(p_reg.sr);
}
inline bool tx_empty(spi_reg_t& p_reg)
{
  return bit_extract<status_register::tx_buffer_empty>(p_reg.sr);
}
inline bool rx_not_empty(spi_reg_t& p_reg)
{
  return bit_extract<status_register::rx_buffer_not_empty>(p_reg.sr);
}
}  // namespace

spi::spi(hal::unsafe, void* p_peripheral_address)
  : m_peripheral_address(p_peripheral_address)
{
}

spi::~spi()
{
}

void spi::configure(hal::spi::settings const& p_settings,
                    hal::hertz p_peripheral_clock_speed)
{
  auto& reg = to_reg(m_peripheral_address);

  // Set SPI to master mode by clearing
  bit_modify(reg.cr1).set(control_register1::master_selection);

  using namespace hal::literals;
  auto const clock_divider = p_peripheral_clock_speed / p_settings.clock_rate;
  auto prescaler = static_cast<std::uint16_t>(clock_divider);
  if (prescaler <= 1) {
    prescaler = 2;
  } else if (prescaler > 256) {
    hal::safe_throw(hal::operation_not_supported(this));
  }

  uint16_t baud_control = 15 - std::countl_zero(prescaler);
  if (std::has_single_bit(prescaler)) {
    baud_control--;
  }
  bit_modify(reg.cr1)
    .insert<control_register1::baud_rate_control>(baud_control)
    .insert<control_register1::clock_phase>(
      p_settings.data_valid_on_trailing_edge)
    .insert<control_register1::clock_polarity>(p_settings.clock_idles_high)
    .set<control_register1::bidirectional_output_enable>()
    .set<control_register1::software_slave_management>()
    .set<control_register1::internal_slave_select>()
    .clear<control_register1::frame_format>();

  bit_modify(reg.cr2)
    .set<control_register2::slave_select_output_enable>()
    .clear<control_register2::frame_format>();

  bit_modify(reg.cr1).set<control_register1::master_selection>();

  bit_modify(reg.cr1)
    .set<control_register1::enable>()
    .clear<control_register1::internal_slave_select>();
}

void spi::transfer(std::span<hal::byte const> p_data_out,
                   std::span<hal::byte> p_data_in,
                   hal::byte p_filler)
{
  auto& reg = to_reg(m_peripheral_address);
  size_t max_length = std::max(p_data_in.size(), p_data_out.size());

  // NOTE: This is a paranoid check to determine that there is no bus activity
  // before proceeding
  while (busy(reg)) {
    continue;
  }

  // The stm's spi driver needs to be internally told that it is selecting a
  // device before it will emit anything on the pins. This will control the NSS
  // pin if it is selected, otherwise, its just an internal enable signal.
  bit_modify(reg.cr1).set<control_register1::internal_slave_select>();

  for (size_t index = 0; index < max_length; index++) {
    hal::byte byte = 0;

    if (index < p_data_out.size()) {
      byte = p_data_out[index];
    } else {
      byte = p_filler;
    }

    while (not tx_empty(reg)) {
      continue;
    }

    reg.dr = byte;

    while (not rx_not_empty(reg)) {
      continue;
    }

    byte = static_cast<uint8_t>(reg.dr);
    if (index < p_data_in.size()) {
      p_data_in[index] = byte;
    }
  }

  bit_modify(reg.cr1).clear<control_register1::internal_slave_select>();

  // Wait for bus activity to cease before leaving the function
  while (busy(reg)) {
    continue;
  }
}
}  // namespace hal::stm32_generic
