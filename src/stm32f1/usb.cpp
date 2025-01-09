#include <bit>
#include <cstdint>

#include <libhal-arm-mcu/stm32f1/clock.hpp>
#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal-arm-mcu/stm32f1/interrupt.hpp>
#include <libhal-arm-mcu/stm32f1/usb.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/enum.hpp>
#include <libhal-util/math.hpp>
#include <libhal-util/static_callable.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <libhal/output_pin.hpp>

#include "libhal-arm-mcu/stm32f1/pin.hpp"
#include "power.hpp"
#include "stm32f1/pin.hpp"

namespace hal::stm32f1 {

namespace {

enum class endpoint_type : std::uint8_t
{
  bulk = 0b00,
  control = 0b01,
  iso = 0b10,
  interrupt = 0b11,
};

struct control  // NOLINT
{
  // FRES: Force USB Reset
  static constexpr auto force_reset = bit_mask::from<0>();
  // PDWN: Power Down
  static constexpr auto power_down = bit_mask::from<1>();
  // LPMODE: Low-power Mode
  static constexpr auto low_power_mode = bit_mask::from<2>();
  // FSUSP: Force Suspend
  static constexpr auto force_suspend = bit_mask::from<3>();
  // RESUME: Resume Request
  static constexpr auto resume_request = bit_mask::from<4>();
  // ESOFM: Expected Start Of Frame Interrupt Mask
  static constexpr auto expected_start_of_frame_interrupt = bit_mask::from<8>();
  // SOFM: Start Of Frame Interrupt Mask
  static constexpr auto start_of_frame_interrupt = bit_mask::from<9>();
  // RESETM: USB Reset Interrupt Mask
  static constexpr auto reset_interrupt = bit_mask::from<10>();
  // SUSPM: Suspend Mode Interrupt Mask
  static constexpr auto suspend_mode_interrupt = bit_mask::from<11>();
  // WKUPM: Wakeup Interrupt Mask
  static constexpr auto wakeup_interrupt = bit_mask::from<12>();
  // ERRM: Error Interrupt Mask
  static constexpr auto error_interrupt = bit_mask::from<13>();
  // PMAOVRM: Packet Memory Area Over / Underrun Interrupt Mask
  static constexpr auto packet_memory_interrupt = bit_mask::from<14>();
  // CTRM: Correct Transfer for Isochronous Endpoint Interrupt Mask
  static constexpr auto correct_transfer_interrupt = bit_mask::from<15>();
};

struct interrupt_status  // NOLINT
{
  static constexpr auto endpoint_id = bit_mask::from<0, 3>();
  static constexpr auto direction = bit_mask::from<4>();
  // ESOF: Expected Start Of Frame
  static constexpr auto expected_start_of_frame = bit_mask::from<8>();
  // SOF: Start Of Frame
  static constexpr auto start_of_frame = bit_mask::from<9>();
  // RESET: USB Reset Request
  static constexpr auto reset_request = bit_mask::from<10>();
  // SUSP: Suspend Mode Request
  static constexpr auto suspend_mode_request = bit_mask::from<11>();
  // WKUP: Wake Up
  static constexpr auto wake_up = bit_mask::from<12>();
  // ERR: Error
  static constexpr auto error = bit_mask::from<13>();
  // PMAOVR: Packet Memory Area Over / Underrun
  static constexpr auto packet_memory_over_underrun = bit_mask::from<14>();
  // CTR: Correct Transfer
  static constexpr auto correct_transfer = bit_mask::from<15>();
};

struct frame_number  // NOLINT
{
  // FN: Frame Number
  static constexpr auto count = bit_mask::from<0, 10>();
  // LSOF: Lost SOF
  static constexpr auto lost_sof = bit_mask::from<11>();
  // LCK: Lock
  static constexpr auto lock = bit_mask::from<12>();
  // RXDM: Receive Data - Line Status
  static constexpr auto receive_data_status = bit_mask::from<13>();
  // RXDP: Transmit Data - Line Status
  static constexpr auto transmit_data_status = bit_mask::from<14>();
};

struct device_address  // NOLINT
{
  // ADD: Device Address
  static constexpr auto address = bit_mask::from<0, 6>();
  // EF: Enable Function
  static constexpr auto enable_function = bit_mask::from<7>();
};

struct endpoint  // NOLINT
{
  // EA: Endpoint Address
  static constexpr auto address = bit_mask::from<0, 3>();
  // STAT_TX: Status Bits, for transmission transfers
  static constexpr auto status_tx = bit_mask::from<4, 5>();
  // DTOG_TX: Data Toggle, for transmission transfers
  static constexpr auto data_toggle_tx = bit_mask::from<6>();
  // CTR_TX: Correct Transfer for Transmission
  static constexpr auto correct_transfer_tx = bit_mask::from<7>();
  // EP_KIND: Endpoint Kind
  static constexpr auto kind = bit_mask::from<8>();
  // EP_TYPE: Endpoint Type
  static constexpr auto type = bit_mask::from<9, 10>();
  // SETUP: Setup Transaction Completed
  static constexpr auto setup_complete = bit_mask::from<11>();
  // STAT_RX: Status Bits, for reception transfers
  static constexpr auto status_rx = bit_mask::from<12, 13>();
  // DTOG_RX: Data Toggle, for reception transfers
  static constexpr auto data_toggle_rx = bit_mask::from<14>();
  // CTR_RX: Correct Transfer for Reception
  static constexpr auto correct_transfer_rx = bit_mask::from<15>();
};

struct block_table
{
  static constexpr std::array<std::uint8_t, 2> block_size_table{
    2,
    32,
  };
  // BL: Block size = 0 means 2 bytes, = 1 means 32 bytes
  static constexpr auto block_size = bit_mask::from<15>();
  static constexpr auto number_of_blocks = bit_mask::from<14, 10>();
  static constexpr auto count = bit_mask::from<9, 0>();
};

// Skipped: Buffer Descriptor Table (BDT) structure as it might vary depending
// on the specific STM32F103 variant

usb_reg_t& reg()
{
  return *usb_reg;
}

// Memory internal to USB used for sending and receiving packets to and from the
// HOST
constexpr std::size_t packet_buffer_sram_size = 512;
// Max number of endpoints
constexpr std::size_t max_endpoints = 8;
// Set the start of the buffer descriptor to the start of the packet buffer;
constexpr std::size_t buffer_descriptor_table_start = 0;
// Page 650 has an equation for the Reception byte count n for endpoint N.
// The equation is `[USB_BTABLE] + n*16 + 12` which also represents the furthest
// the BTABLE could ever be from the start of the packet buffer. Lets make that
// a reserved area of the packet buffer. The rest of the memory can be used for
// endpoint send and receive.
constexpr std::size_t buffer_descriptor_table_end = (max_endpoints * 16) + 12;

constexpr std::size_t initial_packet_buffer_memory =
  packet_buffer_sram_size - buffer_descriptor_table_end;

/// TODO(kammce): make this no longer fixed, maybe...
constexpr std::uint16_t fixed_endpoint_size = 18;
constexpr std::uint16_t crc_tail = 2;
constexpr std::uint16_t rx_endpoint_count_mask =
  hal::bit_value(0U)
    .clear<block_table::block_size>()
    .insert<block_table::number_of_blocks>(9U)
    .to<std::uint16_t>();

std::span<std::uint8_t> usb_packet_buffer_sram()
{
  return { std::bit_cast<std::uint8_t*>(0x4000'6000), packet_buffer_sram_size };
}

std::span<hal::u32> usb_packet_buffer_sram_u32()
{
  // NOTE: dividing by the sizeof(hal::u16) is not a mistake. The memory is u16
  // addressable by the USB peripheral but u32 accessible/aligned for the
  // processor. This means that each u16 has padding of an additional u16 that
  // goes nowhere. The number of u16 blocks in this memory region is equal to
  // the u32 blocks and thats why we do the division here.
  return { std::bit_cast<hal::u32*>(0x4000'6000),
           packet_buffer_sram_size / sizeof(hal::u16) };
}

constexpr std::uint16_t tx_endpoint_memory_address(std::size_t p_endpoint)
{
  auto const offset = (p_endpoint * 2) * fixed_endpoint_size;
  return buffer_descriptor_table_end + offset;
}

constexpr std::uint16_t rx_endpoint_memory_address(std::size_t p_endpoint)
{
  auto const offset = ((p_endpoint * 2) + 1) * fixed_endpoint_size;
  return buffer_descriptor_table_end + offset;
}

struct buffer_descriptor_block
{
  std::uint32_t tx_address;
  std::uint32_t tx_count;
  std::uint32_t rx_address;
  std::uint32_t rx_count;

  void setup_descriptor_for(std::uint8_t p_endpoint)
  {
    tx_address = tx_endpoint_memory_address(p_endpoint);
    tx_count = fixed_endpoint_size;

    rx_address = rx_endpoint_memory_address(p_endpoint);
    rx_count = rx_endpoint_count_mask;
  }

  std::uint16_t bytes_received()
  {
    auto const count = hal::bit_extract<block_table::count>(rx_count);
    auto const bytes_received = count - crc_tail;
    return bytes_received;
  }

  std::uint16_t rx_address_offset()
  {
    return rx_address;
  }

  std::uint16_t tx_address_offset()
  {
    return tx_address;
  }

  std::span<hal::u32> tx_span()
  {
    return usb_packet_buffer_sram_u32().subspan(tx_address / sizeof(u16));
  }

  void set_count_tx(std::uint16_t p_transfer_size)
  {
    tx_count = p_transfer_size;
  }
};

buffer_descriptor_block& endpoint_descriptor_block(std::size_t p_endpoint)
{
  auto* buffer = usb_packet_buffer_sram().data();
  buffer += p_endpoint * sizeof(buffer_descriptor_block);
  return *std::bit_cast<buffer_descriptor_block*>(buffer);
}

constexpr auto endpoint_invariant_mask = hal::bit_value(0U)
                                           .insert<endpoint::address>(0xFUL)
                                           .set<endpoint::correct_transfer_tx>()
                                           .set<endpoint::kind>()
                                           .insert<endpoint::type>(0xFUL)
                                           .set<endpoint::correct_transfer_rx>()
                                           .to<std::uint32_t>();

/// Same stat value for rx and tex
enum class stat : std::uint8_t
{
  // all reception requests addressed to this endpoint are ignored.
  disabled = 0b00,
  stall = 0b01,
  nak = 0b10,
  valid = 0b11,
};

/// Same stat value for rx and tex
enum class dtog : std::uint8_t
{
  // all reception requests addressed to this endpoint are ignored.
  tog0 = 0b0,
  tog1 = 0b1,
};

template<hal::bit_mask mask>
void set_endpoint_register_toggle(std::size_t p_endpoint,
                                  enumeration auto p_enum_value)
{
  if (p_endpoint > reg().EP.size()) {
    return;
  }

  auto& endpoint_register = reg().EP[p_endpoint].EPR;
  auto const reg_value = endpoint_register;
  auto const masked_endpoint_reg = reg_value & endpoint_invariant_mask;
  auto const desired_stat = hal::value(p_enum_value);
  auto const current_stat = hal::bit_extract<mask>(reg_value);
  auto const toggle_mask = current_stat ^ desired_stat;
  auto const final_reg_value = hal::bit_value(masked_endpoint_reg)
                                 .template insert<mask>(toggle_mask)
                                 .template to<std::uint32_t>();
  endpoint_register = final_reg_value;
}

void set_rx_stat(std::size_t p_endpoint, stat p_stat)
{
  set_endpoint_register_toggle<endpoint::status_rx>(p_endpoint, p_stat);
}

void set_tx_stat(std::size_t p_endpoint, stat p_stat)
{
  set_endpoint_register_toggle<endpoint::status_tx>(p_endpoint, p_stat);
}

[[maybe_unused]] void set_endpoint_address_and_type(std::size_t p_endpoint,
                                                    endpoint_type p_type)
{
  auto& endpoint_register = reg().EP[p_endpoint].EPR;
  auto const reg_value = endpoint_register;
  auto const masked_endpoint_reg = reg_value & endpoint_invariant_mask;
  auto const final_reg_value = hal::bit_value(masked_endpoint_reg)
                                 .insert<endpoint::address>(p_endpoint)
                                 .insert<endpoint::kind>(0U)
                                 .insert<endpoint::type>(hal::value(p_type))
                                 .to<std::uint32_t>();
  endpoint_register = final_reg_value;
}

template<hal::bit_mask mask>
void clear_correct_transfer_for(std::uint8_t endpoint_id)
{
  auto& endpoint_reg = reg().EP[endpoint_id].EPR;
  auto const endpoint_value = endpoint_reg;
  auto masked_value = endpoint_value & endpoint_invariant_mask;
  auto const masked_value_with_transfer_cleared =
    hal::bit_value(masked_value).clear(mask).to<hal::u32>();
  endpoint_reg = masked_value_with_transfer_cleared;
}
}  // namespace

int reset_counter = 0;
int transfer_complete_counter = 0;
int error_detected_count = 0;

[[maybe_unused]] void set_control_tog_bits()
{
  set_endpoint_register_toggle<endpoint::data_toggle_tx>(0, dtog::tog1);
  set_endpoint_register_toggle<endpoint::data_toggle_rx>(0, dtog::tog0);
}

void handle_bus_reset()
{
  // assignment to this register acts as an AND gate
  reg().ISTR = 0;
  reg().DADDR = 0u;  // disable USB Function

  // Clear any previous information within the packet buffer
  std::ranges::fill(usb_packet_buffer_sram(), 0);

  // Set the start of the buffer descriptor table to the start of the packet
  // buffer.
  reg().BTABLE = buffer_descriptor_table_start;

  endpoint_descriptor_block(0).setup_descriptor_for(0);
  set_endpoint_address_and_type(0, endpoint_type::control);
  set_rx_stat(0, stat::nak);
  set_tx_stat(0, stat::nak);

  // Reset endpoints to disabled
  for (std::size_t i = 1; i < reg().EP.size(); i++) {
    reg().EP[i].EPR = i;
    set_rx_stat(i, stat::disabled);
    set_tx_stat(i, stat::disabled);
  }

  hal::bit_modify(reg().CNTR)
    .set(control::reset_interrupt)
    .set(control::correct_transfer_interrupt)
    .set(control::wakeup_interrupt)
    .set(control::suspend_mode_interrupt)
    .set(control::packet_memory_interrupt);

  hal::bit_modify(reg().DADDR).set(device_address::enable_function);
}
void usb::interrupt_handler()
{
  auto& interrupt_reg = reg().ISTR;

  auto const interrupt_reg_value = interrupt_reg;
  auto const endpoint_id =
    hal::bit_extract<interrupt_status::endpoint_id>(interrupt_reg_value);
  auto const direction =
    hal::bit_extract<interrupt_status::direction>(interrupt_reg_value);
  bool const transfer_completed =
    hal::bit_extract<interrupt_status::correct_transfer>(interrupt_reg_value);

  if (transfer_completed) {
    if (direction == 0 /* meaning tx */) {
      transfer_complete_counter++;
      m_tx_busy[endpoint_id] = false;
      clear_correct_transfer_for<stm32f1::endpoint::correct_transfer_tx>(
        endpoint_id);
    } else {
      // interrupt status clearing is handled by this function
      read_endpoint_and_pass_to_callback(endpoint_id);
    }
  }

  bool const reset_request =
    hal::bit_extract<interrupt_status::reset_request>(interrupt_reg_value);

  if (reset_request) {
    handle_bus_reset();
  }

  bool const error_detected =
    hal::bit_extract<interrupt_status::error>(interrupt_reg_value);

  if (error_detected) {
    // assignment to this register acts as an AND gate
    interrupt_reg = ~(1U << interrupt_status::error.position);
    error_detected_count++;
  }

  bool const suspend_detected =
    hal::bit_extract<interrupt_status::suspend_mode_request>(
      interrupt_reg_value);

  if (suspend_detected) {
    // assignment to this register acts as an AND gate
    interrupt_reg = ~(1U << interrupt_status::suspend_mode_request.position);
  }

  bool const wake_detected =
    hal::bit_extract<interrupt_status::wake_up>(interrupt_reg_value);

  if (wake_detected) {
    // assignment to this register acts as an AND gate
    interrupt_reg = ~(1U << interrupt_status::wake_up.position);
  }

  bool const overrun_detected =
    hal::bit_extract<interrupt_status::packet_memory_over_underrun>(
      interrupt_reg_value);

  if (overrun_detected) {
    // assignment to this register acts as an AND gate
    interrupt_reg =
      ~(1U << interrupt_status::packet_memory_over_underrun.position);
  }
}

usb::usb(hal::steady_clock& p_clock, hal::time_duration p_power_on_time)
  : m_available_endpoint_memory(initial_packet_buffer_memory)
{
  // USB already active by some other means
  if (is_on(peripheral::usb)) {
    // TODO(kammce): use appropriate exception here
    hal::safe_throw(hal::unknown(this));
  }
  // Cannot be around while CAN is around since they share resources
  if (is_on(peripheral::can1)) {
    // TODO(kammce): use appropriate exception here
    hal::safe_throw(hal::unknown(this));
  }

  auto const usb_frequency = frequency(peripheral::usb);
  if (not hal::equals(usb_frequency, 48.0_MHz)) {
    hal::safe_throw(hal::operation_not_supported(nullptr));
  }

  // Clear any previous information within the packet buffer
  std::ranges::fill(usb_packet_buffer_sram(), 0);

  auto handle =
    static_callable<usb, 0, void(void)>([this] { interrupt_handler(); });
  // Only enable low priority because high priority is for iso endpoints which
  // we do not support yet.
  hal::stm32f1::initialize_interrupts();
  cortex_m::enable_interrupt(irq::usb_lp_can1_rx0, handle.get_handler());
  cortex_m::enable_interrupt(irq::usb_hp_can1_tx, handle.get_handler());

  // Must be done prior to lifting power down bit
  configure_pin({ .port = 'A', .pin = 11 }, push_pull_alternative_output);
  configure_pin({ .port = 'A', .pin = 12 }, push_pull_alternative_output);

  power_on(peripheral::usb);

  hal::delay(p_clock, p_power_on_time);
  // Perform reset
  hal::bit_modify(reg().CNTR)
    .set(control::power_down)
    .set(control::force_reset);
  hal::delay(p_clock, p_power_on_time);

  // The USB peripheral sets this bit on system reset, we need to clear it to
  // allow the device to power on. We must wait approximately 1us before we can
  // proceed for the stm32f103.
  hal::bit_modify(reg().CNTR).clear(control::power_down);
  hal::delay(p_clock, p_power_on_time);

  // Clears everything including the force reset, enabling the USB device.
  reg().CNTR = 0;

  handle_bus_reset();
}

void usb::read_endpoint_and_pass_to_callback(std::uint8_t p_endpoint)
{
  auto usb_sram_buffer = usb_packet_buffer_sram();
  auto& descriptor = endpoint_descriptor_block(p_endpoint);
  auto const memory_offset = descriptor.rx_address_offset();
  auto const bytes_received = descriptor.bytes_received();
  auto const* memory_location =
    std::bit_cast<std::uint32_t*>(usb_sram_buffer.data() + (memory_offset * 2));
  auto rx_endpoint_memory_span =
    std::span<std::uint32_t const>(memory_location, bytes_received / 2);
  std::array<hal::byte, 16> buffer{};

  for (size_t i = 0; i < bytes_received; i++) {
    std::uint32_t shift_amount = 0;
    if (i & 1) {
      shift_amount = 8;
    }
    auto rx_word = rx_endpoint_memory_span[i / 2];
    buffer[i] = (rx_word >> shift_amount) & 0xFF;
  }
  auto bytes_span = std::span(buffer).first(bytes_received);
  m_out_callbacks[p_endpoint](bytes_span);

  clear_correct_transfer_for<stm32f1::endpoint::correct_transfer_rx>(
    p_endpoint);
  set_rx_stat(p_endpoint, stat::valid);
}

void wait_for_endpoint_transfer_completion(std::uint8_t p_endpoint)
{
  constexpr auto nak_u32_value = static_cast<hal::u32>(stat::nak);
  auto& endpoint_reg = reg().EP[p_endpoint].EPR;
  auto endpoint_tx_status = hal::bit_extract<endpoint::status_tx>(endpoint_reg);
  while (endpoint_tx_status == nak_u32_value) {
    endpoint_tx_status = hal::bit_extract<endpoint::status_tx>(endpoint_reg);
  }
}

void usb::write_to_endpoint(std::uint8_t p_endpoint,
                            std::span<hal::byte const> p_data)
{
  auto& descriptor = endpoint_descriptor_block(p_endpoint);

  if (p_data.empty()) {
    // send zero length message
    descriptor.set_count_tx(0);
    set_tx_stat(p_endpoint, stat::valid);
    wait_for_endpoint_transfer_completion(p_endpoint);
    return;
  }

  auto tx_span = descriptor.tx_span();

  while (not p_data.empty()) {
    constexpr std::size_t mtu = fixed_endpoint_size - crc_tail;
    auto const min = std::min(p_data.size(), mtu);

    descriptor.set_count_tx(min);

    auto const even_min = min & ~1;
    for (std::size_t i = 0; i < even_min; i += 2) {
      std::uint32_t temp = (p_data[i + 1] << 8) | p_data[i];
      tx_span[i] = temp;
    }
    if (min & 1) {
      std::uint32_t temp = p_data[min - 1];
      tx_span[min - 1] = temp;
    }

    p_data = p_data.subspan(min);

    if (p_endpoint == 0) {
      set_rx_stat(p_endpoint, stat::nak);
      set_tx_stat(p_endpoint, stat::valid);
    } else {
      set_tx_stat(p_endpoint, stat::valid);
    }

    wait_for_endpoint_transfer_completion(p_endpoint);
  }
}

usb::~usb()
{
  configure_pin({ .port = 'A', .pin = 11 }, input_pull_up);
  configure_pin({ .port = 'A', .pin = 12 }, input_pull_up);
  power_off(peripheral::usb);
  cortex_m::disable_interrupt(irq::usb_lp_can1_rx0);
  cortex_m::disable_interrupt(irq::usb_hp_can1_tx);
}

void usb::set_out_callback(hal::callback<void(std::span<hal::byte>)> p_callback,
                           std::uint8_t p_endpoint)
{
  if (p_endpoint < m_out_callbacks.size()) {
    m_out_callbacks[p_endpoint] = p_callback;
  }
}

usb::control_endpoint usb::acquire_control_endpoint()
{
  return { *this };
}

usb::control_endpoint::control_endpoint(usb& p_usb)
  : m_usb(&p_usb)
{
}

usb::control_endpoint::~control_endpoint()
{
}

void usb::control_endpoint::driver_connect(bool p_should_connect)
{
  hal::bit_modify(reg().DADDR)
    .insert<device_address::enable_function>(p_should_connect);
}

void usb::control_endpoint::driver_set_address(std::uint8_t p_address)
{
  hal::bit_modify(reg().DADDR)
    .set(device_address::enable_function)
    .insert<device_address::address>(p_address);
}

void usb::control_endpoint::driver_write(std::span<hal::byte const> p_data)
{
  m_usb->write_to_endpoint(0, p_data);
}

void usb::control_endpoint::driver_on_request(
  hal::callback<void(std::span<hal::byte>)> p_callback)
{
  m_usb->set_out_callback(p_callback, 0);
  set_rx_stat(0, stat::valid);
}

bool usb::control_endpoint::in_setup_stage()
{
  return hal::bit_extract<endpoint::setup_complete>(reg().EP[0].EPR);
}

// usb::interrupt_in_endpoint implementation
usb::interrupt_in_endpoint::interrupt_in_endpoint()
{
}

usb::interrupt_in_endpoint::~interrupt_in_endpoint()
{
}

void usb::interrupt_in_endpoint::driver_write(
  [[maybe_unused]] std::span<hal::byte const> p_data)
{
  // ... fill this out ...
}

// usb::bulk_in_endpoint implementation
usb::bulk_in_endpoint::bulk_in_endpoint()
{
}

usb::bulk_in_endpoint::~bulk_in_endpoint()
{
}

void usb::bulk_in_endpoint::driver_write(
  [[maybe_unused]] std::span<hal::byte const> p_data)
{
  // ... fill this out ...
}

// usb::interrupt_out_endpoint implementation
usb::interrupt_out_endpoint::interrupt_out_endpoint()
{
}

usb::interrupt_out_endpoint::~interrupt_out_endpoint()
{
}

void usb::interrupt_out_endpoint::driver_on_receive(
  [[maybe_unused]] hal::callback<void(std::span<hal::byte>)> p_callback)
{
  // ... fill this out ...
}

// usb::bulk_out_endpoint implementation
usb::bulk_out_endpoint::bulk_out_endpoint()
{
}

usb::bulk_out_endpoint::~bulk_out_endpoint()
{
}

void usb::bulk_out_endpoint::driver_on_receive(
  [[maybe_unused]] hal::callback<void(std::span<hal::byte>)> p_callback)
{
  // ... fill this out ...
}
}  // namespace hal::stm32f1
