#include <cstdint>

#include <bit>
#include <libhal/experimental/usb.hpp>
#include <memory>
#include <utility>

#include <libhal-arm-mcu/interrupt.hpp>
#include <libhal-arm-mcu/stm32f1/clock.hpp>
#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal-arm-mcu/stm32f1/interrupt.hpp>
#include <libhal-arm-mcu/stm32f1/pin.hpp>
#include <libhal-arm-mcu/stm32f1/usb.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/enum.hpp>
#include <libhal-util/math.hpp>
#include <libhal-util/static_callable.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/steady_clock.hpp>

#include "power.hpp"
#include "stm32f1/pin.hpp"

namespace hal::stm32f1 {

namespace {

enum class endpoint_type : u8
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
  [[maybe_unused]] static constexpr auto low_power_mode = bit_mask::from<2>();
  // FSUSP: Force Suspend
  [[maybe_unused]] static constexpr auto force_suspend = bit_mask::from<3>();
  // RESUME: Resume Request
  [[maybe_unused]] static constexpr auto resume_request = bit_mask::from<4>();
  // ESOFM: Expected Start Of Frame Interrupt Mask
  [[maybe_unused]] static constexpr auto expected_start_of_frame_interrupt =
    bit_mask::from<8>();
  // SOFM: Start Of Frame Interrupt Mask
  [[maybe_unused]] static constexpr auto start_of_frame_interrupt =
    bit_mask::from<9>();
  // RESETM: USB Reset Interrupt Mask
  static constexpr auto reset_interrupt = bit_mask::from<10>();
  // SUSPM: Suspend Mode Interrupt Mask
  static constexpr auto suspend_mode_interrupt = bit_mask::from<11>();
  // WKUPM: Wakeup Interrupt Mask
  static constexpr auto wakeup_interrupt = bit_mask::from<12>();
  // ERRM: Error Interrupt Mask
  [[maybe_unused]] static constexpr auto error_interrupt = bit_mask::from<13>();
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
  [[maybe_unused]] static constexpr auto expected_start_of_frame =
    bit_mask::from<8>();
  // SOF: Start Of Frame
  [[maybe_unused]] static constexpr auto start_of_frame = bit_mask::from<9>();
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
  [[maybe_unused]] static constexpr auto count = bit_mask::from<0, 10>();
  // LSOF: Lost SOF
  [[maybe_unused]] static constexpr auto lost_sof = bit_mask::from<11>();
  // LCK: Lock
  [[maybe_unused]] static constexpr auto lock = bit_mask::from<12>();
  // RXDM: Receive Data - Line Status
  [[maybe_unused]] static constexpr auto receive_data_status =
    bit_mask::from<13>();
  // RXDP: Transmit Data - Line Status
  [[maybe_unused]] static constexpr auto transmit_data_status =
    bit_mask::from<14>();
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
  [[maybe_unused]] static constexpr auto data_toggle_tx = bit_mask::from<6>();
  // CTR_TX: Correct Transfer for Transmission
  static constexpr auto correct_transfer_tx = bit_mask::from<7>();
  // EP_KIND: Endpoint Kind
  static constexpr auto kind = bit_mask::from<8>();
  // EP_TYPE: Endpoint Type
  static constexpr auto type = bit_mask::from<9, 10>();
  // SETUP: Setup Transaction Completed
  [[maybe_unused]] static constexpr auto setup_complete = bit_mask::from<11>();
  // STAT_RX: Status Bits, for reception transfers
  static constexpr auto status_rx = bit_mask::from<12, 13>();
  // DTOG_RX: Data Toggle, for reception transfers
  [[maybe_unused]] static constexpr auto data_toggle_rx = bit_mask::from<14>();
  // CTR_RX: Correct Transfer for Reception
  static constexpr auto correct_transfer_rx = bit_mask::from<15>();
};

struct block_table
{
  [[maybe_unused]] static constexpr std::array<u8, 2> block_size_table{
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
constexpr hal::u16 fixed_endpoint_size = 16;
constexpr auto endpoint_memory_size = fixed_endpoint_size;
constexpr auto block_number = fixed_endpoint_size / 2U;
constexpr auto rx_endpoint_count_mask =
  hal::bit_value(0U)
    .clear<block_table::block_size>()  // each number of blocks = 2x
    .insert<block_table::number_of_blocks>(block_number)
    .to<hal::u16>();

std::span<u8> usb_packet_buffer_sram()
{
  return { reinterpret_cast<u8*>(0x4000'6000), packet_buffer_sram_size };
}

std::span<hal::u32> usb_packet_buffer_sram_u32()
{
  // NOTE: dividing by the sizeof(hal::u16) is not a mistake. The memory is u16
  // addressable by the USB peripheral but u32 accessible/aligned for the
  // processor. This means that each u16 has padding of an additional u16 that
  // goes nowhere. The number of u16 blocks in this memory region is equal to
  // the u32 blocks and thats why we do the division here.
  return { reinterpret_cast<hal::u32*>(0x4000'6000),
           packet_buffer_sram_size / sizeof(hal::u16) };
}

constexpr hal::u16 tx_endpoint_memory_address(std::size_t p_endpoint)
{
  auto const offset = (p_endpoint * 2) * endpoint_memory_size;
  return buffer_descriptor_table_end + offset;
}

constexpr hal::u16 rx_endpoint_memory_address(std::size_t p_endpoint)
{
  auto const offset = ((p_endpoint * 2) + 1) * endpoint_memory_size;
  return buffer_descriptor_table_end + offset;
}

struct buffer_descriptor_block
{
  u32 tx_address;
  u32 tx_count;
  u32 rx_address;
  u32 rx_count;

  void setup_ctrl_descriptor()
  {
    tx_address = tx_endpoint_memory_address(0);
    tx_count = 0;

    rx_address = rx_endpoint_memory_address(0);
    rx_count = rx_endpoint_count_mask;
  }

  void setup_in_endpoint_for(u8 p_endpoint)
  {
    tx_address = tx_endpoint_memory_address(p_endpoint);
    tx_count = 0;
  }

  void setup_out_endpoint_for(u8 p_endpoint)
  {
    rx_address = rx_endpoint_memory_address(p_endpoint);
    rx_count = rx_endpoint_count_mask;
  }

  std::size_t bytes_received()
  {
    auto const bytes_received = hal::bit_extract<block_table::count>(rx_count);
    return bytes_received;
  }

  hal::u16 rx_address_offset()
  {
    return rx_address;
  }

  std::span<hal::u32> rx_span()
  {
    auto const offset = rx_address_offset() / sizeof(u16);
    auto const size = bytes_received() / 2;
    return usb_packet_buffer_sram_u32().subspan(offset, size);
  }

  hal::u16 tx_address_offset()
  {
    return tx_address;
  }

  std::span<hal::u32> tx_span()
  {
    auto const offset = tx_address_offset() / sizeof(u16);
    auto const size = fixed_endpoint_size / 2;
    return usb_packet_buffer_sram_u32().subspan(offset, size);
  }

  void set_count_tx(hal::u16 p_transfer_size)
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
enum class stat : u8
{
  // all reception requests addressed to this endpoint are ignored.
  disabled = 0b00,
  stall = 0b01,
  nak = 0b10,
  valid = 0b11,
};

/// Same stat value for rx and tex
enum class dtog : u8
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
  // hal::cortex_m::disable_all_interrupts();
  set_endpoint_register_toggle<endpoint::status_rx>(p_endpoint, p_stat);
  // hal::cortex_m::enable_all_interrupts();
}

void set_tx_stat(std::size_t p_endpoint, stat p_stat)
{
  // hal::cortex_m::disable_all_interrupts();
  set_endpoint_register_toggle<endpoint::status_tx>(p_endpoint, p_stat);
  // hal::cortex_m::enable_all_interrupts();
}

void set_endpoint_address_and_type(std::size_t p_endpoint, endpoint_type p_type)
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
void clear_correct_transfer_for(u8 endpoint_id)
{
  auto& endpoint_reg = reg().EP[endpoint_id].EPR;
  auto const endpoint_value = endpoint_reg;
  auto masked_value = endpoint_value & endpoint_invariant_mask;
  auto const masked_value_with_transfer_cleared =
    hal::bit_value(masked_value).template clear<mask>().template to<hal::u32>();
  endpoint_reg = masked_value_with_transfer_cleared;
}
}  // namespace

int error_detected_count = 0;

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

  endpoint_descriptor_block(0).setup_ctrl_descriptor();
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

// NOLINTNEXTLINE(bugprone-exception-escape)
void usb::interrupt_handler() noexcept
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
      clear_correct_transfer_for<endpoint::correct_transfer_tx>(endpoint_id);
    } else {
      // Call callback using variant visitor
      std::visit(
        [](auto&& p_callback) {
          using T = std::decay_t<decltype(p_callback)>;
          if constexpr (std::is_same_v<T,
                                       hal::callback<void(ctrl_receive_tag)>>) {
            p_callback(ctrl_receive_tag{});
          } else if constexpr (std::is_same_v<
                                 T,
                                 hal::callback<void(bulk_receive_tag)>>) {
            p_callback(bulk_receive_tag{});
          } else if constexpr (std::is_same_v<
                                 T,
                                 hal::callback<void(interrupt_receive_tag)>>) {
            p_callback(interrupt_receive_tag{});
          } else {
            auto constexpr is_same =
              not std::is_same_v<T, hal::callback<void(ctrl_receive_tag)>>;
            static_assert(hal::error::invalid_option<is_same>,
                          "USB RX Out callback vistor is non-exhaustive!");
          }
        },
        m_out_callbacks[endpoint_id]);

      clear_correct_transfer_for<endpoint::correct_transfer_rx>(endpoint_id);
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

usb::usb(hal::steady_clock& p_clock, hal::time_duration p_write_timeout)
  : m_clock(&p_clock)
  , m_write_timeout(p_write_timeout)
  , m_available_endpoint_memory(initial_packet_buffer_memory)
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

  using namespace std::chrono_literals;

  hal::delay(p_clock, 1ms);
  // Perform reset
  hal::bit_modify(reg().CNTR)
    .set(control::power_down)
    .set(control::force_reset);
  hal::delay(p_clock, 1ms);

  // The USB peripheral sets this bit on system reset, we need to clear it to
  // allow the device to power on. We must wait approximately 1us before we can
  // proceed for the stm32f103.
  hal::bit_modify(reg().CNTR).clear(control::power_down);
  hal::delay(p_clock, 1ms);

  // Clears everything including the force reset, enabling the USB device.
  reg().CNTR = 0;

  handle_bus_reset();
}

std::span<u8 const> usb::read_endpoint(u8 p_endpoint,
                                       std::span<u8> p_buffer,
                                       u16& p_bytes_read)
{
  auto const& ep = usb_reg->EP.at(p_endpoint);
  auto const endpoint_stat =
    static_cast<stat>(hal::bit_extract<endpoint::status_rx>(ep.EPR));

  // If the endpoint status is NAK, it means that we currently have a packet in
  // the USB SRAM, and we haven't consumed all of the memory yet. We
  // automatically move to VALID when all of the data has been consumed.
  if (endpoint_stat != stat::nak) {
    // return zero length buffer if the endpoint status is VALID meaning the
    // endpoint is ready to receive data but hasn't received any data yet.
    return p_buffer.first<0>();
  }

  auto& descriptor = endpoint_descriptor_block(p_endpoint);
  auto const bytes_remaining = descriptor.bytes_received() - p_bytes_read;

  auto const bytes_to_copy = std::min(bytes_remaining, p_buffer.size());
  // Offset the rx span by the number of bytes read divided by 2U (word size)
  auto const rx_span = descriptor.rx_span();

  for (size_t idx = 0; idx < bytes_to_copy; idx++) {
    auto const offset_index = p_bytes_read + idx;
    // Grab next word, divided by 2 to get the u16 word
    auto const value = rx_span[offset_index / 2U];
    // Determine the shift based on if the index is odd or not
    auto const shift = (offset_index & 1U) ? 8U : 0U;
    auto const byte = (value >> shift) & 0xFF;
    p_buffer[idx] = byte;
  }

  p_bytes_read += bytes_to_copy;

  if (p_bytes_read == descriptor.bytes_received()) {
    p_bytes_read = 0;
    set_rx_stat(p_endpoint, stat::valid);
  }

  return std::span(p_buffer).first(bytes_to_copy);
}

void usb::wait_for_endpoint_transfer_completion(u8 p_endpoint)
{
  constexpr auto nak_u32_value = static_cast<hal::u32>(stat::nak);
  auto& endpoint_reg = reg().EP[p_endpoint].EPR;
  auto endpoint_tx_status = hal::bit_extract<endpoint::status_tx>(endpoint_reg);
  auto const deadline = hal::future_deadline(*m_clock, m_write_timeout);
  while (endpoint_tx_status != nak_u32_value) {
    endpoint_tx_status = hal::bit_extract<endpoint::status_tx>(endpoint_reg);
    if (m_clock->uptime() >= deadline) {
      hal::safe_throw(hal::timed_out(this));
    }
  }
}

void usb::fill_endpoint(hal::u8 p_endpoint,
                        std::span<hal::byte const> p_data,
                        hal::u16 p_max_length)
{
  auto& descriptor = endpoint_descriptor_block(p_endpoint);
  auto const tx_span = descriptor.tx_span();
  while (not p_data.empty()) {
    while (descriptor.tx_count < p_max_length) {
      if (p_data.empty()) {
        return;
      }
      if (descriptor.tx_count & 0b1) {
        hal::bit_modify(tx_span[descriptor.tx_count / 2U])
          .insert<hal::byte_m<1>>(p_data[0]);
      } else {
        hal::bit_modify(tx_span[descriptor.tx_count / 2U])
          .insert<hal::byte_m<0>>(p_data[0]);
      }
      p_data = p_data.subspan(1);
      descriptor.tx_count++;
    }

    set_tx_stat(p_endpoint, stat::valid);
    wait_for_endpoint_transfer_completion(p_endpoint);
    descriptor.tx_count = 0;
  }

#if 0
  // FIX THIS needs to fill buffer not what it is doing now.
  while (not p_data.empty()) {
    auto tx_iter = tx_span.begin();
    auto const min = std::min(p_data.size(), static_cast<size_t>(p_max_length));

    descriptor.tx_count++;

    auto const even_min = min & ~1;
    for (std::size_t i = 0; i < even_min; i += 2) {
      u32 temp = (p_data[i + 1] << 8) | p_data[i];
      *(tx_iter++) = temp;
    }
    if (min & 1) {
      u32 temp = p_data[min - 1];
      *tx_iter = temp;
    }

    set_tx_stat(p_endpoint, stat::valid);
    wait_for_endpoint_transfer_completion(p_endpoint);

    data = data.subspan(min);
  }
#endif
}

void usb::write_to_endpoint(u8 p_endpoint,
                            std::span<std::span<hal::byte const>> p_data)
{
  // We use a copy and not the reference to not modify the span.
  for (auto const data : p_data) {
    fill_endpoint(p_endpoint, data, fixed_endpoint_size);
  }
}

void usb::flush_endpoint(u8 p_endpoint)
{
  // send whatever is in the USB buffer
  set_tx_stat(p_endpoint, stat::valid);
  wait_for_endpoint_transfer_completion(p_endpoint);
  endpoint_descriptor_block(p_endpoint).tx_count = 0;
}

usb::~usb()
{
  configure_pin({ .port = 'A', .pin = 11 }, input_pull_up);
  configure_pin({ .port = 'A', .pin = 12 }, input_pull_up);
  power_off(peripheral::usb);
  cortex_m::disable_interrupt(irq::usb_lp_can1_rx0);
  cortex_m::disable_interrupt(irq::usb_hp_can1_tx);
}

usb::manager usb::acquire_manager()
{
  return {};
}

usb::control_endpoint usb::acquire_control_endpoint()
{
  return { *this };
}

std::span<u8 const> usb::control_endpoint::driver_read(std::span<u8> p_buffer)
{
  auto const data_read = m_usb->read_endpoint(0, p_buffer, m_bytes_read);
  if (data_read.empty()) {
    set_rx_stat(0, stat::valid);
  }
  return data_read;
}

std::pair<usb::interrupt_out_endpoint, usb::interrupt_in_endpoint>
usb::acquire_interrupt_endpoint()
{
  auto const endpoint_assignment = m_endpoints_allocated++;
  return { { *this, endpoint_assignment }, { *this, endpoint_assignment } };
}

std::pair<usb::bulk_out_endpoint, usb::bulk_in_endpoint>
usb::acquire_bulk_endpoint()
{
  auto const endpoint_assignment = m_endpoints_allocated++;
  return { { *this, endpoint_assignment }, { *this, endpoint_assignment } };
}

usb::control_endpoint::control_endpoint(usb& p_usb)
  : m_usb(&p_usb)
{
  set_rx_stat(0, stat::valid);
  endpoint_descriptor_block(0).tx_count = 0;
}

usb::control_endpoint::~control_endpoint() = default;

void usb::manager::driver_connect(bool p_should_connect)
{
  hal::bit_modify(reg().DADDR)
    .insert<device_address::enable_function>(p_should_connect);
}

void usb::manager::driver_set_address(u8 p_address)
{
  hal::bit_modify(reg().DADDR)
    .set(device_address::enable_function)
    .insert<device_address::address>(p_address);
}

void usb::control_endpoint::driver_write(
  std::span<std::span<hal::byte const>> p_data)
{
  constexpr auto ctrl_endpoint = 0;

  set_rx_stat(ctrl_endpoint, stat::stall);
  set_tx_stat(ctrl_endpoint, stat::nak);

  m_usb->write_to_endpoint(ctrl_endpoint, p_data);
}

void usb::control_endpoint::driver_on_receive(
  hal::callback<void(on_receive_tag)> p_callback)
{
  m_usb->m_out_callbacks[0] = p_callback;
}

void usb::control_endpoint::driver_stall(bool p_should_stall)
{
  if (p_should_stall) {
    set_rx_stat(0, stat::stall);
    set_tx_stat(0, stat::stall);
  } else {
    set_rx_stat(0, stat::valid);
    set_tx_stat(0, stat::nak);
  }
}

// usb::interrupt_in_endpoint implementation
usb::interrupt_in_endpoint::interrupt_in_endpoint(usb& p_usb,
                                                  u8 p_endpoint_number)
  : m_usb(&p_usb)
  , m_endpoint_number(p_endpoint_number)
{
  reset();
}

void usb::interrupt_in_endpoint::reset()
{
  auto const endpoint = m_endpoint_number;
  endpoint_descriptor_block(endpoint).setup_in_endpoint_for(endpoint);
  set_endpoint_address_and_type(endpoint, endpoint_type::interrupt);
  set_rx_stat(0, stat::stall);
}

usb::interrupt_in_endpoint::~interrupt_in_endpoint() = default;

void usb::interrupt_in_endpoint::driver_stall(bool p_should_stall)
{
  if (p_should_stall) {
    set_tx_stat(m_endpoint_number, stat::stall);
  } else {
    set_tx_stat(m_endpoint_number, stat::nak);
  }
}

void usb::interrupt_in_endpoint::driver_write(
  std::span<std::span<hal::byte const>> p_data)
{
  m_usb->write_to_endpoint(m_endpoint_number, p_data);
}

// usb::bulk_in_endpoint implementation
usb::bulk_in_endpoint::bulk_in_endpoint(usb& p_usb, u8 p_endpoint_number)
  : m_usb(&p_usb)
  , m_endpoint_number(p_endpoint_number)
{
  reset();
}

void usb::bulk_in_endpoint::reset()
{
  auto const endpoint = m_endpoint_number;
  endpoint_descriptor_block(endpoint).setup_in_endpoint_for(endpoint);
  set_endpoint_address_and_type(endpoint, endpoint_type::bulk);
}

usb::bulk_in_endpoint::~bulk_in_endpoint() = default;

void usb::bulk_in_endpoint::driver_stall(bool p_should_stall)
{
  if (p_should_stall) {
    set_tx_stat(m_endpoint_number, stat::stall);
  } else {
    set_tx_stat(m_endpoint_number, stat::nak);
  }
}

void usb::bulk_in_endpoint::driver_write(
  std::span<std::span<hal::byte const>> p_data)
{
  m_usb->write_to_endpoint(m_endpoint_number, p_data);
}

// usb::interrupt_out_endpoint implementation
usb::interrupt_out_endpoint::interrupt_out_endpoint(usb& p_usb,
                                                    u8 p_endpoint_number)
  : m_usb(&p_usb)
  , m_endpoint_number(p_endpoint_number)
{
  reset();
}

void usb::interrupt_out_endpoint::reset()
{
  auto const endpoint = m_endpoint_number;
  endpoint_descriptor_block(endpoint).setup_out_endpoint_for(endpoint);
  set_endpoint_address_and_type(endpoint, endpoint_type::interrupt);
  set_rx_stat(m_endpoint_number, stat::stall);
}

usb::interrupt_out_endpoint::~interrupt_out_endpoint() = default;

void usb::interrupt_out_endpoint::driver_stall(bool p_should_stall)
{
  if (p_should_stall) {
    set_rx_stat(m_endpoint_number, stat::stall);
  } else {
    set_rx_stat(m_endpoint_number, stat::nak);
  }
}

void usb::interrupt_out_endpoint::driver_on_receive(
  hal::callback<void(on_receive_tag)> p_callback)
{
  m_usb->m_out_callbacks[m_endpoint_number] = p_callback;
}

std::span<u8 const> usb::interrupt_out_endpoint::driver_read(
  std::span<u8> p_buffer)
{
  return m_usb->read_endpoint(m_endpoint_number, p_buffer, m_bytes_read);
}

// usb::bulk_out_endpoint implementation
usb::bulk_out_endpoint::bulk_out_endpoint(usb& p_usb, u8 p_endpoint_number)
  : m_usb(&p_usb)
  , m_endpoint_number(p_endpoint_number)
{
  reset();
}

void usb::bulk_out_endpoint::reset()
{
  auto const endpoint = m_endpoint_number;
  endpoint_descriptor_block(endpoint).setup_out_endpoint_for(endpoint);
  set_endpoint_address_and_type(endpoint, endpoint_type::bulk);
  set_rx_stat(m_endpoint_number, stat::valid);
}

usb::bulk_out_endpoint::~bulk_out_endpoint() = default;

void usb::bulk_out_endpoint::driver_stall(bool p_should_stall)
{
  if (p_should_stall) {
    set_rx_stat(m_endpoint_number, stat::stall);
  } else {
    set_rx_stat(m_endpoint_number, stat::nak);
  }
}

void usb::bulk_out_endpoint::driver_on_receive(
  hal::callback<void(on_receive_tag)> p_callback)
{
  m_usb->m_out_callbacks[m_endpoint_number] = p_callback;
}

std::span<u8 const> usb::bulk_out_endpoint::driver_read(std::span<u8> p_buffer)
{
  return m_usb->read_endpoint(m_endpoint_number, p_buffer, m_bytes_read);
}

hal::experimental::usb_endpoint_info usb::control_endpoint::driver_info() const
{
  return { .size = fixed_endpoint_size, .number = 0, .stalled = false };
}

hal::experimental::usb_endpoint_info usb::bulk_out_endpoint::driver_info() const
{
  return {
    .size = fixed_endpoint_size,
    .number = m_endpoint_number,
    .stalled = stalled(),
  };
}

hal::experimental::usb_endpoint_info usb::bulk_in_endpoint::driver_info() const
{
  return {
    .size = fixed_endpoint_size,
    .number = static_cast<u8>(m_endpoint_number | (1U << 7U)),
    .stalled = stalled(),
  };
}

hal::experimental::usb_endpoint_info usb::interrupt_out_endpoint::driver_info()
  const
{
  return {
    .size = fixed_endpoint_size,
    .number = m_endpoint_number,
    .stalled = stalled(),
  };
}

hal::experimental::usb_endpoint_info usb::interrupt_in_endpoint::driver_info()
  const
{
  return {
    .size = fixed_endpoint_size,
    .number = static_cast<u8>(m_endpoint_number | (1U << 7U)),
    .stalled = stalled(),
  };
}

bool usb::bulk_out_endpoint::stalled() const
{
  return hal::bit_extract<endpoint::status_rx>(
           usb_reg->EP[m_endpoint_number].EPR) <= 0b01;
}

bool usb::interrupt_out_endpoint::stalled() const
{
  return hal::bit_extract<endpoint::status_rx>(
           usb_reg->EP[m_endpoint_number].EPR) <= 0b01;
}

bool usb::bulk_in_endpoint::stalled() const
{
  return hal::bit_extract<endpoint::status_tx>(
           usb_reg->EP[m_endpoint_number].EPR) <= 0b01;
}

bool usb::interrupt_in_endpoint::stalled() const
{
  return hal::bit_extract<endpoint::status_tx>(
           usb_reg->EP[m_endpoint_number].EPR) <= 0b01;
}

void usb::control_endpoint::driver_flush()
{
  m_usb->flush_endpoint(0);
  set_tx_stat(0, stat::stall);
  set_rx_stat(0, stat::valid);
}

void usb::bulk_in_endpoint::driver_flush()
{
  m_usb->flush_endpoint(m_endpoint_number);
}

void usb::interrupt_in_endpoint::driver_flush()
{
  m_usb->flush_endpoint(m_endpoint_number);
}
}  // namespace hal::stm32f1
