#include <array>
#include <chrono>
#include <cstdint>
#include <libhal-util/serial.hpp>
#include <libhal-util/spi.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/spi.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>
#include <resource_list.hpp>

// Include hal::literals to enable 2.0_MHz
using namespace hal::literals;

// --- Device Setup Constants ---
constexpr hal::hertz kDisplayClockRate = 2.0_MHz;
constexpr size_t kColumns = 128;
constexpr size_t kRows = 8;  // 8 pages of 8 pixels each

class Ssd1306 {
 private:
  hal::spi& m_spi;
  hal::output_pin& m_cs;
  hal::output_pin& m_dc;
  std::array<hal::byte, kColumns * kRows> m_buffer;

 public:
  // MODIFIED: Constructor only takes SPI, CS, and DC
  Ssd1306(hal::spi& p_spi, hal::output_pin& p_cs, hal::output_pin& p_dc)
      : m_spi(p_spi), m_cs(p_cs), m_dc(p_dc) {
    m_buffer.fill(0x00);
  }

  void initialize() {
    // The previous write_command(0xAE); write_command(0xD5);
    // write_command(0x80); block was redundant and has been simplified to match
    // the sjsu single pass.

    // 1. Single-Byte Commands
    write_command(0xAE);  // Display OFF (turn off oled panel)

    // 2. Two-Byte Commands (Command, Data)
    write_sequence(
        0xD5, 0x80);  // set display clock divide ratio/oscillator frequency
    write_sequence(0xA8, 0x3F);  // set multiplex ratio(1 to 64)
    write_sequence(0xD3, 0x00);  // set display offset = not offset

    // 3. Single-Byte Command
    write_command(0x40);  // Set display start line

    // 4. Two-Byte Command
    write_sequence(0x8D, 0x14);  // Disable Charge Pump (Enable later)

    // 5. Address Mode Setup (Crucial Multi-Byte Sequences)
    // NOTE: This uses the complete sequence from the sjsu's
    // SetHorizontalAddressMode()

    // Set Addressing mode = Horizontal Mode (0b00)
    write_sequence(0x20, 0x00);

    // Set Column Address Start/End (Column 0 to Column 127)
    write_triple_sequence(0x21, 0x00, 0x7F);

    // Set Page Address Start/End (Page 0 to Page 7)
    write_triple_sequence(0x22, 0x00, 0x07);

    // 6. Final Configuration Commands (Single and Two-Byte)
    write_command(0xA1);         // set segment re-map 128 to 0
    write_command(0xC8);         // Set COM Output Scan Direction 64 to 0
    write_sequence(0xDA, 0x12);  // set com pins hardware configuration
    write_sequence(0x81, 0xCF);  // set contrast control register
    write_sequence(0xD9, 0xF1);  // Set pre-charge period
    write_sequence(0xDB, 0x40);  // Set Vcomh
    write_command(0xA4);         // Enable entire display (Non-inverting)
    write_command(0xA6);         // Set display to normal colors (Non-inverting)
    write_command(0xAF);         // Set Display On
  }

void write_command(hal::byte p_command) {
    m_dc.level(false); // D/C LOW (Command)
    m_cs.level(false); // CS LOW (Start)
    
    hal::write(m_spi, std::span(&p_command, 1));
    
    m_cs.level(true);  // CS HIGH (End)
}

  void write_data(std::span<const hal::byte> p_data) {
    m_dc.level(true);   // HIGH for data
    m_cs.level(false);  // CS LOW (active)
    hal::write(m_spi, p_data);
    m_cs.level(true);  // CS HIGH (inactive)
  }

  void write_sequence(hal::byte p_cmd, hal::byte p_data) {
    std::array<hal::byte, 2> buffer = {p_cmd, p_data};

    m_dc.level(false);  // LOW for command
    m_cs.level(false);  // CS LOW (active)

    // Use hal::write to send both bytes under a single CS assertion
    hal::write(m_spi, buffer);

    m_cs.level(true);  // CS HIGH (inactive)
  }

  void write_triple_sequence(hal::byte p_cmd, hal::byte p_data1,
                             hal::byte p_data2) {
    // Buffer containing the command and its two data bytes
    std::array<hal::byte, 3> buffer = {p_cmd, p_data1, p_data2};

    m_dc.level(false);  // Set DC LOW for COMMAND mode
    m_cs.level(false);  // Assert CS LOW (start transaction)

    // Use hal::write to send all 3 bytes in a single SPI transaction
    // The LPC40 SPI driver handles keeping CS low during this call.
    hal::write(m_spi, buffer);

    m_cs.level(true);  // De-assert CS HIGH (end transaction)
  }

  void draw_pixel(int p_x, int p_y) {
    if (p_x < 0 || p_x >= (int)kColumns || p_y < 0 || p_y >= 64) {
      return;
    }

    uint32_t page = p_y / 8;
    uint32_t bit_pos = p_y % 8;
    size_t index = (page * kColumns) + p_x;

    m_buffer[index] |= (1 << bit_pos);
  }

  void clear() { m_buffer.fill(0x00); }

  void update() {
    // Set Column Address Start/End (0x21 0x00 0x7F) - MUST be one transaction
    // The previous implementation was: write_command(0x21); write_command(0x00); write_command(0x7F);
    write_triple_sequence(0x21, 0x00, 0x7F);

    // Set Page Address Start/End (0x22 0x00 0x07) - MUST be one transaction
    // The previous implementation was: write_command(0x22); write_command(0x00); write_command(0x07);
    write_triple_sequence(0x22, 0x00, 0x07);

    // Send Data (The entire buffer)
    // NOTE: write_data must be defined to handle the buffer!
    write_data(m_buffer); 
}
void toggle_pins(hal::steady_clock& p_clock) {
    using namespace std::chrono_literals;
    for (int i = 0; i < 5; ++i) {
        m_cs.level(false);
        m_dc.level(false);
        hal::delay(p_clock, 50ms);
        m_cs.level(true);
        m_dc.level(true);
        hal::delay(p_clock, 50ms);
    }
}
};

// Assuming the resource accessors for CS and DC are defined in the global scope
// (or the same namespace as application) and return strong_ptr.
hal::v5::strong_ptr<hal::output_pin> cs_pin();
hal::v5::strong_ptr<hal::output_pin> dc_pin();

// -----------------------------------------------------------------------------
// Application Function
// -----------------------------------------------------------------------------

void application() {
  using namespace std::chrono_literals;
  using namespace hal::literals;

  auto clock = resources::clock();
  auto console = resources::console();

  // 1. Acquire strong pointers locally (AVOIDS the "deleted function" error)
  auto spi_ptr = resources::spi();
  auto cs_ptr = resources::cs_pin();
  auto dc_ptr = resources::dc_pin();

  // 2. Get references by safely dereferencing the local pointers.
  hal::spi& spi = *spi_ptr;
  hal::output_pin& cs_pin_ref = *cs_ptr;
  hal::output_pin& dc_pin_ref = *dc_ptr;

  // Configure the SPI bus (Mode 0)
  hal::spi::settings display_spi_settings{
      .clock_rate =
          2.0_MHz,  // Use the literal now that hal::literals is active
      .clock_idles_high = true,
      .data_valid_on_trailing_edge = true};
  spi.configure(display_spi_settings);

  // 3. Construct and Initialize the SSD1306 driver
  // (Constructor only takes SPI, CS, DC)
  Ssd1306 display(spi, cs_pin_ref, dc_pin_ref);
  try {
    display.initialize();
    hal::print(*console, "SSD1306 initialized. Filling screen...\n");

    display.clear();
    
    // 1. Draw a 64x64 square in the top-left corner
    hal::print(*console, "Drawing 64x64 pixel square...\n");
    for (int y = 0; y < 64; y++) {
      for (int x = 0; x < 64; x++) {
        display.draw_pixel(x, y);
      }
    }
    
    display.update(); // Send the square to the screen
    hal::delay(*clock, 2000ms); // Wait 2 seconds

    // 2. Clear the screen and fill the entire 128x64 display
    hal::print(*console, "Clearing and filling entire screen...\n");
    display.clear();

    for (int y = 0; y < 64; y++) {
      for (int x = 0; x < 128; x++) {
        display.draw_pixel(x, y);
      }
    }
    
    display.update(); // Send the full screen to the display
    hal::delay(*clock, 2000ms); // Wait 2 seconds

    // 3. Clear the screen again
    hal::print(*console, "Clearing screen.\n");
    display.clear();
    display.update(); 
    
    hal::print(*console, "Screen fully tested. Entering main loop.\n");
  } catch (const hal::io_error&) {
    hal::print(*console, "Error initializing or writing to SSD1306 display.\n");
  }

  // Original loop logic
  while (true) {
    auto led = resources::status_led();
    auto button = resources::input_pin();

    if (button->level()) {
      led->level(false);
      hal::delay(*clock, 200ms);
      led->level(true);
      hal::delay(*clock, 200ms);
    }
  }
}
