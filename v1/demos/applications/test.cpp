#include <cstdint>

int main()
{
  constexpr std::uintptr_t digital_io_port_1 = 0x4000'4C00;

  constexpr std::uintptr_t pin_in_offset = 0x0;
  constexpr std::uintptr_t pin_out_offset = 0x2;
  constexpr std::uintptr_t pin_dir_offset = 0x4;

  constexpr std::uintptr_t pin_in_addr = digital_io_port_1 + pin_in_offset;
  constexpr std::uintptr_t pin_out_addr = digital_io_port_1 + pin_out_offset;
  constexpr std::uintptr_t pin_dir_addr = digital_io_port_1 + pin_dir_offset;

  auto* pin_in = reinterpret_cast<std::uint16_t volatile*>(pin_in_addr);
  auto* pin_out = reinterpret_cast<std::uint16_t volatile*>(pin_out_addr);
  auto* pin_dir = reinterpret_cast<std::uint16_t volatile*>(pin_dir_addr);

  *pin_dir = *pin_dir | (1 << 0);   // Pin P1.0 to output
  *pin_out = *pin_out | (1 << 0);   // Pin P1.0 set to HIGH voltage
  *pin_out = *pin_out & ~(1 << 0);  // Pin P1.0 set to LOW voltage
}
