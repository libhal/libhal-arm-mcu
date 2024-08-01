#pragma once

#include <algorithm>
#include <array>

#include <libhal-armcortex/interrupt.hpp>
#include <libhal/units.hpp>

#include "interrupt_reg.hpp"
#include "system_controller_reg.hpp"

namespace hal::cortex_m {
template<typename T>
class stub_out_registers
{
public:
  stub_out_registers(T** p_register_pointer)
    : m_register_pointer(p_register_pointer)
    , m_original(nullptr)
    , m_stub{}
  {
    m_original = *m_register_pointer;
    *m_register_pointer = reinterpret_cast<T*>(m_stub.data());
  }

  stub_out_registers& operator=(stub_out_registers const&) = delete;
  stub_out_registers(stub_out_registers const&) = delete;
  stub_out_registers& operator=(stub_out_registers&& p_other) noexcept
  {
    m_register_pointer = p_other.m_register_pointer;
    m_original = p_other.m_original;

    std::copy(p_other.m_stub.begin(), p_other.m_stub.end(), m_stub.begin());

    // Point the global register pointer to the new object's moved stub
    *m_register_pointer = reinterpret_cast<T*>(m_stub.data());
    p_other.m_moved = true;

    return *this;
  }

  stub_out_registers(stub_out_registers&& p_other) noexcept
  {
    *this = std::move(p_other);
  }

  ~stub_out_registers()
  {
    if (not m_moved) {
      *m_register_pointer = m_original;
    }
  }

private:
  T** m_register_pointer;
  T* m_original;
  std::array<hal::byte, sizeof(T)> m_stub;
  bool m_moved = false;
};

inline void fake_top_of_stack()
{
  while (true) {
    continue;
  }
}

inline void fake_reset_handler()
{
  while (true) {
    continue;
  }
}

[[nodiscard]] inline auto setup_interrupts_for_unit_testing()
{
  static std::array<interrupt_pointer, 2> original_ivt{ fake_top_of_stack,
                                                        fake_reset_handler };
  auto stub_out_nvic = stub_out_registers(&nvic);
  auto stub_out_scb = stub_out_registers(&scb);
  scb->vtor = reinterpret_cast<std::intptr_t>(original_ivt.data());

  struct type_t
  {
    decltype(stub_out_nvic) remember_nvic;
    decltype(stub_out_scb) remember_scb;
  };

  return type_t{
    .remember_nvic = std::move(stub_out_nvic),
    .remember_scb = std::move(stub_out_scb),
  };
}
}  // namespace hal::cortex_m
