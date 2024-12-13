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

#pragma once

#include <array>
#include <cstdint>

namespace hal::stm32f1 {
struct flash_t
{
  std::uint32_t volatile acr;
  std::uint32_t volatile keyr;
  std::uint32_t volatile optkeyr;
  std::uint32_t volatile sr;
  std::uint32_t volatile cr;
  std::uint32_t volatile ar;
  std::uint32_t volatile reserved;
  std::uint32_t volatile obr;
  std::uint32_t volatile wrpr;
  std::array<uint32_t, 8> reserved1;
  std::uint32_t volatile keyr2;
  uint32_t reserved2;
  std::uint32_t volatile sr2;
  std::uint32_t volatile cr2;
  std::uint32_t volatile ar2;
};

/// Pointer to the flash control register
inline flash_t* flash = reinterpret_cast<flash_t*>(0x4002'2000);
}  // namespace hal::stm32f1
