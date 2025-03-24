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

#include <libhal/units.hpp>

namespace hal::stm32f1 {
struct flash_t
{
  u32 volatile acr;
  u32 volatile keyr;
  u32 volatile optkeyr;
  u32 volatile sr;
  u32 volatile cr;
  u32 volatile ar;
  u32 volatile reserved;
  u32 volatile obr;
  u32 volatile wrpr;
  std::array<u32, 8> reserved1;
  u32 volatile keyr2;
  u32 reserved2;
  u32 volatile sr2;
  u32 volatile cr2;
  u32 volatile ar2;
};

/// Pointer to the flash control register
inline flash_t* flash = reinterpret_cast<flash_t*>(0x4002'2000);
}  // namespace hal::stm32f1
