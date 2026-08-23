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

export module arm_mcu_demos;

import hal;
import hal.arm_mcu;

export namespace resources {
/**
 * @brief Allocator for driver memory
 *
 * The expectation is that the implementation of this allocator is a
 * std::pmr::monotonic_buffer_resource with static memory storage, meaning the
 * memory is fixed in size and memory cannot be deallocated. This is fine for
 * the demos.
 *
 * @return std::pmr::polymorphic_allocator<>
 */
hal::allocator driver_allocator();
hal::ptr<hal::timed_interrupt> timer();
hal::ptr<hal::steady_clock> clock();
hal::ptr<hal::output_pin> status_led();
hal::ptr<hal::input_pin> input_pin();
hal::ptr<hal::serial> console();
hal::ptr<hal::adc16> adc();
hal::ptr<hal::can_transceiver> can_transceiver();
hal::ptr<hal::can_bus_manager> can_bus_manager();
hal::ptr<hal::can_id_filter> can_identifier_filter();

// USB resources are coroutines: acquiring the manager for the first time
// requires `co_await`-ing its power-up sequence (see hal::stm32f1::usb::
// create()).
async::future<hal::ptr<hal::usb::control_endpoint>> usb_control_endpoint(
  async::context&);
async::future<hal::ptr<hal::usb::bulk_out_endpoint>> usb_bulk_out_endpoint1(
  async::context&);
async::future<hal::ptr<hal::usb::bulk_in_endpoint>> usb_bulk_in_endpoint1(
  async::context&);
async::future<hal::ptr<hal::usb::interrupt_out_endpoint>>
usb_interrupt_out_endpoint1(async::context&);
async::future<hal::ptr<hal::usb::interrupt_in_endpoint>>
usb_interrupt_in_endpoint1(async::context&);

inline void reset()
{
  hal::cortex_m::reset();
}
}  // namespace resources

// Application function is implemented by one of the .cpp files.
export void initialize_platform();
export hal::task application(async::context&);
extern "C++"
{
  export extern hal::task task1(async::context&);
}
