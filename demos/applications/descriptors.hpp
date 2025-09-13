#pragma once

/* TODO:
  Class, subclass, proto validator
  Device qualifer descriptor (happens between device and config)
  Other speed descriptor (happens with configuration)
*/

#include <array>
#include <libhal-util/serial.hpp>
#include <libhal/error.hpp>
#include <libhal/pointers.hpp>
#include <libhal/serial.hpp>
#include <libhal/usb.hpp>
#include <memory_resource>
#include <span>
#include <string_view>

#include "libhal-util/as_bytes.hpp"
#include "libhal/units.hpp"
#include "utils.hpp"
#include <libhal-util/serial.hpp>
#include <vector>

namespace hal::v5::usb {

struct device
{
  template<size_t>
  friend class enumerator;

  struct device_arguments
  {
    u16 p_bcd_usb;
    class_code p_device_class;
    u8 p_device_subclass;
    u8 p_device_protocol;
    u16 p_id_vendor;
    u16 p_id_product;
    u16 p_bcd_device;
    std::u16string_view p_manufacturer;
    std::u16string_view p_product;
    std::u16string_view p_serial_number_str;
  };

  constexpr device(device_arguments&& args)
    : manufacturer_str(args.p_manufacturer)
    , product_str(args.p_product)
    , serial_number_str(args.p_serial_number_str)
  {
    u8 idx = 0;
    auto bcd_usb_bytes = hal::as_bytes(&args.p_bcd_usb, 1);
    for (auto& bcd_usb_byte : bcd_usb_bytes) {
      m_packed_arr[idx++] = bcd_usb_byte;  // 0, 1
    }

    m_packed_arr[idx++] = static_cast<u8>(args.p_device_class);  // 2
    m_packed_arr[idx++] = args.p_device_subclass;                // 3
    m_packed_arr[idx++] = args.p_device_protocol;                // 4

    m_packed_arr[idx++] = 0;  // 5 Max Packet length handled by the enumerator

    auto id_vendor_bytes = hal::as_bytes(&args.p_id_vendor, 1);
    for (auto& id_vendor_byte : id_vendor_bytes) {
      m_packed_arr[idx++] = id_vendor_byte;  // 6, 7
    }

    auto id_product_bytes = hal::as_bytes(&args.p_id_product, 1);
    for (auto& id_product_byte : id_product_bytes) {
      m_packed_arr[idx++] = id_product_byte;  // 8, 9
    }

    auto bcd_device_bytes = hal::as_bytes(&args.p_bcd_device, 1);
    for (auto& bcd_device_byte : bcd_device_bytes) {
      m_packed_arr[idx++] = bcd_device_byte;  // 10, 11
    }

    // Evaluated during enumeration
    m_packed_arr[idx++] = 0;  // 12 string idx of manufacturer
    m_packed_arr[idx++] = 0;  // 13 string idx of product
    m_packed_arr[idx++] = 0;  // 14 string idx of serial number
    m_packed_arr[idx++] = 0;  // 15 Number of possible configurations
  };

  u16& bcd_usb()
  {
    return *reinterpret_cast<u16*>(&m_packed_arr[0]);
  }

  constexpr u8& device_class()
  {
    return m_packed_arr[2];
  }

  constexpr u8& device_sub_class()
  {
    return m_packed_arr[3];
  }

  constexpr u8& device_protocol()
  {
    return m_packed_arr[4];
  }

  u16& id_vendor()
  {
    return *reinterpret_cast<u16*>(&m_packed_arr[6]);
  }

  u16& id_product()
  {
    return *reinterpret_cast<u16*>(&m_packed_arr[8]);
  }

  u16& bcd_device()
  {
    return *reinterpret_cast<u16*>(&m_packed_arr[10]);
  }

  operator std::span<byte const>() const
  {
    return m_packed_arr;
  }

  std::u16string_view manufacturer_str;
  std::u16string_view product_str;
  std::u16string_view serial_number_str;

private:
  constexpr u8& max_packet_size()
  {
    return m_packed_arr[5];
  }

  constexpr u8& manufacturer_index()
  {
    return m_packed_arr[12];
  }
  constexpr u8& product_index()
  {
    return m_packed_arr[13];
  }
  constexpr u8& serial_number_index()
  {
    return m_packed_arr[14];
  }
  constexpr u8& num_configurations()
  {
    return m_packed_arr[15];
  }

  std::array<hal::byte, 16> m_packed_arr;
};

// https://www.beyondlogic.org/usbnutshell/usb5.shtml#ConfigurationDescriptors

template<typename T>
concept usb_interface_concept = std::derived_from<T, interface>;

// Calculate: total length, number of interfaces, configuration value
struct configuration
{

  template<size_t>
  friend class enumerator;

  struct bitmap
  {

    constexpr bitmap(u8 p_bitmap)
      : m_bitmap(p_bitmap)
    {
    }

    constexpr bitmap(bool p_self_powered, bool p_remote_wakeup)
    {
      m_bitmap = (1 << 7) | (p_self_powered << 6) | (p_remote_wakeup << 5);
    }

    constexpr u8 to_byte()
    {
      return m_bitmap;
    }

    [[nodiscard]] constexpr bool self_powered() const
    {
      return static_cast<bool>(m_bitmap & 1 << 6);
    }

    [[nodiscard]] constexpr bool remote_wakeup() const
    {
      return static_cast<bool>(m_bitmap & 1 << 5);
    }

  private:
    u8 m_bitmap;
  };

  template<usb_interface_concept... Interfaces>
  constexpr configuration(std::u16string_view p_name,
                          bitmap&& p_attributes,
                          u8&& p_max_power,
                          std::pmr::polymorphic_allocator<> p_allocator,
                          strong_ptr<Interfaces>... p_interfaces)
    : name(p_name)
    , interfaces(p_allocator)
  {
    interfaces.reserve(sizeof...(p_interfaces));
    (interfaces.push_back(p_interfaces), ...);
    u8 idx = 0;

    // Anything marked with 0 is to be populated at enumeration time
    m_packed_arr[idx++] = 0;  // 0 Total Length
    m_packed_arr[idx++] = 0;
    m_packed_arr[idx++] = interfaces.size();  // 2 number of interfaces
    m_packed_arr[idx++] = 0;                  // 3 Config number
    m_packed_arr[idx++] = 0;  // 4 Configuration name string index

    m_packed_arr[idx++] = p_attributes.to_byte();  // 5
    m_packed_arr[idx++] = p_max_power;             // 6
  }

  operator std::span<u8 const>() const
  {
    return m_packed_arr;
  }

  constexpr bitmap attributes()
  {
    return { m_packed_arr[5] };
  }
  constexpr u8& attributes_byte()
  {
    return m_packed_arr[5];
  }

  constexpr u8& max_power()
  {
    return m_packed_arr[6];
  }

  std::u16string_view name;
  std::pmr::vector<strong_ptr<interface>> interfaces;

  // private:
  u16& total_length()
  {
    return *reinterpret_cast<u16*>(&m_packed_arr[0]);
  }
  constexpr u8& num_interfaces()
  {
    return m_packed_arr[2];
  }
  constexpr u8& configuration_value()
  {
    return m_packed_arr[3];
  }
  constexpr u8& configuration_index()
  {
    return m_packed_arr[4];
  }

  std::array<hal::byte, 7> m_packed_arr;
};
}  // namespace hal::v5::usb
