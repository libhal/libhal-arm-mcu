#pragma once

#include <array>
#include <cstddef>
#include <cstdlib>
#include <cwchar>
#include <libhal-util/as_bytes.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/usb/endpoints.hpp>
#include <libhal/error.hpp>
#include <libhal/pointers.hpp>
#include <libhal/scatter_span.hpp>
#include <libhal/serial.hpp>
#include <libhal/units.hpp>
#include <libhal/usb.hpp>
#include <optional>
#include <span>
#include <string_view>
#include <utility>

#include "descriptors.hpp"
#include "utils.hpp"

// TODO: move to util
namespace hal::v5::usb {

void print_payload(strong_ptr<hal::serial> const& p_console,
                   scatter_span<byte const> p_payload)
{
  hal::print(*p_console, "Payload: [ ");
  for (auto const& s : p_payload) {
    for (unsigned char el : s) {
      hal::print<10>(*p_console, "0x%X, ", el);
    }
  }
  hal::print(*p_console, "]\n");
}

template<typename T>
constexpr size_t scatter_span_size(scatter_span<T> ss)
{
  size_t res = 0;
  for (auto const& s : ss) {
    res += s.size();
  }

  return res;
}

template<typename T, typename... Args>
constexpr std::pair<std::array<std::span<T>, sizeof...(Args)>, size_t>
make_sub_scatter_array(size_t p_count, Args&&... p_spans)
{
  std::array<std::span<T>, sizeof...(Args)> full_ss{ std::span<T>(
    std::forward<Args>(p_spans))... };

  size_t total_span_len = scatter_span_size(scatter_span<T>(full_ss));
  std::array<std::span<T>, sizeof...(Args)> res;
  std::array<size_t, sizeof...(Args)> lens{ std::span<T>(p_spans).size()... };

  if (total_span_len <= p_count) {
    return std::make_pair(full_ss, full_ss.size());
  }
  size_t cur_len = 0;
  size_t i = 0;
  for (; i < lens.size(); i++) {
    auto l = lens[i];

    if (p_count >= (cur_len + l)) {
      res[i] = full_ss[i];
      cur_len += l;
      continue;
    }

    if (cur_len >= p_count) {
      return std::make_pair(res, i);
    }

    auto delta = p_count - cur_len;
    std::span<T> subspan = std::span(full_ss[i]).first(delta);
    res[i] = subspan;
    break;
  }

  return std::make_pair(res, i + 1);
}

template<typename... Args>
constexpr std::pair<std::array<std::span<byte const>, sizeof...(Args)>, size_t>
make_sub_scatter_bytes(size_t p_count, Args&&... p_spans)
{
  return make_sub_scatter_array<byte const>(p_count,
                                            std::forward<Args>(p_spans)...);
}

template<size_t num_configs>
class enumerator
{

public:
  enumerator(
    strong_ptr<control_endpoint> const& p_ctrl_ep,
    strong_ptr<device> const& p_device,
    strong_ptr<std::array<configuration, num_configs>> const& p_configs,
    u16 p_lang_str,  // NOLINT
    u8 p_starting_str_idx,
    hal::v5::strong_ptr<hal::serial> const& p_console,
    bool enumerate_immediately = true)
    : m_ctrl_ep(p_ctrl_ep)
    , m_device(p_device)
    , m_configs(p_configs)
    , m_lang_str(p_lang_str)
    , m_console(p_console)
  {
    // Verify there is space to actually allocate indexes for configuration
    // Three string indexes are reserved for the device descriptor, then each
    // configuration has a name which reserves a string index strings and
    // Index 0 is reserved for the lang string
    if (p_starting_str_idx < 1 || p_starting_str_idx > 0xFF - 3 + num_configs) {
      safe_throw(hal::argument_out_of_domain(this));
    }
    m_starting_str_idx = p_starting_str_idx;

    if (enumerate_immediately) {
      enumerate();
    }
  }

  void enumerate()
  {
    // Renumerate, a config will only be set if
    if (m_active_conf != nullptr) {
      m_active_conf = nullptr;
      m_ctrl_ep->connect(false);
    }

    auto cur_str_idx = m_starting_str_idx;
    byte cur_iface_idx = 0;
    // Phase one: Preperation

    // Device
    m_device->manufacturer_index() = cur_str_idx++;
    m_device->product_index() = cur_str_idx++;
    m_device->serial_number_index() = cur_str_idx++;
    m_device->num_configurations() = num_configs;
    hal::print(*m_console, "Device desc: set up\n");

    // Configurations
    for (size_t i = 0; i < num_configs; i++) {
      configuration& config = m_configs->at(i);
      config.configuration_index() = cur_str_idx++;
      config.configuration_value() = i;
    }
    hal::print(*m_console, "conf desc: set up\n");

    for (configuration& config : *m_configs) {
      auto total_length = static_cast<u16>(constants::config_desc_size);
      for (auto const& iface : config.interfaces) {
        interface::descriptor_count deltas = iface->write_descriptors(
          { .interface = cur_iface_idx, .string = cur_str_idx },
          [&total_length](scatter_span<hal::byte const> p_data) {
            total_length += scatter_span_size(p_data);
          });

        cur_iface_idx += deltas.interface;
        cur_str_idx += deltas.string;
      }
      config.total_length() = total_length;
    }
    hal::print(*m_console, "iface/conf desc: set up\n");

    // Phase two: Writing

    // TODO: Make async
    bool finished_enumeration = false;
    bool volatile waiting_for_data = true;

    using on_receive_tag = control_endpoint::on_receive_tag;
    hal::print(*m_console, "Begin writing\n");

    m_ctrl_ep->on_receive(
      [&waiting_for_data](on_receive_tag) { waiting_for_data = false; });
    m_ctrl_ep->connect(true);
    hal::print(*m_console, "Connected\n");
    std::array<hal::byte, constants::size_std_req> raw_req;
    do {
      // Seriously, make this async
      while (waiting_for_data) {
        continue;
      }
      waiting_for_data = true;

      auto scatter_raw_req = make_writable_scatter_bytes(raw_req);
      auto num_bytes_read = m_ctrl_ep->read(scatter_raw_req);

      if (num_bytes_read == 0) {
        continue;
      }

      if (num_bytes_read != constants::size_std_req) {
        hal::print<24>(*m_console, "num_bytes_read: %X\n", num_bytes_read);
        safe_throw(hal::message_size(num_bytes_read, this));
      }
      // hal::print(*m_console, "Raw request: [");
      // for (auto const& el : raw_req) {
      //   hal::print<8>(*m_console, "0x%X, ", el);
      // }
      // hal::print(*m_console, "]\n");
      auto req = from_span(raw_req);
      hal::print<48>(*m_console,
                     "\nGot request: [0x%X, 0x%X, 0x%X, 0x%X, 0x%X]\n",
                     req.request_type,
                     req.request,
                     req.value,
                     req.index,
                     req.length);

      if (req.get_recipient() != setup_packet::recipient::device) {
        safe_throw(hal::not_connected(this));
      }

      // TODO: Handle exception
      handle_standard_device_request(req);
      // m_ctrl_ep->write({});  // Send ZLP to complete Data Transaction
      if (static_cast<standard_request_types>(req.request) ==
          standard_request_types::set_configuration) {
        finished_enumeration = true;
        m_ctrl_ep->on_receive(
          [this](on_receive_tag) { m_has_setup_packet = true; });
      }
    } while (!finished_enumeration);
  }

  [[nodiscard]] configuration& get_active_configuration()
  {
    if (m_active_conf == nullptr) {
      safe_throw(hal::operation_not_permitted(this));
    }

    return *m_active_conf;
  }

  void resume_ctrl_transaction()
  {
    while (!m_has_setup_packet) {
      continue;
    }

    std::array<byte, 8> read_buf;
    auto scatter_read_buf = make_writable_scatter_bytes(read_buf);
    auto bytes_read = m_ctrl_ep->read(scatter_read_buf);
    std::span payload(read_buf.data(), bytes_read);

    setup_packet req = from_span(payload);
    if (determine_standard_request(req) == standard_request_types::invalid) {
      return;
    }

    if (determine_standard_request(req) ==
          standard_request_types::get_descriptor &&
        static_cast<descriptor_type>((req.value & 0xFF << 8) >> 8) ==
          descriptor_type::string) {
      handle_str_descriptors(req.value & 0xFF, req.length > 2);

    } else if (req.get_recipient() == setup_packet::recipient::device) {
      handle_standard_device_request(req);
    } else {
      // Handle iface level requests
      auto f = [this](scatter_span<hal::byte const> resp) {
        m_ctrl_ep->write(resp);
      };
      bool req_handled = false;
      for (auto const& iface : get_active_configuration()) {
        req_handled = iface->handle_request(
          req.request_type, req.request, req.value, req.index, req.length, f);
        if (req_handled) {
          break;
        }
      }
      m_ctrl_ep->write(
        {});  // A ZLP to terminate Data Transaction just to be safe

      if (!req_handled) {
        safe_throw(hal::argument_out_of_domain(this));
      }
    }
  }

private:
  void handle_standard_device_request(setup_packet& req)
  {

    switch (determine_standard_request(req)) {
      case standard_request_types::set_address: {
        m_ctrl_ep->write({});
        m_ctrl_ep->set_address(req.value);
        hal::print<16>(*m_console, "Address set: %x\n", req.value);
        break;
      }

      case standard_request_types::get_descriptor: {
        process_get_descriptor(req);
        break;
      }

      case standard_request_types::get_configuration: {
        if (m_active_conf == nullptr) {
          safe_throw(hal::operation_not_permitted(this));
        }
        auto scatter_conf = make_scatter_bytes(
          std::span(&m_active_conf->configuration_value(), 1));
        m_ctrl_ep->write(scatter_conf);
        break;
      }

      case standard_request_types::set_configuration: {
        m_active_conf = &(m_configs->at(req.value));
        break;
      }

      case standard_request_types::invalid:
      default:
        safe_throw(hal::not_connected(this));
    }
  }

  void process_get_descriptor(setup_packet& req)
  {
    hal::byte desc_type = (req.value & 0xFF << 8) >> 8;
    [[maybe_unused]] hal::byte desc_idx = req.value & 0xFF;

    switch (static_cast<descriptor_type>(desc_type)) {
      case descriptor_type::device: {
        auto header =
          std::to_array({ constants::device_desc_size,
                          static_cast<byte>(descriptor_type::device) });
        m_device->max_packet_size() = static_cast<byte>(m_ctrl_ep->info().size);
        auto scatter_arr_pair =
          make_sub_scatter_bytes(req.length, header, *m_device);
        hal::v5::write_and_flush(
          *m_ctrl_ep,
          scatter_span<byte const>(scatter_arr_pair.first)
            .first(scatter_arr_pair.second));
        hal::print(*m_console, "Dev desc written\n");
        print_payload(m_console,
                      scatter_span<byte const>(scatter_arr_pair.first)
                        .first(scatter_arr_pair.second));
        break;
      }

      case descriptor_type::configuration: {
        configuration& conf = m_configs->at(desc_idx);
        auto conf_hdr =
          std::to_array({ constants::config_desc_size,
                          static_cast<byte>(descriptor_type::configuration) });
        auto scatter_conf_pair = make_sub_scatter_bytes(
          req.length, conf_hdr, static_cast<std::span<u8 const>>(conf));

        m_ctrl_ep->write(scatter_span<byte const>(scatter_conf_pair.first)
                           .first(scatter_conf_pair.second));
        hal::print(*m_console, "conf desc header written\n");

        // Return early if the only thing requested was the config descriptor
        print_payload(m_console,
                      scatter_span<byte const>(scatter_conf_pair.first)
                        .first(scatter_conf_pair.second));
        if (req.length <= constants::config_desc_size) {
          // print_payload(m_console,
          //               scatter_span<byte const>(scatter_conf_pair.first)
          //                 .first(scatter_conf_pair.second));
          m_ctrl_ep->write({});
          return;
        }

        u16 total_size = constants::config_desc_size;
        for (auto const& iface : conf.interfaces) {
          std::ignore = iface->write_descriptors(
            { .interface = std::nullopt, .string = std::nullopt },
            [this, &total_size](scatter_span<hal::byte const> byte_stream) {
              hal::v5::write_and_flush(*this->m_ctrl_ep, byte_stream);
              total_size += scatter_span_size(byte_stream);
              hal::print(*this->m_console, "Iface sent\n");
              print_payload(this->m_console, byte_stream);
            });
        }
        hal::print(*m_console, "iface descs written\n");

        // if (total_size != req.length) {
        //   safe_throw(hal::operation_not_supported(
        //     this));  // TODO: Make specific exception for this
        // }

        break;
      }

      case descriptor_type::string: {
        if (desc_idx == 0) {
          hal::print<32>(*m_console, "Lang str should be: 0x%x\n", m_lang_str);
          auto s_hdr =
            std::to_array({ static_cast<byte>(4),
                            static_cast<byte>(descriptor_type::string) });
          auto lang = setup_packet::to_le_bytes(m_lang_str);
          auto scatter_arr_pair = make_scatter_bytes(s_hdr, lang);
          // auto p = scatter_span<byte const>(scatter_arr_pair.first)
          //            .first(scatter_arr_pair.second);
          m_ctrl_ep->write(scatter_arr_pair);
          m_ctrl_ep->write({});
          print_payload(m_console, scatter_arr_pair);
          hal::print(*m_console, "lang string written\n");
          break;
        }
        handle_str_descriptors(desc_idx, req.length);  // Can throw
        break;
      }

        // TODO: Interface, endpoint, device_qualifier, interface_power,
        // OTHER_SPEED_CONFIGURATION

      default:
        safe_throw(hal::operation_not_supported(this));
    }
  }

  void handle_str_descriptors(u8 const target_idx, u16 p_len)
  {

    u8 cfg_string_end = m_starting_str_idx + 3 + num_configs;
    if (target_idx <= cfg_string_end) {
      auto r = write_cfg_str_descriptor(target_idx, p_len);
      if (!r) {
        safe_throw(hal::argument_out_of_domain(this));
      }
      m_iface_for_str_desc = std::nullopt;
      return;
    }

    if (m_iface_for_str_desc.has_value() &&
        m_iface_for_str_desc->first == target_idx) {
      bool success = m_iface_for_str_desc->second->write_string_descriptor(
        target_idx, [this](scatter_span<hal::byte const> desc) {
          hal::v5::write_and_flush(*m_ctrl_ep, desc);
        });
      if (success) {
        return;
        hal::print(*m_console, "Wrote cached string\n");
      }
    }

    // Iterate through every interface now to find a match
    auto f = [this, p_len](scatter_span<hal::byte const> desc) {
      if (p_len > 2) {
        hal::v5::write_and_flush(*m_ctrl_ep, desc);
      } else {
        std::array<hal::byte const, 1> desc_type{ static_cast<hal::byte>(
          descriptor_type::string) };
        auto scatter_str_hdr =
          make_scatter_bytes(std::span(&desc[0][0], 1), desc_type);
        hal::v5::write_and_flush(*m_ctrl_ep, scatter_str_hdr);
      }
    };

    if (m_active_conf != nullptr) {
      for (auto const& iface : m_active_conf->interfaces) {
        auto res = iface->write_string_descriptor(target_idx, f);
        if (res) {
          return;
        }
      }
    }

    for (configuration const& conf : *m_configs) {
      for (auto const& iface : conf.interfaces) {
        auto res = iface->write_string_descriptor(target_idx, f);
        if (res) {
          break;
        }
      }
    }
    hal::print(*m_console, "Wrote iface string\n");
  }

  bool write_cfg_str_descriptor(u8 const target_idx, u16 p_len)
  {
    constexpr u8 dev_manu_offset = 0;
    constexpr u8 dev_prod_offset = 1;
    constexpr u8 dev_sn_offset = 2;
    std::optional<std::u16string_view> opt_conf_sv;
    if (target_idx == (m_starting_str_idx + dev_manu_offset)) {
      opt_conf_sv = m_device->manufacturer_str;

    } else if (target_idx == (m_starting_str_idx + dev_prod_offset)) {
      opt_conf_sv = m_device->product_str;

    } else if (target_idx == (m_starting_str_idx + dev_sn_offset)) {
      opt_conf_sv = m_device->serial_number_str;

    } else {
      for (size_t i = 0; i < m_configs->size(); i++) {
        configuration const& conf = m_configs->at(i);
        if (target_idx == (m_starting_str_idx + i)) {
          opt_conf_sv = conf.name;
        }
      }
    }

    if (opt_conf_sv == std::nullopt) {
      return false;
    }

    // Acceptable to access without checking because guaranteed to be Some,
    // there is no pattern matching in C++ yet so unable to do this cleanly
    // (would require a check on every single one)
    hal::print<8>(*m_console, "%d: [", target_idx);
    auto sv = opt_conf_sv.value();
    std::mbstate_t state{};
    for (wchar_t const wc : sv) {
      char tmp[MB_CUR_MAX];  // NOLINT
      size_t len = std::wcrtomb(tmp, wc, &state);
      if (len == static_cast<size_t>(-1)) {
        hal::print(*m_console, "unable to convert wc\n");
        continue;
      }

      for (size_t i = 0; i < len; i++) {
        hal::print<8>(*m_console, "%c", tmp[i]);
      }
    }
    hal::print(*m_console, "]\n");
    auto const conf_sv_span = hal::as_bytes(opt_conf_sv.value());
    auto desc_len = static_cast<hal::byte>((conf_sv_span.size() + 2));
    hal::print<32>(*m_console,
                   "%d: (size: 0x%X, psize: 0x%X): [ ",
                   target_idx,
                   conf_sv_span.size(),
                   desc_len);

    for (auto const el : conf_sv_span) {
      hal::print<8>(*m_console, "0x%X, ", el);
    }
    hal::print(*m_console, "]\n");
    hal::print(*m_console, "\n");
    auto hdr_arr = std::to_array(
      { desc_len, static_cast<hal::byte>(descriptor_type::string) });

    auto scatter_arr_pair =
      make_sub_scatter_bytes(p_len, hdr_arr, conf_sv_span);

    auto p = scatter_span<byte const>(scatter_arr_pair.first)
               .first(scatter_arr_pair.second);
    hal::v5::write_and_flush(*m_ctrl_ep, p);
    hal::print(*m_console, "Wrote cfg string\n");
    print_payload(m_console, p);
    return true;
  }

  strong_ptr<control_endpoint> m_ctrl_ep;
  strong_ptr<device> m_device;
  strong_ptr<std::array<configuration, num_configs>> m_configs;
  u16 m_lang_str;
  u8 m_starting_str_idx;
  std::optional<std::pair<u8, strong_ptr<interface>>> m_iface_for_str_desc;
  configuration* m_active_conf = nullptr;
  bool m_has_setup_packet = false;
  hal::v5::strong_ptr<hal::serial> m_console;
};
}  // namespace hal::v5::usb
