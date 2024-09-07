#include <libhal-lpc40/clock.hpp>

#include <cstdint>

#include <libhal-util/bit.hpp>
#include <libhal/error.hpp>
#include <libhal/units.hpp>

#include "system_controller_reg.hpp"

namespace hal::lpc40 {

namespace {
hertz cpu_clock_rate = irc_frequency;
hertz emc_clock_rate = irc_frequency;
hertz usb_clock_rate = irc_frequency;
hertz spifi_clock_source_rate = irc_frequency;
hertz peripheral_clock_rate = irc_frequency / default_peripheral_divider;

struct pll_registers
{
  uint32_t volatile* p_control;
  uint32_t volatile* p_config;
  uint32_t volatile* p_feed;
  uint32_t const volatile* p_stat;
};

hertz setup_pll(clock_tree const& p_clock_config,
                pll_registers const& p_pll_registers,
                std::uint8_t p_pll_index)
{
  using namespace hal::literals;

  auto const& pll_config = p_clock_config.pll[p_pll_index];
  hertz fcco = 0.0_Hz;

  if (pll_config.enabled) {
    hal::bit_modify(*p_pll_registers.p_config)
      .insert<pll_register::multiplier>(
        static_cast<size_t>(pll_config.multiply - 1U));

    if (p_clock_config.use_external_oscillator == false && p_pll_index == 0) {
      fcco = irc_frequency * static_cast<float>(pll_config.multiply);
    } else {
      fcco = p_clock_config.oscillator_frequency *
             static_cast<float>(pll_config.multiply);
    }

    // In the data sheet this is the divider, but it acts to multiply the
    // frequency higher to a point where the fcco is stable.
    //
    // fcco must be between 156 MHz to 320 MHz.
    uint32_t fcco_divide = 0;
    for (auto divide_codes : { 0U, 1U, 2U, 3U }) {
      // Multiply the fcco by 2^divide_code
      hertz final_fcco = fcco * static_cast<float>(1U << divide_codes);
      if (156.0_MHz <= final_fcco && final_fcco <= 320.0_MHz) {
        fcco_divide = divide_codes;
        break;
      }
    }

    hal::bit_modify(*p_pll_registers.p_config)
      .insert<pll_register::divider>(fcco_divide);
    // Enable PLL
    *p_pll_registers.p_control = 1;
    // Feed PLL in order to start the locking process
    *p_pll_registers.p_feed = 0xAA;
    *p_pll_registers.p_feed = 0x55;

    while (!hal::bit_extract<pll_register::pll_lock>(*p_pll_registers.p_stat)) {
      continue;
    }
  }

  return fcco;
}

void enable_external_oscillator(hertz p_oscillator_frequency)
{
  using namespace hal::literals;

  auto frequency = p_oscillator_frequency;
  // Range select is 0 when  1.0_MHz < frequency < 20.0_MHz
  std::uint32_t range_select_value = 0;

  if (20.0_MHz < frequency && frequency <= 25.0_MHz) {
    range_select_value = 1;
  }

  hal::bit_modify(system_controller_reg->scs)
    .insert<oscillator::range_select>(range_select_value)
    .set<oscillator::external_enable>();

  while (
    !hal::bit_extract<oscillator::external_ready>(system_controller_reg->scs)) {
    continue;
  }
}
}  // namespace

void maximum(hertz p_external_crystal_frequency)
{
  static constexpr auto max_speed = 120.0_MHz;
  auto const multiply = max_speed / p_external_crystal_frequency;

  clock_tree config{};
  config.oscillator_frequency = p_external_crystal_frequency;
  config.use_external_oscillator = true;
  config.cpu.use_pll0 = true;
  config.cpu.divider = 1;
  config.emc_half_cpu_divider = false;
  config.peripheral_divider = 1;
  config.usb.clock = usb_clock_source::pll0;
  config.usb.divider = usb_divider::divide_by1;
  config.spifi.clock = spifi_clock_source::pll0;
  config.spifi.divider = 1;
  config.pll[0].enabled = true;
  config.pll[0].multiply = static_cast<uint8_t>(multiply);
  config.pll[1].enabled = false;

  configure_clocks(config);
}

/**
 * @brief Determins if the external oscillator is currently enabled and in use
 *
 * @return true - external oscillator is in use currently
 * @return false - external oscillator is NOT in use currently
 */
bool using_external_oscillator()
{
  return hal::bit_extract<oscillator::external_enable>(
    system_controller_reg->scs);
}

hertz get_frequency(peripheral p_peripheral)
{
  switch (p_peripheral) {
    case peripheral::emc:
      return emc_clock_rate;
    case peripheral::usb:
      return usb_clock_rate;
    case peripheral::spifi:
      return spifi_clock_source_rate;
    case peripheral::cpu:
      return cpu_clock_rate;
    default:
      return peripheral_clock_rate;
  }
}

void configure_clocks(clock_tree const& p_clock_tree)
{
  using namespace hal::literals;

  hertz system_clock = 0.0_Hz;
  hertz pll0 = 0.0_Hz;
  hertz pll1 = 0.0_Hz;
  hertz cpu = 0.0_Hz;
  hertz usb = 0.0_Hz;
  hertz spifi = 0.0_Hz;

  // =========================================================================
  // Step 1. Select IRC as clock source for everything.
  //         Make sure PLLs are not clock sources for everything.
  // =========================================================================
  // Set CPU clock to system clock
  hal::bit_modify(system_controller_reg->cpu_clock_select)
    .insert<cpu_clock::select>(0UL);

  // Set USB clock to system clock
  hal::bit_modify(system_controller_reg->usb_clock_select)
    .insert<usb_clock::select>(value(usb_clock_source::system_clock));

  // Set spifi clock to system clock
  hal::bit_modify(system_controller_reg->spifi_clock_select)
    .insert<spifi_clock::select>(value(spifi_clock_source::system_clock));

  // Set the clock source to IRC (0) and not external oscillator. The next
  // phase disables that clock source, which will stop the system if this is
  // not switched.
  system_controller_reg->clock_source_select = 0;

  // =========================================================================
  // Step 2. Disable PLLs
  // =========================================================================
  // NOTE: The only bit in this register that is used is bit 0 which indicates
  // enabled or disabled status, thus a single assignment is needed.
  system_controller_reg->pll0con = 0;
  system_controller_reg->pll1con = 0;

  // Disabling external oscillator if it is not going to be used
  hal::bit_modify(system_controller_reg->scs)
    .clear(oscillator::external_enable);

  // =========================================================================
  // Step 3. Select oscillator source for System Clock and Main PLL
  // =========================================================================
  // Enable the external oscillator if we are using it, which would be the
  // case if the alternative PLL is enabled or external oscillator is
  // selected.
  if (p_clock_tree.use_external_oscillator == true ||
      p_clock_tree.pll[1].enabled) {
    enable_external_oscillator(p_clock_tree.oscillator_frequency);
  }

  system_controller_reg->clock_source_select =
    p_clock_tree.use_external_oscillator;

  if (p_clock_tree.use_external_oscillator) {
    system_clock = p_clock_tree.oscillator_frequency;
  } else {
    system_clock = irc_frequency;
  }

  // =========================================================================
  // Step 4. Configure PLLs
  // =========================================================================
  pll0 = setup_pll(p_clock_tree,
                   {
                     .p_control = &system_controller_reg->pll0con,
                     .p_config = &system_controller_reg->pll0cfg,
                     .p_feed = &system_controller_reg->pll0feed,
                     .p_stat = &system_controller_reg->pll0stat,
                   },
                   0);

  pll1 = setup_pll(p_clock_tree,
                   {
                     .p_control = &system_controller_reg->pll1con,
                     .p_config = &system_controller_reg->pll1cfg,
                     .p_feed = &system_controller_reg->pll1feed,
                     .p_stat = &system_controller_reg->pll1stat,
                   },
                   1);

  // =========================================================================
  // Step 5. Set clock dividers for each clock source
  // =========================================================================
  // Set CPU clock divider
  hal::bit_modify(system_controller_reg->cpu_clock_select)
    .insert<cpu_clock::divider>(p_clock_tree.cpu.divider);

  // Set EMC clock divider
  hal::bit_modify(system_controller_reg->emmc_clock_select)
    .insert<emc_clock::divider>(p_clock_tree.emc_half_cpu_divider);

  // Set Peripheral clock divider
  hal::bit_modify(system_controller_reg->peripheral_clock_select)
    .insert<peripheral_clock::divider>(p_clock_tree.peripheral_divider);

  // Set USB clock divider
  hal::bit_modify(system_controller_reg->usb_clock_select)
    .insert<usb_clock::divider>(value(p_clock_tree.usb.divider));

  // Set spifi clock divider
  hal::bit_modify(system_controller_reg->spifi_clock_select)
    .insert<spifi_clock::divider>(p_clock_tree.spifi.divider);

  if (p_clock_tree.cpu.use_pll0) {
    cpu = pll0;
  } else {
    cpu = system_clock;
  }

  switch (p_clock_tree.usb.clock) {
    case usb_clock_source::system_clock:
      usb = system_clock;
      break;
    case usb_clock_source::pll0:
      usb = pll0;
      break;
    case usb_clock_source::pll1:
      usb = pll1;
      break;
  }

  switch (p_clock_tree.spifi.clock) {
    case spifi_clock_source::system_clock:
      spifi = system_clock;
      break;
    case spifi_clock_source::pll0:
      spifi = pll0;
      break;
    case spifi_clock_source::pll1:
      spifi = pll1;
      break;
  }

  cpu_clock_rate = cpu / static_cast<float>(p_clock_tree.cpu.divider);
  peripheral_clock_rate =
    cpu / static_cast<float>(p_clock_tree.peripheral_divider);
  emc_clock_rate =
    cpu / static_cast<float>(p_clock_tree.emc_half_cpu_divider + 1);
  usb_clock_rate = usb / static_cast<float>(p_clock_tree.usb.divider);
  spifi_clock_source_rate =
    spifi / static_cast<float>(p_clock_tree.spifi.divider);

  // =========================================================================
  // Step 6. Configure flash cycles per load
  // =========================================================================
  system_controller_reg->power_boost = 0b00;

  if (cpu_clock_rate < 20.0_MHz) {
    system_controller_reg->flashcfg =
      static_cast<uint32_t>(flash_configuration::clock1);
  } else if (20.0_MHz <= cpu_clock_rate && cpu_clock_rate < 40.0_MHz) {
    system_controller_reg->flashcfg =
      static_cast<uint32_t>(flash_configuration::clock2);
  } else if (40.0_MHz <= cpu_clock_rate && cpu_clock_rate < 60.0_MHz) {
    system_controller_reg->flashcfg =
      static_cast<uint32_t>(flash_configuration::clock3);
  } else if (60.0_MHz <= cpu_clock_rate && cpu_clock_rate < 80.0_MHz) {
    system_controller_reg->flashcfg =
      static_cast<uint32_t>(flash_configuration::clock4);
  } else if (80.0_MHz <= cpu_clock_rate && cpu_clock_rate < 100.0_MHz) {
    system_controller_reg->flashcfg =
      static_cast<uint32_t>(flash_configuration::clock5);
  } else if (cpu_clock_rate >= 100.0_MHz) {
    system_controller_reg->flashcfg =
      static_cast<uint32_t>(flash_configuration::clock5);
    system_controller_reg->power_boost = 0b11;
  }

  // =========================================================================
  // Step 7. Finally select the sources for each clock
  // =========================================================================
  // Set CPU clock the source defined in the configuration
  hal::bit_modify(system_controller_reg->cpu_clock_select)
    .insert<cpu_clock::select>(
      static_cast<uint32_t>(p_clock_tree.cpu.use_pll0));

  // Set USB clock the source defined in the configuration
  hal::bit_modify(system_controller_reg->usb_clock_select)
    .insert<usb_clock::select>(static_cast<uint32_t>(p_clock_tree.usb.clock));

  // Set spifi clock the source defined in the configuration
  hal::bit_modify(system_controller_reg->spifi_clock_select)
    .insert<spifi_clock::select>(
      static_cast<uint32_t>(p_clock_tree.spifi.clock));
}
}  // namespace hal::lpc40
