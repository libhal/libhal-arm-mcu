#include <libhal-arm-mcu/lpc40/power.hpp>

#include <libhal-util/bit.hpp>
#include <libhal-util/enum.hpp>

#include "system_controller_reg.hpp"

namespace hal::lpc40 {
void power_on(peripheral p_peripheral)
{
  hal::bit_modify(system_controller_reg->peripheral_power_control0)
    .set(bit_mask::from(value(p_peripheral)));
}

bool is_on(peripheral p_peripheral)
{
  return hal::bit_extract(bit_mask::from(value(p_peripheral)),
                          system_controller_reg->peripheral_power_control0);
}

void power_off(peripheral p_peripheral)
{
  hal::bit_modify(system_controller_reg->peripheral_power_control0)
    .clear(bit_mask::from(value(p_peripheral)));
}
}  // namespace hal::lpc40
