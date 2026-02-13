#include "libhal-arm-mcu/rp/adc.hpp"
#include "libhal-arm-mcu/rp/time.hpp"

#include <hardware/adc.h>
#include <hardware/address_mapped.h>
#include <hardware/dma.h>
#include <hardware/gpio.h>
#include <hardware/platform_defs.h>

#include <libhal-util/bit.hpp>
#include <libhal/error.hpp>
#include <libhal/units.hpp>

namespace hal::rp {

inline namespace v4 {
adc::adc(u8 pin)
  : m_pin(pin)
{
  adc_init();
  if (pin < ADC_BASE_PIN || pin >= ADC_BASE_PIN + NUM_ADC_CHANNELS - 1) {
    hal::safe_throw(hal::argument_out_of_domain(this));
  }
  adc_gpio_init(pin);
}

adc::~adc()
{
  gpio_deinit(m_pin);
}

float adc::driver_read()
{
  adc_select_input(m_pin - ADC_BASE_PIN);
  u16 result = adc_read();
  // weirdly enough the sdk doesn't provide a function to read the error bits
  if (adc_hw->cs & ADC_CS_ERR_BITS) {
    hal::safe_throw(hal::io_error(this));
  }

  return float(result) / float((1 << 12) - 1);
}
};  // namespace v4

namespace v5 {

adc16::adc16(u8 pin)
  : m_pin(pin)
{
  adc_init();
  if (pin < ADC_BASE_PIN || pin >= ADC_BASE_PIN + NUM_ADC_CHANNELS - 1) {
    hal::safe_throw(hal::argument_out_of_domain(this));
  }
  adc_gpio_init(pin);
}

adc16::~adc16()
{
  gpio_deinit(m_pin);
}

u16 adc16::driver_read()
{
  adc_select_input(m_pin - ADC_BASE_PIN);
  u16 result = adc_read();
  // weirdly enough the sdk doesn't provide a function to read the error bits
  if (adc_hw->cs & ADC_CS_ERR_BITS) {
    hal::safe_throw(hal::io_error(this));
  }
  // TODO: Use hal::upscale to actually make this 16 bit
  // for some reason the docs reference a function that
  // doesn't exist yet
  return result;
}

}  // namespace v5

namespace nonstandard {
adc16_pack::adc16_pack(u8 mask)
{
  u32 base_pin = 0;
  if constexpr (internal::pin_max == 30) {
    base_pin = 26;
  } else if constexpr (internal::pin_max == 48) {
    base_pin = 40;
  }
  u8 pinnum = 0;
  bool first_selected = false;
  for (u32 i = 0; i < 8; ++i) {
    if (mask & (1 << i)) {
      gpio_init(i + base_pin);
      pinnum += 1;
      if (!first_selected) {
        adc_select_input(i);
        m_first_pin = i;
        first_selected = true;
      }
    }
  }
  adc_set_round_robin(mask);
  m_read_size = pinnum;
}

void adc16_pack::read_many_now(std::span<u16> s)
{
  if (s.size() < m_read_size)
    safe_throw(hal::io_error(this));
  for (u8 i = 0; i < m_read_size; ++i) {
    s[i] = adc_read();
  }
}

adc16_pack::read_session adc16_pack::async()
{
  int read_dma = dma_claim_unused_channel(false);
  if (read_dma == -1) {
    hal::safe_throw(hal::device_or_resource_busy(this));
  }

  adc_hw->fcs |= ADC_FCS_DREQ_EN_BITS | ADC_FCS_EN_BITS;
  dma_channel_config_t read_cfg = dma_channel_get_default_config(read_dma);
  channel_config_set_read_increment(&read_cfg, false);
  channel_config_set_write_increment(&read_cfg, true);
  channel_config_set_transfer_data_size(&read_cfg, DMA_SIZE_16);
  dma_channel_set_config(read_dma, &read_cfg, false);
  dma_channel_set_read_addr(read_dma, &adc_hw->fifo, false);
  hw_write_masked(&dma_channel_hw_addr(read_dma)->ctrl_trig,
                  DREQ_ADC,
                  DMA_CH0_CTRL_TRIG_TREQ_SEL_BITS);
  hw_write_masked(&adc_hw->fcs, 1, ADC_FCS_THRESH_BITS);
  return { static_cast<u8>(read_dma),
           m_read_size,
           static_cast<u8>(1 << m_first_pin) };
}
adc16_pack::read_session::~read_session()
{
  adc_hw->fcs &= ADC_FCS_DREQ_EN_BITS;
  dma_channel_config_t read_cfg = dma_get_channel_config(m_dma);
  channel_config_set_enable(&read_cfg, false);
  dma_channel_set_config(m_dma, &read_cfg, false);
  dma_channel_unclaim(m_dma);
}

adc16_pack::read_session::promise adc16_pack::read_session::read(
  std::span<u16> span)
{
  dma_channel_set_write_addr(m_dma, span.data(), false);
  dma_channel_set_transfer_count(
    m_dma, dma_encode_transfer_count(span.size()), false);
  hw_set_bits(&adc_hw->cs, ADC_CS_START_ONCE_BITS);
  return promise{ m_dma };
}

microseconds adc16_pack::read_session::promise::poll()
{
  u32 transfers = (dma_channel_hw_addr(m_dma)->transfer_count &
                   DMA_CH0_TRANS_COUNT_COUNT_BITS) >>
                  DMA_CH0_TRANS_COUNT_COUNT_LSB;
  return transfers * microseconds(2);
}

}  // namespace nonstandard

}  // namespace hal::rp
