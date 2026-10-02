/*
 * This program is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include "sensors/pmw3610.h"

#if defined(POINTING_DEVICE_ENABLED)

#include "hardware/hardware.h"
#include "hardware/spi_api.h"

//--------------------------------------------------------------------+
// Configuration
//--------------------------------------------------------------------+

#if !defined(PMW3610_SPI_FREQUENCY)
// Target SPI frequency for the PMW3610 (max 2 MHz)
#define PMW3610_SPI_FREQUENCY 2000000
#endif

#if !defined(PMW3610_CLOCK_ON_DELAY_US)
// Delay required after enabling the sensor SPI clock
#define PMW3610_CLOCK_ON_DELAY_US 300
#endif

// SPI timing constants for the PMW3610 (values in microseconds).
// Sub-microsecond delays from the datasheet are rounded up to 1 us because
// this firmware uses a busy-wait loop with microsecond granularity.
#define T_NCS_SCLK_US 1    // 120 ns, NCS low to first SCLK
#define T_SCLK_NCS_WR_US 10 // 10 us, last SCLK to NCS high for writes
#define T_SRAD_US 4        // 4 us, address write to data read
#define T_SRAD_MOTBR_US 4  // 4 us, burst address to data read
#define T_SRX_US 1         // 250 ns, last SCLK to NCS high for reads
#define T_SWX_US 30        // 30 us, NCS high to next NCS low for writes
#define T_BEXIT_US 1       // 250 ns, last SCLK to NCS high for burst reads

#if !defined(PMW3610_PERFORMANCE_VALUE)
// PERFORMANCE register: 0x0D selects the 4 ms run-mode report rate and 0xF0
// forces the sensor to stay awake (no downshift into the REST1-3 modes). The
// keyboard is USB-powered, so trade idle power for no wake-up latency when the
// ball starts moving.
#define PMW3610_PERFORMANCE_VALUE 0xFD
#endif

//--------------------------------------------------------------------+
// Pin Macros
//--------------------------------------------------------------------+

#define CS_LOW() gpio_write(PMW3610_CS_PORT, PMW3610_CS_PIN, false)
#define CS_HIGH() gpio_write(PMW3610_CS_PORT, PMW3610_CS_PIN, true)

//--------------------------------------------------------------------+
// State
//--------------------------------------------------------------------+

static bool init_ok = false;

//--------------------------------------------------------------------+
// Helpers
//--------------------------------------------------------------------+

static void pmw3610_delay_us(uint32_t us) {
  const uint32_t system_clock = board_clock_frequency();
  const uint32_t cycles = (system_clock / 1000000) * us;
  const uint32_t start = board_cycle_count();
  while ((board_cycle_count() - start) < cycles)
    ;
}

static void pmw3610_write_byte(uint8_t byte) {
  spi_soft_transfer_byte(byte);
}

static uint8_t pmw3610_read_byte(void) {
  return spi_soft_transfer_byte(0x00);
}

static void pmw3610_write_reg_internal(uint8_t reg, uint8_t value) {
  CS_LOW();
  pmw3610_delay_us(T_NCS_SCLK_US);
  spi_soft_set_sdio_input(false);
  pmw3610_write_byte(reg | PMW3610_SPI_WRITE_BIT);
  pmw3610_write_byte(value);
  pmw3610_delay_us(T_SCLK_NCS_WR_US);
  CS_HIGH();
  pmw3610_delay_us(T_SWX_US);
}

static uint8_t pmw3610_read_reg_internal(uint8_t reg) {
  CS_LOW();
  pmw3610_delay_us(T_NCS_SCLK_US);
  // In 3-wire mode the bidirectional SDIO pin must be driven while sending
  // the address and switched to input before receiving data.
  spi_soft_set_sdio_input(false);
  pmw3610_write_byte(reg);
  spi_soft_set_sdio_input(true);
  pmw3610_delay_us(T_SRAD_US);
  const uint8_t value = pmw3610_read_byte();
  pmw3610_delay_us(T_SRX_US);
  CS_HIGH();
  return value;
}

static void pmw3610_spi_clock_on(void) {
  pmw3610_write_reg_internal(PMW3610_REG_SPI_CLK_ON_REQ,
                             PMW3610_SPI_CLOCK_CMD_ENABLE);
  pmw3610_delay_us(PMW3610_CLOCK_ON_DELAY_US);
}

static void pmw3610_spi_clock_off(void) {
  pmw3610_write_reg_internal(PMW3610_REG_SPI_CLK_ON_REQ,
                             PMW3610_SPI_CLOCK_CMD_DISABLE);
}

static void pmw3610_write_reg(uint8_t reg, uint8_t value) {
  pmw3610_spi_clock_on();
  pmw3610_write_reg_internal(reg, value);
  pmw3610_spi_clock_off();
}

// Only register writes need the SPI clock request; reads (including the
// motion burst) work with the clock gated, so skipping the request avoids the
// clock-on settle delay on every read.
static uint8_t pmw3610_read_reg(uint8_t reg) {
  return pmw3610_read_reg_internal(reg);
}

void pmw3610_set_cpi(uint16_t cpi) {
  if (cpi < PMW3610_MIN_CPI)
    cpi = PMW3610_MIN_CPI;
  else if (cpi > PMW3610_MAX_CPI)
    cpi = PMW3610_MAX_CPI;

  // RES_STEP carries only the CPI resolution. Axis orientation
  // (SWAP_XY/INVERT_X/INVERT_Y) is applied in software when reading motion;
  // setting those bits here makes some sensor units stop tracking.
  uint8_t value = (uint8_t)(cpi / 200);

  pmw3610_spi_clock_on();
  pmw3610_write_reg_internal(PMW3610_REG_SPI_PAGE0, 0xFF);
  pmw3610_write_reg_internal(PMW3610_REG_RES_STEP, value);
  pmw3610_write_reg_internal(PMW3610_REG_SPI_PAGE0, 0x00);
  pmw3610_spi_clock_off();
}

void pmw3610_set_enabled(bool enabled) {
  if (!init_ok)
    return;

  if (enabled) {
    pmw3610_write_reg(PMW3610_REG_POWER_UP_RESET, PMW3610_POWERUP_CMD_WAKEUP);
    timer_delay(10);
    // Restore the awake performance setting after wakeup.
    pmw3610_write_reg(PMW3610_REG_PERFORMANCE, PMW3610_PERFORMANCE_VALUE);
  } else {
    pmw3610_write_reg(PMW3610_REG_SHUTDOWN, PMW3610_SHUTDOWN_ENABLE);
  }
}

static void pmw3610_read_burst(uint8_t reg, uint8_t *buf, uint8_t len) {
  CS_LOW();
  pmw3610_delay_us(T_NCS_SCLK_US);
  // In 3-wire mode the bidirectional SDIO pin must be driven while sending
  // the burst address and switched to input before receiving data.
  spi_soft_set_sdio_input(false);
  pmw3610_write_byte(reg);
  spi_soft_set_sdio_input(true);
  pmw3610_delay_us(T_SRAD_MOTBR_US);
  for (uint8_t i = 0; i < len; i++)
    buf[i] = pmw3610_read_byte();
  pmw3610_delay_us(T_BEXIT_US);
  CS_HIGH();
}

//--------------------------------------------------------------------+
// Initialization
//--------------------------------------------------------------------+

bool pmw3610_init(void) {
  // Initialize chip-select high
  gpio_set_output(PMW3610_CS_PORT, PMW3610_CS_PIN);
  CS_HIGH();

  // Initialize soft SPI in mode 3 (CPOL=1, CPHA=1), MSB first
  spi_soft_init(PMW3610_SCK_PORT, PMW3610_SCK_PIN, PMW3610_MOSI_PORT,
                PMW3610_MOSI_PIN, PMW3610_MISO_PORT, PMW3610_MISO_PIN);
  spi_soft_set_mode(1, 1);
  spi_soft_set_frequency(PMW3610_SPI_FREQUENCY);

#if defined(PMW3610_IRQ_PIN)
  // The PMW3610 MOTION pin is active-low; enable the internal pull-up so the
  // line sits high when no motion is detected.
  gpio_set_input_pullup(PMW3610_IRQ_PORT, PMW3610_IRQ_PIN_NUM);
#endif

  // Reset the SPI port and wait for at least one frame, as required by the
  // power-up sequence in the PMW3610 datasheet.
  CS_LOW();
  pmw3610_delay_us(1);
  CS_HIGH();
  pmw3610_delay_us(150);

  // Power-up reset. The sensor requires the SPI clock to be enabled before
  // any register access, including the power-up reset command.
  pmw3610_spi_clock_on();
  pmw3610_write_reg_internal(PMW3610_REG_POWER_UP_RESET,
                             PMW3610_POWERUP_CMD_RESET);
  pmw3610_spi_clock_off();
  timer_delay(10);

  // Clear observation register
  pmw3610_write_reg(PMW3610_REG_OBSERVATION, 0x00);
  timer_delay(200);

  // Verify observation register and product ID
  const uint8_t observation = pmw3610_read_reg(PMW3610_REG_OBSERVATION);
  if ((observation & 0x0F) != 0x0F) {
    init_ok = false;
    return false;
  }

  const uint8_t product_id = pmw3610_read_reg(PMW3610_REG_PRODUCT_ID);
  if (product_id != PMW3610_PRODUCT_ID) {
    init_ok = false;
    return false;
  }

  timer_delay(50);

  // Clear motion registers
  for (uint8_t reg = PMW3610_REG_MOTION; reg <= PMW3610_REG_DELTA_XY_H; reg++)
    (void)pmw3610_read_reg(reg);

  // Configure performance: 4 ms run-mode reporting, forced awake by default
  pmw3610_write_reg(PMW3610_REG_PERFORMANCE, PMW3610_PERFORMANCE_VALUE);

  // Configure CPI (axis orientation is applied in software on read)
  pmw3610_set_cpi(PMW3610_CPI);

  // Configure downshift and sample rates
  pmw3610_write_reg(PMW3610_REG_RUN_DOWNSHIFT, 0x20);
  pmw3610_write_reg(PMW3610_REG_REST1_RATE, 0x04);
  pmw3610_write_reg(PMW3610_REG_REST1_DOWNSHIFT, 0x01);
  pmw3610_write_reg(PMW3610_REG_REST2_RATE, 0x0A);
  pmw3610_write_reg(PMW3610_REG_REST2_DOWNSHIFT, 0x01);
  pmw3610_write_reg(PMW3610_REG_REST3_RATE, 0x64);

  init_ok = true;
  return true;
}

//--------------------------------------------------------------------+
// Motion Reading
//--------------------------------------------------------------------+

bool pmw3610_read_motion(int16_t *dx, int16_t *dy) {
#if defined(PMW3610_IRQ_PIN)
  // MOTION is active-low. If the pin is high there is no new motion data.
  if (gpio_read(PMW3610_IRQ_PORT, PMW3610_IRQ_PIN_NUM))
    return false;
#endif

  uint8_t buf[PMW3610_BURST_SIZE];
  pmw3610_read_burst(PMW3610_REG_MOTION_BURST, buf, PMW3610_BURST_SIZE);

  if ((buf[0] & 0x80) == 0)
    return false;

  uint16_t x_raw = (uint16_t)(buf[PMW3610_BURST_X_L_POS] |
                              ((buf[PMW3610_BURST_XY_H_POS] & 0xF0) << 4));
  uint16_t y_raw = (uint16_t)(buf[PMW3610_BURST_Y_L_POS] |
                              ((buf[PMW3610_BURST_XY_H_POS] & 0x0F) << 8));

  int16_t x = (int16_t)x_raw;
  int16_t y = (int16_t)y_raw;

  if (x & 0x0800)
    x |= 0xF000;
  if (y & 0x0800)
    y |= 0xF000;

  // Axis orientation (swap/invert/rotation) is applied at runtime in
  // pointing_device.c from the persisted pointing configuration; the sensor
  // driver reports raw counts. make.py seeds the persisted defaults from the
  // keyboard.json angle/swap_xy/invert_x/invert_y values.

  *dx = x;
  *dy = y;

  // The burst read itself clears the motion and delta registers, so no extra
  // MOTION write is needed before the next read.
  return true;
}

void pmw3610_get_info(pmw3610_info_t *info) {
  info->product_id = pmw3610_read_reg(PMW3610_REG_PRODUCT_ID);
  info->observation = pmw3610_read_reg(PMW3610_REG_OBSERVATION);
  info->motion = pmw3610_read_reg(PMW3610_REG_MOTION);
#if defined(PMW3610_IRQ_PIN)
  info->irq_low = !gpio_read(PMW3610_IRQ_PORT, PMW3610_IRQ_PIN_NUM);
#else
  info->irq_low = false;
#endif
  info->init_ok = init_ok;
}

#endif // defined(POINTING_DEVICE_ENABLED)
