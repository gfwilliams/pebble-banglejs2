#include "board/board.h"

#include "drivers/gpio.h"
#include "drivers/i2c.h"
#include "drivers/nrf5/uart_definitions.h"
#include "drivers/nrf5/i2c_hal_definitions.h"
#include "drivers/nrf5/spi_definitions.h"
#include "drivers/flash/qspi_flash_definitions.h"
#include "drivers/i2c_definitions.h"
#include "drivers/qspi_definitions.h"
#include "drivers/temperature.h"
#include "drivers/voltage_monitor.h"
#include "drivers/exti.h"
#include "flash_region/flash_region.h"
#include "util/units.h"
#include "system/passert.h"
#include "kernel/util/sleep.h"
#include "drivers/pwm.h"
#include "services/common/system_task.h"
#include "kernel/events.h"

#include <nrfx_i2s.h>

// QSPI
#include <nrfx_qspi.h>
#include <nrfx_gpiote.h>
#include <nrfx_spim.h>
#include <nrfx_twim.h>
#include <hal/nrf_gpio.h>
#include <hal/nrf_clock.h>


/* SPI FLash */
static QSPIPortState s_qspi_port_state;
static QSPIPort QSPI_PORT = {
  .state = &s_qspi_port_state,
  .auto_polling_interval = 16,
  .cs_gpio = NRF_GPIO_PIN_MAP(0, 14),
  .clk_gpio = NRF_GPIO_PIN_MAP(0, 16),
  .data_gpio = {
    NRF_GPIO_PIN_MAP(0, 15),
    NRF_GPIO_PIN_MAP(0, 13),
    NRF_QSPI_PIN_NOT_CONNECTED,
    NRF_QSPI_PIN_NOT_CONNECTED,
  },
};
QSPIPort * const QSPI = &QSPI_PORT;

static QSPIFlashState s_qspi_flash_state;
static QSPIFlash QSPI_FLASH_DEVICE = {
  .state = &s_qspi_flash_state,
  .qspi = &QSPI_PORT,
  .default_fast_read_ddr_enabled = false,
  .reset_gpio = { GPIO_Port_NULL },
};
QSPIFlash * const QSPI_FLASH = &QSPI_FLASH_DEVICE;
IRQ_MAP_NRFX(QSPI, nrfx_qspi_irq_handler);
/* PERIPHERAL ID 43 */

static UARTDeviceState s_dbg_uart_state;
static UARTDevice DBG_UART_DEVICE = {
  .state = &s_dbg_uart_state,
  .tx_gpio = NRF_GPIO_PIN_MAP(1, 11),
  .rx_gpio = NRF_GPIO_PIN_MAP(1, 10),
  .rts_gpio = NRF_UARTE_PSEL_DISCONNECTED,
  .cts_gpio = NRF_UARTE_PSEL_DISCONNECTED,
  .periph = NRFX_UARTE_INSTANCE(0),
  .counter = NRFX_TIMER_INSTANCE(2),
};
UARTDevice * const DBG_UART = &DBG_UART_DEVICE;
IRQ_MAP_NRFX(UART0_UARTE0, nrfx_uarte_0_irq_handler);
/* PERIPHERAL ID 8 */

/* buttons */
IRQ_MAP_NRFX(TIMER1, nrfx_timer_1_irq_handler);
IRQ_MAP_NRFX(TIMER2, nrfx_timer_2_irq_handler);

/* display */
IRQ_MAP_NRFX(SPIM3, nrfx_spim_3_irq_handler);

/* PERIPHERAL ID 10 */

/* EXTI */
IRQ_MAP_NRFX(GPIOTE, nrfx_gpiote_0_irq_handler);

/* Touch */
/*static I2CBusState I2C_TOUCH_IIC1_BUS_STATE = {};

static const I2CBusHal I2C_TOUCH_IIC1_BUS_HAL = {
  .twim = NRFX_TWIM_INSTANCE(1),
  .frequency = NRF_TWIM_FREQ_100K,
  .should_be_init = false
};

static const I2CBus I2C_TOUCH_IIC1_BUS = { // what
  .state = &I2C_TOUCH_IIC1_BUS_STATE,
  .hal = &I2C_TOUCH_IIC1_BUS_HAL,
  .scl_gpio = {
    .gpio = NRF5_GPIO_RESOURCE_EXISTS,
    .gpio_pin = NRF_GPIO_PIN_MAP(1, 2),
  },
  .sda_gpio = {
    .gpio = NRF5_GPIO_RESOURCE_EXISTS,
    .gpio_pin = NRF_GPIO_PIN_MAP(1, 1),
  },
  .name = "I2C_TOUCH_IIC1"
};
IRQ_MAP_NRFX(SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1, nrfx_twim_1_irq_handler); // we don't need this on Bangle - but we do need I2C for HRM/etc

static const I2CSlavePort I2C_SLAVE_TOUCH = {
  .bus = &I2C_TOUCH_IIC1_BUS,
  .address = 0x2A
};
I2CSlavePort * const I2C_TOUCH = &I2C_SLAVE_TOUCH;*/

/* peripheral I2C bus */
static I2CBusState I2C_IIC2_BUS_STATE = {};

static const I2CBusHal I2C_IIC2_BUS_HAL = {
  .twim = NRFX_TWIM_INSTANCE(0),
  .frequency = NRF_TWIM_FREQ_100K,
  .should_be_init = true
};

static const I2CBus I2C_IIC2_BUS = {
  .state = &I2C_IIC2_BUS_STATE,
  .hal = &I2C_IIC2_BUS_HAL,
  .scl_gpio = {
    .gpio = NRF5_GPIO_RESOURCE_EXISTS,
    .gpio_pin = NRF_GPIO_PIN_MAP(1, 6),
  },
  .sda_gpio = {
    .gpio = NRF5_GPIO_RESOURCE_EXISTS,
    .gpio_pin = NRF_GPIO_PIN_MAP(1, 5), // accel
  },
  .name = "I2C_IIC2"
};
IRQ_MAP_NRFX(SPI0_SPIM0_SPIS0_TWI0_TWIM0_TWIS0, nrfx_twim_0_irq_handler);

/* PERIPHERAL ID 11 */

/* sensor SPI bus */

// Bangle puts sensors on I2C

PwmState BACKLIGHT_PWM_STATE;
IRQ_MAP_NRFX(PWM0, nrfx_pwm_0_irq_handler);

void board_early_init(void) {
  PBL_LOG(LOG_LEVEL_ALWAYS, "bangle early init");

  nrf_clock_lf_src_set(NRF_CLOCK, NRF_CLOCK_LFCLK_XTAL);
  nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED);
  nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_LFCLKSTART);
  /* TODO: Add timeout, report failure if LFCLK does not start. For now,
   * WDT should trigger a reboot. Calibrated RC may be used as a fallback,
   * provided we can adjust BLE SCA settings at runtime.
   */
  while (!nrf_clock_event_check(NRF_CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED)) {
  }
  nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED);
}

// ============================================================= super hacky software i2c for touch
#define TOUCH_PIN_RST 35
#define TOUCH_PIN_IRQ 36
#define TOUCH_PIN_SDA 33
#define TOUCH_PIN_SCL 34
#define TOUCH_I2C_TIMEOUT 100000
/// write pin
void wr(int pin, bool state) {
  if (state) {
    nrf_gpio_pin_set(pin); nrf_gpio_cfg_output(pin);
    nrf_gpio_cfg_input(pin, NRF_GPIO_PIN_PULLUP);
  } else {
    nrf_gpio_pin_clear(pin);
    nrf_gpio_cfg_output(pin);
  }
}
/// read pin
bool rd(int pin) {
  return nrf_gpio_pin_read(pin);
}
bool touch_i2c_started = false;
/// start bit
void i2c_start() {
  if (touch_i2c_started) {
    // reset
    wr(TOUCH_PIN_SDA, 1);
    wr(TOUCH_PIN_SCL, 1);
    int timeout = TOUCH_I2C_TIMEOUT;
    while (!rd(TOUCH_PIN_SCL) && --timeout); // clock stretch
    //if (!timeout) err("Timeout (start)");
  }
  //if (!rd(TOUCH_PIN_SDA)) err("Arbitration (start)");
  wr(TOUCH_PIN_SDA, 0);
  wr(TOUCH_PIN_SCL, 0);
  touch_i2c_started = true;
}
/// stop bit
void i2c_stop() {
  wr(TOUCH_PIN_SDA, 0);
  wr(TOUCH_PIN_SCL, 1);
  int timeout = TOUCH_I2C_TIMEOUT;
  while (!rd(TOUCH_PIN_SCL) && --timeout); // clock stretch
  //if (!timeout) err("Timeout (stop)");
  wr(TOUCH_PIN_SDA, 1);
  //if (!rd(TOUCH_PIN_SDA)) err("Arbitration (stop)");
  touch_i2c_started = false;
}
/// write bit
void i2c_wr_bit(bool b) {
  wr(TOUCH_PIN_SDA, b);
  wr(TOUCH_PIN_SCL, 1);
  int timeout = TOUCH_I2C_TIMEOUT;
  while (!rd(TOUCH_PIN_SCL) && --timeout); // clock stretch
  //if (!timeout) err("Timeout (wr)");
  wr(TOUCH_PIN_SCL, 0);
  wr(TOUCH_PIN_SDA, 1); // stop forcing SDA (needed?)
}
/// read bit
bool i2c_rd_bit() {
  wr(TOUCH_PIN_SDA, 1); // stop forcing SDA
  wr(TOUCH_PIN_SCL, 1); // stop forcing SDA
  int timeout = TOUCH_I2C_TIMEOUT;
  while (!rd(TOUCH_PIN_SCL) && --timeout); // clock stretch
  //if (!timeout) err("Timeout (rd)");
  bool b = rd(TOUCH_PIN_SDA);
  wr(TOUCH_PIN_SCL, 0);
  return b;
}
/// write byte, true on ack, false on nack
bool i2c_wr(uint8_t data) {
  int i;
  for (i=0;i<8;i++) {
    i2c_wr_bit(data&128);
    data <<= 1;
  }
  return !i2c_rd_bit();
}
/// read byte
uint8_t i2c_rd(bool nack) {
  int i;
  int data = 0;
  for (i=0;i<8;i++)
    data = (data<<1) | (i2c_rd_bit()?1:0);
  i2c_wr_bit(nack);
  return data;
}
/// Write to I2C register
void touch_write(int addr, int data) {
  int iaddr = 0x15;
  i2c_start();
  i2c_wr(iaddr<<1);
  i2c_wr(addr);
  i2c_wr(data);
  i2c_stop();
  wr(TOUCH_PIN_SDA, 1);
  wr(TOUCH_PIN_SCL, 1);
}
/// Read from I2C register
void touch_read(int addr, int cnt, unsigned char *data) {
  int iaddr = 0x15;
  i2c_start();
  i2c_wr(iaddr<<1);
  i2c_wr(addr);
  i2c_start();
  i2c_wr(1|(iaddr<<1));
  for (int i=0;i<cnt;i++) {
    data[i] = i2c_rd(i==(cnt-1));
  }
  i2c_stop();
  wr(TOUCH_PIN_SDA, 1);
  wr(TOUCH_PIN_SCL, 1);
}
// =============================================================


static void prv_button_press_short(ButtonId button) {
  PebbleEvent e = {
    .type = PEBBLE_BUTTON_DOWN_EVENT,
    .button.button_id = button
  };
  event_put(&e);
  e = (PebbleEvent) {
    .type = PEBBLE_BUTTON_UP_EVENT,
    .button.button_id = button
  };
  event_put(&e);
}

// when we get a touch irq, read it and send out the relevant events
static void prv_touch_sys_task_callback(void* data) {
  unsigned char buf[6];
  touch_read(1, 6, buf);
  //PBL_LOG(LOG_LEVEL_ALWAYS, "Touch IRQ %d, %d %d", (int)nrf_gpio_pin_read(TOUCH_PIN_IRQ), buf[3], buf[5]);
  int gesture = buf[0]; // gesture
  //int touchPts = buf[1];
  //int x = buf[3], y = buf[5];
  static int lastGesture = 0;
  // forward button pressed based on swipe
  // TODO: use physical position on display so we can handle long presses?
  if (gesture != lastGesture) {
    lastGesture = gesture;
    switch (gesture) {
      case 1: prv_button_press_short(BUTTON_ID_DOWN); break; // slide down;
      case 2: prv_button_press_short(BUTTON_ID_UP); break; // slide up;
      case 3: prv_button_press_short(BUTTON_ID_BACK); break; // slide left;
      case 4: prv_button_press_short(BUTTON_ID_SELECT); break; // slide right;
    }
  }
}

static void touch_interrupt_handler(bool *should_context_switch) {
  system_task_add_callback_from_isr(prv_touch_sys_task_callback, NULL, should_context_switch);
}

void board_init(void) {
  PBL_LOG(LOG_LEVEL_ALWAYS, "board_init");
  //i2c_init(&I2C_TOUCH_IIC1_BUS);
  //i2c_use(I2C_TOUCH);
  i2c_init(&I2C_IIC2_BUS);

  // reset the touch to wake it up
  nrf_gpio_pin_set(TOUCH_PIN_SDA);
  nrf_gpio_pin_set(TOUCH_PIN_SCL);
  nrf_gpio_cfg_output(TOUCH_PIN_SDA);
  nrf_gpio_cfg_output(TOUCH_PIN_SCL);
  nrf_gpio_cfg_input(TOUCH_PIN_IRQ, NRF_GPIO_PIN_PULLUP);
  nrf_gpio_pin_clear(TOUCH_PIN_RST);
  nrf_gpio_cfg_output(TOUCH_PIN_RST);

  for (volatile int i = 0; i < 48000; i++); // FIXME 1ms. Where is a delay function??
  nrf_gpio_pin_set(TOUCH_PIN_RST);
  for (volatile int i = 0; i < 480000; i++);
  // write 0xE5,0x03 to sleep touch
  PBL_LOG(LOG_LEVEL_ALWAYS, "touch reset");

  unsigned char buf[6] = {0,0,0,0,0,0};
  touch_read(1, 6, buf);
  PBL_LOG(LOG_LEVEL_ALWAYS, "Touch %d %d %d %d %d %d", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);

  exti_configure_pin(BOARD_CONFIG_TOUCH_EXTI, ExtiTrigger_Falling, touch_interrupt_handler);
  PBL_LOG(LOG_LEVEL_ALWAYS, "touch irq en");
  exti_enable(BOARD_CONFIG_TOUCH_EXTI);

#if 0
  i2c_init(&I2C_PMIC_HRM_BUS);

  voltage_monitor_device_init(VOLTAGE_MONITOR_ALS);
  voltage_monitor_device_init(VOLTAGE_MONITOR_BATTERY);

  qspi_init(QSPI, BOARD_NOR_FLASH_SIZE);
#endif
}
