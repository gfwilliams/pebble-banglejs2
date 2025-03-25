#include "drivers/battery.h"
#include "drivers/gpio.h"

#include "system/passert.h"
#include "services/common/battery/battery_state.h"
#include "services/common/battery/battery_curve.h"
#include "services/common/system_task.h"
#include "system/logging.h"
#include "kernel/events.h"
#include "drivers/exti.h"

#include "util/math.h"
#include "util/net.h"

static void prv_vusb_sys_task_callback(void* data) {
  PebbleEvent event = {
    .type = PEBBLE_BATTERY_CONNECTION_EVENT,
    .battery_connection = {
      .is_connected = battery_is_usb_connected_impl(),
    }
  };
  event_put(&event);
}

static void vusb_interrupt_handler(bool *should_context_switch) {
  system_task_add_callback_from_isr(prv_vusb_sys_task_callback, NULL, should_context_switch);
}

void battery_init(void) {
  nrf_gpio_cfg_input(BOARD_CONFIG_POWER.vusb_stat.gpio_pin, NRF_GPIO_PIN_PULLUP);
  nrf_gpio_cfg_input(BOARD_CONFIG_POWER.chg_stat.gpio_pin, NRF_GPIO_PIN_PULLUP);
  //exti_configure_pin(BOARD_CONFIG_POWER.vusb_gpiote, ExtiTrigger_RisingFalling, vusb_interrupt_handler);
  //exti_enable(BOARD_CONFIG_POWER.vusb_gpiote);
  // FIXME: can cause instability?
}

int battery_get_millivolts(void) {
  return 4000;
  // FIXME need ADC read on D3
}

bool battery_charge_controller_thinks_we_are_charging_impl(void) {
  return nrf_gpio_pin_read(BOARD_CONFIG_POWER.chg_stat.gpio_pin)==0;
}

bool battery_is_usb_connected_impl(void) {
  return nrf_gpio_pin_read(BOARD_CONFIG_POWER.vusb_stat.gpio_pin)==0;
}

void battery_set_charge_enable(bool charging_enabled) {
}

void battery_set_fast_charge(bool fast_charge_enabled) {
}
