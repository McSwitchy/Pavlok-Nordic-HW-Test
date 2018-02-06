
#include <stdint.h>
#include <string.h>

#include "sdk_config.h"

#include "SEGGER_RTT.h"
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "boards.h"
#include "app_timer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "fds.h"
#include "fstorage.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_systick.h"
#include "nrf_systick.h"
#include "mem_manager.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

/* PAVLOK Includes */
#include "debug.h"
#include "pavlok_common.h"
#include "training.h"
#include "accel_mag.h"
#include "notification.h"
#include "configuration.h"
#include "appsvc.h"
#include "solicited.h"
#include "log_info.h"
#include "leds.h"
#include "i2c.h"
#include "pwm.h"
#include "zap.h"
#include "spi.h"
//#include "serial_flash_fsm.h"
#include "serial_flash.h"
#include "gyro.h"

#include "app_uart.h"
#include "app_util_platform.h"

#include "logger.h"
#include "service_task.h"
#include "application_task.h"
#include "serial_flash.h"
#include "habit.h"

// Change this when an incompatible interface change is made.
#define VERSION 0x0004
#define MAGIC_COOKIE 0x12980379

typedef enum {
  AH_CMD_NONE         = 0,
  AH_CMD_SET_LED      = 1,
  AH_CMD_CLEAR_LED    = 2,
  AH_CMD_DELAY        = 3,
  AH_CMD_GET_SYSTICK  = 4,
  AH_CMD_TEST_FLASH   = 5,
  AH_CMD_TEST_RTC     = 6,
  AH_CMD_TEST_GYRO    = 7,
  AH_CMD_TEST_ACCEL   = 8,
  AH_CMD_READ_IN      = 9,
  AH_CMD_READ_OUT     = 10,
  AH_CMD_TEST_PIEZO   = 11,
  AH_CMD_TEST_MOTOR   = 12,
  AH_CMD_READ_ADC     = 13,
  AH_CMD_SET_ZAP      = 14,
  AH_CMD_I2C_SCAN     = 15,
  AH_CMD_TEST_ACCEL2  = 16,
  AH_CMD_ZAP_CHARGE   = 17,
}   e_ate_cmd;


volatile struct {
    uint32_t      magic;    // 00
    uint16_t      version;  // 04
    uint16_t      mark;     // 06
    int32_t       _res1;    // 08
    uint32_t      tick;     // 0c

    uint32_t      cmd;      // 10
    uint32_t      prevcmd;  // 14
    int32_t       live;     // 18
    int32_t       result;   // 1c

    uint32_t      data[4];  // 20-24-28-2c
} gModel __attribute__((section (".ate_fixed")));


/**@brief Function for initializing the clock.
 */
static void clock_init(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
}

#if 1
void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName )
{
  APP_ERROR_HANDLER(4);
}

/**@brief A function which is hooked to idle task.
 * @note Idle hook must be enabled in FreeRTOS configuration (configUSE_IDLE_HOOK).
 */
void vApplicationIdleHook( void )
{

}

#endif



void main_thread(void * arg)
{
  UNUSED_PARAMETER(arg);
  uint8_t ret = 0;

  gModel.mark = 100;

  bool zap_charge_control = false;
  int zap_charge_target = 0;
  int zap_charge_level = 0;
  bool charging = false;

  while (1) {
      while (gModel.cmd == AH_CMD_NONE) {
          gModel.tick++;

          // closed loop control of zapper
          if (zap_charge_control) {
                gModel.live = zap_charge_level = getZapVoltage();

                if (charging) {
                    if (zap_charge_level >= zap_charge_target) {
                        pwm_zap_stop();
                        charging = false;
                    }
                } else {
                    if (zap_charge_level < zap_charge_target - 10) { // arbitrary, matches main code v5.1.1
                        pwm_zap_start();
                        charging = true;
                    }
                }

                nrf_delay_ms(1);
          } else if (charging) {
                pwm_zap_stop();
                charging = false;
                gModel.live = 0;
          }
      }

      int result = gModel.result = -1;
      switch (gModel.cmd) {
        case AH_CMD_SET_LED: // 1
          if (gModel.data[0] < 1 || gModel.data[0] > 7)
            result = 2;
          else
            result = (set_led(gModel.data[0] - 1) == LED_OP_SUCCESS) ? 0 : 1;
          break;

        case AH_CMD_CLEAR_LED: // 2
          if (gModel.data[0] < 1 || gModel.data[0] > 7)
            result = 2;
          else
            result = (clear_led(gModel.data[0] - 1) == LED_OP_SUCCESS) ? 0 : 1;
          break;

        case AH_CMD_DELAY: // 3
          nrf_delay_ms(gModel.data[0] < 10000 ? gModel.data[0] : 10000);
          result = 0;
          break;

        case AH_CMD_GET_SYSTICK: // 4
          result = nrf_systick_val_get();
          break;

        case AH_CMD_TEST_FLASH: // 5
          result = demo_flash_read_write_read() ? 0 : 1;
          break;

        case AH_CMD_TEST_RTC: // 6
          ret = rtc_init();
          if (ret) {
            RTC_TIME_STRUCT_T setTime = {0};
            RTC_TIME_STRUCT_T getTime = {0};
            // memset(&setTime, 0, sizeof(setTime);
            // memset(&getTime, 0, sizeof(getTime);
            rtc_set_time(&setTime);
            nrf_delay_ms(1010);
            rtc_get_time(&getTime);
            result = (getTime.seconds >= 1) ? 0 : 1;
          } else
            result = 2;
          break;

        case AH_CMD_TEST_GYRO: // 7
          result = gyro_whoami() ? 0 : 1;
          break;

        case AH_CMD_TEST_ACCEL: // 8
          result = accelerometer_whoami(I2C_ACC_MAG_ADDRESS);
          result = (result == FXOS8700CQ_WHOAMI_VAL) ? 1 : 0;
          break;

        case AH_CMD_READ_IN: // 9
          result = nrf_gpio_pin_read(gModel.data[0]);
          break;

        case AH_CMD_READ_OUT: // 10
          result = nrf_gpio_pin_out_read(gModel.data[0]);
          break;

        case AH_CMD_TEST_PIEZO: // 11
          result = pwm_piezo_update_duty_and_frequency(gModel.data[0], 3000);
          break;

        case AH_CMD_TEST_MOTOR: // 12
          result = pwm_motor_update_duty_and_frequency(gModel.data[0], 4000);
          break;

        case AH_CMD_READ_ADC: // 13
          result = ADC_RESULT_IN_MILLI_VOLTS(adc_sample_channel(gModel.data[0]));
          // result = adc_sample_channel(gModel.data[0]);
          break;

        case AH_CMD_SET_ZAP: // 14
          if (gModel.data[0])
            vbatt_measure_enable();
          else
            vbatt_measure_disable();
          result = 0;
          break;

        case AH_CMD_I2C_SCAN:   // 15
            result = i2c_scan(gModel.data[0], (uint8_t *) gModel.data, sizeof(gModel.data));
            break;

        case AH_CMD_TEST_ACCEL2: // 16
            result = accelerometer_whoami(I2C_ACC_MAG_ADDRESS);
            if (result >= 0) {
                result |= (I2C_ACC_MAG_ADDRESS << 8);
            }
            else {
                result = accelerometer_whoami(0x1c);    // address for MMA8451Q
                if (result >= 0)
                    result |= (0x1c << 8);
            }

            // Warning: a failure to find a chip results in a -1 result,
            // which looks (for now) like we're still running this test.
            // To avoid that being a problem, don't rely on result == -1
            // to mean test is active... consider it a cosmetic feature.
            // "Test active" should be only "cmd != AH_CMD_NONE".
            break;

        case AH_CMD_ZAP_CHARGE: {   // 17
            zap_charge_control = gModel.data[0];
            zap_charge_target = gModel.data[1];

            if (!zap_charge_control) {
                result = zap_charge_level;
            } else {
                result = 0;
            }
            break;
        }

        default:
          result = -2;
          break;
      }

      // handshake with ATE client
      gModel.prevcmd = gModel.cmd;
      gModel.result = result;
      gModel.cmd = AH_CMD_NONE;
  }
}



#define GYRO_RESET_PIN      (2)
// #define ACC_RESET_PIN       (26)
#define CHG_PIN             (25)


int main(void)
{
  gModel.mark = 1;

  for (int i = 0; i < sizeof(gModel.data) / sizeof(gModel.data[0]); i++)
      gModel.data[i] = 0;
  gModel.result = 0;
  gModel.cmd = gModel.prevcmd = AH_CMD_NONE;

  gModel._res1 = gModel.live = 0;
  gModel.version = VERSION;
  gModel.magic = MAGIC_COOKIE;  // arbitary val

  clock_init();
  nrf_drv_systick_init();

  // timers_init();

  gModel.mark = 2;
  init_leds();
  adc_init();
	i2c_devices_init();
	spi2_devices_init();

  gModel.mark = 5;
  pwm_piezo_init();
  pwm_motor_init();
  pwm_zap_init();
  zap_gpio_init();
  vbatt_measure_init();
  pavlok_vusb_init();

  // accelerometer_init_pulse();

  gModel.mark = 7;
  nrf_gpio_cfg_input(CHG_PIN, NRF_GPIO_PIN_NOPULL);

  nrf_gpio_cfg_output(GYRO_RESET_PIN);
  nrf_gpio_cfg_output(PAVLOK_ACCEL_MAG_RESET_PIN);

  gModel.mark = 9;
  nrf_gpio_pin_clear(PAVLOK_ACCEL_MAG_RESET_PIN);
  nrf_gpio_pin_set(GYRO_RESET_PIN);
  nrf_delay_ms(100);

  gModel.mark = 99;

  main_thread(NULL);
}

