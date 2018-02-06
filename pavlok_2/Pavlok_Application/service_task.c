/**	----------------------------------------------------------------------
**
**	@file
**
**  @defgroup
**  @{
**  @ingroup
**  @brief Alert Notification module.
**
**  @details This module implements
**
**  @note The application must
**
**  @note Attention!
**
**
**	----------------------------------------------------------------------
*/
// static uint32_t myvar __attribute__( ( section( "NoInit"), zero_init) );

/**	----------------------------------------------------------------------
**	@brief System Include(s)
When using the softdevice, the second ram-block is used for the application.
Make sure that both RAM-blocks are being powered under systemoff using sd_power_ramon_set(). P
lease see the nRF51 reference manual for which bits you should set in the register.

**	----------------------------------------------------------------------
*/
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include "nordic_common.h"
#include "SEGGER_RTT.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "boards.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

/**	----------------------------------------------------------------------
**	@brief Project Include(s)
**	----------------------------------------------------------------------
*/
#include "pavlok_common.h"
#include "debug.h"
#include "service_task.h"
#include "ble_bas.h"
#include "adc.h"
#include "leds.h"
#include "pwm.h"
#include "zap.h"
#include "configuration.h"
#include "notification.h"
#include "logger.h"
#include "serial_flash.h"


/**	----------------------------------------------------------------------
**	@brief	Global Data
**	----------------------------------------------------------------------
*/

/**	----------------------------------------------------------------------
**	@brief	Global Extern(s)
**	----------------------------------------------------------------------
*/

/**	----------------------------------------------------------------------
**	@brief	Static Data
**	----------------------------------------------------------------------
*/

#define SVC_TICK_RATE_MS              		pdMS_TO_TICKS(1000)
#define ACTION_TICK_RATE_MS              	pdMS_TO_TICKS(10)
#define ZAP_TOTAL_CYCLE_TIME_MS						pdMS_TO_TICKS(5000)

// TODO add union for different peripherals
typedef struct
{
	eServiceInfo_t		service_type;
	eAction_state_t		state;
	bool							started;
	uint8_t						led_type;
  uint8_t           led_pattern;
	PWM_PERCENT_T			duty_cycle;
	uint16_t					frequency;
	uint8_t						on_time;
	uint8_t						off_time;
  int32_t           zap_on;
  int32_t           zap_off;
	int8_t						total_count;
	int8_t						countdown;

} sPerformAction_t;

// TODO remove if service task is not used for alarms/applications
static sPerformAction_t				motor_action;
static sPerformAction_t				piezo_action;
static sPerformAction_t				zap_action;
static sCfg_service_t 			*	p_config 				= NULL;
static int32_t								check_battery_voltage_countdown		= PLOK_CHECK_VBATT_INTERVAL;
//static bool										piezo_action_started	= false;
//static bool										motor_action_started	= false;

static TimerHandle_t m_perform_action_timer	= NULL;
static TimerHandle_t m_service_timer				= NULL;
static TaskHandle_t  m_service_thread				= NULL;
// REMOVED static uint8_t       m_charging_blinker_state = 0;

static int kbbCounter = 0;
/**	----------------------------------------------------------------------
**	@brief	Static Forward References
**	----------------------------------------------------------------------
*/
void perform_action_timers_stop(void)
{
    if(pdPASS != xTimerStop(m_perform_action_timer , OSTIMER_WAIT_FOR_QUEUE))
    {
        ASSERT(0==1);
    }
}

// Time out is in 10ms increments
static void perform_action_timeout_handler(TimerHandle_t xTimer)
{
    UNUSED_PARAMETER(xTimer);

		// all actions start in the on state and multiple stimuli can be active

	if (0 < piezo_action.countdown)
	{
		if (ACTION_STATE_ON == piezo_action.state)
		{
			piezo_action.on_time		-= ACTION_TICK_RATE_MS;
			if (0 == piezo_action.on_time)
			{
				// turn off stimulus, reset off time and change action state
				pwm_piezo_update_duty_and_frequency(0, piezo_action.frequency);
				if (NULL != p_config)
				{
					piezo_action.on_time = p_config->piezo_value[PLOK_INDEX_ONTIME_BYTE_PIEZO];
				}
				else
				{
					ASSERT(0 == 1);
				}
				piezo_action.state = ACTION_STATE_OFF;
			}
		}
		else if (ACTION_STATE_OFF == piezo_action.state)
		{
			piezo_action.off_time		-= ACTION_TICK_RATE_MS;
			if (0 == piezo_action.off_time)
			{
				// only countdown when a full on_time/off_time cycle is done
				piezo_action.countdown	-= 1;

				// turn on stimulus, reset off time and change action state
				if (NULL != p_config)
				{
					piezo_action.off_time = p_config->piezo_value[PLOK_INDEX_OFFTIME_BYTE_PIEZO];
				}
				else
				{
					ASSERT(0 == 1);
				}

				// we have been one full cycle so we delay 250ms before starting again
				if (0 <= piezo_action.countdown)
				{
					piezo_action.state = ACTION_STATE_DELAY;
					piezo_action.total_count = PLOK_100_MS;
				}
				else
				{
					// we are done so clear the action
					(void)memset(&piezo_action, 0, sizeof(piezo_action));
					piezo_action.state = ACTION_STATE_LAST;
				}
			}
		}
		else if (ACTION_STATE_DELAY == piezo_action.state)
		{
			piezo_action.total_count		-= ACTION_TICK_RATE_MS;
			if (0 <= piezo_action.total_count)
			{
				piezo_action.state = ACTION_STATE_ON;
				pwm_piezo_update_duty_and_frequency(piezo_action.duty_cycle, piezo_action.frequency);
			}
		}
	}

	if (0 < motor_action.countdown)
	{
		if (ACTION_STATE_ON == motor_action.state)
		{
			motor_action.on_time		-= ACTION_TICK_RATE_MS;
			if (0 == motor_action.on_time)
			{
				// turn off stimulus, reset off time and change action state
				pwm_motor_update_duty_and_frequency(0, motor_action.frequency);
				if (NULL != p_config)
				{
					motor_action.on_time = p_config->motor_value[PLOK_INDEX_ONTIME_BYTE_MOTOR];
				}
				else
				{
					ASSERT(0 == 1);
				}
				motor_action.state = ACTION_STATE_OFF;
			}
		}
		else if (ACTION_STATE_OFF == motor_action.state)
		{
			motor_action.off_time		-= ACTION_TICK_RATE_MS;
			if (0 == motor_action.off_time)
			{
				// only countdown when a full on_time/off_time cycle is done
				motor_action.countdown	-= 1;

				// turn on stimulus, reset off time and change action state
				if (NULL != p_config)
				{
					motor_action.off_time = p_config->motor_value[PLOK_INDEX_OFFTIME_BYTE_MOTOR];
				}
				else
				{
					ASSERT(0 == 1);
				}

				// we have been one full cycle so we delay 250ms before starting again
				if (0 <= motor_action.countdown)
				{
					motor_action.state = ACTION_STATE_DELAY;
					motor_action.total_count = PLOK_100_MS;
				}
				else
				{
					// we are done so clear the action
					(void)memset(&motor_action, 0, sizeof(motor_action));
					motor_action.state = ACTION_STATE_LAST;
				}
			}
		}
		else if (ACTION_STATE_DELAY == motor_action.state)
		{
			motor_action.total_count		-= ACTION_TICK_RATE_MS;
			if (0 <= motor_action.total_count)
			{
				motor_action.state = ACTION_STATE_ON;
				pwm_motor_update_duty_and_frequency(motor_action.duty_cycle, motor_action.frequency);
			}
		}
	}

	if (0 < zap_action.countdown)
	{
//    SEGGER_RTT_WriteString(0, "zS\r\n");
		if (ACTION_STATE_ON == zap_action.state)
		{
			zap_action.zap_on		-= ACTION_TICK_RATE_MS;
			if (0 == zap_action.zap_on)
      {
//     SEGGER_RTT_WriteString(0, "zOnr\n");
       // we've run out of time so zap them w/o being fully charged
				zap_action.zap_on = PLOK_ONTIME_BYTE_ZAPPER;
        zap_action.state = ACTION_STATE_OFF;
				zap_gpio_enable();
      }
      else
      {
        voltsZap_t volts = getZapVoltage();
//   SEGGER_RTT_printf(0, "zC %X\r\n", volts);
        if (volts >= zap_action.duty_cycle)
        {
//   SEGGER_RTT_printf(0, "A 0x%X P 0x%X\r\n", volts, zap_action.duty_cycle);
//   SEGGER_RTT_WriteString(0, "zZ\r\n");
          zap_action.state = ACTION_STATE_OFF;

          // we are charged and ready so go ahead and release the zap as we are in the perform action state
          zap_action.zap_on = PLOK_ONTIME_BYTE_ZAPPER;
          zap_action.state = ACTION_STATE_OFF;
          zap_gpio_enable();
        }
      }
		}
		else if (ACTION_STATE_OFF == zap_action.state)
		{
//	    SEGGER_RTT_WriteString(0, "zOff\r\n");
		zap_action.off_time		-= ACTION_TICK_RATE_MS;
			if (0 == zap_action.off_time)
			{
//	    SEGGER_RTT_WriteString(0, "xF\r\n");
			// only countdown when a full zap_on/off_time cycle is done
				zap_action.countdown	-= 1;

         // reload if there is another time
				zap_action.zap_off = PLOK_OFFTIME_BYTE_ZAPPER;
        zap_action.state = ACTION_STATE_ON;

				// we have been one full cycle so we delay 250ms before starting again
				if (0 < zap_action.countdown)
        {
// 	    SEGGER_RTT_WriteString(0, "zD0\r\n");
					zap_action.state = ACTION_STATE_DELAY;
					zap_action.total_count = PLOK_100_MS;
        }
        else
				{
//    SEGGER_RTT_WriteString(0, "zDone\r\n");
					// we are done so clear the action
					(void)memset(&zap_action, 0, sizeof(zap_action));
					zap_action.state = ACTION_STATE_LAST;
				}
			}
		}
		else if (ACTION_STATE_DELAY == zap_action.state)
		{
//	    SEGGER_RTT_WriteString(0, "zD1\r\n");
			zap_action.total_count		-= ACTION_TICK_RATE_MS;
			if (0 <= zap_action.total_count)
			{
//	    SEGGER_RTT_WriteString(0, "zD2\r\n");
			zap_action.state = ACTION_STATE_ON;
        charge_zapper();
			}
		}
	}

	if ((0 == piezo_action.countdown)
			&& (0 == motor_action.countdown)
			&& (0 == zap_action.countdown))
	{
		perform_action_timers_stop();
	}
}

void perform_action_timers_init(void)
{
    m_perform_action_timer     = xTimerCreate("PATM" , ACTION_TICK_RATE_MS, pdTRUE, NULL, perform_action_timeout_handler);
    if(NULL == m_perform_action_timer)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
}

void perform_action_timers_start(void)
{
    BaseType_t xReturn = pdFALSE;

//SEGGER_RTT_WriteString(0, "P TIMER\r\n");
  // start the timer only if not already started
  xReturn = xTimerIsTimerActive( m_perform_action_timer );
  if( pdFALSE == xReturn )
  {
//SEGGER_RTT_WriteString(0, "START P TIMER\r\n");
    if(pdPASS != xTimerStart(m_perform_action_timer, OSTIMER_WAIT_FOR_QUEUE))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
  }
}

static void service_application_timers_stop(void)
{
    if(pdPASS != xTimerStop(m_service_timer , OSTIMER_WAIT_FOR_QUEUE))
    {
        ASSERT(0==1);
    }
}

static void service_task_start_led_action(uint8_t action_type, uint8_t count, uint8_t pattern)
{
  switch (action_type)
	{
    case LED_PATTERN_BLINK :
    {
      pavlok_led_bit_pattern((LED_T)pattern, count);
    }
    break;

    case LED_PATTERN_DOWN_LIGHTNING_BOLT :
    {
      pavlok_led_lightning_down(count);
    }
    break;

    case LED_PATTERN_UP_LIGHTNING_BOLT :

    {
      pavlok_led_lightning_up(count);
    }
    break;


    case LED_PATTERN_FLASH_ALL :
    {
      pavlok_led_flash_all(count);
    }
    break;

    case LED_PATTERN_REWARD :
    {
      pavlok_led_reward_pattern();
    }
    break;

    case LED_PATTERN_WARNING :
    {
      pavlok_led_warning_pattern();
    }
    break;

    default:
    {
    }
    break;
  } // eos
}

static void service_task_timeout_handler(TimerHandle_t xTimer)
{
	UNUSED_PARAMETER(xTimer);
#ifdef TBD
  if (10 > kbbCounter++)
  {
    if (EVT_SFW == current_event)
    {
      PAVLOK_LOG_ENTRY(current_event, current_sfw_event, __LINE__, 0, 0);
    }
    else if (EVT_SFW > current_event)
    {
      PAVLOK_LOG_ENTRY(current_event, param1, param2, param3, param4);
      param1++;
      param2++;
      param3++;
      param4++;
    }
    else
    {
      PAVLOK_LOG_ENTRY(current_event, param5, 0, 0, 0);
      param5++;
    }

    current_event++;
    if (current_event >= EVT_ALARM_OFF)
    {
      current_event = EVT_ZAP;
    }
   }
#endif

	if (NULL != m_service_thread)
	{
		(void)xTaskNotifyGive( m_service_thread );
  }
}

static void service_task_timers_init(void)
{
    m_service_timer     = xTimerCreate("STM" , SVC_TICK_RATE_MS, pdTRUE, NULL, service_task_timeout_handler);
    if(NULL == m_service_timer)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
}

static void check_charging(bool leds_on, millivoltsBattery_t battery_voltage)
{
  if (false == leds_on)
  {
    clear_led(LED1);
    clear_led(LED2);
    clear_led(LED3);
    clear_led(LED4);
    clear_led(LED5);
    clear_led(LED_RED);
    clear_led(LED_GREEN);
  }
  else
  {
    if (PAVLOK_BATTERY_LEVEL_3000 > battery_voltage)
    {
      set_led(LED_RED);
    }

    if (PAVLOK_BATTERY_LEVEL_3000 <= battery_voltage)
    {
      clear_led(LED_RED);
      set_led(LED1);
    }

    if (PAVLOK_BATTERY_LEVEL_3100 <= battery_voltage)
    {
      set_led(LED2);
    }

    if (PAVLOK_BATTERY_LEVEL_3150 <= battery_voltage)
    {
      set_led(LED3);
    }

    if (PAVLOK_BATTERY_LEVEL_3200 <= battery_voltage)
    {
      set_led(LED4);
    }

    if (PAVLOK_BATTERY_LEVEL_3250 <= battery_voltage)
    {
      set_led(LED5);
    }

    if (PAVLOK_BATTERY_LEVEL_3300 <= battery_voltage)
    {
      clear_led(LED1);
      clear_led(LED2);
      clear_led(LED3);
      clear_led(LED4);
      clear_led(LED5);
      set_led(LED_GREEN);
    }
  }
}




static void service_application_timers_start(void)
{
    BaseType_t xReturn = pdFALSE;

    // start the timer only if not already started
  xReturn = xTimerIsTimerActive( m_service_timer );
  if( pdFALSE == xReturn )
  {
    if(pdPASS != xTimerStart(m_service_timer, OSTIMER_WAIT_FOR_QUEUE))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
  }
}

TaskHandle_t get_service_thread_handle(void)
{
	return m_service_thread;
}

#ifdef TEST_FLASH
static uint8_t kbbTxBuffer[4096] = {0};
static uint8_t kbbRxBuffer[4096] = {0};
int send_wrapper = 0;
uint32_t  fake_address    = 0;
#endif


static void service_task_thread(void * arg)
{
	UNUSED_PARAMETER(arg);
	eServiceInfo_t					system_dirty_bit 			= SI_LAST_ENTRY;
	sPavlokServiceInfo_t	*	p_pavlok_cfg					= pavlok_get_configuration();
	p_config 																			= pavlok_get_p_service_info(SI_CFG);
  bool                    usb_check             = false;

	DEBUGS_APP_400("SVC_TASK_START");

	/**	--------------------------------------------------------------------
	**	Do all initialization here
	**	--------------------------------------------------------------------
	*/
	service_task_timers_init();
	service_application_timers_start();
	perform_action_timers_init();
	(void)memset(&motor_action, 0, sizeof(motor_action));
	(void)memset(&piezo_action, 0, sizeof(piezo_action));
	(void)memset(&zap_action, 0, sizeof(zap_action));
	motor_action.state 	= ACTION_STATE_LAST;
	piezo_action.state 	= ACTION_STATE_LAST;
	zap_action.state 		= ACTION_STATE_LAST;
  // defaults for hand detect and button enable
  p_pavlok_cfg->cfg.button_value[0] = (CFG_HAND_DETECT_BTN_LK | CFG_HAND_DETECT_HD_LR);
  p_pavlok_cfg->cfg.button_value[1] = CFG_HAND_DETECT_BP_MOTOR;

  // this is a safety call so that the zapper does not already have a charge on it when things get started
  zap_gpio_enable();

#ifdef TEST_FLASH
#define TEST_BYTE 0xEE
  for (int counter = 0; counter <  4096; counter++)
  {
    kbbTxBuffer[counter] = TEST_BYTE;
  }
#endif
	/**	--------------------------------------------------------------------
	**	This is the heart of the system
	**	All reocurring events/triggers are started from this loop
	**	--------------------------------------------------------------------
	*/

  clear_led(LED1);
  clear_led(LED2);
  clear_led(LED3);
  clear_led(LED4);
  clear_led(LED5);
  clear_led(LED6);
  clear_led(LED7);


  while (1)
	{
		uint8_t		save_data 			= 0;
		uint8_t		perform_action	= 0;
//    RTC_TIME_STRUCT_T testTime = { 0 };

		/* Wait for timeout handler to signal */
    (void)ulTaskNotifyTake( pdTRUE, portMAX_DELAY );




#ifdef TEST_FLASH
      SERIAL_FLASH_TXRX_RET_T rtn_code = SERIAL_FLASH_BUSY;
      if (fake_address > 0)
      {
        memset(kbbRxBuffer, 0, 4096);
        //SEGGER_RTT_WriteString(0, "Reading\r\n");
        rtn_code = serial_flash_read_data(kbbRxBuffer, (fake_address - 4096), 4096);
//        rtn_code = serial_flash_read_data(kbbRxBuffer, fake_address, 4096);
        if (SERIAL_FLASH_SUCCESS == rtn_code)
        {
          for (int rx_counter = 0; rx_counter < 4096; rx_counter++)
          {
            if (TEST_BYTE != kbbRxBuffer[rx_counter])
            {
              clear_led(LED_GREEN);
              set_led(LED_RED);
            }
            else
            {
              clear_led(LED_RED);
              set_led(LED_GREEN);
            }
          }
        }
      }
#endif

#ifdef TEST_FLASH
      {
        //spi2_devices_init();
      // contract SERIAL_FLASH_TXRX_RET_T spi_flash_start_write_wrapper(uint8_t *data, uint32_t address, uint32_t length);
      //if (fake_address > 0) {
      if (fake_address < (4096*3)) {
        SEGGER_RTT_WriteString(0, "Calling Write Wrapper\r\n");
        rtn_code = spi_flash_start_write_wrapper(kbbTxBuffer, fake_address, 4096);

        if (rtn_code != 0)
        {
          rtn_code = 1l;
        }
        send_wrapper++;
        if (rtn_code == SERIAL_FLASH_SUCCESS)
        {
          fake_address += 4096;
          if (fake_address >= 0x6000)
          {
            fake_address = 0;
          }

        }
      }
    }
#endif
#ifndef TEST_FLASH
//    if (!(rtc_get_time(&testTime))
//    {
//      (void)memcpy(&p_config->pavlok_time, &testTime, sizeof(testTime));
//    }

		if (0 > check_battery_voltage_countdown--)
		{
      // this updates the notification entry
			millivoltsBattery_t battery_voltage = battery_level_update();
			check_battery_voltage_countdown	= PLOK_CHECK_VBATT_INTERVAL;
      if (true == (pavlok_get_usb_pin_state()))
      {
        usb_check = false;
        // if charging then this updates the LEDs
        check_charging(usb_check, battery_voltage);
        if (true == usb_check)
        {
          usb_check = false;
        }
        else
        {
          usb_check = true;
        }
      }
		}

      /**	------------------------------------------------------------------
      **	Check to see if any stimuli has completed and if so turn it off
      **	------------------------------------------------------------------
      */
      if ((true == piezo_action.started)
          && (0 == piezo_action.countdown))
      {
        piezo_action.started	= false;
        pwm_piezo_update_duty_and_frequency(0, piezo_action.frequency);
        piezo_action.state = ACTION_STATE_LAST;
        PAVLOK_LOG_ENTRY(EVT_PIEZO, 0, __LINE__, __FILE__);
      }

      if ((true == motor_action.started)
          && (0 == motor_action.countdown))
      {
        motor_action.started	= false;
        pwm_motor_update_duty(0);
        motor_action.state = ACTION_STATE_LAST;
        PAVLOK_LOG_ENTRY(EVT_MOTOR, 0, __LINE__, __FILE__);
      }

      if ((TRAINING_STATE_COMPLETE == (pavlok_get_training_started()))
          && (0 == motor_action.countdown)
          && (0 == piezo_action.countdown))
      {
        pavlok_training_stop_stimulus();
        pavlok_set_training_started(TRAINING_STATE_INACTIVE);
      }

      system_dirty_bit	=	pavlok_get_dirty_bit();

      if (SI_LAST_ENTRY != system_dirty_bit)
      {
        DEBUGS_APP_400("system_dirty_bit");
        // look at the last thing that set it and do the function
        switch (system_dirty_bit)
        {
          case SI_CFG :
          {
            // we need type, pointer to cfg, subtype, save , perform, count, DC, FREQ, on/off
            switch (p_pavlok_cfg->cfg.characteristic)
            {
              case CFG_CHAR_MOTOR :
              {
                if (false == motor_action.started)
                {
                  // normalize to 10ms increments and reset the config struc to the normalized value
                  p_pavlok_cfg->cfg.motor_value[3] /= 10;
                  p_pavlok_cfg->cfg.motor_value[3] *= 10;

                  p_pavlok_cfg->cfg.motor_value[4] /= 10;
                  p_pavlok_cfg->cfg.motor_value[4] *= 10;

                  service_task_perform_motor_action(p_pavlok_cfg->cfg.motor_value);
                  }
              }
              break;

              case CFG_CHAR_PIEZO :
              {
                if (false == piezo_action.started)
                {
                  p_pavlok_cfg->cfg.piezo_value[3] /= 10;
                  p_pavlok_cfg->cfg.piezo_value[3] *= 10;

                  p_pavlok_cfg->cfg.piezo_value[4] /= 10;
                  p_pavlok_cfg->cfg.piezo_value[4] *= 10;

                  service_task_perform_piezo_action(p_pavlok_cfg->cfg.piezo_value);
                  }
              }
              break;

              case CFG_CHAR_ZAP :
              {
                zap_gpio_disable();
                /**	--------------------------------------------------------
                **	There can be up to a half second delay between the
                **	command and the zap depending upon the battery charge
                **	state
                **
                **	At a minimum there is a 320ms delay for 100% zap
                **	with 100% battery
                **	So with that a complete charge and zap time can be at a
                **	minimum 640ms with the battery at full charge.
                **	So I am setting the full cycle time to be 1 sec as per
                **	the old pavlok.bgs file
                **
                **	Zap is different then piezo and motor in that once it has
                **	been started it has to go to the end
                **
                **	@warn WARNING:  We play voodoo magic with this next var
                **									We transpose the action duty  cycle into
                **									the actual target voltage to make faster
                **									compares in the timer expiry above
                **									We also use the on_time element as a
                **									sate var for charging == true/discharging
                **	--------------------------------------------------------
                */
                if (false == zap_action.started)
                {
                  service_task_perform_zap_action(p_pavlok_cfg->cfg.zap_me_value);
                }
              }
              break;

              case CFG_CHAR_TIME :
              {
                // This configuration has already be validated so we alway write it to the rtc
                // contract RTC_TXRX_RET_T rtc_set_time(RTC_TIME_STRUCT_T *setTime, RTC_HOUR_FORMAT_T time_format);
                RTC_TXRX_RET_T rtn_code = rtc_set_time(&p_config->pavlok_time);
                if (RTC_SUCCESS != rtn_code)
                {
                  save_data = 0;
                }
              }
              break;

              case CFG_CHAR_LED :
              {
                uint8_t         bit_pattern   = p_pavlok_cfg->cfg.led_value[PLOK_INDEX_COLOR_BYTE_LED];
                save_data											= (p_pavlok_cfg->cfg.led_value[PLOK_INDEX_CONTROL_BYTE_LED] & PLOK_VB_BYTE_0_SAVE_DATA);
                perform_action								= (p_pavlok_cfg->cfg.led_value[PLOK_INDEX_CONTROL_BYTE_LED] & PLOK_VB_BYTE_0_PERFORM_ACTION);
                if (PLOK_VB_BYTE_0_SAVE_DATA == save_data)
                {

                }

                if (PLOK_VB_BYTE_0_PERFORM_ACTION == perform_action)
                {
                  uint8_t	action_type = LED_TYPE(p_pavlok_cfg->cfg.led_value[PLOK_INDEX_CONTROL_BYTE_LED]);
                  uint8_t	count				= (p_pavlok_cfg->cfg.led_value[PLOK_INDEX_CONTROL_BYTE_LED] & PLOK_VB_CYCLE_COUNT_MASK);

                  notification_increment_led_count();
                  service_task_start_led_action(action_type, count, bit_pattern);
                }
              }

              case CFG_CHAR_BUTTON :
              {
                /** ------------------------------------------------------
                **  states for button pushes
                **  the button must be allowed thru configration for the
                **  following to work
                **    snooze
                **    turn off an alarm
                **    increase zap strength
                **
                **  button will work regardless for the following
                **  OTA (temp)
                **  reset of course
                **--------------------------------------------------------
                */
                if (CFG_HAND_DETECT_BTN_LK == (p_pavlok_cfg->cfg.hand_detect_value[0] & CFG_HAND_DETECT_BTN_LK))
                {
                  uint16_t * button_time = (uint16_t *)&p_pavlok_cfg->cfg.button_value[2];
                  PAVLOK_LOG_ENTRY(EVT_BUTTON, * button_time, __LINE__, __FILE__);

                  notification_increment_button_count();
                  /** ----------------------------------------------------
                  ** PAVLOK_2 Rev 0.1
                  ** Start the zapper
                  **------------------------------------------------------
                  */

                  if ((false == zap_action.started)
                      && (PAVLOK_BUTTON_PRESS_1SEC <= * button_time))
                  {
                    sPavlokServiceInfo_t	*	service_info = pavlok_get_configuration();
                    uint8_t                 zap_cfg[2];

                    zap_cfg[0] = PLOK_ZAP_ONCE;
                    zap_cfg[1] = service_info->cfg.zap_me_value[PLOK_INDEX_DC_BYTE_ZAP];

                    pavlok_led_bit_pattern((LED_T)PLOK_ZAP_LED_NOTIFY, PLOK_ZAP_NOTIFY_COUNT);

                    service_task_perform_zap_action(zap_cfg);
                  }
                  else
                  {
                    p_pavlok_cfg->cfg.zap_me_value[PLOK_INDEX_DC_BYTE_ZAP] += 10;
                    if (100 < p_pavlok_cfg->cfg.zap_me_value[PLOK_INDEX_DC_BYTE_ZAP])
                    {
                      p_pavlok_cfg->cfg.zap_me_value[PLOK_INDEX_DC_BYTE_ZAP] = 50;
                    }
                  }

                  // always reset the button state so that the alarm state does not pick up the wrong time
                  p_pavlok_cfg->cfg.button_value[2] = 0;
                  p_pavlok_cfg->cfg.button_value[3] = 0;

                  // Now look at the button press config to see how we report the button press
                  if ((false == zap_action.started)
                      && (CFG_HAND_DETECT_BP_ZAP == (p_pavlok_cfg->cfg.hand_detect_value[0] & CFG_HAND_DETECT_BP_TYPE)))
                  {
                    sPavlokServiceInfo_t	*	service_info = pavlok_get_configuration();
                    uint8_t                 zap_cfg[2];

                    zap_cfg[PLOK_ZAP_BYTE_0] = PLOK_ZAP_ONCE;
                    zap_cfg[PLOK_INDEX_DC_BYTE_ZAP] = service_info->cfg.zap_me_value[PLOK_INDEX_DC_BYTE_ZAP];

                    service_task_perform_zap_action(zap_cfg);
                  }
                  else if (CFG_HAND_DETECT_BP_MOTOR == (p_pavlok_cfg->cfg.hand_detect_value[0] & CFG_HAND_DETECT_BP_TYPE))
                  {
                    sPavlokServiceInfo_t	*	service_info = pavlok_get_configuration();
                    uint8_t                 motor_cfg[PLOK_VALUE_LENGTH_VB_MOTOR];

                    motor_cfg[PLOK_INDEX_CONTROL_BYTE_MOTOR]  = PLOK_MOTOR_ONCE;
                    motor_cfg[PLOK_INDEX_FREQ_BYTE_MOTOR]     = PLOK_MOTOR_DEFAULT_FREQ;
                    motor_cfg[PLOK_INDEX_DC_BYTE_MOTOR]       = PLOK_MOTOR_DEFAULT_DC;
                    motor_cfg[PLOK_INDEX_ONTIME_BYTE_MOTOR]   = PLOK_MOTOR_DEFAULT_TIME;
                    motor_cfg[PLOK_INDEX_OFFTIME_BYTE_MOTOR]  = PLOK_MOTOR_DEFAULT_TIME;

                    service_task_perform_motor_action(motor_cfg);
                  }
                  else if (CFG_HAND_DETECT_BP_PIEZO == (p_pavlok_cfg->cfg.hand_detect_value[0] & CFG_HAND_DETECT_BP_TYPE))
                  {
                    sPavlokServiceInfo_t	*	service_info = pavlok_get_configuration();
                    uint8_t                 piezo_cfg[PLOK_VALUE_LENGTH_VB_MOTOR];

                    piezo_cfg[PLOK_INDEX_CONTROL_BYTE_PIEZO]  = PLOK_PIEZO_ONCE;
                    piezo_cfg[PLOK_INDEX_FREQ_BYTE_PIEZO]     = PLOK_PIEZO_DEFAULT_FREQ;
                    piezo_cfg[PLOK_INDEX_DC_BYTE_PIEZO]       = PLOK_PIEZO_DEFAULT_DC;
                    piezo_cfg[PLOK_INDEX_ONTIME_BYTE_PIEZO]   = PLOK_PIEZO_DEFAULT_TIME;
                    piezo_cfg[PLOK_INDEX_OFFTIME_BYTE_PIEZO]  = PLOK_PIEZO_DEFAULT_TIME;

                    service_task_perform_motor_action(piezo_cfg);
                  }
                }
              }

              default :
              {
              }
              break;
            } // eos

          if (0 != save_data)
          {
            // wrtie data to configuration table
            DEBUGS_APP_400("Save Characteristic");
          }

          pavlok_clear_dirty_bit();
        } // eoc SI_CFG
        break;
        } // eos
      }
#endif
  } // eow
}


void service_task_perform_piezo_action(uint8_t * piezo_cfg)
{
  if (ACTION_STATE_ON != piezo_action.state)
  {
    piezo_action.duty_cycle 	= piezo_cfg[2];
    piezo_action.frequency 		= (piezo_cfg[1] * PLOK_PWM_FREQ_STEP_VALUE);
    piezo_action.on_time			= (piezo_cfg[3]);
    piezo_action.off_time			= (piezo_cfg[4]);

    piezo_action.total_count 				= (piezo_cfg[0] & PLOK_VB_CYCLE_COUNT_MASK);

    piezo_action.countdown = piezo_action.total_count;

    // start the actual action
    if (!(pwm_piezo_update_duty_and_frequency(piezo_action.duty_cycle, piezo_action.frequency)))
    {
      notification_increment_piezo_count();
      piezo_action.state = ACTION_STATE_ON;
      piezo_action.started	= true;
      perform_action_timers_start();
    }
  }
}

void service_task_perform_motor_action(uint8_t * motor_cfg)
{
  if (ACTION_STATE_ON != motor_action.state)
  {
    motor_action.duty_cycle 	= motor_cfg[2];
    motor_action.frequency 		= (motor_cfg[1] * PLOK_PWM_FREQ_STEP_VALUE);
    motor_action.on_time			= (motor_cfg[3]);
    motor_action.off_time			= (motor_cfg[4]);

    motor_action.total_count 	= (motor_cfg[0] & PLOK_VB_CYCLE_COUNT_MASK);

    motor_action.countdown = motor_action.total_count;

    // start the actual action
    if (!(pwm_motor_update_duty_and_frequency(motor_action.duty_cycle, motor_action.frequency)))
    {
      motor_action.state = ACTION_STATE_ON;
      motor_action.started	= true;
      notification_increment_motor_count();
      perform_action_timers_start();
    }
  }
}

void service_task_perform_zap_action(uint8_t * zap_cfg)
{
  if (ACTION_STATE_ON != zap_action.state)
  {
    uint16_t zap_value = zap_cfg[1];
	  SEGGER_RTT_printf(0, "%d\r\n", zap_cfg[1]);
    zap_value /= 10;
    zap_value *= 51;

    // convert it to the needed zapper value

    zap_action.duty_cycle			= zap_value;
    zap_action.zap_on				  = PLOK_400_MS;
    zap_action.zap_off				= PLOK_1_SEC;
    zap_action.total_count 		= (zap_cfg[PLOK_ZAP_BYTE_0] & PLOK_ZAP_CYCLE_COUNT_MASK);
    zap_action.countdown      = zap_action.total_count;
    notification_increment_zap_count();

  //  PAVLOK_LOG_ENTRY(EVT_ZAP, 0, 0, 0, 0);

    // start the actual action
    charge_zapper();
    zap_action.started	= true;
    zap_action.state = ACTION_STATE_ON;
//	  SEGGER_RTT_printf(0, "0x%X\r\n", zap_cfg[1]);
    perform_action_timers_start();
  }
}

void service_task_perform_led_action(uint8_t * led_cfg)
{
// REMOVED  if (ACTION_STATE_ON != led_action.state)
  {
    notification_increment_led_count();
    pavlok_led_bit_pattern((LED_T)led_cfg[0], 2);
  }
}



/**	----------------------------------------------------------------------
**
**	@fn		Function
**
**	@brief	Description
**
**	@param [in]
**
**	@param	[out]
**
**	@return
**
**	@retval
**
**	@warn
**
**	----------------------------------------------------------------------
*/


BaseType_t service_start_task (void)
{
    return pavlok_common_start_task (&m_service_thread, service_task_thread, "SVCT", 256);
}

void service_action_state_set(eCFG_CHAR_LIST stimulas, eAction_state_t state)
{
	if (CFG_CHAR_MOTOR == stimulas)
	{
		motor_action.state = state;
	}
	else if (CFG_CHAR_PIEZO == stimulas)
	{
		piezo_action.state = state;
	}
	else if (CFG_CHAR_ZAP == stimulas)
	{
		zap_action.state = state;
	}
}

uint32_t		service_action_state_get(eCFG_CHAR_LIST stimulas)
{
	uint32_t	rtn_code = CFG_CHAR_LAST;

	if (CFG_CHAR_MOTOR == stimulas)
	{
		rtn_code = (uint32_t)motor_action.state;
	}
	else if (CFG_CHAR_PIEZO == stimulas)
	{
		rtn_code = (uint32_t)piezo_action.state;
	}
	else if (CFG_CHAR_ZAP == stimulas)
	{
		rtn_code = (uint32_t)zap_action.state;
	}

	return rtn_code;
}

/** @} */
