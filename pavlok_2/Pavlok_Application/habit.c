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
**	----------------------------------------------------------------------
*/
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include "nordic_common.h"
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
#include "habit.h"
#include "ble_bas.h"
#include "adc.h"
#include "leds.h"
#include "pwm.h"
#include "zap.h"
#include "configuration.h"
#include "logger.h"
#include "serial_flash.h"
#include "SEGGER_RTT.h"

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

#define HABIT_TICK_RATE_MS              		pdMS_TO_TICKS(1000)

static TimerHandle_t m_habit_timer				= NULL; 
static TaskHandle_t  m_habit_thread				= NULL; 

/**	----------------------------------------------------------------------
**	@brief	Static Forward References
**	----------------------------------------------------------------------
*/

static void habit_application_timers_stop(void)
{
    if(pdPASS != xTimerStop(m_habit_timer , OSTIMER_WAIT_FOR_QUEUE))
    {
        ASSERT(0==1);
    }
}

static void habit_task_timeout_handler(TimerHandle_t xTimer)
{
	UNUSED_PARAMETER(xTimer);
   
	if (NULL != m_habit_thread)
	{
		(void)xTaskNotifyGive( m_habit_thread );
  }
}

static void habit_task_timers_init(void)
{
    m_habit_timer     = xTimerCreate("STM" , HABIT_TICK_RATE_MS, pdTRUE, NULL, habit_task_timeout_handler);
    if(NULL == m_habit_timer)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
}


static void habit_application_timers_start(void)
{
    BaseType_t xReturn = pdFALSE;

    // start the timer only if not already started
  xReturn = xTimerIsTimerActive( m_habit_timer );
  if( pdFALSE == xReturn )
  {
    if(pdPASS != xTimerStart(m_habit_timer, OSTIMER_WAIT_FOR_QUEUE))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
  }
}

TaskHandle_t get_habit_thread_handle(void)
{
	return m_habit_thread;
}

static void habit_task_thread(void * arg)
{
	UNUSED_PARAMETER(arg);
			
	DEBUGS_APP_400("HABIT_TASK_START");
	
	/**	--------------------------------------------------------------------
	**	Do all initialization here
	**	--------------------------------------------------------------------
	*/
	habit_task_timers_init();
	habit_application_timers_start();

  while (1)
	{
		/* Wait for timeout handler to signal */
    (void)ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
//    SEGGER_RTT_WriteString(0, "HABIT_TASK\r\n");
  } // eow
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


BaseType_t habit_start_task (void)
{
  return pavlok_common_start_task (&m_habit_thread, habit_task_thread, "HABT", 256);
}


/** @} */
