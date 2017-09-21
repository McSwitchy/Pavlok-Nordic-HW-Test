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
**  @note Attention!  TODO add mutex around spi writes
**   
**
**	----------------------------------------------------------------------
*/

/**	----------------------------------------------------------------------
**	@brief System Include(s)
**	----------------------------------------------------------------------
*/
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <nordic_common.h>
#include <nrf.h>
#include <nrf_delay.h>
#include <app_error.h>
#include <ble.h>
#include <ble_hci.h>
#include <ble_srv_common.h>
#include <ble_advdata.h>
#include <ble_advertising.h>
#include <ble_conn_params.h>
#include <boards.h>
#include <softdevice_handler.h>
#include <app_timer.h>
#include <bsp.h>
#include <bsp_btn_ble.h>
#include <mem_manager.h>

#include <FreeRTOS.h>
#include <task.h>
#include <timers.h>
#include <semphr.h>

/**	----------------------------------------------------------------------
**	@brief Project Include(s)
**	----------------------------------------------------------------------
*/
#include "pavlok_common.h"
#include "rtc.h"
#include "logger.h"
#include "serial_flash.h"

/**	----------------------------------------------------------------------
**	@brief	Global Data
**	----------------------------------------------------------------------
*/
typedef struct
{
  sNode_t   * ram_entry_list;
  uint32_t		next_read_ptr;      // always preincremented
  uint32_t		next_write_ptr;     // always postincremented
  eSFEraseState_t     sector_erase_complete;
  uint16_t    sector_erase_countdown;
  int16_t			current_entry;
  int16_t     current_count;
  int8_t      total_count;
  uint8_t     scratch_buffer[PAVLOK_LOG_ENTRY_SIZE];
} sLoggerStatus_t;

/**	----------------------------------------------------------------------
**	@brief	Global Extern(s)
**	----------------------------------------------------------------------
*/

/**	----------------------------------------------------------------------
**	@brief	Static Data
**	----------------------------------------------------------------------
*/

#define PAVLOK_LOG_ADDRESS_START					(0x7F000)
#define PAVLOK_LOG_ADDRESS_END						(0x80000)
#define PAVLOK_LOG_ADDRESS_NEXT						(PAVLOK_LOG_ENTRY_SIZE)
#define PAVLOK_LOG_INVALIDATED  					(0xFFFF)
#define LOGGER_TIME_SLOW_INTERVAL 				pdMS_TO_TICKS(1000)
#define LOGGER_TIME_FAST_INTERVAL 				pdMS_TO_TICKS(10)

static sLoggerStatus_t  log_status = {0};
static TimerHandle_t 	m_logger_timer					= NULL;
static TaskHandle_t  	m_logger_thread					= NULL;

/**	----------------------------------------------------------------------
**	@brief	Static Forward References
**	----------------------------------------------------------------------
*/
static void logger_application_timers_stop(void)
{
  if(pdPASS != xTimerStop(m_logger_timer , OSTIMER_WAIT_FOR_QUEUE))
  {
      ASSERT(0==1);
  }
}

static void logger_task_timeout_handler(TimerHandle_t xTimer)
{
  UNUSED_PARAMETER(xTimer);
  
  if (0 != log_status.sector_erase_countdown)
  {
    log_status.sector_erase_countdown -= LOGGER_TIME_FAST_INTERVAL;
    if (0 == log_status.sector_erase_countdown)
    {
      log_status.sector_erase_complete = SERIAL_FLASH_ERASE_INACTIVE;
    }
  }

  if (NULL != m_logger_thread)
  {
    (void)xTaskNotifyGive( m_logger_thread );
  }
}

static void logger_task_timers_init(void)
{
  m_logger_timer     = xTimerCreate("LOGT" , LOGGER_TIME_SLOW_INTERVAL, pdTRUE, NULL, logger_task_timeout_handler);
  if(NULL == m_logger_timer)
  {
      APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }
}

static void logger_application_timers_start(void)
{
  BaseType_t xReturn = pdFALSE;

  // start the timer only if not already started
  xReturn = xTimerIsTimerActive( m_logger_timer );
  if( pdFALSE == xReturn )
  {
    if(pdPASS != xTimerStart(m_logger_timer, OSTIMER_WAIT_FOR_QUEUE))
    {
      APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
  }
}


static void logger_application_timers_change_timeout(int32_t timeout)
{
  BaseType_t xReturn = pdFALSE;

  // start the timer only if not already started
  xReturn = xTimerIsTimerActive( m_logger_timer );
  if( pdTRUE == xReturn )
  {
    if(pdPASS != xTimerStop(m_logger_timer, OSTIMER_WAIT_FOR_QUEUE))
    {
      ASSERT(0==1);
    }
  }
  
  xReturn = xTimerChangePeriod(m_logger_timer, timeout, 10);
  ASSERT(xReturn == pdPASS);
}


/*------------------------------------------------------------------------
**
**	@fn		Function		pavlok_erase_logs
**
**	@brief	Description		clear all logs
**
**	@param [in]				None
**
**	@param	[out]			None
**
**	@return					None
**
**	@retval					None
**
**	@warn
**
**------------------------------------------------------------------------
*/
void logger_erase_logs(void)
{

}

/*------------------------------------------------------------------------
**
**	@fn		Function		logger_get_entry_count
**
**	@brief	Description		return log count
**
**	@param [in]				None
**
**	@param	[out]			uint16_t - current active log count
**
**	@return					None
**
**	@retval					None
**
**	@warn
**
**------------------------------------------------------------------------
*/
uint16_t logger_get_entry_count(void)
{
  return log_status.total_count;
}


/*------------------------------------------------------------------------
**
**	@fn		Function		pavlok_log_ram_write
**
**	@brief	Description		write logs to a ram area only and the logger
**												will write them to the serial flash
**
**	@param [in]				sPavlokLogEntry_t * - pointer to 10byte log entry
**
**	@param	[out]			None
**
**	@return					  None
**
**	@retval					  None
**
**	@warn
**
**------------------------------------------------------------------------
*/
void logger_ram_write(uint32_t event, int32_t parm1, int16_t line, const char * p_file_name)
{
  sNotificationInfo_t   * p_info        = pavlok_get_p_service_info(SI_NOTIFY);
  sPavlokLogEntry_t     * event_buffer  = (sPavlokLogEntry_t   *)nrf_malloc(PAVLOK_LOG_ENTRY_SIZE);
  RTC_TXRX_RET_T        rtn_code        = RTC_SUCCESS;
  RTC_TIME_STRUCT_T     getTime         = {0};
 
  // event, int16_t,  int16_t
  if (NULL != event_buffer)
  {
    // log count will auto roll over to 0 after 0xFFFF
    log_status.current_count += 1;
    event_buffer->entry = log_status.current_count;
    event_buffer->event = event;

    switch(event)
    {
      case EVT_BUTTON :
      case EVT_SFW :
      case EVT_BATT_LEVEL :
      case EVT_RESET_REASON :
      {
        event_buffer->value = parm1; 
      }
      break;
  
      case EVT_ZAP :
      {
        p_info->count_zap += 1;
      }
      break;
      
      case EVT_PIEZO :
      {
        p_info->count_piezo += 1;
      }
      break;
      
      case EVT_MOTOR :
      {
        p_info->count_motor += 1;
      }
      break;
      
      case EVT_LED :
      {
        p_info->count_led += 1;
      }
      break;
      

#ifdef NOT_USED // as these events are written at the entry into the function     
      case EVT_CHG_CPLT :
      case EVT_USB_ON :
      case EVT_USB_OFF :
      case EVT_ZAP_EXCEED_MAX :
      case EVT_SNOOZE :
      case EVT_ALARM_OFF :
#endif    
      
      default :
      {
        event_buffer->line = line;
        event_buffer->file_name = (char *)p_file_name;
      }
      break;
    } // eos
  
    if (RTC_SUCCESS == rtn_code)
    {
      // contract RTC_TXRX_RET_T rtc_get_time(RTC_TIME_STRUCT_T *getTime);
      rtn_code = rtc_get_time(&getTime);
      if (RTC_SUCCESS == rtn_code)
      {
        event_buffer->seconds = getTime.seconds;
        event_buffer->minutes = getTime.minutes;
        event_buffer->hours   = getTime.hours;
        event_buffer->days    = getTime.days;
        event_buffer->months  = getTime.months;
#ifdef INCLUDE_LOGGER_TASK
        pavlok_list_append(log_status.ram_entry_list, (uint8_t *)event_buffer);
#endif        
        logger_application_timers_start();
      }
    }
  }
}


uint32_t logger_entry_read(uint8_t * entry)
{
  SERIAL_FLASH_TXRX_RET_T rtn_code = SERIAL_FLASH_NULL_DATA;

  // the read pointer is alway a preincrement 
  log_status.next_read_ptr	+= 	PAVLOK_LOG_ENTRY_SIZE;
  if (PAVLOK_LOG_ADDRESS_END <= log_status.next_read_ptr)
  {
    log_status.next_read_ptr = PAVLOK_LOG_ADDRESS_START;
  }
  
  (void)memset(entry, 0, PAVLOK_LOG_ENTRY_SIZE);
  // contract SERIAL_FLASH_TXRX_RET_T serial_flash_read_data(uint8_t *data, uint32_t address, uint32_t length)
  rtn_code = serial_flash_read_data(entry, log_status.next_read_ptr, PAVLOK_LOG_ENTRY_SIZE);

  return (uint32_t)rtn_code;
}

void logger_list_init(void)
{
  uint8_t * temp = nrf_malloc(sizeof(sNode_t));
  (void)memset(&log_status, 0, sizeof(log_status));

  if (NULL != temp)
  {
    (void)memset(temp, 0, sizeof(sNode_t)); 
    log_status.ram_entry_list = (sNode_t *)temp;
  }
}

TaskHandle_t get_logger_thread_handle(void)
{
  return m_logger_thread;
}

static uint32_t logger_task_write_entry(void)
{
  // write  1 log
  sPavlokLogEntry_t       * log = (sPavlokLogEntry_t *)&log_status.scratch_buffer[0];
  sNode_t                 * new_log = pavlok_list_pop(log_status.ram_entry_list);        
  SERIAL_FLASH_TXRX_RET_T   rtn_code = SERIAL_FLASH_NULL_DATA;
    
  if (NULL != new_log)
  {         
    (void)memset(&log_status.scratch_buffer[0], 0, PAVLOK_LOG_ENTRY_SIZE);
    // write in the current ram log
    (void)memcpy(log, new_log->entry, PAVLOK_LOG_ENTRY_SIZE);
    nrf_free(new_log);
            
    // contract SERIAL_FLASH_TXRX_RET_T spi_flash_start_write(uint8_t *data, uint32_t address, uint32_t length)
    rtn_code = spi_flash_start_write_wrapper(&log_status.scratch_buffer[0], log_status.next_write_ptr, sizeof(sPavlokLogEntry_t));
    if (SERIAL_FLASH_SUCCESS == rtn_code)
    {
      log_status.total_count += 1;
      log_status.next_write_ptr += PAVLOK_LOG_ENTRY_SIZE;
      if (PAVLOK_LOG_ADDRESS_END <= log_status.next_write_ptr)
      {
        log_status.next_write_ptr = PAVLOK_LOG_ADDRESS_START;
      }
    }
  }
  else
  {
    // we're done
    logger_application_timers_stop();
    rtn_code = SERIAL_FLASH_SUCCESS;
  }  
  
  return (uint32_t)rtn_code;
}

static void logger_erase_sector(void)
{
// start the 300+ms countdown per sector
  log_status.sector_erase_countdown = SERIAL_FLASH_SECTOR_ERASE_TIMOUT;
  log_status.sector_erase_complete = SERIAL_FLASH_ERASE_ACTIVE;
  logger_application_timers_stop();
  logger_application_timers_change_timeout(LOGGER_TIME_FAST_INTERVAL);
}


static int32_t kbbCounter = 0;

static void logger_task_thread(void * arg)
{
  int32_t   rtn_code = -1;
  
  UNUSED_PARAMETER(arg);
  
  /**	--------------------------------------------------------------------
  **	Do all initialization here
  **	--------------------------------------------------------------------
  */
  log_status.sector_erase_complete  = SERIAL_FLASH_ERASE_INACTIVE;
  log_status.next_write_ptr         = PAVLOK_LOG_ADDRESS_START;
  log_status.next_read_ptr          = PAVLOK_LOG_ADDRESS_END;
  logger_task_timers_init();
  logger_application_timers_start();

  // TODO put the logger task on a log write event only instead of a timer
//log_status.sector_erase_complete = true;
  while (1)
  {
    /* Wait for timeout handler to signal */
    (void)ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
    
    if (SERIAL_FLASH_ERASE_ACTIVE != log_status.sector_erase_complete)
    {
      // conract SPI_FLASH_SM_RET_T spi_flash_sm_check_status(void) 
      rtn_code = spi_flash_tx_status();
      if (SPI_FLASH_SM_SUCCESS_DONE == rtn_code)
      {
        // make sure we are not in an erase state
        if ((log_status.next_write_ptr != log_status.next_read_ptr)
            && (SERIAL_FLASH_ERASE_INACTIVE == log_status.sector_erase_complete))
        {
          // typical write
            rtn_code = logger_task_write_entry();
            APP_ERROR_CHECK(rtn_code);
        }
        // check to see if we looped our flash sector
        else if ((log_status.next_write_ptr == log_status.next_read_ptr)
            && (SERIAL_FLASH_ERASE_INACTIVE == log_status.sector_erase_complete))
        {
          // something went wrong and the phone app has not talked to me in a while
          // so we lose all log entries
          /// @warn if no one has read the max log entries by now they are getting erased      
          rtn_code = (int32_t)serial_flash_sector_erase(PAVLOK_LOG_ADDRESS_START);
          if (0 == rtn_code)
          {
            logger_erase_sector();
          }
        }
        // we are looping the log flash sector to do a read modify write back to the sector
        else if ((PAVLOK_LOG_ADDRESS_START == log_status.next_write_ptr)
                  && (SERIAL_FLASH_ERASE_INACTIVE == log_status.sector_erase_complete))
        {
          /** ----------------------------------------------------------------
          **  We have run out of 4k block spaces to write
          **  We have to copy out the 4K block
          **  memset old log entries
          **  write back the 4K block with the new empties
          ** erase the whole block
          **  ----------------------------------------------------------------
          */
          SERIAL_FLASH_TXRX_RET_T   rtn_code = SERIAL_FLASH_TXRX_ERROR;

          (void)memset(&g_pavlok_scratch_pad[0], 0, W25X40CL_SECTOR_SIZE_BYTES);
          
          // contract SERIAL_FLASH_TXRX_RET_T serial_flash_read_data(uint8_t *, uint32_t, uint32_t);
          rtn_code =  serial_flash_read_data((uint8_t *)&g_pavlok_scratch_pad[0], PAVLOK_LOG_ADDRESS_START, W25X40CL_SECTOR_SIZE_BYTES);
          if (SERIAL_FLASH_SUCCESS == rtn_code)
          {
            uint8_t  * clear_address = (uint8_t *)PAVLOK_LOG_ADDRESS_START;
            // clear all data up to the read pointer
            while (clear_address != (uint8_t *)log_status.next_read_ptr)
            {
              (void)memset(clear_address, 0, PAVLOK_LOG_ENTRY_SIZE);
              clear_address += PAVLOK_LOG_ENTRY_SIZE;
            }
                    
            rtn_code = (int32_t)serial_flash_sector_erase(PAVLOK_LOG_ADDRESS_START);
            if (0 == rtn_code)
            {
              logger_erase_sector();
            }
          }
        }
        else
        {
        }
      } // eoif (SPI_FLASH_SM_SUCCESS_DONE == rtn_code)
    } // eoif (SERIAL_FLASH_ERASE_ACTIVE != log_status.sector_erase_complete)
  } // eow (1)
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


BaseType_t logger_start_task (void)
{
  return pavlok_common_start_task (&m_logger_thread, logger_task_thread, "LOGT", 256);
}



/** @} */
