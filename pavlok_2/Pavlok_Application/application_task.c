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
**  @note TODO have all the lists anchor added when we start the parse 
**        TODO add crc around all apps
**        TODO put all timers in ref to the portTIMOUT from rtos
**        TODO add mutex around pavlok_scratch_pad writes
**        TODO add send error back to appsvc if app load/start fails
**
**	----------------------------------------------------------------------
*/

/**	----------------------------------------------------------------------
**	@brief System Include(s)
**	----------------------------------------------------------------------
*/
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>
#include "SEGGER_RTT.h"
#include "debug.h"
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
#include "crc16.h"
#include "mem_manager.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"


/**	----------------------------------------------------------------------
**	@brief Project Include(s)
**	----------------------------------------------------------------------
*/
#include "pavlok_common.h"
#include "rtc.h"
#include "application_task.h"
#include "serial_flash.h"
#include "accel_mag.h"
#include "accel_mag.h"
#include "serial_flash.h"
#include "appsvc.h"

/**	----------------------------------------------------------------------
**	@brief	Static Data
**	----------------------------------------------------------------------
*/

#define PAVLOK_STIMULUS_ACTION_PIEZO_SIZE								(5)
#define PAVLOK_STIMULUS_ACTION_MOTOR_SIZE								(5)
#define PAVLOK_STIMULUS_ACTION_ZAP_SIZE									(2)
#define PAVLOK_CRC_INDEX                                (PAVLOK_TLV_TAG_LENGTH_SIZE)
#define PAVLOK_TLV_SIZE																	(6)
#define PAVLOK_TLV_AN_START_INDEX												(8)
#define PAVLOK_TLV_AP_INDEX											        (6)
#define PAVLOK_STIMULI_INDEX														(4)
#define PAVLOK_MAX_STIMULI_PATTERN_SIZE									(6)
#define PAVLOK_APP_TIMER_INTEVAL												pdMS_TO_TICKS(1000)
#define PAVLOK_APP_TIMER_SNOOZE_DEFAULT_TIMOUT					(300)
#define PAVLOK_APP_LOADER_TIMER_INTEVAL									pdMS_TO_TICKS(10)
#define PAVLOK_APP_LOADER_CHUNK_SIZE  									(20)
#define PAVLOK_APP_LOADER_EXTRA_TIME                     (5)
#define PAVLOK_APP_RAM_CRC_OFFSET                        (4)


#define PAVLOK_APP_RUN_LIST_ENTRIES											(8)
#define PAVLOK_APP_NAME_LENGTH													(18)


#define PAVLOK_APP_BLOCK_OFFSET														(0)
#define PAVLOK_PIEZO_BLOCK_OFFSET													(400)
#define PAVLOK_MOTOR_BLOCK_OFFSET													(800)
#define PAVLOK_ZAPPER_BLOCK_OFFSET												(1000)
#define PAVLOK_LED_BLOCK_OFFSET														(1500)
#define PAVLOK_UA_BLOCK_OFFSET														(2000)

#define PAVLOK_PIEZO_BLOCK_SIZE													(400)
#define PAVLOK_MOTOR_BLOCK_SIZE													(400)
#define PAVLOK_ZAPPER_BLOCK_SIZE												(200)
#define PAVLOK_LED_BLOCK_SIZE														(500)
#define PAVLOK_UA_BLOCK_SIZE														(2048)

#define PAVLOK_STIMULI_NAME_LENGTH                      (20)
#define PAVLOK_MAX_ALARM_SIMULUS_COUNT                    (6)
#define PAVLOK_MAX_ALARM_USER_ACTION_COUNT                (3)
/**	----------------------------------------------------------------------
**	@brief	Global Data
**	----------------------------------------------------------------------
*/

static int32_t kbbCounter = 0;

typedef struct
{
  uint8_t command;
  uint8_t sub_command;
  uint8_t length;
	char		name[16];
	
} sAppRunList_t;

typedef enum
{
  UA_INDEX_EVEN,
  UA_INDEX_ODD,
  UA_INDEX_LAST
  
} eUaIndex_t;

typedef struct
{
  ACCEL_DATA_T  data[UA_INDEX_LAST]; // 0,0 even, 0,1 odd, etc
} sUaTuple_t;

typedef struct
{
	bool							is_required;  // enabled when the alarm goes off from the rtc

	int16_t						interations;	// the index into the  UA accel read (used as an index to look for the next motion
	uint8_t						current_index;
 	sUaTuple_t        accel[10]; 
 
} sUserActionDesc_t;

typedef struct
{
  bool        complete;
  uint16_t    length;
  uint16_t    crc;
  uint16_t    bytes_rcvd;
  int16_t     expected_timeout_counter;
  
} sAppLoadDesc_t;

typedef struct
{
	bool							enabled;						// can the app be in a run state?
  bool              window_exist;
  bool              alarm_state;        // true if alarm fired and there is a window time
	char					*		alarm;							// this points to an entry in the app_ram
	uint16_t					alarm_index;
  
	// application wide required elements
	char 					*		piezo_index_start;
	char 					*		motor_index_start;
	char 					*		zap_index_start;

  // Current Alarm information
	sAlarmTime_t			time;           // initial alarm time

  /// @brief ll_aplication_list is just a pointer list to the flash applications start address;
	sNode_t	  *		ll_application_list;
  /// @brief application_current_flash_address is just a pointer to the current application in the list;
	sNode_t		*		application_current_flash_address;
  /// @brief ll_alarm_list contains the complete alarm info (TLV) for the current alarm;
	sNode_t		*		ll_alarm_list;
  /// @brief alarm_current_entry is just a pointer to the current entry in the alarm list;
	sNode_t		*		alarm_current_entry;
  /// @brief ll_stimuli_list contains all the stimuli configurations in TLV format for the current alarm
	sNode_t		*		ll_stimuli_list;
  /// @brief stimuli_current_entry is just a pointer to the current entry in the stimulus list;
	sNode_t		*		stimuli_current_entry;
  /// @brief ll_user_action_list contains all the user action configurations in TLV format for the current alarm
	sNode_t		*		ll_user_action_list;
  /// @brief stimuli_current_entry is just a pointer to the current entry in the user action list;
	sNode_t		*		user_action_current_entry;

  sUserActionDesc_t user_action;

	bool					snooze_allowed;
	bool					snooze_disabled;
	bool					alarm_allowed;
	bool					alarm_control_enabled;
  bool          start_stimulus;
  bool          start_user_action;
  int16_t       snooze_disabled_timeout;
  int16_t			  window_interval;
	int16_t			  window_time_left;
	uint8_t				current_stimulus_index;
	uint16_t			current_app_length;	
	char					current_app_name[20];	

} sApplicationState_t;

/**	----------------------------------------------------------------------
**	@brief	Global Extern(s)
**	----------------------------------------------------------------------
*/

static SemaphoreHandle_t 		m_application_mutex					= NULL;
static sApplicationState_t	m_current_status						= {0};
static TimerHandle_t 	      m_application_task_timer		= NULL;
static TaskHandle_t  	      m_application_task_thread		= NULL;
static sAppRunList_t        app_command                 = {0};
static bool						      m_config_active							= false;
static uint8_t __align(8)   current_ram_app[PAVLOK_APP_SCRATCH_SIZE];


// app loader stuff
static TimerHandle_t 	      m_application_loader_timer  = NULL;
static sAppLoadDesc_t       m_app_loader                = {0};

#ifdef REMOVED
static uint8_t	\
\
ah1[] = {'A', 'H', 0x02, 1, 0, 0, \
  'A', 'P', 8, 0, 'S', 'i', 'n', 'g', 'l', 'e', ' ', '2', \
  'H', 'A', 0xF4, 0,   \
  'A', 'N', 7, 0, 'A', 'l', 'a', 'r', 'm', ' ', '1', \
  'T', 'M', 4, 0, 0, 5, 7, 0, \
  'W', 'D', 1, 0, 30, \
  'W', 'I', 2, 0, 0x2c, 1, \
  'S', 'N', 1, 0, 1, \
  'M', 'H', 18, 0, 'M', 'N', 7, 0, 'S', 't', 'y', 'l', 'e', ' ' , '1', 'M', 'C', 0x81, 2, 100, 30, 30, \
  'P', 'H', 18, 0, 'P', 'N', 7, 0, 'S', 't', 'y', 'l', 'e', ' ' , '1', 'P', 'C', 0x81, 5, 100, 30, 30, \
  'M', 'H', 18, 0, 'M', 'N', 7, 0, 'S', 't', 'y', 'l', 'e', ' ' , '1', 'M', 'C', 0x81, 2, 100, 30, 30, \
  'P', 'H', 18, 0, 'P', 'N', 7, 0, 'S', 't', 'y', 'l', 'e', ' ' , '2', 'P', 'C', 0x87, 4, 90, 30, 30, \
  'M', 'H', 18, 0, 'M', 'N', 7, 0, 'S', 't', 'y', 'l', 'e', ' ' , '2', 'M', 'C', 0x87, 2, 90, 30, 30, \
  'L', 'H', 18, 0, 'L', 'N', 7, 0, 'S', 't', 'y', 'l', 'e', ' ' , '1', 'L', 'C', 0x81, 0xFF, 0, 0xFA, 0x64, \
  'Z', 'H', 15, 0, 'Z', 'N', 7, 0, 'S', 't', 'y', 'l', 'e', ' ' , '1', 'M', 'C', 0x81, 100, \
\
  'U', 'H', 32, 0,	\
  'U', 'N', 7, 0, 'J', 'J', 'a', 'c', 'k', 's', \
  'U', 'R', 1, 0, 10, \
  'U', 'C', 1, 0, 12, 20, 0, 0, 0, 0, 0, 0xEC, 0, 0, 0, 0, 0};


// m working x85, 02, x50, x50, x50
  
#endif
  
/**	----------------------------------------------------------------------
**	@brief	Static Forward References
**	----------------------------------------------------------------------
*/
/**	----------------------------------------------------------------------
**	@brief	Timers Section
**	----------------------------------------------------------------------
*/
/**	----------------------------------------------------------------------
**	@brief	Application Task Timers Section
**	----------------------------------------------------------------------
*/
static void application_task_timers_stop(void)
{
    if(pdPASS != xTimerStop(m_application_task_timer , OSTIMER_WAIT_FOR_QUEUE))
    {
        ASSERT(0==1);
    }
}

// for jumping jacks only
static bool even  = false;
static bool odd   = false;
static int kbbTest = 0;
static bool kbbComplete = false;

static void application_task_timeout_handler(TimerHandle_t xTimer)
{
	UNUSED_PARAMETER(xTimer);
	
  if (0 != m_current_status.snooze_disabled_timeout)
  {
    m_current_status.snooze_disabled_timeout -= 1;
    if (0 == m_current_status.snooze_disabled_timeout)
    {
      m_current_status.snooze_disabled = false;
    }
  }
  
	ACCEL_DATA_T	accel_data = {0};
//		kbbTest++;
#ifdef TBD  
	// we are looking for the current required user action
	// read the x axis for jjackflash only
	read_acceleration_after_threshold(&accel_data);

  if ((accel_data.x >= m_current_status.user_action.accel[m_current_status.user_action.current_index].data[UA_INDEX_EVEN].x)
			&& (accel_data.y >= m_current_status.user_action.accel[m_current_status.user_action.current_index].data[UA_INDEX_EVEN].y)
			&& (accel_data.z >= m_current_status.user_action.accel[m_current_status.user_action.current_index].data[UA_INDEX_EVEN].z))		
	{
		even = true;
	}

  if ((accel_data.x >= m_current_status.user_action.accel[m_current_status.user_action.current_index].data[UA_INDEX_ODD].x)
			&& (accel_data.y >= m_current_status.user_action.accel[m_current_status.user_action.current_index].data[UA_INDEX_ODD].y)
			&& (accel_data.z >= m_current_status.user_action.accel[m_current_status.user_action.current_index].data[UA_INDEX_ODD].z))		
	{
		odd = true;
	}

  if ((true == even)
      && (true == odd))
  {
    m_current_status.user_action.interations -= 1;
    if (NULL != m_application_task_thread)
    {
      (void)xTaskNotifyGive( m_application_task_thread );
    }
  }
#endif  
    if (NULL != m_application_task_thread)
    {
      (void)xTaskNotifyGive( m_application_task_thread );
    }
}

static void application_task_timers_init(void)
{
    m_application_task_timer     = xTimerCreate("STM" , PAVLOK_APP_TIMER_INTEVAL, pdTRUE, NULL, application_task_timeout_handler);
    if(NULL == m_application_task_timer)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
}

void application_task_timers_start(void)
{
  BaseType_t xReturn = pdFALSE;

    // start the timer only if not already started
  xReturn = xTimerIsTimerActive( m_application_task_timer );
  if( pdFALSE == xReturn )
  {
    if(pdPASS != xTimerStart(m_application_task_timer, OSTIMER_WAIT_FOR_QUEUE))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
  }
}

/**	----------------------------------------------------------------------
**	@brief	Application Loader Timers Section
**	----------------------------------------------------------------------
*/
static void application_loader_timers_stop(void)
{
    if(pdPASS != xTimerStop(m_application_loader_timer , OSTIMER_WAIT_FOR_QUEUE))
    {
        ASSERT(0==1);
    }
    		kbbTest = 0;
}
static void application_loader_timeout_handler(TimerHandle_t xTimer)
{
	UNUSED_PARAMETER(xTimer);
	
  if (NULL != m_application_task_thread)
  {
    (void)xTaskNotifyGive( m_application_task_thread );
  }
}

static void application_loader_timers_init(void)
{
    m_application_loader_timer     = xTimerCreate("APL" , PAVLOK_APP_LOADER_TIMER_INTEVAL, pdTRUE, NULL, application_loader_timeout_handler);
    if(NULL == m_application_loader_timer)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
}

void application_loader_timers_start(void)
{
  BaseType_t xReturn = pdFALSE;

    // start the timer only if not already started
  xReturn = xTimerIsTimerActive( m_application_loader_timer );
  if( pdFALSE == xReturn )
  {
  if (kbbTest > 0)
  {
    kbbTest = 0;
  }
    SEGGER_RTT_WriteString(0, "START_TIMERS\r\n");
    if(pdPASS != xTimerStart(m_application_loader_timer, OSTIMER_WAIT_FOR_QUEUE))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    		kbbTest++;

  } 
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**	----------------------------------------------------------------------
**	@brief	App Support Functions Section
**	----------------------------------------------------------------------
*/


/**	----------------------------------------------------------------------
**	@brief	User Action Support Functions Section
**	----------------------------------------------------------------------
*/
static void application_parse_user_action(sNode_t * alarm)
{
	sNode_t 	*	next_user_action					= m_current_status.ll_user_action_list->next;
  bool        found                     = false;
	char 			* next_index        			 	= NULL;

  return;
  if (NULL != next_user_action)
  {
    next_index  = (char *)next_user_action->entry;
    int16_t * new_length = (int16_t *)(&next_index[(PAVLOK_TLV_TAG_SIZE)]);
    (void)memset(&m_current_status.user_action, 0, sizeof(sUserActionDesc_t));
    while ((!(found)) && (0 < * new_length))
    {
      if (('U' == next_index[0])
            && ('R' == next_index[1]))
      {
            int16_t * count = (int16_t *)(&next_index[(PAVLOK_TLV_TAG_LENGTH_SIZE)]);
            if (0 < * count)
            {
              m_current_status.user_action.interations = * count;
              next_index = (&next_index[(PAVLOK_TLV_TAG_LENGTH_SIZE)]);
            }
      }
      else if (('U' == next_index[0])
              && ('C' == next_index[1]))
      {
        int16_t * length      = (int16_t *)(&next_index[(PAVLOK_TLV_TAG_SIZE)]);
        int16_t * axis_index  = NULL;
        
        // now index to the UR user action repeat index 
        //  This is an optional field and can be a value of 1 to N if it exist
        for (int32_t counter = 0; (counter < * length); counter++)
        {        
          // index to the actual config data
          axis_index = (int16_t *)(&next_index[(PAVLOK_TLV_TAG_LENGTH_SIZE)]);
        
        // now load the user action checker
        for (int32_t counter = 0; counter < m_current_status.user_action.interations; counter++)
        {
          // even
          m_current_status.user_action.accel[counter].data[UA_INDEX_EVEN].x = * axis_index++;
          m_current_status.user_action.accel[counter].data[UA_INDEX_EVEN].y = * axis_index++;
          m_current_status.user_action.accel[counter].data[UA_INDEX_EVEN].z = * axis_index++;
          // odd
          m_current_status.user_action.accel[counter].data[UA_INDEX_ODD].x = * axis_index++;
          m_current_status.user_action.accel[counter].data[UA_INDEX_ODD].y = * axis_index++;
          m_current_status.user_action.accel[counter].data[UA_INDEX_ODD].z = * axis_index++;
        }
      }
    }
      else
      {
        next_index += 1;
      }
    } // eow UI
  } // eoif (NULL != next_user_action)
}


static void application_set_next_user_action(void)
{
#ifdef REMOVED
  sNode_t         * next_user_action = (sNode_t *)0x0000FFFF;

  // walk the list, index to the configuration and send it to the peripheral
  // contract sNode_t * pavlok_list_next(sNode_t * list, sNode_t * current_entry)
  next_user_action   = pavlok_list_next(m_current_status.ll_user_action_list, NULL);
  while (NULL != next_user_action)
  {
    next_user_action   = pavlok_list_next(m_current_status.ll_user_action_list, next_user_action);
    if (NULL != next_user_action)
    {
      // parse all user actions
//      application_parse_user_action(next_user_action);
    }
  }
#endif  
}

/**	----------------------------------------------------------------------
**	@brief	Stimulus Support Functions Section
**	----------------------------------------------------------------------
*/
void application_start_stimulus(void)
{
  // always start the timer to check for snooze end, verify user actions, etc
  appsvc_send(APPSVC_CHAR_ALARM_TRIGGERED, true);
  m_current_status.start_stimulus = true;
  application_task_timers_start();
  application_task_restart();
}

// all stimulus work in the service task context so therefore do not need a app timer
void application_set_next_stimulus_pattern(void)
{
  sNode_t         * next_stimulus = (sNode_t *)0x0000FFFF;

  // walk the list, index to the configuration and send it to the peripheral
 	// contract sNode_t * pavlok_list_next(sNode_t * list, sNode_t * current_entry)
  next_stimulus   = pavlok_list_next(m_current_status.ll_stimuli_list, NULL);

  while (NULL != next_stimulus)
  {
    if (NULL != next_stimulus)
    {
      /** ----------------------------------------------------------------
      **  Stimulus chan change thru the configuration service after an
      **  application has started
      **  So we need to resetup the configuration structure for the 
      **  next stimulus pattern
      **  ----------------------------------------------------------------
      */
      pavlok_set_stimulus_on(next_stimulus);
    }
    next_stimulus   = pavlok_list_next(m_current_status.ll_stimuli_list, next_stimulus);
  }
  m_current_status.start_stimulus = false;
}

/**	----------------------------------------------------------------------
**	@brief	Flash search functions
**          The applications, the habits, logs, and the Sleep, are all in
**          a sparse table.  After searching the web it seems like the 
**          best and the slowest is to just cycle thru the whole list of 
**          sectors that have been put in an array
**	----------------------------------------------------------------------
*/
//static char   applications_lookup_table[FLASH_SECTOR_APPLICATION_END - FLASH_SECTOR_APPLICATION_START][NOTIFICATION_CHAR_APP_CURRENT_LENGTH];
//static char   habits_lookup_table[FLASH_SECTOR_HABIT_END - FLASH_SECTOR_HABIT_START][NOTIFICATION_CHAR_APP_CURRENT_LENGTH];
//static char   sleeps_lookup_table[FLASH_SECTOR_SLEEP_END - FLASH_SECTOR_SLEEP_START][NOTIFICATION_CHAR_APP_CURRENT_LENGTH];











/**	----------------------------------------------------------------------
**	@brief	Alarm Support Functions Section
**	----------------------------------------------------------------------
*/
static void application_set_stimuli(sNode_t * alarm)
{
	char 			* next_index        			 	= NULL;
	uint32_t		counter										= PAVLOK_MAX_ALARM_SIMULUS_COUNT;

  if (NULL != alarm)
  {
    sTlv_t    * alarm_tlv = NULL;
    uint16_t    length    = 0;
    /** ------------------------------------------------------------------
    *** Snooze is optional
    *** so we have to have a counter to the end of the alarm
    *** ------------------------------------------------------------------
    */
    next_index  = (char *)alarm->entry;
    alarm_tlv   = (sTlv_t *)next_index;
    length      = alarm_tlv->length;
    
    /** ------------------------------------------------------------------
    *** There can be up to 6 stimulus, which can include LH, ZH, MH,  PH
    *** ------------------------------------------------------------------
    */
    while ((0 < counter)
            && (0 < length))
    {
      if (  (('P' == next_index[0])
            && ('H' == next_index[1]))
          ||
            (('M' == next_index[0])
            && ('H' == next_index[1]))
           ||
            (('Z' == next_index[0])
            && ('H' == next_index[1]))
          ||
            (('L' == next_index[0])
            && ('H' == next_index[1])))
      {
        if (PAVLOK_MAX_ALARM_SIMULUS_COUNT == counter)
        {
          m_current_status.stimuli_current_entry  = (sNode_t *)next_index;
        }
        
        counter  -= 1;
        pavlok_list_append(m_current_status.ll_stimuli_list, (uint8_t *)next_index);
      }
        
      next_index  += 1;
      length      -= 1;
    } // eow PH, ZH, MH, LH
  }
}

static void application_set_user_action(sNode_t * alarm)
{
	char 			* next_index        			 	= NULL;
	uint32_t		counter										= PAVLOK_MAX_ALARM_USER_ACTION_COUNT;

  if (NULL != alarm)
  {
    sTlv_t    * alarm_tlv = NULL;
    uint16_t    length    = 0;
    /** ------------------------------------------------------------------
    *** Snooze is optional
    *** so we have to have a counter to the end of the alarm
    *** ------------------------------------------------------------------
    */
    next_index  = (char *)alarm->entry;
    alarm_tlv   = (sTlv_t *)next_index;
    length      = alarm_tlv->length;
    
    /** ------------------------------------------------------------------
    *** There can be up to 3 user actions
    *** ------------------------------------------------------------------
    */
    while ((0 < counter)
            && (0 < length))
    {
      if (('U' == next_index[0])
            && ('H' == next_index[1]))
      {
        if (PAVLOK_MAX_ALARM_USER_ACTION_COUNT == counter)
        {
          m_current_status.user_action_current_entry  = (sNode_t *)next_index;
          m_current_status.start_user_action = true;
        }
        
        counter  -= 1;
        pavlok_list_append(m_current_status.ll_user_action_list, (uint8_t *)next_index);
        // user actions do NOT get written over like stimuli can so we are safe in writing them
        // to the app state structure
        application_parse_user_action((sNode_t *)next_index);
      }
        
      next_index  += 1;
      length      -= 1;
    } // eow
  }
}

static void application_set_window(sNode_t * alarm)
{
  bool        found                     = false;
	char 			* next_index        			 	= NULL;

  if (NULL != alarm)
  {
    sTlv_t    * alarm_tlv = NULL;
    uint16_t    length = 0;
    /** ------------------------------------------------------------------
    *** There can be up to 3 user actions
    *** ------------------------------------------------------------------
    */
    next_index  = (char *)alarm->entry;
    alarm_tlv   = (sTlv_t *)next_index;
    length      = alarm_tlv->length;
    
    while ((!(found))
           && (0 < length))
    {
      if (('W' == next_index[0])
            && ('D' == next_index[1]))
      {
        found = true;
        m_current_status.window_time_left = next_index[PAVLOK_TLV_TAG_LENGTH_SIZE];
        m_current_status.window_time_left *= PLOK_NUM_OF_SEC_IN_A_MIN;
      }
      else
      {
        next_index += 1;
        length      -= 1;
      }
    } // eow WD
          
    /** ------------------------------------------------------------------
    *** Window interval is optional
    *** ------------------------------------------------------------------
    */
    if ((true == found)
        && (0 != m_current_status.window_time_left))
    {
      found = false;
      next_index = (char *)alarm;
      while (!(found))
      {
        if (('W' == next_index[0])
            && ('I' == next_index[1]))
        {
          found = true;
          m_current_status.window_interval = (uint16_t)next_index[PAVLOK_TLV_TAG_LENGTH_SIZE];
        }
        else
        {
          next_index += 1;
        }
      } // eow WI
    }
  }
}

static void application_set_snooze_type(sNode_t * alarm)
{
  bool        found                     = false;
	char 			* next_index        			 	= NULL;

  if (NULL != alarm)
  {
    sTlv_t    * alarm_tlv = NULL;
    uint16_t    length = 0;
    /** ------------------------------------------------------------------
    *** Snooze is optional
    *** so we have to have a counter to the end of the alarm
    *** ------------------------------------------------------------------
    */
    next_index  = (char *)alarm->entry;
    alarm_tlv   = (sTlv_t *)next_index;
    length      = alarm_tlv->length;
    
    while ((!(found))
           && (0 < length))
    {
      if (('S' == next_index[0])
            && ('N' == next_index[1]))
      {
        found = true;
        m_current_status.snooze_allowed = next_index[PAVLOK_TLV_TAG_LENGTH_SIZE]; // zero == disabled one == enabled && enabled by default
      }
      else
      {
        next_index  += 1;
        length      -= 1;
      }
    } // eow SN
  }
}

static void application_set_alarm_control(sNode_t * alarm)
{
  bool        found                     = false;
	char 			* next_index        			 	= NULL;

  if (NULL != alarm)
  {
    sTlv_t    * alarm_tlv = NULL;
    uint16_t    length = 0;
    /** ------------------------------------------------------------------
    *** Alarm control is optional
    *** so we have to have a counter to the end of the alarm
    *** ------------------------------------------------------------------
    */
    next_index  = (char *)alarm->entry;
    alarm_tlv   = (sTlv_t *)next_index;
    length      = alarm_tlv->length;
    
    while ((!(found))
           && (0 < length))
    {
      if (('A' == next_index[0])
            && ('O' == next_index[1]))
      {
        found = true;
        m_current_status.alarm_control_enabled = next_index[PAVLOK_TLV_TAG_LENGTH_SIZE]; // zero == disabled one == enabled && enabled by default
      }
      else
      {
        next_index  += 1;
        length      -= 1;
      }
    } // eow SN
  }
}


static void application_parse_next_alarm(sNode_t * alarm)
{
	sNode_t 	*	next_alarm 								= m_current_status.ll_alarm_list->next;
  bool        found                     = false;
	char 			* next_index        			 	= NULL;

  if (NULL != next_alarm)
  {
    m_current_status.alarm_current_entry = next_alarm;
    
    application_set_window(next_alarm);
    application_set_snooze_type(next_alarm);      
    application_set_alarm_control(next_alarm);
    
    application_set_stimuli(next_alarm);
    
    // normal alarms can not have a user action
    if (0 != m_current_status.window_time_left)
    {
      application_set_user_action(next_alarm);  
    }

    // find the TM start index and we are always starting at the AN TLV
    next_index  = (char *)next_alarm->entry;
    while (!(found))
    {
      if (('T' == next_index[0])
            && ('M' == next_index[1]))
      {
        found = true;
        (void)memcpy(&m_current_status.time, &next_index[PAVLOK_TLV_TAG_LENGTH_SIZE], sizeof(sAlarmTime_t));
      }
      else
      {
        next_index += 1;
      }
    } // eow TM
  } // eoif (NULL != next_alarm)
}

static sNode_t		* application_get_next_alarm(void)
{
  sNode_t   * alarm = NULL;
  bool        found = false;
  
  // check to see if we are currently in an application or just starting
  if (NULL != m_current_status.ll_alarm_list->next)
  {
    // get the next alarm for the current application
    alarm = pavlok_list_next(m_current_status.ll_alarm_list, m_current_status.alarm_current_entry);
  }
  else
  {
    // we are starting a new application
    // The app is in the current_ram_app area
    char      * next_index  = (char *)current_ram_app;
    uint16_t    app_length = m_current_status.current_app_length;
    // we are starting a new app so find the first alarm
    // index to the alarm name 
    
    while ((false == found)
           && (0 < app_length))
    {
      if (('H' == next_index[0])
           && ('A' == next_index[1]))
      {
        found = true;
      }
      else
      {
        next_index += 1;
        app_length -= 1;
      }
    }
    
    if ((0 < app_length)
        && (true == found))
    {
      alarm = (sNode_t *)next_index;
      pavlok_list_append(m_current_status.ll_alarm_list, (uint8_t *)next_index);
    }
  }
 
  return alarm;
}

uint32_t application_set_next_alarm(void)
{
  sNode_t		*       next_alarm  = NULL;
	RTC_TXRX_RET_T		rtn_code 	  = RTC_TX_ERROR;
	RTC_TIME_STRUCT_T	alarm			  = {0};
	
	next_alarm  = application_get_next_alarm();
  if (NULL != next_alarm)
  {
    application_parse_next_alarm(next_alarm);
    
    // now get the current time and then overlay the alarm time
    // contract RTC_TXRX_RET_T rtc_get_time(RTC_TIME_STRUCT_T *getTime);
    rtn_code	= rtc_get_time(&alarm);
    if (RTC_SUCCESS == rtn_code)
    {
      SEGGER_RTT_printf(0, "S %02X M %02X H %02X\r\n", alarm.seconds, alarm.minutes, alarm.hours);
      // now set the rtc
      alarm.minutes 	= m_current_status.time.minutes;
      alarm.hours 		= m_current_status.time.hours;
      alarm.seconds 	= 0;
    
      // contract RTC_TXRX_RET_T rtc_set_alarm(RTC_TIME_STRUCT_T *alarmTime);
      rtn_code = rtc_set_alarm(&alarm);
      if (RTC_SUCCESS == rtn_code)
      {
        SEGGER_RTT_printf(0, "S %02X M %02X H %02X\r\n", alarm.seconds, alarm.minutes, alarm.hours);
        m_current_status.enabled = true;
      }
    }
  }
	
	return (uint32_t)rtn_code;
}

uint32_t application_stop_current_alarm(bool command)
{
	RTC_TXRX_RET_T rtn_code	= rtc_delete_alarm();
	if (RTC_SUCCESS == rtn_code)
	{
		// now if set disable the whole app
		if (true == command)
		{
      pavlok_list_delete(m_current_status.ll_alarm_list);
      pavlok_list_delete(m_current_status.ll_stimuli_list);
      pavlok_list_delete(m_current_status.ll_user_action_list);
			m_current_status.enabled = false;
		}
	}
  
  application_task_timers_stop();
  
	return rtn_code;
}

static bool application_compare_app_names(sTlv_t * name_1)
{		
  bool    found               = false;
  char    looked_for_name[NOTIFICATION_CHAR_APP_CURRENT_LENGTH] = {0};
  char    given_name[NOTIFICATION_CHAR_APP_CURRENT_LENGTH]      = {0};

  uint16_t  length = name_1->length;

  // first turn them into strings so that we can compare
  (void)memcpy(looked_for_name, &name_1->value, length);
  (void)memcpy(given_name, app_command.name, app_command.length);
  looked_for_name[NOTIFICATION_CHAR_APP_CURRENT_LENGTH - 1] = 0;
  given_name[NOTIFICATION_CHAR_APP_CURRENT_LENGTH - 1] = 0;
  
  if (!(strncmp(looked_for_name, given_name, (PAVLOK_STIMULI_NAME_LENGTH - 1))))
  {
    found = true;
    (void)memcpy(m_current_status.current_app_name, app_command.name, app_command.length); 
    m_current_status.current_app_name[NOTIFICATION_CHAR_APP_CURRENT_LENGTH - 1] = 0;
  }
  
  return found;
}

uint32_t application_task_entry_read(uint8_t * entry)
{
	uint32_t rtn_code = NRF_ERROR_INVALID_DATA;
	

	return rtn_code;
}

/**	----------------------------------------------------------------------
**	@brief	Global Function(s)
**	----------------------------------------------------------------------
*/

// called from appsvc.c to start a new app
void application_set_ready(ble_gatts_evt_write_t * p_evt_write)
{   
  if( m_application_mutex != NULL )
	{
		if( xSemaphoreTake( m_application_mutex, ( TickType_t ) 1 ) == pdTRUE )
		{
      app_command.command     = p_evt_write->data[0];
      app_command.sub_command = p_evt_write->data[1];
      app_command.length  = (p_evt_write->len - 2);
      (void)memcpy(&app_command.name[0], &p_evt_write->data[2], (p_evt_write->len - 2));
			
      m_config_active = true;
      xSemaphoreGive( m_application_mutex );
      application_task_restart();
    }
  }
}

static void application_task_list_rebuild_from_flash(void)
{
  sAppDesc_t  * p_app_desc = pavlok_get_p_service_info(SI_APP_INFO);
  uint32_t      flash_address = 0;
  int32_t       max_sectors   = (W25X40CL_SECTOR_COUNT - 1);

pavlok_list_delete(m_current_status.ll_application_list);
  p_app_desc->count = 0;
  
 // now we go out and init the app list from the flash
  do
  {
    (void)memset(current_ram_app, 0, PAVLOK_APP_SCRATCH_SIZE);
    
    uint32_t rtn_code = (uint32_t)serial_flash_read_data(current_ram_app, flash_address, PAVLOK_APP_BLOCK_SIZE);
    if (0 == rtn_code)
    {       
      if (('A' == current_ram_app[0])
          && ('H' == current_ram_app[1]))
      {
        pavlok_list_append(m_current_status.ll_application_list, (uint8_t *)flash_address);
        p_app_desc->count += 1;
        clear_led(LED_GREEN);
        set_led(LED_1);

      }
      
      max_sectors -= 1;
      flash_address += PAVLOK_APP_SCRATCH_SIZE;
    }
  }
  while ((PAVLOK_APP_RUN_LIST_ENTRIES > p_app_desc->count)
         && (max_sectors > 0));

  SEGGER_RTT_printf(0, "%d\r\n", p_app_desc->count);

}

static bool application_find_next(bool delete)
{ 
  bool      found       = false;
  sNode_t * next_entry  = m_current_status.ll_application_list;
  uint32_t  rtn_code    = NRF_ERROR_NOT_FOUND;
  
  
  application_task_list_rebuild_from_flash();
  
  /** --------------------------------------------------------------------
  *** We must first stop the current application if we are not in a config
  *** state.
  *** We then index to the looked for app and set the appropriate pointers.
  *** ---------------------------------------------------------------------
  */
  // we always start at the head of the list which is an empty node so 
  // increment past it
  next_entry = next_entry->next;

  while ((!found)
          && (NULL != next_entry))
  {
    sTlv_t    * ah = NULL;
    char      * an = NULL;
    uint16_t  flash_address = (uint16_t)next_entry->entry;

    (void)memset(current_ram_app, 0, PAVLOK_APP_SCRATCH_SIZE);

    rtn_code = (uint32_t)serial_flash_read_data(current_ram_app, flash_address, PAVLOK_APP_SCRATCH_SIZE);
    if (0 == rtn_code)
    {
      ah = (sTlv_t *)&current_ram_app[0];
      an = (char *)ah;
    
      an += PAVLOK_TLV_AP_INDEX;
    
      // we are pointing to the app
      found = application_compare_app_names((sTlv_t *)an);
      // find the named entry in the app list
      if ((true == found)
          && (false == delete))
      {
        // all app info needs rebuilding due to reading from flash
        m_current_status.application_current_flash_address = next_entry;     
        (void)memcpy(&m_current_status.current_app_name, &an[PAVLOK_TLV_TAG_LENGTH_SIZE], (uint16_t)an[PAVLOK_TLV_TAG_SIZE]);     
        m_current_status.current_app_length    = current_ram_app[PAVLOK_TLV_TAG_SIZE];
        m_current_status.current_app_length    |= (current_ram_app[PAVLOK_TLV_TAG_SIZE + 1] << 8);
      }
      else if ((true == found)
              && (true == delete))
      {
        ret_code_t  rtn_code = serial_flash_sector_erase(flash_address);
        APP_ERROR_CHECK(rtn_code);
      }
      next_entry  = next_entry->next;
    } // eoif (0 == rtn_code)
  } // eow
  
  return found;
}


void application_delete(uint8_t command)
{
    application_find_next(true);
}

sNode_t	* application_get_next(void)
{
  return m_current_status.ll_application_list->next;
}



TaskHandle_t get_application_task_thread_handle(void)
{
	return m_application_task_thread;
}


static void application_task_init_all_list(void)
{
  uint8_t   * temp          = nrf_malloc(sizeof(sNode_t));
  
  if (NULL != temp)
  {
    (void)memset(temp, 0, sizeof(sNode_t)); 
    m_current_status.ll_application_list = (sNode_t *)temp;
  }
  
  temp = nrf_malloc(sizeof(sNode_t));
  if (NULL != temp)
  {
    (void)memset(temp, 0, sizeof(sNode_t)); 
    m_current_status.ll_alarm_list = (sNode_t *)temp;
  }
  
  temp = nrf_malloc(sizeof(sNode_t));
  if (NULL != temp)
  {
    (void)memset(temp, 0, sizeof(sNode_t)); 
    m_current_status.ll_stimuli_list = (sNode_t *)temp;
  }
  
  temp = nrf_malloc(sizeof(sNode_t));
  if (NULL != temp)
  {
    (void)memset(temp, 0, sizeof(sNode_t)); 
    m_current_status.ll_user_action_list = (sNode_t *)temp;
  }
  
  application_task_list_rebuild_from_flash();
}

static void application_flash_new_app(void)
{
  ret_code_t    check_err_code = NRF_ERROR_FORBIDDEN;
  ret_code_t    write_err_code = SERIAL_FLASH_SUCCESS;
  int32_t       flash_block_counter = 0;
  sAppDesc_t  * p_app_desc = pavlok_get_p_service_info(SI_APP_INFO);
  
  do
  {
     // verify the new load CRC
      // remember that the CRC was put in after it was calculated on the length with the CRC bytes zeroed
      // So we have to zero those bytes out first
      g_pavlok_scratch_pad[PAVLOK_APP_RAM_CRC_OFFSET] = 0;
      g_pavlok_scratch_pad[PAVLOK_APP_RAM_CRC_OFFSET + 1] = 0;
      uint16_t		crc_value = crc16_compute(g_pavlok_scratch_pad, m_app_loader.length, NULL);
    
            SEGGER_RTT_printf(0, "C %X %X\r\n", crc_value, m_app_loader.crc);
//      if (crc_value == m_app_loader.crc)
      {
        uint8_t     test_byte = 55;
        SERIAL_FLASH_TXRX_RET_T  rtn_code = SERIAL_FLASH_INVALID_ADDRESS;
        // go to the next flash app entry and write it
        uint32_t  flash_address = (flash_block_counter * PAVLOK_APP_BLOCK_SIZE);

        // we have to read and verify that the next sector is blank
        // g_pavlok_scratch_pad should be zeroed from last load
        check_err_code = (uint32_t)serial_flash_read_data(&test_byte, flash_address, 1);
        if ((SERIAL_FLASH_SUCCESS == check_err_code)
            && (0xFF == test_byte))
        {
          // contract SERIAL_FLASH_TXRX_RET_T spi_flash_start_write_wrapper(uint8_t *data, uint32_t address, uint32_t length)
          write_err_code = spi_flash_start_write_wrapper(g_pavlok_scratch_pad, flash_address, (uint32_t)m_app_loader.length);
             
          if (SERIAL_FLASH_SUCCESS == write_err_code)
          {
            sAppInfo_t * p_app_info = nrf_malloc(sizeof(sAppInfo_t));
            if (NULL != p_app_info)
            {
              p_app_desc->count           += 1;
              if (FLASH_SECTOR_APPLICATION_END < p_app_desc->count)
              {
                p_app_desc->count = 0;
              }
              p_app_info->name            = (char *)(&flash_address + PAVLOK_TLV_AP_NAME_INDEX);
              p_app_info->length          = m_app_loader.length;
              p_app_info->flash_address   = flash_address;
              p_app_desc->list            =  m_current_status.ll_application_list;
              pavlok_list_append(p_app_desc->list, (uint8_t *)p_app_info);

              set_led(LED_GREEN);
            }
          }
        }
        flash_block_counter += 1;    
      }
    }
    while ((FLASH_SECTOR_APPLICATION_END > p_app_desc->count)
            && (SERIAL_FLASH_SUCCESS == write_err_code));
      
//      appsvc_send(APPSVC_CHAR_OTA, write_err_code);
      
      // always clear the state and clean up the buffer
      (void)memset(&m_app_loader, 0, sizeof(m_app_loader));
}



static void application_task_thread(void * arg)
{
	UNUSED_PARAMETER(arg);

	/**	--------------------------------------------------------------------
	**	Do all initialization here\
	**	--------------------------------------------------------------------
	*/
  (void)memset(&m_app_loader, 0, sizeof(m_app_loader));
  (void)memset(current_ram_app, 0, PAVLOK_APP_BLOCK_SIZE);
//  (void)memset(g_pavlok_scratch_pad, 0, PAVLOK_APP_BLOCK_SIZE);
  (void)memset(&m_current_status, 0, sizeof(sApplicationState_t));
  
  m_current_status.snooze_allowed = true;
  m_current_status.alarm_control_enabled = true;

  application_task_init_all_list();
  
	m_application_mutex	= xSemaphoreCreateMutex();
  if (NULL == m_application_mutex)
  {
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }
	
	application_task_timers_init();
  application_loader_timers_init();

	/**	--------------------------------------------------------------------
	**	This is the heart of the system
	**	All reocurring events/triggers are started from this loop
	**	--------------------------------------------------------------------
	*/
  while (1)
	{		
		/* Wait for timeout handler to signal */
    (void)ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
	
    /** ------------------------------------------------------------------
    ** application alarm activated
    **  ------------------------------------------------------------------
    */
    if (true == m_current_status.start_stimulus)
    {
      m_current_status.alarm_state = true;
 
      if ((false == m_current_status.snooze_disabled)
          && (false == m_current_status.alarm_allowed))
      {
        ///@brief start the next stimulus as the alarm just fired or the window interval went off
        application_set_next_stimulus_pattern();
      }
      if (true == m_current_status.start_user_action)
      {
        ///@brief start the next action as the alarm just fired or the window interval went off
        application_set_next_user_action();
      }
    }
    
    // once we are in an alarm state we stay there for the window duration
    if (true == m_current_status.alarm_state)
    {
      /// @brief alarm window countdown timer
      m_current_status.window_time_left -= 1;
      // we have fininshed this alarm set so shut it down
      if (false == m_current_status.window_exist)
      {
        application_task_timers_stop();
       	if (0 != application_set_next_alarm())
        {
          // we are done with apps until another is loaded          
          (void)memset(current_ram_app, 0, PAVLOK_APP_BLOCK_SIZE);
          (void)memset(g_pavlok_scratch_pad, 0, PAVLOK_APP_BLOCK_SIZE);
          (void)memset(&m_current_status, 0, sizeof(sApplicationState_t));
        }
      }
      else if (0 == m_current_status.window_time_left)
      {
        application_task_timers_stop();
        if (0 != application_set_next_alarm())
        {
          // we are done with apps until another is loaded          
          (void)memset(current_ram_app, 0, PAVLOK_APP_BLOCK_SIZE);
          (void)memset(g_pavlok_scratch_pad, 0, PAVLOK_APP_BLOCK_SIZE);
          (void)memset(&m_current_status, 0, sizeof(sApplicationState_t));
        }
      }
    }
      
    if ((true == m_current_status.alarm_state)
        && ((true == odd) || (true == even)))
    {
// TODO      pavlok_training_start_stimulus();
    }
    
    /** ------------------------------------------------------------------
    **  app loader section
    **  ------------------------------------------------------------------
    */
    if (true ==  m_app_loader.complete)
    {      
      application_flash_new_app();
    }
    
    /** ------------------------------------------------------------------
    ** alarm setup
    **  ------------------------------------------------------------------
    */
		if (true == m_config_active)
		{
			if( m_application_mutex != NULL )
			{
				if( xSemaphoreTake( m_application_mutex, ( TickType_t ) 1 ) == pdTRUE )
				{
          m_config_active = false;
          
					if (APP_CONTROL_START == app_command.command)
					{
            bool found = false;

            application_stop_current_alarm(true);
            found = application_find_next(false);
						if (true == found)
            {
              (void)application_set_next_alarm();
            }
					}
					else if (APP_CONTROL_DELETE == app_command.command)
					{
						application_delete(app_command.sub_command);
					}
					else if (APP_CONTROL_STOP == app_command.command)
					{
						application_stop_current_alarm(app_command.sub_command);
					}
					else if (APP_CONTROL_SNOOZE == app_command.command)
					{
						application_set_snooze_disable(app_command.sub_command);
					}
					else if (APP_CONTROL_ALARM == app_command.command)
					{
						application_set_alarm_disable(app_command.sub_command);
					}
          
					xSemaphoreGive( m_application_mutex );
				}
			}
		} // eoif (true == m_config_active)
    
    
	} // eow (1)
}



/**	----------------------------------------------------------------------
**
**	@fn		Function
**
**	@brief	Description  copies and writes the new application from the 
**                        service to the global app ram
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

uint32_t application_write_to_ram (uint8_t * input_data, uint32_t length)
{
  sTlv_t * app_header = (sTlv_t *)input_data;
  
  application_loader_timers_start();
  
  // first data set get the overall length and the crc for the app
  // data is coming in 20 chunks at timeout intervals  
  // we give the timeout 5 timeolut intervals extra
  if (0 == m_app_loader.length)
  {
    uint16_t  * value                     = (uint16_t *)&input_data[PAVLOK_TLV_TAG_SIZE];
    m_app_loader.length                   = * value;
    value                                 += 1;
    m_app_loader.crc                      = * value;
    m_app_loader.expected_timeout_counter = ((app_header->length / PAVLOK_APP_LOADER_CHUNK_SIZE) + PAVLOK_APP_LOADER_EXTRA_TIME);    
  }
  
  
  (void)memcpy(&g_pavlok_scratch_pad[m_app_loader.bytes_rcvd], input_data, length);
  m_app_loader.bytes_rcvd += (uint32_t)length;

//  if (PAVLOK_APP_LOADER_CHUNK_SIZE > length)
  if (m_app_loader.bytes_rcvd >= m_app_loader.length)
  {
    application_loader_timers_stop();
    // we appear to be done
    if (NULL != m_application_task_thread)
    {
      m_app_loader.complete = true;
      (void)xTaskNotifyGive( m_application_task_thread );
    }    
  }
  return 0;
}

BaseType_t application_start_task (void)
{
    return pavlok_common_start_task (&m_application_task_thread, application_task_thread, "APPT", 256);
}

void application_task_restart(void)
{
	if (NULL != m_application_task_thread)
	{
		(void)xTaskNotifyGive( m_application_task_thread );
  }
}

bool application_is_in_alarm_state(void)
{
  return m_current_status.alarm_state;
}

void application_check_button_press(void)
{
  if (true == m_current_status.alarm_state)
  {
    sCfg_service_t * p_cfg_info = pavlok_get_p_service_info(SI_CFG);
    uint16_t       * button_time = (uint16_t *)&p_cfg_info->button_value[2];
    
    /** ----------------------------------------------------------------
    **  @brief The default state for the alarm and snooze is to be on
    **  so the user has to actively turn it off from the phone app
    **  to disable the ability to snooze or to turn off the alarm
    **
    **  if the button is enabled and the state is valid then allow the 
    **  user to turn off/snooze the alarm
    **  ----------------------------------------------------------------
    */
    if ((PLOK_1_SEC == * button_time)
        && (true == m_current_status.snooze_allowed)
        && (CFG_HAND_DETECT_BTN_LK == (p_cfg_info->hand_detect_value[0] & CFG_HAND_DETECT_BTN_LK)))
    {
      m_current_status.snooze_disabled = true;
      m_current_status.snooze_disabled_timeout = MIN(PAVLOK_APP_TIMER_SNOOZE_DEFAULT_TIMOUT, m_current_status.window_interval);
      appsvc_send(APPSVC_CHAR_SNOOZE_SET, true);
    }
    
    // check to see if we disable the alarm
    if ((PLOK_5_SEC == * button_time)
        && (true == m_current_status.alarm_control_enabled)
        && (CFG_HAND_DETECT_BTN_LK == (p_cfg_info->hand_detect_value[0] & CFG_HAND_DETECT_BTN_LK)))
    {
      m_current_status.alarm_allowed = false;
      application_stop_current_alarm(false);
      appsvc_send(APPSVC_CHAR_ALARM_DISABLED, true);
    }
  }
}
void application_set_alarm_disable(uint8_t value)
{
  if (true == m_current_status.alarm_control_enabled)
  {
    m_current_status.alarm_allowed = value; // disable the alarm == 0
    if (false == m_current_status.alarm_allowed)
    {
      application_stop_current_alarm(false);
    }
  }
}

void application_set_snooze_disable(uint8_t value)
{
  if (true == m_current_status.snooze_allowed)
  {
    m_current_status.snooze_disabled = value; // set snooze == 1
  }
}

void application_get_current_name(char * name)
{
  (void)memset(name, 0, NOTIFICATION_CHAR_APP_CURRENT_LENGTH);
  (void)memcpy(name, m_current_status.current_app_name, NOTIFICATION_CHAR_APP_CURRENT_LENGTH);
  name[NOTIFICATION_CHAR_APP_CURRENT_LENGTH - 1] = 0;
}

/** @} */
