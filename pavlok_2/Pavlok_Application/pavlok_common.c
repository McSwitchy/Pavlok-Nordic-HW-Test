/*------------------------------------------------------------------------
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
**  @note Attention! TODO get nrf_delays out of LED functions
**                    TODO must put locks around stimulus during app alarms
**                    TODO move LED function s to service task
**   
**
**------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------
**	@brief System Include(s)
**------------------------------------------------------------------------
*/
#include <stdint.h>
#include <string.h>
#include <nrf_assert.h>
#include <nrf_delay.h>
#include <ble_gap.h>
#include "mem_manager.h"

/*------------------------------------------------------------------------
**	@brief Project Include(s)
**------------------------------------------------------------------------
*/
#include "pavlok_common.h"
#include "adc.h"
#include "leds.h"
#include "debug.h"
#include "service_task.h"

/*------------------------------------------------------------------------
**	@brief	Global Data
**------------------------------------------------------------------------
*/
uint8_t                 encoded_info[PAVLOK_MAX_MSG_SIZE];
const ble_uuid128_t     base_uuid128 = PAVLOK_BASE_UUID;


/*------------------------------------------------------------------------
**	@brief	Global Extern(s)
**------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------
**	@brief	Static Data
**------------------------------------------------------------------------
*/

static	bool									pavlok_configuration_is_dirty = false;
static 	sPavlokServiceInfo_t 	service_info;
uint8_t __align(8) g_pavlok_scratch_pad[32];

size_t pavlok_strnlen (const char* s, size_t maxlen)
{
	size_t len = 0;

	while ((len <= maxlen) && (*s != NULL))
	{
		s++;
		len++;
	}

return len;
}

/*------------------------------------------------------------------------
**	@brief	Static Forward References
**------------------------------------------------------------------------
*/
void pavlok_restart_service_task(void)
{
	TaskHandle_t service_task_handle = get_service_thread_handle();
	if (NULL != service_task_handle)
	{
		(void)xTaskNotifyGive( service_task_handle );
  }
	// TODO do something if task crashed
}



/*------------------------------------------------------------------------
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
**------------------------------------------------------------------------
*/

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */

void pavlok_timers_init(char * timerTextName, TimerHandle_t * timerId, uint32_t timerInterval, timerCB cb)
{
  * timerId  = xTimerCreate(timerTextName , timerInterval, pdTRUE, NULL, cb);
  if  (NULL == timerId)
  {
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    ASSERT(NULL != timerId);
  }
}

void pavlok_encode(uint8_t * value, uint8_t * p_encoded_buffer, uint8_t length)
{
    switch (length)
    {
        // also uses case PAVLOCK_SIZEOF_BOOL :
        case PAVLOK_SIZEOF_UINT8 :
        {
            p_encoded_buffer[0] = * value;
        }
        break;

        case PAVLOK_SIZEOF_UINT16 :
        {
            uint16_encode(* value, p_encoded_buffer);
        }
        break;

        case PAVLOK_SIZEOF_UINT32 :
        {
            uint32_encode(* value, p_encoded_buffer);
        }
        break;

        default : // for strings etc.
        {
            uint32_t loopCounter = 0;
            uint32_t max_size = MIN(length, PAVLOK_MAX_MSG_SIZE);

            for (loopCounter = 0; loopCounter < max_size; loopCounter++)
            {
                p_encoded_buffer[loopCounter] = value[loopCounter];
            }
						
        }
        break;
    } // eos
}


uint8_t * pavlok_common_get_encode_buffer(void)
{
  return &encoded_info[0];
}


/*------------------------------------------------------------------------
**
**	@fn		Function		pavlok_common_start_task
**
**	@brief	Description		common systemn task creation
**
**	@param [in]				eServiceInfo_t - the service type
**
**	@param [out]			void * 		 	- ointer to configuration block
**
**	@return
**
**	@retval					None
**
**	@warn
**
**------------------------------------------------------------------------
*/
BaseType_t pavlok_common_start_task (TaskHandle_t  * m_thread, taskName_t tName, char * textName, uint32_t stackDepth)
{
  BaseType_t xReturn = pdFAIL;

  // Start execution.
  xReturn = xTaskCreate(tName, textName, stackDepth, NULL, 0, m_thread);

  return xReturn;
}

/*------------------------------------------------------------------------
**
**	@fn		Function		pavlok_get_p_service_info
**
**	@brief	Description		get individual service config block
**
**	@param [in]				eServiceInfo_t - the service type
**
**	@param [out]			void * 		 	- ointer to configuration block
**
**	@return
**
**	@retval					None
**
**	@warn
**
**------------------------------------------------------------------------
*/
void * pavlok_get_p_service_info(eServiceInfo_t service)
{
	void * p_rtn = NULL;
	
	switch (service)
	{
		case SI_NOTIFY :
		{
			p_rtn = &service_info.notify;
		}
		break;
		
		case SI_CFG :
		{
			p_rtn = &service_info.cfg;
		}
		break;
		
		case SI_APP_INFO :
		{
			p_rtn = &service_info.applications;
		}
		break;
		
		default :
		{
			ASSERT(0==1);
		}
	} // eos

	return p_rtn;
}

/*------------------------------------------------------------------------
**
**	@fn		Function		pavlok_set_dirty_bit
**
**	@brief	Description		get system configuraiton dirty bit
**
**	@param [in]				eServiceInfo_t - the service type
**	@param [in]				void * 		 	- ointer to configuration block
**
**	@param	[out]			None
**
**	@return
**
**	@retval					None
**
**	@warn
**
**------------------------------------------------------------------------
*/
void pavlok_set_dirty_bit(eServiceInfo_t service, void * service_object)
{
	uint8_t 					* p_service 	= NULL;
	uint8_t							object_size = 0;
	
	switch (service)
	{
		case SI_CFG :
		{
			DEBUGI_APP_400("object_type", service_info.cfg.characteristic);
			
			if (CFG_CHAR_MOTOR == service_info.cfg.characteristic)
			{
				p_service	= (uint8_t *)&service_info.cfg.motor_value;
				object_size = PLOK_VALUE_LENGTH_VB_MOTOR;
			}
			else if (CFG_CHAR_PIEZO == service_info.cfg.characteristic)
			{
				p_service	= (uint8_t *)&service_info.cfg.piezo_value;
				object_size = PLOK_VALUE_LENGTH_PIEZO;
			}
			else if (CFG_CHAR_ZAP == service_info.cfg.characteristic)
			{
				p_service	= (uint8_t *)&service_info.cfg.zap_me_value;
				object_size = PLOK_VALUE_LENGTH_ZAP;
			}
			else if (CFG_CHAR_LED == service_info.cfg.characteristic)
			{
				p_service	= (uint8_t *)&service_info.cfg.led_value;
				object_size = PLOK_VALUE_LENGTH_LED;
			}
			else if (CFG_CHAR_HD == service_info.cfg.characteristic)
			{
				p_service	= (uint8_t *)&service_info.cfg.hand_detect_value;
				object_size = PLOK_VALUE_LENGTH_HD;
			}
			else if (CFG_CHAR_BUTTON == service_info.cfg.characteristic)
			{
				p_service	= (uint8_t *)&service_info.cfg.button_value;
				object_size = PLOK_VALUE_LENGTH_BUTTON;
			}
			else 
			{
				ASSERT(0==1);
			}
			(void)memcpy(p_service, service_object, object_size);
		}
		break;
#ifdef REMOVED		
		case SI_RTC :
		{
			(void)memcpy((uint8_t *)&service_info.rtc, (uint8_t *)service_object, sizeof(sRtcInfo_t));
		}
		break;
		
		case SI_ACCEL :
		{
			(void)memcpy((uint8_t *)&service_info.accel, (uint8_t *)service_object, sizeof(sAccelSvcInfo_t));
		}
		break;
#endif		
		default :
		{
			ASSERT(0==1);
		}
		break;
	} // eos
	
	service_info.last_service = service;
	pavlok_configuration_is_dirty	= true;
	pavlok_restart_service_task();
}

/*------------------------------------------------------------------------
**
**	@fn		Function		pavlok_get_dirty_bit
**
**	@brief	Description		get system configuraiton dirty bit
**
**	@param [in]				None
**
**	@param	[out]			None
**
**	@return					eServiceInfo_t - the service type
**
**	@retval					None
**
**	@warn
**
**------------------------------------------------------------------------
*/
eServiceInfo_t pavlok_get_dirty_bit(void)
{
	eServiceInfo_t	rtn_type	= SI_LAST_ENTRY;
	
	if (true == pavlok_configuration_is_dirty)
	{
		rtn_type = service_info.last_service;
	}
	return rtn_type;
}

/*------------------------------------------------------------------------
**
**	@fn		Function		pavlok_clear_dirty_bit
**
**	@brief	Description		clear system configuraiton dirty bit
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
void  pavlok_clear_dirty_bit(void)
{
	pavlok_configuration_is_dirty	= false;
}

/*------------------------------------------------------------------------
**
**	@fn		Function		pavlok_get_configuration
**
**	@brief	Description		get overall system config struct
**
**	@param [in]				None
**
**	@param	[out]			None
**
**	@return					sPavlokServiceInfo_t * - pointer to sys config
**
**	@retval
**
**	@warn
**
**------------------------------------------------------------------------
*/
sPavlokServiceInfo_t	*	pavlok_get_configuration(void)
{
	return &service_info;
}

/*------------------------------------------------------------------------
**
**	@fn		Function		pavlok_get_time
**
**	@brief	Description		get time from  rtc chip
**
**	@param [in]				sTime_t *   pointer to copy data to
**
**	@param	[out]			None
**
**	@return					None
**
**	@retval
**
**	@warn
**
**------------------------------------------------------------------------
*/
void pavlok_get_time(sTime_t * current_time)
{
	(void)memcpy(current_time, &service_info.cfg.pavlok_time, sizeof(sTime_t));
}


uint32_t pavlok_button_normalize(uint32_t m_button_time)
{
	uint32_t	pavlok_time = 0;

	if ((PAVLOK_BUTTON_PRESS_1SEC <= m_button_time)
			&& (PAVLOK_BUTTON_PRESS_3SEC > m_button_time))
	{
		pavlok_time = PAVLOK_BUTTON_PRESS_1SEC;
	}
	else if ((PAVLOK_BUTTON_PRESS_2SEC <= m_button_time)
			&& (PAVLOK_BUTTON_PRESS_5SEC > m_button_time))
	{
		pavlok_time = PAVLOK_BUTTON_PRESS_3SEC;
	}
	else if ((PAVLOK_BUTTON_PRESS_5SEC <= m_button_time)
			&& (PAVLOK_BUTTON_PRESS_8SEC >m_button_time))
	{
		pavlok_time = PAVLOK_BUTTON_PRESS_5SEC;
	}
	else if (PAVLOK_BUTTON_PRESS_8SEC <= m_button_time)
	{
		pavlok_time = PAVLOK_BUTTON_PRESS_8SEC;
	}

	return pavlok_time;
}

void pavlok_led_clear_all(void)
{
	int8_t	loop_counter 	= 0;
	
	for (loop_counter = 0; loop_counter < (uint8_t)NUM_OF_LEDS; loop_counter++) 
	{
		clear_led((LED_T)loop_counter);
	}
}	

#define PAVLOK_LED_BLINK_MASK (0x80)

void pavlok_led_bit_pattern(LED_T pattern, uint8_t count)
{
	int8_t	bit_mask			= 0;
	int8_t	led_counter 	= 0;
	int8_t	loop_counter 	= 0;
	int8_t	bit						= 7;
	
	pavlok_led_clear_all();
	
	for (loop_counter = 0; loop_counter < count; loop_counter++)
	{
		bit = 7;
		for (led_counter = 0x40; led_counter > 0; led_counter >>= 1) 
		{		
			bit_mask = ((uint8_t)pattern & led_counter);
			if (0 != bit_mask)
			{
				set_led((LED_T)(bit - 1));
			}
			bit -= 1;
		}
		
		nrf_delay_ms(100);	
		
		if (0 == (PAVLOK_LED_BLINK_MASK & pattern))
		{
			nrf_delay_ms(150);		
			break;
		}
		
		pavlok_led_clear_all();
		nrf_delay_ms(100);					
	}
	pavlok_led_clear_all();
}	


void pavlok_led_lightning_down(uint8_t count) 
{
	int8_t	loop_counter = 0;
	int8_t flash_counter = 0;
	
	pavlok_led_clear_all();
	
	for (flash_counter = 0; flash_counter < count; flash_counter ++)
	{
		for (loop_counter = 0; loop_counter < 5; loop_counter++)
		{
			set_led((LED_T)loop_counter);
			nrf_delay_ms(50);
		}
			
		nrf_delay_ms(50);
		pavlok_led_clear_all();
	}
}

void pavlok_led_lightning_up(uint8_t count) 
{
	int8_t	loop_counter = 0;
	int8_t flash_counter = 0;
	
	pavlok_led_clear_all();
	
	for (flash_counter = 0; flash_counter < count; flash_counter ++)
	{
		for (loop_counter = 4; loop_counter >= 0; loop_counter--)
		{
			set_led((LED_T)loop_counter);
			nrf_delay_ms(50);
		}
			
		nrf_delay_ms(50);
		
		// Turn off all LEDS
		pavlok_led_clear_all();
	}
}

void pavlok_led_reward_pattern(void)
{
	int8_t	loop_counter = 0;

	pavlok_led_lightning_down(1);
	nrf_delay_ms(100);
	pavlok_led_lightning_up(1);
	nrf_delay_ms(100);
	
	pavlok_led_clear_all();
	
	for (loop_counter = 0; loop_counter < 3; loop_counter++) 
	{
		set_led(LED_GREEN);
		nrf_delay_ms(200);
		clear_led(LED_GREEN);
		nrf_delay_ms(200);
	}
	
}

void pavlok_led_warning_pattern(void)
{
	int8_t	loop_counter 	= 0;

	pavlok_led_clear_all();

	for (loop_counter = 0; loop_counter < 2; loop_counter++) 
	{
		set_led(LED1);
		set_led(LED2);
		set_led(LED3);
		set_led(LED4);
		set_led(LED5);
		nrf_delay_ms(200);
		pavlok_led_clear_all();
		nrf_delay_ms(200);
	}
		
	for (loop_counter = 0; loop_counter < 3; loop_counter++) 
	{
		set_led(LED_RED);
		nrf_delay_ms(200);
		clear_led(LED_RED);
		nrf_delay_ms(200);
	}
}

void pavlok_led_flash_all(uint8_t count)
{
	int8_t	loop_counter 	= 0;

	for (loop_counter = 0; loop_counter < count; loop_counter++) 
	{
		pavlok_led_clear_all();
		nrf_delay_ms(200);
		set_led(LED1);
		set_led(LED2);
		set_led(LED3);
		set_led(LED4);
		set_led(LED5);
		set_led(LED6);
		set_led(LED7);
		nrf_delay_ms(200);
	}
		
	pavlok_led_clear_all();
}

void pavlok_led_error(void)
{
	int8_t	loop_counter 	= 0;

	for (loop_counter = 0; loop_counter < 5; loop_counter++) 
	{
		pavlok_led_clear_all();
		nrf_delay_ms(200);
		set_led(LED_RED);
		nrf_delay_ms(200);
	}
		
	pavlok_led_clear_all();
}

void pavlok_set_stimulus_on(sNode_t * stimulus)
{
  uint8_t * p_config    = NULL;
  uint8_t   length      = stimulus->entry[PAVLOK_TLV_TAG_SIZE];
  p_config              = (uint8_t *)(&stimulus->entry[PAVLOK_TLV_TAG_LENGTH_SIZE] + length);
  
	if ('P' == stimulus->entry[0])
	{
    /** ------------------------------------------------------------------
    **  Stimulus chan change thru the configuration service after an
    **  application has started
    **  So we need to resetup the configuration structure for the 
    **  next stimulus pattern
    **  The actual configuration is at the END of the app data part
    **  So we index to the end and count back the sizeof the config
    **  ------------------------------------------------------------------
    */
    p_config -= PLOK_VALUE_LENGTH_PIEZO;
    service_info.cfg.piezo_value[0] = (p_config[PLOK_INDEX_CONTROL_BYTE_PIEZO] & PLOK_STIMULUS_COUNT_MASK);
    service_info.cfg.piezo_value[1] = p_config[PLOK_INDEX_FREQ_BYTE_PIEZO];
    service_info.cfg.piezo_value[2] = p_config[PLOK_INDEX_DC_BYTE_PIEZO];
    service_info.cfg.piezo_value[3] = p_config[PLOK_INDEX_ONTIME_BYTE_PIEZO];
    service_info.cfg.piezo_value[4] = p_config[PLOK_INDEX_OFFTIME_BYTE_PIEZO];
    service_task_perform_piezo_action(service_info.cfg.piezo_value);
	}
	else if ('M' == stimulus->entry[0])
	{
    /** ------------------------------------------------------------------
    **  Stimulus chan change thru the configuration service after an
    **  application has started
    **  So we need to resetup the configuration structure for the 
    **  next stimulus pattern
    **  ------------------------------------------------------------------
    */
    p_config -= PLOK_VALUE_LENGTH_VB_MOTOR;
    service_info.cfg.motor_value[0] = (p_config[PLOK_INDEX_CONTROL_BYTE_MOTOR] & PLOK_STIMULUS_COUNT_MASK);
    service_info.cfg.motor_value[1] = p_config[PLOK_INDEX_FREQ_BYTE_MOTOR];
    service_info.cfg.motor_value[2] = p_config[PLOK_INDEX_DC_BYTE_MOTOR];
    service_info.cfg.motor_value[3] = p_config[PLOK_INDEX_ONTIME_BYTE_MOTOR];
    service_info.cfg.motor_value[4] = p_config[PLOK_INDEX_OFFTIME_BYTE_MOTOR];
    service_task_perform_motor_action(service_info.cfg.motor_value);
	}
	else if ('Z' == stimulus->entry[0])
	{
    /** ------------------------------------------------------------------
    **  Stimulus chan change thru the configuration service after an
    **  application has started
    **  So we need to resetup the configuration structure for the 
    **  next stimulus pattern
    **  ------------------------------------------------------------------
    */
    p_config -= PLOK_VALUE_LENGTH_ZAP;
    service_info.cfg.zap_me_value[0] = (p_config[PLOK_ZAP_CYCLE_COUNT_MASK] & PLOK_STIMULUS_COUNT_MASK);
    service_info.cfg.zap_me_value[1] = p_config[PLOK_INDEX_DC_BYTE_ZAP];
    service_task_perform_zap_action(service_info.cfg.zap_me_value);
	}
	else if ('L' == stimulus->entry[0])
	{
    /** ------------------------------------------------------------------
    **  Stimulus chan change thru the configuration service after an
    **  application has started
    **  So we need to resetup the configuration structure for the 
    **  next stimulus pattern
    **  ------------------------------------------------------------------
    */
    p_config -= PLOK_VALUE_LENGTH_LED;
    service_info.cfg.led_value[0] = (p_config[PLOK_INDEX_CONTROL_BYTE_MOTOR] & PLOK_STIMULUS_COUNT_MASK);
    service_info.cfg.led_value[1] = p_config[PLOK_INDEX_COLOR_BYTE_LED];
    service_info.cfg.led_value[2] = p_config[PLOK_INDEX_CODE_LED];
    service_info.cfg.led_value[3] = p_config[PLOK_INDEX_ONTIME_BYTE_LED];
    service_info.cfg.led_value[4] = p_config[PLOK_INDEX_OFFTIME_BYTE_LED];
    service_task_perform_led_action(service_info.cfg.led_value);
	}
	else 
	{
		// TODO log error
	}
}

void pavlok_training_notifiy_user(void)
{
	service_info.cfg.piezo_value[0] = (PLOK_PIEZO_PERFORM_BIT | PLOK_PIEZO_COUNT_VALUE(3));
	service_info.cfg.piezo_value[1] = (2000 / PLOK_PWM_FREQ_STEP_VALUE);
	service_info.cfg.piezo_value[2] = 90;
	service_info.cfg.piezo_value[3] = PLOK_250_MS;
	service_info.cfg.piezo_value[4] = PLOK_100_MS;
	service_task_perform_piezo_action(service_info.cfg.piezo_value);
	
	service_info.cfg.motor_value[0] = (PLOK_MOTOR_PERFORM_BIT | PLOK_MOTOR_COUNT_VALUE(3));
	service_info.cfg.motor_value[1] = (3500 / PLOK_PWM_FREQ_STEP_VALUE);
	service_info.cfg.motor_value[2] = 50;
	service_info.cfg.motor_value[3] = PLOK_100_MS;
	service_info.cfg.motor_value[4] = PLOK_100_MS;
	service_task_perform_piezo_action(service_info.cfg.motor_value);
	
	return;
}

void pavlok_training_start_stimulus(void)
{
	service_info.cfg.piezo_value[0] = (PLOK_PIEZO_PERFORM_BIT | PLOK_PIEZO_COUNT_VALUE(1));
	service_info.cfg.piezo_value[1] = (4000 / PLOK_PWM_FREQ_STEP_VALUE);
	service_info.cfg.piezo_value[2] = 90;
	service_info.cfg.piezo_value[3] = PLOK_250_MS;
	service_info.cfg.piezo_value[4] = PLOK_250_MS;
	service_task_perform_piezo_action(service_info.cfg.piezo_value);

	return;
}

void pavlok_training_stop_stimulus(void)
{
	service_info.cfg.piezo_value[0] = (PLOK_PIEZO_PERFORM_BIT | PLOK_PIEZO_COUNT_VALUE(1));
	service_info.cfg.piezo_value[1] = (4000 / PLOK_PWM_FREQ_STEP_VALUE);
	service_info.cfg.piezo_value[2] = 90;
	service_info.cfg.piezo_value[3] = PLOK_250_MS;
	service_info.cfg.piezo_value[4] = PLOK_250_MS;
	service_task_perform_piezo_action(service_info.cfg.piezo_value);
	
	service_info.cfg.motor_value[0] = (PLOK_MOTOR_PERFORM_BIT | PLOK_MOTOR_COUNT_VALUE(3));
	service_info.cfg.motor_value[1] = (1000 / PLOK_PWM_FREQ_STEP_VALUE);
	service_info.cfg.motor_value[2] = 100;
	service_info.cfg.motor_value[3] = PLOK_250_MS;
	service_info.cfg.motor_value[4] = PLOK_80_MS;
	service_task_perform_motor_action(service_info.cfg.motor_value);
	
	return;
}

void pavlok_zap_notification(void)
{
  sPavlokServiceInfo_t	*	service_info = pavlok_get_configuration();
  pavlok_led_bit_pattern((LED_T)PLOK_ZAP_LED_NOTIFY, PLOK_ZAP_NOTIFY_COUNT);
}

void pavlok_set_training_started(eTrainingState_t state)
{
	service_info.training_active	= state;
}

eTrainingState_t pavlok_get_training_started(void)
{
	return service_info.training_active;
}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**	----------------------------------------------------------------------
**	@brief	List Section
**	----------------------------------------------------------------------
*/
void pavlok_list_insert(sNode_t * list, uint8_t * new_entry)
{
  if (NULL != new_entry)
  {
    /* Iterate through the list till we encounter the last node.*/
    while(list->next != NULL)
    {
      list = list->next;
    }

    /* Allocate memory for the new node and put data in it.*/
    list->next = (sNode_t *)nrf_malloc(sizeof(sNode_t));
    list = list->next;
    list->entry = new_entry;
    list->next = NULL;
   }
}

void pavlok_list_append(sNode_t * list, uint8_t * new_entry)
{
  if (NULL != list)
  {
      /* Iterate through the list till we encounter the last node.*/
      while(list->next != NULL)
      {
        list = list->next;
      }
        
      /* Allocate memory for the new node and put data in it.*/
      list->next  = (sNode_t *)nrf_malloc(sizeof(sNode_t));
      if (NULL != list->next)
      {
        list        = list->next;
        list->entry = new_entry;
        list->next  = NULL;
      }
   }
}

sNode_t * pavlok_list_next(sNode_t * list, sNode_t * current_entry)
{
  sNode_t * next_entry = NULL;
  
  if ((NULL != list)
      && (NULL != list->next))
  {
    // if current_entry is NULL then pull the firs list->next off the list
    // else go until you find the current_entry and return the current_entry->next
    if (NULL != current_entry)
    {
      while ((NULL != list->next)
             && (NULL == next_entry))
      {
        if (list->next == current_entry)
        {
          next_entry = list->next->next;
        }          
        list = list->next;
      }
    }
    else
    {
        next_entry = list->next;
    }
  }
  return next_entry;
}

int pavlok_list_find(sNode_t * pointer, uint8_t * new_entry)
{
        pointer =  pointer -> next; //First node is dummy node.
        /* Iterate through the entire linked list and search for the key. */
        while(pointer!=NULL)
        {
                if(pointer->entry == new_entry) //key is found.
                {
                        return 1;
                }
                pointer = pointer -> next;//Search in the next node.
        }
        /*Key is not found */
        return 0;
}

// This will alway delete the whole list
void pavlok_list_delete(sNode_t * list)
{
  sNode_t * delete_node = NULL;
  
  /* Go to the node for which the node next to it has to be deleted */
  while(NULL != list->next)
  {
    delete_node = list->next;
    list->next = delete_node->next;
       
    if (NULL != delete_node)
    {
      nrf_free(delete_node);
    }
  } // eow
       
  return;
}


sNode_t * pavlok_list_pop(sNode_t * list)
{
  sNode_t * pop_node = list->next;
  
  if (NULL != list->next)
  {
    list->next = list->next->next;
  }   
  return pop_node;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void pavlok_set_default_configuration(void)
{
  (void)memset((uint8_t *)&service_info, 0, sizeof(service_info));
  
	service_info.training_active	= TRAINING_STATE_INACTIVE;
  service_info.cfg.zap_me_value[PLOK_INDEX_DC_BYTE_ZAP] = PLOK_DEFAULT_ZAP_DC;
}

void pavlok_vusb_init(void) 
{
	nrf_gpio_cfg_input(PAVLOK_V_USB_PIN, NRF_GPIO_PIN_NOPULL);
}

bool pavlok_get_usb_pin_state(void)
{
  bool rtn_code = false;
  
  rtn_code = nrf_gpio_pin_read(PAVLOK_V_USB_PIN);
  
  return rtn_code;
}



int8_t pavlok_flash_area_start(eFlashAreaName_t area)
{
  uint8_t     start_counter   = 0;

  if (eFlashApp == area)
  {
    start_counter = FLASH_SECTOR_APPLICATION_START;
  }
  else if (eFlashHabit == area)
  {
    start_counter = FLASH_SECTOR_HABIT_START;
  }
  else if (eFlashLog == area)
  {
    start_counter = FLASH_SECTOR_LOG_START;
  }
  else if (eFlashSleep == area)
  {
    start_counter = FLASH_SECTOR_SLEEP_START;
  }
  else
  {
    start_counter   = UNUSED_SECTOR;
  }
  
  return start_counter;
}

int8_t pavlok_flash_area_stop(eFlashAreaName_t area)
{
  uint8_t     start_counter   = 0;

  if (eFlashApp == area)
  {
    start_counter = FLASH_SECTOR_APPLICATION_END;
  }
  else if (eFlashHabit == area)
  {
    start_counter = FLASH_SECTOR_HABIT_END;
  }
  else if (eFlashLog == area)
  {
    start_counter = FLASH_SECTOR_LOG_END;
  }
  else if (eFlashSleep == area)
  {
    start_counter = FLASH_SECTOR_SLEEP_END;
  }
  else
  {
    start_counter   = UNUSED_SECTOR;
  }
  
  return start_counter;
}


/*------------------------------------------------------------------------
**
**	@fn		Function			find_name_in_sectors
**
**	@brief	Description	called from a ble event write
**
**	@param [in]					ble_evt_t * p_ble_evt 
**
**	@param	[out]				None
**
**	@return							None
**
**	@retval
**
**	@warn
**
**------------------------------------------------------------------------
*/
static uint8_t find_name_in_sectors(char * name, eFlashAreaName_t area)
{
  bool        found           = false;
  uint8_t     sector_name[PAVLOK_SECTOR_NAME_SIZE] = {0};
  ret_code_t  rtn_code        = 0xFFFFFFFF;
  int16_t   * name_length     = NULL;
  int32_t     start_counter   = 0;
  int32_t     end_counter     = 0;
  int32_t     sector_address  = 0;

  if (eFlashApp == area)
  {
    start_counter = FLASH_SECTOR_APPLICATION_START;
    end_counter   = FLASH_SECTOR_APPLICATION_END;
  }
  else if (eFlashHabit == area)
  {
    start_counter = FLASH_SECTOR_HABIT_START;
    end_counter   = FLASH_SECTOR_HABIT_END;
  }
  else if (eFlashLog == area)
  {
    start_counter = FLASH_SECTOR_LOG_START;
    end_counter   = FLASH_SECTOR_LOG_END;
  }
  else if (eFlashSleep == area)
  {
    start_counter = FLASH_SECTOR_SLEEP_START;
    end_counter   = FLASH_SECTOR_SLEEP_END;
  }
  else
  {
    start_counter   = UNUSED_SECTOR;
    end_counter     = UNUSED_SECTOR;
  }
 
  if ((start_counter != UNUSED_SECTOR)
      && (end_counter != UNUSED_SECTOR))
  {
    for (int32_t  counter = start_counter; (counter < end_counter) && (false == found); counter++)
    {
      (void)memset(sector_name, 0, sizeof(sector_name));
      
      sector_address = (counter * 32);
      // contract SERIAL_FLASH_TXRX_RET_T serial_flash_read_data(uint8_t *, uint32_t, uint32_t);
      rtn_code = serial_flash_read_data(sector_name, sector_address, PAVLOK_SECTOR_NAME_SIZE);
      APP_ERROR_CHECK(rtn_code);
      
      // now index to the length field then find the end of the name
      name_length = (int16_t *)&sector_name[(PAVLOK_TLV_AP_NAME_INDEX - PAVLOK_TLV_VALUE_START)];
      rtn_code = memcmp(&sector_name[PAVLOK_SECTOR_NAME_SIZE], name, *name_length);
      if (0 == rtn_code)
      {
        found = true;
        break;
      }
    }
    
    if (false == found)
    {
      sector_address = UNUSED_SECTOR;
    }
  }
  return sector_address;
}


/** @} */
