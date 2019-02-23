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
**  @note Attention!  TODO verify all time ranges
**   
**
**------------------------------------------------------------------------
*/
/*------------------------------------------------------------------------
**	@brief System Include(s)
**------------------------------------------------------------------------
*/

#include <string.h>
#include <math.h>
#include <nrf_assert.h>
#include "SEGGER_RTT.h"

#include "nordic_common.h"
#include "ble_l2cap.h"
#include "ble_srv_common.h"
#include "app_util.h"



/*------------------------------------------------------------------------
**	@brief Project Include(s)
**------------------------------------------------------------------------
*/
#include "pavlok_common.h"
#include "configuration.h"
#include "service_task.h"
#include "application_task.h"
#include "logger.h"
#include "pwm.h"

/*------------------------------------------------------------------------
**	@brief	Global Data
**------------------------------------------------------------------------
*/
#define PLOK_VB_MOTOR_DESC			"Vibration Motor"
#define PLOK_PIEZO_DESC					"Piezo Transducer"
#define PLOK_ZAP_DESC						"Zap"
#define PLOK_LED_DESC						"LED"
#define PLOK_HD_DESC						"Human Detect"
#define PLOK_TIME_DESC					"Set Time"
#define PLOK_ALM_CTL_DESC			  "Alarm Control"
#define PLOK_DR_DESC					  "Device Reset"

#define PLOK_VALUE_DTAP_ZAP					(0x02)
#define PLOK_VALUE_DTAP_URGE				(0x42)
#define PLOK_VALUE_HD_LEFT					(0x11)
#define PLOK_VALUE_HD_RIGHT					(0x01)


/*------------------------------------------------------------------------
**	@brief	Global Extern(s)
**------------------------------------------------------------------------
*/


/*------------------------------------------------------------------------
**	@brief	Static Data
**------------------------------------------------------------------------
*/

static uint8_t * plck_vb_motor_descriptor 	= (uint8_t *)PLOK_VB_MOTOR_DESC;
static uint8_t * plck_piezo_descriptor 			= (uint8_t *)PLOK_PIEZO_DESC;
static uint8_t * plck_zap_descriptor 				= (uint8_t *)PLOK_ZAP_DESC;
static uint8_t * plck_led_descriptor 				= (uint8_t *)PLOK_LED_DESC;
static uint8_t * plck_hd_descriptor 				= (uint8_t *)PLOK_HD_DESC;
static uint8_t * plck_time_descriptor 		  = (uint8_t *)PLOK_TIME_DESC;
static uint8_t * plck_alm_ctl_descriptor 		= (uint8_t *)PLOK_ALM_CTL_DESC;
static uint8_t * plck_dr_descriptor 				= (uint8_t *)PLOK_DR_DESC;

static cfg_t						m_cfg;
static cfg_init_t				m_cfg_init;
static cfg_t * p_cfg = &m_cfg;
static sCfg_service_t	*	p_cfg_info;
#undef APP_TIMERS
//#define APP_TIMERS (1)
#ifdef APP_TIMERS
#define CFG_SEND_INTERVAL             pdMS_TO_TICKS(50)                                       /**< Heart rate measurement interval (ms). */

static TimerHandle_t                m_cfg_timer;     /**< Definition of heart rate timer. */
static void cfg_timers_start(void);
static void cfg_timers_stop(void);
#endif

static uint32_t cfg_send(eCFG_CHAR_LIST char_type);

/*------------------------------------------------------------------------
**	@brief	Static Forward References
**------------------------------------------------------------------------
*/
#ifdef REMOVED // not used
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

uint16_t cfg_service_get_connection_handle(void)
{
  return p_cfg->conn_handle;
}
#endif

#ifdef REMOVED // not used
uint16_t cfg_service_get_handle(void)
{
  return p_cfg->service_handle;
}
#endif

#ifdef REMOVED
uint16_t cfg_service_get_cccd_handle(void)
{
  return p_cfg->vb_motor_handle.cccd_handle;
}
#endif

#ifdef APP_TIMERS
/**@brief Function for handling the Heart rate measurement timer time-out.
 *
 * @details This function will be called each time the heart rate measurement timer excfges.
 *          It will exclude RR Interval data from every third measurement.
 *
 * @param[in] xTimer Handler to the timer that called this function.
 *                   You may get identifier given to the function xTimerCreate using pvTimerGetTimerID.
 */
static void cfg_timeout_handle(TimerHandle_t xTimer)
{
  UNUSED_PARAMETER(xTimer);

  if (cfg_send_index <= CFG_CHAR_LAST)
  {
	  (void)cfg_send(cfg_send_index);
	  cfg_send_index += 1;
  }
  else
  {
	  // stop the timer
	  cfg_timers_stop();
	  cfg_send_index = CFG_CHAR_LAST;
  }
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
void cfg_timers_init(void)
{
	pavlok_timers_init("CFG", &m_cfg_timer, CFG_SEND_INTERVAL, cfg_timeout_handle);
}


/**@brief Function for starting application timers.
 */
void cfg_timers_start(void)
{
  if (pdFALSE == xTimerIsTimerActive(m_cfg_timer))
  {
    // Start application timers.
    if(pdPASS != xTimerStart(m_cfg_timer, OSTIMER_WAIT_FOR_QUEUE))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
  }
}

void cfg_timers_stop(void)
{
    if(pdPASS != xTimerStop(m_cfg_timer , OSTIMER_WAIT_FOR_QUEUE))
    {
      APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
    }
}
#endif


static void update_cccd_enabled(ble_evt_t * p_ble_evt)
{
	ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

	if (p_evt_write->handle == p_cfg->zap_me_handle.cccd_handle)
	{
		DEBUGI_APP_400("CFG_CHAR_ZAP", p_evt_write->data[0]);
		p_cfg->char_enable[CFG_CHAR_ZAP] = p_evt_write->data[0];
	}
	else if (p_evt_write->handle == p_cfg->device_reset_handle.cccd_handle)
	{
		DEBUGI_APP_400("CFG_CHAR_DR", p_evt_write->data[0]);
		p_cfg->char_enable[CFG_CHAR_DR] = p_evt_write->data[0];
	}
}

static uint32_t cfg_send(eCFG_CHAR_LIST char_type)
{
    uint32_t err_code	=	NRF_ERROR_INVALID_PARAM;

    if (p_cfg->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        ble_gatts_hvx_params_t hvx_params;
        uint8_t				*  hvx_data = NULL;
        uint16_t               hvx_len = 0;

        /** ---------------------------------------------------------------
         **	Send data
         **	all data bytes are sent even if set to zero
         **	---------------------------------------------------------------
         */
        memset(&hvx_params, 0, sizeof(hvx_params));

        if (CFG_CHAR_MOTOR == char_type)
        {
        	hvx_data = (uint8_t *)&p_cfg_info->motor_value;
            hvx_params.handle = p_cfg->vb_motor_handle.value_handle;
            hvx_len = PLOK_VALUE_LENGTH_VB_MOTOR;
					DEBUGS_APP_400("VB");
        }
        else if (CFG_CHAR_PIEZO == char_type)
        {
        	hvx_data = (uint8_t *)&p_cfg_info->piezo_value;
            hvx_params.handle = p_cfg->piezo_handle.value_handle;
            hvx_len = PLOK_VALUE_LENGTH_PIEZO;
        }
        else if (CFG_CHAR_ZAP == char_type)
        {
        	hvx_data = (uint8_t *)p_cfg_info->zap_me_value;
          hvx_params.handle = p_cfg->zap_me_handle.value_handle;
          hvx_len = PLOK_VALUE_LENGTH_ZAP;
        }
        else if (CFG_CHAR_LED == char_type)
        {
        	hvx_data = (uint8_t *)&p_cfg_info->led_value;
            hvx_params.handle = p_cfg->led_handle.value_handle;
            hvx_len = PLOK_VALUE_LENGTH_LED;
        }
        else if (CFG_CHAR_HD == char_type)
        {
        	hvx_data = (uint8_t *)&p_cfg_info->hand_detect_value;
            hvx_params.handle = p_cfg->hand_detect_handle.value_handle;
            hvx_len = PLOK_VALUE_LENGTH_HD;
        }
        else if (CFG_CHAR_TIME == char_type)
        {
        	hvx_data = (uint8_t *)&p_cfg_info->pavlok_time;
            hvx_params.handle = p_cfg->time_handle.value_handle;
            hvx_len = PLOK_VALUE_LENGTH_TIME;
        }
        else
        {
        	APP_ERROR_HANDLER(CFG_CHAR_LAST);
        }

        if (NULL != hvx_data)
        {
        	uint8_t * encoded_info = pavlok_common_get_encode_buffer();  // get common buffer
        	pavlok_encode((uint8_t *)&hvx_data, encoded_info, hvx_len);
          hvx_params.p_data = encoded_info;
        }

        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;

        err_code = sd_ble_gatts_hvx(p_cfg->conn_handle, &hvx_params);
        if (err_code == NRF_SUCCESS)
        {
            err_code = NRF_ERROR_DATA_SIZE;
						DEBUGI_APP_400("Send", err_code);
        }
				else
				{
					PAVLOK_LOG_ENTRY(EVT_SFW, 0, __LINE__, __FILE__);
				}
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
				DEBUGI_APP_400("Not Connected", err_code);
    }

    return err_code;
}

static bool verify_time_configuration(ble_gatts_evt_write_t * p_evt_write)
{
  /*  ------------------------------------------------------------------
  **  RTC data to verify  -- All is in BCD format
  **  ------------------------------------------------------------------
  */
  int8_t	upper_nibble = 0;
  int8_t	lower_nibble = 0;
  bool		data_is_valid = false;
  
  upper_nibble = UPPER_NIBBLE(p_evt_write->data[0]);
  lower_nibble = LOWER_NIBBLE(p_evt_write->data[0]);
  if ((TIME_MAX_UPPER_SECONDS >= upper_nibble)
      && (0 <= upper_nibble)
      && (TIME_MAX_LOWER_SECONDS >= lower_nibble)
      && (0 <= lower_nibble))
  {			
    data_is_valid = true;
  }
  else
  {			
    data_is_valid = false;
  }
  
  upper_nibble = UPPER_NIBBLE(p_evt_write->data[1]);
  lower_nibble = LOWER_NIBBLE(p_evt_write->data[1]);
  if ((true == data_is_valid)
      && (TIME_MAX_UPPER_MINUTES >= upper_nibble)
      && (0 <= upper_nibble)
      && (TIME_MAX_LOWER_MINUTES >= lower_nibble)
      && (0 <= lower_nibble))
  {			
    data_is_valid = true;
  }
  else
  {			
    data_is_valid = false;
  }

  upper_nibble = UPPER_NIBBLE(p_evt_write->data[2]);
  lower_nibble = LOWER_NIBBLE(p_evt_write->data[2]);
  if ((true == data_is_valid)
      && (TIME_MAX_UPPER_HOURS_24 >= upper_nibble)
      && (TIME_MIN_UPPER_HOURS_24 <= upper_nibble)
      && (TIME_MAX_LOWER_HOURS_24 >= lower_nibble)
      && (TIME_MIN_LOWER_HOURS_24 <= lower_nibble))
  {			
    data_is_valid = true;
  }
  else
  {			
    data_is_valid = false;
  }
  
  upper_nibble = UPPER_NIBBLE(p_evt_write->data[3]);
  lower_nibble = LOWER_NIBBLE(p_evt_write->data[3]);
  if ((true == data_is_valid)
      && (TIME_MAX_UPPER_DAYS >= upper_nibble)
      && (TIME_MIN_UPPER_DAYS <= upper_nibble)
      && (TIME_MAX_LOWER_DAYS >= lower_nibble)
      && (TIME_MIN_LOWER_DAYS <= lower_nibble))
  {			
    if ((0 == upper_nibble)
        && (1 == lower_nibble))
    {
      data_is_valid = false;
    }
    else
    {
      data_is_valid = true;
    }
  }
  else
  {			
    data_is_valid = false;
  }

  upper_nibble = UPPER_NIBBLE(p_evt_write->data[4]);
  lower_nibble = LOWER_NIBBLE(p_evt_write->data[4]);
  if ((true == data_is_valid)
      && (TIME_MAX_UPPER_WDAYS >= upper_nibble)
      && (TIME_MIN_UPPER_WDAYS <= upper_nibble)
      && (TIME_MAX_LOWER_WDAYS >= lower_nibble)
      && (TIME_MIN_LOWER_WDAYS <= lower_nibble))
  {			
    data_is_valid = true;
  }
  else
  {			
    data_is_valid = false;
  }
  
  upper_nibble = UPPER_NIBBLE(p_evt_write->data[5]);
  lower_nibble = LOWER_NIBBLE(p_evt_write->data[5]);
  if ((true == data_is_valid)
      && (TIME_MAX_UPPER_MONTH >= upper_nibble)
      && (TIME_MIN_UPPER_MONTH <= upper_nibble)
      && (TIME_MAX_LOWER_MONTH >= lower_nibble)
      && (TIME_MIN_LOWER_MONTH <= lower_nibble))
  {			
    data_is_valid = true;
  }
  else
  {			
    data_is_valid = false;
  }
  
  upper_nibble = UPPER_NIBBLE(p_evt_write->data[6]);
  lower_nibble = LOWER_NIBBLE(p_evt_write->data[6]);
  if ((true == data_is_valid)
      && (TIME_MAX_UPPER_YEARS >= upper_nibble)
      && (TIME_MIN_UPPER_YEARS <= upper_nibble)
      && (TIME_MAX_LOWER_YEARS >= lower_nibble)
      && (TIME_MIN_LOWER_YEARS <= lower_nibble))
  {			
    data_is_valid = true;
  }
  else
  {			
    data_is_valid = false;
  }
  return data_is_valid;
}

/**@brief Function for handling the Write event.
 *
 * @param[in]   p_pavlok       Heart Rate Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_evt_t * p_ble_evt)
{
  ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

//  DEBUGI_APP_400("sz", p_evt_write->len);

  if (CCCD_MSG_SIZE == p_evt_write->len)
  {
    // update cccd
    update_cccd_enabled(p_ble_evt);
  }

		if ((p_evt_write->handle == p_cfg->vb_motor_handle.value_handle)
				&& (ACTION_STATE_LAST == (service_action_state_get(CFG_CHAR_MOTOR))))
		{
			/*  ------------------------------------------------------------------
			**  First verify all data
			**  then if store save off all values store off bytes 2,3,4 but only 
			**  if different than last time
			**  Then perform if needed
			**  ------------------------------------------------------------------
			*/
			if ((p_evt_write->data[PLOK_INDEX_OFFTIME_BYTE_MOTOR] >= PLOK_20_MS)
					&& (p_evt_write->data[PLOK_INDEX_OFFTIME_BYTE_MOTOR] <= PLOK_250_MS)
					&& (p_evt_write->data[PLOK_INDEX_ONTIME_BYTE_MOTOR] >= PLOK_20_MS)
					&& (p_evt_write->data[PLOK_INDEX_ONTIME_BYTE_MOTOR] <= PLOK_250_MS)
					&& (p_evt_write->data[PLOK_INDEX_DC_BYTE_MOTOR] <= MOTOR_PWM_MAX)
					&& (p_evt_write->data[PLOK_INDEX_DC_BYTE_MOTOR] >= MOTOR_PWM_MIN)
					&& ((p_evt_write->data[PLOK_INDEX_FREQ_BYTE_MOTOR] * PLOK_PWM_FREQ_STEP_VALUE) <= MOTOR_FREQUENCY_MAX)
					&& ((p_evt_write->data[PLOK_INDEX_FREQ_BYTE_MOTOR] * PLOK_PWM_FREQ_STEP_VALUE) >= MOTOR_FREQUENCY_MIN))
			{
				p_cfg_info->characteristic      = CFG_CHAR_MOTOR;
				p_cfg_info->motor_value[0]   = p_evt_write->data[0];
				p_cfg_info->motor_value[1]   = p_evt_write->data[1];
				p_cfg_info->motor_value[2]   = p_evt_write->data[2];
				p_cfg_info->motor_value[3]   = p_evt_write->data[3];
				p_cfg_info->motor_value[4]   = p_evt_write->data[4];
				DEBUGI_APP_400("MOTOR", p_cfg_info->characteristic);
				pavlok_set_dirty_bit(SI_CFG, &p_cfg_info->motor_value);
			}
			
			cfg_send(CFG_CHAR_MOTOR);
		}
		else if ((p_evt_write->handle == p_cfg->piezo_handle.value_handle)
							&& (ACTION_STATE_LAST == (service_action_state_get(CFG_CHAR_PIEZO))))
 		{
			/*  ------------------------------------------------------------------
			**  First verify all data
			**  then if store save off all values store off bytes 2,3,4 but only 
			**  if different than last time
			**  Then perform if needed
			**  ------------------------------------------------------------------
			*/
			if ((p_evt_write->data[PLOK_INDEX_OFFTIME_BYTE_PIEZO] >= PLOK_10_MS)
					&& (p_evt_write->data[PLOK_INDEX_OFFTIME_BYTE_PIEZO] <= PLOK_250_MS)
					&& (p_evt_write->data[PLOK_INDEX_ONTIME_BYTE_PIEZO] >= PLOK_10_MS)
					&& (p_evt_write->data[PLOK_INDEX_ONTIME_BYTE_PIEZO] <= PLOK_250_MS)
					&& (p_evt_write->data[PLOK_INDEX_DC_BYTE_MOTOR] <= PIEZO_PWM_MAX)
					&& (p_evt_write->data[PLOK_INDEX_DC_BYTE_MOTOR] >= PIEZO_PWM_MIN)
					&& ((p_evt_write->data[PLOK_INDEX_FREQ_BYTE_PIEZO] * PLOK_PWM_FREQ_STEP_VALUE) <= PIEZO_FREQUENCY_MAX)
					&& ((p_evt_write->data[PLOK_INDEX_FREQ_BYTE_PIEZO] * PLOK_PWM_FREQ_STEP_VALUE) >= PIEZO_FREQUENCY_MIN))
			{
				p_cfg_info->characteristic  = CFG_CHAR_PIEZO;
				p_cfg_info->piezo_value[PLOK_INDEX_CONTROL_BYTE_PIEZO] = p_evt_write->data[0];
				p_cfg_info->piezo_value[PLOK_INDEX_FREQ_BYTE_PIEZO] = p_evt_write->data[1];
				p_cfg_info->piezo_value[PLOK_INDEX_DC_BYTE_PIEZO] = p_evt_write->data[2];
				p_cfg_info->piezo_value[PLOK_INDEX_ONTIME_BYTE_PIEZO] = p_evt_write->data[3];
				p_cfg_info->piezo_value[PLOK_INDEX_OFFTIME_BYTE_PIEZO]   = p_evt_write->data[4];
				DEBUGI_APP_400("CFG_CHAR_PIEZO", p_cfg_info->characteristic);
				pavlok_set_dirty_bit(SI_CFG, &p_cfg_info->piezo_value);
			}

			cfg_send(CFG_CHAR_PIEZO);
		} 
		else if ((p_evt_write->handle == p_cfg->zap_me_handle.value_handle)
							&& (ACTION_STATE_LAST == (service_action_state_get(CFG_CHAR_ZAP))))
//							&& (BLE_SERVICE_ENABLED == p_cfg->char_enable[CFG_CHAR_ZAP]))
		{
			/*  ------------------------------------------------------------------
			**  First verify all data
			**  then if store save off all values store off bytes 2,3,4 but only 
			**  if different than last time
			**  Then perform if needed
			**  ------------------------------------------------------------------
			*/
			p_cfg_info->characteristic  = CFG_CHAR_ZAP;
			if ((p_evt_write->data[PLOK_INDEX_DC_BYTE_ZAP] <= ZAP_PWM_MAX)
					&& (p_evt_write->data[PLOK_INDEX_DC_BYTE_ZAP] >= ZAP_PWM_MIN))
			{
        p_cfg_info->zap_me_value[0] = p_evt_write->data[0];
        p_cfg_info->zap_me_value[1] = p_evt_write->data[1];
        DEBUGI_APP_400("CFG_CHAR_ZAP", p_cfg_info->characteristic);
        pavlok_set_dirty_bit(SI_CFG, &p_cfg_info->zap_me_value);
      }
		} 
		else if (p_evt_write->handle == p_cfg->led_handle.value_handle)
		{
			/*  ------------------------------------------------------------------
			**  First verify all data
			**  then if store save off all values store off bytes 2,3,4 but only 
			**  if different than last time
			**  Then perform if needed
			**  ------------------------------------------------------------------
			*/
      p_cfg_info->characteristic  = CFG_CHAR_LED;
      p_cfg_info->led_value[PLOK_INDEX_CONTROL_BYTE_LED] = p_evt_write->data[0];
      p_cfg_info->led_value[PLOK_INDEX_COLOR_BYTE_LED] = p_evt_write->data[1];
      p_cfg_info->led_value[PLOK_INDEX_ONTIME_BYTE_LED] = p_evt_write->data[2];
      p_cfg_info->led_value[PLOK_INDEX_OFFTIME_BYTE_LED]   = p_evt_write->data[3];
      pavlok_set_dirty_bit(SI_CFG, &p_cfg_info->led_value);
		} 
		else if (p_evt_write->handle == p_cfg->time_handle.value_handle)
		{
			if (true == (verify_time_configuration(p_evt_write)))
			{
				// store real values all off
				p_cfg_info->characteristic      	= CFG_CHAR_TIME;
				p_cfg_info->pavlok_time.seconds 	= p_evt_write->data[0];
				p_cfg_info->pavlok_time.minutes 	= p_evt_write->data[1];
				p_cfg_info->pavlok_time.hours 		= p_evt_write->data[2];
				p_cfg_info->pavlok_time.days 			= p_evt_write->data[3];
				p_cfg_info->pavlok_time.weekdays 	= p_evt_write->data[4];
				p_cfg_info->pavlok_time.months 		= p_evt_write->data[5];
				p_cfg_info->pavlok_time.years 		= p_evt_write->data[6];

				pavlok_set_dirty_bit(SI_CFG, &p_cfg_info->led_value);
			}

			cfg_send(CFG_CHAR_TIME);
		}  
		else if (p_evt_write->handle == p_cfg->alarm_control_handle.value_handle)
		{
			if (0 == p_evt_write->data[0])
			{
        application_set_snooze_disable(p_evt_write->data[1]);
      }
      
			if (1 == p_evt_write->data[0])
			{
        application_set_alarm_disable(p_evt_write->data[1]);
      }
		}  
		else if(p_evt_write->handle == p_cfg->hand_detect_handle.value_handle)
		{
      p_cfg_info->hand_detect_value[0] = p_evt_write->data[0];
      p_cfg_info->hand_detect_value[1] = p_evt_write->data[1];

			p_cfg_info->characteristic      	= CFG_CHAR_HD;
			pavlok_set_dirty_bit(SI_CFG, &p_cfg_info->hand_detect_value);
		} 
		else if(p_evt_write->handle == p_cfg->device_reset_handle.value_handle)
		{
      if (1 == p_evt_write->data[0])
      {
        cfg_send(CFG_CHAR_DR);       
      }
		}
}

void cfg_on_ble_evt(ble_evt_t * p_ble_evt)
{
  	switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
        	p_cfg->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
           break;

        case BLE_GAP_EVT_DISCONNECTED:
        	p_cfg->conn_handle = BLE_CONN_HANDLE_INVALID;
#ifdef APP_TIMERS
      cfg_timers_stop();
#endif
            break;

        case BLE_GATTS_EVT_WRITE:
#ifdef APP_TIMERS
      cfg_timers_start();
#endif
          on_write(p_ble_evt);
       break;

        default:
            break;
    }
}

/**@brief Function for adding the Heart Rate Measurement characteristic.
 *
 * @param[in]   p_pavlok        Heart Rate Service structure.
 * @param[in]   p_pavlok_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t cfg_vb_motor_char_add(void)
{
	ble_uuid_t          	service_uuid;
  ble_gatts_char_md_t 	char_md;
  ble_gatts_attr_t    	attr_char_value;
  ble_gatts_attr_md_t 	attr_md;
  ble_gatts_char_pf_t   presentation;
  uint32_t							err_code = NRF_ERROR_INVALID_PARAM;

  memset(&char_md, 0, sizeof(char_md));
  memset(&presentation, 0, sizeof(presentation));
  presentation.format                 = BLE_GATT_CPF_FORMAT_UINT8; 
  presentation.unit                   = BLUETOOTH_ORG_CPF_UNITLESS; 

  char_md.p_char_pf                   = &presentation;
  char_md.char_props.auth_signed_wr   = 1;
  char_md.char_props.write   					= 1;
  char_md.char_props.read   					= 1;
  char_md.p_char_user_desc            = plck_vb_motor_descriptor;
  char_md.char_user_desc_size         = strlen((char *)plck_vb_motor_descriptor);
  char_md.char_user_desc_max_size     = strlen((char *)plck_vb_motor_descriptor);

  memset(&attr_md, 0, sizeof(attr_md));

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

  attr_md.vloc                        = BLE_GATTS_VLOC_STACK;
  attr_md.rd_auth                     = 0;  // no acks required
  attr_md.wr_auth                     = 0;  // no acks required
  attr_md.vlen                        = 0;  // must be a one ????

  memset(&attr_char_value, 0, sizeof(attr_char_value));

  BLE_UUID_BLE_ASSIGN(service_uuid, eCFG_CHAR_MOTOR_UUID);
  attr_char_value.p_uuid              = &service_uuid;
  attr_char_value.p_attr_md           = &attr_md;
  
  attr_char_value.init_len            = PLOK_VALUE_LENGTH_VB_MOTOR;
  attr_char_value.init_offs           = 0;
  attr_char_value.max_len             = PLOK_VALUE_LENGTH_VB_MOTOR;
  attr_char_value.p_value             = NULL;

  err_code = sd_ble_gatts_characteristic_add( p_cfg->service_handle,
                                              &char_md,
                                              &attr_char_value,
                                              &p_cfg->vb_motor_handle);

    return err_code;
}


/**@brief Function for adding the Heart Rate Measurement characteristic.
 *
 * @param[in]   p_pavlok        Heart Rate Service structure.
 * @param[in]   p_pavlok_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t cfg_piezo_char_add(void)
{
	ble_uuid_t          	service_uuid;
  ble_gatts_char_md_t 	char_md;
  ble_gatts_attr_t    	attr_char_value;
  ble_gatts_attr_md_t 	attr_md;
  ble_gatts_char_pf_t   presentation;
  ret_code_t			      err_code;

  memset(&char_md, 0, sizeof(char_md));
  memset(&presentation, 0, sizeof(presentation));
  presentation.format                 = BLE_GATT_CPF_FORMAT_UINT8; 
  presentation.unit                   = BLUETOOTH_ORG_CPF_UNITLESS; 

  char_md.p_char_pf                   = &presentation;
  char_md.char_props.auth_signed_wr   = 1;
  char_md.char_props.write   			= 1;
  char_md.char_props.read   			= 1;
  char_md.p_char_user_desc            = plck_piezo_descriptor;
  char_md.char_user_desc_size         = strlen((char *)plck_piezo_descriptor);
  char_md.char_user_desc_max_size     = strlen((char *)plck_piezo_descriptor);

  memset(&attr_md, 0, sizeof(attr_md));

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
 	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
 	attr_md.vloc                        = BLE_GATTS_VLOC_STACK;
 	attr_md.rd_auth                     = 0;  // no acks required
 	attr_md.wr_auth                     = 0;  // no acks required
 	attr_md.vlen                        = 1;  // must be a one ????

 	memset(&attr_char_value, 0, sizeof(attr_char_value));

	BLE_UUID_BLE_ASSIGN(service_uuid, eCFG_CHAR_PIEZO_UUID);
 	attr_char_value.p_uuid              = &service_uuid;
 	attr_char_value.p_attr_md           = &attr_md;
 	attr_char_value.init_len            = PLOK_VALUE_LENGTH_PIEZO;
 	attr_char_value.init_offs           = 0;
 	attr_char_value.max_len             = PLOK_VALUE_LENGTH_PIEZO;
 	attr_char_value.p_value             = NULL;

 	err_code = sd_ble_gatts_characteristic_add( p_cfg->service_handle,
                                              &char_md,
                                              &attr_char_value,
                                              &p_cfg->piezo_handle);

    return err_code;
}


static uint32_t cfg_zap_char_add(void)
{
	ble_uuid_t          	service_uuid;
  ble_gatts_char_md_t 	char_md;
  ble_gatts_attr_t    	attr_char_value;
  ble_gatts_attr_md_t 	attr_md;
  ble_gatts_char_pf_t   presentation;
  ret_code_t				    err_code;
  
#ifdef TBD
  // use the  CCD security only in the first characteristic add
  ble_gatts_attr_md_t cccd_md;

  memset(&cccd_md, 0, sizeof(cccd_md));
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
	cccd_md.vlen                        = 1;  // must be a one ????
  cccd_md.vloc                        = BLE_GATTS_VLOC_STACK;
#endif
  memset(&char_md, 0, sizeof(char_md));
  memset(&presentation, 0, sizeof(presentation));
  presentation.format                 = BLE_GATT_CPF_FORMAT_UINT8; 
  presentation.unit                   = BLUETOOTH_ORG_CPF_UNITLESS; 

  char_md.p_char_pf                   = &presentation;
  char_md.char_props.auth_signed_wr   = 1;
  char_md.char_props.write   			    = 1;
  char_md.char_props.read   			    = 1;
//    char_md.char_props.notify   		= 1;
  char_md.p_char_user_desc            = plck_zap_descriptor;
  char_md.char_user_desc_size         = strlen((char *)plck_zap_descriptor);
  char_md.char_user_desc_max_size     = strlen((char *)plck_zap_descriptor);

//    char_md.p_cccd_md                   = &cccd_md;
//    char_md.p_sccd_md                   = NULL;

  memset(&attr_md, 0, sizeof(attr_md));

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
  attr_md.vloc                        = BLE_GATTS_VLOC_STACK;
  attr_md.rd_auth                     = 0;  // no acks required
  attr_md.wr_auth                     = 0;  // no acks required
  attr_md.vlen                        = 1;  // must be a one ????

  memset(&attr_char_value, 0, sizeof(attr_char_value));

	BLE_UUID_BLE_ASSIGN(service_uuid, eCFG_CHAR_ZAP_UUID);
  attr_char_value.p_uuid              = &service_uuid;
  attr_char_value.p_attr_md           = &attr_md;
  attr_char_value.init_len            = PLOK_VALUE_LENGTH_ZAP;
  attr_char_value.init_offs           = 0;
  attr_char_value.max_len             = PLOK_VALUE_LENGTH_ZAP;
  attr_char_value.p_value             = NULL;

  err_code = sd_ble_gatts_characteristic_add( p_cfg->service_handle,
                                              &char_md,
                                              &attr_char_value,
                                              &p_cfg->zap_me_handle);

  return err_code;
}

static uint32_t cfg_led_char_add(void)
{
	ble_uuid_t          	service_uuid;
  ble_gatts_char_md_t 	char_md;
  ble_gatts_attr_t    	attr_char_value;
  ble_gatts_attr_md_t 	attr_md;
  ble_gatts_char_pf_t   presentation;
  ret_code_t				    err_code;

  memset(&char_md, 0, sizeof(char_md));
  memset(&presentation, 0, sizeof(presentation));
  presentation.format                 = BLE_GATT_CPF_FORMAT_UINT8; 
  presentation.unit                   = BLUETOOTH_ORG_CPF_UNITLESS; 

  char_md.p_char_pf                   = &presentation;
  char_md.char_props.auth_signed_wr   = 1;
  char_md.char_props.write   			    = 1;
  char_md.char_props.read   			    = 1;
  char_md.p_char_user_desc            = plck_led_descriptor;
  char_md.char_user_desc_size         = strlen((char *)plck_led_descriptor);
  char_md.char_user_desc_max_size     = strlen((char *)plck_led_descriptor);

  memset(&attr_md, 0, sizeof(attr_md));

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
  attr_md.vloc                        = BLE_GATTS_VLOC_STACK;
  attr_md.rd_auth                     = 0;  // no acks required
  attr_md.wr_auth                     = 0;  // no acks required
  attr_md.vlen                        = 1;  // must be a one ????

  memset(&attr_char_value, 0, sizeof(attr_char_value));

	BLE_UUID_BLE_ASSIGN(service_uuid, eCFG_CHAR_LED_UUID);
  attr_char_value.p_uuid              = &service_uuid;
  attr_char_value.p_attr_md           = &attr_md;
  attr_char_value.init_len            = PLOK_VALUE_LENGTH_LED;
  attr_char_value.init_offs           = 0;
  attr_char_value.max_len             = PLOK_VALUE_LENGTH_LED;
  attr_char_value.p_value             = NULL;

  err_code = sd_ble_gatts_characteristic_add( p_cfg->service_handle,
                                              &char_md,
                                              &attr_char_value,
                                              &p_cfg->led_handle);

    return err_code;
}

static uint32_t cfg_hand_detect_char_add(void)
{
	ble_uuid_t          	service_uuid;
  ble_gatts_char_md_t 	char_md;
  ble_gatts_attr_t    	attr_char_value;
  ble_gatts_attr_md_t 	attr_md;
  ble_gatts_char_pf_t   presentation;
  ret_code_t				    err_code;

  memset(&char_md, 0, sizeof(char_md));
  memset(&presentation, 0, sizeof(presentation));
  presentation.format                 = BLE_GATT_CPF_FORMAT_UINT8; 
  presentation.unit                   = BLUETOOTH_ORG_CPF_UNITLESS; 

  char_md.p_char_pf                   = &presentation;
  char_md.char_props.auth_signed_wr   = 1;
  char_md.char_props.write   			    = 1;
  char_md.char_props.read   			    = 1;
  char_md.p_char_user_desc            = plck_hd_descriptor;
  char_md.char_user_desc_size         = strlen((char *)(char *)plck_hd_descriptor);
  char_md.char_user_desc_max_size     = strlen((char *)(char *)plck_hd_descriptor);

  memset(&attr_md, 0, sizeof(attr_md));

 	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
 	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
 	attr_md.vloc                        = BLE_GATTS_VLOC_STACK;
 	attr_md.rd_auth                     = 0;  // no acks required
 	attr_md.wr_auth                     = 0;  // no acks required
 	attr_md.vlen                        = 1;  // must be a one ????

 	memset(&attr_char_value, 0, sizeof(attr_char_value));

	BLE_UUID_BLE_ASSIGN(service_uuid, eCFG_CHAR_HAND_DETECT_UUID);
 	attr_char_value.p_uuid              = &service_uuid;
 	attr_char_value.p_attr_md           = &attr_md;
 	attr_char_value.init_len            = PLOK_VALUE_LENGTH_HD;
 	attr_char_value.init_offs           = 0;
 	attr_char_value.max_len             = PLOK_VALUE_LENGTH_HD;
 	attr_char_value.p_value             = NULL;

  err_code = sd_ble_gatts_characteristic_add( p_cfg->service_handle,
                                              &char_md,
                                              &attr_char_value,
                                              &p_cfg->hand_detect_handle);

  return err_code;
}

static uint32_t cfg_time_char_add(void)
{
	ble_uuid_t          	service_uuid;
  ble_gatts_char_md_t 	char_md;
  ble_gatts_attr_t    	attr_char_value;
  ble_gatts_attr_md_t 	attr_md;
  ble_gatts_char_pf_t   presentation;
  ret_code_t				    err_code;

	memset(&char_md, 0, sizeof(char_md));
  memset(&presentation, 0, sizeof(presentation));
  presentation.format                 = BLE_GATT_CPF_FORMAT_UINT8; 
  presentation.unit                   = BLUETOOTH_ORG_CPF_UNITLESS; 

  char_md.p_char_pf                   = &presentation;
	char_md.char_props.auth_signed_wr   = 1;
	char_md.char_props.write   					= 1;
	char_md.char_props.read   					= 1;
	char_md.p_char_user_desc            = plck_time_descriptor;
	char_md.char_user_desc_size         = strlen((char *)plck_time_descriptor);
	char_md.char_user_desc_max_size     = strlen((char *)plck_time_descriptor);

	memset(&attr_md, 0, sizeof(attr_md));

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
	attr_md.vloc                        = BLE_GATTS_VLOC_STACK;
	attr_md.rd_auth                     = 0;  // no acks required
	attr_md.wr_auth                     = 0;  // no acks required
	attr_md.vlen                        = 1;  // must be a one ????

	memset(&attr_char_value, 0, sizeof(attr_char_value));

	BLE_UUID_BLE_ASSIGN(service_uuid, eCFG_CHAR_TIME_SET_UUID);
	attr_char_value.p_uuid              = &service_uuid;
	attr_char_value.p_attr_md           = &attr_md;
	attr_char_value.init_len            = PLOK_VALUE_LENGTH_TIME;
	attr_char_value.init_offs           = 0;
	attr_char_value.max_len             = PLOK_VALUE_LENGTH_TIME;
	attr_char_value.p_value             = NULL;

	err_code = sd_ble_gatts_characteristic_add( p_cfg->service_handle,
                                              &char_md,
                                              &attr_char_value,
                                              &p_cfg->time_handle);

	return err_code;
}

static uint32_t cfg_alarm_control_char_add(void)
{
	ble_uuid_t          	service_uuid;
  ble_gatts_char_md_t 	char_md;
  ble_gatts_attr_t    	attr_char_value;
  ble_gatts_attr_md_t 	attr_md;
  ble_gatts_char_pf_t   presentation;
  ret_code_t				    err_code;

	memset(&char_md, 0, sizeof(char_md));
  memset(&presentation, 0, sizeof(presentation));
  presentation.format                 = BLE_GATT_CPF_FORMAT_UINT8; 
  presentation.unit                   = BLUETOOTH_ORG_CPF_UNITLESS; 

  char_md.p_char_pf                   = &presentation;
	char_md.char_props.auth_signed_wr   = 1;
	char_md.char_props.write   					= 1;
	char_md.char_props.read   					= 1;
	char_md.p_char_user_desc            = plck_dr_descriptor;
	char_md.char_user_desc_size         = strlen((char *)plck_alm_ctl_descriptor);
	char_md.char_user_desc_max_size     = strlen((char *)plck_alm_ctl_descriptor);

	memset(&attr_md, 0, sizeof(attr_md));

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
	attr_md.vloc                        = BLE_GATTS_VLOC_STACK;
	attr_md.rd_auth                     = 0;  // no acks required
	attr_md.wr_auth                     = 0;  // no acks required
	attr_md.vlen                        = 1;  // must be a one ????

	memset(&attr_char_value, 0, sizeof(attr_char_value));

	BLE_UUID_BLE_ASSIGN(service_uuid, eCFG_CHAR_ALM_CTL_UUID);
	attr_char_value.p_uuid              = &service_uuid;
	attr_char_value.p_attr_md           = &attr_md;
	attr_char_value.init_len            = PLOK_VALUE_LENGTH_ALM_CTL;
	attr_char_value.init_offs           = 0;
	attr_char_value.max_len             = PLOK_VALUE_LENGTH_ALM_CTL;
	attr_char_value.p_value             = NULL;

	err_code = sd_ble_gatts_characteristic_add( p_cfg->service_handle,
                                              &char_md,
                                              &attr_char_value,
                                              &p_cfg->alarm_control_handle);

	return err_code;
}

static uint32_t cfg_device_reset_char_add(void)
{
	ble_uuid_t          	service_uuid;
  ble_gatts_char_md_t 	char_md;
  ble_gatts_attr_t    	attr_char_value;
  ble_gatts_attr_md_t 	attr_md;
  ble_gatts_char_pf_t   presentation;
  ret_code_t				    err_code;

	memset(&char_md, 0, sizeof(char_md));
  memset(&presentation, 0, sizeof(presentation));
  presentation.format                 = BLE_GATT_CPF_FORMAT_UINT8; 
  presentation.unit                   = BLUETOOTH_ORG_CPF_UNITLESS; 

  char_md.p_char_pf                   = &presentation;
	char_md.char_props.auth_signed_wr   = 1;
	char_md.char_props.write   					= 1;
	char_md.char_props.read   					= 1;
	char_md.p_char_user_desc            = plck_dr_descriptor;
	char_md.char_user_desc_size         = strlen((char *)plck_dr_descriptor);
	char_md.char_user_desc_max_size     = strlen((char *)plck_dr_descriptor);

	memset(&attr_md, 0, sizeof(attr_md));

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
	attr_md.vloc                        = BLE_GATTS_VLOC_STACK;
	attr_md.rd_auth                     = 0;  // no acks required
	attr_md.wr_auth                     = 0;  // no acks required
	attr_md.vlen                        = 1;  // must be a one ????

	memset(&attr_char_value, 0, sizeof(attr_char_value));

	BLE_UUID_BLE_ASSIGN(service_uuid, eCFG_CHAR_DR_UUID);
	attr_char_value.p_uuid              = &service_uuid;
	attr_char_value.p_attr_md           = &attr_md;
	attr_char_value.init_len            = PLOK_VALUE_LENGTH_TIME;
	attr_char_value.init_offs           = 0;
	attr_char_value.max_len             = PLOK_VALUE_LENGTH_TIME;
	attr_char_value.p_value             = NULL;

	err_code = sd_ble_gatts_characteristic_add( p_cfg->service_handle,
                                              &char_md,
                                              &attr_char_value,
                                              &p_cfg->device_reset_handle);

	return err_code;
}

uint32_t cfg_service_init(void)
{
	ble_uuid_t          service_uuid;
  uint8_t             address_type = 0;
  
    uint32_t   err_code = NRF_ERROR_NO_MEM;

    // Initialize service structure
    p_cfg->conn_handle                = BLE_CONN_HANDLE_INVALID;

    (void)memset(&m_cfg, 0, sizeof(m_cfg));
    (void)memset(&m_cfg_init, 0, sizeof(m_cfg_init));
    (void)memset(&service_uuid, 0, sizeof(service_uuid));

		p_cfg_info = (sCfg_service_t *)pavlok_get_p_service_info(SI_CFG);

  err_code = sd_ble_uuid_vs_add(&base_uuid128, &address_type);
  if (err_code != NRF_SUCCESS)
  {
    return err_code;
  }
  
  service_uuid.uuid = eCFG_SVC_UUID;
  service_uuid.type = address_type;

     err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &service_uuid,
                                        &p_cfg->service_handle);
    if (NRF_SUCCESS == err_code)
    {
        err_code = cfg_vb_motor_char_add();
    }

    if (NRF_SUCCESS == err_code)
    {
        err_code = cfg_piezo_char_add();
    }

    if (NRF_SUCCESS == err_code)
    {
        err_code = cfg_zap_char_add();
    }

    if (NRF_SUCCESS == err_code)
    {
        err_code = cfg_led_char_add();
    }

    if (NRF_SUCCESS == err_code)
    {
        err_code = cfg_time_char_add();
    }

    if (NRF_SUCCESS == err_code)
    {
        err_code = cfg_hand_detect_char_add();
    }
    
    if (NRF_SUCCESS == err_code)
    {
        err_code = cfg_alarm_control_char_add();
    }
    
    if (NRF_SUCCESS == err_code)
    {
        err_code = cfg_device_reset_char_add();
    }
    
#ifdef APP_TIMERS
				cfg_timers_init();
#endif
    return err_code;
}


/** @} */
