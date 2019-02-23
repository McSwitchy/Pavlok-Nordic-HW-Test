/*------------------------------------------------------------------------
**
**	@file				notification.c
**
**  @brief			notification service functions
**  
**  @details 		This module implements PAVLOK application service 
**							functions
**  
**  @note Attention! TODO add cccd protection in send around all char
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
#include <stdio.h>
#include <nrf_assert.h>

#include "nordic_common.h"
#include "ble_l2cap.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "nrf_log.h"

/*------------------------------------------------------------------------
**	@brief Project Include(s)
**------------------------------------------------------------------------
*/
#include "pavlok_common.h"
#include "debug.h"
#include "rtc.h"
#include "application_task.h"


#include "notification.h"
#include "logger.h"

/*------------------------------------------------------------------------
**	@brief	Global Data
**------------------------------------------------------------------------
*/
#define 	NOTIFICATION_CHAR_TIME_DESC						"Get Time"
#define 	NOTIFICATION_CHAR_LOG_CNT_DESC				"Log Count"
#define 	NOTIFICATION_CHAR_APP_CURRENT_DESC		"Application Name"
#define 	NOTIFICATION_CHAR_BTN_PRESS_DESC			"Button Press"
#define 	NOTIFICATION_CHAR_ZAP_CNT_DESC				"Zap Count"
#define 	NOTIFICATION_CHAR_PIEZO_CNT_DESC			"Piezo Count"
#define 	NOTIFICATION_CHAR_MOTOR_DESC					"Motor Count"
#define 	NOTIFICATION_CHAR_LED_DESC						"LED Count"
#define 	NOTIFICATION_CHAR_SLEEP_DATA_DESC			"Sleep Data"
#define 	NOTIFICATION_CHAR_ALARM_TIME_DESC			"Get Alarm Time"

/*------------------------------------------------------------------------
**	@brief	Global Extern(s)
**------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------
**	@brief	Static Data
**------------------------------------------------------------------------
*/

static uint8_t * plok_time_descriptor 				= (uint8_t *)NOTIFICATION_CHAR_TIME_DESC;
static uint8_t * plok_log_descriptor 					= (uint8_t *)NOTIFICATION_CHAR_LOG_CNT_DESC;
static uint8_t * plok_app_descriptor 					= (uint8_t *)NOTIFICATION_CHAR_APP_CURRENT_DESC;
static uint8_t * plok_btn_descriptor 					= (uint8_t *)NOTIFICATION_CHAR_BTN_PRESS_DESC;
static uint8_t * plok_zap_descriptor 					= (uint8_t *)NOTIFICATION_CHAR_ZAP_CNT_DESC;
static uint8_t * plok_piezo_descriptor 				= (uint8_t *)NOTIFICATION_CHAR_PIEZO_CNT_DESC;
static uint8_t * plok_motor_descriptor 				= (uint8_t *)NOTIFICATION_CHAR_MOTOR_DESC;
static uint8_t * plok_led_descriptor 					= (uint8_t *)NOTIFICATION_CHAR_LED_DESC;
static uint8_t * plok_sleep_descriptor 				= (uint8_t *)NOTIFICATION_CHAR_SLEEP_DATA_DESC;
static uint8_t * plok_alarm_time_descriptor 	= (uint8_t *)NOTIFICATION_CHAR_ALARM_TIME_DESC;

static notification_t					m_notification;
static notification_init_t		m_notification_init;

static notification_t 		* 	p_notification = &m_notification;

#define APP_TIMERS (1)
#ifdef APP_TIMERS
#define NOTIFY_SEND_INTERVAL             pdMS_TO_TICKS(100)
static TimerHandle_t                	m_notification_timer;
static uint32_t		char_send_count						= 0;
static void notify_timers_start(void);
static void notify_timers_stop(void);
#endif

/*------------------------------------------------------------------------
**	@brief	Static Forward References
**------------------------------------------------------------------------
*/
static uint32_t notification_send(eNOT_CHAR_LIST char_type);

#ifdef APP_TIMERS

/*------------------------------------------------------------------------
**
**	@fn		Function			notify_timeout_handle
**
**	@brief	Description	service timer timeout function
**
**	@param [in]						TimerHandle_t xTimer - not used
**
**	@param	[out]					None
**
**	@return								None
**
**	@retval
**
**	@note									
**
**	@warn
**
**------------------------------------------------------------------------
*/
static void notify_timeout_handle(TimerHandle_t xTimer)
{
	(void)notification_send((eNOT_CHAR_LIST)char_send_count);
	char_send_count += 1;
	if (char_send_count >= (uint32_t)eNOTIFICATION_CHAR_LAST)
	{
		char_send_count = 0;
	}

}

/*------------------------------------------------------------------------
**
**	@fn		Function			notify_timers_init
**
**	@brief	Description	service timer initialization function
**
**	@param [in]						None
**
**	@param	[out]					None
**
**	@return								None
**
**	@retval
**
**	@note									
**
**	@warn
**
**------------------------------------------------------------------------
*/
static void notify_timers_init(void)
{
	pavlok_timers_init("NTFY", &m_notification_timer, NOTIFY_SEND_INTERVAL, notify_timeout_handle);
}

/*------------------------------------------------------------------------
**
**	@fn		Function			notify_timers_start
**
**	@brief	Description	service timer start function
**
**	@param [in]						None
**
**	@param	[out]					None
**
**	@return								None
**
**	@retval
**
**	@note									
**
**	@warn
**
**------------------------------------------------------------------------
*/
static void notify_timers_start(void)
{
  if (pdFALSE == xTimerIsTimerActive(m_notification_timer))
  {
    // Start application timers.
    if(pdPASS != xTimerStart(m_notification_timer, OSTIMER_WAIT_FOR_QUEUE))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
  }
}

/*------------------------------------------------------------------------
**
**	@fn		Function			notify_timers_stop
**
**	@brief	Description	service timer stop function
**
**	@param [in]						None
**
**	@param	[out]					None
**
**	@return								None
**
**	@retval
**
**	@note									
**
**	@warn
**
**------------------------------------------------------------------------
*/
static void notify_timers_stop(void)
{
    if(pdPASS != xTimerStop(m_notification_timer , OSTIMER_WAIT_FOR_QUEUE))
    {
        ASSERT(0==1);
    }
}
#endif

/*------------------------------------------------------------------------
**
**	@fn		Function			update_cccd_enabled
**
**	@brief	Description	enable the service - this comes from the app
**											zero disables and one enables
**
**	@param [in]					ble_evt_t * p_ble_evt - pointer the latest BLE event
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
static void update_cccd_enabled(ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (CCCD_MSG_SIZE == p_evt_write->len)
    {
    	if (p_evt_write->handle == p_notification->time_handle.cccd_handle)
    	{
    		p_notification->char_enable[eNOTIFICATION_CHAR_TIME] = p_evt_write->data[0];
     	}
      else if (p_evt_write->handle == p_notification->log_cnt_handle.cccd_handle)
    	{
    		p_notification->char_enable[eNOTIFICATION_CHAR_LOG_CNT] = p_evt_write->data[0];
    	}
     	else if (p_evt_write->handle == p_notification->app_name_handle.cccd_handle)
    	{
    		p_notification->char_enable[eNOTIFICATION_CHAR_APP_CURRENT] = p_evt_write->data[0];
    	}
     	else if (p_evt_write->handle == p_notification->btn_press_handle.cccd_handle)
    	{
    		p_notification->char_enable[eNOTIFICATION_CHAR_BTN_PRESS] = p_evt_write->data[0];
    	}
     	else if (p_evt_write->handle == p_notification->zap_cnt_handle.cccd_handle)
    	{
    		p_notification->char_enable[eNOTIFICATION_CHAR_ZAP_CNT] = p_evt_write->data[0];
    	}
     	else if (p_evt_write->handle == p_notification->piezo_cnt_handle.cccd_handle)
    	{
    		p_notification->char_enable[eNOTIFICATION_CHAR_PIEZO_CNT] = p_evt_write->data[0];
    	}
     	else if (p_evt_write->handle == p_notification->motor_cnt_handle.cccd_handle)
    	{
    		p_notification->char_enable[eNOTIFICATION_CHAR_MOTOR] = p_evt_write->data[0];
    	}
     	else if (p_evt_write->handle == p_notification->led_cnt_handle.cccd_handle)
    	{
    		p_notification->char_enable[eNOTIFICATION_CHAR_LED] = p_evt_write->data[0];
    	}
     	else if (p_evt_write->handle == p_notification->sleep_handle.cccd_handle)
    	{
    		p_notification->char_enable[eNOTIFICATION_CHAR_SLEEP_DATA] = p_evt_write->data[0];
    	}
      	else if (p_evt_write->handle == p_notification->alarm_time_handle.cccd_handle)
    	{
    		p_notification->char_enable[eNOTIFICATION_CHAR_ALARM_TIME] = p_evt_write->data[0];
    	}
			notify_timers_start();
		}
}

/*------------------------------------------------------------------------
**
**	@fn		Function			notification_send
**
**	@brief	Description	send service events/data to application
**
**	@param [in]					eACCEL_CHAR_LIST char_type - the sub service that 
**											was requested by the application
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
static uint32_t notification_send(eNOT_CHAR_LIST char_type)
{
    uint32_t err_code	=	NRF_ERROR_INVALID_PARAM;

    // Send value if connected and notifying
    if (p_notification->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        ble_gatts_hvx_params_t hvx_params;
        uint16_t              hvx_len = 0;
				uint8_t								send_buffer[NOTIFICATION_CHAR_APP_CURRENT_LENGTH];

        /** ---------------------------------------------------------------
         **	Send data
         **	all data bytes are sent even if set to zero
         **	---------------------------------------------------------------
         */
        memset(&hvx_params, 0, sizeof(hvx_params));
        memset(send_buffer, 0, NOTIFICATION_CHAR_APP_CURRENT_LENGTH);
				sNotificationInfo_t	* p_info = pavlok_get_p_service_info(SI_NOTIFY);
			
				if ((eNOTIFICATION_CHAR_TIME == char_type)
             && (true == p_notification->char_enable[eNOTIFICATION_CHAR_TIME]))
				{
					if (!(rtc_get_time((sTime_t *)send_buffer)))
					{
						hvx_len = sizeof(sTime_t);
						hvx_params.handle = p_notification->time_handle.value_handle;				
					}
				}
				else if ((eNOTIFICATION_CHAR_LOG_CNT == char_type)
                  && (true == p_notification->char_enable[eNOTIFICATION_CHAR_LOG_CNT]))
				{
          uint16_t * log_count = (uint16_t *)&send_buffer[0];
          * log_count = logger_get_entry_count();
          
					hvx_len = NOTIFICATION_CHAR_LOG_CNT_LENGTH;
					hvx_params.handle = p_notification->log_cnt_handle.value_handle;				
				}
				else if ((eNOTIFICATION_CHAR_APP_CURRENT == char_type)
                  && (true == p_notification->char_enable[eNOTIFICATION_CHAR_APP_CURRENT]))
				{
          application_get_current_name((char *)send_buffer);
					hvx_len = strlen((char *)send_buffer);
					hvx_params.handle = p_notification->app_name_handle.value_handle;				
				}
				else if ((eNOTIFICATION_CHAR_BTN_PRESS == char_type)
                  && (true == p_notification->char_enable[eNOTIFICATION_CHAR_BTN_PRESS]))
				{
					send_buffer[0]= p_info->count_button_press;
					hvx_len = NOTIFICATION_CHAR_BTN_PRESS_LENGTH;
					hvx_params.handle = p_notification->btn_press_handle.value_handle;				
				}
				else if ((eNOTIFICATION_CHAR_ZAP_CNT == char_type)
                  && (true == p_notification->char_enable[eNOTIFICATION_CHAR_ZAP_CNT]))
				{
					send_buffer[0]= p_info->count_zap;
					hvx_len = NOTIFICATION_CHAR_ZAP_CNT_LENGTH;
					hvx_params.handle = p_notification->zap_cnt_handle.value_handle;				
				}
				else if ((eNOTIFICATION_CHAR_PIEZO_CNT == char_type)
                  && (true == p_notification->char_enable[eNOTIFICATION_CHAR_PIEZO_CNT]))
				{
					send_buffer[0]= p_info->count_piezo;
					hvx_len = NOTIFICATION_CHAR_PIEZO_CNT_LENGTH;
					hvx_params.handle = p_notification->piezo_cnt_handle.value_handle;				
				}
				else if ((eNOTIFICATION_CHAR_MOTOR == char_type)
                  && (true == p_notification->char_enable[eNOTIFICATION_CHAR_MOTOR]))
				{
					send_buffer[0]= p_info->count_motor;
					hvx_len = NOTIFICATION_CHAR_MOTOR_LENGTH;
					hvx_params.handle = p_notification->motor_cnt_handle.value_handle;				
				}
				else if ((eNOTIFICATION_CHAR_LED == char_type)
                  && (true == p_notification->char_enable[eNOTIFICATION_CHAR_LED]))
				{
					send_buffer[0]= p_info->count_led;
					hvx_len = NOTIFICATION_CHAR_LED_LENGTH;
					hvx_params.handle = p_notification->led_cnt_handle.value_handle;				
				}
				else if ((eNOTIFICATION_CHAR_SLEEP_DATA == char_type)
                  && (true == p_notification->char_enable[eNOTIFICATION_CHAR_SLEEP_DATA]))
				{
					// TODO fix this so that all sleep data gets xferred
					hvx_len = strlen((const char *)p_info->sleep_data);
					(void)memcpy(send_buffer, (const void *)&p_info->sleep_data, hvx_len);
					hvx_params.handle = p_notification->sleep_handle.value_handle;				
				}
				else if ((eNOTIFICATION_CHAR_ALARM_TIME == char_type)
                  && (true == p_notification->char_enable[eNOTIFICATION_CHAR_ALARM_TIME]))
				{
          RTC_TIME_STRUCT_T alarmTime;
          (void)memset(&alarmTime, 0, sizeof(RTC_TIME_STRUCT_T));
          RTC_TXRX_RET_T rtn_code = rtc_get_alarm(&alarmTime);
          if (RTC_SUCCESS == rtn_code)
          {
            hvx_len =  NOTIFICATION_CHAR_ALARM_TIME_LENGTH;                                          /**< Handles related to the notification Measurement characteristic. */

            (void)memcpy(send_buffer, (const void *)&alarmTime, hvx_len);
             hvx_params.handle = p_notification->alarm_time_handle.value_handle;		
          }            
				}
				else
				{
					ASSERT(0==1);
				}
				
        if (0 < hvx_len)
        { 
        	uint8_t * encoded_info = pavlok_common_get_encode_buffer();  // get common buffer
        	pavlok_encode(&send_buffer[0], &encoded_info[0], hvx_len);

          hvx_params.p_data = encoded_info;
					hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
					hvx_params.offset = 0;
					hvx_params.p_len  = &hvx_len;
        }
				
        err_code = sd_ble_gatts_hvx(p_notification->conn_handle, &hvx_params);
        if (err_code == NRF_SUCCESS)
        {
            err_code = NRF_ERROR_DATA_SIZE;
        }
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}

/** ----------------------------------------------------------------------
**
**	@fn		Function			notification_on_ble_evt
**
**	@brief	Description	called from the ble event handler in main
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
void notification_on_ble_evt(ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
        	p_notification->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
          break;

        case BLE_GAP_EVT_DISCONNECTED:
        	p_notification->conn_handle = BLE_CONN_HANDLE_INVALID;
#ifdef APP_TIMERS
					notify_timers_stop();
#endif
            break;

        case BLE_GATTS_EVT_WRITE:
					if (CCCD_MSG_SIZE == p_ble_evt->evt.gatts_evt.params.write.len)
					{
						// update cccd
						update_cccd_enabled(p_ble_evt);
					}
          break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
					notify_timers_stop();
          break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
 					notify_timers_stop();
         break;

        case BLE_GATTS_EVT_HVC:
					notify_timers_stop();
          break;

        case BLE_GATTS_EVT_SC_CONFIRM:
 					notify_timers_stop();
         break;

        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
 					notify_timers_stop();
         break;

        case BLE_GATTS_EVT_TIMEOUT:
 					notify_timers_stop();
         break;

        default:
            // No implementation needed.
            break;
    }
}

/** ----------------------------------------------------------------------
**
**	@fn		Function			notification_char_add_time
**
**	@brief	Description	start the control gatt characteristic
**
**	@param [in]					None
**
**	@param	[out]				None
**
**	@return							uint32_t
**
**	@retval							NRF_ERROR_INVALID_PARAM
**	@retval							NRF_SUCCESS
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
static uint32_t notification_char_add_time(void)
{
	ble_uuid_t          	service_uuid;
  ble_gatts_char_md_t 	char_md;
  ble_gatts_char_pf_t   presentation;
  uint32_t				err_code;


   // use the  CCD security only in the first characteristic add
    ble_gatts_attr_md_t cccd_md;

    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
		cccd_md.vlen                        = 1;  // must be a one ????
    cccd_md.vloc                        = BLE_GATTS_VLOC_STACK;


    memset(&char_md, 0, sizeof(char_md));
    memset(&presentation, 0, sizeof(presentation));
    presentation.format                 = BLE_GATT_CPF_FORMAT_UINT8; 
    presentation.unit                   = BLUETOOTH_ORG_CPF_TIME; 

    char_md.p_char_pf                   = &presentation;
    char_md.char_props.read   					= 1;
    char_md.char_props.notify   				= 1;
    char_md.p_char_user_desc            = plok_time_descriptor;
    char_md.char_user_desc_size         = strlen((char *)plok_time_descriptor);
    char_md.char_user_desc_max_size     = strlen((char *)plok_time_descriptor);
    char_md.p_cccd_md                   = &cccd_md;

		BLE_UUID_BLE_ASSIGN(service_uuid, eNOTIFICATION_CHAR_TIME_UUID);

    ble_gatts_attr_t    	attr_char_value;
    ble_gatts_attr_md_t 	attr_md;

    memset(&attr_md, 0, sizeof(attr_md));

   	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
   	BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
   	attr_md.vloc                        = BLE_GATTS_VLOC_STACK;
   	attr_md.rd_auth                     = 0;  // no acks required
   	attr_md.wr_auth                     = 0;  // no acks required
   	attr_md.vlen                        = 1;  // must be a one ????

   	memset(&attr_char_value, 0, sizeof(attr_char_value));

   	attr_char_value.p_uuid              = &service_uuid;
   	attr_char_value.p_attr_md           = &attr_md;
   	attr_char_value.init_len            = NOTIFICATION_CHAR_TIME_LENGTH;
   	attr_char_value.init_offs           = 0;
   	attr_char_value.max_len             = NOTIFICATION_CHAR_TIME_LENGTH;
   	attr_char_value.p_value             = NULL;

   	err_code = sd_ble_gatts_characteristic_add(p_notification->service_handle,
   			&char_md,
				&attr_char_value,
				&p_notification->time_handle);

    return err_code;
}



/** ----------------------------------------------------------------------
**
**	@fn		Function			notification_char_add_log
**
**	@brief	Description	start the control gatt characteristic
**
**	@param [in]					None
**
**	@param	[out]				None
**
**	@return							uint32_t
**
**	@retval							NRF_ERROR_INVALID_PARAM
**	@retval							NRF_SUCCESS
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
static uint32_t notification_char_add_log(void)
{
	ble_uuid_t          	service_uuid;
  ble_gatts_char_md_t 	char_md;
  ble_gatts_char_pf_t   presentation;
  uint32_t				err_code;


   // use the  CCD security only in the first characteristic add
    ble_gatts_attr_md_t cccd_md;

    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
		cccd_md.vlen                        = 1;  // must be a one ????
    cccd_md.vloc                        = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));
    memset(&presentation, 0, sizeof(presentation));
    presentation.format                 = BLE_GATT_CPF_FORMAT_UINT8; 
    presentation.unit                   = BLUETOOTH_ORG_CPF_UNITLESS; 

    char_md.p_char_pf                   = &presentation;
    char_md.char_props.read   					= 1;
    char_md.char_props.notify   				= 1;
    char_md.p_char_user_desc            = plok_log_descriptor;
    char_md.char_user_desc_size         = strlen((char *)plok_log_descriptor);
    char_md.char_user_desc_max_size     = strlen((char *)plok_log_descriptor);
    char_md.p_cccd_md                   = &cccd_md;

		BLE_UUID_BLE_ASSIGN(service_uuid, eNOTIFICATION_CHAR_LOG_CNT_UUID);

        ble_gatts_attr_t    	attr_char_value;
        ble_gatts_attr_md_t 	attr_md;

        memset(&attr_md, 0, sizeof(attr_md));

    	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    	BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    	attr_md.vloc                        = BLE_GATTS_VLOC_STACK;
    	attr_md.rd_auth                     = 0;  // no acks required
    	attr_md.wr_auth                     = 0;  // no acks required
    	attr_md.vlen                        = 1;  // must be a one ????

    	memset(&attr_char_value, 0, sizeof(attr_char_value));

    	attr_char_value.p_uuid              = &service_uuid;
    	attr_char_value.p_attr_md           = &attr_md;
    	attr_char_value.init_len            = NOTIFICATION_CHAR_LOG_CNT_LENGTH;
    	attr_char_value.init_offs           = 0;
    	attr_char_value.max_len             = NOTIFICATION_CHAR_LOG_CNT_LENGTH;
    	attr_char_value.p_value             = NULL;

    	err_code = sd_ble_gatts_characteristic_add(p_notification->service_handle,
    			&char_md,
					&attr_char_value,
					&p_notification->log_cnt_handle);

    return err_code;
}



/** ----------------------------------------------------------------------
**
**	@fn		Function			notification_char_add_app
**
**	@brief	Description	start the control gatt characteristic
**
**	@param [in]					None
**
**	@param	[out]				None
**
**	@return							uint32_t
**
**	@retval							NRF_ERROR_INVALID_PARAM
**	@retval							NRF_SUCCESS
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
static uint32_t notification_char_add_app(void)
{
	ble_uuid_t          	service_uuid;
  ble_gatts_char_md_t 	char_md;
  ble_gatts_char_pf_t   presentation;
  uint32_t				err_code;


   // use the  CCD security only in the first characteristic add
    ble_gatts_attr_md_t cccd_md;

    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
		cccd_md.vlen                        = 1;  // must be a one ????
    cccd_md.vloc                        = BLE_GATTS_VLOC_STACK;


    memset(&char_md, 0, sizeof(char_md));
    memset(&presentation, 0, sizeof(presentation));
    presentation.format                 = BLE_GATT_CPF_FORMAT_UINT8; 
    presentation.unit                   = BLUETOOTH_ORG_CPF_UNITLESS; 

    char_md.p_char_pf                   = &presentation;
    char_md.char_props.read   					= 1;
    char_md.char_props.notify   				= 1;
    char_md.p_char_user_desc            = plok_app_descriptor;
    char_md.char_user_desc_size         = strlen((char *)plok_app_descriptor);
    char_md.char_user_desc_max_size     = strlen((char *)plok_app_descriptor);
    char_md.p_cccd_md                   = &cccd_md;
    char_md.p_sccd_md                   = NULL;

		BLE_UUID_BLE_ASSIGN(service_uuid, eNOTIFICATION_CHAR_APP_CURRENT_UUID);

        ble_gatts_attr_t    	attr_char_value;
        ble_gatts_attr_md_t 	attr_md;

        memset(&attr_md, 0, sizeof(attr_md));

    	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    	BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    	attr_md.vloc                        = BLE_GATTS_VLOC_STACK;
    	attr_md.rd_auth                     = 0;  // no acks required
    	attr_md.wr_auth                     = 0;  // no acks required
    	attr_md.vlen                        = 1;  // must be a one ????

    	memset(&attr_char_value, 0, sizeof(attr_char_value));

    	attr_char_value.p_uuid              = &service_uuid;
    	attr_char_value.p_attr_md           = &attr_md;
    	attr_char_value.init_len            = NOTIFICATION_CHAR_APP_CURRENT_LENGTH;
    	attr_char_value.init_offs           = 0;
    	attr_char_value.max_len             = NOTIFICATION_CHAR_APP_CURRENT_LENGTH;
    	attr_char_value.p_value             = NULL;

    	err_code = sd_ble_gatts_characteristic_add(p_notification->service_handle,
    			&char_md,
					&attr_char_value,
					&p_notification->app_name_handle);

    return err_code;
}



/** ----------------------------------------------------------------------
**
**	@fn		Function			notification_char_add_btn
**
**	@brief	Description	start the control gatt characteristic
**
**	@param [in]					None
**
**	@param	[out]				None
**
**	@return							uint32_t
**
**	@retval							NRF_ERROR_INVALID_PARAM
**	@retval							NRF_SUCCESS
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
static uint32_t notification_char_add_btn(void)
{
	ble_uuid_t          	service_uuid;
  ble_gatts_char_md_t 	char_md;
  ble_gatts_char_pf_t   presentation;
  uint32_t				err_code;


   // use the  CCD security only in the first characteristic add
    ble_gatts_attr_md_t cccd_md;

    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
		cccd_md.vlen                        = 1;  // must be a one ????
    cccd_md.vloc                        = BLE_GATTS_VLOC_STACK;


    memset(&char_md, 0, sizeof(char_md));
    memset(&presentation, 0, sizeof(presentation));
    presentation.format                 = BLE_GATT_CPF_FORMAT_UINT8; 
    presentation.unit                   = BLUETOOTH_ORG_CPF_UNITLESS;   //TODO replace with proper

    char_md.p_char_pf                   = &presentation;
    char_md.char_props.read   					= 1;
    char_md.char_props.notify   				= 1;
    char_md.p_char_user_desc            = plok_btn_descriptor;
    char_md.char_user_desc_size         = strlen((char *)plok_btn_descriptor);
    char_md.char_user_desc_max_size     = strlen((char *)plok_btn_descriptor);
    char_md.p_cccd_md                   = &cccd_md;
    char_md.p_sccd_md                   = NULL;

		BLE_UUID_BLE_ASSIGN(service_uuid, eNOTIFICATION_CHAR_BTN_PRESS_UUID);

        ble_gatts_attr_t    	attr_char_value;
        ble_gatts_attr_md_t 	attr_md;

        memset(&attr_md, 0, sizeof(attr_md));

    	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    	BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    	attr_md.vloc                        = BLE_GATTS_VLOC_STACK;
    	attr_md.rd_auth                     = 0;  // no acks required
    	attr_md.wr_auth                     = 0;  // no acks required
    	attr_md.vlen                        = 1;  // must be a one ????

    	memset(&attr_char_value, 0, sizeof(attr_char_value));

    	attr_char_value.p_uuid              = &service_uuid;
    	attr_char_value.p_attr_md           = &attr_md;
    	attr_char_value.init_len            = NOTIFICATION_CHAR_SLEEP_DATA_LENGTH;
    	attr_char_value.init_offs           = 0;
    	attr_char_value.max_len             = NOTIFICATION_CHAR_SLEEP_DATA_LENGTH;
    	attr_char_value.p_value             = NULL;

    	err_code = sd_ble_gatts_characteristic_add(p_notification->service_handle,
    			&char_md,
					&attr_char_value,
					&p_notification->btn_press_handle);

    return err_code;
}



/** ----------------------------------------------------------------------
**
**	@fn		Function			notification_char_add_zap
**
**	@brief	Description	start the control gatt characteristic
**
**	@param [in]					None
**
**	@param	[out]				None
**
**	@return							uint32_t
**
**	@retval							NRF_ERROR_INVALID_PARAM
**	@retval							NRF_SUCCESS
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
static uint32_t notification_char_add_zap(void)
{
	ble_uuid_t          	service_uuid;
  ble_gatts_char_md_t 	char_md;
  ble_gatts_char_pf_t   presentation;
  uint32_t				err_code;


   // use the  CCD security only in the first characteristic add
    ble_gatts_attr_md_t cccd_md;

    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
		cccd_md.vlen                        = 1;  // must be a one ????
    cccd_md.vloc                        = BLE_GATTS_VLOC_STACK;


    memset(&char_md, 0, sizeof(char_md));
    memset(&presentation, 0, sizeof(presentation));
    presentation.format                 = BLE_GATT_CPF_FORMAT_UINT8; 
    presentation.unit                   = BLUETOOTH_ORG_CPF_UNITLESS; 

    char_md.p_char_pf                   = &presentation;
    char_md.char_props.read   					= 1;
    char_md.char_props.notify   				= 1;
    char_md.p_char_user_desc            = plok_zap_descriptor;
    char_md.char_user_desc_size         = strlen((char *)plok_zap_descriptor);
    char_md.char_user_desc_max_size     = strlen((char *)plok_zap_descriptor);
    char_md.p_cccd_md                   = &cccd_md;
    char_md.p_sccd_md                   = NULL;

		BLE_UUID_BLE_ASSIGN(service_uuid, eNOTIFICATION_CHAR_ZAP_CNT_UUID);

        ble_gatts_attr_t    	attr_char_value;
        ble_gatts_attr_md_t 	attr_md;

        memset(&attr_md, 0, sizeof(attr_md));

    	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    	BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    	attr_md.vloc                        = BLE_GATTS_VLOC_STACK;
    	attr_md.rd_auth                     = 0;  // no acks required
    	attr_md.wr_auth                     = 0;  // no acks required
    	attr_md.vlen                        = 1;  // must be a one ????

    	memset(&attr_char_value, 0, sizeof(attr_char_value));

    	attr_char_value.p_uuid              = &service_uuid;
    	attr_char_value.p_attr_md           = &attr_md;
    	attr_char_value.init_len            = NOTIFICATION_CHAR_ZAP_CNT_LENGTH;
    	attr_char_value.init_offs           = 0;
    	attr_char_value.max_len             = NOTIFICATION_CHAR_ZAP_CNT_LENGTH;
    	attr_char_value.p_value             = NULL;

    	err_code = sd_ble_gatts_characteristic_add(p_notification->service_handle,
    			&char_md,
					&attr_char_value,
					&p_notification->zap_cnt_handle);

    return err_code;
}



/** ----------------------------------------------------------------------
**
**	@fn		Function			notification_char_add_piezo
**
**	@brief	Description	start the control gatt characteristic
**
**	@param [in]					None
**
**	@param	[out]				None
**
**	@return							uint32_t
**
**	@retval							NRF_ERROR_INVALID_PARAM
**	@retval							NRF_SUCCESS
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
static uint32_t notification_char_add_piezo(void)
{
	ble_uuid_t          	service_uuid;
  ble_gatts_char_md_t 	char_md;
  ble_gatts_char_pf_t   presentation;
  uint32_t				err_code;


   // use the  CCD security only in the first characteristic add
    ble_gatts_attr_md_t cccd_md;

    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
		cccd_md.vlen                        = 1;  // must be a one ????
    cccd_md.vloc                        = BLE_GATTS_VLOC_STACK;


    memset(&char_md, 0, sizeof(char_md));
    memset(&presentation, 0, sizeof(presentation));
    presentation.format                 = BLE_GATT_CPF_FORMAT_UINT8; 
    presentation.unit                   = BLUETOOTH_ORG_CPF_UNITLESS; 

    char_md.p_char_pf                   = &presentation;
    char_md.char_props.read   					= 1;
    char_md.char_props.notify   				= 1;
    char_md.p_char_user_desc            = plok_piezo_descriptor;
    char_md.char_user_desc_size         = strlen((char *)plok_piezo_descriptor);
    char_md.char_user_desc_max_size     = strlen((char *)plok_piezo_descriptor);
    char_md.p_cccd_md                   = &cccd_md;
    char_md.p_sccd_md                   = NULL;

		BLE_UUID_BLE_ASSIGN(service_uuid, eNOTIFICATION_CHAR_PIEZO_CNT_UUID);

        ble_gatts_attr_t    	attr_char_value;
        ble_gatts_attr_md_t 	attr_md;

        memset(&attr_md, 0, sizeof(attr_md));

    	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    	BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    	attr_md.vloc                        = BLE_GATTS_VLOC_STACK;
    	attr_md.rd_auth                     = 0;  // no acks required
    	attr_md.wr_auth                     = 0;  // no acks required
    	attr_md.vlen                        = 1;  // must be a one ????

    	memset(&attr_char_value, 0, sizeof(attr_char_value));

    	attr_char_value.p_uuid              = &service_uuid;
    	attr_char_value.p_attr_md           = &attr_md;
    	attr_char_value.init_len            = NOTIFICATION_CHAR_PIEZO_CNT_LENGTH;
    	attr_char_value.init_offs           = 0;
    	attr_char_value.max_len             = NOTIFICATION_CHAR_PIEZO_CNT_LENGTH;
    	attr_char_value.p_value             = NULL;

    	err_code = sd_ble_gatts_characteristic_add(p_notification->service_handle,
    			&char_md,
					&attr_char_value,
					&p_notification->piezo_cnt_handle);

    return err_code;
}



/** ----------------------------------------------------------------------
**
**	@fn		Function			notification_char_add_motor
**
**	@brief	Description	start the control gatt characteristic
**
**	@param [in]					None
**
**	@param	[out]				None
**
**	@return							uint32_t
**
**	@retval							NRF_ERROR_INVALID_PARAM
**	@retval							NRF_SUCCESS
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
static uint32_t notification_char_add_motor(void)
{
	ble_uuid_t          	service_uuid;
  ble_gatts_char_md_t 	char_md;
  ble_gatts_char_pf_t   presentation;
  uint32_t				err_code;


   // use the  CCD security only in the first characteristic add
    ble_gatts_attr_md_t cccd_md;

    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
		cccd_md.vlen                        = 1;  // must be a one ????
    cccd_md.vloc                        = BLE_GATTS_VLOC_STACK;


    memset(&char_md, 0, sizeof(char_md));
    memset(&presentation, 0, sizeof(presentation));
    presentation.format                 = BLE_GATT_CPF_FORMAT_UINT8; 
    presentation.unit                   = BLUETOOTH_ORG_CPF_UNITLESS; 

    char_md.p_char_pf                   = &presentation;
    char_md.char_props.read   					= 1;
    char_md.char_props.notify   				= 1;
    char_md.p_char_user_desc            = plok_motor_descriptor;
    char_md.char_user_desc_size         = strlen((char *)plok_motor_descriptor);
    char_md.char_user_desc_max_size     = strlen((char *)plok_motor_descriptor);
    char_md.p_cccd_md                   = &cccd_md;

		BLE_UUID_BLE_ASSIGN(service_uuid, eNOTIFICATION_CHAR_MOTOR_UUID);

        ble_gatts_attr_t    	attr_char_value;
        ble_gatts_attr_md_t 	attr_md;

        memset(&attr_md, 0, sizeof(attr_md));

    	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    	BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    	attr_md.vloc                        = BLE_GATTS_VLOC_STACK;
    	attr_md.rd_auth                     = 0;  // no acks required
    	attr_md.wr_auth                     = 0;  // no acks required
    	attr_md.vlen                        = 1;  // must be a one ????

    	memset(&attr_char_value, 0, sizeof(attr_char_value));

    	attr_char_value.p_uuid              = &service_uuid;
    	attr_char_value.p_attr_md           = &attr_md;
    	attr_char_value.init_len            = NOTIFICATION_CHAR_MOTOR_LENGTH;
    	attr_char_value.init_offs           = 0;
    	attr_char_value.max_len             = NOTIFICATION_CHAR_MOTOR_LENGTH;
    	attr_char_value.p_value             = NULL;

    	err_code = sd_ble_gatts_characteristic_add(p_notification->service_handle,
    			&char_md,
					&attr_char_value,
					&p_notification->motor_cnt_handle);

    return err_code;
}

/** ----------------------------------------------------------------------
**
**	@fn		Function			notification_char_add_led
**
**	@brief	Description	start the control gatt characteristic
**
**	@param [in]					None
**
**	@param	[out]				None
**
**	@return							uint32_t
**
**	@retval							NRF_ERROR_INVALID_PARAM
**	@retval							NRF_SUCCESS
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
static uint32_t notification_char_add_led(void)
{
	ble_uuid_t          	service_uuid;
  ble_gatts_char_md_t 	char_md;
  ble_gatts_char_pf_t   presentation;
  uint32_t				err_code;


   // use the  CCD security only in the first characteristic add
    ble_gatts_attr_md_t cccd_md;

    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
		cccd_md.vlen                        = 1;  // must be a one ????
    cccd_md.vloc                        = BLE_GATTS_VLOC_STACK;


    memset(&char_md, 0, sizeof(char_md));
    memset(&presentation, 0, sizeof(presentation));
    presentation.format                 = BLE_GATT_CPF_FORMAT_UINT8; 
    presentation.unit                   = BLUETOOTH_ORG_CPF_UNITLESS; 

    char_md.p_char_pf                   = &presentation;
    char_md.char_props.read   					= 1;
    char_md.char_props.notify   				= 1;
    char_md.p_char_user_desc            = plok_led_descriptor;
    char_md.char_user_desc_size         = strlen((char *)plok_led_descriptor);
    char_md.char_user_desc_max_size     = strlen((char *)plok_led_descriptor);
    char_md.p_cccd_md                   = &cccd_md;

		BLE_UUID_BLE_ASSIGN(service_uuid, eNOTIFICATION_CHAR_LED_UUID);

        ble_gatts_attr_t    	attr_char_value;
        ble_gatts_attr_md_t 	attr_md;

        memset(&attr_md, 0, sizeof(attr_md));

    	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    	BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    	attr_md.vloc                        = BLE_GATTS_VLOC_STACK;
    	attr_md.rd_auth                     = 0;  // no acks required
    	attr_md.wr_auth                     = 0;  // no acks required
    	attr_md.vlen                        = 1;  // must be a one ????

    	memset(&attr_char_value, 0, sizeof(attr_char_value));

    	attr_char_value.p_uuid              = &service_uuid;
    	attr_char_value.p_attr_md           = &attr_md;
    	attr_char_value.init_len            = NOTIFICATION_CHAR_LED_LENGTH;
    	attr_char_value.init_offs           = 0;
    	attr_char_value.max_len             = NOTIFICATION_CHAR_LED_LENGTH;
    	attr_char_value.p_value             = NULL;

    	err_code = sd_ble_gatts_characteristic_add(p_notification->service_handle,
    			&char_md,
					&attr_char_value,
					&p_notification->led_cnt_handle);

    return err_code;
}



/** ----------------------------------------------------------------------
**
**	@fn		Function			notification_char_add_sleep
**
**	@brief	Description	start the control gatt characteristic
**
**	@param [in]					None
**
**	@param	[out]				None
**
**	@return							uint32_t
**
**	@retval							NRF_ERROR_INVALID_PARAM
**	@retval							NRF_SUCCESS
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
static uint32_t notification_char_add_sleep(void)
{
	ble_uuid_t          	service_uuid;
  ble_gatts_char_md_t 	char_md;
  ble_gatts_char_pf_t   presentation;
  uint32_t				err_code;


   // use the  CCD security only in the first characteristic add
    ble_gatts_attr_md_t cccd_md;

    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
		cccd_md.vlen                        = 1;  // must be a one ????
    cccd_md.vloc                        = BLE_GATTS_VLOC_STACK;


    memset(&char_md, 0, sizeof(char_md));
    memset(&presentation, 0, sizeof(presentation));
    presentation.format                 = BLE_GATT_CPF_FORMAT_UINT8; 
    presentation.unit                   = BLUETOOTH_ORG_CPF_UNITLESS; 

    char_md.p_char_pf                   = &presentation;
    char_md.char_props.read   					= 1;
    char_md.char_props.notify   				= 1;
    char_md.p_char_user_desc            = plok_sleep_descriptor;
    char_md.char_user_desc_size         = strlen((char *)plok_sleep_descriptor);
    char_md.char_user_desc_max_size     = strlen((char *)plok_sleep_descriptor);
    char_md.p_cccd_md                   = &cccd_md;
    char_md.p_sccd_md                   = NULL;

		BLE_UUID_BLE_ASSIGN(service_uuid, eNOTIFICATION_CHAR_SLEEP_DATA_UUID);

    ble_gatts_attr_t    	attr_char_value;
    ble_gatts_attr_md_t 	attr_md;

    memset(&attr_md, 0, sizeof(attr_md));

    	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    	BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    	attr_md.vloc                        = BLE_GATTS_VLOC_STACK;
    	attr_md.rd_auth                     = 0;  // no acks required
    	attr_md.wr_auth                     = 0;  // no acks required
    	attr_md.vlen                        = 1;  // must be a one ????

    	memset(&attr_char_value, 0, sizeof(attr_char_value));

    	attr_char_value.p_uuid              = &service_uuid;
    	attr_char_value.p_attr_md           = &attr_md;
    	attr_char_value.init_len            = NOTIFICATION_CHAR_SLEEP_DATA_LENGTH;
    	attr_char_value.init_offs           = 0;
    	attr_char_value.max_len             = NOTIFICATION_CHAR_SLEEP_DATA_LENGTH;
    	attr_char_value.p_value             = NULL;

    	err_code = sd_ble_gatts_characteristic_add(p_notification->service_handle,
    			&char_md,
					&attr_char_value,
					&p_notification->sleep_handle);

    return err_code;
}

/** ----------------------------------------------------------------------
**
**	@fn		Function			notification_char_add_led
**
**	@brief	Description	start the control gatt characteristic
**
**	@param [in]					None
**
**	@param	[out]				None
**
**	@return							uint32_t
**
**	@retval							NRF_ERROR_INVALID_PARAM
**	@retval							NRF_SUCCESS
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
static uint32_t notification_char_add_get_alarm_time(void)
{
	ble_uuid_t          	service_uuid;
  ble_gatts_char_md_t 	char_md;
  ble_gatts_char_pf_t   presentation;
  uint32_t				err_code;


   // use the  CCD security only in the first characteristic add
    ble_gatts_attr_md_t cccd_md;

    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
		cccd_md.vlen                        = 1;  // must be a one ????
    cccd_md.vloc                        = BLE_GATTS_VLOC_STACK;


    memset(&char_md, 0, sizeof(char_md));
    memset(&presentation, 0, sizeof(presentation));
    presentation.format                 = BLE_GATT_CPF_FORMAT_UINT8; 
    presentation.unit                   = BLUETOOTH_ORG_CPF_TIME; 

    char_md.p_char_pf                   = &presentation;
    char_md.char_props.read   					= 1;
    char_md.char_props.notify   				= 1;
    char_md.p_char_user_desc            = plok_alarm_time_descriptor;
    char_md.char_user_desc_size         = strlen((char *)plok_alarm_time_descriptor);
    char_md.char_user_desc_max_size     = strlen((char *)plok_alarm_time_descriptor);
    char_md.p_cccd_md                   = &cccd_md;

		BLE_UUID_BLE_ASSIGN(service_uuid, eNOTIFICATION_CHAR_ALARM_TIME_UUID);

        ble_gatts_attr_t    	attr_char_value;
        ble_gatts_attr_md_t 	attr_md;

        memset(&attr_md, 0, sizeof(attr_md));

    	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    	BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    	attr_md.vloc                        = BLE_GATTS_VLOC_STACK;
    	attr_md.rd_auth                     = 0;  // no acks required
    	attr_md.wr_auth                     = 0;  // no acks required
    	attr_md.vlen                        = 1;  // must be a one ????

    	memset(&attr_char_value, 0, sizeof(attr_char_value));

    	attr_char_value.p_uuid              = &service_uuid;
    	attr_char_value.p_attr_md           = &attr_md;
    	attr_char_value.init_len            = NOTIFICATION_CHAR_ALARM_TIME_LENGTH;
    	attr_char_value.init_offs           = 0;
    	attr_char_value.max_len             = NOTIFICATION_CHAR_ALARM_TIME_LENGTH;
    	attr_char_value.p_value             = NULL;

    	err_code = sd_ble_gatts_characteristic_add(p_notification->service_handle,
    			&char_md,
					&attr_char_value,
					&p_notification->alarm_time_handle);

    return err_code;
}



/** ----------------------------------------------------------------------
**
**	@fn		Function				notification_service_init
**
**	@brief	Description		initialize the notify service
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
uint32_t notification_service_init()
{
	ble_uuid_t          service_uuid;
  uint32_t   					err_code = NRF_ERROR_NO_MEM;
  uint8_t             address_type = 0;

	service_uuid.type = BLE_UUID_TYPE_VENDOR_BEGIN;
	
    // Initialize service structure
    p_notification->conn_handle                = BLE_CONN_HANDLE_INVALID;

    (void)memset(&m_notification, 0, sizeof(m_notification));
    (void)memset(&m_notification_init, 0, sizeof(m_notification_init));
    (void)memset(&service_uuid, 0, sizeof(service_uuid));
  
  err_code = sd_ble_uuid_vs_add(&base_uuid128, &address_type);
  if (err_code != NRF_SUCCESS)
  {
    return err_code;
  }
  
  service_uuid.uuid = eNOTIFICATION_SVC_UUID;
  service_uuid.type = address_type;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &service_uuid,
                                        &p_notification->service_handle);


    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    if (NRF_SUCCESS == err_code)
    {
        err_code = notification_char_add_time();
    }

    if (NRF_SUCCESS == err_code)
    {
        err_code = notification_char_add_log();
    }

    if (NRF_SUCCESS == err_code)
    {
        err_code = notification_char_add_app();
    }

    if (NRF_SUCCESS == err_code)
    {
        err_code = notification_char_add_btn();
    }

    if (NRF_SUCCESS == err_code)
    {
        err_code = notification_char_add_zap();
    }

    if (NRF_SUCCESS == err_code)
    {
        err_code = notification_char_add_piezo();
    }

    if (NRF_SUCCESS == err_code)
    {
        err_code = notification_char_add_motor();
    }

    if (NRF_SUCCESS == err_code)
    {
        err_code = notification_char_add_led();
    }

    if (NRF_SUCCESS == err_code)
    {
        err_code = notification_char_add_sleep();
    }

    if (NRF_SUCCESS == err_code)
    {
        err_code = notification_char_add_get_alarm_time();
    }

		notify_timers_init();			
		
    return err_code;
}

void notification_increment_button_count(void)
{  
  sNotificationInfo_t	* p_info = pavlok_get_p_service_info(SI_NOTIFY);
  p_info->count_button_press += 1;
}

void notification_increment_motor_count(void)
{  
  sNotificationInfo_t	* p_info = pavlok_get_p_service_info(SI_NOTIFY);
  p_info->count_motor += 1;
}

void notification_increment_piezo_count(void)
{  
  sNotificationInfo_t	* p_info = pavlok_get_p_service_info(SI_NOTIFY);
  p_info->count_piezo += 1;
}

void notification_increment_zap_count(void)
{  
  sNotificationInfo_t	* p_info = pavlok_get_p_service_info(SI_NOTIFY);
  p_info->count_led += 1;
}


void notification_increment_led_count(void)
{  
  sNotificationInfo_t	* p_info = pavlok_get_p_service_info(SI_NOTIFY);
  p_info->count_led += 1;
}


/** @} */
