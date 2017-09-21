/*------------------------------------------------------------------------
**
**	@file				appsvc.c
**
**  @brief			appsvc service functions
**  
**  @details 		This module implements PAVLOK application service 
**							functions
**  
**  @note Attention!
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
#include "nrf_delay.h"

/*------------------------------------------------------------------------
**	@brief Project Include(s)
**------------------------------------------------------------------------
*/
#include "pavlok_common.h"
#include "debug.h"
#include "application_task.h"

#include "appsvc.h"


/*------------------------------------------------------------------------
**	@brief	Global Data
**------------------------------------------------------------------------
*/


#define PLOK_APPSVC_DESC						"Application Download"
#define PLOK_APPSVC_START_DESC			"Application Control"
#define PLOK_APPSVC_AT_DESC			"Application Alarm Triggered"
#define PLOK_APPSVC_ST_DESC			"Application Snooze Triggered"
#define PLOK_APPSVC_AD_DESC			"Application Alarm Disabled"


#define PLOK_APPSVC_CHAR_OTA_LEN		  (20)
#define PLOK_APPSVC_CHAR_CONTROL_LEN	(20)
#define PLOK_APPSVC_CHAR_AT_LEN	      (1)
#define PLOK_APPSVC_CHAR_ST_LEN	      (1)
#define PLOK_APPSVC_CHAR_AD_LEN	      (1)

/*------------------------------------------------------------------------
**	@brief	Global Extern(s)
**------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------
**	@brief	Static Data
**------------------------------------------------------------------------
*/
static uint8_t * plok_ota_descriptor 				= (uint8_t *)PLOK_APPSVC_DESC;
static uint8_t * plok_control_descriptor 		= (uint8_t *)PLOK_APPSVC_START_DESC;
static uint8_t * plok_at_descriptor 		    = (uint8_t *)PLOK_APPSVC_AT_DESC;
static uint8_t * plok_st_descriptor 		    = (uint8_t *)PLOK_APPSVC_ST_DESC;
static uint8_t * plok_ad_descriptor 		    = (uint8_t *)PLOK_APPSVC_AD_DESC;

static appsvc_t			    m_appsvc;
static appsvc_init_t		m_appsvc_init;
static appsvc_t 		* 	p_appsvc = &m_appsvc;
static int8_t           m_appsvc_list_count = 0;

#define APP_TIMERS (1)
#ifdef APP_TIMERS
#define APPSVC_SEND_INTERVAL         pdMS_TO_TICKS(100)  /**< APPSVC_SEND_INTERVAL (ms). */

static TimerHandle_t                	m_appsvc_timer = NULL;     /**< Definition of m_appsvc_timer. */

static void appsvc_timers_start(void);
static void appsvc_timers_stop(void);
#endif

/*------------------------------------------------------------------------
**	@brief	Static Forward References
**------------------------------------------------------------------------
*/
#ifdef APP_TIMERS
/*------------------------------------------------------------------------
**
**	@fn		Function			appsvc_timeout_handle
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
static void appsvc_timeout_handle(TimerHandle_t xTimer)
{
	if (BLE_SERVICE_ENABLED == p_appsvc->char_enable[APPSVC_CHAR_CONTROL])
	{
		appsvc_send(APPSVC_CHAR_CONTROL, APP_CONTROL_LIST);
	}
}

/*------------------------------------------------------------------------
**
**	@fn		Function			appsvc_timers_init
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
static void appsvc_timers_init(void)
{
	pavlok_timers_init("ASVC", &m_appsvc_timer, APPSVC_SEND_INTERVAL, appsvc_timeout_handle);
}

/*------------------------------------------------------------------------
**
**	@fn		Function			appsvc_timers_start
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
static void appsvc_timers_start(void)
{
  if (pdFALSE == xTimerIsTimerActive(m_appsvc_timer))
  {
    // Start application timers.
    if(pdPASS != xTimerStart(m_appsvc_timer, OSTIMER_WAIT_FOR_QUEUE))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
  }
}

/*------------------------------------------------------------------------
**
**	@fn		Function			appsvc_timers_stop
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
static void appsvc_timers_stop(void)
{
  if (pdPASS == xTimerIsTimerActive(m_appsvc_timer))
  {
    if(pdPASS != xTimerStop(m_appsvc_timer , OSTIMER_WAIT_FOR_QUEUE))
    {
       ASSERT(0==1);
    }
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
    	if (p_evt_write->handle == p_appsvc->download_handle.cccd_handle)
    	{
    		p_appsvc->char_enable[APPSVC_CHAR_OTA] = p_evt_write->data[0];
     	}
    	else if (p_evt_write->handle == p_appsvc->control_handle.cccd_handle)
    	{
    		p_appsvc->char_enable[APPSVC_CHAR_CONTROL] = p_evt_write->data[0];
    	}
	}
}

/*------------------------------------------------------------------------
**
**	@fn		Function			appsvc_send
**
**	@brief	Description	send service events/data to application
**
**	@param [in]					eAPPSVC_CHAR_LIST char_type - the sub service that 
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
void appsvc_send(eAPPSVC_CHAR_LIST char_type, uint32_t data)
{
  // Send value if connected and notifying
  if (p_appsvc->conn_handle != BLE_CONN_HANDLE_INVALID)
  {
    ble_gatts_hvx_params_t hvx_params;
    uint16_t              hvx_len = 0;
    uint8_t								stream_buffer[PLOK_APPSVC_CHAR_OTA_LEN];

    /** ---------------------------------------------------------------
     **	Send data
     **	all data bytes are sent even if set to zero
     **	---------------------------------------------------------------
     */
    memset(&hvx_params, 0, sizeof(hvx_params));
    memset(stream_buffer, 0, PLOK_APPSVC_CHAR_CONTROL_LEN);

    if ((APPSVC_CHAR_CONTROL == char_type)
        && (true == p_appsvc->char_enable[APPSVC_CHAR_CONTROL])
        && (APP_CONTROL_LIST == (eAppControl_t)data))
    {
      if (FLASH_SECTOR_APPLICATION_END > m_appsvc_list_count)
      {
        // we start off at flash address zero and work up to the largest address
        ret_code_t  check_err_code;
        // go to the next flash app entry and write it
        uint32_t  flash_address = (m_appsvc_list_count * PAVLOK_APP_BLOCK_SIZE);

        // we have to read and verify that the next sector is blank
        // g_pavlok_scratch_pad should be zeroed from last load
        (void)memset(g_pavlok_scratch_pad, 0, PAVLOK_APP_BLOCK_SIZE);
        // TODO verify that the g_pavlok_scratch_pad is not busy
        check_err_code = (uint32_t)serial_flash_read_data(g_pavlok_scratch_pad, flash_address, 30);
        if ((SERIAL_FLASH_SUCCESS == check_err_code)
            && (UNUSED_SECTOR != g_pavlok_scratch_pad[0]))
        {
          (void)memcpy(&stream_buffer[0], &g_pavlok_scratch_pad[PAVLOK_TLV_AP_NAME_INDEX], g_pavlok_scratch_pad[PAVLOK_TLV_AP_NAME_SIZE_INDEX]);
          hvx_len = g_pavlok_scratch_pad[PAVLOK_TLV_AP_NAME_SIZE_INDEX];
          hvx_params.handle = p_appsvc->control_handle.value_handle;				
        }
      }
      else
      {
        appsvc_timers_stop();
      }    
    }
    else  if ((APPSVC_CHAR_ALARM_TRIGGERED == char_type)
              && (true == p_appsvc->char_enable[APPSVC_CHAR_ALARM_TRIGGERED]))   
    {
      (void)memcpy(&stream_buffer[0], &data, sizeof(data));
      hvx_len = sizeof(data);
      hvx_params.handle = p_appsvc->download_handle.value_handle;				         
    }          
    else  if ((APPSVC_CHAR_SNOOZE_SET == char_type)
              && (true == p_appsvc->char_enable[APPSVC_CHAR_SNOOZE_SET]))   
    {
      (void)memcpy(&stream_buffer[0], &data, sizeof(data));
      hvx_len = sizeof(data);
      hvx_params.handle = p_appsvc->download_handle.value_handle;				         
    }          
    else  if ((APPSVC_CHAR_ALARM_DISABLED == char_type)
              && (true == p_appsvc->char_enable[APPSVC_CHAR_ALARM_DISABLED]))   
    {
      (void)memcpy(&stream_buffer[0], &data, sizeof(data));
      hvx_len = sizeof(data);
      hvx_params.handle = p_appsvc->download_handle.value_handle;				         
    }                 
    
    if (0 < hvx_len)
    { 
      uint8_t * encoded_info = pavlok_common_get_encode_buffer();  // get common buffer
      pavlok_encode(stream_buffer, encoded_info, hvx_len);

      hvx_params.p_data = encoded_info;
      hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
      hvx_params.handle = p_appsvc->download_handle.value_handle;
      hvx_params.offset = 0;
      hvx_params.p_len  = &hvx_len;

      (void)sd_ble_gatts_hvx(p_appsvc->conn_handle, &hvx_params);
    }
}
}

/*------------------------------------------------------------------------
**
**	@fn		Function			on_write
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
static void on_write(ble_evt_t * p_ble_evt)
{
	ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

	if ((CCCD_MSG_SIZE == p_evt_write->len)
			&& ((p_evt_write->handle == p_appsvc->download_handle.cccd_handle)
			|| (p_evt_write->handle == p_appsvc->control_handle.cccd_handle)))
	{
		// update cccd
		update_cccd_enabled(p_ble_evt);
	}

  if ((p_evt_write->handle == p_appsvc->control_handle.value_handle)
		  && (BLE_SERVICE_ENABLED == p_appsvc->char_enable[APPSVC_CHAR_CONTROL]))
	{
		// This service 
		// can start an application by name,
		// can cancel alarms by name
		// can delete alarms
		// can enable multiple (chain linear) alarms by name
		// can overlay alarms by name
		// all data is written to the scratch area and then the app will deal with it
		if (APP_CONTROL_ALARM >= p_evt_write->data[1])
		{
			application_set_ready(p_evt_write);
		}
    else if (APP_CONTROL_LIST == p_evt_write->data[1])
		{
			appsvc_timers_start();
		}
	}
	else if ((p_evt_write->handle == p_appsvc->download_handle.value_handle)
          && (BLE_SERVICE_ENABLED == p_appsvc->char_enable[APPSVC_CHAR_OTA]))
	{
    application_write_to_ram(&p_evt_write->data[0], p_evt_write->len);
	}
}


/** ----------------------------------------------------------------------
**
**	@fn		Function			appsvc_on_ble_evt
**
**	@brief	Description	called from the ble event handler in main
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
void appsvc_on_ble_evt(ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
        	p_appsvc->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
           break;

        case BLE_GAP_EVT_DISCONNECTED:
        	p_appsvc->conn_handle = BLE_CONN_HANDLE_INVALID;
#ifdef APP_TIMERS
      appsvc_timers_stop();
#endif
            break;

        case BLE_GATTS_EVT_WRITE:
           on_write(p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}

/** ----------------------------------------------------------------------
**
**	@fn		Function			appsvc_data_char_add
**
**	@brief	Description	start the data gatt characteristic
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
static uint32_t appsvc_download_char_add(void)
{
	ble_uuid_t          	service_uuid;
    ble_gatts_char_md_t 	char_md;
    ble_gatts_attr_t    	attr_char_value;
    ble_gatts_attr_md_t 	attr_md;
  ble_gatts_char_pf_t   presentation;
    uint32_t				err_code = NRF_ERROR_INVALID_PARAM;

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
    char_md.char_props.auth_signed_wr   = 1;
    char_md.char_props.write   			    = 1;
    char_md.char_props.read   			    = 1;
    char_md.char_props.notify   		    = 1;
    char_md.p_char_user_desc            = plok_ota_descriptor;
    char_md.char_user_desc_size         = strlen((char *)plok_ota_descriptor); // use sizeof so no NULL is xmitted
    char_md.char_user_desc_max_size     = strlen((char *)plok_ota_descriptor);
    char_md.p_cccd_md                   = &cccd_md;

		BLE_UUID_BLE_ASSIGN(service_uuid, eAPP_CHAR_DOWNLOAD_UUID);
    	memset(&attr_md, 0, sizeof(attr_md));

    	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    	attr_md.vloc                        = BLE_GATTS_VLOC_STACK;
    	attr_md.rd_auth                     = 0;  // no acks required
    	attr_md.wr_auth                     = 0;  // no acks required
    	attr_md.vlen                        = 1;  // must be a one ????

    	memset(&attr_char_value, 0, sizeof(attr_char_value));

    	attr_char_value.p_uuid              = &service_uuid;
    	attr_char_value.p_attr_md           = &attr_md;
    	attr_char_value.init_len            = PLOK_APPSVC_CHAR_OTA_LEN;
    	attr_char_value.init_offs           = 0;
    	attr_char_value.max_len             = PLOK_APPSVC_CHAR_OTA_LEN;
    	attr_char_value.p_value             = NULL;

    	err_code = sd_ble_gatts_characteristic_add(p_appsvc->service_handle,
    			&char_md,
				&attr_char_value,
				&p_appsvc->download_handle);

    return err_code;
}

/** ----------------------------------------------------------------------
**
**	@fn		Function			appsvc_control_char_add
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
static uint32_t appsvc_control_char_add(void)
{
	ble_uuid_t          	service_uuid;
  ble_gatts_char_md_t 	char_md;
  ble_gatts_attr_t    	attr_char_value;
  ble_gatts_attr_md_t 	attr_md;
  ble_gatts_char_pf_t   presentation;
  uint32_t				      err_code;

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
    char_md.char_props.auth_signed_wr   = 1;
    char_md.char_props.write   			= 1;
    char_md.char_props.read   			= 1;
    char_md.char_props.notify   		= 1;
    char_md.p_char_user_desc            = plok_control_descriptor;
    char_md.char_user_desc_size         = strlen((char *)plok_control_descriptor);
    char_md.char_user_desc_max_size     = strlen((char *)plok_control_descriptor);
    char_md.p_char_pf = &presentation;

    char_md.p_cccd_md                   = &cccd_md;
    char_md.p_sccd_md                   = NULL;


        memset(&attr_md, 0, sizeof(attr_md));

    	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    	attr_md.vloc                        = BLE_GATTS_VLOC_STACK;
    	attr_md.rd_auth                     = 0;  // no acks required
    	attr_md.wr_auth                     = 0;  // no acks required
    	attr_md.vlen                        = 1;  // must be a one ????

    	memset(&attr_char_value, 0, sizeof(attr_char_value));

		BLE_UUID_BLE_ASSIGN(service_uuid, eAPP_CHAR_CONTROL_UUID);

    	attr_char_value.p_uuid              = &service_uuid;
    	attr_char_value.p_attr_md           = &attr_md;
    	attr_char_value.init_len            = PLOK_APPSVC_CHAR_CONTROL_LEN;
    	attr_char_value.init_offs           = 0;
    	attr_char_value.max_len             = PLOK_APPSVC_CHAR_CONTROL_LEN;
    	attr_char_value.p_value             = NULL;

    	err_code = sd_ble_gatts_characteristic_add(p_appsvc->service_handle,
    			&char_md,
				&attr_char_value,
				&p_appsvc->control_handle);

    return err_code;
}

/** ----------------------------------------------------------------------
**
**	@fn		Function			appsvc_alarm_triggered_char_add
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
static uint32_t appsvc_alarm_triggered_char_add(void)
{
	ble_uuid_t          	service_uuid;
  ble_gatts_char_md_t 	char_md;
  ble_gatts_attr_t    	attr_char_value;
  ble_gatts_attr_md_t 	attr_md;
  ble_gatts_char_pf_t   presentation;
  uint32_t				      err_code;

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
    char_md.char_props.read   			= 1;
    char_md.char_props.notify   		= 1;
    char_md.p_char_user_desc            = plok_at_descriptor;
    char_md.char_user_desc_size         = strlen((char *)plok_at_descriptor);
    char_md.char_user_desc_max_size     = strlen((char *)plok_at_descriptor);
    char_md.p_char_pf = &presentation;

    char_md.p_cccd_md                   = &cccd_md;
    char_md.p_sccd_md                   = NULL;


        memset(&attr_md, 0, sizeof(attr_md));

    	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    	attr_md.vloc                        = BLE_GATTS_VLOC_STACK;
    	attr_md.rd_auth                     = 0;  // no acks required
    	attr_md.wr_auth                     = 0;  // no acks required
    	attr_md.vlen                        = 1;  // must be a one ????

    	memset(&attr_char_value, 0, sizeof(attr_char_value));

		BLE_UUID_BLE_ASSIGN(service_uuid, eAPP_CHAR_ALARM_NOTIFY_UUID);

    	attr_char_value.p_uuid              = &service_uuid;
    	attr_char_value.p_attr_md           = &attr_md;
    	attr_char_value.init_len            = PLOK_APPSVC_CHAR_AT_LEN;
    	attr_char_value.init_offs           = 0;
    	attr_char_value.max_len             = PLOK_APPSVC_CHAR_AT_LEN;
    	attr_char_value.p_value             = NULL;

    	err_code = sd_ble_gatts_characteristic_add(p_appsvc->service_handle,
    			&char_md,
				&attr_char_value,
				&p_appsvc->at_handle);

    return err_code;
}

/** ----------------------------------------------------------------------
**
**	@fn		Function			appsvc_snooze_triggered_char_add
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
static uint32_t appsvc_snooze_triggered_char_add(void)
{
	ble_uuid_t          	service_uuid;
  ble_gatts_char_md_t 	char_md;
  ble_gatts_attr_t    	attr_char_value;
  ble_gatts_attr_md_t 	attr_md;
  ble_gatts_char_pf_t   presentation;
  uint32_t				      err_code;

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
    char_md.char_props.read   			= 1;
    char_md.char_props.notify   		= 1;
    char_md.p_char_user_desc            = plok_st_descriptor;
    char_md.char_user_desc_size         = strlen((char *)plok_st_descriptor);
    char_md.char_user_desc_max_size     = strlen((char *)plok_st_descriptor);
    char_md.p_char_pf = &presentation;

    char_md.p_cccd_md                   = &cccd_md;
    char_md.p_sccd_md                   = NULL;


        memset(&attr_md, 0, sizeof(attr_md));

    	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    	attr_md.vloc                        = BLE_GATTS_VLOC_STACK;
    	attr_md.rd_auth                     = 0;  // no acks required
    	attr_md.wr_auth                     = 0;  // no acks required
    	attr_md.vlen                        = 1;  // must be a one ????

    	memset(&attr_char_value, 0, sizeof(attr_char_value));

		BLE_UUID_BLE_ASSIGN(service_uuid, eAPP_CHAR_SNOOZE_NOTIFY_UUID);

    	attr_char_value.p_uuid              = &service_uuid;
    	attr_char_value.p_attr_md           = &attr_md;
    	attr_char_value.init_len            = PLOK_APPSVC_CHAR_AT_LEN;
    	attr_char_value.init_offs           = 0;
    	attr_char_value.max_len             = PLOK_APPSVC_CHAR_AT_LEN;
    	attr_char_value.p_value             = NULL;

    	err_code = sd_ble_gatts_characteristic_add(p_appsvc->service_handle,
    			&char_md,
				&attr_char_value,
				&p_appsvc->st_handle);

    return err_code;
}

/** ----------------------------------------------------------------------
**
**	@fn		Function			appsvc_alarm_disabled_char_add
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
static uint32_t appsvc_alarm_disabled_char_add(void)
{
	ble_uuid_t          	service_uuid;
  ble_gatts_char_md_t 	char_md;
  ble_gatts_attr_t    	attr_char_value;
  ble_gatts_attr_md_t 	attr_md;
  ble_gatts_char_pf_t   presentation;
  uint32_t				      err_code;

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
    char_md.char_props.read   			= 1;
    char_md.char_props.notify   		= 1;
    char_md.p_char_user_desc            = plok_ad_descriptor;
    char_md.char_user_desc_size         = strlen((char *)plok_ad_descriptor);
    char_md.char_user_desc_max_size     = strlen((char *)plok_ad_descriptor);
    char_md.p_char_pf = &presentation;

    char_md.p_cccd_md                   = &cccd_md;
    char_md.p_sccd_md                   = NULL;


        memset(&attr_md, 0, sizeof(attr_md));

    	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    	attr_md.vloc                        = BLE_GATTS_VLOC_STACK;
    	attr_md.rd_auth                     = 0;  // no acks required
    	attr_md.wr_auth                     = 0;  // no acks required
    	attr_md.vlen                        = 1;  // must be a one ????

    	memset(&attr_char_value, 0, sizeof(attr_char_value));

		BLE_UUID_BLE_ASSIGN(service_uuid, eAPP_CHAR_ALARM_DISABLE_NOTIFY_UUID);

    	attr_char_value.p_uuid              = &service_uuid;
    	attr_char_value.p_attr_md           = &attr_md;
    	attr_char_value.init_len            = PLOK_APPSVC_CHAR_AD_LEN;
    	attr_char_value.init_offs           = 0;
    	attr_char_value.max_len             = PLOK_APPSVC_CHAR_AD_LEN;
    	attr_char_value.p_value             = NULL;

    	err_code = sd_ble_gatts_characteristic_add(p_appsvc->service_handle,
    			&char_md,
				&attr_char_value,
				&p_appsvc->ad_handle);

    return err_code;
}

/** ----------------------------------------------------------------------
**
**	@fn		Function				get_appsvc_service_data_cccd_handle
**
**	@brief	Description		return the cccd handle
**
**  @note   See include file for further infor on this function 
**
**	@warn		TODO remove if not used
**
**  ----------------------------------------------------------------------
*/
uint16_t	get_appsvc_service_data_cccd_handle(void)
{
	return m_appsvc.download_handle.cccd_handle;
}

/** ----------------------------------------------------------------------
**
**	@fn		Function				appsvc_service_init
**
**	@brief	Description		initialize the appsvc service
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
uint32_t appsvc_service_init()
{
	ble_uuid_t          service_uuid;
  uint32_t   					err_code = NRF_ERROR_NO_MEM;
  uint8_t             address_type = 0;

	service_uuid.type = BLE_UUID_TYPE_VENDOR_BEGIN;

    // Initialize service structure
    p_appsvc->conn_handle                = BLE_CONN_HANDLE_INVALID;

    (void)memset(&m_appsvc, 0, sizeof(m_appsvc));
    (void)memset(&m_appsvc_init, 0, sizeof(m_appsvc_init));
    (void)memset(&service_uuid, 0, sizeof(service_uuid));

  err_code = sd_ble_uuid_vs_add(&base_uuid128, &address_type);
  if (err_code != NRF_SUCCESS)
  {
    return err_code;
  }
  
  service_uuid.uuid = eAPP_SVC_UUID;
  service_uuid.type = address_type;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &service_uuid,
                                        &p_appsvc->service_handle);

    if (NRF_SUCCESS == err_code)
    {
        err_code = appsvc_control_char_add();
    }

    if (NRF_SUCCESS == err_code)
    {
        err_code = appsvc_download_char_add();
    }

    if (NRF_SUCCESS == err_code)
    {
        err_code = appsvc_alarm_triggered_char_add();
    }

    if (NRF_SUCCESS == err_code)
    {
        err_code = appsvc_snooze_triggered_char_add();
    }

    if (NRF_SUCCESS == err_code)
    {
        err_code = appsvc_alarm_disabled_char_add();
    }

    appsvc_timers_init();
    
    return err_code;
}


/** @} */
