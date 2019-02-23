/*------------------------------------------------------------------------
**
**	@file				solicited.c
**
**  @brief			solicited service functions
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

#include "SEGGER_RTT.h"
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

#include "solicited.h"
#include "serial_flash.h"



/*------------------------------------------------------------------------
**	@brief	Global Data
**------------------------------------------------------------------------
*/


#define PLOK_SOLICITED_DESC						"Control"
#define PLOK_SOLICITED_DATA_DESC			"DATA"


#define PLOK_SOLICITED_CHAR_DATA_LEN		(20)
#define PLOK_SOLICITED_CHAR_CONTROL_LEN	(2)

/*------------------------------------------------------------------------
**	@brief	Global Extern(s)
**------------------------------------------------------------------------
*/
typedef struct 
{
  bool                active;
  eFlashAreaName_t    type;
  int32_t             current_sector;
  
} sFlashErase_t;
/*------------------------------------------------------------------------
**	@brief	Static Data
**------------------------------------------------------------------------
*/

static const uint8_t * plok_data_descriptor 				= (uint8_t *)PLOK_SOLICITED_DATA_DESC;
static const uint8_t * plok_control_descriptor 			= (uint8_t *)PLOK_SOLICITED_DESC;

static solicited_t			  m_solicited;
static solicited_init_t		m_solicited_init;
static solicited_t 		* 	p_solicited = &m_solicited;
static sFlashErase_t      flash_control;

#define APP_TIMERS (1)
#ifdef APP_TIMERS
#define SOLICITED_TIMER_INTERVAL             pdMS_TO_TICKS(310)  /**< solicited_SEND_INTERVAL (ms). */

static TimerHandle_t                	m_solicited_timer = NULL;     /**< Definition of m_solicited_timer. */

static void solicited_timers_start(void);
static void solicited_timers_stop(void);
#endif

/*------------------------------------------------------------------------
**	@brief	Static Forward References
**------------------------------------------------------------------------
*/
//static uint8_t skbbTxBuffer[4096] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 0, 'S', 'i', 'n', 'g', 'l', 'e', ' ', '0'};
static uint8_t skbbTxBuffer[4096];
static uint8_t skbbRxBuffer[4096];
#define TEST_BYTE  (0xA5)
/*------------------------------------------------------------------------
**
**	@fn		Function			solicited_service_get_handle
**
**	@brief	Description	get service handle
**
**	@param [in]						None
**
**	@param	[out]					None
**
**	@return								None
**
**	@retval
**
**	@warn
**
**------------------------------------------------------------------------
*/

uint16_t solicited_service_get_handle(void)
{
  return p_solicited->service_handle;
}

/*------------------------------------------------------------------------
**
**	@fn		Function			solicited_service_get_connection_handle
**
**	@brief	Description	get service connection handle
**
**	@param [in]						None
**
**	@param	[out]					None
**
**	@return								None
**
**	@retval
**
**	@note									connection handle is a zero when enabled
**
**	@warn
**
**------------------------------------------------------------------------
*/
uint16_t solicited_service_get_connection_handle(void)
{
  return p_solicited->conn_handle;
}

#ifdef APP_TIMERS

/*------------------------------------------------------------------------
**
**	@fn		Function			solicited_timeout_handle
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
static uint8_t  command = 55;
static uint8_t  sector = 128;
static int8_t   name_counter = 0x30;

static void solicited_timeout_handle(TimerHandle_t xTimer)
{
    switch (command)
    {
#ifdef TBD      
      case 0 :
      {
        flash_control.type = eFlashAll;
        flash_control.current_sector = 0;
        solicited_timers_start();
        ret_code_t  rtn_code = serial_flash_sector_erase(0 * 4096);
        APP_ERROR_CHECK(rtn_code);
      }
      break;
      
      case 1 :
      {
        // flash selected sector
        uint32_t flash_address = (p_evt_write->data[1] * 4096);
        ret_code_t  rtn_code = serial_flash_sector_erase(flash_address);
        APP_ERROR_CHECK(rtn_code);
      }
      break;
      
      case 2 :
      {
        // erase application by name
        // we must make a string out of the data..N
        uint8_t   sector_to_erase = BAD_SECTOR;
        char      name[20]        = {0};
        
        (void)memcpy(name, &p_evt_write->data[0], (p_evt_write->len - 1));
        name[19] = 0;
        sector_to_erase = find_name_in_sectors(name, eFlashApp);
        if (BAD_SECTOR != sector_to_erase)
        {
          ret_code_t  rtn_code = serial_flash_sector_erase(sector_to_erase);
          APP_ERROR_CHECK(rtn_code);
        }
      }
      break;
      
      case 3 :
      {
        // erase all apps
        flash_control.type = eFlashApp;
        flash_control.current_sector = find_flash_area_start(eFlashApp);
        solicited_timers_start();
        ret_code_t  rtn_code = serial_flash_sector_erase((flash_control.current_sector * 4096));
        APP_ERROR_CHECK(rtn_code);
      }
      break;
      
      case 4 :
      {
        // erase habit by name
        // we must make a string out of the data..N
        uint8_t   sector_to_erase = 128;
        char      name[20]        = {0};
        
        (void)memcpy(name, &p_evt_write->data[0], (p_evt_write->len - 1));
        name[19] = 0;
        sector_to_erase = find_name_in_sectors(name, eFlashApp);
        ret_code_t  rtn_code = serial_flash_sector_erase(sector_to_erase);
        APP_ERROR_CHECK(rtn_code);
      }
      break;
      
      case 5 :
      {
        // erase all habits
        flash_control.type = eFlashHabit;
        flash_control.current_sector = find_flash_area_start(eFlashHabit);
        solicited_timers_start();
        ret_code_t  rtn_code = serial_flash_sector_erase((flash_control.current_sector * 4096));
        APP_ERROR_CHECK(rtn_code);
      }
      break;
      
      case 6 :
      {
        // erase sleep data
        flash_control.type = eFlashSleep;
        flash_control.current_sector = find_flash_area_start(eFlashSleep);
        solicited_timers_start();
        ret_code_t  rtn_code = serial_flash_sector_erase((flash_control.current_sector * 4096));
        APP_ERROR_CHECK(rtn_code);
      }
      break;
#endif      
      case 7 :
      {
        uint32_t flash_address = (sector * 4096);
              set_led(LED_RED);
              clear_led(LED4);
              clear_led(LED_GREEN);
        SEGGER_RTT_WriteString(0, "EC\r\n");
        ret_code_t  rtn_code = serial_flash_sector_erase(flash_address);
        APP_ERROR_CHECK(rtn_code);
      }
      break;
     
      case 8 :
      {
        uint32_t flash_address = (sector * 4096);
        SEGGER_RTT_WriteString(0, "WC\r\n");
              set_led(LED4);
              clear_led(LED_RED);
              clear_led(LED_GREEN);
        skbbTxBuffer[17] = name_counter++;
        ret_code_t  rtn_code = spi_flash_start_write_wrapper(skbbTxBuffer, flash_address, 4096);
        APP_ERROR_CHECK(rtn_code);
      }
      break;
      
      case 9 :
      {
        uint32_t flash_address = (sector * 4096);
        SEGGER_RTT_WriteString(0, "RC\r\n");
        clear_led(LED4);
        ret_code_t  rtn_code = serial_flash_read_data(skbbRxBuffer, flash_address, 4096);
        if (SERIAL_FLASH_SUCCESS == rtn_code)
        {
          for (int rx_counter = 0; rx_counter < 4096; rx_counter++)
          {
            if (skbbTxBuffer[rx_counter] != skbbRxBuffer[rx_counter])
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
        memset(skbbRxBuffer, 0, 4096);
      }
      break;
     
    } // eos
    
    command = 55;
    sector = 128;
    solicited_timers_stop();

#ifdef REMOVED
	if (BLE_SERVICE_ENABLED == p_solicited->control_enable)
	{
		if (eFlashLast != flash_control.type)
    {
      if (eFlashAll == flash_control.type)
      {
        
      }
      else if (eFlashApp == flash_control.type)
      {
      }
      else if (eFlashHabit == flash_control.type)
      {
      }
      else if (eFlashSleep == flash_control.type)
      {
      }
      else if (eFlashLog == flash_control.type)
      {
      }
    }
	}
#endif

    
}


static void solicited_total_test_timeout_handle(TimerHandle_t xTimer)
{
  solicited_timers_stop();
}

/*------------------------------------------------------------------------
**
**	@fn		Function			solicited_timers_init
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
static void solicited_timers_init(void)
{
pavlok_timers_init("SOLC", &m_solicited_timer, SOLICITED_TIMER_INTERVAL, solicited_timeout_handle);
}

/*------------------------------------------------------------------------
**
**	@fn		Function			solicited_timers_start
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
static void solicited_timers_start(void)
{
  if (pdFALSE == xTimerIsTimerActive(m_solicited_timer))
  {
    // Start application timers.
    if(pdPASS != xTimerStart(m_solicited_timer, OSTIMER_WAIT_FOR_QUEUE))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
  }
}

/*------------------------------------------------------------------------
**
**	@fn		Function			solicited_timers_stop
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
static void solicited_timers_stop(void)
{
  if (pdPASS == xTimerIsTimerActive(m_solicited_timer))
  {
    if(pdPASS != xTimerStop(m_solicited_timer , OSTIMER_WAIT_FOR_QUEUE))
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
    	if (p_evt_write->handle == p_solicited->control_handle.cccd_handle)
    	{
    		p_solicited->control_enable = p_evt_write->data[0];
     	}
      
    	if (p_evt_write->handle == p_solicited->data_handle.cccd_handle)
    	{
    		p_solicited->data_enable = p_evt_write->data[0];
     	}
	}
}

/*------------------------------------------------------------------------
**
**	@fn		Function			solicited_send
**
**	@brief	Description	send service events/data to application
**
**	@param [in]					esolicited_CHAR_LIST char_type - the sub service that 
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
uint32_t solicited_send(bool value)
{
    uint32_t err_code	=	NRF_ERROR_INVALID_PARAM;
  
    // Send value if connected and notifying
    if (p_solicited->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        ble_gatts_hvx_params_t hvx_params;
        uint16_t              hvx_len = 0;
				uint8_t								stream_buffer[PLOK_SOLICITED_CHAR_DATA_LEN];
        bool              *   p_data = (bool *)stream_buffer;

        /** ---------------------------------------------------------------
         **	Send data
         **	all data bytes are sent even if set to zero
         **	---------------------------------------------------------------
         */
        * p_data  = value;
        memset(&hvx_params, 0, sizeof(hvx_params));
        memset(stream_buffer, 0, PLOK_SOLICITED_CHAR_DATA_LEN);
        
        uint8_t * encoded_info = pavlok_common_get_encode_buffer();  // get common buffer
        pavlok_encode(stream_buffer, encoded_info, hvx_len);

        hvx_params.p_data = encoded_info;
				hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
				hvx_params.handle = p_solicited->data_handle.value_handle;
				hvx_params.offset = 0;
				hvx_params.p_len  = &hvx_len;

        err_code = sd_ble_gatts_hvx(p_solicited->conn_handle, &hvx_params);
				DEBUGI_APP_500("SEND", err_code);
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

// called from main/butten event to start the solicited
void start_solicited_session(void)
{
#ifdef APP_TIMERS
	solicited_timers_init();			
	solicited_timers_start();
#endif
}

void stop_solicited_session(void)
{
#ifdef APP_TIMERS
	solicited_timers_stop();					
#endif
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
			&& (p_evt_write->handle == p_solicited->control_handle.cccd_handle))
	{
		// update cccd
		update_cccd_enabled(p_ble_evt);
	}

  if ((p_evt_write->handle == p_solicited->control_handle.value_handle)
		  && (BLE_SERVICE_ENABLED == p_solicited->control_enable))
	{
     command = p_evt_write->data[0];
     sector = p_evt_write->data[1];
     start_solicited_session();
  }
  
  if ((p_evt_write->handle == p_solicited->data_handle.value_handle)
		  && (BLE_SERVICE_ENABLED == p_solicited->data_enable))
	{
	}

}


/** ----------------------------------------------------------------------
**
**	@fn		Function			log_info_on_ble_evt
**
**	@brief	Description	called from the ble event handler in main
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
void solicited_on_ble_evt(ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
        	p_solicited->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    clear_led(LED_GREEN);
    clear_led(LED_RED);
    clear_led(LED4);
               break;

        case BLE_GAP_EVT_DISCONNECTED:
        	p_solicited->conn_handle = BLE_CONN_HANDLE_INVALID;
#ifdef APP_TIMERS
      solicited_timers_stop();
#endif
            break;

        case BLE_GATTS_EVT_WRITE:
//			DEBUGS_APP_400("ble_evt_dispatch");
           on_write(p_ble_evt);
            break;

        default:
          // TODO add in timer for sends and attach to interrupt
            // No implementation needed.
            break;
    }
}

/** ----------------------------------------------------------------------
**
**	@fn		Function			solicited_data_char_add
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
static uint32_t solicited_data_char_add(void)
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
//    char_md.char_props.read   			    = 1;
    char_md.char_props.notify   		    = 1;
    char_md.p_char_user_desc            = plok_data_descriptor;
    char_md.char_user_desc_size         = strlen((char *)plok_data_descriptor); // use sizeof so no NULL is xmitted
    char_md.char_user_desc_max_size     = strlen((char *)plok_data_descriptor);
    char_md.p_cccd_md                   = &cccd_md;

		BLE_UUID_BLE_ASSIGN(service_uuid, eSOLICITED_CHAR_LOG_DATA_UUID);
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
    	attr_char_value.init_len            = PLOK_SOLICITED_CHAR_DATA_LEN;
    	attr_char_value.init_offs           = 0;
    	attr_char_value.max_len             = PLOK_SOLICITED_CHAR_DATA_LEN;
    	attr_char_value.p_value             = NULL;

    	err_code = sd_ble_gatts_characteristic_add(p_solicited->service_handle,
    			&char_md,
				&attr_char_value,
				&p_solicited->data_handle);

    return err_code;
}

/** ----------------------------------------------------------------------
**
**	@fn		Function			solicited_control_char_add
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
static uint32_t solicited_control_char_add(void)
{
	ble_uuid_t          	service_uuid;
  ble_gatts_char_md_t 	char_md;
  ble_gatts_attr_t    	attr_char_value;
  ble_gatts_attr_md_t 	attr_md;
  ble_gatts_char_pf_t   presentation;
  uint32_t				      err_code;
  ble_gatts_attr_md_t   cccd_md;

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
  char_md.char_props.write   			    = 1;
  char_md.char_props.read   			    = 1;
  char_md.char_props.notify   		    = 1;
  char_md.p_char_user_desc            = plok_control_descriptor;
  char_md.char_user_desc_size         = strlen((char *)plok_control_descriptor);
  char_md.char_user_desc_max_size     = strlen((char *)plok_control_descriptor);
  char_md.p_char_pf = &presentation;

  char_md.p_cccd_md                   = &cccd_md;

  memset(&attr_md, 0, sizeof(attr_md));

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
  attr_md.vloc                        = BLE_GATTS_VLOC_STACK;
  attr_md.rd_auth                     = 0;  // no acks required
  attr_md.wr_auth                     = 0;  // no acks required
  attr_md.vlen                        = 1;  // must be a one ????

  memset(&attr_char_value, 0, sizeof(attr_char_value));

	BLE_UUID_BLE_ASSIGN(service_uuid, eSOLICITED_CHAR_CONTROL_UUID);

  attr_char_value.p_uuid              = &service_uuid;
  attr_char_value.p_attr_md           = &attr_md;
  attr_char_value.init_len            = PLOK_SOLICITED_CHAR_CONTROL_LEN;
  attr_char_value.init_offs           = 0;
  attr_char_value.max_len             = PLOK_SOLICITED_CHAR_CONTROL_LEN;
  attr_char_value.p_value             = NULL;

  err_code = sd_ble_gatts_characteristic_add(p_solicited->service_handle,
    		&char_md,
				&attr_char_value,
				&p_solicited->control_handle);

  return err_code;
}

/** ----------------------------------------------------------------------
**
**	@fn		Function				solicited_service_init
**
**	@brief	Description		initialize the solicited service
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
uint32_t solicited_service_init()
{
	ble_uuid_t          service_uuid;
  uint32_t   					err_code = NRF_ERROR_NO_MEM;
  uint8_t             address_type = 0;

	service_uuid.type = BLE_UUID_TYPE_VENDOR_BEGIN;

    // Initialize service structure
    p_solicited->conn_handle                = BLE_CONN_HANDLE_INVALID;

    (void)memset(&m_solicited, 0, sizeof(m_solicited));
    (void)memset(&m_solicited_init, 0, sizeof(m_solicited_init));
    (void)memset(&service_uuid, 0, sizeof(service_uuid));

  err_code = sd_ble_uuid_vs_add(&base_uuid128, &address_type);
  if (err_code != NRF_SUCCESS)
  {
    return err_code;
  }
  
  service_uuid.uuid = eSOLICITED_SVC_UUID;
  service_uuid.type = address_type;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &service_uuid,
                                        &p_solicited->service_handle);

    if (NRF_SUCCESS == err_code)
    {
        err_code = solicited_control_char_add();
    }

    if (NRF_SUCCESS == err_code)
    {
        err_code = solicited_data_char_add();
    }
        memset(&skbbTxBuffer[18], TEST_BYTE, 4096);


    return err_code;
}


/** @} */
