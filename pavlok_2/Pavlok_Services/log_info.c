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
#include <nrf_assert.h>
#include "FreeRTOS.h"
#include "timers.h"

#include "nordic_common.h"
#include "ble_l2cap.h"
#include "ble_srv_common.h"
#include "app_util.h"




/*------------------------------------------------------------------------
**	@brief Project Include(s)
**------------------------------------------------------------------------
*/
#include "pavlok_common.h"
#include "debug.h"
#include "logger.h"
#include "log_info.h"

/*------------------------------------------------------------------------
**	@brief	Global Data
**------------------------------------------------------------------------
*/
#define LOG_TIMERS (1)


#define PLOK_LOG_CMD_DESC		"Log Command"
#define PLOK_LOG_DATA_DESC	"Log Data"

#define PLOK_LOG_CMD_LENGTH		(1)
#define PLOK_LOG_COUNT_LENGTH	(2)
#define PLOK_LOG_CMD_INDEX		(0)
#define PLOK_LOG_CMD_INDEX		(0)
#define LOG_INFO_MULTI_ENTRIES_SEND_INC_COUNT	(1) // this can be one or 2 to fit in 23 byte writes
#define	LOG_INFO_SEND_INTERVAL	pdMS_TO_TICKS(1000)


typedef enum
{
//  REMOVED	LOG_CMD_GET_COUNT,
	LOG_CMD_READ    = 1,
	LOG_CMD_READ_ALL,
	LOG_CMD_STOP_READ,
	
	LOG_CMD_LAST

} eLogCmd_t;

typedef struct
{
  uint8_t	cmd;
  uint8_t	msb_count; // 00
  uint8_t	lsb_count; // 00 means dump all logs

} sLogInfoCmd_t;


typedef struct
{
	bool			dirty_bit;
	eLogCmd_t	current_command;
	uint16_t	current_log_count;	// total number of logs
	uint16_t	current_log_read;		//  current index for individual and all log reads

} sLogInfo_t;

/*------------------------------------------------------------------------
**	@brief	Global Extern(s)
**------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------
**	@brief	Static Data
**------------------------------------------------------------------------
*/
static uint8_t * log_cmd_descriptor 	= (uint8_t *)PLOK_LOG_CMD_DESC;
static uint8_t * log_data_descriptor 	= (uint8_t *)PLOK_LOG_DATA_DESC;

static log_info_t				m_log_info;
static log_info_init_t	m_log_info_init;
static log_info_t 		* p_log_info = &m_log_info;

static sLogInfo_t			log_status;
static uint32_t 			log_info_send(eLogCmd_t cmd);

/*------------------------------------------------------------------------
**	@brief	Static Forward References
**------------------------------------------------------------------------
*/
#ifdef LOG_TIMERS
#define LOG_ENTRY_SEND_INTERVAL             pdMS_TO_TICKS(100)

static TimerHandle_t               			m_log_info_timer;
static void 														log_info_timers_start(void);
static void 														log_info_timers_stop(void);
#endif

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
#ifdef LOG_TIMERS

/**@brief Function for handling the Heart rate measurement timer time-out.
 *
 * @details This function will be called each time the heart rate measurement timer expavlok_cfges.
 *          It will exclude RR Interval data from every third measurement.
 *
 * @param[in] xTimer Handler to the timer that called this function.
 *                   You may get identifier given to the function xTimerCreate using pvTimerGetTimerID.
 */

static void log_info_timeout_handle(TimerHandle_t xTimer)
{
  UNUSED_PARAMETER(xTimer);
	log_info_send(log_status.current_command);
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts log service timers.
 * this timer is to check the flash r/w states to see if we can remove the pending action
 */
void log_info_timers_init(void)
{
	pavlok_timers_init("LOGI", &m_log_info_timer, LOG_INFO_SEND_INTERVAL, log_info_timeout_handle);
}


/**@brief Function for starting application timers.
 */
void log_info_timers_start(void)
{
  if (pdFALSE == xTimerIsTimerActive(m_log_info_timer))
  {
    // Start application timers.
    if(pdPASS != xTimerStart(m_log_info_timer, OSTIMER_WAIT_FOR_QUEUE))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
  }
  else
  {
		ASSERT(0 == 1);
  }
}

void log_info_timers_stop(void)
{
    if(pdPASS != xTimerStop(m_log_info_timer , OSTIMER_WAIT_FOR_QUEUE))
    {
        ASSERT(0==1);
    }
}
#endif

uint16_t log_info_service_get_handle(void)
{
  return p_log_info->service_handle;
}

static void update_cccd_enabled(ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (CCCD_MSG_SIZE == p_evt_write->len)
    {
    	if (p_evt_write->handle == p_log_info->log_cmd_handle.cccd_handle)
    	{
    		p_log_info->char_enable[LOG_CHAR_CMD] = p_evt_write->data[0];
     	}
    	else if (p_evt_write->handle == p_log_info->log_data_handle.cccd_handle)
    	{
    		p_log_info->char_enable[LOG_CHAR_DATA] = p_evt_write->data[0];
    	}
    	else
    	{
    		ASSERT(0==1);
    	}
	}
}

static uint32_t log_info_send(eLogCmd_t cmd)
{
		uint32_t								err_code = NRF_ERROR_NOT_SUPPORTED;
	
    // Send value if connected and notifying
    if (p_log_info->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        ble_gatts_hvx_params_t hvx_params;
			
        uint8_t				hvx_data[PAVLOK_LOG_ENTRY_SIZE] = {0};
        uint16_t      hvx_len = 0;

        /** ---------------------------------------------------------------
         **	Send data
         **	all data bytes are sent even if set to zero
         **	---------------------------------------------------------------
         */
        memset(&hvx_params, 0, sizeof(hvx_params));

        if (LOG_CMD_READ == cmd)
        {
					// get the next entry
        	err_code = logger_entry_read(hvx_data);
					if (0 == err_code)
					{
						hvx_params.handle = p_log_info->log_data_handle.value_handle;
						hvx_len = PAVLOK_LOG_ENTRY_SIZE;
					  log_info_timers_stop(); // TODO test if there is an error
					}
       }
        else  if (LOG_CMD_READ_ALL == cmd)
        {
        	err_code = logger_entry_read(hvx_data);
					if (0 == err_code)
					{
						hvx_params.handle = p_log_info->log_data_handle.value_handle;
						hvx_len = PAVLOK_LOG_ENTRY_SIZE;
					}
					else
					{
						log_info_timers_stop();
					}
        }
        else  if (LOG_CMD_STOP_READ == cmd)
				{
					log_info_timers_stop();
				}
        else
        {
          APP_ERROR_HANDLER(LOG_CMD_LAST);
        }
				
				// encode to little endiam
        if (0 != hvx_len)
        {
       	  uint8_t * encoded_info = pavlok_common_get_encode_buffer();  // get common buffer
        	pavlok_encode((uint8_t *)&hvx_data, encoded_info, hvx_len);
          hvx_params.p_data = encoded_info;
					hvx_params.type = BLE_GATT_HVX_NOTIFICATION;
        }

        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;

				// send to the client
        err_code = sd_ble_gatts_hvx(p_log_info->conn_handle, &hvx_params);
        if (NRF_SUCCESS !=  err_code)
				{
#ifdef LOG_TIMERS
						log_info_timers_stop();
#endif
					//PAVLOK_LOG_ENTRY(EVT_SFW, LOG_C, __LINE__);
				}
    }

    return err_code;
}

/**@brief Function for handling the Write event.
 *
 * @param[in]   p_pavlok       Heart Rate Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_evt_t * p_ble_evt)
{
	ble_gatts_evt_write_t	*	p_evt_write	= &p_ble_evt->evt.gatts_evt.params.write;

	(void)memset(&log_status, 0, sizeof(log_status));
		
	if (CCCD_MSG_SIZE == p_evt_write->len)
	{
		update_cccd_enabled(p_ble_evt);
	}

	if ((p_evt_write->handle == p_log_info->log_cmd_handle.value_handle)
			&& (BLE_SERVICE_ENABLED == p_log_info->char_enable[LOG_CHAR_CMD]))
	{
		if  (LOG_CMD_LAST > p_evt_write->data[0])
		{
			log_status.current_command 	= (eLogCmd_t)p_evt_write->data[0];
      log_info_timers_start();			
		}
	}
}

void log_info_on_ble_evt(ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
        	p_log_info->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
           break;

        case BLE_GAP_EVT_DISCONNECTED:
        	p_log_info->conn_handle = BLE_CONN_HANDLE_INVALID;
#ifdef LOG_TIMERS
					log_info_timers_stop();
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

/**@brief Function for adding the Heart Rate Measurement characteristic.
 *
 * @param[in]   p_pavlok        Heart Rate Service structure.
 * @param[in]   p_pavlok_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t log_info_cmd_char_add(void)
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
    char_md.char_props.write   			= 1;
    char_md.char_props.read   			= 1;
    char_md.char_props.notify   		= 1;
    char_md.p_char_user_desc            = log_cmd_descriptor;
    char_md.char_user_desc_size         = strlen((char *)log_cmd_descriptor);
    char_md.char_user_desc_max_size     = strlen((char *)log_cmd_descriptor);

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

    BLE_UUID_BLE_ASSIGN(service_uuid, eLOG_CHAR_CMD);
    	attr_char_value.p_uuid              = &service_uuid;
    	attr_char_value.p_attr_md           = &attr_md;
    	attr_char_value.init_len            = PLOK_LOG_CMD_LENGTH;
    	attr_char_value.init_offs           = 0;
    	attr_char_value.max_len             = PLOK_LOG_CMD_LENGTH;
    	attr_char_value.p_value             = (uint8_t *)&p_log_info->info_cmd;

    	err_code = sd_ble_gatts_characteristic_add(p_log_info->service_handle,
    			&char_md,
				&attr_char_value,
				&p_log_info->log_cmd_handle);

    return err_code;
}


/**@brief Function for adding the Heart Rate Measurement characteristic.
 *
 * @param[in]   p_pavlok        Heart Rate Service structure.
 * @param[in]   p_pavlok_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t log_info_data_char_add(void)
{
	ble_uuid_t          	service_uuid;
    ble_gatts_char_md_t 	char_md;
    uint32_t				err_code;
         ble_gatts_attr_t    	attr_char_value;
        ble_gatts_attr_md_t 	attr_md;
  ble_gatts_char_pf_t   presentation;

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
    char_md.char_props.read   			= 1;
    char_md.char_props.notify   		= 1;
    char_md.p_char_user_desc            = log_data_descriptor;
    char_md.char_user_desc_size         = strlen((char *)log_data_descriptor);
    char_md.char_user_desc_max_size     = strlen((char *)log_data_descriptor);

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

    BLE_UUID_BLE_ASSIGN(service_uuid, eLOG_CHAR_DATA);
    	attr_char_value.p_uuid              = &service_uuid;
    	attr_char_value.p_attr_md           = &attr_md;
    	attr_char_value.init_len            = PAVLOK_LOG_ENTRY_SIZE;
    	attr_char_value.init_offs           = 0;
    	attr_char_value.max_len             = PAVLOK_LOG_ENTRY_SIZE;
    	attr_char_value.p_value             = (uint8_t *)&log_data_descriptor;

    	err_code = sd_ble_gatts_characteristic_add(p_log_info->service_handle,
    			&char_md,
				&attr_char_value,
				&p_log_info->log_data_handle);

    return err_code;
}

uint32_t log_info_init()
{
	ble_uuid_t          service_uuid;
  uint32_t            err_code      = NRF_ERROR_NO_MEM;
  uint8_t             address_type = 0;

    // Initialize service structure
    p_log_info->conn_handle                = BLE_CONN_HANDLE_INVALID;

    (void)memset(&m_log_info, 0, sizeof(m_log_info));
    (void)memset(&m_log_info_init, 0, sizeof(m_log_info_init));
    (void)memset(&service_uuid, 0, sizeof(service_uuid));

  err_code = sd_ble_uuid_vs_add(&base_uuid128, &address_type);
  if (err_code != NRF_SUCCESS)
  {
    return err_code;
  }
  
  service_uuid.uuid = eLOG_SVC_UUID;
  service_uuid.type = address_type;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                &service_uuid,
                &p_log_info->service_handle);

    if (NRF_SUCCESS == err_code)
    {
        err_code = log_info_cmd_char_add();
    }

    if (NRF_SUCCESS == err_code)
    {
        err_code = log_info_data_char_add();
    }

		log_info_timers_init();
		
    return err_code;
}


/** @} */
