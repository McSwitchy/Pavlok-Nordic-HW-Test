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
// TODO verify and send error/success from service updates back to phone
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
#include "ota.h"
#include "service_task.h"
#include "logger.h"
#include "debug.h"

/*------------------------------------------------------------------------
**	@brief	Global Data
**------------------------------------------------------------------------
*/
#define PLOK_APP_DESC			"Start OTA"
#define PLOK_FW_DESC					"Send FW"

/*------------------------------------------------------------------------
**	@brief	Global Extern(s)
**------------------------------------------------------------------------
*/


/*------------------------------------------------------------------------
**	@brief	Static Data
**------------------------------------------------------------------------
*/

static uint8_t * plck_app_descriptor 	= (uint8_t *)PLOK_APP_DESC;
static uint8_t * plck_fw_descriptor 			= (uint8_t *)PLOK_FW_DESC;

static ota_t						m_ota;
static ota_init_t				m_ota_init;
static ota_t * p_ota = &m_ota;
static sCfg_service_t	*	p_ota_info;
#undef APP_TIMERS
//#define APP_TIMERS (1)
#ifdef APP_TIMERS
#define OTA_SEND_INTERVAL             (50)                                       /**< Heart rate measurement interval (ms). */

static TimerHandle_t                m_ota_timer;     /**< Definition of heart rate timer. */
static void ota_timers_start(void);
static void ota_timers_stop(void);
#endif

static uint32_t ota_send(eOTA_CHAR_LIST char_type);

/********************************************************SAM START*****************************************************************************/
#include "nrf_log.h"
#include <string.h>
#include "ble_hci.h"
#include "sdk_macros.h"
#include "ble_srv_common.h"
#include "nrf_nvic.h"
#include "nrf_sdm.h"
#include "nrf_soc.h"

#define MAX_CTRL_POINT_RESP_PARAM_LEN   3
#define IRQ_ENABLED                     0x01                        /**< Field that identifies if an interrupt is enabled. */
#define MAX_NUMBER_INTERRUPTS           32                          /**< Maximum number of interrupts available. */
#define BOOTLOADER_DFU_START            0xB1

static void interrupts_disable(void)
{
    uint32_t interrupt_setting_mask;
    uint32_t irq;

    // Fetch the current interrupt settings.
    interrupt_setting_mask = NVIC->ISER[0];

    // Loop from interrupt 0 for disabling of all interrupts.
    for (irq = 0; irq < MAX_NUMBER_INTERRUPTS; irq++)
    {
        if (interrupt_setting_mask & (IRQ_ENABLED << irq))
        {
            // The interrupt was enabled, hence disable it.
            NVIC_DisableIRQ((IRQn_Type)irq);
        }
    }
}

static uint32_t bootloader_start(void)
{
    uint32_t err_code;

    err_code = sd_power_gpregret_clr(0, 0xffffffff);
    VERIFY_SUCCESS(err_code);

    err_code = sd_power_gpregret_set(0, BOOTLOADER_DFU_START);
    VERIFY_SUCCESS(err_code);

    err_code = sd_softdevice_disable();
    VERIFY_SUCCESS(err_code);

    err_code = sd_softdevice_vector_table_base_set(NRF_UICR->NRFFW[0]);
    VERIFY_SUCCESS(err_code);

    NVIC_ClearPendingIRQ(SWI2_IRQn);
    interrupts_disable();

    NVIC_SystemReset();
    return NRF_SUCCESS;
}
/********************************************************SAM END******************************************************************************/

/*------------------------------------------------------------------------
**	@brief	Static Forward References
**------------------------------------------------------------------------
*/

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


#ifdef APP_TIMERS
/**@brief Function for handling the Heart rate measurement timer time-out.
 *
 * @details This function will be called each time the heart rate measurement timer exotaes.
 *          It will exclude RR Interval data from every third measurement.
 *
 * @param[in] xTimer Handler to the timer that called this function.
 *                   You may get identifier given to the function xTimerCreate using pvTimerGetTimerID.
 */
static void ota_timeout_handle(TimerHandle_t xTimer)
{
  UNUSED_PARAMETER(xTimer);

  if (ota_send_index <= OTA_CHAR_LAST)
  {
	  (void)ota_send(ota_send_index);
	  ota_send_index += 1;
  }
  else
  {
	  // stop the timer
	  ota_timers_stop();
	  ota_send_index = OTA_CHAR_LAST;
  }
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
void ota_timers_init(void)
{
	pavlok_timers_init("OTA", &m_ota_timer, OTA_SEND_INTERVAL, ota_timeout_handle);
}


/**@brief Function for starting application timers.
 */
void ota_timers_start(void)
{
  if (pdFALSE == xTimerIsTimerActive(m_ota_timer))
  {
    // Start application timers.
    if(pdPASS != xTimerStart(m_ota_timer, OSTIMER_WAIT_FOR_QUEUE))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
  }

}

void ota_timers_stop(void)
{
    if(pdPASS != xTimerStop(m_ota_timer , OSTIMER_WAIT_FOR_QUEUE))
    {
        ASSERT(0==1);
    }
}
#endif


static void update_cccd_enabled(ble_evt_t * p_ble_evt)
{
	ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

	if (p_evt_write->handle == p_ota->app_handle.cccd_handle)
	{
		p_ota->char_enable[OTA_CHAR_APP] = p_evt_write->data[0];
	}
	else if (p_evt_write->handle == p_ota->fw_handle.cccd_handle)
	{
		p_ota->char_enable[OTA_CHAR_FW] = p_evt_write->data[0];
	}
	else
	{
		ASSERT(0==1);
	}
}


static uint32_t ota_send(eOTA_CHAR_LIST char_type)
{
    uint32_t err_code	=	NRF_ERROR_INVALID_PARAM;

    if (p_ota->conn_handle != BLE_CONN_HANDLE_INVALID)
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

        if (OTA_CHAR_APP == char_type)
        {
        	hvx_data = (uint8_t *)&p_ota_info->motor_value;
            hvx_params.handle = p_ota->app_handle.value_handle;
            hvx_len = 1;
        }
        else if (OTA_CHAR_FW == char_type)
        {
        	hvx_data = (uint8_t *)&p_ota_info->piezo_value;
            hvx_params.handle = p_ota->fw_handle.value_handle;
            hvx_len = 1;
        }
        else
        {
        	ASSERT(OTA_CHAR_LAST <= char_type);
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

        err_code = sd_ble_gatts_hvx(p_ota->conn_handle, &hvx_params);
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

		if ((p_evt_write->handle == p_ota->app_handle.value_handle)
        && (true == p_ota->char_enable[OTA_CHAR_APP]))
		{
/********************************************************SAM START****************************************************************************/			
//			SAM_S Put the start OTA here
        (void)bootloader_start();
/********************************************************SAM END******************************************************************************/
		}
		else if ((p_evt_write->handle == p_ota->fw_handle.value_handle)
             && (true == p_ota->char_enable[OTA_CHAR_FW]))
 		{
      // SAM_S put the fw load dere
			ota_send(OTA_CHAR_FW);
		} 
}

void ota_on_ble_evt(ble_evt_t * p_ble_evt)
{
  	switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
        	p_ota->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
           break;

        case BLE_GAP_EVT_DISCONNECTED:
        	p_ota->conn_handle = BLE_CONN_HANDLE_INVALID;
#ifdef APP_TIMERS
      ota_timers_stop();
#endif
            break;

        case BLE_GATTS_EVT_WRITE:
#ifdef APP_TIMERS
      ota_timers_start();
#endif
//					DEBUGS_APP_100("on_write");
          on_write(p_ble_evt);
       break;

        default:
//					DEBUGI_APP_100("default", p_ble_evt->header.evt_id);
            break;
    }
}

static uint32_t ota_app_char_add(void)
{
	ble_uuid_t          	service_uuid;
  ble_gatts_char_md_t 	char_md;
  ble_gatts_attr_t    	attr_char_value;
  ble_gatts_attr_md_t 	attr_md;
  ble_gatts_char_pf_t   presentation;
  ret_code_t				    err_code;

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
	char_md.char_props.write   					= 1;
	char_md.char_props.read   					= 1;
	char_md.char_props.notify   				= 1;
	char_md.p_char_user_desc            = plck_app_descriptor;
	char_md.char_user_desc_size         = strlen((char *)plck_app_descriptor);
	char_md.char_user_desc_max_size     = strlen((char *)plck_app_descriptor);
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

	BLE_UUID_BLE_ASSIGN(service_uuid, eOTA_CHAR_APP_UUID);
	attr_char_value.p_uuid              = &service_uuid;
	attr_char_value.p_attr_md           = &attr_md;
	attr_char_value.init_len            = PLOK_VALUE_LENGTH_TIME;
	attr_char_value.init_offs           = 0;
	attr_char_value.max_len             = PLOK_VALUE_LENGTH_TIME;
	attr_char_value.p_value             = NULL;

	err_code = sd_ble_gatts_characteristic_add( p_ota->service_handle,
                                              &char_md,
                                              &attr_char_value,
                                              &p_ota->app_handle);

	return err_code;
}

static uint32_t ota_fw_char_add(void)
{
	ble_uuid_t          	service_uuid;
  ble_gatts_char_md_t 	char_md;
  ble_gatts_attr_t    	attr_char_value;
  ble_gatts_attr_md_t 	attr_md;
  ble_gatts_char_pf_t   presentation;
  ret_code_t				    err_code;

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
	char_md.char_props.write   					= 1;
	char_md.char_props.read   					= 1;
	char_md.char_props.notify   				= 1;
	char_md.p_char_user_desc            = plck_fw_descriptor;
	char_md.char_user_desc_size         = strlen((char *)plck_fw_descriptor);
	char_md.char_user_desc_max_size     = strlen((char *)plck_fw_descriptor);
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

	BLE_UUID_BLE_ASSIGN(service_uuid, eOTA_CHAR_FIRMWARE_UUID);
	attr_char_value.p_uuid              = &service_uuid;
	attr_char_value.p_attr_md           = &attr_md;
	attr_char_value.init_len            = PLOK_VALUE_LENGTH_TIME;
	attr_char_value.init_offs           = 0;
	attr_char_value.max_len             = PLOK_VALUE_LENGTH_TIME;
	attr_char_value.p_value             = NULL;

	err_code = sd_ble_gatts_characteristic_add( p_ota->service_handle,
                                              &char_md,
                                              &attr_char_value,
                                              &p_ota->app_handle);

	return err_code;
}

uint32_t ota_service_init(void)
{
	ble_uuid_t          service_uuid;
  uint8_t             address_type = 0;
  
    uint32_t   err_code = NRF_ERROR_NO_MEM;

    // Initialize service structure
    p_ota->conn_handle                = BLE_CONN_HANDLE_INVALID;

    (void)memset(&m_ota, 0, sizeof(m_ota));
    (void)memset(&m_ota_init, 0, sizeof(m_ota_init));
    (void)memset(&service_uuid, 0, sizeof(service_uuid));

		p_ota_info = (sCfg_service_t *)pavlok_get_p_service_info(SI_OTA);

  err_code = sd_ble_uuid_vs_add(&base_uuid128, &address_type);
  if (err_code != NRF_SUCCESS)
  {
    return err_code;
  }
  
  service_uuid.uuid = eOTA_SVC_UUID;
  service_uuid.type = address_type;

     err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &service_uuid,
                                        &p_ota->service_handle);
    if (NRF_SUCCESS == err_code)
    {
        err_code = ota_app_char_add();
    }

    if (NRF_SUCCESS == err_code)
    {
        err_code = ota_fw_char_add();
    }
    
#ifdef APP_TIMERS
				ota_timers_init();
#endif
    return err_code;
}


/** @} */
