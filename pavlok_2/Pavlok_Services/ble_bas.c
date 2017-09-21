/** ----------------------------------------------------------------------
**
**	@file			ble_bas.c
**
**  @brief battery Notification module.
**  
**  @details This module implements the pavlok battery service
**  
**  @note This file has been modified from the original to use the ADC
**				section of the nrf52 to monitor the battery on the device
**				The original file is @ SDK/components/ble/service/ble_bas.c
**  
**  ----------------------------------------------------------------------
*/
/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/* Attention!
*  To maintain compliance with Nordic Semiconductor ASA’s Bluetooth profile
*  qualification listings, this section of source code must not be modified.
*/
/** ----------------------------------------------------------------------
**	@brief System Include(s)
**------------------------------------------------------------------------
*/
#include "sdk_config.h"

#include <string.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "nrf_delay.h"

/** ----------------------------------------------------------------------
**	@brief Project Include(s)
**  ----------------------------------------------------------------------
*/
#include "pavlok_common.h"
#include "ble_bas.h"
#include "logger.h"
#include "zap.h"
#include "adc.h"
#include "debug.h"

/** ----------------------------------------------------------------------
**	@brief	Global Data
**  ----------------------------------------------------------------------
*/
#define INVALID_BATTERY_LEVEL 255
#define BATTERY_LEVEL_MEAS_INTERVAL     (180000)                         /**< Battery level measurement interval 3 min (ms). */
//#define BATTERY_LEVEL_MEAS_INTERVAL       (10000)                         /**< Battery level measurement interval 3 min (ms). */
#define MIN_BATTERY_LEVEL                81                               /**< Minimum simulated battery level. */
#define MAX_BATTERY_LEVEL                100                              /**< Maximum simulated battery level. */
#define BATTERY_LEVEL_INCREMENT          1                                /**< Increment between each simulated battery level measurement. */
#define BATTERY_VOLTAGE_MULTIPLIER			 (1.52)		
/** ----------------------------------------------------------------------
**	@brief	Static Data
**  ----------------------------------------------------------------------
*/
static ble_bas_t 					m_bas;
static ble_bas_init_t  		m_bas_init;
static ble_bas_t				*	p_bas = &m_bas;
static ble_bas_init_t  	*	p_bas_init;

#define BATT_TIMERS (1)
#ifdef BATT_TIMERS
static TimerHandle_t 		  m_battery_timer;        /**< Definition of battery timer. */


/** ----------------------------------------------------------------------
**
**	@fn		Function			ble_bas_battery_level_send
**
**	@brief	Description	This function reads the ADC and updates the 
**											battery data structure
**
**	@param [in]					ble_bas_t * p_bas - batter data structure
**	@param [in]					uint8_t battery_level - the transposed voltage
**
**	@param	[out]				None
**
**	@return						None
**
**	@retval
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
static void ble_bas_battery_level_send(TimerHandle_t xTimer)
{
  UNUSED_PARAMETER(xTimer);

	if (p_bas == NULL)
  {
    return;
  }
		
    ble_gatts_value_t gatts_value;
		
    if ((p_bas->battery_level_current != p_bas->battery_level_last)
        && (0 != p_bas->battery_level_current))
    {
        // Initialize value struct.
        memset(&gatts_value, 0, sizeof(gatts_value));

        gatts_value.len     = sizeof(uint16_t);
        gatts_value.offset  = 0;
        gatts_value.p_value = (uint8_t *)&p_bas->battery_level_current;

        p_bas->battery_level_last = p_bas->battery_level_current;

        // Send value if connected and notifying.
        if ((p_bas->conn_handle != BLE_CONN_HANDLE_INVALID) && p_bas->is_notification_supported)
        {
        	uint8_t * encoded_info = pavlok_common_get_encode_buffer();  // get common buffer
          ble_gatts_hvx_params_t hvx_params;

            memset(&hvx_params, 0, sizeof(hvx_params));

        	pavlok_encode((uint8_t *)p_bas->battery_level_current, encoded_info, 2);
            hvx_params.handle = p_bas->battery_level_handles.value_handle;
            hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset = gatts_value.offset;
            hvx_params.p_len  = &gatts_value.len;
            hvx_params.p_data = encoded_info;

            (void)sd_ble_gatts_hvx(p_bas->conn_handle, &hvx_params);
        }
    }

    return;
}

/** ----------------------------------------------------------------------
**
**	@fn		Function			battery_timer_init
**
**	@brief	Description	initialize the service timer
**
**	@param [in]					None
**
**	@param	[out]				None
**
**	@return							None
**
**	@retval
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
static void battery_timer_init(void)
{
  // Create timers.
  m_battery_timer = xTimerCreate("BATT",
								 BATTERY_LEVEL_MEAS_INTERVAL,
								 pdTRUE,
								 NULL,
								 ble_bas_battery_level_send);

  /* Error checking */
  if (NULL == m_battery_timer)
  {
		APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }
}


/** ----------------------------------------------------------------------
**
**	@fn		Function			battery_timers_start
**
**	@brief	Description	start the service timer
**
**	@param [in]					None
**
**	@param	[out]				None
**
**	@return							None
**
**	@retval
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
static void battery_timers_start(void)
{
    // Start application timers.
    if (pdPASS != xTimerStart(m_battery_timer, OSTIMER_WAIT_FOR_QUEUE))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
}

#endif
/** ----------------------------------------------------------------------
**	@brief	Static Forward References
**------------------------------------------------------------------------
*/
/** ----------------------------------------------------------------------
**
**	@fn		Function			battery_level_update
**
**	@brief	Description performing battery measurement and updating the 
**											Battery Level characteristic
**
**	@param [in]					None
**
**	@param	[out]				None
**
**	@return							None
**
**	@retval
**
**	@warn								Called from service_task to decrease timer usage
**
**  ----------------------------------------------------------------------
*/
millivoltsBattery_t battery_level_update(void)
{
	p_bas->battery_level_current = (uint16_t)getBatteryVoltage();
 //	PAVLOK_LOG_ENTRY(EVT_BATT_LEVEL, p_bas->battery_level_current, __LINE__, __FILE__);
  return p_bas->battery_level_current;
}

void	ble_set_enable_battery_voltage_test(void)
{
	/**	--------------------------------------------------------------------
	**	called from service task to start the adc read
	**	--------------------------------------------------------------------
	*/
	nrf_gpio_pin_set(VBATT_ENABLE);
}


uint16_t return_battery_voltage(void) 
{
  return p_bas->battery_level_current; 
}

/** ----------------------------------------------------------------------
**
**	@fn		Function			on_write
**
**	@brief	Description	service write function to act on the data received
**											from the BLE stack
**
**	@param [in]					ble_bas_t * p_bas - static pointer to battery data
**	@param [in]					ble_evt_t * p_ble_evt - BLE stack event
**
**	@param	[out]				None
**
**	@return							None
**
**	@retval
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
static void on_write(ble_evt_t * p_ble_evt)
{
    if (p_bas->is_notification_supported)
    {
        ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

        if ((p_evt_write->handle == p_bas->battery_level_handles.cccd_handle)
            && (p_evt_write->len == 2))
        {
            // CCCD written, call application event handler
            if (p_bas->evt_handler != NULL)
            {
                ble_bas_evt_t evt;

                if (ble_srv_is_notification_enabled(p_evt_write->data))
                {
                    evt.evt_type = BLE_BAS_EVT_NOTIFICATION_ENABLED;
                }
                else
                {
                    evt.evt_type = BLE_BAS_EVT_NOTIFICATION_DISABLED;
                }
            }
        }
    }
}

/** ----------------------------------------------------------------------
**
**	@fn		Function			ble_bas_on_ble_evt
**
**	@brief	Description	called from the ble event handler in main
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
void ble_bas_on_ble_evt(ble_evt_t * p_ble_evt)
{
    if ((p_ble_evt == NULL)
				|| (NULL == p_bas))
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            p_bas->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            p_bas->conn_handle = BLE_CONN_HANDLE_INVALID;
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
**	@fn		Function			battery_level_char_add
**
**	@brief	Description	start the command gatt characteristic
**
**	@param [in]					p_bas        Battery Service structure
**	@param [in]					p_bas_init   Information needed to initialize the service.
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
**	@note								TOD reduce by removing parameter like the rest of the servivces 
**
**  ----------------------------------------------------------------------
*/
static uint32_t battery_level_char_add(const ble_bas_init_t * p_bas_init)
{
    uint32_t            err_code = NRF_SUCCESS;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    uint8_t             initial_battery_level = 85;

    // Add Battery Level characteristic
    if (p_bas->is_notification_supported)
    {
        memset(&cccd_md, 0, sizeof(cccd_md));

        // According to BAS_SPEC_V10, the read operation on cccd should be possible without
        // authentication.
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&m_bas_init.battery_level_char_attr_md.cccd_write_perm);
        cccd_md.write_perm = p_bas_init->battery_level_char_attr_md.cccd_write_perm;
        cccd_md.vloc       = BLE_GATTS_VLOC_STACK;
    }

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_BATTERY_LEVEL_CHAR);

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&m_bas_init.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&m_bas_init.battery_level_char_attr_md.write_perm);
    attr_md.read_perm  = p_bas_init->battery_level_char_attr_md.read_perm;
    attr_md.write_perm = p_bas_init->battery_level_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    initial_battery_level = p_bas_init->initial_batt_level;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint16_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof(uint16_t);
    attr_char_value.p_value   = &initial_battery_level;

    err_code = sd_ble_gatts_characteristic_add(p_bas->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_bas->battery_level_handles);
    return err_code;
}

/** ----------------------------------------------------------------------
**
**	@fn		Function				ble_bas_init
**
**	@brief	Description		initialize the battery service
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
uint32_t ble_bas_init()
{
    uint32_t   err_code = NRF_ERROR_INVALID_PARAM;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_bas->evt_handler               = p_bas_init->evt_handler;
    p_bas->conn_handle               = BLE_CONN_HANDLE_INVALID;
    p_bas->is_notification_supported = p_bas_init->support_notification;
    p_bas->battery_level_last        = 0;

    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_BATTERY_SERVICE);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_bas->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add battery level characteristic
    err_code = battery_level_char_add(&m_bas_init);

		/**	------------------------------------------------------------------
		**	The adc is read and triggered in the service task as it is always
		**	running
		**	------------------------------------------------------------------
		*/
		nrf_gpio_cfg_output(VBATT_ENABLE);	
#ifdef BATT_TIMERS
    battery_timer_init();
    battery_timers_start();
#endif
    return err_code;
}


