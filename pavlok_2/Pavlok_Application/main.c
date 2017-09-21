/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
// Board/nrf6310/ble/ble_app_hrs_rtx/main.c
/**
 *
 * @brief Heart Rate Service Sample Application with RTX main file.
 *
 * This file contains the source code for a sample application using RTX and the
 * Heart Rate service (and also Battery and Device Information services).
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "SEGGER_RTT.h"
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_bas.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "boards.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "fds.h"
#include "fstorage.h"
#include "ble_conn_state.h"
#include "nrf_drv_clock.h"
#include "nrf_ble_gatt.h"
#include "mem_manager.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

/* PAVLOK Includes */
#include "debug.h"
#include "pavlok_common.h"
#include "training.h"
#include "accel_mag.h"
#include "notification.h"
#include "configuration.h"
#include "appsvc.h"
#include "solicited.h"
#include "log_info.h"
#include "leds.h"
#include "i2c.h"
#include "pwm.h"
#include "zap.h"
#include "spi.h"
//#include "serial_flash_fsm.h"
#include "serial_flash.h"
#include "gyro.h"

#include "app_uart.h"
#include "app_util_platform.h"

// NEW UUID 156eb100-a300-4fea-897b-86f698d74461 

#include "logger.h"
#include "service_task.h"
#include "application_task.h"
#include "serial_flash.h"
#include "habit.h"

#if 0

static uint16_t  m_conn_handle = BLE_CONN_HANDLE_INVALID;   /**< Handle of the current connection. */
static nrf_ble_gatt_t m_gatt;                               /**< GATT module instance. */
static SemaphoreHandle_t m_ble_event_ready;                 /**< Semaphore raised if there is a new event to be processed in the BLE thread. */
static TaskHandle_t m_ble_stack_thread;                     /**< Definition of BLE stack thread. */
static TaskHandle_t m_logger_thread;                        /**< Definition of Logger thread. */
static TimerHandle_t m_button_timer;     /**< Definition of heart rate timer. */
static uint32_t	 m_current_tick = 0;
static uint32_t	 m_button_time = 0;
static 		ble_uuid_t m_adv_uuids[]  = {eSOLICITED_SVC_UUID, BLE_UUID_TYPE_VENDOR_BEGIN};

static void advertising_start(bool erase_bonds);

#ifdef NRF_LOG_ENABLED 
int32_t   app_debugi_on = 5;
int32_t		app_debugs_on = 5;
int32_t		trace_debug_on = 1;
#endif


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
            NRF_LOG_INFO("Connected to a previously bonded device.\r\n");
        } break;

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            NRF_LOG_INFO("Connection secured: role: %d, conn_handle: 0x%x, procedure: %d.\r\n",
                         ble_conn_state_role(p_evt->conn_handle),
                         p_evt->conn_handle,
                         p_evt->params.conn_sec_succeeded.procedure);
        } break;

        case PM_EVT_CONN_SEC_FAILED:
        {
            /* Often, when securing fails, it shouldn't be restarted, for security reasons.
             * Other times, it can be restarted directly.
             * Sometimes it can be restarted, but only after changing some Security Parameters.
             * Sometimes, it cannot be restarted until the link is disconnected and reconnected.
             * Sometimes it is impossible, to secure the link, or the peer device does not support it.
             * How to handle this error is highly application dependent. */
        } break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break;

        case PM_EVT_STORAGE_FULL:
        {
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_BUSY || err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }
            else
            {
                APP_ERROR_CHECK(err_code);
            }
        } break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        {
            advertising_start(false);
        } break;

        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
        {
            // The local database has likely changed, send service changed indications.
            pm_local_database_has_changed();
        } break;

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
        } break;

        case PM_EVT_PEER_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
        } break;

        case PM_EVT_PEERS_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
        } break;

        case PM_EVT_ERROR_UNEXPECTED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
        } break;

        case PM_EVT_CONN_SEC_START:
        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        case PM_EVT_PEER_DELETE_SUCCEEDED:
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }
}

static uint32_t	button_press_time(void)
{
  BaseType_t xReturn = pdFALSE;

  xReturn = xTimerIsTimerActive(m_button_timer);
  if (pdTRUE == xReturn)
  {
      if (pdPASS != xTimerStop(m_button_timer, OSTIMER_WAIT_FOR_QUEUE))
      {
          APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
      }
   }
  
  return (m_current_tick * PAVLOK_BUTTON_TICK_TIME);
}

static void button_timer_handler(TimerHandle_t xTimer)
{
  UNUSED_PARAMETER(xTimer);
  m_current_tick += 1;
}

static void button_press_start(void)
{
    BaseType_t xReturn = pdFALSE;
  // restarts the buttonTickCount to zero in the next timer interrupt

  m_current_tick = 0;

  // start the button timer if not already started
  xReturn = xTimerIsTimerActive( m_button_timer );
  if( pdFALSE == xReturn )
  {
    if(pdPASS != xTimerStart(m_button_timer, OSTIMER_WAIT_FOR_QUEUE))
    {
        ASSERT(0 == 1);
    }
  }
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    m_button_timer = xTimerCreate("BTN",
								  PAVLOK_BUTTON_TICK_TIME,
                                      pdTRUE,
                                      NULL,
                                      button_timer_handler);
    /* Error checking */
    if (NULL == m_button_timer)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
}



/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                  err_code = NRF_ERROR_FORBIDDEN;
    sPavlokServiceInfo_t    * service_info = pavlok_get_configuration();
    uint8_t * device_addr =   (uint8_t *)&service_info->device_addr;
    ble_gap_conn_params_t     gap_conn_params = {0};
    ble_gap_conn_sec_mode_t   sec_mode = {0};
    uint8_t                   ble_addr[13] = {0x50, 0x61, 0x76, 0x6C, 0x6F, 0x6B, 0x2D, 0x32, 0x2D, 0, 0, 0, 0};
  
    err_code = sd_ble_gap_addr_get(&service_info->device_addr);
    if (NRF_SUCCESS == err_code)
    {
      uint32_t * p_ble_addr = (uint32_t *)&device_addr[5];
      (void)sprintf((char *)&ble_addr[9], "%X", * p_ble_addr);
    }
    else
    {
      APP_ERROR_HANDLER(err_code);
    }
    
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)ble_addr,
                                          strlen((const char *)ble_addr));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_WATCH_SPORTS_WATCH);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Heart Rate, Battery and Device Information services.
 */
static void services_init(void)
{
  ret_code_t     err_code;  

#ifdef PAVLOK_SOL_ENABLED
  err_code = solicited_service_init();
  APP_ERROR_CHECK(err_code);
#endif  
#ifdef PAVLOK_CFG_ENABLED
  err_code = cfg_service_init();
  APP_ERROR_CHECK(err_code);
#endif
#ifdef PAVLOK_NOTIFY_ENABLED
  err_code = notification_service_init();
  APP_ERROR_CHECK(err_code);
#endif

#ifdef PAVLOK_LOG_ENABLED
  err_code = log_info_init();
  APP_ERROR_CHECK(err_code);
#endif
#ifdef PAVLOK_TRAIN_ENABLED
  err_code = training_service_init();
  APP_ERROR_CHECK(err_code);
#endif

#ifdef PAVLOK_APPSVC_ENABLED
  err_code = appsvc_service_init();
  APP_ERROR_CHECK(err_code);
#endif

#ifdef PAVLOK_OTA_ENABLED
  err_code = ota_service_init();
  APP_ERROR_CHECK(err_code);
#endif

	err_code = ble_bas_init();
  APP_ERROR_CHECK(err_code);

  err_code = ble_dis_init();
  APP_ERROR_CHECK(err_code);

}

/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
    // Start application timers.
  if (pdPASS != xTimerStart(m_button_timer, OSTIMER_WAIT_FOR_QUEUE))
  {
      APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = get_train_service_data_cccd_handle();
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
// REMOVED    err_code = bsp_btn_ble_sleep_mode_prepare();
// REMOVED    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.\r\n");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;

        default:
            break;
    }
}


/**@brief Function for receiving the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected\r\n");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break; // BLE_GAP_EVT_CONNECTED

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected\r\n");
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(m_conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_EVT_USER_MEM_REQUEST

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    /** The Connection state module has to be fed BLE events in order to function correctly
     * Remember to call ble_conn_state_on_ble_evt before calling any ble_conns_state_* functions. */
    ble_conn_state_on_ble_evt(p_ble_evt);
    pm_on_ble_evt(p_ble_evt);
    ble_bas_on_ble_evt(p_ble_evt);

    ble_conn_params_on_ble_evt(p_ble_evt);
#ifdef REMOVED  
    bsp_btn_ble_on_ble_evt(p_ble_evt);
#endif  
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
    nrf_ble_gatt_on_ble_evt(&m_gatt, p_ble_evt);
  
  
#ifdef PAVLOK_SOL_ENABLED
  solicited_on_ble_evt(p_ble_evt);
#endif
  
#ifdef PAVLOK_CFG_ENABLED
   	cfg_on_ble_evt(p_ble_evt);
#endif
#ifdef PAVLOK_NOTIFY_ENABLED
   	notification_on_ble_evt(p_ble_evt);
#endif
#ifdef PAVLOK_LOG_ENABLED
   	log_info_on_ble_evt(p_ble_evt);
#endif
#ifdef PAVLOK_TRAIN_ENABLED
   	training_on_ble_evt(p_ble_evt);
#endif
#ifdef PAVLOK_APPSVC_ENABLED
   	appsvc_on_ble_evt(p_ble_evt);
#endif
#ifdef PAVLOK_OTAL_ENABLED
  ota_on_ble_evt(p_ble_evt);
#endif
  
}

/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    // Dispatch the system event to the fstorage module, where it will be
    // dispatched to the Flash Data Storage (FDS) module.
    fs_sys_event_handler(sys_evt);

    // Dispatch to the Advertising module last, since it will check if there are any
    // pending flash operations in fstorage. Let fstorage process system events first,
    // so that it can report correctly to the Advertising module.
    ble_advertising_on_sys_evt(sys_evt);
}


/**
 * @brief Event handler for new BLE events
 *
 * This function is called from the SoftDevice handler.
 * It is called from interrupt level.
 *
 * @return The returned value is checked in the softdevice_handler module,
 *         using the APP_ERROR_CHECK macro.
 */
static uint32_t ble_new_event_handler(void)
{
    BaseType_t yield_req = pdFALSE;

    // The returned value may be safely ignored, if error is returned it only means that
    // the semaphore is already given (raised).
    UNUSED_VARIABLE(xSemaphoreGiveFromISR(m_ble_event_ready, &yield_req));
    portYIELD_FROM_ISR(yield_req);
    return NRF_SUCCESS;
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, ble_new_event_handler);

    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = softdevice_app_ram_start_get(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Overwrite some of the default configurations for the BLE stack.
    ble_cfg_t ble_cfg;

    // Configure the number of custom UUIDS.
    memset(&ble_cfg, 0, sizeof(ble_cfg));
  
    // Configure the number of custom UUIDS.
    memset(&ble_cfg, 0, sizeof(ble_cfg));
    ble_cfg.common_cfg.vs_uuid_cfg.vs_uuid_count = 1;
    err_code = sd_ble_cfg_set(BLE_COMMON_CFG_VS_UUID, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);

    memset(&ble_cfg, 0, sizeof(ble_cfg));
    ble_cfg.gatts_cfg.attr_tab_size.attr_tab_size = 0xE00;
    err_code = sd_ble_cfg_set(BLE_GATTS_CFG_ATTR_TAB_SIZE, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);

    // Configure the maximum number of connections.
    memset(&ble_cfg, 0, sizeof(ble_cfg));
    ble_cfg.gap_cfg.role_count_cfg.periph_role_count  = BLE_GAP_ROLE_COUNT_PERIPH_DEFAULT;
    ble_cfg.gap_cfg.role_count_cfg.central_role_count = 0;
    ble_cfg.gap_cfg.role_count_cfg.central_sec_count  = 0;
    err_code = sd_ble_cfg_set(BLE_GAP_CFG_ROLE_COUNT, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = softdevice_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
    
    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);    
}

/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
      case PAVLOK_BUTTON_DOWN :
      {
        button_press_start();
      }
      break;

      case PAVLOK_BUTTON_UP :
      {
        uint32_t button_time = 0;
				m_button_time = button_press_time();

				// go and normalize the button time into pavlok increments
				button_time = pavlok_button_normalize(m_button_time);

        sCfg_service_t * p_cfg_info = pavlok_get_p_service_info(SI_CFG);
        uint16_t       * button_data = (uint16_t *)&p_cfg_info->button_value[2];
        * button_data = button_time;
        p_cfg_info->characteristic      = CFG_CHAR_BUTTON;
        if (false == (application_is_in_alarm_state()))
        {
          pavlok_set_dirty_bit(SI_CFG, &p_cfg_info->button_value);
        }
        else
        {
          application_check_button_press();
        }
      }
      break;

        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist();
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!\r\n");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t                err_code;
    ble_advdata_t             advdata;
    ble_advdata_t             scanrsp;
    ble_adv_modes_config_t    options;
  
                              
    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME ;
    advdata.include_appearance      = false;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids;
  
    memset(&options, 0, sizeof(options));
    options.ble_adv_fast_enabled  = true;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

// REMOVED    err_code = bsp_btn_ble_init(NULL, &startup_event);
// REMOVED    APP_ERROR_CHECK(err_code);

   *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


// only use after the softdevice is initialized
static void log_reset_reason(void)
{
  uint32_t reset_reason;
  sd_power_reset_reason_get(&reset_reason);

//  PAVLOK_LOG_ENTRY(EVT_RESET_REASON, reset_reason, __LINE__, __FILE__);
  sd_power_reset_reason_clr(0xFFFFFFFF);
}

/**@brief Thread for handling the Application's BLE Stack events.
 *
 * @details This thread is responsible for handling BLE Stack events sent from on_ble_evt().
 *
 * @param[in]   arg   Pointer used for passing some arbitrary information (context) from the
 *                    osThreadCreate() call to the thread.
 */

static void ble_stack_thread(void * arg)
{
    bool erase_bonds;

    UNUSED_PARAMETER(arg);

    // Initialize.
    timers_init();
    buttons_leds_init(&erase_bonds);
    log_reset_reason();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
    peer_manager_init();
  
    application_timers_start();

    advertising_start(erase_bonds);

    while (1)
    {
        /* Wait for event from SoftDevice */
        while (pdFALSE == xSemaphoreTake(m_ble_event_ready, portMAX_DELAY))
        {
            // Just wait again in the case when INCLUDE_vTaskSuspend is not enabled
        }

        // This function gets events from the SoftDevice and processes them by calling the function
        // registered by softdevice_ble_evt_handler_set during stack initialization.
        // In this code ble_evt_dispatch would be called for every event found.
        intern_softdevice_events_execute();
    }
}


/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
    }
    else
    {
        ret_code_t err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);
    }
}

#if NRF_LOG_ENABLED
/**@brief Thread for handling the logger.
 *
 * @details This thread is responsible for processing log entries if logs are deferred.
 *          Thread flushes all log entries and suspends. It is resumed by idle task hook.
 *
 * @param[in]   arg   Pointer used for passing some arbitrary information (context) from the
 *                    osThreadCreate() call to the thread.
 */
static void logger_thread(void * arg)
{
    UNUSED_PARAMETER(arg);

    while(1)
    {
        NRF_LOG_FLUSH();

        vTaskSuspend(NULL); // Suspend myself
    }
}
#endif //NRF_LOG_ENABLED

#endif

/**@brief Function for initializing the clock.
 */
static void clock_init(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
}

#if 1
void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName )
{
  APP_ERROR_HANDLER(4);
}

/**@brief A function which is hooked to idle task.
 * @note Idle hook must be enabled in FreeRTOS configuration (configUSE_IDLE_HOOK).
 */
void vApplicationIdleHook( void )
{
     
}

/**@brief Function for application main entry.
 */
#endif

static bool reset_test = false;
RTC_TIME_STRUCT_T setTime = {0};
RTC_TIME_STRUCT_T getTime = {0};
// NRF_LOG_ENABLED NRF_LOG_USES_RTT=1 
int main(void)
{
    BaseType_t xReturn = pdFAIL;
    ret_code_t err_code = 0;

  clock_init();

#if 0
    // Do not start any interrupt that uses system functions before system initialisation.
    // The best solution is to start the OS before any other initalisation.

    // Init a semaphore for the BLE thread.
    m_ble_event_ready = xSemaphoreCreateBinary();
    if (NULL == m_ble_event_ready)
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

    log_init();

    // Start execution.
    if (pdPASS != xTaskCreate(ble_stack_thread, "BLE", 256, NULL, 2, &m_ble_stack_thread))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

#if NRF_LOG_ENABLED
    // Start execution.
    if (pdPASS != xTaskCreate(logger_thread, "LOGGER", 256, NULL, 1, &m_logger_thread))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
#endif //NRF_LOG_ENABLED


#ifdef INCLUDE_SVC_TASK
    xReturn = service_start_task();
    if(pdPASS != xReturn)
    {
        ASSERT(pdPASS == xReturn);
    }
#endif

#ifdef INCLUDE_APP_TASK
    xReturn = application_start_task();
    if(pdPASS != xReturn)
    {
        ASSERT(pdPASS == xReturn);
    }
#endif

#ifdef INCLUDE_LOGGER_TASK
    xReturn = logger_start_task();
    if(pdPASS != xReturn)
    {
        ASSERT(pdPASS == xReturn);
    }
#endif

#ifdef INCLUDE_LOGGER_TASK
    xReturn = serial_flash_fsm_start_task();
    if(pdPASS != xReturn)
    {
        ASSERT(pdPASS == xReturn);
    }
    // TODO: Check if returned mutex is null!
#endif

#ifdef INCLUDE_HABIT_TASK
    xReturn = habit_start_task();
    if(pdPASS != xReturn)
    {
        ASSERT(pdPASS == xReturn);
    }
#endif

	/**	--------------------------------------------------------------------
	**	It is possible to run w/o configuration straight from an app
	**	so we must init peripherals here
	**	--------------------------------------------------------------------
	*/

    
  init_leds();
	i2c_devices_init();
	spi2_devices_init();
	pwm_piezo_init();
	pwm_motor_init();
	pwm_zap_init();
	zap_gpio_init();

	vbatt_measure_init();
   if (false == reset_test)
   {
     err_code = (uint32_t)rtc_init();
     reset_test = true;
   }
  APP_ERROR_CHECK(err_code);
  err_code = nrf_mem_init();
  APP_ERROR_CHECK(err_code);
  // looger list must be initialized here and after mem_manager so that the reset reason can get written to flash
  logger_list_init();
  pavlok_vusb_init();
  pavlok_set_default_configuration();
   
	accelerometer_init_pulse();

//#define TEST_FLASH 1
#ifdef TEST_FLASH    
  flash_sectors_erase_test();
  nrf_delay_ms(50);
 // flash_sectors_write_test();
//  nrf_delay_ms(50);
#endif  
#undef TEST_FLASH
#ifdef TBD
{
  #include "crc16.h"
  uint16_t  current_crc = 0;
  static uint16_t crc_output[4096];
  
  (void)memset(g_pavlok_scratch_pad, 0xA5, 4096);
  for (int32_t counter = 0; counter < 4096; counter++)
  {
    current_crc = crc16_compute(&g_pavlok_scratch_pad[counter], 1, &current_crc);
    SEGGER_RTT_printf(0, "count = %d CRC = %04X\r\n", counter, current_crc);
    crc_output[counter] = current_crc;
    nrf_delay_ms(50);
  }
    memset(crc_output, 0, sizeof(crc_output));
}
#endif

   /* Activate deep sleep mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    // Start FreeRTOS scheduler.
    vTaskStartScheduler();

    while (true)
    {
        APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
    }
#endif
#define ACCEL_ERROR_SHIFT_MASK   (uint32_t)(1<<0)
#define GYRO_ERROR_SHIFT_MASK    (uint32_t)(1<<1)     
#define RTC_ERROR_SHIFT_MASK     (uint32_t)(1<<2)      
#define FLASH_ERROR_SHIFT_MASK   (uint32_t)(1<<3)
#define CHARGE_STATUS_SHIFT_MASK (uint32_t)(1<<4)    

#define GYRO_RESET_PIN      (2)
#define ACC_RESET_PIN       (26)    
#define CHG_PIN             (25)    
  
  init_leds();
	i2c_devices_init();
	spi2_devices_init();
  pwm_piezo_init();
	pwm_motor_init();
  
  nrf_gpio_cfg_input(CHG_PIN, NRF_GPIO_PIN_NOPULL);    
  set_led(LED_RED);
  set_led(LED_GREEN);
  
  nrf_gpio_cfg_output(GYRO_RESET_PIN);
  nrf_gpio_cfg_output(ACC_RESET_PIN);  
  
  nrf_gpio_pin_clear(ACC_RESET_PIN);  
  nrf_gpio_pin_set(GYRO_RESET_PIN);
  nrf_delay_ms(100);  
    
  uint8_t ret = 0;
  uint32_t leds = 0;  
  uint8_t errorFound = 0;
    
  // Turn on all LEDs. They will turn off as each test passes.  
  set_led(LED1);
  set_led(LED2);
  set_led(LED3);
  set_led(LED4);
  set_led(LED5);
  
  pwm_pizeo_update_duty_and_frequency(50, 2000);
  pwm_motor_update_duty_and_frequency(70, 4000);
  nrf_delay_ms(2000);
  pwm_pizeo_update_duty_and_frequency(0, 2000);
  pwm_motor_update_duty_and_frequency(0, 2000);

  ret = accelerometer_whoami();
  if (ret) {
    clear_led(LED1);
  } else {
    leds |= ACCEL_ERROR_SHIFT_MASK;
    //set_led(LED1);
    errorFound = 1;
  }
  
  ret = gyro_whoami();
  if (ret) {
    clear_led(LED2);
  } else {
    leds |= GYRO_ERROR_SHIFT_MASK;
    //set_led(LED2);
    errorFound = 1;
  }
  
  ret = rtc_init();
  if (ret) {
    rtc_set_time(&setTime);
    nrf_delay_ms(1010);
    rtc_get_time(&getTime);
    if (getTime.seconds >= 1) {  
      clear_led(LED3);
    }
  } else {
    leds |= RTC_ERROR_SHIFT_MASK;
    //set_led(LED3);
    errorFound = 1;
  }
  
  ret = demo_flash_read_write_read();  
  if (ret) {
    clear_led(LED4);
  } else {
    leds |= FLASH_ERROR_SHIFT_MASK;
    //set_led(LED4);
    errorFound = 1;
  }
  
  if (!errorFound) {
    clear_led(LED_RED);
  } else {
    clear_led(LED_GREEN);
  }
  
  // Check this last
  ret = nrf_gpio_pin_read(CHG_PIN);
  if (ret) {
    // Diode wants this LED to be on if charge is high
    leds |= CHARGE_STATUS_SHIFT_MASK;
    set_led(LED5);
  } else {
    //errorFound = 1;
    clear_led(LED5);
  }
  
  while(1);
}


