/** ----------------------------------------------------------------------
**
**	@file
**
**  @defgroup BATTER_SERVICE
**  @{
**  @ingroup PAVLOK_SERVICE
**  
**  @note See below included NOrdic description 
**  
**  @note Attention!
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

/** @file
 *
 * @defgroup ble_bas Battery Service
 * @{
 * @ingroup ble_sdk_srv
 * @brief Battery Service module.
 *
 * @details This module implements the Battery Service with the Battery Level characteristic.
 *          During initialization it adds the Battery Service and Battery Level characteristic
 *          to the BLE stack database. Optionally it can also add a Report Reference descriptor
 *          to the Battery Level characteristic (used when including the Battery Service in
 *          the HID service).
 *
 *          If specified, the module will support notification of the Battery Level characteristic
 *          through the ble_bas_battery_level_update() function.
 *          If an event handler is supplied by the application, the Battery Service will
 *          generate Battery Service events to the application.
 *
 * @note The application must propagate BLE stack events to the Battery Service module by calling
 *       ble_bas_on_ble_evt() from the @ref softdevice_handler callback.
 *
 * @note Attention!
 *  To maintain compliance with Nordic Semiconductor ASA Bluetooth profile
 *  qualification listings, this section of source code must not be modified.
 */

#ifndef BLE_BAS_H__
#define BLE_BAS_H__

/** ----------------------------------------------------------------------
**	@brief System Include(s)
**------------------------------------------------------------------------
*/
#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

/** ----------------------------------------------------------------------
**	@brief Project Include(s)
**  ----------------------------------------------------------------------
*/
#include "zap.h"

#ifdef __cplusplus
extern "C" {
#endif

/** ----------------------------------------------------------------------
**	@brief	Global Data
**  ----------------------------------------------------------------------
*/
typedef enum
{
	BATT_CHAR_STATUS,
	BATT_CHAR_POLL_RATE,
	BATT_CHAR_LAST

} eBATT_CHAR_LIST;

/**@brief Battery Service event type. */
typedef enum
{
    BLE_BAS_EVT_NOTIFICATION_ENABLED,                             /**< Battery value notification enabled event. */
    BLE_BAS_EVT_NOTIFICATION_DISABLED                             /**< Battery value notification disabled event. */
} ble_bas_evt_type_t;

/**@brief Battery Service event. */
typedef struct
{
    ble_bas_evt_type_t evt_type;                                  /**< Type of event. */
} ble_bas_evt_t;

// Forward declaration of the ble_bas_t type.
typedef struct ble_bas_s ble_bas_t;

/**@brief Battery Service event handler type. */
typedef void (*ble_bas_evt_handler_t) (ble_bas_t * p_bas, ble_bas_evt_t * p_evt);

/**@brief Battery Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_bas_evt_handler_t         evt_handler;                    /**< Event handler to be called for handling events in the Battery Service. */
    bool                          support_notification;           /**< TRUE if notification of Battery Level measurement is supported. */
    ble_srv_report_ref_t *        p_report_ref;                   /**< If not NULL, a Report Reference descriptor with the specified value will be added to the Battery Level characteristic */
    uint32_t                       initial_batt_level;             /**< Initial battery level */
    ble_srv_cccd_security_mode_t  battery_level_char_attr_md;     /**< Initial security level for battery characteristics attribute */
    ble_gap_conn_sec_mode_t       battery_level_report_read_perm; /**< Initial security level for battery report read attribute */
} ble_bas_init_t;

/**@brief Battery Service structure. This contains various status information for the service. */
struct ble_bas_s
{
    ble_bas_evt_handler_t         evt_handler;                    /**< Event handler to be called for handling events in the Battery Service. */
    uint16_t                      service_handle;                 /**< Handle of Battery Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t      battery_level_handles;          /**< Handles related to the Battery Level characteristic. */
    uint16_t                      report_ref_handle;              /**< Handle of the Report Reference descriptor. */
    uint32_t                      battery_level_last;             /**< Last Battery Level measurement passed to the Battery Service. */
    uint32_t                      battery_level_current;             /**< Last Battery Level measurement passed to the Battery Service. */
    uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    bool                          is_notification_supported;      /**< TRUE if notification of Battery Level is supported. */
};

/**@brief Function for initializing the Battery Service.
 *
 * @param[out]  p_bas       Battery Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_bas_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_bas_init(void);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Battery Service.
 *
 * @note For the requirements in the BAS specification to be fulfilled,
 *       ble_bas_battery_level_update() must be called upon reconnection if the
 *       battery level has changed while the service has been disconnected from a bonded
 *       client.
 *
 * @param[in]   p_bas      Battery Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_bas_on_ble_evt(ble_evt_t * p_ble_evt);

void	ble_start_enable_battery_voltage_test(void);
millivoltsBattery_t battery_level_update(void);
uint16_t return_battery_voltage(void);

#ifdef __cplusplus
}
#endif

#endif // BLE_BAS_H__

/** @} */
