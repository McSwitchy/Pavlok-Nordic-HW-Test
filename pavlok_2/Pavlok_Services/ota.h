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
#ifndef SERVICE_OTA_H_
#define SERVICE_OTA_H_

/*------------------------------------------------------------------------
**	@brief System Include(s)
**------------------------------------------------------------------------
*/
/*------------------------------------------------------------------------
**	@brief Project Include(s)
**------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------
**	@brief	Global Data
**------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------
**	@brief	Global Extern(s)
**------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------
**	@brief	Static Data
**------------------------------------------------------------------------
*/

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

/*------------------------------------------------------------------------
**	@brief System Include(s)
**------------------------------------------------------------------------
*/

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"


/*------------------------------------------------------------------------
**	@brief Project Include(s)
**------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------
**	@brief	Global Data
**------------------------------------------------------------------------
*/
typedef enum
{
  OTA_CHAR_APP,
  OTA_CHAR_FW,
  OTA_CHAR_LAST
} eOTA_CHAR_LIST;

// Forward declaration of the ota_t type.
typedef struct ota_s ota_t;

/**@brief Heart Rate Service init structure. This contains all options and data needed for
 *        initialization of the service. */
typedef struct
{
    ble_srv_cccd_security_mode_t 	cccd_attr_md;                                      /**< Initial security level for heart rate service measurement attribute */
    ble_srv_security_mode_t       	attr_md;                                      /**< Initial security level for body sensor location attribute */
} ota_init_t;

/**@brief Heart Rate Service structure. This contains various status information for the service. */
struct ota_s
{
    ble_gatts_char_handles_t     app_handle;                                          /**< Handles related to the Heart Rate Measurement characteristic. */
    ble_gatts_char_handles_t     fw_handle;                                          /**< Handles related to the Heart Rate Measurement characteristic. */
    uint16_t                     service_handle;                                       /**< Handle of Heart Rate Service (as provided by the BLE stack). */
    uint16_t                     conn_handle;                                          /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint8_t            char_enable[OTA_CHAR_LAST];	/**< is characteristic enabled >**/
    uint8_t					 	 uuid_type;
};

/*------------------------------------------------------------------------
**	@brief	Global Extern(s)
**------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------
**	@brief	Static Data
**------------------------------------------------------------------------
*/

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
/** @file
 *
 * @defgroup ble_sdk_srv_pavlok_ota Heart Rate Service
 * @{
 * @ingroup ble_sdk_srv
 * @brief Heart Rate Service module.
 *
 * @details This module implements the Heart Rate Service with the Heart Rate Measurement,
 *          Body Sensor Location and Heart Rate Control Point characteristics.
 *          During initialization it adds the Heart Rate Service and Heart Rate Measurement
 *          characteristic to the BLE stack database. Optionally it also adds the
 *          Body Sensor Location and Heart Rate Control Point characteristics.
 *
 *          If enabled, notification of the Heart Rate Measurement characteristic is performed
 *          when the application calls ota_heart_rate_measurement_send().
 *
 *          The Heart Rate Service also provides a set of functions for manipulating the
 *          various fields in the Heart Rate Measurement characteristic, as well as setting
 *          the Body Sensor Location characteristic value.
 *
 *          If an event handler is supplied by the application, the Heart Rate Service will
 *          generate Heart Rate Service events to the application.
 *
 * @note The application must propagate BLE stack events to the Heart Rate Service module by calling
 *       ota_on_ble_evt() from the @ref softdevice_handler callback.
 *
 * @note Attention!
 *  To maintain compliance with Nordic Semiconductor ASA Bluetooth profile
 *  qualification listings, this section of source code must not be modified.
 */


/**@brief Function for initializing the Heart Rate Service.
 *
 * @param[out]  p_pavlok_ota       Heart Rate Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_pavlok_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ota_service_init(void);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Heart Rate Service.
 *
 * @param[in]   p_pavlok_ota      Heart Rate Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void        ota_on_ble_evt(ble_evt_t * p_ble_evt);
uint32_t ota_service_init(void);


#endif /* SERVICE_OTA_H_ */
/** @} */
