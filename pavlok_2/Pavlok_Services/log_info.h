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
#ifndef SERVICE_LOG_INFO_H_
#define SERVICE_LOG_INFO_H_


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

/*------------------------------------------------------------------------
**	@brief	Global Extern(s)
**------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------
**	@brief	Static Data
**------------------------------------------------------------------------
*/
typedef enum
{
	LOG_CHAR_CMD,
	LOG_CHAR_DATA,
	LOG_CHAR_LAST

} eLOG_CHAR_LIST;

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
**	@brief	Global Data
**------------------------------------------------------------------------
*/


// Forward declaration of the log_info_t type.
typedef struct log_info_s log_info_t;

/**@brief Heart Rate Service init structure. This contains all options and data needed for
 *        initialization of the service. */
typedef struct
{
    ble_srv_cccd_security_mode_t cccd_attr_md;                                      /**< Initial security level for heart rate service measurement attribute */
    ble_srv_security_mode_t       attr_md;                                      /**< Initial security level for body sensor location attribute */
} log_info_init_t;

/**@brief Heart Rate Service structure. This contains various status information for the service. */
struct log_info_s
{
    ble_gatts_char_handles_t     log_cmd_handle;                                          /**< Handles related to the Heart Rate Measurement characteristic. */
    ble_gatts_char_handles_t     log_data_handle;                                          /**< Handles related to the Heart Rate Measurement characteristic. */
    uint16_t                     service_handle;                                       /**< Handle of Heart Rate Service (as provided by the BLE stack). */
    uint16_t                     conn_handle;                                          /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint8_t					 	 info_cmd;
    uint8_t					 	 info_data;
    uint8_t            char_enable[LOG_CHAR_LAST];	/**< is characteristic enabled >**/
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
 * @defgroup ble_sdk_srv_pavlok_cfg Heart Rate Service
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
 *          when the application calls log_info_heart_rate_measurement_send().
 *
 *          The Heart Rate Service also provides a set of functions for manipulating the
 *          various fields in the Heart Rate Measurement characteristic, as well as setting
 *          the Body Sensor Location characteristic value.
 *
 *          If an event handler is supplied by the application, the Heart Rate Service will
 *          generate Heart Rate Service events to the application.
 *
 * @note The application must propagate BLE stack events to the Heart Rate Service module by calling
 *       log_info_on_ble_evt() from the @ref softdevice_handler callback.
 *
 * @note Attention!
 *  To maintain compliance with Nordic Semiconductor ASA Bluetooth profile
 *  qualification listings, this section of source code must not be modified.
 */


/**@brief Function for initializing the Heart Rate Service.
 *
 * @param[out]  p_pavlok_cfg       Heart Rate Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_pavlok_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t log_info_init(void);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Heart Rate Service.
 *
 * @param[in]   p_pavlok_cfg      Heart Rate Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void        log_info_on_ble_evt(ble_evt_t * p_ble_evt);
uint16_t    log_info_service_get_handle(void);
void        log_info_get_handle_range(uint16_t * start, uint16_t * end);



#endif /* SERVICE_LOG_INFO_H_ */
/** @} */
