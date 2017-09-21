/*------------------------------------------------------------------------
**
**	@file		appsvc.h
**
**  @defgroup appsvc
**  @{
**  @ingroup 
**  @brief Alert Notification module.
**  
**  @details This module implements 
** @details This module implements the appsvc Service for PAVLOK,
**          During initialization it adds the appsvc Service and
**          characteristic to the BLE stack database.
**
**          If an event handler is supplied by the application, the appsvc Service will
**          generate appsvc Service events to the application.
**
** @note The application must propagate BLE stack events to the appsvc Service module by calling
**       appsvc_on_ble_evt() from the @ref softdevice_handler callback.
**  
**  @note The application must 
**  
**  @note Attention!
**   
**
**------------------------------------------------------------------------
*/
#ifndef SERVICE_APSVC_H_
#define SERVICE_APSVC_H_

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
#include "pavlok_common.h"

/*------------------------------------------------------------------------
**	@brief	Global Data
**------------------------------------------------------------------------
*/

// Forward declaration of the appsvc_t type.
typedef struct appsvc_s appsvc_t;

/**@brief appsvc Service init structure. This contains all options and data needed for
 *        initialization of the service. */
typedef struct
{
    ble_srv_cccd_security_mode_t 	cccd_attr_md;                                      /**< Initial security level for heart rate service measurement attribute */
    ble_srv_security_mode_t       	attr_md;                                      /**< Initial security level for body sensor location attribute */
} appsvc_init_t;

/**@brief appsvc Service structure. This contains various status information for the service. */
struct appsvc_s
{
    ble_gatts_char_handles_t     download_handle;                                          /**< Handles related to the appsvc Measurement characteristic. */
    ble_gatts_char_handles_t     control_handle;                                          /**< Handles related to the appsvc Measurement characteristic. */
    ble_gatts_char_handles_t     at_handle;                                          /**< Handles related to the appsvc Measurement characteristic. */
    ble_gatts_char_handles_t     st_handle;                                          /**< Handles related to the appsvc Measurement characteristic. */
    ble_gatts_char_handles_t     ad_handle;                                          /**< Handles related to the appsvc Measurement characteristic. */
    uint16_t                     service_handle;                                       /**< Handle of appsvc Service (as provided by the BLE stack). */
    uint16_t                     conn_handle;                                          /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint8_t					 	 info_cmd;
    uint8_t					 	 info_data;
    uint8_t            char_enable[APPSVC_CHAR_LAST];	/**< is characteristic enabled >**/
    uint8_t					 	 uuid_type;
};

/*------------------------------------------------------------------------
**
**	@fn		Function		uint32_t appsvc_service_init(void);
**
**	@brief	Description		initializes the PAVLOK appsvc
**
**	@param [in]				None
**
**	@param	[out]			None
**
**	@return					uint32_t
**
**	@retval					NRF_ERROR_NO_MEM
**							NRF_SUCCESS
**
**	@warn
**
**------------------------------------------------------------------------
*/
uint32_t appsvc_service_init(void);

/*------------------------------------------------------------------------
**
**	@fn		Function		void  appsvc_on_ble_evt(ble_evt_t * p_ble_evt);
**
**	@brief	Description		called from ble_evt_dispatch which is called
**							from the soft device
**
**	@param [in]				ble_evt_t  Event received from the BLE stack.
**
**	@param	[out]			None
**
**	@return					None
**
**	@warn
**
**------------------------------------------------------------------------
*/
void appsvc_on_ble_evt(ble_evt_t * p_ble_evt);
void appsvc_send(eAPPSVC_CHAR_LIST char_type, uint32_t data);

#endif /* SERVICE_APSVC_H_ */
/** @} */
