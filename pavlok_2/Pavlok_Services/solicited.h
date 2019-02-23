/*------------------------------------------------------------------------
**
**	@file		solicited.h
**
**  @defgroup solicited
**  @{
**  @ingroup 
**  @brief Alert Notification module.
**  
**  @details This module implements 
** @details This module implements the solicited Service for PAVLOK,
**          During initialization it adds the solicited Service and
**          characteristic to the BLE stack database.
**
**          If an event handler is supplied by the application, the solicited Service will
**          generate solicited Service events to the application.
**
** @note The application must propagate BLE stack events to the solicited Service module by calling
**       solicited_on_ble_evt() from the @ref softdevice_handler callback.
**  
**  @note The application must 
**  
**  @note Attention!
**   
**
**------------------------------------------------------------------------
*/
#ifndef SERVICE_SOLICITED_H_
#define SERVICE_SOLICITED_H_

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

// Forward declaration of the solicited_t type.
typedef struct solicited_s solicited_t;

/**@brief solicited Service init structure. This contains all options and data needed for
 *        initialization of the service. */
typedef struct
{
    ble_srv_cccd_security_mode_t 	cccd_attr_md;                                      /**< Initial security level for heart rate service measurement attribute */
    ble_srv_security_mode_t       attr_md;                                      /**< Initial security level for body sensor location attribute */
} solicited_init_t;

/**@brief solicited Service structure. This contains various status information for the service. */
struct solicited_s
{
    ble_gatts_char_handles_t     control_handle;                                          /**< Handles related to the solicited Measurement characteristic. */
    ble_gatts_char_handles_t     data_handle;                                          /**< Handles related to the solicited Measurement characteristic. */
    uint16_t                     service_handle;                                       /**< Handle of solicited Service (as provided by the BLE stack). */
    uint16_t                     conn_handle;                                          /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint8_t                      control_enable;	/**< is characteristic enabled >**/
    uint8_t                      data_enable;	/**< is characteristic enabled >**/
    uint8_t					 	           uuid_type;
};

/*------------------------------------------------------------------------
**
**	@fn		Function		uint32_t solicited_service_init(void);
**
**	@brief	Description		initializes the PAVLOK solicited
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
uint32_t solicited_service_init(void);

/*------------------------------------------------------------------------
**
**	@fn		Function		void  solicited_on_ble_evt(ble_evt_t * p_ble_evt);
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
void        solicited_on_ble_evt(ble_evt_t * p_ble_evt);

/*------------------------------------------------------------------------
**
**	@fn		Function		uint16_t solicited_service_get_handle(void);
**
**	@brief	Description		service function for use by the application
**
**	@param [in]				None
**
**	@param	[out]			None
**
**	@return					None
**
**	@warn
**
**------------------------------------------------------------------------
*/
uint16_t    solicited_service_get_handle(void);

/*------------------------------------------------------------------------
**
**	@fn		Function		void  solicited_get_handle_range(uint16_t * start, uint16_t * end);
**
**	@brief	Description		service function for use by the ble evnet call
**							to save time in the BLE timing cycle
**
**	@param [in]				None
**
**	@param	[out]			None
**
**	@return					None
**
**	@warn
**
**------------------------------------------------------------------------
*/
void  solicited_get_handle_range(uint16_t * start, uint16_t * end);

/*------------------------------------------------------------------------
**
**	@fn		Function		uint16_t solicited_service_get_connection_handle(void);
**
**	@brief	Description		service function for use by gap init function
**
**	@param [in]				None
**
**	@param	[out]			None
**
**	@return					None
**
**	@warn
**
**------------------------------------------------------------------------
*/
uint16_t solicited_service_get_connection_handle(void);

uint16_t	get_train_service_data_cccd_handle(void);
void 			start_solicited_session(void);
uint32_t solicited_send(bool value);

#endif /* SERVICE_APSVC_H_ */
/** @} */
