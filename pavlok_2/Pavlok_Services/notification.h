/*------------------------------------------------------------------------
**
**	@file		notification.h
**
**  @defgroup notification
**  @{
**  @ingroup 
**  @brief Pvlok Notification module.
**  
**  @details This module implements 
** @details This module implements the notification Service for PAVLOK,
**          During initialization it adds the notification Service and
**          characteristic to the BLE stack database.
**
**          If an event handler is supplied by the application, the notification Service will
**          generate notification Service events to the application.
**
** @note The application must propagate BLE stack events to the notification Service module by calling
**       notification_on_ble_evt() from the @ref softdevice_handler callback.
**  
**  @note The application must 
**  
**  @note Attention!
**   
**
**------------------------------------------------------------------------
*/
#ifndef SERVICE_NOTIFICAITON_H_
#define SERVICE_NOTIFICAITON_H_

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
	eNOTIFICATION_CHAR_TIME,
	eNOTIFICATION_CHAR_LOG_CNT,
	eNOTIFICATION_CHAR_APP_CURRENT,
	eNOTIFICATION_CHAR_BTN_PRESS,
	eNOTIFICATION_CHAR_ZAP_CNT,
	eNOTIFICATION_CHAR_PIEZO_CNT,
	eNOTIFICATION_CHAR_MOTOR,
	eNOTIFICATION_CHAR_LED,
	eNOTIFICATION_CHAR_SLEEP_DATA,
	eNOTIFICATION_CHAR_ALARM_TIME,
	eNOTIFICATION_CHAR_LAST
	
} eNOT_CHAR_LIST;


// Forward declaration of the notification_t type.
typedef struct notification_s notification_t;

/**@brief notification Service init structure. This contains all options and data needed for
 *        initialization of the service. */
typedef struct
{
    ble_srv_cccd_security_mode_t 	cccd_attr_md;                                      /**< Initial security level for heart rate service measurement attribute */
    ble_srv_security_mode_t       	attr_md;                                      /**< Initial security level for body sensor location attribute */
} notification_init_t;

/**@brief notification Service structure. This contains various status information for the service. */
struct notification_s
{
    ble_gatts_char_handles_t     time_handle;                                          /**< Handles related to the notification Measurement characteristic. */
    ble_gatts_char_handles_t     log_cnt_handle;                                          /**< Handles related to the notification Measurement characteristic. */
    ble_gatts_char_handles_t     app_name_handle;                                          /**< Handles related to the notification Measurement characteristic. */
    ble_gatts_char_handles_t     btn_press_handle;                                          /**< Handles related to the notification Measurement characteristic. */
    ble_gatts_char_handles_t     zap_cnt_handle;                                          /**< Handles related to the notification Measurement characteristic. */
    ble_gatts_char_handles_t     piezo_cnt_handle;                                          /**< Handles related to the notification Measurement characteristic. */
    ble_gatts_char_handles_t     motor_cnt_handle;                                          /**< Handles related to the notification Measurement characteristic. */
    ble_gatts_char_handles_t     led_cnt_handle;                                          /**< Handles related to the notification Measurement characteristic. */
    ble_gatts_char_handles_t     sleep_handle;                                          /**< Handles related to the notification Measurement characteristic. */
    ble_gatts_char_handles_t     alarm_time_handle;                                          /**< Handles related to the notification Measurement characteristic. */
    uint16_t                     service_handle;                                       /**< Handle of notification Service (as provided by the BLE stack). */
    uint16_t                     conn_handle;                                          /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint8_t					 	 						info_cmd;
    uint8_t					 	 						info_data;
    uint8_t                      char_enable[eNOTIFICATION_CHAR_LAST];	/**< is characteristic enabled >**/
    uint8_t					 	 						uuid_type;
};

/*------------------------------------------------------------------------
**
**	@fn		Function		uint32_t notification_service_init(void);
**
**	@brief	Description		initializes the PAVLOK acceleromter
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
uint32_t notification_service_init(void);

/*------------------------------------------------------------------------
**
**	@fn		Function		void  notification_on_ble_evt(ble_evt_t * p_ble_evt);
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
void        notification_on_ble_evt(ble_evt_t * p_ble_evt);

/*------------------------------------------------------------------------
**
**	@fn		Function		uint16_t notification_service_get_handle(void);
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
uint16_t    notification_service_get_handle(void);

/*------------------------------------------------------------------------
**
**	@fn		Function		void  notification_get_handle_range(uint16_t * start, uint16_t * end);
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
void  notification_get_handle_range(uint16_t * start, uint16_t * end);

/*------------------------------------------------------------------------
**
**	@fn		Function		uint16_t notification_service_get_connection_handle(void);
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
uint16_t notification_service_get_connection_handle(void);


void notification_increment_button_count(void);
void notification_increment_motor_count(void);
void notification_increment_piezo_count(void);
void notification_increment_zap_count(void);
void notification_increment_led_count(void);


#endif /* SERVICE_NOTIFICAITON_H_ */
/** @} */
