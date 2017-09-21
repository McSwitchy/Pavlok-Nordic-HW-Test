/**	----------------------------------------------------------------------
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
**	----------------------------------------------------------------------
*/
#ifndef _SERVICE_TASK_H_
#define _SERVICE_TASK_H_

/**	----------------------------------------------------------------------
**	@brief System Include(s)
**	----------------------------------------------------------------------
*/
/**	----------------------------------------------------------------------
**	@brief Project Include(s)
**	----------------------------------------------------------------------
*/

/**	----------------------------------------------------------------------
**	@brief	Global Data
**	----------------------------------------------------------------------
*/

/**	----------------------------------------------------------------------
**	@brief	Global Extern(s)
**	----------------------------------------------------------------------
*/

/**	----------------------------------------------------------------------
**	@brief	Static Data
**	----------------------------------------------------------------------
*/

/**	----------------------------------------------------------------------
**	@brief	Static Forward References
**	----------------------------------------------------------------------
*/

/**	----------------------------------------------------------------------
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
**	----------------------------------------------------------------------
*/
BaseType_t 	service_start_task (void);
TaskHandle_t get_service_thread_handle(void);
uint32_t		service_action_state_get(eCFG_CHAR_LIST stimulas);

void service_task_perform_piezo_action(uint8_t * piezo_cfg);
void service_task_perform_motor_action(uint8_t * motor_cfg);
void service_task_perform_zap_action(uint8_t * motor_cfg);
void service_task_perform_led_action(uint8_t * led_cfg);

#endif /* _SERVICE_TASK_H_ */
/** @} */
