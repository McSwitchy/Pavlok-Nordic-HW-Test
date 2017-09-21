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
#ifndef _LOGGER_TASK_H_
#define _LOGGER_TASK_H_

/**	----------------------------------------------------------------------
**	@brief System Include(s)
**	----------------------------------------------------------------------
*/
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
/**	----------------------------------------------------------------------
**	@brief Project Include(s)
**	----------------------------------------------------------------------
*/
//#include "serial_flash_fsm.h"

/**	----------------------------------------------------------------------
**	@brief	Global Data
**	----------------------------------------------------------------------
*/
enum
{
//	EVT_ACCEL,              // additive
//	EVT_GYRO,              // additive
	EVT_ZAP,              // additive
	EVT_PIEZO,              // additive
	EVT_MOTOR,              // additive
	EVT_BUTTON,              // time
	EVT_LED,              // additive
	EVT_SFW,              // additive
	EVT_CHG_CPLT,              // additive
	EVT_USB_ON,              // additive
	EVT_USB_OFF,              // additive
	EVT_BATT_LEVEL,              // uint16
	EVT_ZAP_EXCEED_MAX,              // uint16
	EVT_RESET_REASON,        // additive
	EVT_SNOOZE,              // additive
	EVT_ALARM_OFF              // additive

};

typedef struct
{
	int32_t						entry;
	uint32_t	        event;
  uint32_t          value;
  uint32_t          line;
  char *            file_name;
  
	int8_t 						seconds;
	int8_t 						minutes;
	int8_t 						hours;
	int8_t 						days;
	int8_t 						months;

} sPavlokLogEntry_t;

typedef struct 
{
	uint32_t	        event;
  uint32_t          value;
  char *            file_name;
} sPavlokRamLogEntry_t;

#define PAVLOK_LOG_ENTRY(a,b,c,d)	logger_ram_write((a), (b), (c), (d))

/**	----------------------------------------------------------------------
**	@brief	Global Extern(s)
**	----------------------------------------------------------------------
*/
extern uint16_t current_log_count;
extern uint16_t	total_log_count;

#define PAVLOK_LOG_ENTRY_SIZE	            (64)
#define PAVLOK_LOG_MAX_ENTRIES		        (64) 	// always power of 2


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
BaseType_t 	logger_start_task (void);
TaskHandle_t get_logger_thread_handle(void);
uint32_t logger_entry_read(uint8_t * entry);


void 											logger_ram_write(uint32_t event, int32_t parm1, int16_t line, const char * p_file_name);

void                      logger_list_init(void);
uint16_t                  logger_get_entry_count(void);

#endif /* _LOGGER_TASK_H_ */
/** @} */
