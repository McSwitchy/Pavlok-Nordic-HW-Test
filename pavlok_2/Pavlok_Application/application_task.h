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
#ifndef _APPLICATION_TASK_TASK_H_
#define _APPLICATION_TASK_TASK_H_

/**	----------------------------------------------------------------------
**	@brief System Include(s)
**	----------------------------------------------------------------------
*/
#include <stdint.h>
#include "ble_gatts.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_assert.h"
#include "app_error.h"
#include "ble.h"
#include "ble_gap.h"
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
// #include "peer_manager.h"
#include "bsp.h"
// #include "bsp_btn_ble.h"
#include "FreeRTOS.h"
#include "task.h"

/**	----------------------------------------------------------------------
**	@brief Project Include(s)
**	----------------------------------------------------------------------
*/
#include "rtc.h"
/**	----------------------------------------------------------------------
**	@brief	Global Data
**	----------------------------------------------------------------------
*/
// repeat mask
#define REPEAT_ENABLED		((1) << 7)
#define REPEAT_SUNDAY			((1) << 6)
#define REPEAT_MONDAY			((1) << 5)
#define REPEAT_TUESDAY			((1) << 4)
#define REPEAT_WEDNESDAY		((1) << 3)
#define REPEAT_THURSDAY		((1) << 2)
#define REPEAT_FRIDAY			((1) << 1)
#define REPEAT_SATURDAY		((1) << 0)

#define ALARM_SNOOZE_DEFAULT  (60 * 5)

/*
// application tag types
	AH,			// application header
	An,			// alarm with number – zero is always the first in any series of alarms
	Pn,			// piezo stimulus with number – zero is always the first in any series of alarms
	Mn,			// motor stimulus with number – zero is always the first in any series of alarms
	Zn,			// zapper stimulus with number – zero is always the first in any series of alarms
	Un,			// user action  with number – zero is always the first in any series of alarms
	RM,			// repeat mask
	WD,			// window
	WI,			// window interval
	SN,			// snooze enable
	TM,			// time
*/

typedef struct
{
	int8_t seconds;
	int8_t minutes;
	int8_t hours;
	int8_t weekdays; // the repeat mask if any for this alarm

} sAlarmTime_t;


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
BaseType_t 		application_start_task (void);
//TaskHandle_t 	get_application_task_thread_handle(void);
uint32_t 			application_task_entry_read(uint8_t * entry);
void 					application_set_ready(ble_gatts_evt_write_t * p_evt_write);
void 					application_task_restart(void);
void          application_start_stimulus(void);
uint32_t      application_write_to_ram (uint8_t * input_data, uint32_t length);
bool          application_is_in_alarm_state(void);

void application_set_alarm_disable(uint8_t value);
void application_set_snooze_disable(uint8_t value);

void application_get_current_name(char * name);
void application_check_button_press(void);

#endif /* _APPLICATION_TASK_TASK_H_ */
/** @} */
