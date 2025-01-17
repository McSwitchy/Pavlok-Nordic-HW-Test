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
#ifndef _PAVLOK_COMMON_H__
#define _PAVLOK_COMMON_H__

/*------------------------------------------------------------------------
**	@brief System Include(s)
**------------------------------------------------------------------------
*/
//#include <stdint.h>
//#include <string.h>

#include <ble_l2cap.h>

//#include "nordic_common.h"
//#include "nrf.h"
//#include "app_timer.h"
//#include "app_error.h"
#include "ble_gap.h"
//#include "ble_hci.h"
//#include "ble_srv_common.h"
//#include "ble_advdata.h"
//#include "ble_advertising.h"
//#include "ble_conn_params.h"
//#include "boards.h"
//#include "softdevice_handler.h"

#include <FreeRTOS.h>
#include <task.h>
#include <timers.h>

//#include "bsp_btn_ble.h"


/*------------------------------------------------------------------------
**	@brief Project Include(s)
**------------------------------------------------------------------------
*/
#include "rtc.h"
#include "leds.h"
#include "serial_flash.h"

/*------------------------------------------------------------------------
**	@brief	Global Data
**------------------------------------------------------------------------
*/
#define PAVLOK_BASE_ADV_UUID		(0x0000)
#define PAVLOK_BASE_UUID				{{0x61, 0x44, 0xd7, 0x98, 0xf6, 0x86, 0x7b, 0x89, 0xea, 0x4f, 0x00, 0xa3, 0x00, 0x00, 0x6e, 0x15}}


// new uuid 156eb100-a300-4fea-897b-86f698d74461

#define PAVLOK_BUTTONS (1)
#define PAVLOK_APP_SCRATCH_SIZE													W25X40CL_SECTOR_SIZE_BYTES
#define PAVLOK_V_USB_PIN            (31)

#define PAVLOK_RESET_PIN           (21)

#define PAVLOK_MAX_TASKS                      (5)

#define OPCODE_LENGTH 1                                                    /**< Length of opcode inside Heart Rate Measurement packet. */
#define HANDLE_LENGTH 2                                                    /**< Length of handle inside Heart Rate Measurement packet. */
#define PAVLOK_MAX_MSG_SIZE   (23 - OPCODE_LENGTH - HANDLE_LENGTH)  /**< usually 23. */

#define PAVLOK_SIZEOF_UINT8                   (sizeof(uint8_t))
#define PAVLOK_SIZEOF_BOOL                    (sizeof(bool))
#define PAVLOK_SIZEOF_UINT16                  (sizeof(uint16_t))
#define PAVLOK_SIZEOF_UINT32                  (sizeof(uint32_t))

#define PAVLOK_MAX_CONNECTION_HANDLE			(100)

#define PAVLOK_BUTTON_TICK_TIME       (100) // ms
#define PAVLOK_BUTTON_PRESS_1SEC      (PAVLOK_BUTTON_TICK_TIME * 10)
#define PAVLOK_BUTTON_PRESS_2SEC      (PAVLOK_BUTTON_TICK_TIME * 20)
#define PAVLOK_BUTTON_PRESS_3SEC      (PAVLOK_BUTTON_TICK_TIME * 30)
#define PAVLOK_BUTTON_PRESS_5SEC      (PAVLOK_BUTTON_TICK_TIME * 50)
#define PAVLOK_BUTTON_PRESS_8SEC      (PAVLOK_BUTTON_TICK_TIME * 80)

#define PAVLOK_BUTTON_INTERVAL        PAVLOK_BUTTON_TICK_TIME

#define    _CRLF_                   ('\r')

#define ENAB_BB (19)            // enables battery on  the  real device

// added from https://developer.bluetooth.org/gatt/units/Pages/default.aspx" but changed the name to pavlok as it is not in the Nordic software
#define BLUETOOTH_ORG_CPF_UNIT_EPD   (0x2728)
#define BLUETOOTH_ORG_CPF_UNITLESS   (0x2700)
#define BLUETOOTH_ORG_CPF_FREQ       (0x2722)
#define BLUETOOTH_ORG_CPF_DAYS       (0x2762)
#define BLUETOOTH_ORG_CPF_PERC       (0x27AD)
#define BLUETOOTH_ORG_CPF_TIME       (0x2703)

#define PAVLOK_SERVICE_ADV_COUNT               (1)
#define PAVLOK_GATT_TABLE_SIZE                (0x1500)  /**<Very IMPORTANT -- the linker script matches this value only change all 3 sizes if any are changed */

/////////////////////////////
#define APP_ADV_INTERVAL                 300                                    /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS       180                                    /**< The advertising time-out in units of seconds. */

#define MIN_CONN_INTERVAL                MSEC_TO_UNITS(400, UNIT_1_25_MS)       /**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL                MSEC_TO_UNITS(650, UNIT_1_25_MS)       /**< Maximum acceptable connection interval (0.65 second). */
#define SLAVE_LATENCY                    0                                      /**< Slave latency. */
#define CONN_SUP_TIMEOUT                 MSEC_TO_UNITS(4000, UNIT_10_MS)        /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY   5000                                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY    30000                                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT     3                                      /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                   1                                      /**< Perform bonding. */
#define SEC_PARAM_MITM                   0                                      /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                   0                                      /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS               0                                      /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES        BLE_GAP_IO_CAPS_NONE                   /**< No I/O capabilities. */
#define SEC_PARAM_OOB                    0                                      /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE           7                                      /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE           16                                     /**< Maximum encryption key size. */

#define DEAD_BEEF                        0xDEADBEEF                             /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define OSTIMER_WAIT_FOR_QUEUE           2                                      /**< Number of ticks to wait for the timer queue to be ready */

#define APP_FEATURE_NOT_SUPPORTED        BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2   /**< Reply when unsupported features are requested. */



//////////////////////////////

#define APP_TIMER_OP_QUEUE_SIZE              10                                          /**< Size of timer operation queues. */


#define CCCD_MSG_SIZE						(2)

/** ----------------------------------------------------------------------
**	@brief	Flash Defines
**  ----------------------------------------------------------------------
*/
#define FLASH_SECTOR_APPLICATION_START    (0)
//#define FLASH_SECTOR_APPLICATION_END      (49)
#define FLASH_SECTOR_APPLICATION_END      (8)
#define FLASH_SECTOR_HABIT_START          (50)
#define FLASH_SECTOR_HABIT_END            (69)
#define FLASH_SECTOR_SLEEP_START          (70)
#define FLASH_SECTOR_SLEEP_END            (90)
#define FLASH_SECTOR_LOG_START            (120)
#define FLASH_SECTOR_LOG_END              (127)
#define UNUSED_SECTOR                     (uint8_t)(0xFF)

typedef enum
{
  eFlashAll,
  eFlashApp,
  eFlashHabit,
  eFlashSleep,
  eFlashLog,
  eFlashLast

} eFlashAreaName_t;

/**	----------------------------------------------------------------------
 **	@def PAVLOK_TLV_AP_NAME_INDEX
 ** offset to the product areas name for apps, habit
 **	----------------------------------------------------------------------
 */
#define PAVLOK_TLV_AP_NAME_INDEX											  (10)
#define PAVLOK_TLV_AP_NAME_SIZE_INDEX										(8)
#define PAVLOK_TLV_VALUE_START													(2)
#define PAVLOK_SECTOR_NAME_SIZE                         (32)
#define PAVLOK_APP_BLOCK_SIZE														(4096)

/**	----------------------------------------------------------------------
 **	@def PAVLOK pin defines
 **	----------------------------------------------------------------------
 */
#define VBATT_ENABLE				(2)
#define VBATT_READ					(30)
#define VUSB_READ						(31)
#define HVMON_READ					(29)
#define PAVLOK_ACCEL_MAG_RESET_PIN (26)

/**	----------------------------------------------------------------------
 **	@def PAVLOK Time defines
 **	----------------------------------------------------------------------
 */
// TODO add back in real number here #define PLOK_CHECK_VBATT_INTERVAL				(120000)
#define PLOK_CHECK_VBATT_INTERVAL				(10)
#define PLOK_NUM_OF_SEC_IN_A_MIN        (60)
#define PLOK_TIME_MSG_LEN								(8)
#define PLOK_TIME_MSG_DATA_LEN					(7)
#define PLOK_SECONDS_MINUTE_MAX_VALUE		(59)
#define PLOK_SECONDS_HOURS_MAX_VALUE		(23)
#define PLOK_SECONDS_WEEK_DAY_MIN_VALUE		(1)
#define PLOK_SECONDS_WEEK_DAY_MAX_VALUE		(7)
#define PLOK_SECONDS_MONTH_DAY_MIN_VALUE	(1)
#define PLOK_SECONDS_MONTH_DAY_MAX_VALUE	(31)
#define PLOK_SECONDS_MONTH_MIN_VALUE		(1)
#define PLOK_SECONDS_MONTH_MAX_VALUE		(12)
#define PLOK_SECONDS_YEAR_MIN_VALUE			(17)
#define PLOK_SECONDS_YEAR_MAX_VALUE			(99)

#define	PLOK_TIME_SECONDS_INDEX				(1)
#define	PLOK_TIME_MINUTE_INDEX				(2)
#define	PLOK_TIME_HOURS_INDEX				(3)
#define	PLOK_TIME_WEEK_DAY_INDEX			(4)
#define	PLOK_TIME_MONTH_DAY_INDEX			(5)
#define	PLOK_TIME_MONTH_INDEX				(6)
#define	PLOK_TIME_YEAR_INDEX				(7)

#define	PLOK_10_MS									(10)
#define	PLOK_20_MS									(20)
#define	PLOK_30_MS									(30)
#define	PLOK_40_MS									(40)
#define	PLOK_50_MS									(50)
#define	PLOK_60_MS									(60)
#define	PLOK_70_MS									(70)
#define	PLOK_80_MS									(80)
#define	PLOK_90_MS									(90)
#define	PLOK_100_MS									(100)

#define	PLOK_150_MS									(150)
#define	PLOK_200_MS									(200)
#define	PLOK_250_MS									(250)
#define	PLOK_300_MS									(300)
#define	PLOK_350_MS									(350)
#define	PLOK_400_MS									(400)
#define	PLOK_450_MS									(450)
#define	PLOK_500_MS									(500)
#define	PLOK_1_SEC									(1000)
#define	PLOK_3_SEC									(3000)
#define	PLOK_5_SEC									(5000)
#define	PLOK_30_SEC		              (30000)

#define MSB_OFFSET							(0)
#define LSB_OFFSET							(1)

/**	----------------------------------------------------------------------
**	The peripheral control sizes
**	----------------------------------------------------------------------
*/
#define PAVLOK_TLV_TAG_SIZE															(2)
#define PAVLOK_TLV_TAG_LENGTH_SIZE											(4)
#define PLOK_STIMULUS_COUNT_MASK			(0x07)
#define PLOK_DC_10_PERCENT_VALUE      (10)
#define PLOK_DC_50_PERCENT_VALUE      (50)
#define PLOK_DC_100_PERCENT_VALUE      (100)

#define PLOK_VALUE_LENGTH_VB_MOTOR		(5)
#define PLOK_VALUE_LENGTH_PIEZO				(5)
#define PLOK_VALUE_LENGTH_TIME				(7)
#define PLOK_VALUE_LENGTH_ZAP					(2)
#define PLOK_VALUE_LENGTH_LED					(5)
#define PLOK_VALUE_LENGTH_HD					(2)
#define PLOK_VALUE_LENGTH_ALM_CTL			(2)
#define PLOK_VALUE_LENGTH_BUTTON			(4) // first 2 bytes are configuration and second 2 are actual buttontime value

#define PLOK_INDEX_CONTROL_BYTE_LED		(0)
#define PLOK_INDEX_COLOR_BYTE_LED			(1)
#define PLOK_INDEX_CODE_LED			      (2)
#define PLOK_INDEX_ONTIME_BYTE_LED		(3)
#define PLOK_INDEX_OFFTIME_BYTE_LED		(4)
#define PLOK_LED_CYCLE_COUNT_MASK			(0x07)
#define LED_TYPE_SHIFT								(3)
#define LED_TYPE_MASK									(0x38)
#define LED_TYPE(x)										(((uint8_t)(((uint8_t)(x)) & LED_TYPE_MASK) >> LED_TYPE_SHIFT))
#define	PLOK_PIEZO_COUNT_SHIFT				(0)
#define PLOK_PIEZO_COUNT_VALUE(x)			(uint8_t)((x) & PLOK_VB_CYCLE_COUNT_MASK)
#define	PLOK_PIEZO_PERFORM_SHIFT			(7)
#define PLOK_PIEZO_PERFORM_BIT				(uint8_t)((1) << PLOK_PIEZO_PERFORM_SHIFT)

#define	PLOK_MOTOR_COUNT_SHIFT				(0)
#define PLOK_MOTOR_COUNT_VALUE(x)			(uint8_t)((x) & PLOK_VB_CYCLE_COUNT_MASK)
#define	PLOK_MOTOR_PERFORM_SHIFT			(7)
#define PLOK_MOTOR_PERFORM_BIT				(uint8_t)((1) << PLOK_PIEZO_PERFORM_SHIFT)


#define PLOK_ZAP_ONCE			              (0x81)
#define PLOK_MOTOR_ONCE			            (0x81)
#define PLOK_PIEZO_ONCE			            (0x81)
#define PLOK_MOTOR_DEFAULT_FREQ         (2)
#define PLOK_MOTOR_DEFAULT_DC           (0x50) // 80 %
#define PLOK_MOTOR_DEFAULT_TIME         (0x50) // 80ms
#define PLOK_PIEZO_DEFAULT_FREQ         (2)
#define PLOK_PIEZO_DEFAULT_DC           (0x50) // 80 %
#define PLOK_PIEZO_DEFAULT_TIME         (0x50) // 80ms

#define PLOK_ZAP_LED_NOTIFY             (0x20)
#define PLOK_ZAP_NOTIFY_COUNT           (0x2)  // TODO fix to use performace timers??????

#define PLOK_ZAP_CYCLE_COUNT_MASK				(0x07)
#define PLOK_VB_CYCLE_COUNT_MASK				(0x07)
#define PLOK_VB_BYTE_0_PERFORM_ACTION		(0x80) // TODO verify endian
#define PLOK_VB_BYTE_0_SAVE_DATA				(0x40)
#define PLOK_ZAP_BYTE_0									(0)
#define PLOK_INDEX_DC_BYTE_ZAP				  (1)
#define PLOK_DEFAULT_ZAP_DC             (0x32) // 50%

#define PLOK_INDEX_CONTROL_BYTE_PIEZO		(0)
#define PLOK_INDEX_FREQ_BYTE_PIEZO			(1)
#define PLOK_INDEX_DC_BYTE_PIEZO				(2)
#define PLOK_INDEX_ONTIME_BYTE_PIEZO		(3)
#define PLOK_INDEX_OFFTIME_BYTE_PIEZO		(4)
#define PLOK_OFFTIME_BYTE_ZAPPER		    (1000)
#define PLOK_ONTIME_BYTE_ZAPPER		      (500)

#define PLOK_INDEX_CONTROL_BYTE_MOTOR		(0)
#define PLOK_INDEX_FREQ_BYTE_MOTOR			(1)
#define PLOK_INDEX_DC_BYTE_MOTOR				(2)
#define PLOK_INDEX_ONTIME_BYTE_MOTOR		(3)
#define PLOK_INDEX_OFFTIME_BYTE_MOTOR		(4)

#define PLOK_PWM_FREQ_STEP_VALUE				(500)

#define UPPER_NIBBLE(b)									(((b) >> 4) & 0x0F)
#define LOWER_NIBBLE(b)									((b) & 0x0F)

/// @def PLOK_TIME_BETWEEN_STIMULI is used to gate the alarm stimulus patterns
#define PLOK_TIME_BETWEEN_STIMULI				(500) // 500ms


#define TIME_MAX_UPPER_SECONDS					(5)
#define TIME_MAX_LOWER_SECONDS					(9)
#define TIME_MAX_UPPER_MINUTES					(5)
#define TIME_MAX_LOWER_MINUTES					(9)

#define TIME_MAX_UPPER_HOURS_24					(2)
#define TIME_MAX_LOWER_HOURS_24					(9)
#define TIME_MIN_UPPER_HOURS_24					(0)
#define TIME_MIN_LOWER_HOURS_24					(0)

#define TIME_MAX_UPPER_DAYS							(3)
#define TIME_MAX_LOWER_DAYS							(9)
#define TIME_MIN_UPPER_DAYS							(0)
#define TIME_MIN_LOWER_DAYS							(0)

#define TIME_MAX_UPPER_WDAYS							(0)
#define TIME_MAX_LOWER_WDAYS							(6)
#define TIME_MIN_UPPER_WDAYS							(0)
#define TIME_MIN_LOWER_WDAYS							(0)

#define TIME_MAX_UPPER_MONTH							(1)
#define TIME_MAX_LOWER_MONTH							(9)
#define TIME_MIN_UPPER_MONTH							(0)
#define TIME_MIN_LOWER_MONTH							(1)

#define TIME_MAX_UPPER_YEARS							(9)
#define TIME_MAX_LOWER_YEARS							(9)
#define TIME_MIN_UPPER_YEARS							(1)
#define TIME_MIN_LOWER_YEARS							(0)


#define CFG_HAND_DETECT_HD_LR             (1 << 0)
#define CFG_HAND_DETECT_D_T               (1 << 1)
#define CFG_HAND_DETECT_BTN_LK            (1 << 2)
#define CFG_HAND_DETECT_S_P               (1 << 3)
#define CFG_HAND_DETECT_HD_LO             (1 << 4)
#define CFG_HAND_DETECT_DT_2              (1 << 5)
#define CFG_HAND_DETECT_SL_TR             (1 << 6)
#define CFG_HAND_DETECT_BP_TYPE           (3 << 0)
#define CFG_HAND_DETECT_BP_MOTOR          (1)
#define CFG_HAND_DETECT_BP_PIEZO          (2)
#define CFG_HAND_DETECT_BP_ZAP            (3)

// REMOVED #define CFG_HAND_DETECT_HD_TYPE           (3 << 4)


#define 	NOTIFICATION_CHAR_TIME_LENGTH						(8)
#define 	NOTIFICATION_CHAR_LOG_CNT_LENGTH				(1)
#define 	NOTIFICATION_CHAR_APP_CURRENT_LENGTH		(20)
#define 	NOTIFICATION_CHAR_BTN_PRESS_LENGTH			(1)
#define 	NOTIFICATION_CHAR_ZAP_CNT_LENGTH				(1)
#define 	NOTIFICATION_CHAR_PIEZO_CNT_LENGTH			(1)
#define 	NOTIFICATION_CHAR_MOTOR_LENGTH					(1)
#define 	NOTIFICATION_CHAR_LED_LENGTH						(1)
#define 	NOTIFICATION_CHAR_SLEEP_DATA_LENGTH			(21)
#define 	NOTIFICATION_CHAR_ALARM_TIME_LENGTH			(5)

/**	----------------------------------------------------------------------
**	The Device Information Service definse -- see ble_dis.c for use
**	----------------------------------------------------------------------
*/
#define PAVLOK_PNPID_VERSION               	{0x01, 0x000D, 0x00, 0x0110}
#define PAVLOK_MANUFACTURER_NAME  	  		  "Behavioral Technology Group, Inc."                     /**< Manufacturer. Will be passed to Device Information Service. */
#define PAVLOK_MODEL_NUMBER                 "Pavlok-S"
#define PAVLOK_SERIAL_NUMBER                "S UNK"
#define PAVLOK_HARDWARE_VERSION             "052781-9B"
#define PAVLOK_FIRMWARE_VERSION             "170817_08:30"
#define PAVLOK_SOFTWARE_VERSION             "V:0.7.2"

/*------------------------------------------------------------------------
**	@brief	Global Extern(s)
**------------------------------------------------------------------------
*/
extern uint8_t g_pavlok_scratch_pad[32];

/**	----------------------------------------------------------------------
 **	@def PAVLOK service info defines
 **	----------------------------------------------------------------------
 */
typedef enum
{
 	eSOLICITED_SVC_UUID										= 0x0000,
	eSOLICITED_CHAR_CONTROL_UUID			    = (eSOLICITED_SVC_UUID + 1),
	eSOLICITED_CHAR_LOG_DATA_UUID					= (eSOLICITED_SVC_UUID + 2),

	eCFG_SVC_UUID													= 0x1000,
	eCFG_CHAR_MOTOR_UUID									= (eCFG_SVC_UUID + 1),
	eCFG_CHAR_PIEZO_UUID									= (eCFG_SVC_UUID + 2),
	eCFG_CHAR_ZAP_UUID										= (eCFG_SVC_UUID + 3),
	eCFG_CHAR_LED_UUID										= (eCFG_SVC_UUID + 4),
	eCFG_CHAR_TIME_SET_UUID								= (eCFG_SVC_UUID + 5),
	eCFG_CHAR_HAND_DETECT_UUID						= (eCFG_SVC_UUID + 6),
	eCFG_CHAR_BTN_PUSH_UUID								= (eCFG_SVC_UUID + 7),
	eCFG_CHAR_ALM_CTL_UUID								= (eCFG_SVC_UUID + 8),
	eCFG_CHAR_DR_UUID								      = (eCFG_SVC_UUID + 9),

	eNOTIFICATION_SVC_UUID								= 0x2000,
	eNOTIFICATION_CHAR_TIME_UUID					= (eNOTIFICATION_SVC_UUID + 1),
	eNOTIFICATION_CHAR_LOG_CNT_UUID				= (eNOTIFICATION_SVC_UUID + 2),
	eNOTIFICATION_CHAR_APP_CURRENT_UUID		= (eNOTIFICATION_SVC_UUID + 3),
	eNOTIFICATION_CHAR_BTN_PRESS_UUID			= (eNOTIFICATION_SVC_UUID + 4),
	eNOTIFICATION_CHAR_ZAP_CNT_UUID				= (eNOTIFICATION_SVC_UUID + 5),
	eNOTIFICATION_CHAR_PIEZO_CNT_UUID			= (eNOTIFICATION_SVC_UUID + 6),
	eNOTIFICATION_CHAR_MOTOR_UUID					= (eNOTIFICATION_SVC_UUID + 7),
	eNOTIFICATION_CHAR_LED_UUID						= (eNOTIFICATION_SVC_UUID + 8),
	eNOTIFICATION_CHAR_SLEEP_DATA_UUID		= (eNOTIFICATION_SVC_UUID + 9),
	eNOTIFICATION_CHAR_ALARM_TIME_UUID		= (eNOTIFICATION_SVC_UUID + 10),

	eLOG_SVC_UUID													= 0x3000,
	eLOG_CHAR_CMD													= (eLOG_SVC_UUID + 1),
	eLOG_CHAR_DATA												= (eLOG_SVC_UUID + 2),

	eTRAINING_SVC_UUID										= 0x4000,
	eTRAINING_CHAR_READ_DATA_UUID					= (eTRAINING_SVC_UUID + 1),
	eTRAINING_CHAR_CONTROL_UUID				    = (eTRAINING_SVC_UUID + 2),

	eAPP_SVC_UUID													= 0x5000,
	eAPP_CHAR_CONTROL_UUID								= (eAPP_SVC_UUID + 1),
	eAPP_CHAR_DOWNLOAD_UUID								= (eAPP_SVC_UUID + 2),
	eAPP_CHAR_ALARM_NOTIFY_UUID					  = (eAPP_SVC_UUID + 3),
	eAPP_CHAR_SNOOZE_NOTIFY_UUID					= (eAPP_SVC_UUID + 4),
	eAPP_CHAR_ALARM_DISABLE_NOTIFY_UUID		= (eAPP_SVC_UUID + 5),

	eOTA_SVC_UUID													= 0x6000,
	eOTA_CHAR_APP_UUID										= (eOTA_SVC_UUID + 1),
	eOTA_CHAR_FIRMWARE_UUID								= (eOTA_SVC_UUID + 2)

} ePLOK_SERVICE_CHAR_UUIDS;


typedef enum
{
    BLE_SERVICE_DISABLED,
    BLE_SERVICE_ENABLED
} eService_t;

#define PAVLOK_BATTERY_LEVEL_3000   (3000)
#define PAVLOK_BATTERY_LEVEL_3100   (3100)
#define PAVLOK_BATTERY_LEVEL_3150   (3150)
#define PAVLOK_BATTERY_LEVEL_3200   (3200)
#define PAVLOK_BATTERY_LEVEL_3250   (3250)
#define PAVLOK_BATTERY_LEVEL_3300   (3300)

/**	----------------------------------------------------------------------
 **	@def PAVLOK log entry defines
 **	----------------------------------------------------------------------
 */
typedef enum
{
	APPSVC_CHAR_OTA,
	APPSVC_CHAR_CONTROL,
	APPSVC_CHAR_ALARM_TRIGGERED,
	APPSVC_CHAR_SNOOZE_SET,
	APPSVC_CHAR_ALARM_DISABLED,


	APPSVC_CHAR_LAST
} eAPPSVC_CHAR_LIST;


typedef struct
{
	bool	piezo;
	bool	motor;
	bool	zap;

} sStimuliStart_t;


typedef struct Node
{
	uint8_t     * entry;
  struct Node * next;

} sNode_t;

typedef enum
{
  APP_CONTROL_START = 0,  // will start the named alarm in the run list
  APP_CONTROL_DELETE,     // delete the named application from the device
  APP_CONTROL_STOP,       // stop the current alarm in the application. The next alarm will auto
  APP_CONTROL_SNOOZE,     // snoozes the current alarm if allowed in the app
  APP_CONTROL_ALARM,      // stops the current alarm if allowed in the app
  APP_CONTROL_LIST,       // sends back to the phone the list of apps stored on the phone at 100ms interval

	APP_CONTROL_LAST

} eAppControl_t;


#ifdef REMOVED

typedef struct
{
	bool						dirty_bit;
	uint32_t				cmd_index;
  sTime_t 				time_setting;
} sRtcInfo_t;
#endif

typedef struct
{
	/**	--------------------------------------------------------------------
	**	sTime_t	pavlok_time;
	**	Is not needed as notification is a read only
	**	So we just use the set time struct from the cfg structure
	**	--------------------------------------------------------------------
	*/
	uint8_t log_count[NOTIFICATION_CHAR_LOG_CNT_LENGTH];
	char		current_app[NOTIFICATION_CHAR_APP_CURRENT_LENGTH];
	uint8_t	count_button_press;
	uint8_t	count_zap;
	uint8_t	count_piezo;
	uint8_t	count_motor;
	uint8_t	count_led;
	uint8_t	sleep_data[NOTIFICATION_CHAR_SLEEP_DATA_LENGTH];

} sNotificationInfo_t;

#ifdef REMOVED
typedef struct
{
	bool		state_hand_detect;
	bool		state_double_tap;
	bool		state_button;
	bool		state_safe_pair;
	bool		state_ssleep_tracking;
	bool		hand_in_use;				// false = right hand, true = left
	bool		double_tap;					// false = zap strength increment, true = record urge by user
	uint8_t	hand_detect_stimulus;
	uint8_t	bp_stimulus;				// TODO what is bp

} sAccelSvcInfo_t;
#endif

typedef struct
{
  uint32_t	perform_action		: 1;
  uint32_t	save_duty_cycle		: 1;
  uint32_t	reserved_1			: 5;
  uint32_t	always				: 1;
  uint32_t	zap_strength		: 8;
  uint32_t	reserved_2			: 5;
  uint32_t	count				: 3;
  uint32_t	reserved_3			: 8;

} sZap_t;

typedef enum
{
	TRAINING_STATE_INACTIVE,
	TRAINING_STATE_ACTIVE,
	TRAINING_STATE_COMPLETE

} eTrainingState_t;

typedef enum
{
	LED_PATTERN_BLINK,
	LED_PATTERN_UP_LIGHTNING_BOLT,
	LED_PATTERN_DOWN_LIGHTNING_BOLT,
	LED_PATTERN_FLASH_ALL,
	LED_PATTERN_REWARD,
	LED_PATTERN_WARNING,

	LED_LAST_PATTERN

} eLed_action_t;

typedef enum
{
	SI_TRAIN,
	SI_NOTIFY,
	SI_CFG,
	SI_LOG_INFO,
	SI_OTA,
	SI_DEVICE_CONTROL,
  SI_APP_INFO,
	SI_LAST_ENTRY

} eServiceInfo_t;


typedef enum
{
	CFG_CHAR_MOTOR,
	CFG_CHAR_PIEZO,
	CFG_CHAR_ZAP,
	CFG_CHAR_LED,
	CFG_CHAR_TIME,
	CFG_CHAR_HD,
	CFG_CHAR_BUTTON,
	CFG_CHAR_DR,
	CFG_CHAR_LAST

} eCFG_CHAR_LIST;

typedef struct
{
	eCFG_CHAR_LIST	characteristic;

	uint8_t	motor_value[PLOK_VALUE_LENGTH_VB_MOTOR];
	uint8_t	piezo_value[PLOK_VALUE_LENGTH_PIEZO];
	uint8_t	zap_me_value[PLOK_VALUE_LENGTH_ZAP];
	uint8_t	led_value[PLOK_VALUE_LENGTH_LED];
	uint8_t	hand_detect_value[PLOK_VALUE_LENGTH_HD];
	uint8_t	button_value[PLOK_VALUE_LENGTH_BUTTON];
	sTime_t	pavlok_time;
// TODO REMOVE	uint8_t	hour_format;

} sCfg_service_t;

typedef struct
{
  char *      name;
  uint16_t    length;
  uint32_t    flash_address;

} sAppInfo_t;


typedef struct
{
  sNode_t * list;
  char    * current;
  uint8_t   count;

} sAppDesc_t;

typedef struct
{
	eTrainingState_t	  training_active;
	eServiceInfo_t		  last_service;
	sNotificationInfo_t	notify;
	sCfg_service_t	    cfg;
// REMOVED	sRtcInfo_t				  rtc;
// REMOVED	sAccelSvcInfo_t 	  accel;
// REMOVED	sDevCtlInfo_t			  dev_ctl;
	uint16_t					  usb_voltage;
	sAppDesc_t          applications;
  ble_gap_addr_t      device_addr;

} sPavlokServiceInfo_t;


typedef enum
{
	ACTION_STATE_OFF,
	ACTION_STATE_ON,
	ACTION_STATE_DELAY,

	ACTION_STATE_LAST	// WARNING:  Always set default action to this state!!
} eAction_state_t;


typedef struct
{
	char				tag[2];
	uint16_t		length;
	uint8_t		*	value;

} sTlv_t;

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
**	@brief	Public Data
**------------------------------------------------------------------------
*/
extern uint8_t              encoded_info[PAVLOK_MAX_MSG_SIZE];

extern const ble_uuid128_t  base_uuid128;

/*------------------------------------------------------------------------
**	@brief	Public Functions
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
typedef void (*taskName_t) (void * arg);
typedef void (* timerCB) (TimerHandle_t xTimer);


extern uint8_t * pavlok_common_get_encode_buffer(void);


void pavlok_timers_init(char * timerTextName, TimerHandle_t * timerId, uint32_t timerInterval, timerCB cb);

void pavlok_application_timers_start(void);


void 				pavlok_encode(uint8_t * value, uint8_t * p_encoded_buffer, uint8_t length);
uint8_t 		* 	main_get_encoded_info(void);
BaseType_t 			pavlok_common_start_task (TaskHandle_t  * m_thread, taskName_t  tName, char * textName, uint32_t stackDepth);

void											pavlok_set_dirty_bit(eServiceInfo_t service, void * service_object);
void 											pavlok_clear_dirty_bit(void);
eServiceInfo_t  					pavlok_get_dirty_bit(void);


sPavlokServiceInfo_t	*		pavlok_get_configuration(void);
void											pavlok_get_time(sTime_t * current_time);
void									*		pavlok_get_p_service_info(eServiceInfo_t service);
uint32_t 									pavlok_button_normalize(uint32_t m_button_time);
void 											pavlok_restart_service_task(void);
void 											get_pavlok_usb_voltage(void);

size_t 										pavlok_strnlen (const char * s, size_t maxlen);

void 											pavlok_led_bit_pattern(LED_T pattern, uint8_t count) ;
void 											pavlok_led_lightning_down(uint8_t count) ;
void 											pavlok_led_lightning_up(uint8_t count);
void 											pavlok_led_reward_pattern(void);
void 											pavlok_led_warning_pattern(void);
void 											pavlok_led_flash_all(uint8_t count);
void											pavlok_training_notifiy_user(void);
void											pavlok_training_start_stimulus(void);
void 											pavlok_training_stop_stimulus(void);
void											pavlok_set_training_started(eTrainingState_t state);
eTrainingState_t					pavlok_get_training_started(void);
void 											pavlok_set_stimulus_on(sNode_t * stimulus);

void                      pavlok_list_insert(sNode_t * list, uint8_t * new_entry);
void                      pavlok_list_append(sNode_t * list, uint8_t * new_entry);
sNode_t *                 pavlok_list_next(sNode_t * list, sNode_t * current_entry);
int                       pavlok_list_find(sNode_t * pointer, uint8_t * new_entry);
void                      pavlok_list_delete(sNode_t * list);
sNode_t *                 pavlok_list_pop(sNode_t * list);
void                      pavlok_set_default_configuration(void);
void                      pavlok_zap_notification(void);
bool                      pavlok_get_usb_pin_state(void);
void                      pavlok_vusb_init(void);




extern uint8_t skbbTxBuffer[4096];

#endif // _PAVLOK_COMMON_H__
