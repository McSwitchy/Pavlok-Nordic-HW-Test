/*------------------------------------------------------------------------
**
**	@file								rtc.h
**
**  @brief Description	data and prototypes to implement the pavlok rtc
**											peripheral chip
**  
**  @note 
**  
**  @note 
**
**------------------------------------------------------------------------
*/
#ifndef _RTC_H_
#define _RTC_H_

/*------------------------------------------------------------------------
**	@brief System Include(s)
**------------------------------------------------------------------------
*/
#include <stdint.h>
#include <string.h>
#include "boards.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"

/**	----------------------------------------------------------------------
**	@brief Project Include(s)
**	----------------------------------------------------------------------
*/
#include "debug.h"
#include "i2c.h"

/**	----------------------------------------------------------------------
**	@brief	Global Data
**	----------------------------------------------------------------------
*/

#define RTC_DEBUG												(0)
#define RED_LED													(9)
#define GREEN_LED												(10)
//#define NEED_TO_CONVERT_TO_BCD          (1)

#define PCF85063A_REG_RANGE_LOW					(0x00)
#define PCF85063A_REG_RANGE_HIGH				(0x11)
#define PCF85063A_REG_SIZE							(1)
#define I2C_PCF85063A_ADDRESS						(0xA2>>1)
#define I2C_RTC_MAX_TRANSFER						(7)
#define RTC_PCF85063A_ADDRESS_SIZE			(1)
#define RTC_PCF85063A_REGISTER_SIZE			(1)
#define QUARTZ_7PF 											(0)
#define QUARTZ_12_POINT_5_PF 						(1)
#if (!QUARTZ_7PF && !QUARTZ_12_POINT_5_PF)
#error Must define quartz load capacitance!
#endif
#if (QUARTZ_7PF && QUARTZ_12_POINT_5_PF)
#error Must only define one quartz load capacitance!
#endif

#if RTC_DEBUG
#define RTC_INT_PIN 										(11)
#else
#define RTC_INT_PIN 										(15)
#endif

// BCD definitions
#define HI_NIBBLE(b) 										(((b) >> 4) & 0x0F)
#define LO_NIBBLE(b) 										((b) & 0x0F)

// X Macros
#define DAYS_TABLE \
X(SUNDAY, 		"Sunday") \
X(MONDAY, 		"Monday") \
X(TUESDAY, 		"Tuesday") \
X(WEDNESDAY, 	"Wednesday") \
X(THURSDAY,		"Thursday") \
X(FRIDAY,			"Friday") \
X(SATURDAY,		"Saturday") 

/**	----------------------------------------------------------------------
**	@typedef	rtcTXRXPacket_T i2c packet
**	----------------------------------------------------------------------
*/
typedef struct 
{
	uint8_t address;
	uint8_t data[I2C_RTC_MAX_TRANSFER];
} rtcTXRXPacket_T;

/**	----------------------------------------------------------------------
**	@typedef	RTC_TIME_STRUCT_T and sTime_t for system time
**	----------------------------------------------------------------------
*/
typedef struct 
{
	int8_t seconds;
	int8_t minutes;
	int8_t hours;
	int8_t days;
	int8_t weekdays;
	int8_t months;
	int8_t years;
} RTC_TIME_STRUCT_T;
typedef RTC_TIME_STRUCT_T sTime_t;

/**	----------------------------------------------------------------------
**	@enum	RTC_INIT_RET_T
**	----------------------------------------------------------------------
*/
typedef enum 
{
	RTC_INIT_INIT_SUCCESS,
	RTC_INIT_FAIL
} RTC_INIT_RET_T;

/**	----------------------------------------------------------------------
**	@enum	RTC_TXRX_RET_T
**	----------------------------------------------------------------------
*/
typedef enum
{
	RTC_SUCCESS,
	RTC_NULL_DATA,
	RTC_INVALID_ADDRESS,
	RTC_INVALID_SIZE,
	RTC_TX_ERROR		
} RTC_TXRX_RET_T;

/**	----------------------------------------------------------------------
**	@enum	RTC_HOUR_FORMAT_T
**	----------------------------------------------------------------------
*/
typedef enum
{
	FORMAT_24HR,
	FORMAT_12HR
} RTC_HOUR_FORMAT_T;

/**	----------------------------------------------------------------------
**	@brief	Global Extern(s)
**	----------------------------------------------------------------------
*/
extern uint8_t intSemaphore;

/**	----------------------------------------------------------------------
**	@brief	Global Functions
**	----------------------------------------------------------------------
*/

/**	----------------------------------------------------------------------
**
**	@fn		Function			rtc_init
**
**	@brief	Description	rtc chip initialization
**
**	@param [in]					None
**
**	@param	[out]				None
**
**	@return							RTC_INIT_RET_T
**
**	@retval
**
**	@warn
**
**	----------------------------------------------------------------------
*/
RTC_INIT_RET_T rtc_init(void);

/**	----------------------------------------------------------------------
**
**	@fn		Function			rtc_get_hour_format
**
**	@brief	Description	return the 12/24 hour format for the part
**
**	@param [in]					RTC_HOUR_FORMAT_T * format 
**
**	@param	[out]				None
**
**	@return							RTC_TXRX_RET_T
**
**	@retval
**
**	@warn
**
**	----------------------------------------------------------------------
*/
RTC_TXRX_RET_T rtc_get_hour_format(RTC_HOUR_FORMAT_T * format);

/**	----------------------------------------------------------------------
**
**	@fn		Function			rtc_get_time
**
**	@brief	Description	get the pavlok system time
**
**	@param [in]					RTC_TIME_STRUCT_T * getTime - pointer to the time
**											struct to copy the data into
**
**	@param	[out]				None
**
**	@return							RTC_TXRX_RET_T
**
**	@retval
**
**	@warn								All data is returned in BCD format
**
**	----------------------------------------------------------------------
*/
RTC_TXRX_RET_T rtc_get_time(RTC_TIME_STRUCT_T *getTime);

/**	----------------------------------------------------------------------
**
**	@fn		Function			rtc_set_time
**
**	@brief	Description	function to set pavlok system time
**
**	@param [in]					RTC_TIME_STRUCT_T * setTime - pointer to time data
**											7 bytes
**
**	@param	[out]				None
**
**	@return							RTC_TXRX_RET_T
**
**	@retval
**
**	@warn								Currently PAVLOK only uses 24 our format
**	@warn								All data must be verified before calling this
**											function (i.e. that means you must query the 
**											hour format to validate your input data
**
**	----------------------------------------------------------------------
*/
RTC_TXRX_RET_T rtc_set_time(RTC_TIME_STRUCT_T * setTime);

/**	----------------------------------------------------------------------
**
**	@fn		Function			rtc_set_alarm
**
**	@brief	Description	set the next alarm to the second
**
**	@param [in]					RTC_TIME_STRUCT_T * alarmTime
**
**	@param	[out]				None
**
**	@return							RTC_TXRX_RET_T
**
**	@retval
**
**	@warn
**
**	----------------------------------------------------------------------
*/
RTC_TXRX_RET_T rtc_set_alarm(RTC_TIME_STRUCT_T *alarmTime);

/**	----------------------------------------------------------------------
**
**	@fn		Function			rtc_delete_alarm
**
**	@brief	Description	delete the enabled alarm
**
**	@param [in]					None
**
**	@param	[out]				None
**
**	@return							RTC_TXRX_RET_T
**
**	@retval
**
**	@warn
**
**	----------------------------------------------------------------------
*/
RTC_TXRX_RET_T rtc_delete_alarm(void);

/**	----------------------------------------------------------------------
**
**	@fn		Function			rtc_clear_int
**
**	@brief	Description	clear the pending/active interrupt
**
**	@param [in]					None
**
**	@param	[out]				None
**
**	@return							RTC_TXRX_RET_T
**
**	@retval
**
**	@warn
**
**	----------------------------------------------------------------------
*/
RTC_TXRX_RET_T rtc_clear_int(void);

/**	----------------------------------------------------------------------
**
**	@fn		Function			rtc_write
**
**	@brief	Description	write the rtc register set
**
**	@param [in]					uint8_t addr - register address
**	@param [in]					uint8_t * data - register value
**	@param [in]					uint8_t number of registers
**
**	@param	[out]				None
**
**	@return							RTC_TXRX_RET_T
**
**	@retval
**
**	@warn
**
**	----------------------------------------------------------------------
*/
RTC_TXRX_RET_T rtc_write(uint8_t addr, uint8_t *data, uint8_t length);

/**	----------------------------------------------------------------------
**
**	@fn		Function			rtc_read
**
**	@brief	Description	read rtc registers
**
**	@param [in]					uint8_t addr - register address
**	@param [in]					uint8_t * data - register value
**	@param [in]					uint8_t number of registers
**
**	@param	[out]				None
**
**	@return							RTC_TXRX_RET_T
**
**	@retval
**
**	@warn
**
**	----------------------------------------------------------------------
*/
RTC_TXRX_RET_T rtc_read(uint8_t addr, uint8_t *data, uint8_t length);

/**	----------------------------------------------------------------------
**
**	@fn		Function			rtc_software_reset
**
**	@brief	Description	perform software reset
**
**	@param [in]					None
**
**	@param	[out]				None
**
**	@return							RTC_TXRX_RET_T
**
**	@retval
**
**	@warn
**
**	----------------------------------------------------------------------
*/
RTC_TXRX_RET_T rtc_software_reset(void);
void rtc_enable_int(void) ;
RTC_TXRX_RET_T rtc_get_alarm(RTC_TIME_STRUCT_T *alarmTime);

/** ----------------------------------------------------------------------
**	@def	PCF85063A internal register addresses
**	----------------------------------------------------------------------
*/
#define PCF85063A_CONTROL1						(0x00)
#define PCF85063A_CONTROL2						(0x01)
#define	PCF85063A_OFFSET							(0x02)
#define PCF85063A_RAM									(0x03)
#define PCF85063A_SECONDS							(0x04)
#define PCF85063A_MINUTES							(0x05)
#define PCF85063A_HOURS								(0x06)
#define PCF85063A_DAYS								(0x07)
#define PCF85063A_WEEKDAYS						(0x08)
#define PCF85063A_MONTHS							(0x09)
#define PCF85063A_YEARS								(0x0A)
#define PCF85063A_SECOND_ALARM				(0x0B)
#define PCF85063A_MINUTE_ALARM				(0x0C)
#define PCF85063A_HOUR_ALARM					(0x0D)
#define PCF85063A_DAY_ALARM						(0x0E)
#define PCF85063A_WEEKDAY_ALARM				(0x0F)
#define PCF85063A_TIMER_VALUE					(0x10)
#define PCF85063A_TIMER_MODE					(0x11)

// Register Macros
#define CONTROL1_CAP_SEL_SHIFT				0
#define CONTROL1_CAP_SEL_MASK					0x01
#define CONTROL1_CAP_SEL(x)						(((uint8_t)(((uint8_t)(x))<<CONTROL1_CAP_SEL_SHIFT))&CONTROL1_CAP_SEL_MASK)

#define CONTROL1_12_24_SHIFT					1
#define CONTROL1_12_24_MASK						0x02
#define CONTROL1_12_24(x)							(((uint8_t)(((uint8_t)(x))<<CONTROL1_12_24_SHIFT))&CONTROL1_12_24_MASK)

#define CONTROL1_CIE_SHIFT						2
#define CONTROL1_CIE_MASK							0x04
#define CONTROL1_CIE(x)								(((uint8_t)(((uint8_t)(x))<<CONTROL1_CIE_SHIFT))&CONTROL1_CIE_MASK)

#define CONTROL1_SR_SHIFT							4
#define CONTROL1_SR_MASK							0x10
#define CONTROL1_SR(x)								(((uint8_t)(((uint8_t)(x))<<CONTROL1_SR_SHIFT))&CONTROL1_SR_MASK)

#define CONTROL1_STOP_SHIFT						5
#define CONTROL1_STOP_MASK						0x20
#define CONTROL1_STOP(x)							(((uint8_t)(((uint8_t)(x))<<CONTROL1_STOP_SHIFT))&CONTROL1_STOP_MASK)

#define CONTROL1_EXT_TEST_SHIFT				7
#define CONTROL1_EXT_TEST_MASK				0x80
#define CONTROL1_EXT_TEST(x)					(((uint8_t)(((uint8_t)(x))<<CONTROL1_EXT_TEST_SHIFT))&CONTROL1_EXT_TEST_MASK)

#define CONTROL2_COF_SHIFT						0
#define CONTROL2_COF_MASK							0x07
#define CONTROL2_COF_SEL(x)						(((uint8_t)(((uint8_t)(x))<<CONTROL2_COF_SHIFT))&CONTROL2_COF_MASK)

#define CONTROL2_TF_SHIFT							3
#define CONTROL2_TF_MASK							0x08
#define CONTROL2_TF(x)								(((uint8_t)(((uint8_t)(x))<<CONTROL2_TF_SHIFT))&CONTROL2_TF_MASK)

#define CONTROL2_HMI_SHIFT						4
#define CONTROL2_HMI_MASK							0x10
#define CONTROL2_HMI(x)								(((uint8_t)(((uint8_t)(x))<<CONTROL2_HMI_SHIFT))&CONTROL2_HMI_MASK)

#define CONTROL2_MI_SHIFT							5
#define CONTROL2_MI_MASK							0x20
#define CONTROL2_MI(x)								(((uint8_t)(((uint8_t)(x))<<CONTROL2_MI_SHIFT))&CONTROL2_MI_MASK)

#define CONTROL2_AF_SHIFT							6
#define CONTROL2_AF_MASK							0x40
#define CONTROL2_AF(x)								(((uint8_t)(((uint8_t)(x))<<CONTROL2_AF_SHIFT))&CONTROL2_AF_MASK)

#define CONTROL2_AIE_SHIFT						7
#define CONTROL2_AIE_MASK							0x80
#define CONTROL2_AIE(x)								(((uint8_t)(((uint8_t)(x))<<CONTROL2_AIE_SHIFT))&CONTROL2_AIE_MASK)

#define SECONDS_SECONDS_SHIFT					0
#define SECONDS_SECONDS_MASK					0x7F
#define SECONDS_SECONDS(x)						(((uint8_t)(((uint8_t)(x))<<SECONDS_SECONDS_SHIFT))&SECONDS_SECONDS_MASK)

#define SECONDS_OS_SHIFT							7
#define SECONDS_OS_MASK								0x80
#define SECONDS_OS_SECONDS(x)					(((uint8_t)(((uint8_t)(x))<<SECONDS_OS_SHIFT))&SECONDS_OS_MASK)

#define MINUTES_MINUTES_SHIFT					0
#define MINUTES_MINUTES_MASK					0x7F
#define MINUTES_MINUTES(x)						(((uint8_t)(((uint8_t)(x))<<MINUTES_MINUTES_SHIFT))&MINUTES_MINUTES_MASK)

#define HOURS_HOURS_12_SHIFT					0
#define HOURS_HOURS_12_MASK						0x1F
#define HOURS_HOURS_12(x)							(((uint8_t)(((uint8_t)(x))<<HOURS_HOURS_12_SHIFT))&HOURS_HOURS_12_MASK)

#define HOURS_AMPM_SHIFT							5
#define HOURS_AMPM_MASK								0x20
#define HOURS_AMPM(x)									(((uint8_t)(((uint8_t)(x))<<HOURS_AMPM_SHIFT))&HOURS_AMPM_MASK)

#define HOURS_HOURS_24_SHIFT					0
#define HOURS_HOURS_24_MASK						0x3F
#define HOURS_HOURS_24(x)							(((uint8_t)(((uint8_t)(x))<<HOURS_HOURS_24_SHIFT))&HOURS_HOURS_24_MASK)

#define DAYS_DAYS_SHIFT								0
#define DAYS_DAYS_MASK								0x3F
#define DAYS_DAYS(x)									(((uint8_t)(((uint8_t)(x))<<DAYS_DAYS_SHIFT))&DAYS_DAYS_MASK)

#define WEEKDAYS_WEEKDAYS_SHIFT				0
#define WEEKDAYS_WEEKDAYS_MASK				0x07
#define WEEKDAYS_WEEKDAYS(x)					(((uint8_t)(((uint8_t)(x))<<WEEKDAYS_WEEKDAYS_SHIFT))&WEEKDAYS_WEEKDAYS_MASK)

#define MONTHS_MONTHS_SHIFT						0	
#define MONTHS_MONTHS_MASK						0x1F
#define MONTHS_MONTHS(x)							(((uint8_t)(((uint8_t)(x))<<MONTHS_MONTHS_SHIFT))&MONTHS_MONTHS_MASK)

#define YEARS_YEARS_SHIFT							0
#define YEARS_YEARS_MASK							0xFF
#define YEARS_YEARS(x)								(((uint8_t)(((uint8_t)(x))<<YEARS_YEARS_SHIFT))&YEARS_YEARS_MASK)

#define SECONDS_ALARM_SECONDS_SHIFT		0
#define SECONDS_ALARM_SECONDS_MASK		0x7F
#define SECONDS_ALARM_SECONDS(x)			(((uint8_t)(((uint8_t)(x))<<SECONDS_ALARM_SECONDS_SHIFT))&SECONDS_ALARM_SECONDS_MASK)

#define SECONDS_ALARM_AENS_SHIFT			7
#define SECONDS_ALARM_AENS_MASK				0x80
#define SECONDS_ALARM_AENS(x)					(((uint8_t)(((uint8_t)(x))<<SECONDS_ALARM_AENS_SHIFT))&SECONDS_ALARM_AENS_MASK)

#define MINUTES_ALARM_MINUTES_SHIFT		0
#define MINUTES_ALARM_MINUTES_MASK		0x7F
#define MINUTES_ALARM_MINUTES(x)			(((uint8_t)(((uint8_t)(x))<<MINUTES_ALARM_MINUTES_SHIFT))&MINUTES_ALARM_MINUTES_MASK)

#define MINUTES_ALARM_AENM_SHIFT			7
#define MINUTES_ALARM_AENM_MASK				0x80
#define MINUTES_ALARM_AENM(x)					(((uint8_t)(((uint8_t)(x))<<MINUTES_ALARM_AENM_SHIFT))&MINUTES_ALARM_AENM_MASK)

#define HOURS_ALARM_HOURS_12_SHIFT		0
#define HOURS_ALARM_HOURS_12_MASK			0x1F
#define HOURS_ALARM_HOURS_12(x)				(((uint8_t)(((uint8_t)(x))<<HOURS_ALARM_HOURS_12_SHIFT))&HOURS_ALARM_HOURS_12_MASK)

#define HOURS_ALARM_AMPM_SHIFT				5
#define HOURS_ALARM_AMPM_MASK					0x20
#define HOURS_ALARM_AMPM(x)						(((uint8_t)(((uint8_t)(x))<<HOURS_ALARM_AMPM_SHIFT))&HOURS_ALARM_AMPM_MASK)

#define HOURS_ALARM_HOURS_24_SHIFT		0
#define HOURS_ALARM_HOURS_24_MASK			0x3F
#define HOURS_ALARM_HOURS_24(x)				(((uint8_t)(((uint8_t)(x))<<HOURS_ALARM_HOURS_24_SHIFT))&HOURS_ALARM_HOURS_24_MASK)

#define HOURS_ALARM_AENH_SHIFT				7
#define HOURS_ALARM_AENH_MASK					0x80
#define HOURS_ALARM_AENH(x)						(((uint8_t)(((uint8_t)(x))<<HOURS_ALARM_AENH_SHIFT))&HOURS_ALARM_AENH_MASK)	

#define DAYS_ALARM_DAYS_SHIFT					0
#define DAYS_ALARM_DAYS_MASK					0x3F
#define DAYS_ALARM_DAYS(x)						(((uint8_t)(((uint8_t)(x))<<DAYS_ALARM_DAYS_SHIFT))&DAYS_ALARM_DAYS_MASK)

#define DAYS_ALARM_AEND_SHIFT					7
#define DAYS_ALARM_AEND_MASK					0x80
#define DAYS_ALARM_AEND(x)						(((uint8_t)(((uint8_t)(x))<<DAYS_ALARM_AEND_SHIFT))&DAYS_ALARM_AEND_MASK)

#define WEEKDAYS_ALARM_WEEKDAYS_SHIFT	0
#define WEEKDAYS_ALARM_WEEKSDAYS_MASK	0x07
#define WEEKDAYS_ALARM_WEEKDAYS(x)		(((uint8_t)(((uint8_t)(x))<<WEEKDAYS_ALARM_WEEKDAYS_SHIFT))&WEEKDAYS_ALARM_WEEKSDAYS_MASK)

#define WEEKDAYS_ALARM_AENW_SHIFT			7
#define WEEKDAYS_ALARM_AENW_MASK			0x80
#define WEEKDAYS_ALARM_AENW(x)				(((uint8_t)(((uint8_t)(x))<<WEEKDAYS_ALARM_AENW_SHIFT))&WEEKDAYS_ALARM_AENW_MASK)

#endif
