/*------------------------------------------------------------------------
**
**	@file			rtc.c
**
**  @defgroup PAVLOK_RTC
**  @{
**  @ingroup 	PAVLOK_RTC
**  @brief Pavlok RTC chip function
**  
**  @details 	This module implements the functions for the PAVLOK rtc
**						peripheral
**  
**  @note The application must 
**  
**  @note Attention! time data to/from the user is in BCD format
**   
**
**------------------------------------------------------------------------
*/

/*------------------------------------------------------------------------
**	@brief System Include(s)
**------------------------------------------------------------------------
*/

/**	----------------------------------------------------------------------
**	@brief Project Include(s)
**	----------------------------------------------------------------------
*/
#include "rtc.h"
#include "application_task.h"

/**	----------------------------------------------------------------------
**	@brief	Global Data
**	----------------------------------------------------------------------
*/
#define X(a, b) a, 
typedef enum {
	DAYS_TABLE
} WEEKDAY_T;
#undef X

#define X(a, b) b,
char *days[] = {
	DAYS_TABLE	
};
#undef X

/**	----------------------------------------------------------------------
**	@brief	Global Extern(s)
**	----------------------------------------------------------------------
*/
uint8_t intSemaphore = 0;

/**	----------------------------------------------------------------------
**	@brief	Static Data
**	----------------------------------------------------------------------
*/
// The default interrupt behavior of the RTC is active-low
static const nrf_drv_gpiote_in_config_t configRTCInt = 
 {
	.is_watcher = false,
	.hi_accuracy = false,
	.pull = NRF_GPIO_PIN_PULLUP,
	.sense = NRF_GPIOTE_POLARITY_HITOLO
};
 
/**	----------------------------------------------------------------------
**	@brief	Static Forward References
**	----------------------------------------------------------------------
*/
/**	----------------------------------------------------------------------
**	@brief	Static Functions
**	----------------------------------------------------------------------
*/
static void rtc_input_int_init(void);
static void rtc_pin_handler_int(nrf_drv_gpiote_pin_t, nrf_gpiote_polarity_t);
#ifdef NEED_TO_CONVERT_TO_BCD
static uint8_t rtc_get_seconds(uint8_t);
static uint8_t rtc_get_minutes(uint8_t);
static uint8_t rtc_get_hours(uint8_t);
static uint8_t rtc_get_days(uint8_t);
static uint8_t rtc_get_weekdays(uint8_t);
static uint8_t rtc_get_months(uint8_t);
static uint8_t rtc_get_years(uint8_t);
static uint8_t rtc_set_seconds(uint8_t);
static uint8_t rtc_set_minutes(uint8_t);
static uint8_t rtc_set_hours(uint8_t);
static uint8_t rtc_set_days(uint8_t);
static uint8_t rtc_set_weekdays(uint8_t);
static uint8_t rtc_set_months(uint8_t);
static uint8_t rtc_set_years(uint8_t);
#endif


/**	----------------------------------------------------------------------
**
**	@fn		Function			rtc_input_int_init
**
**	@brief	Description	rtc init function
**
**	@param [in]					None
**
**	@param	[out]				None
**
**	@return							None
**
**	@retval
**
**	@warn
**
**	----------------------------------------------------------------------
*/
static void rtc_input_int_init(void) 
{
	// First, init GPIOTE if it isn't already initialized
	if (!nrf_drv_gpiote_is_init()) 
  {
	  UNUSED_RETURN_VALUE(nrf_drv_gpiote_init());
	}
	// Set designated pin used for interrupt and specify the call-back
	// Int 1
	UNUSED_RETURN_VALUE(nrf_drv_gpiote_in_init(RTC_INT_PIN, &configRTCInt, rtc_pin_handler_int));
	nrf_drv_gpiote_in_event_enable(RTC_INT_PIN, true);
}

/**	----------------------------------------------------------------------
**
**	@fn		Function			rtc_pin_handler_int
**
**	@brief	Description	rtcv interrupt handler
**
**	@param [in]					nrf_drv_gpiote_pin_t pin - i2c pin
**	@param [in]					nrf_gpiote_polarity_t action
**
**	@param	[out]				None
**
**	@return							None
**
**	@retval
**
**	@warn
**
**	----------------------------------------------------------------------
*/
static void rtc_pin_handler_int(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) 
{
	if ((pin == RTC_INT_PIN) && (action == NRF_GPIOTE_POLARITY_HITOLO)) 
  {
		//application_start_stimulus();
	}
}

/**	----------------------------------------------------------------------
**	@brief	Global Functions
**	----------------------------------------------------------------------
*/

/** ----------------------------------------------------------------------
**
**	@fn		Function				rtc_clear_int
**
**	@brief	Description		clear the current interrupt
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
RTC_TXRX_RET_T rtc_clear_int(void) 
{
	uint8_t byte = 0;
  
	// Read status of control 2 register
	if (rtc_read(PCF85063A_CONTROL2, &byte, 1) != RTC_SUCCESS) { return RTC_TX_ERROR; }
  
	// Clear the alarm bit
	byte |= CONTROL2_AF(0);
	
	// Clear interrupt
	if (rtc_write(PCF85063A_CONTROL2, &byte, 1) != RTC_SUCCESS) { return RTC_TX_ERROR; }
	
	return RTC_SUCCESS;
}

/**	----------------------------------------------------------------------
**
**	@fn		Function			rtc_enable_int
**
**	@brief	Description	enable interrupt function
**
**	@param [in]					None
**
**	@param	[out]				None
**
**	@return							None
**
**	@retval
**
**	@warn
**
**	----------------------------------------------------------------------
*/
void rtc_enable_int(void) 
{
	nrf_drv_gpiote_in_event_enable(RTC_INT_PIN, true);
}

/** ----------------------------------------------------------------------
**
**	@fn		Function				rtc_write
**
**	@brief	Description		write the rtc registers
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
RTC_TXRX_RET_T rtc_write(uint8_t addr, uint8_t *data, uint8_t length) 
{
	ret_code_t errCode;
	rtcTXRXPacket_T rtcTXRXPacket;
	
	if (data == NULL) 
  {
		return RTC_NULL_DATA;
	}
	if (addr > PCF85063A_REG_RANGE_HIGH)
  {
		return RTC_INVALID_ADDRESS;
	}
	if (length > I2C_RTC_MAX_TRANSFER) 
  {
		return RTC_INVALID_SIZE;
	}
	rtcTXRXPacket.address = addr;
	memcpy(&rtcTXRXPacket.data[0], data, length);
	
	errCode = i2c0_sync_write(I2C_PCF85063A_ADDRESS, (uint8_t *)&rtcTXRXPacket, RTC_PCF85063A_ADDRESS_SIZE+length);
	
	if (NRF_SUCCESS == errCode) 
  {
		return RTC_SUCCESS;
	} else {
		return RTC_TX_ERROR;	
	}
}

/** ----------------------------------------------------------------------
**
**	@fn		Function				rtc_read
**
**	@brief	Description		read the rtc registers
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
RTC_TXRX_RET_T rtc_read(uint8_t addr, uint8_t *data, uint8_t length)
{
	ret_code_t errCode;
	rtcTXRXPacket_T rtcTXRXPacket;
	
	if (addr > PCF85063A_REG_RANGE_HIGH) 
  {
		return RTC_INVALID_ADDRESS;
	}
	if (length > I2C_RTC_MAX_TRANSFER) 
  {
		return RTC_INVALID_SIZE;
	}
	memset(&rtcTXRXPacket.data[0], I2C_RTC_MAX_TRANSFER, 0);
	rtcTXRXPacket.address = addr;
	
	errCode = i2c0_sync_read(I2C_PCF85063A_ADDRESS, (uint8_t *)&rtcTXRXPacket, RTC_PCF85063A_ADDRESS_SIZE, data, length);
	
	if (NRF_SUCCESS == errCode) 
  {
		return RTC_SUCCESS;
	} else 
  {
		return RTC_TX_ERROR;	
	}		
}

/** ----------------------------------------------------------------------
**
**	@fn		Function				rtc_init
**
**	@brief	Description		initialize the rtc
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
#define RAM_TEST_BYTE	(0xAA)
RTC_INIT_RET_T rtc_init(void) {
	RTC_INIT_RET_T ret = RTC_INIT_INIT_SUCCESS;
	uint8_t byteOut = 0;
	uint8_t byteIn = 0;
	
	UART_DEBUG("Entered: %s\r\n", __func__);

	// Test to see if RTC is on bus by writing RAM register and reading it back.
	byteOut = RAM_TEST_BYTE;
	if (rtc_write(PCF85063A_RAM, &byteOut, 1) != RTC_SUCCESS) { return RTC_INIT_INIT_SUCCESS; }
	if (rtc_read(PCF85063A_RAM, &byteIn, 1) != RTC_SUCCESS) { return RTC_INIT_FAIL; }
	
	if (byteIn == RAM_TEST_BYTE) 
  {
		//UART_DEBUG("SUCCESS: Found PCF85063A RTC on bus!\r\n");
		return 1;
    //rtc_software_reset();
    //nrf_delay_us(250);  
    // Set interrupt pin and handler for nordic
    //rtc_input_int_init();
	} 
  else 
  {
		//UART_DEBUG("ERROR: Did NOT find PCF85063A RTC on bus!\r\n");
		//ret = RTC_INIT_FAIL;
    return 0;
	}
	
	//return ret;
}

/** ----------------------------------------------------------------------
**
**	@fn		Function				rtc_get_time
**
**	@brief	Description		get the current pavlok system time
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
#define SIZE_OF_ALL_TIME_REGISTERS (7)
RTC_TXRX_RET_T rtc_get_time(RTC_TIME_STRUCT_T *getTime)
{
	uint8_t buffer[SIZE_OF_ALL_TIME_REGISTERS];
	memset(buffer, SIZE_OF_ALL_TIME_REGISTERS, 0);
	
	UART_DEBUG("Entered: %s\r\n", __func__);
	
	if (rtc_read(PCF85063A_SECONDS, buffer, SIZE_OF_ALL_TIME_REGISTERS) != RTC_SUCCESS) { return RTC_TX_ERROR; }
  
#ifdef NEED_TO_CONVERT_TO_BCD  
	getTime->seconds 	= rtc_get_seconds(SECONDS_SECONDS(buffer[0]));
	getTime->minutes 	= rtc_get_minutes(buffer[1]);
	getTime->hours 		= rtc_get_hours(buffer[2]);
	getTime->days 		= rtc_get_days(buffer[3]);
	getTime->weekdays = rtc_get_weekdays(buffer[4]);
	getTime->months 	= rtc_get_months(buffer[5]);
	getTime->years 		= rtc_get_years(buffer[6]);
#else	
  // We need to strip off any extra bits in the bytes
	getTime->seconds 	= SECONDS_SECONDS(buffer[0]);
	getTime->minutes 	= buffer[1];
	getTime->hours 		= buffer[2];
	getTime->days 		= buffer[3];
	getTime->weekdays = buffer[4];
	getTime->months 	= buffer[5];
	getTime->years 		= buffer[6];
#endif

	// Print time
	UART_DEBUG("Current Time: %-5s %02d-%02d-%02d %02d:%02d:%02d\r\n", 
		days[getTime->weekdays], getTime->years, getTime->months, getTime->days,
		getTime->hours, getTime->minutes, getTime->seconds);
	
	return RTC_SUCCESS;
}

/** ----------------------------------------------------------------------
**
**	@fn		  Function			rtc_set_time
**
**	@brief	Description		set the current pavlok system time
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
RTC_TXRX_RET_T rtc_set_time(RTC_TIME_STRUCT_T *setTime) {
	uint8_t buffer[SIZE_OF_ALL_TIME_REGISTERS];
	uint8_t byte = 0;
	memset(buffer, SIZE_OF_ALL_TIME_REGISTERS, 0);
	
	UART_DEBUG("Entered: %s\r\n", __func__);
	
	// Print Time
	UART_DEBUG("Set Time: %-5s %02d-%02d-%02d %02d:%02d:%02d\r\n", 
		days[setTime->weekdays], setTime->years, setTime->months, setTime->days,
		setTime->hours, setTime->minutes, setTime->seconds);
	
	// Set format and stop clock
	byte = 0;
	byte = CONTROL1_12_24(0) | CONTROL1_STOP(1) | CONTROL1_CAP_SEL(1);
	
	UART_DEBUG("%-15s 0x%02X\r\n", "CONTROL1:", byte);
	
	if (rtc_write(PCF85063A_CONTROL1, &byte, 1) != RTC_SUCCESS) { return RTC_TX_ERROR; }
			
#ifdef NEED_TO_CONVERT_TO_BCD
	buffer[0] = rtc_set_seconds(setTime->seconds);
	SECONDS_SECONDS(buffer[0]);
	
	buffer[1] = rtc_set_minutes(setTime->minutes);
	MINUTES_MINUTES(buffer[1]);
	
	buffer[2] = rtc_set_hours(setTime->hours);
	HOURS_HOURS_24(buffer[2]);
	
	buffer[3] = rtc_set_days(setTime->days);
	DAYS_DAYS(buffer[3]);
	
	buffer[4] = rtc_set_weekdays(setTime->weekdays);
	WEEKDAYS_WEEKDAYS(buffer[4]);
	
	buffer[5] = rtc_set_months(setTime->months);
	MONTHS_MONTHS(buffer[5]);
	
	buffer[6] = rtc_set_years(setTime->years);	
	YEARS_YEARS(buffer[6]);
#else
	buffer[0] = SECONDS_SECONDS(setTime->seconds);
	buffer[1] = MINUTES_MINUTES(setTime->minutes);
	buffer[2] = HOURS_HOURS_24(setTime->hours);
	buffer[3] = DAYS_DAYS(setTime->days);
	buffer[4] = WEEKDAYS_WEEKDAYS(setTime->weekdays);
	buffer[5] = MONTHS_MONTHS(setTime->months);
	buffer[6] = YEARS_YEARS(setTime->years);
#endif

	UART_DEBUG("%-15s 0x%02X\r\n", "SECONDS:", 	buffer[0]);
	UART_DEBUG("%-15s 0x%02X\r\n", "MINUTES:", 	buffer[1]);
	UART_DEBUG("%-15s 0x%02X\r\n", "HOURS:", 		buffer[2]);
	UART_DEBUG("%-15s 0x%02X\r\n", "DAYS:", 		buffer[3]);
	UART_DEBUG("%-15s 0x%02X\r\n", "WEEKDAYS:", buffer[4]);
	UART_DEBUG("%-15s 0x%02X\r\n", "MONTHS:", 	buffer[5]);
	UART_DEBUG("%-15s 0x%02X\r\n", "YEARS:", 		buffer[6]);
	
	if (rtc_write(PCF85063A_SECONDS, buffer, SIZE_OF_ALL_TIME_REGISTERS) != RTC_SUCCESS) { return RTC_TX_ERROR; }
	
	// Start clock
	byte = 0;
	byte = CONTROL1_STOP(0);
	
	UART_DEBUG("%-15s 0x%02X\r\n", "CONTROL1:", byte);
	
	if (rtc_write(PCF85063A_CONTROL1, &byte, 1) != RTC_SUCCESS) { return RTC_TX_ERROR; }
	
	return RTC_SUCCESS;
}
#undef SIZE_OF_ALL_TIME_REGISTERS

/** ----------------------------------------------------------------------
**
**	@fn		Function				rtc_set_alarm
**
**	@brief	Description		set rtc alarm
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
#define SIZE_OF_ALL_ALARM_REGISTERS (5)
RTC_TXRX_RET_T rtc_set_alarm(RTC_TIME_STRUCT_T *alarmTime) 
{
	uint8_t buffer[SIZE_OF_ALL_ALARM_REGISTERS];
	uint8_t byte = 0;
	memset(buffer, SIZE_OF_ALL_ALARM_REGISTERS, 0);
	
	UART_DEBUG("Entered: %s\r\n", __func__);
	
	// Print Time
	UART_DEBUG("Set Alarm: %-5s %02d-%02d-%02d %02d:%02d:%02d\r\n", 
		days[alarmTime->weekdays], alarmTime->years, alarmTime->months, alarmTime->days,
		alarmTime->hours, alarmTime->minutes, alarmTime->seconds);
	
	// AENx bit: 1 to disable alarm, 0 to enable
#ifdef NEED_TO_CONVERT_TO_BCD
	buffer[0] = rtc_set_seconds(alarmTime->seconds);
	buffer[0] = SECONDS_ALARM_SECONDS(buffer[0]) | SECONDS_ALARM_AENS(0);
	
	buffer[1] = rtc_set_minutes(alarmTime->minutes);
	buffer[1] = MINUTES_ALARM_MINUTES(buffer[1]) | MINUTES_ALARM_AENM(0);
	
	buffer[2] = rtc_set_hours(alarmTime->hours);
  buffer[2] = HOURS_ALARM_HOURS_24(buffer[2]) | HOURS_ALARM_AENH(0);

	buffer[3] = rtc_set_days(alarmTime->days);
	buffer[3] = DAYS_ALARM_DAYS(buffer[3]) | DAYS_ALARM_AEND(1);
	
	buffer[4] = rtc_set_weekdays(alarmTime->weekdays);
	buffer[4] = WEEKDAYS_ALARM_WEEKDAYS(buffer[4]) | WEEKDAYS_ALARM_AENW(1);
#else
  buffer[0] = SECONDS_SECONDS(alarmTime->seconds);
	buffer[0] = SECONDS_ALARM_SECONDS(buffer[0]) | SECONDS_ALARM_AENS(0);
	
  buffer[1] = MINUTES_MINUTES(alarmTime->minutes);
	buffer[1] = MINUTES_ALARM_MINUTES(buffer[1]) | MINUTES_ALARM_AENM(0);
  
  buffer[2] = HOURS_HOURS_24(alarmTime->hours);
	buffer[2] = HOURS_ALARM_HOURS_24(buffer[2]) | HOURS_ALARM_AENH(0);

  buffer[3] = DAYS_DAYS(alarmTime->days);
	buffer[3] = DAYS_ALARM_DAYS(buffer[3]) | DAYS_ALARM_AEND(1);
	
  buffer[4] = WEEKDAYS_WEEKDAYS(alarmTime->weekdays);
	buffer[4] = WEEKDAYS_ALARM_WEEKDAYS(buffer[4]) | WEEKDAYS_ALARM_AENW(1);

#endif

	UART_DEBUG("%-15s 0x%02X\r\n", "SECONDS_ALARM:", 	buffer[0]);
	UART_DEBUG("%-15s 0x%02X\r\n", "MINUTES_ALARM:", 	buffer[1]);
	UART_DEBUG("%-15s 0x%02X\r\n", "HOURS_ALARM:", 		buffer[2]);
	UART_DEBUG("%-15s 0x%02X\r\n", "DAYS_ALARM:", 		buffer[3]);
	UART_DEBUG("%-15s 0x%02X\r\n", "WEEKDAYS_ALARM:", buffer[4]);
	
	if (rtc_write(PCF85063A_SECOND_ALARM, buffer, SIZE_OF_ALL_ALARM_REGISTERS) != RTC_SUCCESS) { return RTC_TX_ERROR; }
	
	// Enable alarm
	byte = 0;
	byte = CONTROL2_AIE(1);
	
	UART_DEBUG("%-15s 0x%02X\r\n", "CONTROL2:", byte);
	
	if (rtc_write(PCF85063A_CONTROL2, &byte, 1) != RTC_SUCCESS) { return RTC_TX_ERROR; }
	
	return RTC_SUCCESS;
}

/** ----------------------------------------------------------------------
**
**	@fn		Function				rtc_get_alarm
**
**	@brief	Description		get rtc alarm
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
#define SIZE_OF_ALL_ALARM_REGISTERS (5)
RTC_TXRX_RET_T rtc_get_alarm(RTC_TIME_STRUCT_T *alarmTime) 
{
	uint8_t buffer[SIZE_OF_ALL_ALARM_REGISTERS];

	memset(buffer, SIZE_OF_ALL_ALARM_REGISTERS, 0);
	
	if (rtc_read(PCF85063A_SECOND_ALARM, buffer, SIZE_OF_ALL_ALARM_REGISTERS) != RTC_SUCCESS) { return RTC_TX_ERROR; }
  
#ifdef NEED_TO_CONVERT_TO_BCD  
	alarmTime->seconds 	= rtc_get_seconds(SECONDS_SECONDS(buffer[0]));
	alarmTime->minutes 	= rtc_get_minutes(buffer[1]);
	alarmTime->hours 		= rtc_get_hours(buffer[2]);
	alarmTime->days 		= rtc_get_days(buffer[3]);
	alarmTime->weekdays = rtc_get_weekdays(buffer[4]);
#else	
  // We need to strip off any extra bits in the bytes
	alarmTime->seconds 	= SECONDS_SECONDS(buffer[0]);
	alarmTime->minutes 	= MINUTES_MINUTES(buffer[1]);
	alarmTime->hours 		= HOURS_HOURS_24(buffer[2]);
	alarmTime->days 		= DAYS_DAYS(buffer[3]);
	alarmTime->weekdays = WEEKDAYS_WEEKDAYS(buffer[4]);

#endif

	return RTC_SUCCESS;
}

/** ----------------------------------------------------------------------
**
**	@fn		Function				rtc_delete_alarm
**
**	@brief	Description		delete rtc alarm
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
RTC_TXRX_RET_T rtc_delete_alarm(void) 
{
	RTC_TIME_STRUCT_T alarmTime = {0};
	uint8_t buffer[SIZE_OF_ALL_ALARM_REGISTERS];
	memset(buffer, SIZE_OF_ALL_ALARM_REGISTERS, 0);
	uint8_t byte = 0;
	
	UART_DEBUG("Entered: %s\r\n", __func__);
	
	// Disable alarm
	byte = 0;
	byte = CONTROL2_AIE(0);
	
	UART_DEBUG("%-15s 0x%02X\r\n", "CONTROL2:", byte);
	
	if (rtc_write(PCF85063A_CONTROL2, &byte, 1) != RTC_SUCCESS) { return RTC_TX_ERROR; }
	
	// AENx bit: 1 to disable alarm, 0 to enable

	buffer[0] = SECONDS_SECONDS(alarmTime.seconds);
	buffer[0] = SECONDS_ALARM_SECONDS(buffer[0]) | SECONDS_ALARM_AENS(1);
	
	buffer[1] = MINUTES_MINUTES(alarmTime.minutes);
	buffer[1] = MINUTES_ALARM_MINUTES(buffer[1]) | MINUTES_ALARM_AENM(1);

  buffer[2] = HOURS_HOURS_24(alarmTime.hours);
	buffer[2] = HOURS_ALARM_HOURS_24(buffer[2]) | HOURS_ALARM_AENH(1);
	
	buffer[3] = DAYS_DAYS(alarmTime.days);
	buffer[3] = DAYS_ALARM_DAYS(buffer[3]) | DAYS_ALARM_AEND(1);
	
	buffer[4] = WEEKDAYS_WEEKDAYS(alarmTime.weekdays);
	buffer[4] = WEEKDAYS_ALARM_WEEKDAYS(buffer[4]) | WEEKDAYS_ALARM_AENW(1);
	
	UART_DEBUG("%-15s 0x%02X\r\n", "SECONDS_ALARM:", 	buffer[0]);
	UART_DEBUG("%-15s 0x%02X\r\n", "MINUTES_ALARM:", 	buffer[1]);
	UART_DEBUG("%-15s 0x%02X\r\n", "HOURS_ALARM:", 		buffer[2]);
	UART_DEBUG("%-15s 0x%02X\r\n", "DAYS_ALARM:", 		buffer[3]);
	UART_DEBUG("%-15s 0x%02X\r\n", "WEEKDAYS_ALARM:", buffer[4]);
	
	if (rtc_write(PCF85063A_SECOND_ALARM, buffer, SIZE_OF_ALL_ALARM_REGISTERS) != RTC_SUCCESS) { return RTC_TX_ERROR; }
	
	return RTC_SUCCESS;
}
#undef SIZE_OF_ALL_ALARM_REGISTERS

/** ----------------------------------------------------------------------
**
**	@fn		Function				rtc_soft_reset
**
**	@brief	Description		perform rtc soft reset
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
#define SOFTWARE_RESET_BYTE (0x58)
RTC_TXRX_RET_T rtc_software_reset(void) 
{
  uint8_t byte = 0;
  
  byte = SOFTWARE_RESET_BYTE;
  if (rtc_write(PCF85063A_CONTROL1, &byte, 1) != RTC_SUCCESS) { return RTC_TX_ERROR; }
  
  return RTC_SUCCESS;
}
#undef SOFTWARE_RESET_BYTE

#ifdef NEED_TO_CONVERT_TO_BCD
static uint8_t rtc_get_seconds(uint8_t byte) 
{
	// Mask out OS bit
	byte &= 0x7F;
	return (uint8_t)((HI_NIBBLE(byte)*10) + (LO_NIBBLE(byte)));	
}

static uint8_t rtc_get_minutes(uint8_t byte) 
{
	return (uint8_t)((HI_NIBBLE(byte)*10) + (LO_NIBBLE(byte))); 
}

static uint8_t rtc_get_hours(uint8_t byte) 
{
	// Format will only be 24 hr
	return (uint8_t)((HI_NIBBLE(byte)*10) + (LO_NIBBLE(byte))); 
}

static uint8_t rtc_get_days(uint8_t byte) 
{
	return (uint8_t)((HI_NIBBLE(byte)*10) + (LO_NIBBLE(byte))); 
}

static uint8_t rtc_get_weekdays(uint8_t byte) 
{
	return (byte & 0x07);	
}

static uint8_t rtc_get_months(uint8_t byte) 
{
	return (uint8_t)((HI_NIBBLE(byte)*10) + (LO_NIBBLE(byte))); 
}

static uint8_t rtc_get_years(uint8_t byte) 
{
	return (uint8_t)((HI_NIBBLE(byte)*10) + (LO_NIBBLE(byte))); 
}

static uint8_t rtc_set_seconds(uint8_t byte) 
{
	return (uint8_t)((byte/10)*16) + (byte % 10);
}

static uint8_t rtc_set_minutes(uint8_t byte) 
{
	return (uint8_t)((byte/10)*16) + (byte % 10);
}

static uint8_t rtc_set_hours(uint8_t byte) 
{
	return (uint8_t)((byte/10)*16) + (byte % 10);
}

static uint8_t rtc_set_days(uint8_t byte)
{
	return (uint8_t)((byte/10)*16) + (byte % 10);
}

static uint8_t rtc_set_weekdays(uint8_t byte) 
{
	return (byte & 0x07);
}

static uint8_t rtc_set_months(uint8_t byte) 
{
	return (uint8_t)((byte/10)*16) + (byte % 10);
}

static uint8_t rtc_set_years(uint8_t year)
{
	return (uint8_t)((year/10)*16) + (year % 10);
}
#endif
