/**	----------------------------------------------------------------------
**
**	@file	accel.c
**
**  @defgroup PAVLOK_ACCELEROMETER
**  @{
**  @ingroup PAVLOK_ACCELEROMETER
**  
**  @details This module implements the PAVLOK accelerometer driver code
**  
**  @note The application must alaways first initialize the part
**  
**  @note
**   
**
**	----------------------------------------------------------------------
*/

/**	----------------------------------------------------------------------
**	@brief System Include(s)
**	----------------------------------------------------------------------
*/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"

/**	----------------------------------------------------------------------
**	@brief Project Include(s)
**	----------------------------------------------------------------------
*/
#include "pavlok_common.h"
#include "accel_mag.h"
#include "i2c.h"
#include "debug.h"
#include "leds.h"
#include "application_task.h"

/**	----------------------------------------------------------------------
**	@brief	Global Data
**	----------------------------------------------------------------------
*/
volatile uint8_t demoSemaphore1 = 0;
volatile uint8_t demoSemaphore2 = 0;

/**	----------------------------------------------------------------------
**	@brief	Static Forward References
**	----------------------------------------------------------------------
*/
static uint8_t is_accel_data_different(ACCEL_DATA_T *, ACCEL_DATA_T *);
static void print_acceleration_data(ACCEL_DATA_T *);
static ACC_MAG_INIT_RET_T accelerometer_config_continuous_read(void);
static ACC_MAG_INIT_RET_T accelerometer_config_transient(void);
static void accel_mag_input_int1_init(void);
static void accel_mag_input_int2_init(void);
static void accel_mag_pin_handler_int1(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
static void accel_mag_pin_handler_int2(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
//static ACC_MAG_INIT_RET_T accelerometer_config_vector_magnitude(void);
static ACC_MAG_INIT_RET_T accelerometer_config_pulse(void);

/**	----------------------------------------------------------------------
**	@brief	Static Data
**	----------------------------------------------------------------------
*/
// The default interrupt behavior of the accelerometer is active-low
static const nrf_drv_gpiote_in_config_t configAccMagInt1 = 
{
	.is_watcher = false,
	.hi_accuracy = false,
	.pull = NRF_GPIO_PIN_PULLUP,
	.sense = NRF_GPIOTE_POLARITY_HITOLO
};

// The default interrupt behavior of the accelerometer is active-low
static const nrf_drv_gpiote_in_config_t configAccMagInt2 = 
{
	.is_watcher = false,
	.hi_accuracy = false,
	.pull = NRF_GPIO_PIN_PULLUP,
	.sense = NRF_GPIOTE_POLARITY_HITOLO
};

/**	----------------------------------------------------------------------
**
**	@fn			Function			accel_mag_input_int1_init
**
**	@brief	Description		Function to initialize interrupt 1 handler
**												
**
**	@param 	[in]					None	
**
**	@param	[out]					None
**
**	@return								uint8_t		
**
**	@retval								None
**
**	@warn									None
**
**	----------------------------------------------------------------------
*/
static void accel_mag_input_int1_init(void) 
{
	// First, init GPIOTE if it isn't already initialized
	if (!nrf_drv_gpiote_is_init()) 
	{
	  UNUSED_RETURN_VALUE(nrf_drv_gpiote_init());
	}
	// Set designated pin used for interrupt and specify the call-back
	// Int 1
	UNUSED_RETURN_VALUE(nrf_drv_gpiote_in_init(ACC_MAG_INT1_PIN, &configAccMagInt1, accel_mag_pin_handler_int1));
	nrf_drv_gpiote_in_event_enable(ACC_MAG_INT1_PIN, true);
	
}

/**	----------------------------------------------------------------------
**
**	@fn			Function			accel_mag_input_int2_init
**
**	@brief	Description		Function to initialize interrupt 2 handler
**												
**
**	@param 	[in]					None	
**
**	@param	[out]					None
**
**	@return								uint8_t		
**
**	@retval								None
**
**	@warn									None
**
**	----------------------------------------------------------------------
*/
static void accel_mag_input_int2_init(void) 
{
	// First, init GPIOTE if it isn't already initialized
	if (!nrf_drv_gpiote_is_init()) 
	{
	  UNUSED_RETURN_VALUE(nrf_drv_gpiote_init());
	}
	// Set designated pin used for interrupt and specify the call-back
	// Int 2
	UNUSED_RETURN_VALUE(nrf_drv_gpiote_in_init(ACC_MAG_INT2_PIN, &configAccMagInt2, accel_mag_pin_handler_int2));
	nrf_drv_gpiote_in_event_enable(ACC_MAG_INT2_PIN, true);
	
}

/**	----------------------------------------------------------------------
**
**	@fn			Function			accel_mag_pin_handler_int1
**
**	@brief	Description		Handler for interrupt 1 from accelerometer
**												
**
**	@param 	[in]					None	
**
**	@param	[out]					None
**
**	@return								uint8_t		
**
**	@retval								None
**
**	@warn									None
**
**	----------------------------------------------------------------------
*/
static void accel_mag_pin_handler_int1(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) 
{

	if ((pin == ACC_MAG_INT1_PIN) && (action == NRF_GPIOTE_POLARITY_HITOLO)) 
	{
	}
}

/**	----------------------------------------------------------------------
**
**	@fn			Function			accel_mag_pin_handler_int2
**
**	@brief	Description		Handler for interrupt 2 from accelerometer
**												
**
**	@param 	[in]					None	
**
**	@param	[out]					None
**
**	@return								uint8_t		
**
**	@retval								None
**
**	@warn									None
**
**	----------------------------------------------------------------------
*/
static void accel_mag_pin_handler_int2(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) 
{
#if 0
	if ((pin == ACC_MAG_INT2_PIN) && (action == NRF_GPIOTE_POLARITY_HITOLO)) 
	{
    if (TRAINING_STATE_ACTIVE != (pavlok_get_training_started()))
    {
      sPavlokServiceInfo_t * service_info = pavlok_get_configuration();
      service_info->cfg.zap_me_value[PLOK_INDEX_DC_BYTE_ZAP] += PLOK_DC_10_PERCENT_VALUE;
      if (PLOK_DC_100_PERCENT_VALUE  > service_info->cfg.zap_me_value[PLOK_INDEX_DC_BYTE_ZAP])
      {
        service_info->cfg.zap_me_value[PLOK_INDEX_DC_BYTE_ZAP] = PLOK_DC_50_PERCENT_VALUE;
      }
    }
	}
  accel_mag_unlatch_int1();
#endif
}

/**	----------------------------------------------------------------------
**
**	@fn		Function		accel_mag_unlatch_int1
**
**	@brief	Please read header in include file for function description
**
**	----------------------------------------------------------------------
*/
ACC_MAG_TXRX_RET_T accel_mag_unlatch_int1(void) 
{
	uint8_t byte = 0;
	
	if (accel_magnet_read(FXOS8700CQ_TRANSIENT_SRC, &byte, 1) != ACC_MAG_SUCCESS) { return ACC_MAG_TX_ERROR; }
	
	return ACC_MAG_SUCCESS;
}

/**	----------------------------------------------------------------------
**
**	@fn		Function		accel_mag_unlatch_int2
**
**	@brief	Please read header in include file for function description
**
**	----------------------------------------------------------------------
*/
ACC_MAG_TXRX_RET_T accel_mag_unlatch_int2(void) 
{
	uint8_t byte = 0;
	
	if (accel_magnet_read(FXOS8700CQ_PULSE_SRC, &byte, 1) != ACC_MAG_SUCCESS) { return ACC_MAG_TX_ERROR; }
	
	return ACC_MAG_SUCCESS;
}

/**	----------------------------------------------------------------------
**
**	@fn			Function		accel_magnet_write
**
**	@brief	Please read header in include file for function description
**
**	----------------------------------------------------------------------
*/
ACC_MAG_TXRX_RET_T accel_magnet_write(uint8_t addr, uint8_t *data, uint8_t length) 
{
	ret_code_t errCode;
	accmagTXRXPacket_T accmagTXRXPacket;
	
	if (data == NULL) {
		return ACC_MAG_NULL_DATA;
	}
	/*if (addr < FXOS8700CQ_REG_RANGE_LOW) 
	{
		return ACC_MAG_INVALID_ADDRESS;
	}*/
	if (addr > FXOS8700CQ_REG_RANGE_HIGH) 
	{
		return ACC_MAG_INVALID_ADDRESS;
	}
	if (length > I2C_ACC_MAG_MAX_TRANSFER) 
	{
		return ACC_MAG_INVALID_SIZE;
	}
	accmagTXRXPacket.address = addr;
	memcpy(&accmagTXRXPacket.data[0], data, length);
	
	errCode = i2c1_sync_write(I2C_ACC_MAG_ADDRESS, (uint8_t *)&accmagTXRXPacket, ACC_MAG_FXOS8700CQ_ADDRESS_SIZE+length);
	
	if (NRF_SUCCESS == errCode) 
	{
		return ACC_MAG_SUCCESS;
	} 
	else 
	{
		return ACC_MAG_TX_ERROR;	
	}
}

/**	----------------------------------------------------------------------
**
**	@fn			Function		accel_magnet_read
**
**	@brief	Please read header in include file for function description
**
**	----------------------------------------------------------------------
*/
ACC_MAG_TXRX_RET_T accel_magnet_read(uint8_t addr, uint8_t *data, uint8_t length) 
{
	ret_code_t errCode;
	accmagTXRXPacket_T accmagTXRXPacket;
	
	/*if (addr < FXOS8700CQ_REG_RANGE_LOW) 
	{
		return ACC_MAG_INVALID_ADDRESS;
	}*/
	if (addr > FXOS8700CQ_REG_RANGE_HIGH) 
	{
		return ACC_MAG_INVALID_ADDRESS;
	}
	if (length > I2C_ACC_MAG_MAX_TRANSFER) 
	{
		return ACC_MAG_INVALID_SIZE;
	}
	memset(&accmagTXRXPacket.data[0], I2C_ACC_MAG_MAX_TRANSFER, 0);
	accmagTXRXPacket.address = addr;
	
	errCode = i2c1_sync_read(I2C_ACC_MAG_ADDRESS, (uint8_t *)&accmagTXRXPacket, ACC_MAG_FXOS8700CQ_ADDRESS_SIZE, data, length);
	
	if (NRF_SUCCESS == errCode) 
	{
		return ACC_MAG_SUCCESS;
	} 
	else 
	{
		return ACC_MAG_TX_ERROR;	
	}		
}

/**	----------------------------------------------------------------------
**
**	@fn			Function			accelerometer_whoami
**
**	@brief	Description		i2c bus test function to verify the part is 
**												working
**
**	@param 	[in]					None	
**
**	@param	[out]					None
**
**	@return								uint8_t		
**
**	@retval								ACC_MAG_INIT_FAIL
**	@retval								ACC_MAG_INIT_SUCCESS
**
**	@warn									None
**
**	----------------------------------------------------------------------
*/
uint8_t accelerometer_whoami(void) 
{
	ret_code_t errCode;
	uint8_t byte = 0;
	
	errCode = accel_magnet_read(FXOS8700CQ_WHOAMI, &byte, 1);
	
	if ((errCode != NRF_SUCCESS) || (byte != FXOS8700CQ_WHOAMI_VAL)) 
	{
		return 0;
	}
	
	return 1;
}

/**	----------------------------------------------------------------------
**
**	@fn			Function			accelerometer_config_continuous_read
**
**	@brief	Description		Configures accelerometer to read acceleration data continuously
**												
**
**	@param 	[in]					None	
**
**	@param	[out]					None
**
**	@return								uint8_t		
**
**	@retval								ACC_MAG_INIT_FAIL
**	@retval								ACC_MAG_INIT_SUCCESS
**
**	@warn									None
**
**	----------------------------------------------------------------------
*/
static ACC_MAG_INIT_RET_T accelerometer_config_continuous_read(void) 
{
	uint8_t configByte = 0;
	
	// Write to accelerometer control register 1
	configByte = 0;
	
	// 0 – Standby mode
	// 1 – Active mode 
	configByte |= CTRL_REG1_ACTIVE(0);
	
	UART_DEBUG("%-20s: 0x%02X\r\n", "CTRL_REG_1", configByte);
	
	if (accel_magnet_write(FXOS8700CQ_CTRL_REG1, &configByte, 1) != ACC_MAG_SUCCESS) { return ACC_MAG_INIT_FAIL; } 
	//nrf_delay_ms(50);
	
	// Write to magnetometer control register 1
	configByte = 0;
	
	// 0b00 – Only accelerometer sensor is active
	// 0b01 – Only magnetometer sensor is active
	// 0b11 – Hybrid mode, both accelerometer and magnetometer sensors are active[1]
	configByte = M_CTRL_REG1_M_HMS(0);
	
	UART_DEBUG("%-20s: 0x%02X\r\n", "M_CTRL_REG_1", configByte);
	
	if (accel_magnet_write(FXOS8700CQ_M_CTRL_REG1, &configByte, 1) != ACC_MAG_SUCCESS) { return ACC_MAG_INIT_FAIL; } 
	//nrf_delay_ms(50);
	
	// Write to magnetometer control register 2
	// Not needed 
	
	// Write to XYZ config register
	configByte = 0;
	
	// 1 – Output data is high-pass filtered
	// 0 – High-pass filter is disabled.
	configByte |= XZY_DATA_CFG_HPF_OUT(0);
	
	// fs[1] fs[0]
	// 0 			0 		- ±0.244 mg/LSB 	±2 g
	// 0 			1 		- ±0.488 mg/LSB 	±4 g
	// 1 			0 		- ±0.976 mg/LSB 	±8 g
	// 1 			1 		- Reserved
	configByte |= XZY_DATA_CFG_FS(1);
	
	UART_DEBUG("%-20s: 0x%02X\r\n", "XYZ_DATA_CONFIG", configByte);
	
	if (accel_magnet_write(FXOS8700CQ_XYZ_DATA_CFG, &configByte, 1) != ACC_MAG_SUCCESS) { return ACC_MAG_INIT_FAIL; } 
	//nrf_delay_ms(50);
	
	// Write accelerometer control register 1
	configByte = 0;
	
	// 0 – Standby mode
	// 1 – Active mode 
	configByte |= CTRL_REG1_ACTIVE(1);
	
	// 0 – Normal mode 
	// 1 – Fast-read mode
	configByte |= CTRL_REG1_F_READ(0);
	
	// 0 – Normal mode
	// 1 – Reduced noise mode; Note that the FSR setting is restricted to ±2 g or ±4 g mode. This feature cannot be used in ±8 g mode.
	configByte |= CTRL_REG1_INOISE(1);
	
	/*
	dr[2] dr[1] dr[0] ODR (Hz)
	0 		0 		0 		800.0 
	0 		0 		1 		400.0 
	0 		1 		0 		200.0 
	0 		1 		1 		100.0 
	1 		0 		0 		50.0 
	1 		0 		1 		12.5 
	1 		1 		0 		6.25 
	1 		1 		1 		1.5625
	*/
	configByte |= CTRL_REG1_DR(4);
	
	/*
	aslp_rate[1] 	aslp_rate[0] 	Frequency (Hz)
	0 						0 						50
	0 						1 						12.5
	1 						0 						6.25
	1 						1 						1.56
	*/
	configByte |= CTRL_REG1_ASLP_RATE(0);
	
	UART_DEBUG("%-20s: 0x%02X\r\n", "CTRL_REG_1", configByte);
	
	if (accel_magnet_write(FXOS8700CQ_CTRL_REG1, &configByte, 1) != ACC_MAG_SUCCESS) { return ACC_MAG_INIT_FAIL; }
	//nrf_delay_ms(50);

	return ACC_MAG_INIT_SUCCESS;
}

/**	----------------------------------------------------------------------
**
**	@fn			Function			accelerometer_config_transient
**
**	@brief	Description		Configures accelerometer to alert of transient acceleration
**												
**
**	@param 	[in]					None	
**
**	@param	[out]					None
**
**	@return								uint8_t		
**
**	@retval								ACC_MAG_INIT_FAIL
**	@retval								ACC_MAG_INIT_SUCCESS
**
**	@warn									None
**
**	----------------------------------------------------------------------
*/
static ACC_MAG_INIT_RET_T accelerometer_config_transient(void) 
{
	uint8_t configByte = 0;
	
	// Write to accelerometer control register 1
	configByte = 0;
	
	// 0 – Standby mode
	// 1 – Active mode 
	configByte |= CTRL_REG1_ACTIVE(0);
	
	UART_DEBUG("%-20s: 0x%02X\r\n", "CTRL_REG_1", configByte);
	
	if (accel_magnet_write(FXOS8700CQ_CTRL_REG1, &configByte, 1) != ACC_MAG_SUCCESS) { return ACC_MAG_INIT_FAIL; } 
	//nrf_delay_ms(50);
	
	// Write to transient config register
	configByte = 0;
	
	// 0 - High-pass filter is applied
	// 1 - High-pass filter is NOT applied
	configByte |= TRANSIENT_CFG_TRAN_HPF_BYP(0);
	
	// 0 - X-axis event detection disabled
	// 1 - X-axis event detection enabled
	configByte |= TRANSIENT_CFG_TRAN_XEFE(1); 
	
	// 0 - Y-axis event detection disabled
	// 1 - Y-axis event detection enabled
	configByte |= TRANSIENT_CFG_TRAN_YEFE(1);

	// 0 - Z-axis event detection disabled
	// 1 - Z-axis event detection enabled
	configByte |= TRANSIENT_CFG_TRAN_ZEFE(1); 
	
	// 0 - Event flag latch disabled
	// 1 - Event flag latch enabled
	configByte |= TRANSIENT_CFG_TRAN_ELE(1);
	
	UART_DEBUG("%-20s: 0x%02X\r\n", "TRANSIENT_CFG", configByte);
	
	if (accel_magnet_write(FXOS8700CQ_TRANSIENT_CFG, &configByte, 1) != ACC_MAG_SUCCESS) { return ACC_MAG_INIT_FAIL; } 
	//nrf_delay_ms(50);
	
	// Write to transient threshold register
	configByte = 0;
	
	// Threshold is 63 mg/LSB
	// 63*24 = 1512
	// 63*16 = 1008
	// 63*8  = 504
	// 63*5  = 315
	// 63*3  = 189
	configByte |= TRANSIENT_THS_TR_THS(6
  );
	
	// 0 - Decrements debounce counter when the transient event condition is not true during current ODR period.
	// 1 - Clears debounce counter when the transient event condition is not true during the current ODR period
	configByte |= TRANSIENT_THS_TR_DBCNTM(1);
	
	UART_DEBUG("%-20s: 0x%02X\r\n", "TRANSIENT_THS", configByte);
	
	if (accel_magnet_write(FXOS8700CQ_TRANSIENT_THS, &configByte, 1) != ACC_MAG_SUCCESS) { return ACC_MAG_INIT_FAIL; } 
	//nrf_delay_ms(50);
	
	// Write to transient count register
	configByte = 0;
	
	// Check Table 122 to calculate number of counts based on ODR, mode, etc.
	// For example, with magnetometer disabled, @ 50 MHZ ODR, 20 ms per count.
	configByte |= TRANSIENT_COUNT(3);
	
	UART_DEBUG("%-20s: 0x%02X\r\n", "TRANSIENT_COUNT", configByte);
	
	if (accel_magnet_write(FXOS8700CQ_TRANSIENT_COUNT, &configByte, 1) != ACC_MAG_SUCCESS) { return ACC_MAG_INIT_FAIL; } 
	//nrf_delay_ms(50);
	
	// Enable interrupts for this feature using CTRL_REG4
	configByte = 0;
	
	// 0 - Disable interrupt for transient function
	// 1 - Enable interrupt for transient function
	configByte |= CTRL_REG4_INT_EN_TRANS(1);
	
	if (accel_magnet_write(FXOS8700CQ_CTRL_REG4, &configByte, 1) != ACC_MAG_SUCCESS) { return ACC_MAG_INIT_FAIL; } 
	//nrf_delay_ms(50);
	
	UART_DEBUG("%-20s: 0x%02X\r\n", "CTRL_REG_4", configByte);
	
	// Route interrupts to INT1 pin using CTRL_REG5
	configByte = 0;
	
	// 0 - Interrupt is routed to INT2 pin
	// 1 - Interrupt is routed to INT1 pin
	configByte |= CTRL_REG5_INT_CFG_TRANS(1);
	
	if (accel_magnet_write(FXOS8700CQ_CTRL_REG5, &configByte, 1) != ACC_MAG_SUCCESS) { return ACC_MAG_INIT_FAIL; } 
	//nrf_delay_ms(50);
	
	UART_DEBUG("%-20s: 0x%02X\r\n", "CTRL_REG_5", configByte);
	
	// Write to magnetometer control register 1
	configByte = 0;
	
	// 0b00 – Only accelerometer sensor is active
	// 0b01 – Only magnetometer sensor is active
	// 0b11 – Hybrid mode, both accelerometer and magnetometer sensors are active[1]
	configByte = M_CTRL_REG1_M_HMS(0);
	
	UART_DEBUG("%-20s: 0x%02X\r\n", "M_CTRL_REG_1", configByte);
	
	if (accel_magnet_write(FXOS8700CQ_M_CTRL_REG1, &configByte, 1) != ACC_MAG_SUCCESS) { return ACC_MAG_INIT_FAIL; } 
	//nrf_delay_ms(50);
	
	// Write to magnetometer control register 2
	// Not needed 
	
	// Write to XYZ config register
	configByte = 0;
	
	// 1 – Output data is high-pass filtered
	// 0 – High-pass filter is disabled.
	configByte |= XZY_DATA_CFG_HPF_OUT(0);
	
	// fs[1] fs[0]
	// 0 			0 		- ±0.244 mg/LSB 	±2 g
	// 0 			1 		- ±0.488 mg/LSB 	±4 g
	// 1 			0 		- ±0.976 mg/LSB 	±8 g
	// 1 			1 		- Reserved
	configByte |= XZY_DATA_CFG_FS(1);
	
	UART_DEBUG("%-20s: 0x%02X\r\n", "XYZ_DATA_CONFIG", configByte);
	
	if (accel_magnet_write(FXOS8700CQ_XYZ_DATA_CFG, &configByte, 1) != ACC_MAG_SUCCESS) { return ACC_MAG_INIT_FAIL; } 
	
	// Write accelerometer control register 1
	configByte = 0;
	
	// 0 – Standby mode
	// 1 – Active mode 
	configByte |= CTRL_REG1_ACTIVE(1);
	
	// 0 – Normal mode 
	// 1 – Fast-read mode
	configByte |= CTRL_REG1_F_READ(0);
	
	// 0 – Normal mode
	// 1 – Reduced noise mode; Note that the FSR setting is restricted to ±2 g or ±4 g mode. This feature cannot be used in ±8 g mode.
	configByte |= CTRL_REG1_INOISE(1);
	
	/*
	dr[2] dr[1] dr[0] ODR (Hz)
	0 		0 		0 		800.0 
	0 		0 		1 		400.0 
	0 		1 		0 		200.0 
	0 		1 		1 		100.0 
	1 		0 		0 		50.0 
	1 		0 		1 		12.5 
	1 		1 		0 		6.25 
	1 		1 		1 		1.5625
	*/
	configByte |= CTRL_REG1_DR(4);
	
	/*
	aslp_rate[1] 	aslp_rate[0] 	Frequency (Hz)
	0 						0 						50
	0 						1 						12.5
	1 						0 						6.25
	1 						1 						1.56
	*/
	configByte |= CTRL_REG1_ASLP_RATE(0);
	
	UART_DEBUG("%-20s: 0x%02X\r\n", "CTRL_REG_1", configByte);
	
	if (accel_magnet_write(FXOS8700CQ_CTRL_REG1, &configByte, 1) != ACC_MAG_SUCCESS) { return ACC_MAG_INIT_FAIL; }
	//nrf_delay_ms(50);
	
	accel_mag_input_int1_init();
	// Always un-latch interrupt in case it has already triggered
	accel_mag_unlatch_int1();
	
	return ACC_MAG_INIT_SUCCESS;
}

/**	----------------------------------------------------------------------
**
**	@fn			Function			accelerometer_config_pulse
**
**	@brief	Description		Configures accelerometer to alert of pulse (double-tap)
**												
**
**	@param 	[in]					None	
**
**	@param	[out]					None
**
**	@return								uint8_t		
**
**	@retval								ACC_MAG_INIT_FAIL
**	@retval								ACC_MAG_INIT_SUCCESS
**
**	@warn									None
**
**	----------------------------------------------------------------------
*/
static ACC_MAG_INIT_RET_T accelerometer_config_pulse(void) 
{
	uint8_t configByte = 0;
	
	// Write to accelerometer control register 1
	configByte = 0;
	
	// 0 – Standby mode
	// 1 – Active mode 
	configByte |= CTRL_REG1_ACTIVE(0);
	
	UART_DEBUG("%-20s: 0x%02X\r\n", "CTRL_REG_1", configByte);
	
	if (accel_magnet_write(FXOS8700CQ_CTRL_REG1, &configByte, 1) != ACC_MAG_SUCCESS) { return ACC_MAG_INIT_FAIL; } 
	
	// Write to pulse config register 
	configByte = 0;
	
	// Event flag enable on double-pulse event on Z/Y/Z-axis. 
  // 0 – Event detection disabled
  // 1 – Raise event flag on detection of double-pulse event on Z/Y/Z-axis 
	configByte |= PULSE_CFG_PLS_XDPEFE(1);
	configByte |= PULSE_CFG_PLS_YDPEFE(1);
	configByte |= PULSE_CFG_PLS_ZDPEFE(1);
	
	// Pulse event flag latch enable. When enabled, a read of the PULSE_SRC register is needed to clear the event flag. 
  // 0 – Event flag latch disabled
  // 1 – Event flag latch enabled
	configByte |= PULSE_CFG_PLS_ELE(1);
	
	// Double-pulse abort.
	// 0 – Double-pulse detection is not aborted if the start of a pulse is detected during the time period specified by the PULSE_LTCY register. 
  // 1 – Setting the pls_dpa bit momentarily suspends the double-tap detection if the start of a pulse is detected during the time period specified by the PULSE_LTCY register and the pulse ends before the end of the time period specified by the PULSE_LTCY register.
	configByte |= PULSE_CFG_PLS_DPA(1);
	
	UART_DEBUG("%-20s: 0x%02X\r\n", "PULSE_CFG", configByte);
	
	if (accel_magnet_write(FXOS8700CQ_PULSE_CFG, &configByte, 1) != ACC_MAG_SUCCESS) { return ACC_MAG_INIT_FAIL; } 
	
	// Write X Threshold register
	configByte = 0;
	
	// Every step is 0.063g
	// 48 * .063g = 3.024g
	configByte |= PULSE_THSX(10);
	
	UART_DEBUG("%-20s: 0x%02X\r\n", "PULSE_THSX", configByte);
	
	if (accel_magnet_write(FXOS8700CQ_PULSE_THSX, &configByte, 1) != ACC_MAG_SUCCESS) { return ACC_MAG_INIT_FAIL; } 
	
	// Write Y Threshold register
	configByte = 0;
	
	// Every step is 0.063g
	// 48 * .063g = 3.024g
	configByte |= PULSE_THSY(10);
	
	UART_DEBUG("%-20s: 0x%02X\r\n", "PULSE_THSY", configByte);
	
	if (accel_magnet_write(FXOS8700CQ_PULSE_THSY, &configByte, 1) != ACC_MAG_SUCCESS) { return ACC_MAG_INIT_FAIL; } 
	
	// Write Z Threshold register
	configByte = 0;
	
	// Every step is 0.063g
	// 79 * .063g = 4.977g
	configByte |= PULSE_THSZ(10);
	
	UART_DEBUG("%-20s: 0x%02X\r\n", "PULSE_THSZ", configByte);
	
	if (accel_magnet_write(FXOS8700CQ_PULSE_THSZ, &configByte, 1) != ACC_MAG_SUCCESS) { return ACC_MAG_INIT_FAIL; } 
	
	// Write time limit for pulse detection register
	configByte = 0;
	
	// By default, HPF is enabled and LPF is disabled
	// @ 50 Hz ODR, each step is 5 ms (since pls_lpf_en = 0 by default)
	// Set to 60 ms
	configByte |= PULSE_TMLT(12);
	
	UART_DEBUG("%-20s: 0x%02X\r\n", "PULSE_TMLT", configByte);
	
	if (accel_magnet_write(FXOS8700CQ_PULSE_TMLT, &configByte, 1) != ACC_MAG_SUCCESS) { return ACC_MAG_INIT_FAIL; }
	
	// Write latency time
	configByte = 0;
	
	// By default, HPF is enabled and LPF is disabled
	// @ 50 Hz ODR, each step is 10 ms (since pls_lpf_en = 0 by default)
	// Set to 200 ms
	configByte |= PULSE_LTCY(20);
	
	UART_DEBUG("%-20s: 0x%02X\r\n", "PULSE_LTCY", configByte);
	
	if (accel_magnet_write(FXOS8700CQ_PULSE_LTCY, &configByte, 1) != ACC_MAG_SUCCESS) { return ACC_MAG_INIT_FAIL; }
	
	// Write time window
	configByte = 0;
	
	// By default, HPF is enabled and LPF is disabled
	// @ 50 Hz ODR, each step is 10 ms (since pls_lpf_en = 0 by default)
	// Set to 300 ms
	configByte |= PULSE_WIND(30);
	
	UART_DEBUG("%-20s: 0x%02X\r\n", "PULSE_WIND", configByte);
	
	if (accel_magnet_write(FXOS8700CQ_PULSE_WIND, &configByte, 1) != ACC_MAG_SUCCESS) { return ACC_MAG_INIT_FAIL; }
	
	// Enable Pulse Interrupt in System CTRL_REG4
	configByte = 0;
	
	// Pulse interrupt enable
  // 0 – Pulse detection interrupt disabled
  // 1 – Pulse detection interrupt enabled
	configByte |= CTRL_REG4_INT_EN_PULSE(1);
	
	UART_DEBUG("%-20s: 0x%02X\r\n", "CTRL_REG4", configByte);
	
	if (accel_magnet_write(FXOS8700CQ_CTRL_REG4, &configByte, 1) != ACC_MAG_SUCCESS) { return ACC_MAG_INIT_FAIL; }
	
	// Route Pulse Interrupt to INT1 hardware Pin CTRL_REG5
	configByte = 0;
	
	// Pulse detection interrupt routing
  // 0 – Interrupt is routed to INT2 pin 
  // 1 – Interrupt is routed to INT1 pin
	configByte |= CTRL_REG5_INT_CFG_PULSE(0);
	
	UART_DEBUG("%-20s: 0x%02X\r\n", "CTRL_REG5", configByte);
	
	if (accel_magnet_write(FXOS8700CQ_CTRL_REG5, &configByte, 1) != ACC_MAG_SUCCESS) { return ACC_MAG_INIT_FAIL; }
	
	// Write accelerometer control register 1
	configByte = 0;
	
	// 0 – Standby mode
	// 1 – Active mode 
	configByte |= CTRL_REG1_ACTIVE(1);
	
	// 0 – Normal mode 
	// 1 – Fast-read mode
	configByte |= CTRL_REG1_F_READ(0);
	
	// 0 – Normal mode
	// 1 – Reduced noise mode; Note that the FSR setting is restricted to ±2 g or ±4 g mode. This feature cannot be used in ±8 g mode.
	configByte |= CTRL_REG1_INOISE(1);
	
	/*
	dr[2] dr[1] dr[0] ODR (Hz)
	0 		0 		0 		800.0 
	0 		0 		1 		400.0 
	0 		1 		0 		200.0 
	0 		1 		1 		100.0 
	1 		0 		0 		50.0 
	1 		0 		1 		12.5 
	1 		1 		0 		6.25 
	1 		1 		1 		1.5625
	*/
	configByte |= CTRL_REG1_DR(4);
	
	/*
	aslp_rate[1] 	aslp_rate[0] 	Frequency (Hz)
	0 						0 						50
	0 						1 						12.5
	1 						0 						6.25
	1 						1 						1.56
	*/
	configByte |= CTRL_REG1_ASLP_RATE(0);
	
	UART_DEBUG("%-20s: 0x%02X\r\n", "CTRL_REG_1", configByte);
	
	if (accel_magnet_write(FXOS8700CQ_CTRL_REG1, &configByte, 1) != ACC_MAG_SUCCESS) { return ACC_MAG_INIT_FAIL; }
	//nrf_delay_ms(50);
	
	UART_DEBUG("Success: FXOS8700CQ Pulse Initialized\r\n");

	accel_mag_input_int2_init();
	// Always un-latch interrupt in case it has already triggered
	accel_mag_unlatch_int2();
	
	return ACC_MAG_INIT_SUCCESS;
}

#if 0
static ACC_MAG_INIT_RET_T accelerometer_config_vector_magnitude(void) 
{

}
#endif

/**	----------------------------------------------------------------------
**
**	@fn		Function		read_stream_data
**
**	@brief	Please read header in include file for function description
**
**	----------------------------------------------------------------------
*/
uint32_t read_stream_data(uint8_t * p_data) 
{
	uint32_t ret  = (uint32_t)ACC_MAG_SUCCESS;	
	ret = get_acceleration((ACCEL_DATA_T *)p_data);

	return ret;
}



/**	----------------------------------------------------------------------
**
**	@fn		Function		accelerometer_init_transient
**
**	@brief	Please read header in include file for function description
**
**	----------------------------------------------------------------------
*/
void accelerometer_init_continuouse_read(void)
{

	nrf_gpio_pin_set(PAVLOK_ACCEL_MAG_RESET_PIN);
	nrf_delay_ms(1);
	nrf_gpio_pin_clear(PAVLOK_ACCEL_MAG_RESET_PIN);
	nrf_delay_ms(1);
	(void)accelerometer_config_continuous_read();
}

/**	----------------------------------------------------------------------
**
**	@fn		Function		accelerometer_init_pulse
**
**	@brief	Please read header in include file for function description
**
**	----------------------------------------------------------------------
*/
void accelerometer_init_pulse(void) 
{

	nrf_gpio_pin_set(PAVLOK_ACCEL_MAG_RESET_PIN);
	nrf_delay_ms(1);
	nrf_gpio_pin_clear(PAVLOK_ACCEL_MAG_RESET_PIN);
	nrf_delay_ms(1);
	(void)accelerometer_config_pulse();
}

/**	----------------------------------------------------------------------
**
**	@fn		Function		accelerometer_init_transient
**
**	@brief	Please read header in include file for function description
**
**	----------------------------------------------------------------------
*/
void accelerometer_init_transient(void) 
{

	nrf_gpio_pin_set(PAVLOK_ACCEL_MAG_RESET_PIN);
	nrf_delay_ms(1);
	nrf_gpio_pin_clear(PAVLOK_ACCEL_MAG_RESET_PIN);
	nrf_delay_ms(1);
	(void)accelerometer_config_transient();
}

/**	----------------------------------------------------------------------
**
**	@fn		Function		get_acceleration
**
**	@brief	Please read header in include file for function description
**
**	----------------------------------------------------------------------
*/
#define SIZE_OF_ACC_DATA (7)
ACC_MAG_TXRX_RET_T get_acceleration(ACCEL_DATA_T *accelData) 
{
	uint8_t buffer[SIZE_OF_ACC_DATA]; 
	memset(buffer, SIZE_OF_ACC_DATA, 0);
	
	if (accel_magnet_read(FXOS8700CQ_STATUS, buffer, SIZE_OF_ACC_DATA) != ACC_MAG_SUCCESS) { return ACC_MAG_TX_ERROR; }
	
	// Copy the 14-bit accelerometer byte data into 16 bit words
	accelData->x = (int16_t)(((buffer[1] << 8) | buffer[2]))>> 2;
	accelData->y = (int16_t)(((buffer[3] << 8) | buffer[4]))>> 2;
	accelData->z = (int16_t)(((buffer[5] << 8) | buffer[6]))>> 2;
	
	return ACC_MAG_SUCCESS;
}
#undef SIZE_OF_ACC_DATA

/**	----------------------------------------------------------------------
**
**	@fn			Function			print_acceleration_data
**
**	@brief	Description		Debug function to print accelerometer data
**
**	@param 	[in]					ACCEL_DATA_T *accelData - pointer to print
**												data
**
**	@param	[out]					None
**
**	@return								None
**
**	@warn
**
**	----------------------------------------------------------------------
*/
static void print_acceleration_data(ACCEL_DATA_T *accelData) 
{
	ACCEL_DATA_T scaled = { .x = 0, .y = 0, .z = 0 };
	
	if (accelData == NULL) 
	{ 
		return; 
	}
	
	scaled.x = (int16_t)(ACC_MAG_SCALING_PER_LSB*(accelData->x)); 
	scaled.y = (int16_t)(ACC_MAG_SCALING_PER_LSB*(accelData->y)); 
	scaled.z = (int16_t)(ACC_MAG_SCALING_PER_LSB*(accelData->z)); 
	// Print raw data
	//UART_DEBUG("Raw Data --- X: %05d, Y: %05d, Z: %05d\r\n", accelData->x, accelData->y, accelData->z);
	// Print scaled data
	UART_DEBUG("X: %5d mg, Y: %5d mg, Z: %5d mg\r\n", scaled.x, scaled.y, scaled.z);
}

#define NUM_TO_READ_AFTER_THRESHOLD (1)
void read_acceleration_after_threshold(ACCEL_DATA_T * data) 
{
	ACC_MAG_TXRX_RET_T ret;
	uint32_t numToRead = NUM_TO_READ_AFTER_THRESHOLD;
	
	//UART_DEBUG("Threshold Detected!\r\n");
	while (numToRead > 0) 
	{ 
		ret = get_acceleration(data);
		if (ret != ACC_MAG_SUCCESS) 
		{
				data->x = 0;
				data->y = 0;
				data->z = 0;
		} 
		numToRead--;
	}
	accel_mag_unlatch_int1();
}
#undef NUM_TO_READ_AFTER_THRESHOLD

/**	----------------------------------------------------------------------
**
**	@fn		Function		poll_acceleration_data
**
**	@brief	Please read header in include file for function description
**
**	----------------------------------------------------------------------
*/
#define POLL_RATE_MS	(100)
void poll_acceleration_data(void) 
{
	static ACCEL_DATA_T oldData = { .x = 0, .y = 0, .z = 0 };
	static ACCEL_DATA_T newData = { .x = 0, .y = 0, .z = 0 };
	ACC_MAG_TXRX_RET_T ret;
	
	nrf_delay_ms(POLL_RATE_MS);
	ret = get_acceleration(&newData);
	if (ret == ACC_MAG_SUCCESS) 
	{
		if (is_accel_data_different(&oldData, &newData)) 
		{
			print_acceleration_data(&newData);
		}
		oldData.x = newData.x;
		oldData.y = newData.y;
		oldData.z = newData.z;
	} 
	else 
	{
		UART_DEBUG("Error in reading acceleration data\r\n");
	}
}
#undef POLL_RATE_MS

/**	----------------------------------------------------------------------
**
**	@fn			Function			is_accel_data_different
**
**	@brief	Description		Function used to test for Pavlok movement
**
**	@param 	[in]					ACCEL_DATA_T *oldData -  pointer to last data
**												movement
**
**	@param 	[in]					ACCEL_DATA_T *newData -  pointer to new data
**												sample
**
**	@param 	[out]					None
**
**	@return								uint8+t
**
**	@retval
**
**	@warn
**
**	----------------------------------------------------------------------
*/
#define MIN_DIFFERENCE (50)
static uint8_t is_accel_data_different(ACCEL_DATA_T *oldData, ACCEL_DATA_T *newData) 
{
	int16_t difference = 0, scaledOld = 0, scaledNew = 0;
	
	scaledOld = (int16_t)(ACC_MAG_SCALING_PER_LSB*(oldData->x)); 
	scaledNew = (int16_t)(ACC_MAG_SCALING_PER_LSB*(newData->x)); 
	difference = abs(scaledOld - scaledNew);
	if (difference > MIN_DIFFERENCE) { return 1; }
	
	scaledOld = (int16_t)(ACC_MAG_SCALING_PER_LSB*(oldData->y)); 
	scaledNew = (int16_t)(ACC_MAG_SCALING_PER_LSB*(newData->y)); 
	difference = abs(scaledOld - scaledNew);
	if (difference > MIN_DIFFERENCE) { return 1; }
	
	scaledOld = (int16_t)(ACC_MAG_SCALING_PER_LSB*(oldData->z)); 
	scaledNew = (int16_t)(ACC_MAG_SCALING_PER_LSB*(newData->z)); 
	difference = abs(scaledOld - scaledNew);
	if (difference > MIN_DIFFERENCE) { return 1; }
	
	return 0;
}
#undef MIN_DIFFERENCE


/**	----------------------------------------------------------------------
**
**	@fn		Function		accelerometer_init
**
**	@brief	Please read header in include file for function description
**
**	----------------------------------------------------------------------
*/
ACC_MAG_INIT_RET_T accelerometer_init(void) {
	uint8_t ret = 0;
	ACC_MAG_INIT_RET_T accInitRet;
	
	// First, set red LED. This will be cleared when accelerometer is found on bus. (This is in case the who_am_i call never returns, we won't know what happened.)
	nrf_gpio_pin_set(RED_LED);
	// First check to see if accelerometer is on bus
	ret = accelerometer_whoami();
	if (ret) {
		nrf_gpio_pin_clear(RED_LED);
		nrf_gpio_pin_set(GREEN_LED);
		UART_DEBUG("Success: FXOS8700CQ found on i2c bus\r\n");
		nrf_delay_ms(50);
	} else { 
		UART_DEBUG("ERROR: FXOS8700CQ NOT found on i2c bus\r\n");
		return ACC_MAG_INIT_FAIL;
	}
#if DK_CONT_READ_DEMO
	accInitRet = accelerometer_config_continuous_read();
#endif

#if DK_TRANSIENT_DEMO
	accInitRet = accelerometer_config_transient();
#endif
	
#if DK_PULSE_DEMO
	accInitRet = accelerometer_config_pulse();
#endif	
	return accInitRet;
}

/**	----------------------------------------------------------------------
**
**	@fn		Function		verify_acc_mag_loop
**
**	@brief	Please read header in include file for function description
**
**	----------------------------------------------------------------------
*/
void verify_acc_mag_loop(void) 
{
	uint8_t testReturn = 0;
	while (1) 
	{
		nrf_delay_ms(200);
		testReturn = accelerometer_whoami();
		if (testReturn) 
		{
			UART_DEBUG("Success: FXOS8700CQ found on i2c bus\r\n");
		} 
		else 
		{
			UART_DEBUG("ERROR: FXOS8700CQ NOT found on i2c bus\r\n");
		}
	}	
}


ACC_MAG_INIT_RET_T accel_main_init(void) 
{
	ACC_MAG_INIT_RET_T accRet = ACC_MAG_INIT_SUCCESS;
#define RESET_PIN (29)
	
	UART_DEBUG("Welcome to FXOS8700CQ Unit Test\r\n");
	// TODO: Make configuration parameter to switch between dev board and target hardware
	nrf_gpio_cfg_output(RESET_PIN);
	nrf_gpio_pin_clear(RESET_PIN);
	nrf_delay_ms(200);
	i2c_devices_init();
	nrf_delay_ms(50);
	accRet = accelerometer_init();
	return accRet;
}

/** @} */
