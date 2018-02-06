/**	----------------------------------------------------------------------
**
**	@file		i2c.h
**
**  @defgroup 	I2C
**  @{
**  @ingroup 	I2C
**  @brief port zero for the i2c bus.
**
**  @details These module implements the nRF52 i2c interface
**
**
**	----------------------------------------------------------------------
*/
#ifndef _I2C_H
#define _I2C_H

/**	----------------------------------------------------------------------
**	@brief System Include(s)
**	----------------------------------------------------------------------
*/
#include <stdint.h>
#include "boards.h"

/**	----------------------------------------------------------------------
**	@brief Project Include(s)
**	----------------------------------------------------------------------
*/

#include "rtc.h"
#include "accel_mag.h"
#if RTC_DEBUG
#warning "RTC_DEBUG IS SET!"
#endif
#if ACCEL_MAG_DEBUG
#warning "ACCEL_MAG_DEBUG IS SET!"
#endif
#if (RTC_DEBUG && ACCEL_MAG_DEBUG)
#error "CAN'T ENABLE BOTH RTC_DEBUG AND ACCEL_MAG_DEBUG!"
#endif

/**	----------------------------------------------------------------------
**	@brief	Global Data
**	----------------------------------------------------------------------
*/
/**	----------------------------------------------------------------------
**	@def	I2C Definitions
**	----------------------------------------------------------------------
*/
// Check to see if either RTC_DEBUG (for RTC) or ACCEL_MAG_DEBUG (for Accel) are set
#if (ACCEL_MAG_DEBUG)
#define I2C1_SDA_PIN 			(30)
#define I2C1_SCL_PIN 			(31)
#else
#define I2C1_SDA_PIN 			(0)
#define I2C1_SCL_PIN 			(1)
#endif
#if (RTC_DEBUG)
#define I2C0_SDA_PIN 			(30)
#define I2C0_SCL_PIN 			(31)
#else
#define I2C0_SDA_PIN			(12)
#define I2C0_SCL_PIN			(11)
#endif

#define USE_NON_BLOCKING	(0)

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
**	@fn			Function			i2c_devices_init
**
**	@brief	Description		initialization of the i2c0
**
**	@param	[in]					None
**
**	@param	[out]					None
**
**	@return								None
**
**	@warn									None
**
**	----------------------------------------------------------------------
*/
void i2c_devices_init(void);

/**	----------------------------------------------------------------------
**
**	@fn			Function			i2c0_sync_write
**
**	@brief	Description		data write for i2c0 bus
**
**	@param	[in]					uint8_t devAddr - bus device address
**	@param	[in]					uint8_t *data	- pointer to the write data
**	@param	[in]					uint8_t length	- write length in bytes
**
**	@param	[out]					None
**
**	@return								None
**
**	@warn									None
**
**	----------------------------------------------------------------------
*/
ret_code_t i2c0_sync_write(uint8_t, uint8_t *, uint8_t);

/**	----------------------------------------------------------------------
**
**	@fn			Function			i2c0_sync_read
**
**	@brief	Description		data read/write for i2c0 bus
**
**	@param	[in]					uint8_t devAddr - bus device address
**	@param	[in]					uint8_t *data	- pointer to the read buffer
**	@param	[in]					uint8_t length	- read length in bytes
**
**	@param	[out]					None
**
**	@return								ret_code_t
**
**	@retval								NRF_SUCCESS
**	@retval								non zero - errors
**
**	@warn									None
**
**	----------------------------------------------------------------------
*/
ret_code_t i2c0_sync_read(uint8_t, uint8_t *, uint8_t, uint8_t *, uint8_t);

/**	----------------------------------------------------------------------
**
**	@fn			Function			i2c1_sync_write
**
**	@brief	Description		data write for i2c0 bus
**
**	@param	[in]					uint8_t devAddr - bus device address
**	@param	[in]					uint8_t *data	- pointer to the write data
**	@param	[in]					uint8_t length	- write length in bytes
**
**	@param	[out]					None
**
**	@return								None
**
**	@warn									None
**
**	----------------------------------------------------------------------
*/
ret_code_t i2c1_sync_write(uint8_t, uint8_t *, uint8_t);

/**	----------------------------------------------------------------------
**
**	@fn			Function			i2c1_sync_read
**
**	@brief	Description		data read/write for i2c0 bus
**
**	@param	[in]					uint8_t devAddr - bus device address
**	@param	[in]					uint8_t *data	- pointer to the read buffer
**	@param	[in]					uint8_t length	- read length in bytes
**
**	@param	[out]					None
**
**	@return								ret_code_t
**
**	@retval								NRF_SUCCESS
**	@retval								non zero - errors
**
**	@warn									None
**
**	----------------------------------------------------------------------
*/
ret_code_t i2c1_sync_read(uint8_t, uint8_t *, uint8_t, uint8_t *, uint8_t);

extern int i2c_scan(int bus, uint8_t * buffer, int buf_len);


#endif /* _I2C_H */
/** @} */
