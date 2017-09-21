/**	----------------------------------------------------------------------
**
**	@file     gyro.c
**
**  @defgroup	
**  @{
**  @ingroup	
**  @brief 
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

/**	----------------------------------------------------------------------
**	@brief Project Include(s)
**	----------------------------------------------------------------------
*/
#include "gyro.h"
#include "i2c.h"
#include "string.h"

/**	----------------------------------------------------------------------
**	@brief	Static Forward References
**	----------------------------------------------------------------------
*/
//static uint8_t gyro_whoami(void);

/**	----------------------------------------------------------------------
**	@brief	Static Functions
**	----------------------------------------------------------------------
*/
/**	----------------------------------------------------------------------
**
**	@fn		  Function    gyro_whoami
**
**	@brief	Description Queries Gyro WHOAMI value
**
**	@param  [in]				None
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
uint8_t gyro_whoami(void) {
	ret_code_t errCode;
	uint8_t byte = 0;
	
	errCode = gyro_read(FXAS21002C_REG_WHOAMI, &byte, 1);
	
	if ((errCode != NRF_SUCCESS) || (byte != GYRO_FXAS21002C_WHOAMI_VAL)) {
		return 0;
	}
	return 1;
}

/**	----------------------------------------------------------------------
**	@brief	Global Functions
**	----------------------------------------------------------------------
*/
/** ----------------------------------------------------------------------
**
**	@fn		  Function			gyro_write
**
**	@brief	Description		Writes data to Gyro
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
GYRO_TXRX_RET_T gyro_write(uint8_t addr, uint8_t *data, uint8_t length) {
	ret_code_t errCode;
	gyroTXRXPacket_T gyroTXRXPacket;
	
	if (data == NULL) {
		return GYRO_NULL_DATA;
	}
	/*if (addr < GYRO_FXAS21002C_REG_RANGE_LOW) {
		return GYRO_INVALID_ADDRESS;
	}*/
	if (addr > GYRO_FXAS21002C_REG_RANGE_HIGH) {
		return GYRO_INVALID_ADDRESS;
	}
	if (length > I2C_GYRO_MAX_TRANSFER) {
		return GYRO_INVALID_SIZE;
	}
	gyroTXRXPacket.address = addr;
	memcpy(&gyroTXRXPacket.data[0], data, length);
	
	errCode = i2c0_sync_write(I2C_GYRO_ADDRESS, (uint8_t *)&gyroTXRXPacket, GYRO_FXAS21002C_ADDRESS_SIZE+length);
	
	if (NRF_SUCCESS == errCode) {
		return GYRO_SUCCESS;
	} else {
		return GYRO_TX_ERROR;	
	}
}

/** ----------------------------------------------------------------------
**
**	@fn		  Function			gyro_read
**
**	@brief	Description		Reads data from Gyro
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
GYRO_TXRX_RET_T gyro_read(uint8_t addr, uint8_t *data, uint8_t length) {
	ret_code_t errCode;
	accmagTXRXPacket_T gyroTXRXPacket;
	
	/*if (addr < GYRO_FXAS21002C_REG_RANGE_LOW) {
		return GYRO_INVALID_ADDRESS;
	}*/
	if (addr > GYRO_FXAS21002C_REG_RANGE_HIGH) {
		return GYRO_INVALID_ADDRESS;
	}
	if (length > I2C_GYRO_MAX_TRANSFER) {
		return GYRO_INVALID_SIZE;
	}
	memset(&gyroTXRXPacket.data[0], I2C_GYRO_MAX_TRANSFER, 0);
	gyroTXRXPacket.address = addr;
	
	errCode = i2c0_sync_read(I2C_GYRO_ADDRESS, (uint8_t *)&gyroTXRXPacket, GYRO_FXAS21002C_ADDRESS_SIZE, data, length);
	
	if (NRF_SUCCESS == errCode) {
		return GYRO_SUCCESS;
	} else {
		return GYRO_TX_ERROR;	
	}		
}

/** ----------------------------------------------------------------------
**
**	@fn		  Function			gyro_init
**
**	@brief	Description		Initializes Gyro
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
GYRO_INIT_RET_T gyro_init(void) {
	GYRO_INIT_RET_T ret = GYRO_INIT_SUCCESS;
	uint8_t val = 0;
	
	val = gyro_whoami();
	ret = ((val) ? GYRO_INIT_SUCCESS: GYRO_INIT_FAIL);
	
	return ret;
}

/** @} */
