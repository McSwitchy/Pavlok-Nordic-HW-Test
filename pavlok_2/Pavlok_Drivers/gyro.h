/**	----------------------------------------------------------------------
**
**	@file		  gyro.h
**
**  @details  This module implements the pwm on the nRF52 for the PAVLOK
**  		      product
**
**  @note    
**
**  @note
**
**
**	----------------------------------------------------------------------
*/
#ifndef _GYRO_H
#define _GYRO_H

/**	----------------------------------------------------------------------
**	@brief System Include(s)
**	----------------------------------------------------------------------
*/
#include <stdio.h>
#include "boards.h"

/**	----------------------------------------------------------------------
**	@brief Project Include(s)
**	----------------------------------------------------------------------
*/
/**	----------------------------------------------------------------------
**	@brief	Global Data
**	----------------------------------------------------------------------
*/

uint8_t gyro_whoami(void);

// Debug enable/disab;e
#define GYRO_DEBUG												(0)
#define GYRO_FXAS21002C_REG_RANGE_LOW 		(0x00)
#define GYRO_FXAS21002C_REG_RANGE_HIGH 		(0x15)
#define GYRO_FXAS21002C_REG_SIZE 					(1)
#define I2C_GYRO_ADDRESS									(0x20)
#define I2C_GYRO_MAX_TRANSFER							(8)
#define GYRO_FXAS21002C_ADDRESS_SIZE			(1)
#define GYRO_FXAS21002C_REGISTER_SIZE			(1)
#define GYRO_FXAS21002C_WHOAMI_VAL				(0xD7)
#define GYRO_INT1_PIN 										(23)
#define GYRO_INT2_PIN											(22)

/** ----------------------------------------------------------------------
**	@def	FXAS21002C internal register addresses
**	----------------------------------------------------------------------
*/
#define FXAS21002C_REG_WHOAMI 				    (0x0C)

/**	----------------------------------------------------------------------
**	@struct	gyroTXRXPacket_T
**	----------------------------------------------------------------------
*/
typedef struct {
	uint8_t address;
	uint8_t data[I2C_GYRO_MAX_TRANSFER];
} gyroTXRXPacket_T;

/**	----------------------------------------------------------------------
**	@enum	GYRO_INIT_RET_T
**	----------------------------------------------------------------------
*/
typedef enum {
	GYRO_INIT_SUCCESS,
	GYRO_INIT_FAIL
} GYRO_INIT_RET_T;

/**	----------------------------------------------------------------------
**	@enum	GYRO_TXRX_RET_T
**	----------------------------------------------------------------------
*/
typedef enum {
	GYRO_SUCCESS,
	GYRO_NULL_DATA,
	GYRO_INVALID_ADDRESS,
	GYRO_INVALID_SIZE,
	GYRO_TX_ERROR		
} GYRO_TXRX_RET_T;

/**	----------------------------------------------------------------------
**
**	@fn			Function	  gyro_init
**
**	@brief	Description 
**
**	@param  [in]			  None
**
**	@param	[out]		    None
**
**	@return					    GYRO_INIT_RET_T
**
**	@warn						    None
**
**	----------------------------------------------------------------------
*/
GYRO_INIT_RET_T gyro_init(void);

/**	----------------------------------------------------------------------
**
**	@fn			Function	  gyro_write
**
**	@brief	Description Writes data to Gyro
**
**	@param  [in]			  uint8_t address
**
**	@param  [in]			  uint8_t* data
**
**	@param  [in]			  uint8_t length
**
**	@param	[out]		    None
**
**	@return					    GYRO_INIT_RET_T
**
**	@warn						    None
**
**	----------------------------------------------------------------------
*/
GYRO_TXRX_RET_T gyro_write(uint8_t, uint8_t *, uint8_t);

/**	----------------------------------------------------------------------
**
**	@fn			Function	  gyro_read
**
**	@brief	Description Reads data from Gyro
**
**	@param  [in]			  uint8_t address
**
**	@param  [in]			  uint8_t* data
**
**	@param  [in]			  uint8_t length
**
**	@param	[out]		    None
**
**	@return					    GYRO_INIT_RET_T
**
**	@warn						    None
**
**	----------------------------------------------------------------------
*/
GYRO_TXRX_RET_T gyro_read(uint8_t, uint8_t *, uint8_t);

#endif
