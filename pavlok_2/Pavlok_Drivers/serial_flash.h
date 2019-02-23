/*------------------------------------------------------------------------
**
**	@file
**
**  @brief Description
**
**  @details This module implements
**
**  @note The application must
**
**  @note Attention!
**
**------------------------------------------------------------------------
*/
#ifndef _SERIAL_FLASH_H_
#define _SERIAL_FLASH_H_

/**	----------------------------------------------------------------------
**	@brief System Include(s)
**	----------------------------------------------------------------------
*/
#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"

/**	----------------------------------------------------------------------
**	@brief Project Include(s)
**	----------------------------------------------------------------------
*/

/**	----------------------------------------------------------------------
**	@brief	Global Data
**	----------------------------------------------------------------------
*/

uint8_t demo_flash_read_write_read(void);
int serial_flash_chip_erase(void);

/** ----------------------------------------------------------------------
**	@def	W25X40CL Definitions
**	----------------------------------------------------------------------
*/
// The max transfer bytes is defined by the control structure of the nrf52 SPI driver
#define W25X40CL_MAX_TRANSFER_BYTES 					(240)
#define W25X40CL_PAGE_SIZE_BYTES							(256)
#define W25X40CL_REG_RANGE_LOW								(0x00)
#define W25X40CL_REG_RANGE_HIGH								(0x78)
#define W25X40CL_REG_SIZE											(2)
#define W25X40CL_MAX_TRANSFER									(16)
#define W25X40CL_REGISTER_SIZE								(1)
#define W25X40CL_MAX_ADDRESS                            (uint32_t)(0x7FFFF)

#define W25X40CL_OPCODE_DEVICE_ID							(0xAB)
#define W25X40CL_DEVICE_ID										(0x12)
#define W25X40CL_OPCODE_READ_DATA							(0x03)
#define W25X40CL_OPCODE_WRITE_DISABLE					(0x04)
#define W25X40CL_OPCODE_READ_STATUS						(0x05)
#define W25X40CL_OPCODE_WRITE_ENABLE					(0x06)
#define W25X40CL_OPCODE_PAGE_PROGRAM					(0x02)
#define W25X40CL_OPCODE_SECTOR_ERASE					(0x20)
#define W25X40CL_OPCODE_CHIP_ERASE                      0x60

// Status register definitions
#define STATUS_BUSY														(uint8_t)(1<<0)
#define STATUS_WEL														(uint8_t)(1<<1)
#define STATUS_BP0														(uint8_t)(1<<2)
#define STATUS_BP1														(uint8_t)(1<<3)
#define STATUS_BP2														(uint8_t)(1<<4)
#define STATUS_TB															(uint8_t)(1<<5)
#define STATUS_RESERVED												(uint8_t)(1<<6)
#define STATUS_SRP														(uint8_t)(1<<7)

#define IS_STATUS_BIT_TRUE(x, y)							(uint8_t)(x & y)

/**	----------------------------------------------------------------------
**	@enum	WEL_CONTROL_T
**	----------------------------------------------------------------------
*/
typedef enum {
	WRITE_DISABLE = 0,
	WRITE_ENABLE
} WEL_CONTROL_T;

/**	----------------------------------------------------------------------
**	@struct	STATUS_REGISTER_T
**	----------------------------------------------------------------------
*/
typedef struct {
	uint8_t busy:1;
	uint8_t wel:1;
	uint8_t bp0:1;
	uint8_t bp1:1;
	uint8_t bp2:1;
	uint8_t tb:1;
	uint8_t reserved:1;
	uint8_t srp:1;
} STATUS_REGISTER_T;

/**	----------------------------------------------------------------------
**	@enum	SERIAL_FLASH_TXRX_RET_T
**	----------------------------------------------------------------------
*/
typedef enum {
	SERIAL_FLASH_SUCCESS,
	SERIAL_FLASH_NULL_DATA,
	SERIAL_FLASH_INVALID_ADDRESS,
	SERIAL_FLASH_INVALID_CODE,
	SERIAL_FLASH_INVALID_SIZE,
	SERIAL_FLASH_TXRX_ERROR,
	SERIAL_FLASH_INVALID_PARAMETER,
	SERIAL_FLASH_UNABLE_TO_ENABLE_WRITE_LATCH,
	SERIAL_FLASH_BUSY,
	SERIAL_FLASH_INVALID_BOUNDARY
} SERIAL_FLASH_TXRX_RET_T;

/**	----------------------------------------------------------------------
**	@brief	Global Extern(s)
**	----------------------------------------------------------------------
*/

/**	----------------------------------------------------------------------
**	@brief	Global Functions
**	----------------------------------------------------------------------
*/

/**	----------------------------------------------------------------------
**
**	@fn			Function	  serial_flash_check_get_device_id
**
**	@brief	Description Queries flash chip's device ID
**
**	@param  [in]		    None
**
**	@param	[out]		    None
**
**	@return					    SERIAL_FLASH_TXRX_RET_T
**
**	@warn
**
**	----------------------------------------------------------------------
*/
SERIAL_FLASH_TXRX_RET_T serial_flash_check_get_device_id(void);

/**	----------------------------------------------------------------------
**
**	@fn			Function	  serial_flash_get_status
**
**	@brief	Description Queries flash chip's status
**
**	@param  [in]		    STATUS_REGISTER_T*
**
**	@param	[out]		    None
**
**	@return					    SERIAL_FLASH_TXRX_RET_T
**
**	@warn
**
**	----------------------------------------------------------------------
*/
SERIAL_FLASH_TXRX_RET_T serial_flash_get_status(STATUS_REGISTER_T *);

/**	----------------------------------------------------------------------
**
**	@fn			Function	  serial_flash_write_control
**
**	@brief	Description Enables/Disables write enable latch
**
**	@param  [in]		    WEL_CONTROL_T
**
**	@param	[out]		    None
**
**	@return					    SERIAL_FLASH_TXRX_RET_T
**
**	@warn
**
**	----------------------------------------------------------------------
*/
SERIAL_FLASH_TXRX_RET_T serial_flash_write_control(WEL_CONTROL_T);

/**	----------------------------------------------------------------------
**
**	@fn			Function	  serial_flash_read_data
**
**	@brief	Description Reads data from the flash chip
**
**	@param  [in]		    uint8_t* txData
**
**  @param  [in]		    uint8_t txLength
**
**  @param  [in]		    uint8_t* rxData
**
**  @param  [in]		    uint8_t rxLength
**
**	@param	[out]		    None
**
**	@return					    SERIAL_FLASH_TXRX_RET_T
**
**	@warn
**
**	----------------------------------------------------------------------
*/
SERIAL_FLASH_TXRX_RET_T serial_flash_read_data(uint8_t *, uint32_t, uint32_t);

/**	----------------------------------------------------------------------
**
**	@fn			Function	  serial_flash_write_data
**
**	@brief	Description Writes data from the flash chip
**
**	@param  [in]		    uint8_t* txData
**
**  @param  [in]		    uint8_t txLength
**
**  @param  [in]		    uint8_t* rxData
**
**  @param  [in]		    uint8_t rxLength
**
**	@param	[out]		    None
**
**	@return					    SERIAL_FLASH_TXRX_RET_T
**
**	@warn
**
**	----------------------------------------------------------------------
*/
SERIAL_FLASH_TXRX_RET_T serial_flash_write_data(uint8_t *, uint32_t, uint32_t);

/**	----------------------------------------------------------------------
**
**	@fn			Function	  serial_flash_sector_erase
**
**	@brief	Description Erases sector from the flash chip
**
**	@param  [in]		    uint32_t sector
**
**	@param	[out]		    None
**
**	@return					    SERIAL_FLASH_TXRX_RET_T
**
**	@warn
**
**	----------------------------------------------------------------------
*/
SERIAL_FLASH_TXRX_RET_T serial_flash_sector_erase(uint32_t);

#endif
