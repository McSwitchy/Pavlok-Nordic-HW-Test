/*------------------------------------------------------------------------
**
**	@file
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
#include "serial_flash.h"
#include "spi.h"

/**	----------------------------------------------------------------------
**	@brief	Global Data
**	----------------------------------------------------------------------
*/

/**	----------------------------------------------------------------------
**	@brief	Global Extern(s)
**	----------------------------------------------------------------------
*/

/**	----------------------------------------------------------------------
**	@brief	Static Data
**	----------------------------------------------------------------------
*/
// These two buffers are intended to be used with the read/write data commands.
// The user will typically pass in a buffer, address, and length. The operations, 
// however, always require additional dummy bytes. To make this transparent to the 
// user, these two buffers are used during the tx/rx and then their data is copied
// into the user supplied buffer. The extra 8 bytes are used for control/dummy bytes.

#define SPI_TXRX_BUF_GLOBAL_SIZE_BYTES (W25X40CL_MAX_TRANSFER_BYTES + 8)
static uint8_t spiTxBufGlobal[SPI_TXRX_BUF_GLOBAL_SIZE_BYTES] = {0};
static uint8_t spiRxBufGlobal[SPI_TXRX_BUF_GLOBAL_SIZE_BYTES] = {0};
#define get_buffers(x, y)					uint8_t txBuf[x] = {0}; \
											uint8_t rxBuf[y] = {0}
/**	----------------------------------------------------------------------
**	@brief	Static Forward References
**	----------------------------------------------------------------------
*/
static SERIAL_FLASH_TXRX_RET_T serial_flash_txrx(uint8_t *txData, uint8_t txLength, uint8_t *rxData, uint8_t rxLength);
static void set_instruction(uint8_t *, uint8_t);

#define TRANSFER_SIZE 	(240)

uint8_t dataIn[TRANSFER_SIZE] 	= {0};
uint8_t dataOut[TRANSFER_SIZE] 	= {0};

uint8_t demo_flash_read_write_read(void) {
	uint16_t i = 0;
  
	serial_flash_sector_erase(0);
	nrf_delay_ms(305);
	
  memset(dataIn, 0, sizeof(dataIn));
	serial_flash_read_data(dataIn, 0, TRANSFER_SIZE);
	for (i=0; i<TRANSFER_SIZE; i++) {
		if (dataIn[i] != 0xFF) {
			return 0;
		}
	}
	
	memset(dataOut, 0xAA, sizeof(dataOut));
	serial_flash_write_data(dataOut, 0, sizeof(dataOut));
	nrf_delay_ms(1);
	
	serial_flash_read_data(dataIn, 0, TRANSFER_SIZE);
	for (i=0; i<TRANSFER_SIZE; i++) {
		if (dataIn[i] != 0xAA) {
			return 0;
		}
	}

  memset(dataIn, 0, sizeof(dataIn));
  serial_flash_sector_erase(0);
	nrf_delay_ms(305);
	
	serial_flash_read_data(dataIn, 0, TRANSFER_SIZE);
	for (i=0; i<TRANSFER_SIZE; i++) {
		if (dataIn[i] != 0xFF) {
			return 0;
		}
	}
  
  /*
	memset(dataIn, 0, sizeof(dataIn));
	
	serial_flash_sector_erase(0);
	nrf_delay_ms(305);
	
	memset(dataOut, 0xBB, sizeof(dataOut));
	serial_flash_write_data(dataOut, 0, TRANSFER_SIZE);
	nrf_delay_ms(1);
	
	serial_flash_read_data(dataIn, 0, TRANSFER_SIZE);
	for (i=0; i<TRANSFER_SIZE; i++) {
		if (dataIn[i] != 0xBB) {
			return 0;
		}
	}
  */
  
  return 1;
}                      
                      
/**	----------------------------------------------------------------------
**	@brief	Static Functions
**	----------------------------------------------------------------------
*/
/**	----------------------------------------------------------------------
**
**	@fn		Function  	serial_flash_txrx
**
**	@brief	Description	Mid-level function to transfer SPI data
**
**	@param  [in]		uint8_t* txData
**                                  
**  @param  [in]		uint8_t txLength
**                                  
**  @param  [in]		uint8_t* rxData   
**                                  
**  @param  [in]		uint8_t rxLength   
**
**	@param  [out]		None
**
**	@return				None
**
**	@warn
**
**	----------------------------------------------------------------------
*/   
static SERIAL_FLASH_TXRX_RET_T serial_flash_txrx(uint8_t *txData, uint8_t txLength, uint8_t *rxData, uint8_t rxLength) 
{
	ret_code_t errCode = 0;
	
#if 0
	if (*txData == NULL && txLength > 0 ) {
		return SERIAL_FLASH_NULL_DATA;
	}
	if (*rxData == NULL && rxLength > 0 ) {
		return SERIAL_FLASH_NULL_DATA;
	}
	if (*rxData != NULL && rxLength == 0) {
		return SERIAL_FLASH_INVALID_SIZE;
	}
	if (*txData != NULL && txLength == 0) {
		return SERIAL_FLASH_INVALID_SIZE;
	}
#endif
	
	errCode = spi2_sync_txrx(txData, txLength, rxData, rxLength);
	
	if (errCode != NRF_SUCCESS) {
		return SERIAL_FLASH_TXRX_ERROR;
	}
	
	return SERIAL_FLASH_SUCCESS;
}	

/**	----------------------------------------------------------------------
**
**	@fn		Function  	set_instruction
**
**	@brief	Description	Sets instruction's op code
**
**	@param  [in]		uint8_t* data
**                                  
**  @param  [in]		uint8_t opcode                                  
**
**	@param  [out]		None
**
**	@return				None
**
**	@warn
**
**	----------------------------------------------------------------------
*/   
static void set_instruction(uint8_t *buffer, uint8_t instructionCode) 
{
	buffer[0] = instructionCode;
}

/**	----------------------------------------------------------------------
**	@brief	Global Functions
**	----------------------------------------------------------------------
*/
/** ----------------------------------------------------------------------
**
**	@fn		Function		serial_flash_check_get_device_id
**
**	@brief	Description		Queries flash chip's device ID
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
#define READ_DEVICE_ID_TXRX_SIZE (5)
SERIAL_FLASH_TXRX_RET_T serial_flash_check_get_device_id(void) 
{
	get_buffers(READ_DEVICE_ID_TXRX_SIZE, READ_DEVICE_ID_TXRX_SIZE);
	set_instruction(txBuf, W25X40CL_OPCODE_DEVICE_ID);
	
	if (serial_flash_txrx(txBuf, READ_DEVICE_ID_TXRX_SIZE, rxBuf, READ_DEVICE_ID_TXRX_SIZE) != NRF_SUCCESS) { return SERIAL_FLASH_TXRX_ERROR; }
	
	// Loop through rx buf until device ID is found
	for (uint8_t i=0; i<READ_DEVICE_ID_TXRX_SIZE; i++) {
		if (rxBuf[i] == W25X40CL_DEVICE_ID) {
			return SERIAL_FLASH_SUCCESS;
		}
	}
	
	return SERIAL_FLASH_TXRX_ERROR;
}
#undef READ_DEVICE_ID_TXRX_SIZE

/** ----------------------------------------------------------------------
**
**	@fn		Function		serial_flash_get_status
**
**	@brief	Description		Queries flash chip's status
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
#define READ_STATUS_TXRX_SIZE (2)
SERIAL_FLASH_TXRX_RET_T serial_flash_get_status(STATUS_REGISTER_T *status) 
{
	volatile uint8_t statusRegister = 0;
	
	get_buffers(READ_STATUS_TXRX_SIZE, READ_STATUS_TXRX_SIZE);
	set_instruction(txBuf, W25X40CL_OPCODE_READ_STATUS);
	
	if (serial_flash_txrx(txBuf, READ_STATUS_TXRX_SIZE, rxBuf, READ_STATUS_TXRX_SIZE) != NRF_SUCCESS) { return SERIAL_FLASH_TXRX_ERROR; }
	
	statusRegister = rxBuf[1];
	
	status->busy 	= IS_STATUS_BIT_TRUE(statusRegister, STATUS_BUSY) ? 1 : 0;
	status->wel		= IS_STATUS_BIT_TRUE(statusRegister, STATUS_WEL) 	? 1 : 0;
	status->bp0 	= IS_STATUS_BIT_TRUE(statusRegister, STATUS_BP0) 	? 1 : 0;
	status->bp1 	= IS_STATUS_BIT_TRUE(statusRegister, STATUS_BP1) 	? 1 : 0;
	status->bp2 	= IS_STATUS_BIT_TRUE(statusRegister, STATUS_BP2) 	? 1 : 0;
	status->tb		= IS_STATUS_BIT_TRUE(statusRegister, STATUS_TB) 	? 1 : 0;
	status->srp		= IS_STATUS_BIT_TRUE(statusRegister, STATUS_SRP) 	? 1 : 0;
	
	return SERIAL_FLASH_SUCCESS;
}
#undef READ_STATUS_TXRX_SIZE

/** ----------------------------------------------------------------------
**
**	@fn		Function		serial_flash_write_control
**
**	@brief	Description		Enables/Disables write enable latch
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
#define WRITE_WEL_TXRX_SIZE (1)
#define LATCH_CONTROL_RETRY (10)
SERIAL_FLASH_TXRX_RET_T serial_flash_write_control(WEL_CONTROL_T control) 
{
	get_buffers(WRITE_WEL_TXRX_SIZE, WRITE_WEL_TXRX_SIZE);
	
	if (control == WRITE_ENABLE) {
		set_instruction(txBuf, W25X40CL_OPCODE_WRITE_ENABLE);
	} else if (control == WRITE_DISABLE) {
		set_instruction(txBuf, W25X40CL_OPCODE_WRITE_DISABLE);
	} else {
		return SERIAL_FLASH_INVALID_PARAMETER;
	}
	
	if (serial_flash_txrx(txBuf, WRITE_WEL_TXRX_SIZE, rxBuf, WRITE_WEL_TXRX_SIZE) != NRF_SUCCESS) { return SERIAL_FLASH_TXRX_ERROR; }
	
	nrf_delay_us(10);

#if 0
	uint16_t retry = 0;
	do {
		// Let's check to see if the latch has been enabled/disabled
		if (serial_flash_get_status(&status) != SERIAL_FLASH_SUCCESS) { return SERIAL_FLASH_TXRX_ERROR; }
		
		retry++;
	} while (retry < LATCH_CONTROL_RETRY && status.wel != control);
	
	if (retry == LATCH_CONTROL_RETRY) { return SERIAL_FLASH_UNABLE_TO_ENABLE_WRITE_LATCH; }
#endif		
	
	return SERIAL_FLASH_SUCCESS;
}
#undef WRITE_WEL_TXRX_SIZE
#undef LATCH_CONTROL_RETRY

/** ----------------------------------------------------------------------
**
**	@fn		Function		serial_flash_read_data
**
**	@brief	Description		Reads data from the flash chip
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
#define READ_DATA_CODE_ADDR_SIZE (4)
SERIAL_FLASH_TXRX_RET_T serial_flash_read_data(uint8_t *data, uint32_t address, uint32_t length) 
{
	STATUS_REGISTER_T status  = {0};
	
	// Check parameters
	if (address > W25X40CL_MAX_ADDRESS) {
		return SERIAL_FLASH_INVALID_ADDRESS;
	}
	if (length > W25X40CL_MAX_TRANSFER_BYTES) {
		return SERIAL_FLASH_INVALID_SIZE;
	}
	if (length == 0) {
		return SERIAL_FLASH_INVALID_SIZE;
	}
	if (data == NULL) {
		return SERIAL_FLASH_NULL_DATA;
	}
	
	// Check to see if chip is busy
	if (serial_flash_get_status(&status) != SERIAL_FLASH_SUCCESS) { return SERIAL_FLASH_TXRX_ERROR; }
	if (status.busy == 1) { 
		return SERIAL_FLASH_BUSY; 
	}
	
	// Clear global buffers
	memset(spiTxBufGlobal, SPI_TXRX_BUF_GLOBAL_SIZE_BYTES, 0);
	memset(spiRxBufGlobal, SPI_TXRX_BUF_GLOBAL_SIZE_BYTES, 0);
	
	// Populate tx global buffer with op code and data address
	spiTxBufGlobal[0] = W25X40CL_OPCODE_READ_DATA;
	spiTxBufGlobal[1] = ((address & (uint32_t)0xFF0000) >> 16);
	spiTxBufGlobal[2] = ((address & (uint32_t)0x00FF00) >> 8);
	spiTxBufGlobal[3] = ((address & (uint32_t)0x0000FF));
	
	if (serial_flash_txrx(spiTxBufGlobal, READ_DATA_CODE_ADDR_SIZE+length, spiRxBufGlobal, READ_DATA_CODE_ADDR_SIZE+length) != NRF_SUCCESS) { return SERIAL_FLASH_TXRX_ERROR; }
	
	// Copy data into user provided buffer after stripping dummy bytes
	memcpy(data, spiRxBufGlobal+READ_DATA_CODE_ADDR_SIZE, length);
	
	return SERIAL_FLASH_SUCCESS;
}
#undef READ_DATA_CODE_ADDR_SIZE

/** ----------------------------------------------------------------------
**
**	@fn		Function		serial_flash_write_data
**
**	@brief	Description		Writes data from the flash chip
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
#define WRITE_DATA_CODE_ADDR_SIZE (4)
SERIAL_FLASH_TXRX_RET_T serial_flash_write_data(uint8_t *data, uint32_t address, uint32_t length) 
{
	STATUS_REGISTER_T status  = {0};
	uint16_t reference = 0;
	
	reference = address % W25X40CL_PAGE_SIZE_BYTES;
	
	// Check parameters
	if (address > W25X40CL_MAX_ADDRESS) {
		return SERIAL_FLASH_INVALID_ADDRESS;
	}
	if (length > W25X40CL_MAX_TRANSFER_BYTES) {
		return SERIAL_FLASH_INVALID_SIZE;
	}
	if ((reference + length) > W25X40CL_PAGE_SIZE_BYTES) {
		return SERIAL_FLASH_INVALID_BOUNDARY;
	}
	if (length == 0) {
		return SERIAL_FLASH_INVALID_SIZE;
	}
	if (data == NULL) {
		return SERIAL_FLASH_NULL_DATA;
	}
	
	// Check to see if chip is busy
	if (serial_flash_get_status(&status) != SERIAL_FLASH_SUCCESS) { return SERIAL_FLASH_TXRX_ERROR; }
	if (status.busy == 1) { 
		return SERIAL_FLASH_BUSY; 
	}
	
	// Clear global buffers
	memset(spiTxBufGlobal, SPI_TXRX_BUF_GLOBAL_SIZE_BYTES, 0);
	memset(spiRxBufGlobal, SPI_TXRX_BUF_GLOBAL_SIZE_BYTES, 0);
	
	// Populate tx global buffer with op code and data address
	spiTxBufGlobal[0] = W25X40CL_OPCODE_PAGE_PROGRAM;
	spiTxBufGlobal[1] = ((address & (uint32_t)0xFF0000) >> 16);
	spiTxBufGlobal[2] = ((address & (uint32_t)0x00FF00) >> 8);
	spiTxBufGlobal[3] = ((address & (uint32_t)0x0000FF));
	
	// Copy data into buffer
	memcpy(&spiTxBufGlobal[4], data, length);
	
	// Disable/Enable Write latch
	if (serial_flash_write_control(WRITE_DISABLE) != SERIAL_FLASH_SUCCESS) { return SERIAL_FLASH_TXRX_ERROR; }
	if (serial_flash_write_control(WRITE_ENABLE) != SERIAL_FLASH_SUCCESS) { return SERIAL_FLASH_TXRX_ERROR; }
	serial_flash_get_status(&status);
	
	if (serial_flash_txrx(spiTxBufGlobal, WRITE_DATA_CODE_ADDR_SIZE+length, spiRxBufGlobal, WRITE_DATA_CODE_ADDR_SIZE+length) != NRF_SUCCESS) { return SERIAL_FLASH_TXRX_ERROR; }
	
	return SERIAL_FLASH_SUCCESS;
}
#undef WRITE_DATA_CODE_ADDR_SIZE

/** ----------------------------------------------------------------------
**
**	@fn		Function		serial_flash_sector_erase
**
**	@brief	Description		Erases sector from the flash chip
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
#define SECTOR_ERASE_CODE_ADDR_SIZE (4)
SERIAL_FLASH_TXRX_RET_T serial_flash_sector_erase(uint32_t address) 
{
	STATUS_REGISTER_T status  = {0};
	
	// Check parameters
	if (address > W25X40CL_MAX_ADDRESS) {
		return SERIAL_FLASH_INVALID_ADDRESS;
	}
	
	// Check to see if chip is busy
	if (serial_flash_get_status(&status) != SERIAL_FLASH_SUCCESS) { return SERIAL_FLASH_TXRX_ERROR; }
	if (status.busy == 1) { 
		return SERIAL_FLASH_BUSY; 
	}
	
	// Clear global buffers
	memset(spiTxBufGlobal, SPI_TXRX_BUF_GLOBAL_SIZE_BYTES, 0);
	memset(spiRxBufGlobal, SPI_TXRX_BUF_GLOBAL_SIZE_BYTES, 0);
	
	// Populate tx global buffer with op code and data address
	spiTxBufGlobal[0] = W25X40CL_OPCODE_SECTOR_ERASE;
	spiTxBufGlobal[1] = ((address & (uint32_t)0xFF0000) >> 16);
	spiTxBufGlobal[2] = ((address & (uint32_t)0x00FF00) >> 8);
	spiTxBufGlobal[3] = ((address & (uint32_t)0x0000FF));
	
	if (serial_flash_write_control(WRITE_DISABLE) != SERIAL_FLASH_SUCCESS) { return SERIAL_FLASH_TXRX_ERROR; }
	if (serial_flash_write_control(WRITE_ENABLE) != SERIAL_FLASH_SUCCESS) { return SERIAL_FLASH_TXRX_ERROR; }
	serial_flash_get_status(&status);
	
	if (serial_flash_txrx(spiTxBufGlobal, SECTOR_ERASE_CODE_ADDR_SIZE, spiRxBufGlobal, SECTOR_ERASE_CODE_ADDR_SIZE) != NRF_SUCCESS) { return SERIAL_FLASH_TXRX_ERROR; }
	
	return SERIAL_FLASH_SUCCESS;
}
#undef SECTOR_ERASE_CODE_ADDR_SIZE

/** @} */
