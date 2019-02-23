/**	----------------------------------------------------------------------
**
**	@file		spi.h
**
**  @details This module implements the spi on the nRF52 for the PAVLOK
**  					product
**
**  @note The application must
**
**  @note Attention!
**
**
**	----------------------------------------------------------------------
*/
#ifndef _SPI_H_
#define _SPI_H_

/**	----------------------------------------------------------------------
**	@brief System Include(s)
**	----------------------------------------------------------------------
*/
#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include <string.h>
#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

/**	----------------------------------------------------------------------
**	@brief Project Include(s)
**	----------------------------------------------------------------------
*/
/**	----------------------------------------------------------------------
**	@brief	Global Data
**	----------------------------------------------------------------------
*/

#define PAVLOK_SPI_SS_PIN					(3)
#define PAVLOK_SPI_SCK_PIN					(4)
#define PAVLOK_SPI_MOSI_PIN					(5)
#define PAVLOK_SPI_MISO_PIN					(6)

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
**	@fn		Function	spi2_devices_init
**
**	@brief	Description initialize instance 2 of the SPI module
**
**	@param 	[in]		None
**
**	@param	[out]		None
**
**	@return				None				
**
**	@warn
**
**	----------------------------------------------------------------------
*/
void spi2_devices_init(void);
void spi2_devices_uninit(void);

/**	----------------------------------------------------------------------
**
**	@fn		Function	spi2_sync_txrx
**
**	@brief	Description low-level spi2 tx/rx transfer
**
**	@param 	[in]		uint8_t *txData
**
**	@param 	[in]		uint8_t txLength
**
**	@param 	[in]		uint8_t *rxData
**
**	@param 	[in]		uint8_t rxLength
**
**	@param	[out]		ret_code_t
**
**	@return				None				
**
**	@warn
**
**	----------------------------------------------------------------------
*/
ret_code_t spi2_sync_txrx(uint8_t *, uint8_t, uint8_t *, uint8_t);

#endif
