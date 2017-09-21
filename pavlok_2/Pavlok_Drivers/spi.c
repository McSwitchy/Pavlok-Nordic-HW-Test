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
#include "spi.h"

/**	----------------------------------------------------------------------
**	@brief	Global Data
**	----------------------------------------------------------------------
*/
#define SPI_INSTANCE	2 /**< SPI instance index. */

/**	----------------------------------------------------------------------
**	@brief	Global Extern(s)
**	----------------------------------------------------------------------
*/

/**	----------------------------------------------------------------------
**	@brief	Static Data
**	----------------------------------------------------------------------
*/
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

/**	----------------------------------------------------------------------
**	@brief	Static Forward References
**	----------------------------------------------------------------------
*/

/**	----------------------------------------------------------------------
**	@brief	Static Functions
**	----------------------------------------------------------------------
*/

/**	----------------------------------------------------------------------
**	@brief	Global Functions
**	----------------------------------------------------------------------
*/
/** ----------------------------------------------------------------------
**
**	@fn		Function		spi_event_handler
**
**	@brief	Description		Callback for asynchronous SPI functions
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
void spi_event_handler(nrf_drv_spi_evt_t const * p_event) 
{
   spi_xfer_done = true;
}

/** ----------------------------------------------------------------------
**
**	@fn		Function		spi2_devices_init
**
**	@brief	Description		initialize instance 2 of the SPI module
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
void spi2_devices_init(void) 
{
	ret_code_t ret = 0;
	
	nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
	spi_config.sck_pin  = PAVLOK_SPI_SCK_PIN;
	spi_config.miso_pin = PAVLOK_SPI_MISO_PIN;
	spi_config.mosi_pin = PAVLOK_SPI_MOSI_PIN;
	// The SS pin shouldn't be controlled automatically by the peripheral. 
	// It will be configured as a GPIO and manually controlled in order to obtain specfic timings.
	spi_config.ss_pin = NRF_DRV_SPI_PIN_NOT_USED;
	
	spi_config.mode = NRF_DRV_SPI_MODE_0;
	
	// Manually configure SS line
	spi_config.frequency = NRF_DRV_SPI_FREQ_8M;
	nrf_gpio_cfg_output(PAVLOK_SPI_SS_PIN);
	
	//nrf_gpio_cfg(
	//	PAVLOK_SPI_SS_PIN,
	//	NRF_GPIO_PIN_DIR_OUTPUT,
	//	NRF_GPIO_PIN_INPUT_DISCONNECT,
	//	NRF_GPIO_PIN_NOPULL,
	//	NRF_GPIO_PIN_S0D1,
	//	NRF_GPIO_PIN_NOSENSE);
	
	nrf_gpio_pin_set(PAVLOK_SPI_SS_PIN);
	
	ret = nrf_drv_spi_init(&spi, &spi_config, NULL, NULL);
	APP_ERROR_CHECK(ret);
  
}
void spi2_devices_uninit(void) 
{
  // contract void nrf_drv_spi_uninit(nrf_drv_spi_t const * const p_instance)
  nrf_drv_spi_uninit(&spi);
}
/** ----------------------------------------------------------------------
**
**	@fn		Function		spi2_sync_txrx
**
**	@brief	Description		low-level spi2 tx/rx transfer
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
ret_code_t spi2_sync_txrx(uint8_t *tx, uint8_t txLength, uint8_t *rx, uint8_t rxLength) 
{
	ret_code_t errCode = 0;
	
	nrf_gpio_pin_clear(PAVLOK_SPI_SS_PIN);
	errCode = nrf_drv_spi_transfer(&spi, tx, txLength, rx, rxLength);
	nrf_gpio_pin_set(PAVLOK_SPI_SS_PIN);
	
	return errCode;
}

/** @} */
