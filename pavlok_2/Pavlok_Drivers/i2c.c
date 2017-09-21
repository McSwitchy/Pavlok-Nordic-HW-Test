/**	----------------------------------------------------------------------
**
**	@file		i2c.c
**
**  @ingroup 	I2C
**  @brief Alert Notification module.
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
**	@brief System Include(s)
**	----------------------------------------------------------------------
*/
#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"

/**	----------------------------------------------------------------------
**	@brief Project Include(s)
**	----------------------------------------------------------------------
*/
#include "i2c.h"

/**	----------------------------------------------------------------------
**	@brief	Global Data
**	----------------------------------------------------------------------
*/
/**	----------------------------------------------------------------------
**	@def	TWI instance ID.
**	----------------------------------------------------------------------
*/
#define TWI_INSTANCE_ID_0	0
#define TWI_INSTANCE_ID_1	1

/**	----------------------------------------------------------------------
**	@brief	Static Data
**	TWI instance.
**	----------------------------------------------------------------------
*/
static const nrf_drv_twi_t m_twi0 = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID_0);
static const nrf_drv_twi_t m_twi1 = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID_1);
static uint8_t twiInitialized = 0;
static volatile uint8_t twi0XferCompleted = false;
static volatile uint8_t twi1XferCompleted = false;

/**	----------------------------------------------------------------------
**	@brief	Static Forward References
**	----------------------------------------------------------------------
*/
/**	----------------------------------------------------------------------
**
**	@fn			Function			twi_event_handler
**
**	@brief	Description		soft device call back for the i2c
**
**	@param 	[in]					nrf_drv_twi_evt_t * p_event
**	@param 	[in]					void * p_context
**
**	@param	[out]					None
**
**	@return								None
**
**	@warn									None
**
**	----------------------------------------------------------------------
*/
void twi_event_handler_twi0(nrf_drv_twi_evt_t * p_event, void * p_context);

/**	----------------------------------------------------------------------
**
**	@fn		Function		i2c_devices_init
**
**	@brief	Please read header in include file for function description
**
**	----------------------------------------------------------------------
*/
void i2c_devices_init(void) 
{
	ret_code_t err_code;

	if (twiInitialized) 
	{
		return;
	}
	
	const nrf_drv_twi_config_t twi0_config = {
		 .scl                = I2C0_SCL_PIN,
		 .sda                = I2C0_SDA_PIN,
		 .frequency          = NRF_TWI_FREQ_400K,
		 .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
		 .clear_bus_init     = false
	};
	
	const nrf_drv_twi_config_t twi1_config = {
		 .scl                = I2C1_SCL_PIN,
		 .sda                = I2C1_SDA_PIN,
		 .frequency          = NRF_TWI_FREQ_400K,
		 .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
		 .clear_bus_init     = false
	};

#if RTC_DEBUG	
	
	err_code = nrf_drv_twi_init(&m_twi0, &twi0_config, NULL, NULL);
	APP_ERROR_CHECK(err_code);
	nrf_drv_twi_enable(&m_twi0);

#elif ACCEL_MAG_DEBUG
	
	err_code = nrf_drv_twi_init(&m_twi1, &twi1_config, NULL, NULL);
	APP_ERROR_CHECK(err_code);
	nrf_drv_twi_enable(&m_twi1);
	
#else
	
	err_code = nrf_drv_twi_init(&m_twi0, &twi0_config, NULL, NULL);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_twi_init(&m_twi1, &twi1_config, NULL, NULL);
	APP_ERROR_CHECK(err_code);
	nrf_drv_twi_enable(&m_twi0);
	nrf_drv_twi_enable(&m_twi1);

#endif
	
	twiInitialized = 1;
}

/**	----------------------------------------------------------------------
**
**	@fn		Function		twi_event_handler
**
**	@brief	Please read header in include file for function description
**
**	----------------------------------------------------------------------
*/
void twi_event_handler_twi0(nrf_drv_twi_evt_t * p_event, void * p_context)
{
    if ((p_event->type == NRF_DRV_TWI_EVT_DONE) && (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_TXRX)) 
	{ 
		twi0XferCompleted = true; 
	} 
}

/**	----------------------------------------------------------------------
**
**	@fn		Function		i2c0_sync_write
**
**	@brief	Please read header in include file for function description
**
**	----------------------------------------------------------------------
*/
ret_code_t i2c0_sync_write(uint8_t devAddr, uint8_t *data, uint8_t length) 
{
	ret_code_t err_code = 0;
	
	err_code = nrf_drv_twi_tx(&m_twi0, devAddr, data, length, false);
	return err_code;
}

/**	----------------------------------------------------------------------
**
**	@fn		Function		i2c0_sync_read
**
**	@brief	Please read header in include file for function description
**
**	----------------------------------------------------------------------
*/
ret_code_t i2c0_sync_read(uint8_t devAddr, uint8_t *dataOut, uint8_t dataOutLength, uint8_t *dataIn, uint8_t dataInLength) 
{
	ret_code_t err_code = 0;	
	
#if USE_NON_BLOCKING
	nrf_drv_twi_xfer_desc_t xfer = NRF_DRV_TWI_XFER_DESC_TXRX(devAddr, dataOut, dataOutLength, dataIn, dataInLength);
	twi0XferCompleted = false;
	ret_code_t ret = nrf_drv_twi_xfer(&m_twi0, &xfer, 0);
	
	if (ret == NRF_SUCCESS) 
	{
		while(twi0XferCompleted == false){}
	}
#else
	err_code = nrf_drv_twi_tx(&m_twi0, devAddr, dataOut, dataOutLength, true);
	
	if (NRF_SUCCESS == err_code) 
	{
	    err_code = nrf_drv_twi_rx(&m_twi0, devAddr, dataIn, dataInLength);
	}
#endif
	
	return err_code;
}

/**	----------------------------------------------------------------------
**
**	@fn		Function		i2c1_sync_write
**
**	@brief	Please read header in include file for function description
**
**	----------------------------------------------------------------------
*/
ret_code_t i2c1_sync_write(uint8_t devAddr, uint8_t *data, uint8_t length) 
{
	ret_code_t err_code = 0;
	
	err_code = nrf_drv_twi_tx(&m_twi1, devAddr, data, length, false);
	return err_code;
}

/**	----------------------------------------------------------------------
**
**	@fn		Function		i2c1_sync_read
**
**	@brief	Please read header in include file for function description
**
**	----------------------------------------------------------------------
*/
ret_code_t i2c1_sync_read(uint8_t devAddr, uint8_t *dataOut, uint8_t dataOutLength, uint8_t *dataIn, uint8_t dataInLength) 
{
	ret_code_t err_code = 0;	
	
#if USE_NON_BLOCKING
	nrf_drv_twi_xfer_desc_t xfer = NRF_DRV_TWI_XFER_DESC_TXRX(devAddr, dataOut, dataOutLength, dataIn, dataInLength);
	twi1XferCompleted = false;
	ret_code_t ret = nrf_drv_twi_xfer(&m_twi1, &xfer, 0);
	
	if (ret == NRF_SUCCESS) 
	{
		while(twi1XferCompleted == false){}
	}
#else
	err_code = nrf_drv_twi_tx(&m_twi1, devAddr, dataOut, dataOutLength, true);
	
	if (NRF_SUCCESS == err_code) 
	{
	    err_code = nrf_drv_twi_rx(&m_twi1, devAddr, dataIn, dataInLength);
	}
#endif
	
	return err_code;
}

// Misc
#if 0	
#if USE_NON_BLOCKING
	err_code = nrf_drv_twi_init(&m_twi0, &twi0_config, twi_event_handler_twi0, NULL);
#else
	err_code = nrf_drv_twi_init(&m_twi0, &twi0_config, NULL, NULL);
#endif
#endif	
