/*------------------------------------------------------------------------
**
**	@file									adc.h
**
**  @brief Description		data necessary to implement the adc for pavlok
**  
**  @note 
**
**------------------------------------------------------------------------
*/
#ifndef __ADC_H__
#define __ADC_H__

/*------------------------------------------------------------------------
**	@brief System Include(s)
**------------------------------------------------------------------------
*/
#include "nrf_drv_saadc.h"
#include "boards.h"

/**	----------------------------------------------------------------------
**	@brief Project Include(s)
**	----------------------------------------------------------------------
*/
/**	----------------------------------------------------------------------
**	@brief	Global Data
**	----------------------------------------------------------------------
*/
#define ADC_CHANNEL_BATTERY_VOLTAGE 				(NRF_SAADC_INPUT_AIN6) // P0.30
#define ADC_CHANNEL_HIGH_VOLTAGE_MONITOR		(NRF_SAADC_INPUT_AIN5) // P0.29

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS 			(600)
#define ADC_PRE_SCALING_COMPENSATION  			(6)
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE) \
				((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / 1024.0) * ADC_PRE_SCALING_COMPENSATION)

#define SAMPLES_IN_BUFFER 									(10)
#if((SAMPLES_IN_BUFFER%2)!=0)
#error Samples in buffer must be even!
#endif

/**	----------------------------------------------------------------------
**	@typedef	adc channels for pavlok
**	----------------------------------------------------------------------
*/
typedef enum {
	BATTERY_VOLTAGE = 0,
	HIGH_VOLTAGE_MONITOR,
	NUMBER_OF_SUPPORTED_ADC_CHANNELS
} ADC_CHANNEL_T;

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
**	@fn		Function				adc_init
**
**	@brief	Description		initialize the adc channels for pavlok
**
**	@param [in]						None
**
**	@param	[out]					None
**
**	@return								None
**
**	@retval
**
**	@warn
**
**	----------------------------------------------------------------------
*/
void adc_init(void);

/**	----------------------------------------------------------------------
**
**	@fn		Function				adc_sample_channel
**
**	@brief	Description		sets the adc channel and gathers the adc sample
**
**	@param [in]						ADC_CHANNEL_T
**
**	@param	[out]					None
**
**	@return								nrf_saadc_value_t - uint16_t averaged value
**
**	@retval
**
**	@warn
**
**	----------------------------------------------------------------------
*/
nrf_saadc_value_t adc_sample_channel(ADC_CHANNEL_T);

#endif

