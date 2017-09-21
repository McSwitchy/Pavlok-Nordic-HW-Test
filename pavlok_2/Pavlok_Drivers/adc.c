/*------------------------------------------------------------------------
**
**	@file			adc.c
**
**  @defgroup PAVLOK_ADC
**  @{
**  @ingroup PAVLOK_ADC
**  
**  @details This module implements the pavlok analog functins
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
#include "adc.h"

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
static nrf_saadc_channel_config_t s_channel0Config;
static nrf_saadc_channel_config_t s_channel1Config;

/**	----------------------------------------------------------------------
**	@brief	Static Forward References
**	----------------------------------------------------------------------
*/
static nrf_saadc_value_t adc_find_average(nrf_saadc_value_t *, uint8_t);

/**	----------------------------------------------------------------------
**	@brief	Static Functions
**	----------------------------------------------------------------------
*/
/**	----------------------------------------------------------------------
**
**	@fn		Function			saadc_callback
**
**	@brief	Description	This is just a place holder function to support
**											the adc on the nrf52
**
**	@param [in]
**
**	@param	[out]
**
**	@return
**
**	@retval
**
**	@warn
**
**	----------------------------------------------------------------------
*/
static void saadc_callback(nrf_drv_saadc_evt_t const *p_event) 
{

}

/**	----------------------------------------------------------------------
**
**	@fn		Function				adc_find_average
**
**	@brief	Description		Average the adc sample for the caller
**
**	@param [in]						nrf_saadc_value_t * raw - value from th adc
**	@param [in]						uint8_t number - number of samples
**
**	@param	[out]					None
**
**	@return								nrf_saadc_value_t - a uint16_t in millivolts
**
**	@retval
**
**	@warn
**
**	----------------------------------------------------------------------
*/
static nrf_saadc_value_t adc_find_average(nrf_saadc_value_t *raw, uint8_t number) 
{

	uint8_t i = 0;
	nrf_saadc_value_t sum = 0;
	nrf_saadc_value_t avg = 0;

	for (i = 0; i < number; i++) 
	{
		sum += raw[i];
	}
	avg = (nrf_saadc_value_t)(sum/number);
	
	return avg;
}

/**	----------------------------------------------------------------------
**	@brief	Global Functions
**	----------------------------------------------------------------------
*/
/** ----------------------------------------------------------------------
**
**	@fn		Function				adc_init
**
**	@brief	Description		initialize the Pavlok ADC module
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
void adc_init(void) 
{
   ret_code_t err_code = 0;
  // Set configuration for saadc channel 0
	s_channel0Config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;
	s_channel0Config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;
	s_channel0Config.gain       = NRF_SAADC_GAIN1_6;
	s_channel0Config.reference  = NRF_SAADC_REFERENCE_INTERNAL;
	s_channel0Config.acq_time   = NRF_SAADC_ACQTIME_20US;
	s_channel0Config.mode       = NRF_SAADC_MODE_SINGLE_ENDED;
	s_channel0Config.pin_p      = (nrf_saadc_input_t)(ADC_CHANNEL_BATTERY_VOLTAGE);
	s_channel0Config.pin_n      = NRF_SAADC_INPUT_DISABLED;

	// Set configuration for saadc channel 1
	s_channel1Config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;
	s_channel1Config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;
	s_channel1Config.gain       = NRF_SAADC_GAIN1_6;
	s_channel1Config.reference  = NRF_SAADC_REFERENCE_INTERNAL;
	s_channel1Config.acq_time   = NRF_SAADC_ACQTIME_20US;
	s_channel1Config.mode       = NRF_SAADC_MODE_SINGLE_ENDED;
	s_channel1Config.pin_p      = (nrf_saadc_input_t)(ADC_CHANNEL_HIGH_VOLTAGE_MONITOR);
	s_channel1Config.pin_n      = NRF_SAADC_INPUT_DISABLED;

	err_code = nrf_drv_saadc_init(NULL, saadc_callback);
	APP_ERROR_CHECK( err_code );
	err_code = nrf_drv_saadc_channel_init(BATTERY_VOLTAGE, &s_channel0Config);
	APP_ERROR_CHECK( err_code );
	err_code = nrf_drv_saadc_channel_init(HIGH_VOLTAGE_MONITOR, &s_channel1Config);
	APP_ERROR_CHECK( err_code );
	
}

/** ----------------------------------------------------------------------
**
**	@fn		Function				adc_sample_channel
**
**	@brief	Description		get the next sample from the specified channel
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
nrf_saadc_value_t adc_sample_channel(ADC_CHANNEL_T channel) 
{
	ret_code_t 		ret = 0;
	uint8_t 			i 	= 0;
	
	nrf_saadc_value_t samples[SAMPLES_IN_BUFFER], average = 0;

	if (channel >= NUMBER_OF_SUPPORTED_ADC_CHANNELS) 
	{
		return 0;
	}
	
	(void)memset(samples, 0, SAMPLES_IN_BUFFER);

	for (i = 0; i < SAMPLES_IN_BUFFER; i++) 
	{
		ret = nrf_drv_saadc_sample_convert(channel, samples+i);
		if (ret != 0) 
		{
			return 0;
		}
	}
	average = adc_find_average(samples, SAMPLES_IN_BUFFER);

	return average;
}

/** @} */
