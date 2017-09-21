/*------------------------------------------------------------------------
**
**	@file				zap.c
**
**  @defgroup 	PVLOK_ZAP
**  @{
**  @ingroup 
**  @brief 			zap peripheral code set
**  
**  @details		This module implements and calls the necessary io to 
**							enable the zapper
**  
**  @note 			
**  
**  @note 			you must init the function for it to work!
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
#include "pwm.h"
#include "zap.h"

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
static bool zapper_enabled = false;
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
**	@fn		Function				zap_gpio_init
**
**	@brief	Description		setup the zapper gpio
**
**  @note   See include file for further infor on this function 
**
**	@warn		
**
**  ----------------------------------------------------------------------
*/
void zap_gpio_init(void) 
{
	nrf_gpio_cfg_output(ZAP_RELEASE_PIN);
	nrf_gpio_pin_clear(ZAP_RELEASE_PIN);
}

/** ----------------------------------------------------------------------
**
**	@fn		Function				zap_gpio_enable
**
**	@brief	Description		turn on zapper enable pin
**
**  @note   See include file for further infor on this function 
**
**	@warn		This is the one that zaps you
**
**  ----------------------------------------------------------------------
*/
void zap_gpio_enable(void)
{
  
	pwm_zap_update_duty(0);
	nrf_gpio_pin_set(ZAP_RELEASE_PIN);
  zapper_enabled = false;
}

/** ----------------------------------------------------------------------
**
**	@fn		Function				zap_gpio_disable
**
**	@brief	Description		turn off zapper enable pin
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
void zap_gpio_disable(void) 
{
	nrf_gpio_pin_clear(ZAP_RELEASE_PIN);
}

/** ----------------------------------------------------------------------
**
**	@fn		Function				charge_zapper
**
**	@brief	Description		turn on the zapper pwm charger
**
**  @note   See include file for further infor on this function 
**
**	@warn		DO NOT CHANGE PWM VALUE
**					this is the value needed for the zapper to work
**
**  ----------------------------------------------------------------------
*/
void charge_zapper(void) 
{
  zapper_enabled = true;
	pwm_zap_update_duty(50);
}

/** ----------------------------------------------------------------------
**
**	@fn		Function				vbatt_measure_init
**
**	@brief	Description		setup the gpio read enable pin
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
void vbatt_measure_init(void) 
{
	nrf_gpio_cfg_output(VBATT_MEASURE_ENABLE_PIN);
	nrf_gpio_pin_clear(VBATT_MEASURE_ENABLE_PIN);
}

/** ----------------------------------------------------------------------
**
**	@fn		Function				vbatt_measure_enable
**
**	@brief	Description		turn on battery read enable pin
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
void vbatt_measure_enable(void) 
{
	nrf_gpio_pin_set(VBATT_MEASURE_ENABLE_PIN);
}

/** ----------------------------------------------------------------------
**
**	@fn		Function				vbatt_measure_disable
**
**	@brief	Description		turn off battery read enable pin
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
void vbatt_measure_disable(void) 
{
	nrf_gpio_pin_clear(VBATT_MEASURE_ENABLE_PIN);
}


/** ----------------------------------------------------------------------
**
**	@fn		Function				getBatteryVoltage
**
**	@brief	Description		read the battery charge
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
millivoltsBattery_t getBatteryVoltage(void) 
{
	nrf_saadc_value_t 	sampleVal = 0;
	uint32_t 						mVChannel = 0;
	millivoltsBattery_t mV = 0;
	
  if(false == zapper_enabled)
  {
    vbatt_measure_enable();
	
    sampleVal 	= adc_sample_channel(BATTERY_VOLTAGE);
    mVChannel 	= (uint32_t)ADC_RESULT_IN_MILLI_VOLTS(sampleVal);
    mV 					= BATT_MILLIVOLTS_RESISTOR_RATIO(mVChannel);
	
    vbatt_measure_disable();
	}
  
	return mV;
}

/** ----------------------------------------------------------------------
**
**	@fn		Function				getZapVoltage
**
**	@brief	Description		read the zapper charge
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
voltsZap_t getZapVoltage(void) 
{
	nrf_saadc_value_t 	sampleVal = 0;
	uint32_t 						mVChannel = 0;
	voltsZap_t 					volts = 0;
	
	sampleVal 				= adc_sample_channel(HIGH_VOLTAGE_MONITOR);	
	mVChannel 				= ADC_RESULT_IN_MILLI_VOLTS(sampleVal);
	volts 						= ZAP_VOLTS_RESISTOR_RATIO(mVChannel);

	return volts;
}

/** @} */
