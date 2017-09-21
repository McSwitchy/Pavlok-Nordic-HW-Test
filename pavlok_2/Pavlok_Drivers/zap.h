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
#ifndef ZAP_H
#define ZAP_H

/*------------------------------------------------------------------------
**	@brief System Include(s)
**------------------------------------------------------------------------
*/
#include <stdio.h>
#include <string.h>
#include "nrf_drv_pwm.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "boards.h"
#include "bsp.h"
#include "nrf_delay.h"

/**	----------------------------------------------------------------------
**	@brief Project Include(s)
**	----------------------------------------------------------------------
*/
#include "adc.h"
#include "zap.h"

/**	----------------------------------------------------------------------
**	@brief	Global Data
**	----------------------------------------------------------------------
*/

#define ZAP_RELEASE_PIN											(8)
#define VBATT_MEASURE_ENABLE_PIN						(8)

/**	----------------------------------------------------------------------
**	@brief	Global Extern(s)
**	----------------------------------------------------------------------
*/
typedef uint16_t millivoltsBattery_t;
typedef uint16_t voltsZap_t;

// With the voltage divider of 1500 ohms and 3500 ohms, the ratio is 1.5
#define BATT_MILLIVOLTS_RESISTOR_RATIO(x)		(millivoltsBattery_t)((x*3)/2)
// With the voltage divider of 68K ohms and 10M ohms, the ratio is ~148
#define ZAP_VOLTS_RESISTOR_RATIO(x)				(voltsZap_t)((x/1000.0)*148)

/** ----------------------------------------------------------------------
**  @brief  The max value on the HVMON is ~x200-x20a (512d)
**          So 10% == 51 20% == 102 100% = 510
**	----------------------------------------------------------------------
*/
/**	----------------------------------------------------------------------
**	@brief	Global Functions
**	----------------------------------------------------------------------
*/
/**	----------------------------------------------------------------------
**
**	@fn		Function			zap_gpio_init
**
**	@brief	Description	initialize the gpio to release the zap
**
**	@param [in]					None
**
**	@param	[out]				None
**
**	@return							None
**
**	@retval
**
**	@warn								None
**
**	----------------------------------------------------------------------
*/
void zap_gpio_init(void);

/**	----------------------------------------------------------------------
**
**	@fn		Function			zap_gpio_enable
**
**	@brief	Description set the gpio pins to enable the zapper
**
**	@param [in]					None
**
**	@param	[out]				None
**
**	@return							None
**
**	@retval
**
**	@warn								None
**
**	----------------------------------------------------------------------
*/
void zap_gpio_enable(void);

/**	----------------------------------------------------------------------
**
**	@fn		Function			zap_gpio_disable
**
**	@brief	Description clear the gpio pins to turn off zapper
**
**	@param [in]					None
**
**	@param	[out]				None
**
**	@return							None
**
**	@retval
**
**	@warn								None
**
**	----------------------------------------------------------------------
*/
void zap_gpio_disable(void);

/**	----------------------------------------------------------------------
**
**	@fn		Function			charge_zapper
**
**	@brief	Description start the zapper charging
**
**	@param [in]					None
**
**	@param	[out]				None
**
**	@return							None
**
**	@retval
**
**	@warn								None
**
**	----------------------------------------------------------------------
*/
void charge_zapper(void);

/**	----------------------------------------------------------------------
**
**	@fn		Function			vbatt_measure_init
**
**	@brief	Description set the gpio pins to measure the battery
**
**	@param [in]					None
**
**	@param	[out]				None
**
**	@return							None
**
**	@retval
**
**	@warn								None
**
**	----------------------------------------------------------------------
*/
void vbatt_measure_init(void);

/**	----------------------------------------------------------------------
**
**	@fn		Function			vbatt_measure_enable
**
**	@brief	Description turn on the battery read enable pin
**
**	@param [in]					None
**
**	@param	[out]				None
**
**	@return							None
**
**	@retval
**
**	@warn								None
**
**	----------------------------------------------------------------------
*/
void vbatt_measure_enable(void);

/**	----------------------------------------------------------------------
**
**	@fn		Function			vbatt_measure_disable
**
**	@brief	Description turn off the battery read enable pin
**
**	@param [in]					None
**
**	@param	[out]				None
**
**	@return							None
**
**	@retval
**
**	@warn								None
**
**	----------------------------------------------------------------------
*/
void vbatt_measure_disable(void);

/**	----------------------------------------------------------------------
**
**	@fn		Function			getBatteryVoltage
**
**	@brief	Description	read the read the battery voltage
**
**	@param [in]					None
**
**	@param	[out]				None
**
**	@return							millivoltsBattery_t
**
**	@retval							millivolts
**
**	@warn								None
**
**	----------------------------------------------------------------------
*/
millivoltsBattery_t getBatteryVoltage(void);

/**	----------------------------------------------------------------------
**
**	@fn		Function			getZapVoltage
**
**	@brief	Description	read the zap charger voltage
**
**	@param [in]					None
**
**	@param	[out]				None
**
**	@return							voltsZap_t
**
**	@retval							millivolts
**
**	@warn								None
**
**	----------------------------------------------------------------------
*/
voltsZap_t getZapVoltage(void);

#endif
