/**	----------------------------------------------------------------------
**
**	@file		led.c
**
**  @defgroup	PAVLOK_LED
**  @{
**  @ingroup	PAVLOK_LED
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
#include "nrf_gpio.h"

/**	----------------------------------------------------------------------
**	@brief Project Include(s)
**	----------------------------------------------------------------------
*/
#include "leds.h"

/**	----------------------------------------------------------------------
**	@brief	Global Data
**	----------------------------------------------------------------------
*/
/**	----------------------------------------------------------------------
**	@def	xmacro pin table
**	----------------------------------------------------------------------
*/
#define X(a, b) b,
uint8_t ledPinLookup[] = {
	LED_TABLE	
};
#undef X

/**	----------------------------------------------------------------------
**	@brief	Static Data
**	----------------------------------------------------------------------
*/
static uint8_t ledsInitialized = 0;

/**	----------------------------------------------------------------------
**
**	@fn		Function		init_leds
**
**	@brief	Please read header in include file for function description
**
**	----------------------------------------------------------------------
*/
void init_leds(void) {
	LED_T i = LED1;
	for (i = LED1; i<NUM_OF_LEDS; i++) {
		nrf_gpio_cfg_output(ledPinLookup[i]);
		nrf_gpio_pin_clear(ledPinLookup[i]);
	}
	ledsInitialized = 1;
}

/**	----------------------------------------------------------------------
**
**	@fn		Function		set_led
**
**	@brief	Please read header in include file for function description
**
**	----------------------------------------------------------------------
*/
LED_OP_T set_led(LED_T led) {
	if (ledsInitialized == 0) {
		return LED_OP_UNINIITALIZED;
	}
	if (led > NUM_OF_LEDS) {
		return LED_OP_INVALID_PIN;
	}
	nrf_gpio_pin_set(ledPinLookup[led]);
	
	return LED_OP_SUCCESS;
}

/**	----------------------------------------------------------------------
**
**	@fn		Function		clear_led
**
**	@brief	Please read header in include file for function description
**
**	----------------------------------------------------------------------
*/
LED_OP_T clear_led(LED_T led) {
	if (led > NUM_OF_LEDS) {
		return LED_OP_INVALID_PIN;
	}
	nrf_gpio_pin_clear(ledPinLookup[led]);
	
	return LED_OP_SUCCESS;
}
/** @} */
