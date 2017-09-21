/**	----------------------------------------------------------------------
**
**	@file		led.h
**
**  @brief 		This file include the PAVLOK LED pin configuration
**  
**  @details This module includes the led init set and clear prototypes 
**  
**  @note  
**  
**  @note 
**   
**
**	----------------------------------------------------------------------
*/
#ifndef _LED_H_
#define _LED_H_

/**	----------------------------------------------------------------------
**	@brief System Include(s)
**	----------------------------------------------------------------------
*/
/**	----------------------------------------------------------------------
**	@brief Project Include(s)
**	----------------------------------------------------------------------
*/

/**	----------------------------------------------------------------------
**	@brief	Global Data
**	----------------------------------------------------------------------
*/

/**	----------------------------------------------------------------------
**	@def	led table xmacro
**	----------------------------------------------------------------------
*/
#define LED_TABLE \
X(LED1, 16) \
X(LED2, 17) \
X(LED3, 18)	\
X(LED4, 19) \
X(LED5, 20) \
X(LED6,  9) \
X(LED7, 10) 

#define LED_RED	(LED6)
#define LED_GREEN	(LED7)

/**	----------------------------------------------------------------------
**	@enum	LED_T - led table
**	----------------------------------------------------------------------
*/
#define X(a, b) a,
typedef enum {
	LED_TABLE
} LED_T;
#undef X
#define NUM_OF_LEDS (LED7+1)

/**	----------------------------------------------------------------------
**	@enum	LED_OP_T
**	----------------------------------------------------------------------
*/
typedef enum {
	LED_OP_UNINIITALIZED,
	LED_OP_SUCCESS,
	LED_OP_INVALID_PIN
} LED_OP_T;

/**	----------------------------------------------------------------------
**	@brief	Global Extern(s)
**	----------------------------------------------------------------------
*/

/**	----------------------------------------------------------------------
**	@brief	Static Data
**	----------------------------------------------------------------------
*/

/**	----------------------------------------------------------------------
**	@brief	Static Forward References
**	----------------------------------------------------------------------
*/

/**	----------------------------------------------------------------------
**
**	@fn		Function		init_leds
**
**	@brief	Description		initialize the leds for the pavlok
**
**	@param [in]				None
**
**	@param	[out]			None
**
**	@return					None
**
**	@warn					None
**
**	----------------------------------------------------------------------
*/
void init_leds(void);

/**	----------------------------------------------------------------------
**
**	@fn		Function		set_led
**
**	@brief	Description		
**
**	@param [in]				LED_T led - which led to set
**
**	@param	[out]			None
**
**	@return					LED_OP_T
**
**	@retval					LED_OP_UNINIITALIZED,
**	@retval					LED_OP_SUCCESS,
**	@retval					LED_OP_INVALID_PI
**
**	@warn					None
**
**	----------------------------------------------------------------------
*/
LED_OP_T set_led(LED_T led);

/**	----------------------------------------------------------------------
**
**	@fn		Function		clear_led
**
**	@brief	Description		
**
**	@param [in]				LED_T led - which led to clear
**
**	@param	[out]			None
**
**	@return					LED_OP_T
**
**	@retval					LED_OP_UNINIITALIZED,
**	@retval					LED_OP_SUCCESS,
**	@retval					LED_OP_INVALID_PI
**
**	@warn					None
**
**	----------------------------------------------------------------------
*/
LED_OP_T clear_led(LED_T led);

#endif /* _LED_H_ */
