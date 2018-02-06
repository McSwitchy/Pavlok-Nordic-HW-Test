/**	----------------------------------------------------------------------
**
**	@file		pwm.h
**
**  @details This module implements the pwm on the nRF52 for the PAVLOK
**  					product
**
**  @note The application must
**
**  @note Attention!
**
**
**	----------------------------------------------------------------------
*/
#ifndef PWM_H
#define PWM_H

/**	----------------------------------------------------------------------
**	@brief System Include(s)
**	----------------------------------------------------------------------
*/
#include <stdio.h>
#include <string.h>
#include "nrf_drv_pwm.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "boards.h"
#include "bsp.h"
#include "nrf_drv_clock.h"

/**	----------------------------------------------------------------------
**	@brief Project Include(s)
**	----------------------------------------------------------------------
*/
/**	----------------------------------------------------------------------
**	@brief	Global Data
**	----------------------------------------------------------------------
*/
/**	----------------------------------------------------------------------
**	@def	PWM Polarity
**	----------------------------------------------------------------------
*/
#define PWM_POLARITY_HIGH(x)				(uint16_t)((uint16_t)(1 << 15) | x)
#define PWM_POLARITY_LOW(x) 	      (uint16_t)x

// Vibration motor instance is driven through a clock frequency of 125 KHz (Tpwmclock: 8 us). According to PWM section 45.1 of
// NRF52 product spec, Tpwm = Tpwmclock * COUNTERTOP. Vibration Motor will be operated at 1000 Hz (Tpwm: 1 ms).
// Thus, COUNTERTOP value is 125.

#define PWM_DEBUG											(0)

#if PWM_DEBUG
#define PWM_MOTOR_PIN									(26)
#else
#define PWM_MOTOR_PIN									(24)
#endif
#define MOTOR_FREQUENCY_MIN           (100)
#define MOTOR_FREQUENCY_MAX           (10000)
#define MOTOR_PWM_MIN                 (10)
#define MOTOR_PWM_MAX                 (100)
#define PWM_MOTOR_COUNTERTOP					(1000)
#define PWM_MOTOR_SCALE_FACTOR	      (PWM_MOTOR_COUNTERTOP/100)
#define PWM_MOTOR_USE_ACTIVE_HIGH			(1)

// PWM frequency for Piezo ranges between 2 KHz (500 us) to 6 KHz (167 us). This instance is driven through a clock frequency
// of 1 MHz (1 us). Thus, the COUNTERTOP value will respectively range between 500 and 167.

#if PWM_DEBUG
#define PWM_PIEZO_PIN									(27)
#else
#define PWM_PIEZO_PIN									(14)
#endif
#define PIEZO_FREQUENCY_MIN						(100) // Hz 2000
#define PIEZO_FREQUENCY_MAX						(10000) // Hz
#define PWM_PIEZO_USE_ACTIVE_HIGH			(1)

// TODO remove #define PIEZO_UPDATE_FREQUENCY      	(1)
#define PWM_PIEZO_COUNTERTOP       		(1000)
#define PIEZO_OUTPUT_OFF      				(0)
#define PIEZO_PWM_MIN                 (10)
#define PIEZO_PWM_MAX                 (100)

#define PWM_ZAP_PIN										(7)
#define PWM_ZAP_COUNTERTOP						(91)
#define PWM_ZAP_SCALE_FACTOR(x)				(uint16_t)((x*PWM_ZAP_COUNTERTOP)/100)
#define PWM_ZAP_USE_ACTIVE_HIGH				(1)
#define ZAP_PWM_MIN                   (10)
#define ZAP_PWM_MAX                   (100)

/**	----------------------------------------------------------------------
**	@typedef PWM_PERCENT_T
**	----------------------------------------------------------------------
*/
typedef uint16_t PWM_PERCENT_T;

/**	----------------------------------------------------------------------
**	@enum	PWM_UPDATE_RC
**	----------------------------------------------------------------------
*/
typedef enum
{
	UPDATE_SUCCESS = 0,
	INVALID_DUTY_CYCLE,
	INVALID_FREQUENCY
} PWM_UPDATE_RC;

/**	----------------------------------------------------------------------
**	@brief	Global Extern(s)
**	----------------------------------------------------------------------
*/

/**	----------------------------------------------------------------------
**
**	@fn		Function	pwm_piezo_init
**
**	@brief	Description Initializes the PWM peripheral for the Piezo
**						Annunciator
**
**	@param [in]			None
**
**	@param	[out]		None
**
**	@return				None
**
**	@warn				None
**
**	----------------------------------------------------------------------
*/
void pwm_piezo_init(void);

/**	----------------------------------------------------------------------
**
**	@fn		Function	pwm_motor_init
**
**	@brief	Description Initializes the PWM peripheral for the Vibration
**						Motor
**
**	@param [in]			None
**
**	@param	[out]		None
**
**	@return				None
**
**	@warn				None
**
**	----------------------------------------------------------------------
*/
void pwm_motor_init(void);

/**	----------------------------------------------------------------------
**
**	@fn		Function	pwm_zap_init
**
**	@brief	Description Initializes the PWM peripheral for the Zap function
**
**
**	@param [in]			None
**
**	@param	[out]		None
**
**	@return				None
**
**	@warn				None
**
**	----------------------------------------------------------------------
*/
void pwm_zap_init(void);

/**	----------------------------------------------------------------------
**
**	@fn		Function	pwm_piezo_update_duty_and_frequency
**
**	@brief	Description realtime change for duty cycle and frequency
**
**	@param [in]			PWM_PERCENT_T duty - this has limits
**	@param [in]			uint16_t frequency - this has limits
**
**	@param	[out]		None
**
**	@return				PWM_UPDATE_RC
**
**	@retval				UPDATE_SUCCESS = 0,
**	@retval				INVALID_DUTY_CYCLE,
**
**	@warn				more test needed for testing limits
**
**	----------------------------------------------------------------------
*/
PWM_UPDATE_RC pwm_piezo_update_duty_and_frequency(PWM_PERCENT_T duty, uint16_t frequency);

/**	----------------------------------------------------------------------
**
**	@fn		Function	pwm_motor_update_duty
**
**	@brief	Description realtime change for duty cycle
**
**	@param [in]			PWM_PERCENT_T duty - this has limits
**
**	@param	[out]		None
**
**	@return				PWM_UPDATE_RC
**
**	@retval				UPDATE_SUCCESS = 0,
**	@retval				INVALID_DUTY_CYCLE,
**
**	@warn				Not all duty cycles produce viable change
**
**	----------------------------------------------------------------------
*/
PWM_UPDATE_RC pwm_motor_update_duty(PWM_PERCENT_T duty);

/**	----------------------------------------------------------------------
**
**	@fn		Function	pwm_motor_update_duty_and_frequency
**
**	@brief	Description realtime change for duty cycle and frequency
**
**	@param [in]			PWM_PERCENT_T duty - this has limits
**
**	@param	[out]		None
**
**	@return				PWM_UPDATE_RC
**
**	@retval				UPDATE_SUCCESS = 0,
**	@retval				INVALID_DUTY_CYCLE,
**
**	@warn				Not all duty cycles produce viable change
**
**	----------------------------------------------------------------------
*/
PWM_UPDATE_RC pwm_motor_update_duty_and_frequency(PWM_PERCENT_T duty, uint16_t frequency);


void pwm_zap_start(void);
void pwm_zap_stop(void);


#endif /* ${PWM_H} */
/** @} */
