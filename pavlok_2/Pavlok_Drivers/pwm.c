/**	----------------------------------------------------------------------
**
**	@file
**
**  @defgroup		PAVLOK_PWM
**  @{
**  @ingroup		PAVLOK_PWM
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
**	@brief Project Include(s)
**	----------------------------------------------------------------------
*/
#include "pwm.h"

/**	----------------------------------------------------------------------
**	@brief	Static Data
**	----------------------------------------------------------------------
*/
static nrf_drv_pwm_t g_PWMMotorInstance = NRF_DRV_PWM_INSTANCE(0);
static nrf_drv_pwm_t g_PWMPiezoInstance = NRF_DRV_PWM_INSTANCE(1);
static nrf_drv_pwm_t g_PWMZapInstance = NRF_DRV_PWM_INSTANCE(2);

static uint16_t g_motorSequenceValue = 0;
static uint16_t g_piezoSequenceValue = 0;
static uint16_t g_zapSequenceValue = 0;

static nrf_pwm_sequence_t const g_motorSequence = 
{
	.values.p_common = &g_motorSequenceValue,
	.length = NRF_PWM_VALUES_LENGTH(g_motorSequenceValue),
	.repeats = 0,
	.end_delay = 0
};

static nrf_pwm_sequence_t const g_piezoSequence = 
{
	.values.p_common = &g_piezoSequenceValue,
	.length = NRF_PWM_VALUES_LENGTH(g_piezoSequenceValue),
	.repeats = 0,
	.end_delay = 0
};

static nrf_pwm_sequence_t const g_zapSequence = 
{
	.values.p_common = &g_zapSequenceValue,
	.length = NRF_PWM_VALUES_LENGTH(g_zapSequenceValue),
	.repeats = 0,
	.end_delay = 0
};

static nrf_drv_pwm_config_t *piezoConfigPtr = NULL;
static nrf_drv_pwm_config_t *motorConfigPtr = NULL;
static uint16_t g_PWMPiezoCountertop = PWM_PIEZO_COUNTERTOP;
static uint16_t g_PWMMotorCountertop = PWM_MOTOR_COUNTERTOP;

/** ----------------------------------------------------------------------
**
**	@fn			Function				pwm_piezo_init
**
**	@brief	Description			initialize the piezo PWM
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
void pwm_piezo_init(void) 
{
	uint32_t errorCode = 0;
	static nrf_drv_pwm_config_t piezoConfig = 
	{
		// The pin name cab or or'd with the 'NRF_DRV_PWM_PIN_INVERTED' mask if the signal is active low
		.output_pins = 
		{
				PWM_PIEZO_PIN,       				// channel 0
				NRF_DRV_PWM_PIN_NOT_USED, 	// channel 1
				NRF_DRV_PWM_PIN_NOT_USED, 	// channel 2
				NRF_DRV_PWM_PIN_NOT_USED	  // channel 3 
		},
		.base_clock = NRF_PWM_CLK_1MHz,
		.count_mode = NRF_PWM_MODE_UP,
		.top_value  = PWM_PIEZO_COUNTERTOP,
		.load_mode  = NRF_PWM_LOAD_COMMON,
		.step_mode  = NRF_PWM_STEP_AUTO 
	};
	piezoConfigPtr = &piezoConfig;
	
	errorCode = nrf_drv_pwm_init(&g_PWMPiezoInstance, &piezoConfig, NULL);
	APP_ERROR_CHECK(errorCode);
}

/** ----------------------------------------------------------------------
**
**	@fn		Function				pwm_motor_init
**
**	@brief	Description		initialize the motor PWM
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
void pwm_motor_init(void) 
{
  uint32_t errorCode = 0;
  static nrf_drv_pwm_config_t motorConfig =
  {
		// The pin name can be or'd with the 'NRF_DRV_PWM_PIN_INVERTED' mask if the signal is active low	
		.output_pins =
		{
			PWM_MOTOR_PIN, 	        		// channel 0
			NRF_DRV_PWM_PIN_NOT_USED, 	// channel 1
			NRF_DRV_PWM_PIN_NOT_USED, 	// channel 2
			NRF_DRV_PWM_PIN_NOT_USED	  // channel 3 
		},
		// Tpwm = Tpwmclock * COUNTERTOP; Tpwmclock = 1/1000000Hz = 1 us;
		// To achieve Tpwm of 1/1000 Hz = 1ms, COUNTERTOP = 1000. 
		.base_clock = NRF_PWM_CLK_1MHz,
		.count_mode = NRF_PWM_MODE_UP,
		.top_value  = PWM_MOTOR_COUNTERTOP,
		.load_mode  = NRF_PWM_LOAD_COMMON,
		.step_mode  = NRF_PWM_STEP_AUTO
  };
  motorConfigPtr = &motorConfig;

	errorCode = nrf_drv_pwm_init(&g_PWMMotorInstance, &motorConfig, NULL);
  APP_ERROR_CHECK(errorCode);
}

/** ----------------------------------------------------------------------
**
**	@fn			Function				pwm_zap_init
**
**	@brief	Description			initialize the zapper PWM
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
void pwm_zap_init(void) 
{
  uint32_t errorCode = 0;
  static nrf_drv_pwm_config_t const zapConfig =
  {
		// The pin name can be or'd with the 'NRF_DRV_PWM_PIN_INVERTED' mask if the signal is active low	
		.output_pins =
		{
			PWM_ZAP_PIN, 	        		  // channel 0
			NRF_DRV_PWM_PIN_NOT_USED, 	// channel 1
			NRF_DRV_PWM_PIN_NOT_USED, 	// channel 2
			NRF_DRV_PWM_PIN_NOT_USED	  // channel 3 
		},
		// Tpwm = Tpwmclock * COUNTERTOP
		// Tpwmclock = 1/8,000,000 Hz = 125 ns;
		// To achieve Tpwm of 1/88,000 Hz ~ 11 us, COUNTERTOP ~ 91.
		// This will yield 87912 Hz... this is as close to 88000 that we can get
		.base_clock = NRF_PWM_CLK_8MHz,
		.count_mode = NRF_PWM_MODE_UP,
		.top_value  = PWM_ZAP_COUNTERTOP,
		.load_mode  = NRF_PWM_LOAD_COMMON,
		.step_mode  = NRF_PWM_STEP_AUTO
  };

  errorCode = nrf_drv_pwm_init(&g_PWMZapInstance, &zapConfig, NULL);
  APP_ERROR_CHECK(errorCode);
}

/** ----------------------------------------------------------------------
**
**	@fn		Function				pwm_pizeo_update_duty_and_frequency
**
**	@brief	Description		turn on/off piezo 
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
PWM_UPDATE_RC pwm_motor_update_duty(PWM_PERCENT_T duty) 
{
	uint16_t dutyScaled = 0;

	// Check to see if duty cycle is valid
	if (duty > 100) 
	{
		return INVALID_DUTY_CYCLE;
	}
	// Scale duty cycle with COUNTERTOP Value
	dutyScaled = (duty * PWM_MOTOR_SCALE_FACTOR);
	// Stop current sequence
	nrf_drv_pwm_stop(&g_PWMMotorInstance, 1);
	// Update sequence

#if PWM_MOTOR_USE_ACTIVE_HIGH
	g_motorSequenceValue = PWM_POLARITY_HIGH(dutyScaled);
#else
	g_motorSequenceValue = PWM_POLARITY_LOW(dutyScaled);
#endif
	// Start playback of sequence
	nrf_drv_pwm_simple_playback(&g_PWMMotorInstance, &g_motorSequence, 1, NRF_DRV_PWM_FLAG_LOOP);
  
	return UPDATE_SUCCESS;
}

/** ----------------------------------------------------------------------
**
**	@fn		Function				pwm_motor_update_duty_and_frequency
**
**	@brief	Description		turn on/off motor 
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
PWM_UPDATE_RC pwm_motor_update_duty_and_frequency(PWM_PERCENT_T duty, uint16_t frequency) 
{
	uint16_t dutyScaled = 0;
	uint16_t frequencyHHz = 0;
	uint16_t newCounterTop = 0;
	uint16_t periodUs = 0;

	// Check to see if duty cycle is valid
	if ( duty > 100 ) 
	{
		return INVALID_DUTY_CYCLE;
	}
	// Check to see if frequency is valid
	if ( frequency < MOTOR_FREQUENCY_MIN || frequency > MOTOR_FREQUENCY_MAX ) 
	{
		return INVALID_FREQUENCY;
	}
	// Frequency must be multiple of 100 (1 KHz, 1.1 KHz, 2 Khz, etc.)
	if (frequency % 100 != 0) 
	{
		return INVALID_FREQUENCY;
	}

	// In order to do integer divison, first convert frequency from Hz to HHz (hecto-hertz: factor of 100).
	// Then, take 10,000 and divide by this number, effectively leaving units of us.
	frequencyHHz = frequency/100;
	periodUs = 10000/frequencyHHz;
	// Since Tpwm = Tpwmclk * COUNTERTOP and Tpwmclk is 1 us (frequency of 1 Mhz):
	newCounterTop = periodUs;
	g_PWMMotorCountertop = newCounterTop;
	// Scale duty cycle to new top value
	dutyScaled = (duty/100.0) * newCounterTop;
	// Stop current playback
	nrf_drv_pwm_stop(&g_PWMMotorInstance, 1);
	// Un-initialize current instance
	nrf_drv_pwm_uninit(&g_PWMMotorInstance);
	// Edit exisiting COUNTERTOP value
	motorConfigPtr->top_value = newCounterTop,
	// Init instance (again)
	nrf_drv_pwm_init(&g_PWMMotorInstance, motorConfigPtr, NULL);
	// Update duty cycle
#if PWM_MOTOR_USE_ACTIVE_HIGH
	g_motorSequenceValue = PWM_POLARITY_HIGH(dutyScaled);
#else
	g_motorSequenceValue = PWM_POLARITY_LOW(dutyScaled);
#endif
	// Start playback
	nrf_drv_pwm_simple_playback(&g_PWMMotorInstance, &g_motorSequence, 1, NRF_DRV_PWM_FLAG_LOOP);

	return UPDATE_SUCCESS;
}

/** ----------------------------------------------------------------------
**
**	@fn		Function				pwm_pizeo_update_duty_and_frequency
**
**	@brief	Description		turn on/off piezo 
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
PWM_UPDATE_RC pwm_pizeo_update_duty_and_frequency(PWM_PERCENT_T duty, uint16_t frequency) 
{
	uint16_t dutyScaled = 0;
	uint16_t frequencyHHz = 0;
	uint16_t newCounterTop = 0;
	uint16_t periodUs = 0;

	// Check to see if duty cycle is valid
	if ( duty > 100 ) 
	{
		return INVALID_DUTY_CYCLE;
	}
	// Check to see if frequency is valid
	if ( frequency < PIEZO_FREQUENCY_MIN || frequency > PIEZO_FREQUENCY_MAX ) 
	{
		return INVALID_FREQUENCY;
	}
	// Frequency must be multiple of 100 (1 KHz, 1.1 KHz, 2 Khz, etc.)
	if (frequency % 100 != 0) 
	{
		return INVALID_FREQUENCY;
	}

	// In order to do integer divison, first convert frequency from Hz to HHz (hecto-hertz: factor of 100).
	// Then, take 10,000 and divide by this number, effectively leaving units of us.
	frequencyHHz = frequency/100;
	periodUs = 10000/frequencyHHz;
	// Since Tpwm = Tpwmclk * COUNTERTOP and Tpwmclk is 1 us (frequency of 1 Mhz):
	newCounterTop = periodUs;
	g_PWMPiezoCountertop = newCounterTop;
	// Scale duty cycle to new top value
	dutyScaled = (duty/100.0) * newCounterTop;
	// Stop current playback
	nrf_drv_pwm_stop(&g_PWMPiezoInstance, 1);
	// Un-initialize current instance
	nrf_drv_pwm_uninit( &g_PWMPiezoInstance );
	// Edit exisiting COUNTERTOP value
	piezoConfigPtr->top_value = newCounterTop,
	// Init instance (again)
	nrf_drv_pwm_init(&g_PWMPiezoInstance, piezoConfigPtr, NULL);
	// Update duty cycle
#if PWM_PIEZO_USE_ACTIVE_HIGH
	g_piezoSequenceValue = PWM_POLARITY_HIGH(dutyScaled);
#else
	g_piezoSequenceValue = PWM_POLARITY_LOW(dutyScaled);
#endif
	// Start playback
	nrf_drv_pwm_simple_playback(&g_PWMPiezoInstance, &g_piezoSequence, 1, NRF_DRV_PWM_FLAG_LOOP);

	return UPDATE_SUCCESS;
}

/** ----------------------------------------------------------------------
**
**	@fn		Function				pwm_zap_update_duty
**
**	@brief	Description		turn on the charger pwm 
**
**  @note   See include file for further infor on this function 
**
**	@warn
**
**  ----------------------------------------------------------------------
*/
PWM_UPDATE_RC pwm_zap_update_duty(PWM_PERCENT_T duty) 
{
	uint16_t dutyScaled = 0;

	// Check to see if duty cycle is valid
	if (duty > 100) 
	{
		return INVALID_DUTY_CYCLE;
	}
	if (duty % 10 != 0) 
	{
		return INVALID_DUTY_CYCLE;
	}
	// Scale duty cycle with COUNTERTOP Value
	dutyScaled = PWM_ZAP_SCALE_FACTOR(duty);
	// Stop current sequence
	nrf_drv_pwm_stop(&g_PWMZapInstance, 1);
	// Update sequence

#if PWM_MOTOR_USE_ACTIVE_HIGH
	g_zapSequenceValue = PWM_POLARITY_HIGH(dutyScaled);
#else
	g_zapSequenceValue = PWM_POLARITY_LOW(dutyScaled);
#endif
	// Start playback of sequence
	nrf_drv_pwm_simple_playback(&g_PWMZapInstance, &g_zapSequence, 1, NRF_DRV_PWM_FLAG_LOOP);
  
	return UPDATE_SUCCESS;
}

/** @} */
