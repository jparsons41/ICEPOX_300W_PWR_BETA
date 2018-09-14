/*
 * cntrl_PWM.c
 *
 * Created: 3/15/2017 12:03:48 PM
 *  Author: Gerald
 */

 #include "cntrl_PWM.h"

 /* TTC module declaration */
 struct tcc_module tcc_instance;

/*-----------------------------------------------------------*/
void configure_tcc(void)
{

	/*this is the structure holding the TCC configuration*/
	struct tcc_config config_tcc;

	/*get the defaults*/
	tcc_get_config_defaults(&config_tcc, CONF_PWM_MODULE);

	/*set relevant parameters*/
	config_tcc.counter.clock_source    = GCLK_GENERATOR_1; /*GCLK_1 is derived from  DPLL @ 48 Mhz */
	config_tcc.counter.clock_prescaler = TCC_CLOCK_PRESCALER_DIV1024;  /*sets GCLK_1 to peripheral clock divider*/
	config_tcc.counter.period = USER_PWM_PERIOD; /*set the count-up-to value, up to 24-bits*/
	config_tcc.compare.wave_generation = TCC_WAVE_GENERATION_SINGLE_SLOPE_PWM; /*sets the waveform mode*/

	/* do some math here ...*/
	//GCLK_1 = 48 Mhz,  TCC_Prescaler = 1, Period = 2^12 = 4096
	//thus, PWM freq = 48E6/1/4096 = 11.718 Hz.
	/*this ratio sets the initial duty cycle*/
	config_tcc.compare.match[CONF_PWM_CHANNEL] = (0);	/*this is modified upon call to xTask_PWM*/

	config_tcc.pins.enable_wave_out_pin[CONF_PWM_OUTPUT] = true;

	/*IMPORTANT NOTE ABOUT PIN MUXING!!*
	// see http://www.atmel.com/Images/Atmel-42256-SAM-Timer-Counter-for-Control-Applications-TCC-Driver_ApplicationNote_AT07058.pdf
	// page 15/16.
	/*      PWM_CHANNEL = 0 OUTPUT only on WO[0] or WO [4]. */
    /*      PWM_CHANNEL = 1 OUTPUT only on WO[1] or WO [5]. */
    /*      PWM_CHANNEL = 2 OUTPUT only on WO[2] or WO [6]. */
	/*      PWM_CHANNEL = 3 OUTPUT only on WO[3] or WO [7]. */

	config_tcc.pins.wave_out_pin[CONF_PWM_OUTPUT]        = CONF_PWM_OUT_PIN;
	config_tcc.pins.wave_out_pin_mux[CONF_PWM_OUTPUT]    = CONF_PWM_OUT_MUX;

	/*initialize the module*/
	tcc_init(&tcc_instance, CONF_PWM_MODULE, &config_tcc);

	/*go ahead an enable tcc*/
	tcc_set_compare_value(&tcc_instance,CONF_PWM_CHANNEL, 0);
	tcc_enable(&tcc_instance);

	/* create the queue for PWM message, this has to be the size of the command typedef pwm_cmd_t */
	xPwmQHndle = xQueueCreate(PWM_RTOS_CMD_QUEUE_LEN , sizeof(pwm_cmd_t));

}//end configure_tcc


/**
 * \brief This function set the duty cycle for the PWM created using TCC0 above
 */
void pwm_set_duty(uint16_t duty){

	uint16_t period;

	/*scale the period value to 0..USER_PWM_PERIOD, based on duty of 0..100*/
	if (duty>=100) {
		period = USER_PWM_PERIOD; /*this is the max*/
	}else {
		period = duty * (uint16_t) (USER_PWM_PERIOD / 100);
	}//

	printf("!! pwm_set_duty - Duty %d, Period %d \r\n", duty, period);
	tcc_set_compare_value(&tcc_instance,CONF_PWM_CHANNEL, period);

}//end pwm_set_duty

/**cc
 * \brief This task, when activated, modulates a PWM signal
 */
void xTaskPwm(void *pvParameters)
{
	/*declare command which will be received from the queue */
	pwm_cmd_t xPwmCmd;

	/* this is the infinite task loop */
	for (;;) {

		/* set task delay time in ms - this is a handy function */
		//const TickType_t xDelay = pdMS_TO_TICKS( 100 );

	    /* get a pwm command from the queue */
		if (xQueueReceive(xPwmQHndle, &xPwmCmd, portMAX_DELAY)) {

			/* this is command interpreter, separated by channel */
			/* do this for channel 1 */
			if (xPwmCmd.uxChId == 1) {
			   /*if this channel is ON */
			   if (xPwmCmd.uxOffOn == 1) {
				     /* enable the channel, and set the initial duty cycle */
					 printf("!! xTaskPwm ON - uxDuty %d, on/off %d \r\n", xPwmCmd.uxDuty, xPwmCmd.uxOffOn);
					 pwm_set_duty(xPwmCmd.uxDuty);
			         //tcc_enable(&tcc_instance);
			   } else {
				     printf("!! xTaskPwm OFF - uxDuty %d, on/off %d \r\n", xPwmCmd.uxDuty, xPwmCmd.uxOffOn);
			         /* disable the channel, and zero the duty cycle */
			         //tcc_disable(&tcc_instance);
			         pwm_set_duty(0);
			   }//end if xPwmCmd.uxOffOn
			}//end if xPwmCmd.uxChId

		}//end if xQueueReceive

	}//end for

	/* for completeness, this should never happen */
	vTaskDelete( NULL );

}//end xTaskPwm


