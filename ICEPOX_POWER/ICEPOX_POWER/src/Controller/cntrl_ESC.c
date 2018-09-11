/*
 * cntrl_ESC.c
 *
 * Created: 9/10/2018 17:24:45
 *  Author: lphillips
*/


#include "cntrl_ESC.h"

struct tcc_module	esc_tcc_instance;

void esc_config(void) {
////// ESC_RELAY
	struct port_config pin;		// lmp check for application
	port_get_config_defaults(&pin);
	pin.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(ESC_RELAY_PIN, &pin);
	port_pin_set_output_level(ESC_RELAY_PIN, ESC_RELAY_INACTIVE);



////// ESC_PWM
	/*this is the structure holding the TCC configuration*/
	struct tcc_config config_tcc;

	/*get the defaults*/
	tcc_get_config_defaults(&config_tcc, ESC_PWM_MODULE);

	/*set relevant parameters*/
	config_tcc.counter.clock_source    = GCLK_GENERATOR_0; /*GCLK_1 is derived from  DPLL @ 48 Mhz */
	config_tcc.counter.clock_prescaler = TCC_CLOCK_PRESCALER_DIV1;  /*sets GCLK_1 to peripheral clock divider*/
	config_tcc.counter.period = 0xFFFF; /*set the count-up-to value, up to 24-bits*/
	config_tcc.compare.wave_generation = TCC_WAVE_GENERATION_SINGLE_SLOPE_PWM; /*sets the waveform mode*/

	config_tcc.compare.match[ESC_PWM_CHANNEL] = (0xFFFFFFFF);	/*this is modified upon call to xTask_PWM*/

	config_tcc.pins.enable_wave_out_pin[ESC_PWM_OUTPUT_SELECT] = true;
	config_tcc.pins.wave_out_pin[ESC_PWM_OUTPUT_SELECT]        = ESC_PWM_PIN;
	config_tcc.pins.wave_out_pin_mux[ESC_PWM_OUTPUT_SELECT]    = ESC_PWM_MUX;


	tcc_init(&esc_tcc_instance, ESC_PWM_MODULE, &config_tcc);

	tcc_set_compare_value(&esc_tcc_instance,ESC_PWM_CHANNEL, 0);
	tcc_enable(&esc_tcc_instance);


}


void esc_pwm_set_dutyCycle (void) {

}