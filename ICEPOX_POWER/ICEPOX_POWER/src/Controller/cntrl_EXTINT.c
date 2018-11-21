/*
 * cntrl_EXTINT.c
 *
 * Created: 3/29/2017 5:00:17 PM
 *  Author: Gerald
 */ 

#include "cntrl_EXTINT.h"

/*-----------------------------------------------------------*/
/*Configure the external interrupt channel PA28 EXTINT[8] for COMPARATOR Undervoltage TRIP*/
void configure_extint_channel(void)
 {

	struct port_config uvTrip_pin_conf;
	port_get_config_defaults(&uvTrip_pin_conf);
	
	/* Set buttons as inputs */
	uvTrip_pin_conf.direction  = PORT_PIN_DIR_INPUT;
	uvTrip_pin_conf.input_pull = PORT_PIN_PULL_UP;
	port_pin_set_config(PIN_PA28, &uvTrip_pin_conf);
	
	 struct extint_chan_conf config_extint_chan;

	 /*as normal, get configuration defaults*/
	 extint_chan_get_config_defaults(&config_extint_chan);

	 /*set custom stuff*/
	 config_extint_chan.gpio_pin           = PIN_PA28A_EIC_EXTINT8;			/*Undervoltage TRIP, PA28 (pin 53)*/
	 config_extint_chan.gpio_pin_mux       = MUX_PA28A_EIC_EXTINT8;
	 config_extint_chan.gpio_pin_pull      = EXTINT_PULL_UP;				/*uses an internal pull up*/
	 config_extint_chan.detection_criteria = EXTINT_DETECT_RISING;			/*we want RISING EDGES*/
	 
	 /*register the configuration*/
	 extint_chan_set_config( EXTINT_CH8,  &config_extint_chan);						/*IMPORTANT, set CHANNEL to 8 corresponding to EXTINT[8]*/

 }//configure_extint_channel


/*-----------------------------------------------------------*/
/*Configures the external interrupt callback*/
 void configure_extint_callbacks(void)
 {

	 /*register callback for crank interrupt processing*/
	 extint_register_callback(undervoltage_int_callback, EXTINT_CH8, EXTINT_CALLBACK_TYPE_DETECT);

	 /*enables the callback*/
	 extint_chan_enable_callback(EXTINT_CH8, EXTINT_CALLBACK_TYPE_DETECT);

 }//configure_extint_callbacks


/*Output EN (OUTPUT), PA27*/
#define OUTPUT_EN					  PIN_PA27
#define OUTPUT_EN_ACTIVE              true
#define OUTPUT_EN_INACTIVE            !OUTPUT_EN_ACTIVE

/*-----------------------------------------------------------*/

/*External interrupt COMPARATOR Under-voltage trip callback (uses EXTINT[8], PA28, Pin 53)*/
 void undervoltage_int_callback(void) {
		gbl_DigInputs.uv_trip = 1;
		port_pin_set_output_level(OUTPUT_EN, OUTPUT_EN_INACTIVE);
}
