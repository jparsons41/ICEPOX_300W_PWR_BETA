/*
 * cntrl_ESC.c
 *
 * Created: 9/10/2018 17:24:45
 *  Author: lphillips
 */


 #include "cntrl_ESC.h"



 void esc_config(void) {

 // ESC_RELAY
	 struct port_config pin;		// lmp check for application
	 port_get_config_defaults(&pin);
	 pin.direction = PORT_PIN_DIR_OUTPUT;
	 port_pin_set_config(PIN_PB13, &pin);
	 port_pin_set_output_level(PIN_PB13, LED1_INACTIVE);
 }