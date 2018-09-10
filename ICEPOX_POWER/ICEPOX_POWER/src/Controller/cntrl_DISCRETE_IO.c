/*
 * cntrl_DISCRETE_IO.c
 *
 * Created: 5/31/2017 11:52:53 AM
 *  Author: Gerald
 */ 

 #include <asf.h>

 #include "cntrl_DISCRETE_IO.h"

 void init_DisreteIO(void){

	struct port_config pin;
	uint8_t i;

	/*for simplicity, we'll use the ASF port IO driver*/

	/*Configures PORT for LED0 (green)*/
	port_get_config_defaults(&pin);
	pin.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(LED0_PIN, &pin);
	port_pin_set_output_level(LED0_PIN, LED0_INACTIVE);

	/*Configures PORT for LED1 (red)*/
	port_get_config_defaults(&pin);
	pin.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(LED1_PIN, &pin);
	port_pin_set_output_level(LED1_PIN, LED1_INACTIVE);

	/*Configures PORT for Motor DIR (OUTPUT), PB23*/
	port_get_config_defaults(&pin);
	pin.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(MOTOR_DIR, &pin);
	port_pin_set_output_level(MOTOR_DIR, MOTOR_DIR_INACTIVE);

	/*Configures PORT for Output EN (OUTPUT), PA27*/
	port_get_config_defaults(&pin);
	pin.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(OUTPUT_EN, &pin);
	port_pin_set_output_level(OUTPUT_EN, OUTPUT_EN_INACTIVE);

	/*Configures PORT for Battery EN (OUTPUT), PB30 */
	port_get_config_defaults(&pin);
	pin.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(BATTERY_EN, &pin);
	port_pin_set_output_level(BATTERY_EN, BATTERY_EN_INACTIVE);	

	/*Configures PORT for Battery RUN/CHARGE (OUTPUT), PB31 */
	port_get_config_defaults(&pin);
	pin.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(BATTERY_CHARGE_EN, &pin);
	port_pin_set_output_level(BATTERY_CHARGE_EN, BATTERY_CHARGE_EN_INACTIVE);

	/*Configures PORT for MCU Switch Status (INPUT), PB00 */
	port_get_config_defaults(&pin);
	pin.direction = PORT_PIN_DIR_INPUT;
	port_pin_set_config(MCU_SWITCH_STATUS, &pin);

	/*just a little light show on boot*/
	for (int i = 0; i < 20; i++) {
		port_pin_toggle_output_level(LED0_PIN);
		delay_ms(40);
		port_pin_toggle_output_level(LED1_PIN);
		port_pin_toggle_output_level(LED0_PIN);
		delay_ms(40);
		port_pin_toggle_output_level(LED1_PIN);
	}
	port_pin_set_output_level(LED0_PIN, LED0_INACTIVE);
	port_pin_set_output_level(LED1_PIN, LED1_INACTIVE);

 } //init_DiscreteIO

