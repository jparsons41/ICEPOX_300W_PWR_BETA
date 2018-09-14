/*
 * conf_application.h
 *
 * Created: 5/7/2017 10:31:12 PM
 *  Author: Gerald
 */

#ifndef CONF_APPLICATION_H_
#define CONF_APPLICATION_H_



#define BLDC_CONTROLLER_SET_CONFIGS		1




#define STRING_EOL    "\r"
#define STRING_HEADER "\r\n\r\n-- ICEPOX II POWER BOARD RTOS DEVELOPMENT\r\n" \
"-- "BOARD_NAME" \r\n" \
"-- Compiled: "__DATE__" "__TIME__" \r\n" \
"-------------------------------------------------"STRING_EOL

#define HEARTBEAT_TASK_PRIORITY         ( 1 )
#define HEARTBEAT_TASK_STACK_SIZE       ( configMINIMAL_STACK_SIZE * 2)

#define CLI_TASK_PRIORITY			    ( 2 )
#define CLI_TASK_STACK_SIZE			    ( configMINIMAL_STACK_SIZE * 3)

#define CAN_RX_TASK_PRIORITY		    ( 3 )
#define CAN_RX_TASK_STACK_SIZE		    ( configMINIMAL_STACK_SIZE * 3)

#define CAN_TX_TASK_PRIORITY		    ( 1 )
#define CAN_TX_TASK_STACK_SIZE		    ( configMINIMAL_STACK_SIZE * 3)

#define CAN_ERR_HANDLER_TASK_PRIORITY	( 3 )
#define CAN_ERR_HANDLER_TASK_STACK_SIZE ( configMINIMAL_STACK_SIZE * 2)

#define PWM_TASK_PRIORITY			    ( 2 )
#define PWM_TASK_STACK_SIZE		        ( configMINIMAL_STACK_SIZE * 2)

#define I2C_DAC_TASK_PRIORITY		    ( 2 )
#define I2C_DAC_TASK_STACK_SIZE	        ( configMINIMAL_STACK_SIZE * 2)

#define CONTROL_LOOP_TASK_PRIORITY		( 2 )
#define CONTROL_LOOP_TASK_STACK_SIZE    ( configMINIMAL_STACK_SIZE * 3)

#define MOTOR_TASK_PRIORITY				( 2 )
#define MOTOR_TASK_STACK_SIZE			( configMINIMAL_STACK_SIZE * 3)

/* Circuit breakers for task enable/disable (DEBUGGING) */
/* These are FREERTOS TASKS -->*/
#define TASK_HEARTBEAT_ENABLED				( 0 )	   /*heartbeat task (time-cyclic stuff can go here ...)*/
#define TASK_CLI_ENABLED					( 1 )      /*serial command line interface (CLI)*/
#define TASK_MCAN_ENABLED					( 1 )	   /*CAN message processing TX/RX*/
#define TASK_PWM_ENABLED					( 1 )	   /*PWM (using TCC peripheral) to driver motor ENABLE signal */
#define TASK_I2C_DAC_ENABLED				( 1 )	   /*DAC for setting voltage/current levels on three channels */
#define TASK_CONTROL_LOOP_ENABLED			( 1 )	   /* Main Process Control Loop */
#define MOTOR_CONTROLLER_ENABLED			( 1 )

/* These are PROCESSES (not Dependant on FreeRTOS) -->*/
#define PROCESS_ADC_ENABLED					( 1 )		/*ADC (SAR) analog data acquisition (high speed MAP sensor)*/
#define PROCESS_UNDERVOLTAGE_TRIP_ENABLED   ( 0 )		/*Under voltage Trip Comparator, PA28 EXTINT[8]*/

/* The LED to toggle and the rate at which the LED will be toggled. */
#define LED_TOGGLE_PERIOD		         pdMS_TO_TICKS( 1000)

#endif /* CONF_APPLICATION_H_ */