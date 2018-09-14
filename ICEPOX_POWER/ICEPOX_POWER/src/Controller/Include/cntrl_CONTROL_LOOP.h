/*
 * cntrl_CONTROL_LOOP.h
 *
 * Created: 6/6/2017 4:24:34 PM
 *  Author: Gerald
 */


#ifndef CNTRL_CONTROL_LOOP_H_
#define CNTRL_CONTROL_LOOP_H_

/* Standard includes */
#include <asf.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <math.h>

#include "status_codes.h"
#include "compiler.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"

 /* User Application Includes */
 #include "cntrl_GLOBALS.h"
 #include "cntrl_DISCRETE_IO.h"
 #include "cntrl_MCAN.h"
 #include "cntrl_ADC.h"
 #include "cntrl_PWM.h"
 #include "cntrl_I2C_DAC.h"
 #include "cntrl_EXTINT.h"
 #include "cntrl_BLDC_CONTROLLER.h"

 #define CONTROL_LOOP_TASK_MS			25

/*init DIO output state variables*/
/*these are STATIC, i.e. retain their value */
uint8_t	 static OUT_motor_dir			= MOTOR_DIR_INACTIVE;				/*Motor DIR (OUTPUT),  PB23*/
uint8_t	 static OUT_output_en			= OUTPUT_EN_INACTIVE;				/*Output EN (OUTPUT),  PA27*/
uint8_t	 static OUT_battery_en			= BATTERY_EN_INACTIVE;				/*Battery EN (OUTPUT), PB30 */
uint8_t	 static OUT_charge_en  			= BATTERY_CHARGE_EN_INACTIVE;		/*Battery RUN/CHARGE (OUTPUT), PB31 */

//battery status state variables
int16_t  static	IBattery_mA;					/*battery current, mA, negative means current out of battery*/
int16_t  static IBattery_offset_mA;				/*battery offset used to tare reading*/
uint16_t static	VBattery_mV	;					/*battery voltage, mV*/
uint8_t	 static	bat_chrg_status;			/*battery charge status 1-charging, 0-off*/
uint16_t static	AltUnregVmon_mV;					/*buss voltage, mV*/
uint8_t	 static	state_of_chrg;			/*state of charge*/

//power status state variables
uint16_t static ILoadMeas_mA;					/*load current, mA*/
uint16_t static ILoadMeas_offset_mA;			/*load current offset used to tare, mA*/
uint16_t static	DCDCVmon_mV;					/*load voltage, mV*/
uint16_t static	DCDCImon_mA;					/*dc/dc current, mA*/
uint16_t static	IShortMeas_mA;					/*load current, mA, measured by PROFET for high current */

//thermistor status state variables
int16_t  static Thermistor_C;					/*thermistor in deg C*/

//power status flags
uint8_t	static	load_state;						/*Bit 0 - 1-on, 0-off*/
uint8_t	static	load_tripped_state;				/*Bit 1 - 1-tripped, 0-not tripped*/
uint8_t	static	sw_status_state;				/*Bit 2 - 1-on, 0-off*/
uint8_t	static	auto_man_state;					/*Bit 3 - 1 - AUTO, 0 - MANUAL */

//dac commands
uint8_t	static	Bat_I_Set;						/*Command for the Bat_I_Set channel of the DAC*/
uint8_t	static	Bat_I_Set_last;				    /*remembers last value (avoid sending command multiple times)*/
uint16_t	static	ISet_DAC_mA;					/*Command for the ISet_DAC_mA channel of the DAC in mA of output*/
uint16_t	static	ISet_DAC_mA_last;				/*remembers last value (avoid sending command multiple times)*/
uint16_t	static	VSet_DAC_mV;				/*Command for the VSet_DAC channel of the DAC, in mV from 20000 to 29000*/
uint16_t	static	VSet_DAC_mV_last;			/*remembers last value (avoid sending command multiple times)*/

//VSET DAC
#define VSET_DAC_CAN_TO_MV(x)					( (uint16_t) ((double) x * 35.29411) + 20000)

//ISET DAC
#define ISET_DAC_CAN_TO_MA(x)					( (uint16_t) ( x * 100) )


//other misc memory variables
uint8_t static	motor_cmd_last;					/*remembers last motor command value, avoid resetting PWM each cycle*/
uint16_t static monitor_counter;
uint8_t static	first_loop;						// flag to know if this is the first executed control loop

uint16_t static charge_control_state;
//struct for the PWM command (setup to handle multiple channels, but right now we only have one)
static pwm_cmd_t xPwmCmd = {
	.uxChId = 0,
	.uxDuty = 0,
	.uxOffOn = 0
};

//struct for the DAC commands (using changes A, B, and C)
static I2CDac_cmd_t xI2CDacCmd = {
	.uxChId   = 0,
	.uxCounts = 0
};

/*scales 8-bit PWM command from controller (0..255) to duty cycle (0..100%)*/
#define PWM_CMD_TO_DUTY(x)				( (uint16_t) ( (double) x * (100.0 / 255.0) ) )  /*output is 0..100 % to be passed to PWM module*/

#define BATT_OPEN_COUNTER_MAX				 10
#define BATT_OVERVOLT_COUNTER_MAX			 10				//
#define BATT_UNDERVOLT_COUNTER_MAX			 10			//
#define BATT_OVERCURRENT_COUNTER_MAX		 10			//
#define ALT_UNDERVOLT_COUNTER_MAX			 10			//
#define ALT_OVERVOLT_COUNTER_MAX			 10			//
#define LOAD_OVERCURRENT_COUNTER_MAX	     10				//
#define TWENTYEIGHT_UNDERVOLT_COUNTER_MAX    10
#define TWENTYEIGHT_OVERCURRENT_COUNTER_MAX	 10
#define BACKFEED_COUNTER_MAX		         10          //
#define IBATT_COUNTER_MAX			         10          //

#define CTRL_CAN_TIMEOUT					1000		// time in mS with no CAN message from controller before declaring comms invalid



//forward decelerations
void xInitCntrlLoop(void);						/*this is a simple init, creates the message Q xCntrlLoopQHndle*/
void controlLoop(void);							/*this is the control loop code*/
void xTaskControlLoop(void *pvParameters);		/*this is the task that regulates the control loop*/




#endif /* CNTRL_CONTROL_LOOP_H_ */