/*
 * cntrl_CONTROL_LOOP.c
 *
 * Created: 6/6/2017 4:24:53 PM
 *  Author: Gerald
 */

#include "cntrl_CONTROL_LOOP.h"

/*-----------------------------------------------------------*/
 /**
 * \brief This task inits the control loop, including creating the message Q
 */
void xInitCntrlLoop(void) {

	/*init DIO output state variables*/
	/*these are STATIC, i.e. retain their value */
	OUT_motor_dir			= MOTOR_DIR_INACTIVE;					/*Motor DIR (OUTPUT),  PB23*/
	OUT_output_en			= OUTPUT_EN_INACTIVE;				/*User Load Output EN (OUTPUT),  PA27*/
	OUT_battery_en			= BATTERY_EN_INACTIVE;				/*Enable Battery to discharge (OUTPUT), PB30 */
	OUT_charge_en			= BATTERY_CHARGE_EN_INACTIVE;		/*Enable uModule to CHARGE battery (OUTPUT), PB31 */

	//battery status state variables
	IBattery_mA					= 0;			/*battery current, mA*/
	IBattery_offset_mA			= 0;			/*battery offset for tare, mA*/
	VBattery_mV					= 0;			/*battery voltage, mV*/
	bat_chrg_status				= 0;			/*battery charge status 1-charging, 0-off*/
	AltUnregVmon_mV				= 0;			/*buss voltage, mV*/
	state_of_chrg				= 0;			/*state of charge*/

	//power status state variables
	ILoadMeas_mA				= 0;			/*load current, mA*/
	ILoadMeas_offset_mA			= 0;			/*load current offset, mA*/
	DCDCVmon_mV					= 0;			/*load voltage, mV*/
	DCDCImon_mA					= 0;			/*dc/dc current, mA*/

	//power status flags
	load_state					= 0;			/*Bit 0 - 1-on, 0-off*/
	load_tripped_state			= 0;			/*Bit 1 - 1-tripped, 0-not tripped*/
	sw_status_state				= 0;			/*Bit 2 - 1-on, 0-off*/
	auto_man_state				= 0;			/*Bit 3 - 1 - AUTO, 0 - MANUAL */

	Bat_I_Set					= 0;			/*Command for the Bat_I_Set channel of the DAC*/
	Bat_I_Set_last				= 0;			/*remembers last value (avoid sending command multiple times)*/
	ISet_DAC_mA					= 25000;		/*Command for the ISet_DAC_mA channel of the DAC*/
	ISet_DAC_mA_last			= 0;			/*remembers last value (avoid sending command multiple times)*/
	VSet_DAC_mV					= 24000;		/*Command for the VSet_DAC_mV channel of the DAC*/
	VSet_DAC_mV_last			= 0;			/*remembers last value (avoid sending command multiple times)*/

	motor_cmd_last				= 0;			/*remembers the last value of the motor command*/
	monitor_counter				= 0;            //frequency counter for printing monitor mode
	charge_control_state		= 0;            //Charge Control State machine state
	/*init PWM command structure*/
	xPwmCmd.uxChId				= 0;			/*This is the requested PWM channel (right now, only 1)*/
	xPwmCmd.uxDuty				= 0;			/*Requested duty cycle 0..100%*/
	xPwmCmd.uxOffOn				= 0;			/*1-ON, 0-OFF*/

	/*init DAC command structure */
	xI2CDacCmd.uxChId			= 0;			/*This stores the channel 1,2,3,4*/
	xI2CDacCmd.uxCounts			= 0;			/*This store the command counts 0..4095*/

	first_loop					= 1;			// 1 to indicate this is the first loop, gets reset to 0 after first_loop code is ran

}//xInitCntrlLoop

int32_t limit_int(int32_t value, int32_t lo_limit, int32_t hi_limit) {
	// ensure value is within lo_limit and hi_limit
	if (value < lo_limit) {
		value = lo_limit;
	}
	if (value > hi_limit) {
		value = hi_limit;
	}
	return value;

}

uint16_t Vset_DAC_mV2cnt(uint16_t dcdc_mV) {
	// returns the counts needed to set the DC/DC output voltage wtih dcdc_mV

	double	vset_dac_cnt = 0;

	#define VSET_LOW						( (double) 20000 )
	#define VSET_HIGH						( (double) 29000 )
	#define VSET_DAC_LOW				    ( (double) 0     )
	#define VSET_DAC_HIGH					( (double) 4095  )
	#define VSET_SCALE						( (double) (VSET_HIGH -VSET_LOW) / VSET_DAC_HIGH )

	vset_dac_cnt = ( ((double) dcdc_mV - VSET_LOW) / VSET_SCALE);


	return limit_int((uint16_t) vset_dac_cnt, VSET_DAC_LOW, VSET_DAC_HIGH);
}

/*-----------------------------------------------------------*/
 /**
 * \brief This the main process control loop, regulated by the xTaskControlLoop below
 */
 void controlLoop(void){
	 
	 // lmp rmv dbg code - COMMIT THIS CHANGE


     //Config COMMAND DATA from CONTROLLER in 0x41 MCAN message
    uint32_t cfg_Ibatt_Sp_state	    	= gbl_PwrCfg.cfg_Ibatt_Sp;				//Set Point charge current to battery
    uint32_t cfg_Vbatt_high_state		= gbl_PwrCfg.cfg_Vbatt_high;			//High Limit of Battery Voltage
   	uint32_t cfg_Vbatt_low_state	   	= gbl_PwrCfg.cfg_Vbatt_low;				//Low Limit of Battery Voltage
   	int32_t cfg_Ibatt_high_state		= gbl_PwrCfg.cfg_Ibatt_high;			//High Limit of battery Current (+Ibatt = Charge)
    int32_t cfg_Ibatt_low_state			= gbl_PwrCfg.cfg_Ibatt_low;				//High Limit of battery Current (-Ibatt = discharge)
   	uint32_t cfg_Ibatt_trickle_cut_state= gbl_PwrCfg.cfg_Ibatt_trickle_cut;		//Current at which to stop battery charging
   	uint32_t cfg_Vbatt_recharge_state	= gbl_PwrCfg.cfg_Vbatt_recharge;		//Battery Voltage to re initiate battery charge
  	uint32_t cfg_Alt_unreg_min_state	= gbl_PwrCfg.cfg_Alt_unreg_min;			//Min Voltage of alt_unreg
    uint32_t cfg_Alt_unreg_max_state	= gbl_PwrCfg.cfg_Alt_unreg_max;			//Max voltage of alt_unreg
    uint32_t cfg_I_load_max_state		= gbl_PwrCfg.cfg_I_load_max;			//Max user load for ms

	uint8_t battOpen_state				 = gbl_bat_Protection_Case.battOpen;								//
	bool battOvervolt_state				 = gbl_bat_Protection_Case.battOvervolt;				//
	bool battUndervolt_state			 = gbl_bat_Protection_Case.battUndervolt;					//
	uint8_t battOvercurrent_state		 = gbl_bat_Protection_Case.battOvercurrent;					//
	bool altBusUndervolt_state			 = gbl_bat_Protection_Case.altBusUndervolt;					//
	bool altBusOvervolt_state			 = gbl_bat_Protection_Case.altBusOvervolt;					//
	uint8_t loadOvercurrent_state		 = gbl_bat_Protection_Case.loadOvercurrent;					//
	bool twenty8vBusUndervolt_state		 = gbl_bat_Protection_Case.twenty8vBusUndervolt;		//
	bool twenty8vBusOvercurrent_state	 = gbl_bat_Protection_Case.twenty8vBusOvercurrent;	//
	uint8_t backfeedWarning_state		 = gbl_bat_Protection_Case.backfeedWarning;				    //

	//Counters for different Battery Protection Cases

	uint8_t battOpenCounter_state				 = gbl_bat_Protection_Case_Counter.battOpenCounter;				//
	uint8_t battOvervoltCounter_state			 = gbl_bat_Protection_Case_Counter.battOvervoltCounter;			//
	uint8_t battUndervoltCounter_state			 = gbl_bat_Protection_Case_Counter.battUndervoltCounter;			//
	uint8_t battOvercurrentCounter_state		 = gbl_bat_Protection_Case_Counter.battOvercurrentCounter;		//
	uint8_t altBusUndervoltCounter_state		 = gbl_bat_Protection_Case_Counter.altBusUndervoltCounter;		//
	uint8_t altBusOvervoltCounter_state			 = gbl_bat_Protection_Case_Counter.altBusOvervoltCounter;				//
	uint8_t loadOvercurrentCounter_state		 = gbl_bat_Protection_Case_Counter.loadOvercurrentCounter;			//
	uint8_t twenty8vBusUndervoltCounter_state	 = gbl_bat_Protection_Case_Counter.twenty8vBusUndervoltCounter;			//
	uint8_t twenty8vBusOvercurrentCounter_state	 = gbl_bat_Protection_Case_Counter.twenty8vBusOvercurrentCounter;			//
	uint8_t backfeedWarningCounter_state		 = gbl_bat_Protection_Case_Counter.backfeedWarningCounter;          //
	uint8_t ibatt_stateMachineCounter_state		 = gbl_bat_Protection_Case_Counter.ibatt_stateMachineCounter;          //

	//latest COMMAND DATA from CONTROLLER in 0x40 MCAN message
	uint8_t			motor_cmd		= gbl_PwrCmd.motor;						/*motor pwm command 0..255 (0..100%)*/
	uint16_t		set_v_cmd_mV	= VSET_DAC_CAN_TO_MV(gbl_PwrCmd.set_v);	/* 20V=0 ... 30V =255 set_v_cmd in mV of DC/DC output 20,000 to 29,000 */
	uint16_t		set_i_cmd_mA	= ISET_DAC_CAN_TO_MA(gbl_PwrCmd.set_i);	/*??A=0 ... 25A=255*/
    uint32_t		CAN_PwrCmd_tic	= gbl_PwrCmd.stats.rcv_tic_this;		// tic count when the Power Command CAN frame was received
	bool			CTRL_comms_valid= false;								// flag if power board is in communication with controller board

	//latest COMMAND FLAGS from CONTROLLER in 0x40 MCAN message
	uint8_t			run_cmd			= gbl_CmdFlags.sys_run;			/*Bit 0 - shutdown, 1-run, 0-off*/
	uint8_t			output_en_cmd	= gbl_CmdFlags.output_en;		/*Bit 1 - output enabled 1-on, 0-off*/
	uint8_t			bat_chrg_en_cmd = gbl_CmdFlags.bat_chrg_en;		/*Bit 2 - battery charge enabled, 1-charging, 0-off*/
	uint8_t			resv_3_cmd		= gbl_CmdFlags.resv_3;			/*Bit 3 - reserved*/
	uint8_t			resv_4_cmd		= gbl_CmdFlags.resv_4;			/*Bit 4 - reserved*/
	uint8_t			resv_5_cmd		= gbl_CmdFlags.resv_5;			/*Bit 5 - reserved*/
	uint8_t			resv_6_cmd		= gbl_CmdFlags.resv_6;			/*Bit 6 - reserved*/
	uint8_t			resv_7_cmd		= gbl_CmdFlags.resv_7;			/*Bit 7 - reserved*/

	//GET LATEST ANALOG INPUT DATA
	//data in mV
	//uint16_t	ain0_DCDCImon_mv			=	ADC_RAW_TO_MV(gbl_AnalogIn.ain0_DCDCImon);		/*PA02_DCDC_Imon cnts->mv*/
	//uint16_t	ain1_DCDCVmon_mv			= 	ADC_RAW_TO_MV(gbl_AnalogIn.ain1_DCDCVmon);		/*PA03_DCDC_Vmon cnts->mv*/
	//uint16_t	ain2_ILoadMeas_mv			= 	ADC_RAW_TO_MV(gbl_AnalogIn.ain2_ILoadMeas);		/*PB08_iLoad_Meas cnts->mv*/
	uint16_t	ain3_IShortMeas_mv			= 	ADC_RAW_TO_MV(gbl_AnalogIn.ain3_IShortMeas);	/*PB09_iShortCircuit cnts->mv*/
	//uint16_t	ain4_IBattery_mv			= 	ADC_RAW_TO_MV(gbl_AnalogIn.ain4_IBattery);		/*PA04_Batt_Imon cnts->mv*/
	//uint16_t	ain5_VBattery_mv			= 	ADC_RAW_TO_MV(gbl_AnalogIn.ain5_VBattery);		/*PA05_Bat_Vmon cnts->mv*/
	//uint16_t	ain6_AltUnregVmon_mv		= 	ADC_RAW_TO_MV(gbl_AnalogIn.ain6_AltUnregVmon);	/*PA06_ALT_Unreg_Vmon cnts->mv*/
	//uint16_t	ain7_Thermistor_mv			= 	ADC_RAW_TO_MV(gbl_AnalogIn.ain7_Thermistor);	/*PA07_Thermistor cnt->mv*/

	//data scaled
	DCDCImon_mA					=	limit_int(DCDC_IMON_COUNT_TO_MV(gbl_AnalogIn.ain0_DCDCImon),	0,	30000);							/*PA02_DCDC_Imon cnt->mA*/
	DCDCVmon_mV					= 	limit_int(DCDC_VMON_COUNT_TO_MV(gbl_AnalogIn.ain1_DCDCVmon),	0,	35000);							/*PA03_DCDC_Vmon cnt->mV*/
	ILoadMeas_mA				= 	limit_int(ILOAD_COUNT_TO_MA(gbl_AnalogIn.ain2_ILoadMeas) - ILoadMeas_offset_mA,	0,	35000);			/*PB08_iLoad_Meas cnt->mA*/
	IShortMeas_mA				= 	ISHORTCIR_MV_TO_MA(ain3_IShortMeas_mv);																/*PB09_iShortCircuit cnt->mA*/
	IBattery_mA					= 	limit_int(IBAT_COUNT_TO_MV(gbl_AnalogIn.ain4_IBattery),		-25000, 25000) /*- IBattery_offset_mA*/;	/*PA04_Batt_Imon cnt->mA*/
	VBattery_mV					= 	limit_int(BATV_VMON_COUNT_TO_MV(gbl_AnalogIn.ain5_VBattery),	0,	20000);							/*PA05_Bat_Vmon cnt->mV*/
	AltUnregVmon_mV				= 	limit_int(ALTV_VMON_COUNT_TO_MV(gbl_AnalogIn.ain6_AltUnregVmon),0,	60000);							/*PA06_ALT_Unreg_Vmon cnt->mV*/
	Thermistor_C				= 	0;//THERM_MV_TO_DEGC(ain7_Thermistor_mv);															/*PA07_Thermistor cnt->degC * 100 */


	//static uint16_t incr = 0;		// lmp used to printf for debugging
//
	////if (incr== 0) printf("vDCDC: %u mV (%u),\tvBatt: %u mV (%u),\tvAlt: %u mV (%u)\r\n", DCDCVmon_mV, gbl_AnalogIn.ain1_DCDCVmon, VBattery_mV, gbl_AnalogIn.ain5_VBattery, AltUnregVmon_mV, gbl_AnalogIn.ain6_AltUnregVmon);
	////if (incr== 0) printf("vDCDC: %u, vBatt: %u, vAlt: %u\r\n", gbl_AnalogIn.ain1_DCDCVmon, gbl_AnalogIn.ain5_VBattery, gbl_AnalogIn.ain6_AltUnregVmon);
	//incr++;
	//incr %= 50;



	//UPDATE STATES
	//battery status state variables
	state_of_chrg			= 0;	/*TBD*/						/*state of charge*/
	uint8_t	static	STATE_charge_control	= 0;					// state machine for battery charging

	//power status flags
	load_state					= 0;	/*TBD*/						/*Bit 0 - 1-on, 0-off*/
	load_tripped_state			= 0;	/*TBD*/						/*Bit 1 - 1-tripped, 0-not tripped*/
	sw_status_state				= (uint8_t) port_pin_get_input_level(MCU_SWITCH_STATUS); /*MCU Switch Status (INPUT), PB00*/
	auto_man_state				= gbl_PwrStatusFlags.auto_man;		/*Bit 3 - 1 - AUTO, 0 - MANUAL */

	// timing states
	uint32_t	current_tic		= xTaskGetTickCount();				// current tic count in milliseconds

	//BMF don't know what these are for
	bool motorDir_level;   //motor_direction
	bool output_level;   //user_load_enable
	bool battery_En_level;  //battery_enable
	bool battery_Charge_Enable;  //batery_charge_enable

	static uint8_t debug_cnt = 0 ;

	static uint8_t cnt	=0;  // temp

	////////////////////////////////////////////////////////////////////////////////////////////
	//CONTROL LOOP LOGIC --->>>>>>
	////////////////////////////////////////////////////////////////////////////////////////////

	// used to send data in the serial terminal interface
	// put as sub to declutter main control loop


	//vvvvvvvvvvvvvvvvvvv First Loop Actions vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
	if (first_loop) {
		first_loop++;
		if (first_loop > 20) {
			//Turn on battery enable to startup system
			OUT_battery_en = 1; //turn on battery to the rest of the system
		}
		
		if (first_loop > 20) {

			//get current from batt sensor and zero with estimated current from just power PCB 3.3V load
			IBattery_offset_mA = (IBattery_mA + 200);
			IBattery_offset_mA = limit_int(IBattery_offset_mA,-2000,2000); //limit amount of offset change

			//get current from userload sensor and zero
			ILoadMeas_offset_mA = ILoadMeas_mA;
			ILoadMeas_offset_mA = limit_int(ILoadMeas_offset_mA, -1000,1000);
			
			first_loop = 0; // reset first loop flag, code no longer runs
		}

	} // end first loop
	//^^^^^^^^^^^^^^^^^^^ END First Loop Actions ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^




	//Check Protections States and update Counters
	//Battery Open Case
	//if(x > y ){
	//battOpenCounter_state ++;
	//if (battOpenCounter_state > BATT_OPEN_COUNTER_MAX){
	//battOpen_state == 1;
	//}
	//}
	//if(y >= x){
	//battOpenCounter_state == 0;
	//battOpen_state == 0;
	//}

	// Battery Overvolt Case
	if ((VBattery_mV > cfg_Vbatt_high_state) && (battOvervoltCounter_state < 255)) {
		battOvervoltCounter_state ++;
	} else { // not overvolt
		battOvervoltCounter_state = 0;
		battOvervolt_state = false;
	}
	if (battOvervoltCounter_state > BATT_OVERVOLT_COUNTER_MAX){
		battOvervolt_state = true;
	}


	// Battery Undervolt Case
	if ((VBattery_mV < cfg_Vbatt_low_state) && (battUndervoltCounter_state < 255)){
		battUndervoltCounter_state ++;
	} else { // not undervolt
		battUndervoltCounter_state = 0;
		battUndervolt_state = false;
	}
	if (battUndervoltCounter_state > BATT_UNDERVOLT_COUNTER_MAX){
		battUndervolt_state = true;
	}

	//Battery Overcurrent Case
	if ( ((IBattery_mA > cfg_Ibatt_high_state) || (IBattery_mA < cfg_Ibatt_low_state)) && (battOvercurrentCounter_state < 255)){
		battOvercurrentCounter_state ++;
	} else { //not overcurrent
		battOvercurrentCounter_state = 0;
		battOvercurrent_state = 0;
	}
	if (battOvercurrentCounter_state > BATT_OVERCURRENT_COUNTER_MAX){
		battOvercurrent_state = 1;
	}


	//altBusUndervolt_state Case
	if(AltUnregVmon_mV < cfg_Alt_unreg_min_state){
		altBusUndervoltCounter_state ++;
		if (altBusUndervoltCounter_state > ALT_UNDERVOLT_COUNTER_MAX){
			altBusUndervolt_state = true;
		}
	}
	if(AltUnregVmon_mV >= cfg_Alt_unreg_min_state){
		altBusUndervoltCounter_state = 0;
		altBusUndervolt_state = false;
	}

	//altBusOvervolt_state Case
	if(AltUnregVmon_mV > cfg_Alt_unreg_max_state){
		altBusOvervoltCounter_state ++;
		if (altBusOvervoltCounter_state > ALT_OVERVOLT_COUNTER_MAX){
			altBusOvervolt_state = true;
		}
	}
	if(AltUnregVmon_mV <= cfg_Alt_unreg_max_state){
		altBusOvervoltCounter_state = 0;
		altBusOvervolt_state = false;
	}

	//loadOvercurrent_state Case
	//if(ILoadMeas_mA > cfg_I_load_max_state){
	//	loadOvercurrentCounter_state ++;						// commented out DSp. 11/21/2019, give capacitive load tolerance with Ed's board protection
	//	if (loadOvercurrentCounter_state > LOAD_OVERCURRENT_COUNTER_MAX){
	//		loadOvercurrent_state == 1;
	//	}
	//}
	if(ILoadMeas_mA <= cfg_I_load_max_state){
		loadOvercurrentCounter_state == 0;
		loadOvercurrent_state == 0;
	}

	//twenty8vBusUndervolt_state Case
	//if(x > y){
	//	  twenty8vBusUndervoltCounter_state ++;
	//if (twenty8vBusUndervoltCounter_state > TWENTYEIGHT_UNDERVOLT_COUNTER_MAX){
	//twenty8vBusUndervolt_state == 1;
	// }
	//}
	//if(y <= x){
	//twenty8vBusUndervoltCounter_state == 0;
	//twenty8vBusUndervolt_state == 0;
	// }

	//twenty8vBusOvercurrent_state Case
	//if(x > y){
	//	  twenty8vBusOvercurrentCounter_state ++;
	//if (twenty8vBusOvercurrentCounter_state > TWENTYEIGHT_OVERCURRENT_COUNTER_MAX){
	//twenty8vBusOvercurrent_state == 1;
	// }
	//}
	//if(y <= x){
	//twenty8vBusOvercurrentCounter_state == 0;
	//twenty8vBusOvercurrent_state == 0;
	// }

	//backfeedWarningCounter_state Case
	//if(x > y){
	//	  backfeedWarningCounter_state ++;
	//if (twenty8vBusOvercurrentCounter_state > BACKFEED_COUNTER_MAX){
	//backfeedWarning_state == 1;
	// }
	//}
	//if(y <= x){
	//backfeedWarningCounter_state == 0;
	//backfeedWarning_state == 0;
	// }




	//check for valid controller comms
	if (((current_tic - CAN_PwrCmd_tic) > CTRL_CAN_TIMEOUT) ||(gbl_PwrCmd.stats.fault>0)){ // last message was greater than 1000 ms.  current tic can go up to 1193 hours
		CTRL_comms_valid= false;
	} else {
		CTRL_comms_valid= true;
	}


	if (CTRL_comms_valid) { // good communications from controller

		if (run_cmd) { //controller is indicating that the system should be running

			OUT_battery_en		= 1;				//Power from battery
			OUT_output_en		= output_en_cmd;	// turn on user load based on CAN command
			
			if ((OUT_output_en == 0) && (gbl_DigInputs.uv_trip == 1)) gbl_DigInputs.uv_trip = 0;




			if (bat_chrg_en_cmd == 1) {

				OUT_charge_en = 1;					//turn on uModule
			}

			////vvvvvvvvvvvvvvvvvvv Battery Charge Control State Machine vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
			//if (((AltUnregVmon_mV) > (cfg_Alt_unreg_min_state)) && (bat_chrg_en_cmd == 1 )) {
//
				//switch (STATE_charge_control) {
					//case (0):  // -----------Qualify Battery for Charge---------------------
//
						////exit criteria
						//if ((!battOvervolt_state) &&
						//(!battUndervolt_state) &&
						//(IBattery_mA <= 500) &&		// battery current is roughly 0 mA, nothing in or out
						//(IBattery_mA >= -500)) { 	//50 mAmps
							//// PLACEHOLDER - TARE BATTERY CURRENT SENSOR
							//STATE_charge_control = 1;
						//} else {
							//STATE_charge_control = 0;
						//}
						//break;
//
					//case (1): // -------------Charge Battery---------------------------------
						//Bat_I_Set = cfg_Ibatt_Sp_state;		//set uModule I set point to cfg value
						//OUT_charge_en = 1;					//turn on uModule
//
						////exit criteria
						//if (battOvervolt_state) { 	// safeties
							//STATE_charge_control = 0;	//return to batt qualify
						//}
//
						//if ((IBattery_mA <= cfg_Ibatt_trickle_cut_state) && (VBattery_mV >= 16600)) {	// charge complete or open circuit battery
							//STATE_charge_control = 2;
						//}
						//break;
//
					//case (2): //-----------------Charge Maintain Mode----------------------------------------
						//Bat_I_Set = cfg_Ibatt_Sp_state;		//set uModule I set point to cfg value
						//OUT_charge_en = 0;			//turn off uModule
//
						////exit criteria
						//if (VBattery_mV < 15500) {	//batt voltage has dipped down enough to start charging again or open battery
							//ibatt_stateMachineCounter_state++;
						//} else {
							//ibatt_stateMachineCounter_state = 0;
						//}
						//if (ibatt_stateMachineCounter_state > IBATT_COUNTER_MAX) {
							//STATE_charge_control = 0;
						//}
//
						//break;
				//}
			//}
			else {  //AltUnregVmon_mV below threshold or CAN commanded charge enable=0, do not allow battery to charge
				STATE_charge_control = 0;
				OUT_charge_en = 0;			  //turn off uModule
			}
			//^^^^^^^^^^^^^^^^^^^ END Battery Charge Control State Machine ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

			//PLACEHOLDED - user load logic including short circuit
		} else { // controller has commanded a shutdown
			// controller has already done all the housekeeping and is ready for shutdown
			OUT_charge_en	= 0;
			OUT_output_en	= 0;

			motor_cmd		= 0;	// make sure motor command is not on
			if (sw_status_state == 0) {
				OUT_battery_en	= 0;	// kills the battery, uC should loose power
			}
		} // END controller run_cmd



	} else { // ---------------invalid COMMS----------------------
		OUT_charge_en		= 0;
		OUT_output_en		= 0;	// turn on user load based on CAN command
		motor_cmd			= 0;
		STATE_charge_control= 0;
		clearCANcommands();			// clear CAN commands

		if (sw_status_state == 0) {	// lost controller comms and switch is off
			OUT_battery_en		= 0;				//Power from battery, shutdown
		}


		//PLACEHOLDER - restart controller
	} // ---------------------end valid COMMS---------------------

	//PLACEHOLDED safeties - done at all times

	if (cnt < 200) {
		cnt++;
	} else {
		cnt = 0;
		printf("Motor output %d \r\n", motor_cmd);
	}


	VSet_DAC_mV = set_v_cmd_mV;	// sets the DC/DC output voltage based on CAN command for all states
	ISet_DAC_mA = 24999; //set_i_cmd_mA;	// sets the DC/DC output current limit based on CAN command for all states	// lmp - this was already hard coded to 24999 9/26/2018
	

	/*
	// This section allows voltage to droop from DC-DC based on instantaneous engine speed to allow larger capacitive loads
	//Works instead of "ISet_DAC_mA" value above
	//UNTESTED!! 2-25-2020  DSp
	float32_t set_current = (1.1765 * AltUnregVmon_mV) - 11176;
	
	if (set_current < 18000) ISet_DAC_mA = 18000;
	
	else if (set_current > 24999) ISet_DAC_mA = 24999;  // 24999 is the max value that we can command
	
	else ISet_DAC_mA = set_current;
	*/






	///////////////////////////////////////////////////////////////////////////////////////////
	//UPDATE OUTPUTS -- >>>>
	///////////////////////////////////////////////////////////////////////////////////////////


	//UPDATE DIO OUTPUT STATES
	//  MOTOR_DIR
	port_pin_set_output_level(MOTOR_DIR, OUT_motor_dir);							/*Motor DIR (OUTPUT),  PB23*/
	
	//  USER LOAD OUPUT_EN
	#if (PROCESS_UNDERVOLTAGE_TRIP_ENABLED)
		if (gbl_DigInputs.uv_trip == 0) port_pin_set_output_level(OUTPUT_EN, OUT_output_en);							/*Output EN (OUTPUT),  PA27*/
		else  port_pin_set_output_level(OUTPUT_EN, OUTPUT_EN_INACTIVE);							/*Output EN (OUTPUT),  PA27*/
	#else
		port_pin_set_output_level(OUTPUT_EN, OUT_output_en);							/*Output EN (OUTPUT),  PA27*/
	#endif
	port_pin_set_output_level(BATTERY_EN, OUT_battery_en);							/*Battery EN (OUTPUT), PB30 */
	
	//  BATT CHARGE ENABLE
	//port_pin_set_output_level(BATTERY_CHARGE_EN, BATTERY_CHARGE_EN_ACTIVE);					/*Battery RUN/CHARGE (OUTPUT), PB31 */



	////////
	////  lmp - this was added to retry enabling the user output every 5 seconds  // 9/25/2018
//
	//static uint32_t uvResetCount = 0;
	//if (gbl_DigInputs.uv_trip == 1) {
		//uvResetCount++;
		////printf("cnt: %u\ttrip: %u\n", uvResetCount, gbl_DigInputs.uv_trip);
	//}
	//else uvResetCount = 0;
	//if (uvResetCount >= 200) {
		//uvResetCount = 0;
		//gbl_DigInputs.uv_trip = 0;
		////printf("cnt: %u\ttrip: %u\n", uvResetCount, gbl_DigInputs.uv_trip);
	//}
	
	port_pin_set_output_level(LED0_PIN, gbl_DigInputs.uv_trip);	// lmp dbg  -  this can be removed

	////////




	////UPDATE PWM MOTOR COMMAND
	//if (motor_cmd!=motor_cmd_last) {
		////xPwmCmd.uxChId				= 1;			/*This is the requested PWM channel (right now, only 1)*/		// lmp rmv - replaced by the spi command
		//xPwmCmd.uxDuty				= PWM_CMD_TO_DUTY(motor_cmd); /*Requested duty cycle 0..100%*/
		//if ((xPwmCmd.uxDuty>0)&&(xPwmCmd.uxDuty<=100)) {
			//xPwmCmd.uxOffOn				= 1;			/*1-ON, 0-OFF*/
			//} else {
			//xPwmCmd.uxOffOn				= 0;			/*1-ON, 0-OFF*/
		//}//end if
		/////*send command to PWM task*/
		////if (!xQueueSend(xPwmQHndle, &xPwmCmd, 1000)){
		////printf	("\r\nfail to send to xPwmQHndle queue\n");
		////}//end if
		////motor_cmd_last = motor_cmd;		//remember the last value, to avoid setting pwm multiple times if no change*/
//
//
		//////////// new
		//if (xPwmCmd.uxOffOn	== 1)	motor_orque((uint16_t)xPwmCmd.uxDuty*10);
		////else motor_set_torque(0);
	//}//end if motor_cmd

	//////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////
	static uint8_t theLast = 0;
	if (gbl_PwrCmd.motor != theLast) {		// lmp this is the new command to send the torque command to the motor controller via SPI
		motor_run((uint16_t)PWM_CMD_TO_DUTY(motor_cmd));
		theLast = gbl_PwrCmd.motor;
	}//end if motor_cmd
	





	static uint32_t temp_oneShot = 0;  // lmp - i think I had added this to prevent power on issues to make sure the DAC is updated correctly


	//UPDATE DAC COMMANDS
	if (VSet_DAC_mV!=VSet_DAC_mV_last) {
		//VSet_DAC_mV
		xI2CDacCmd.uxChId			= 1;			/*Set VSet_DAC_mV Channel*/
		xI2CDacCmd.uxCounts			= Vset_DAC_mV2cnt(VSet_DAC_mV);	/*Command counts*/
		/*send command to I2C_DAC task*/
		if (!xQueueSend(xI2CDacQHndle, &xI2CDacCmd, 1000)){
			printf	("\r\nfail to send to xI2CDacQHndle queue\n");
		}//end if
		VSet_DAC_mV_last = VSet_DAC_mV;	//remember the last value, so as to not re-send the same value multiple times
	}//end if



	//ISET_DAC
	else if ((ISet_DAC_mA!=ISet_DAC_mA_last) || (temp_oneShot < 10)) {
		temp_oneShot++;
		//Iset_DAC
		xI2CDacCmd.uxChId			= 2;			/*Set Iset_DAC Channel*/
		xI2CDacCmd.uxCounts			= ISET_DAC_MA_TO_CNTS(ISet_DAC_mA);			/*Command counts*/
		/*send command to I2C_DAC task*/
		if (!xQueueSend(xI2CDacQHndle, &xI2CDacCmd, 1000)){
			printf	("\r\nfail to send to xI2CDacQHndle queue\n");
		}//end if
		ISet_DAC_mA_last = ISet_DAC_mA;	//remember the last value, so as to not re-send the same value multiple times
	}//end ISet_DAC_mA

	//BAT_I_SET
	else if (Bat_I_Set!=Bat_I_Set_last) {
		xI2CDacCmd.uxChId			= 3;		 /*Set Bat_I_Set Channel*/
		xI2CDacCmd.uxCounts			= BATT_I_SET_DAC_MV_TO_CNTS(Bat_I_Set);			/*Command counts*/
		/*send command to I2C_DAC task*/
		if (!xQueueSend(xI2CDacQHndle, &xI2CDacCmd, 1000)){
			printf	("\r\nfail to send to xI2CDacQHndle queue\n");
		}//endif
		Bat_I_Set_last = Bat_I_Set;	//remember the last value, so as to not re-send the same value multiple times

	}//end Bat_I_Set



	//UPDATE GLOBALS
	//DIO Inputs
	gbl_DigInputs.sw_status       = sw_status_state;								/*MCU Switch Status (INPUT), PB00*/
	//DIO Output
	gbl_DigOutputs.motor_dir      = OUT_motor_dir;								/*Motor DIR (OUTPUT),  PB23*/
	gbl_DigOutputs.output_en      = OUT_output_en;								/*Output EN (OUTPUT),  PA27*/
	gbl_DigOutputs.bat_en         = OUT_battery_en;								/*Battery EN (OUTPUT), PB30 */
    gbl_DigOutputs.bat_chrg_en    = OUT_charge_en;								/*Battery RUN/CHARGE (OUTPUT), PB31 */

	//Dac Output Levels
	gbl_DacOutputs.bat_i_set = Bat_I_Set;
	//UPDATEPROCESS VARIABLES

	//update battery status global (used in MCAN msg 0x30)
	gbl_BatStatus.bat_chrg_status	= STATE_charge_control;						/*battery charge status 0-off or testing, 1=charging, 2=charged*/
	gbl_BatStatus.bat_i				= IBattery_mA;								/*battery current, mA*/
	gbl_BatStatus.bat_v				= VBattery_mV;								/*battery voltage, mV*/
	gbl_BatStatus.buss_v			= AltUnregVmon_mV;							/*buss voltage, mV*/
	gbl_BatStatus.state_of_chrg		= state_of_chrg;							/*state of charge*/

	//update power status flags global (used for MCAN msg 0x31)
	gbl_PwrStatusFlags.load				= OUT_output_en;								/*Bit 0 - 1-on, 0-off*/
	gbl_PwrStatusFlags.load_tripped		= load_tripped_state;							/*Bit 1 - 1-tripped, 0-not tripped*/
	gbl_PwrStatusFlags.sw_status		= sw_status_state;								/*Bit 2 - 1-on, 0-off*/
	gbl_PwrStatusFlags.auto_man			= auto_man_state;								/*Bit 3 - 1 - AUTO, 0 - MANUAL */
	gbl_PwrStatusFlags.resv_4			= 0;											/*Bit 4 - reserved*/
	gbl_PwrStatusFlags.resv_5			= 0;											/*Bit 5 - reserved*/
	gbl_PwrStatusFlags.flt_battOpen		= battOpen_state;								/*Bit 6 - reserved*/
	gbl_PwrStatusFlags.flt_battOvervolt	= battOvervolt_state;							/*Bit 7 - reserved*/

	//update power status fault flags global (used for MCAN msg 0x31)
	gbl_PwrFaultFlags.flt_battUndervolt			= battUndervolt_state;					/*Bit 0 - reserved*/
	gbl_PwrFaultFlags.flt_battOvercurrent		= battOvercurrent_state;				/*Bit 1 - reserved*/
	gbl_PwrFaultFlags.flt_altBusUndervolt		= altBusUndervolt_state;				/*Bit 2 - reserved*/
	gbl_PwrFaultFlags.flt_altBusOvervolt		= altBusOvervolt_state;					/*Bit 3 - reserved*/
	gbl_PwrFaultFlags.flt_loadOvercurrent		= loadOvercurrent_state;				/*Bit 4 - reserved*/
	gbl_PwrFaultFlags.flt_twenty8vBusUndervolt	= twenty8vBusUndervolt_state;			/*Bit 5 - reserved*/
	gbl_PwrFaultFlags.flt_twenty8vBusOvercurrent= twenty8vBusOvercurrent_state;			/*Bit 6 - reserved*/
	gbl_PwrFaultFlags.flt_backfeedWarning		= backfeedWarning_state;				/*Bit 7 - reserved*/

	gbl_bat_Protection_Case_Counter.battOpenCounter = battOpenCounter_state;				//
	gbl_bat_Protection_Case_Counter.battOvervoltCounter = battOvervoltCounter_state;			//
	gbl_bat_Protection_Case_Counter.battUndervoltCounter = battUndervoltCounter_state;			//
	gbl_bat_Protection_Case_Counter. battOvercurrentCounter = battOvercurrentCounter_state;		//
	gbl_bat_Protection_Case_Counter.altBusUndervoltCounter = altBusUndervoltCounter_state;		//
	gbl_bat_Protection_Case_Counter.altBusOvervoltCounter = altBusOvervoltCounter_state	;				//
	gbl_bat_Protection_Case_Counter.loadOvercurrentCounter = loadOvercurrentCounter_state;			//
	gbl_bat_Protection_Case_Counter.twenty8vBusUndervoltCounter = twenty8vBusUndervoltCounter_state;			//
	gbl_bat_Protection_Case_Counter.twenty8vBusOvercurrentCounter = twenty8vBusOvercurrentCounter_state;			//
	gbl_bat_Protection_Case_Counter.backfeedWarningCounter = backfeedWarningCounter_state;          //
	gbl_bat_Protection_Case_Counter.ibatt_stateMachineCounter = ibatt_stateMachineCounter_state;          //

	gbl_bat_Protection_Case.battOpen = battOpen_state;								//
	gbl_bat_Protection_Case.battOvervolt = battOvervolt_state;						//
	gbl_bat_Protection_Case.battUndervolt = battUndervolt_state;					//
	gbl_bat_Protection_Case.battOvercurrent = battOvercurrent_state;					//
	gbl_bat_Protection_Case.altBusUndervolt = altBusUndervolt_state;					//
	gbl_bat_Protection_Case.altBusOvervolt = altBusOvervolt_state;					//
	gbl_bat_Protection_Case.loadOvercurrent = loadOvercurrent_state;					//
	gbl_bat_Protection_Case.twenty8vBusUndervolt = twenty8vBusUndervolt_state;		//
	gbl_bat_Protection_Case.twenty8vBusOvercurrent = twenty8vBusOvercurrent_state;	//
	gbl_bat_Protection_Case.backfeedWarning = backfeedWarning_state;				    //


	//update power status global (used for MCAN msg 0x31)
	gbl_PwrStatus.load_i			= ILoadMeas_mA;									/*load current, mA*/
	gbl_PwrStatus.load_v			= DCDCVmon_mV;									/*load voltage, mV*/
	gbl_PwrStatus.dc_dc_i			= DCDCImon_mA;								/*dc/dc current, mA*/
	gbl_PwrStatus.pwr_status		= (gbl_PwrStatusFlags.load				        << 0) |		/*Bit 0 - 1-on, 0-off*/
									  (gbl_PwrStatusFlags.load_tripped			    << 1) |		/*Bit 1 - 1-tripped, 0-not tripped*/
									  (gbl_PwrStatusFlags.sw_status  		        << 2) |		/*Bit 2 - 1-on, 0-off*/
									  (gbl_PwrStatusFlags.auto_man	    			<< 3) |		/*Bit 3 - 1 - auto, 0 - manual */
									  (gbl_PwrStatusFlags.resv_4	    			<< 4) |		/*Bit 4 - reserved*/
									  (gbl_PwrStatusFlags.resv_5				    << 5) |		/*Bit 5 - reserved*/
								      (gbl_PwrStatusFlags.flt_battOpen				<< 6) |		/*Bit 6 - reserved*/
									  (gbl_PwrStatusFlags.flt_battOvervolt			<< 7);		/*Bit 7 - reserved*/
	gbl_PwrStatus.faults			= (gbl_PwrFaultFlags.flt_battUndervolt			<< 0) |		/*Bit 0 - reserved*/
									  (gbl_PwrFaultFlags.flt_battOvercurrent		<< 1) |		/*Bit 1 - reserved*/
									  (gbl_PwrFaultFlags.flt_altBusUndervolt		<< 2) |		/*Bit 2 - reserved*/
									  (gbl_PwrFaultFlags.flt_altBusOvervolt			<< 3) |		/*Bit 3 - reserved*/
									  (gbl_PwrFaultFlags.flt_loadOvercurrent		<< 4) |		/*Bit 4 - reserved*/
									  (gbl_PwrFaultFlags.flt_twenty8vBusUndervolt	<< 5) |		/*Bit 5 - reserved*/
									  (gbl_PwrFaultFlags.flt_twenty8vBusOvercurrent	<< 6) |		/*Bit 6 - reserved*/
									  (gbl_PwrFaultFlags.flt_backfeedWarning		<< 7);   	/*Bit 7 - reserved*/



 }//controlLoop



 /*-----------------------------------------------------------*/
 /**
 * \brief This task is the wrapper for the main process control loop
 */
void xTaskControlLoop(void *pvParameters)
{

	/* set global flag that this task has been instantiate*/
	/* this allows code in adc_complete_callback() to send msgs to this task via xCntrlLoopQHndle */
	gbl_CntrlLoopStartedMem = 1;


	/* this is the infinite task loop */
	for (;;) {

		/*spin the control loop */
		/*You spin me right 'round, baby, right 'round; Like a record, baby, right 'round, 'round, 'round*/
		/*https://www.youtube.com/watch?v=PGNiXGX2nLU*/
		controlLoop();

		// lmp rmv dbg code - COMMIT THIS CHANGE

		vTaskDelay(CONTROL_LOOP_TASK_MS);

	}//end for

	/* for completeness, this should never happen */
	vTaskDelete( NULL );

}//end xTaskControlLoop

