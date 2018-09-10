/*
 * cntrl_GLOBALS.c
 *
 * Created: 5/7/2017 10:27:20 PM
 *  Author: Gerald
 */ 

#include "cntrl_GLOBALS.h"

/* global containing most recent analog input sample*/
struct analogInputType			gbl_AnalogIn		= {0};

/* global containing Different Battery Protection Cases*/
struct batProtectionCase		gbl_bat_Protection_Case		= {0};

/* global containing Different Battery Protection Case Counters*/
struct batProtectionCaseCounter		gbl_bat_Protection_Case_Counter		= {0};

/* global object to contain info for message BATTERY_STATUS 0x30 */
struct batStatusMsgtype			gbl_BatStatus		= {0};

/* global bit-mapped STATUS FLAGS used in message POWER_STATUS 0x31 */
struct statusFlagType			gbl_PwrStatusFlags	= {0};

/* global bit-mapped FAULT FLAGS used in message POWER_STATUS 0x31 */
struct fltFlagType				gbl_PwrFaultFlags	= {0};

/* global object to contain info for message POWER_STATUS 0x31 */
struct pwrStatusMsgtype			gbl_PwrStatus		= {0};

//global bit-mapped COMMAND FLAGS used in message POWER_COMMAND 0x40 */
struct cmdFlagType				gbl_CmdFlags		= {0};

/* command data FROM the controller for message POWER_COMMAND 0x40 */
struct pwrCmdMsgType			gbl_PwrCmd			= {0};

/* command data FROM the controller for message POWER_CFG_COMMAND 0x41 */
struct pwrCfgCmdMsgType			gbl_PwrCfgCmd		= {0};
       
struct pwrCfg					gbl_PwrCfg			= {0};

struct dioInputType             gbl_DigInputs       = {0};

struct DacOutputType            gbl_DacOutputs       = {0};

struct dioOutputType            gbl_DigOutputs      = {0};      

struct configType				gbl_configType		= {0};

uint8_t gbl_CntrlLoopStartedMem;

//this function does a one-time reset of all globals declared here //
void initGlobalsSafe(void){

	gbl_PwrStatusFlags.auto_man			= 1; //jmp
	gbl_CmdFlags.sys_run				= 1; //jmp
	gbl_CntrlLoopStartedMem				= 0;

	gbl_AnalogIn.ain0_DCDCImon			= 0;
	gbl_AnalogIn.ain1_DCDCVmon			= 0;
	gbl_AnalogIn.ain2_ILoadMeas			= 0;
	gbl_AnalogIn.ain3_IShortMeas		= 0;
	gbl_AnalogIn.ain4_IBattery			= 0;
	gbl_AnalogIn.ain5_VBattery			= 0;
	gbl_AnalogIn.ain6_AltUnregVmon		= 0;
	gbl_AnalogIn.ain2_ILoadMeas			= 0;
		
	gbl_PwrCfg.cfg_Ibatt_Sp			 = 1000;			 //Set Point charge current to battery
	gbl_PwrCfg.cfg_Vbatt_high		 = 17000;			 //High Limit of Battery Voltage
	gbl_PwrCfg.cfg_Vbatt_low		 = 0000;			 //Low Limit of Battery Voltage
	gbl_PwrCfg.cfg_Ibatt_high		 = 6000;			 //High Limit of battery Current (+Ibatt = Charge)
	gbl_PwrCfg.cfg_Ibatt_low		 = -25000;			 //High Limit of battery Current (-Ibatt = discharge)
	gbl_PwrCfg.cfg_Ibatt_trickle_cut = 100;				 //Current at which to stop battery charging
	gbl_PwrCfg.cfg_Vbatt_recharge    = 15500;			 //Battery Voltage to re initiate battery charge
	gbl_PwrCfg.cfg_Alt_unreg_min	 = 18000;			 //Min Voltage of alt_unreg
	gbl_PwrCfg.cfg_Alt_unreg_max	 = 50000;			 //Max voltage of alt_unreg
	gbl_PwrCfg.cfg_I_load_max		 = 10;				 //Max user load for ms


	 gbl_bat_Protection_Case.battOpen					= 0;					//
	 gbl_bat_Protection_Case.battOvervolt				= 0;					//
	 gbl_bat_Protection_Case.battUndervolt				= 0;					//
	 gbl_bat_Protection_Case.battOvercurrent			= 0;					//
	 gbl_bat_Protection_Case.altBusUndervolt			= 0;					//
	 gbl_bat_Protection_Case.altBusOvervolt				= 0;					//
	 gbl_bat_Protection_Case.loadOvercurrent			= 0;					//
	 gbl_bat_Protection_Case.twenty8vBusUndervolt		= 0;					//
	 gbl_bat_Protection_Case.twenty8vBusOvercurrent		= 0;					//
	 gbl_bat_Protection_Case.backfeedWarning			= 0;				    //

		//Counters for different Battery Protection Cases
	 gbl_bat_Protection_Case_Counter.battOpenCounter				= 0;		//
	 gbl_bat_Protection_Case_Counter.battOvervoltCounter			= 0;		//
	 gbl_bat_Protection_Case_Counter.battUndervoltCounter			= 0;		//
	 gbl_bat_Protection_Case_Counter.battOvercurrentCounter			= 0;		//
	 gbl_bat_Protection_Case_Counter.altBusUndervoltCounter			= 0;		//
     gbl_bat_Protection_Case_Counter.altBusOvervoltCounter			= 0;		//
	 gbl_bat_Protection_Case_Counter.loadOvercurrentCounter			= 0;		//
	 gbl_bat_Protection_Case_Counter.twenty8vBusUndervoltCounter	= 0;		//
	 gbl_bat_Protection_Case_Counter.twenty8vBusOvercurrentCounter	= 0;		//
	 gbl_bat_Protection_Case_Counter.backfeedWarningCounter			= 0;        //
	 gbl_bat_Protection_Case_Counter.ibatt_stateMachineCounter		= 0;

	 gbl_configType.monitor_mode = 1;
}//initGlobalsSafe

void clearCANcommands(void) {
	// clears old CAN command messages from global variables so it does not start with their values

	//latest COMMAND DATA from CONTROLLER in 0x40 MCAN message
	gbl_PwrCmd.motor	= 0;	//turn off motor control
	//gbl_PwrCmd.set_v) = 0;	// leave DC/DC Vset at last value
	//gbl_PwrCmd.set_i) = 255;	// leave DC/DC Iset at last value


	//latest COMMAND FLAGS from CONTROLLER in 0x40 MCAN message
	gbl_CmdFlags.sys_run	= 1;			/*Bit 0 - shutdown, 1-run, 0-off*/
	gbl_CmdFlags.output_en	= 0;			/*Bit 1 - output enabled 1-on, 0-off*/
	gbl_CmdFlags.bat_chrg_en= 0;			/*Bit 2 - battery charge enabled, 1-charging, 0-off*/
	gbl_CmdFlags.resv_3		= 0;			/*Bit 3 - reserved*/
	gbl_CmdFlags.resv_4		= 0;			/*Bit 4 - reserved*/
	gbl_CmdFlags.resv_5		= 0;			/*Bit 5 - reserved*/
	gbl_CmdFlags.resv_6		= 0;			/*Bit 6 - reserved*/
	gbl_CmdFlags.resv_7		= 0;			/*Bit 7 - reserved*/
	
	
}