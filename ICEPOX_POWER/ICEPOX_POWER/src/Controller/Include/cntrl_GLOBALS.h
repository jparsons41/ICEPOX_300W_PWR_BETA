/*
 * cntrl_GLOBALS.h
 *
 * Created: 5/7/2017 10:27:05 PM
 *  Author: Gerald
 */ 


#ifndef CNTRL_GLOBALS_H_
#define CNTRL_GLOBALS_H_

/* Standard includes */
#include <asf.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>

#define		MANUAL_MODE					0
#define		AUTO_MODE					1

extern uint8_t gbl_CntrlLoopStartedMem;

//This structure is used to store the most recent collection of analog inputs
//A queue also store a number of samples for debugging, but these will be the 
//most recent
struct analogInputType {
	uint16_t ain0_DCDCImon;				/*PA02 DC/DC I monitor*/
	uint16_t ain1_DCDCVmon;				/*PA03 DC/DC C monitor*/
	uint16_t ain2_ILoadMeas;			/*PB08 Output Current Measurement (hall sensor)*/
	uint16_t ain3_IShortMeas;			/*PB09 Short Circuit Current Measurement*/
	uint16_t ain4_IBattery;				/*PA04 Battery Current Measurement*/
	uint16_t ain5_VBattery;				/*PA05 Battery Voltage Measurement*/
	uint16_t ain6_AltUnregVmon;			/*PA06 Alternator Unregulated Voltage Measurement*/
	uint16_t ain7_Thermistor;			/*PA07 Thermistor*/
};//analogInputType
//forward deceleration of global object to contain the most recent analog input sample */
extern struct analogInputType  gbl_AnalogIn;

struct batProtectionCase {
	uint8_t battOpen;				//
	uint8_t battOvervolt;			//	
	bool battUndervolt;			//
	uint8_t battOvercurrent;		//
	uint8_t altBusUndervolt;		//
	uint8_t altBusOvervolt;				//
	uint8_t loadOvercurrent;			//
	uint8_t twenty8vBusUndervolt;			//
	uint8_t twenty8vBusOvercurrent;			//
	uint8_t backfeedWarning;          //
};//batProtectionCase
//forward deceleration of global object to contain the most recent analog input sample */
extern struct batProtectionCase  gbl_bat_Protection_Case;

struct batProtectionCaseCounter {
	uint8_t battOpenCounter;				//
	uint8_t battOvervoltCounter;			//
	int battUndervoltCounter;			//
	uint8_t battOvercurrentCounter;		//
	uint8_t altBusUndervoltCounter;		//
	uint8_t altBusOvervoltCounter;				//
	uint8_t loadOvercurrentCounter;			//
	uint8_t twenty8vBusUndervoltCounter;			//
	uint8_t twenty8vBusOvercurrentCounter;			//
	uint8_t backfeedWarningCounter;          //
	uint8_t ibatt_stateMachineCounter;
};//batProtectionCase
//forward deceleration of global object to contain the most recent analog input sample */
extern struct batProtectionCaseCounter  gbl_bat_Protection_Case_Counter;


//This structure is used to store values in message BATTERY_STATUS (0x30)
//This is sent back via CAN to CONTROLLER on a periodic basis.
struct batStatusMsgtype {
	int16_t			bat_i;						/*battery current, mA*/
	uint16_t		bat_v;						/*battery voltage, mV*/
	uint8_t			bat_chrg_status;			/*battery charge status 1-charging, 0-off*/
	uint16_t		buss_v;						/*buss voltage, mV*/
	uint8_t			state_of_chrg;				/*state of charge*/	
};//batStatusMsgtype 
//forward deceleration of global object to contain info in message BATTERY_STATUS 0x30 */
extern struct batStatusMsgtype  gbl_BatStatus;

//object to contain bit-mapped STATUS FLAGS used in POWER_STATUS 0x31
struct statusFlagType {
	uint8_t			load;						/*Bit 0 - 1-on, 0-off*/
	uint8_t			load_tripped;				/*Bit 1 - 1-tripped, 0-not tripped*/
	uint8_t			sw_status;					/*Bit 2 - 1-on, 0-off*/	
	uint8_t			auto_man;					/*Bit 3 - 1 - AUTO, 0 - MANUAL */	
	uint8_t			resv_4;						/*Bit 4 - reserved*/	
	uint8_t			resv_5;						/*Bit 5 - reserved*/	
	uint8_t			flt_battOpen;				/*Bit 6 - reserved*/	
	uint8_t			flt_battOvervolt;			/*Bit 7 - reserved*/										
};//statusFlagType
//forward deceleration of info for global bit-mapped  byte used in message POWER_STATUS 0x31
extern struct statusFlagType gbl_PwrStatusFlags;

//object to contain bit-mapped FAULT FLAGS used in POWER_STATUS 0x31
struct fltFlagType {
	uint8_t			flt_battUndervolt;					/*Bit 0 - reserved*/
	uint8_t			flt_battOvercurrent;				/*Bit 1 - reserved*/	
	uint8_t			flt_altBusUndervolt;				/*Bit 2 - reserved*/	
	uint8_t			flt_altBusOvervolt;					/*Bit 4 - reserved*/	
	uint8_t			flt_loadOvercurrent;				/*Bit 5 - reserved*/	
	uint8_t			flt_twenty8vBusUndervolt;			/*Bit 6 - reserved*/	
	uint8_t			flt_twenty8vBusOvercurrent;			/*Bit 7 - reserved*/	
	uint8_t			flt_backfeedWarning;				/*Bit 8 - reserved*/	
										
};//fltFlagType
//forward deceleration of info for global bit-mapped byte used in message POWER_STATUS 0x31
extern struct fltFlagType gbl_PwrFaultFlags;

//This structure is used to store values for the POWER_STATUS (0x31) data packet.
//This is sent back via CAN to CONTROLLER on a periodic basis.
struct pwrStatusMsgtype {
	uint16_t		load_i;						/*load current, mA*/
	uint16_t		load_v;						/*load voltage, mV*/
	uint16_t		dc_dc_i;					/*dc/dc current, mA*/
	uint8_t			pwr_status;					/*power status (bit mapped)*/
	uint8_t			faults;						/*faults (bit mapped)*/
};//pwrStatusMsgtype
//forward deceleration of global power status object POWER_STATUS 0x31
extern struct pwrStatusMsgtype gbl_PwrStatus;

//object to contain bit-mapped COMMAND FLAGS used in message POWER_COMMAND 0x40
struct cmdFlagType {
	uint8_t			sys_run;					/*Bit 0 - shutdown, 1-run, 0-off*/
	uint8_t			output_en;					/*Bit 1 - output enabled 1-on, 0-off*/
	uint8_t			bat_chrg_en;				/*Bit 2 - battery charge enabled, 1-charging, 0-off*/
	uint8_t			resv_3;						/*Bit 3 - reserved*/	
	uint8_t			resv_4;						/*Bit 4 - reserved*/	
	uint8_t			resv_5;						/*Bit 5 - reserved*/	
	uint8_t			resv_6;						/*Bit 6 - reserved*/	
	uint8_t			resv_7;						/*Bit 7 - reserved*/										
};//cmdFlagType
//forward deceleration of info for global bit-mapped byte used in POWER_COMMAND 0x40
extern struct cmdFlagType gbl_CmdFlags;

/*This structure contains some tracking variables for keeping up with when messages are received*/
struct rcvMsgStatsType {
	uint32_t		rcv_tic_this;
	uint32_t		rcv_tic_last;
	uint32_t		rcv_elapsed;
	uint8_t			fault;
};//rcvMsgStatsType

//This stores the command data from the controller POWER_COMMAND 0x40
struct pwrCmdMsgType {
	uint8_t			motor;					/*motor pwm command 0..255 (0..100%)*/
	uint8_t			cmd;					/*(bit mapped)*/
	uint8_t			set_v;					/*20V=0 ... 30V =255*/
	uint8_t			set_i;					/*??A=0 ... ??A=255*/
	struct rcvMsgStatsType	stats;			/*contains stats about when this data was received*/
};//pwrCommandMsgtype
//forward deceleration of the command data FROM the controller POWER_COMMAND 0x40
extern struct pwrCmdMsgType gbl_PwrCmd;

//This stores the command data from the controller POWER_CFG_COMMAND 0x41
struct pwrCfgCmdMsgType {
	uint8_t			command;					/*motor pwm command 0..255 (0..100%)*/
	uint32_t		set_value;				      	/*8 bit value 0-10*/
	struct rcvMsgStatsType	stats;			/*contains stats about when this data was received*/
};//pwrCommandMsgtype
//forward deceleration of the command data FROM the controller POWER_CFG_COMMAND 0x41
extern struct pwrCfgCmdMsgType gbl_PwrCfgCmd;

//This stores the command data from the controller POWER_CFG_COMMAND 0x41
struct pwrCfg {
		uint32_t        cfg_Ibatt_Sp;
		uint32_t		cfg_Vbatt_high;                      //High Limit of Battery Voltage
		uint32_t		cfg_Vbatt_low;                       //Low Limit of Battery Voltage
		uint32_t		cfg_Ibatt_high;                      //High Limit of battery Current (+Ibatt = Charge)
		int32_t			cfg_Ibatt_low;						 //High Limit of battery Current (-Ibatt = discharge)
		uint32_t		cfg_Ibatt_trickle_cut;               //Current at which to stop battery charging
		uint32_t		cfg_Vbatt_recharge;                  //Battery Voltage to re initiate battery charge
		uint32_t		cfg_Alt_unreg_min;                   //Min Voltage of alt_unreg
		uint32_t		cfg_Alt_unreg_max;                   //Max voltage of alt_unreg
		uint32_t		cfg_I_load_max;                      //Max user load for ms
};//pwrCfg
//forward deceleration of the command data FROM the controller POWER_CFG_COMMAND 0x41
extern struct pwrCfg gbl_PwrCfg;

//This stores LOGICAL STATE of Digital Inputs
struct dioInputType{
	uint8_t			uv_trip;				/*logical state of PA28 Under-voltage Trip (Set by PIN_PA28A_EIC_EXTINT8 in cntrl_EXTINT callback)*/
	uint8_t			sw_status;				/*logical state of PB00 SW Status*/
};//dioInputType
extern struct dioInputType  gbl_DigInputs;

//This stores DAC levels of DAC Outputs
struct DacOutputType{
	uint8_t			bat_i_set;				/*DAC level of Channel 3*/

};//AdcOutputType
extern struct DacOutputType  gbl_DacOutputs;

//This stores LOGICAL STATE of Digital Outputs
struct dioOutputType{
	uint8_t			bat_en;					/*logical state of PB30 Battery Enable Output*/
	uint8_t			bat_chrg_en;			/*logical state of PB31 Battery Charge En Output*/
	uint8_t			motor_dir;				/*logical state of PB23 Motor Direction Output*/
	uint8_t			output_en;				/*logical state of PA27 Output Enable*/
};//dioInputType
extern struct dioOutputType  gbl_DigOutputs;

//This structure is used to store values for the POWER_STATUS (0x31) data packet.
//This is sent back via CAN to CONTROLLER on a periodic basis.
struct configType {
	bool		monitor_mode;						/*load current, mA*/
};//pwrStatusMsgtype
//forward deceleration of global power status object POWER_STATUS 0x31
extern struct configType gbl_configType;

//forward function decelerations
void initGlobalsSafe(void);
void clearCANcommands(void);					// clears gbl CAN commands from memory

#endif /* CNTRL_GLOBALS_H_ */