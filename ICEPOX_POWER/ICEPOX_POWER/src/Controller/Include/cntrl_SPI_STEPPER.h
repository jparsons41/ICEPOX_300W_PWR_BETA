/*
 * cntrl_SPI_STEPPER.h
 *
 * Created: 5/7/2017 10:24:29 PM
 *  Author: Gerald
 */ 


#ifndef CNTRL_SPI_STEPPER_H_
#define CNTRL_SPI_STEPPER_H_

#include <asf.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"

/*globals*/
 #include "cntrl_GLOBALS.h"

 #define STEPPER_DEBUG_PRINT				1		/*turns on verbose printing of debug messages*/

 #define isequal(X, Y)  ((X) == (Y) ? (1) : (0))

//REMINDER - 1 TICK = 250 nSec (4 MHZ)
//MOTOR PARAMETER REGISTERS AND FLAGS
//http://www.st.com/content/ccc/resource/technical/document/datasheet/a5/86/06/1c/fa/b2/43/db/CD00255075.pdf/files/CD00255075.pdf/jcr:content/translations/en.CD00255075.pdf
//See Data Sheet Table 9. Register Map, Page 40
//Important: Note that these registers have different lengths, from 4 to 22 bits!!!
//All commands are formed as a single byte. Arguments may be needed. These can be 1 to 3 bytes
    										//NUM			//LEN BITS			//RESET VALUE (HEX)
#define MOTOR_PARA_ABS_POS					0x01			//22				000000
#define MOTOR_PARA_EL_POS					0x02			//9					000
#define MOTOR_PARA_MARK						0x03			//22				000000
#define MOTOR_PARA_SPEED					0x04			//20				00000
#define MOTOR_PARA_ACC						0x05			//12				08A
#define MOTOR_PARA_DEC						0x06			//12				08A
#define	MOTOR_PARA_MAX_SPEED				0x07			//10				041
#define MOTOR_PARA_MIN_SPEED				0x08			//13				000
#define MOTOR_PARA_FS_SPEED					0x15			//10				027
#define MOTOR_PARA_KVAL_HOLD				0x09			//8					29
#define MOTOR_PARA_KVAL_RUN					0x0A			//8					29
#define MOTOR_PARA_KVAL_ACC					0x0B			//8					29
#define	MOTOR_PARA_KVAL_DEC					0x0C			//8					29
#define MOTOR_PARA_INT_SPEED				0x0D			//14				0408
#define MOTOR_PARA_ST_SLP					0x0E			//8					19
#define MOTOR_PARA_FN_SLP_ACC				0x0F			//8					29
#define MOTOR_PARA_FN_SLP_DEC				0x10			//8					29
#define MOTOR_PARA_K_THERM					0x11			//4					0
#define MOTOR_PARA_ADC_OUT					0x12			//5					NOTUSED
#define MOTOR_PARA_OCD_TH					0x13			//4					8
#define MOTOR_PARA_STALL_TH					0x14			//7					40
#define MOTOR_PARA_STEP_MODE				0x16			//8					7
#define MOTOR_PARA_ALARM_EN					0x17			//8					FF
#define MOTOR_PARA_CONFIG					0x18			//18				2E88
#define MOTOR_PARA_STATUS					0x19			//16				XXXX (according to status conditions)

//DEFAULT VALUEs (HEX)
#define MOTOR_PARA_ABS_POS_DEF_VALUE		0x0					
#define MOTOR_PARA_EL_POS_DEF_VALUE			0x0			
#define MOTOR_PARA_MARK_DEF_VALUE			0x0				
#define MOTOR_PARA_SPEED_DEF_VALUE			0x0		
#define MOTOR_PARA_ACC_DEF_VALUE			0x8A					
#define MOTOR_PARA_DEC_DEF_VALUE			0x8A				
#define	MOTOR_PARA_MAX_SPEED_DEF_VALUE		0x41			
#define MOTOR_PARA_MIN_SPEED_DEF_VALUE		0x0		
#define MOTOR_PARA_FS_SPEED_DEF_VALUE		0x27				
#define MOTOR_PARA_KVAL_HOLD_DEF_VALUE		0x29			
#define MOTOR_PARA_KVAL_RUN_DEF_VALUE		0x29					
#define MOTOR_PARA_KVAL_ACC_DEF_VALUE		0x29				
#define	MOTOR_PARA_KVAL_DEC_DEF_VALUE		0x29					
#define MOTOR_PARA_INT_SPEED_DEF_VALUE		0x408				
#define MOTOR_PARA_ST_SLP_DEF_VALUE			0x19			
#define MOTOR_PARA_FN_SLP_ACC_DEF_VALUE		0x29			
#define MOTOR_PARA_FN_SLP_DEC_DEF_VALUE		0x29			
#define MOTOR_PARA_K_THERM_DEF_VALUE		0x0				
#define MOTOR_PARA_ADC_OUT_DEF_VALUE		0x99		/*XX (see note 2)*/			
#define MOTOR_PARA_OCD_TH_DEF_VALUE			0x8			
#define MOTOR_PARA_STALL_TH_DEF_VALUE		0x40			
#define MOTOR_PARA_STEP_MODE_DEF_VALUE		0x7		
#define MOTOR_PARA_ALARM_EN_DEF_VALUE		0xFF				
#define MOTOR_PARA_CONFIG_DEF_VALUE			0x2E88			
#define MOTOR_PARA_STATUS_DEF_VALUE			0x9999		/*XXXX (see note 2)*/	

#define MOTOR_PARA_ABS_POS_STR				"ABS_POS"
#define MOTOR_PARA_EL_POS_STR				"EL_POS"
#define MOTOR_PARA_MARK_STR					"MARK"
#define MOTOR_PARA_SPEED_STR				"SPEED"
#define MOTOR_PARA_ACC_STR					"ACC"
#define MOTOR_PARA_DEC_STR					"DEC"
#define	MOTOR_PARA_MAX_SPEED_STR			"MAX_SPEED"
#define MOTOR_PARA_MIN_SPEED_STR			"MIN_SPEED"
#define MOTOR_PARA_FS_SPEED_STR				"FS_SPD"
#define MOTOR_PARA_KVAL_HOLD_STR			"KVAL_HOLD"
#define MOTOR_PARA_KVAL_RUN_STR				"KVAL_RUN"
#define MOTOR_PARA_KVAL_ACC_STR				"KVAL_ACC"
#define	MOTOR_PARA_KVAL_DEC_STR				"KVAL_DEC"
#define MOTOR_PARA_INT_SPEED_STR			"INT_SPEED"
#define MOTOR_PARA_ST_SLP_STR				"ST_SLP"
#define MOTOR_PARA_FN_SLP_ACC_STR			"FN_SLP_ACC"
#define MOTOR_PARA_FN_SLP_DEC_STR			"FN_SLP_DEC"
#define MOTOR_PARA_K_THERM_STR				"K_THERM"
#define MOTOR_PARA_ADC_OUT_STR				"ADC_OUT"
#define MOTOR_PARA_OCD_TH_STR				"OCD_TH"
#define MOTOR_PARA_STALL_TH_STR				"STALL_TH"
#define MOTOR_PARA_STEP_MODE_STR			"STEP_MODE"
#define MOTOR_PARA_ALARM_EN_STR				"ALARM"
#define MOTOR_PARA_CONFIG_STR				"CONFIG"
#define MOTOR_PARA_STATUS_STR				"STATUS"

//MOTOR APPLICATION COMMANDS (see Table 37, Sec. 9.2)
#define MOTOR_CMD_SET_PARA					0b00000000		/*this gets OR'd w/ the PARAM bits 4..0 to be written to*/
#define MOTOR_CMD_GET_PARA					0b00100000      /*this gets OR'd w/ the PARAM bits 4..0 to be read from*/
#define MOTOR_CMD_STATUS					0b11010000
#define MOTOR_CMD_RESET						0b11000000
#define MOTOR_CMD_RESET_ABS_POS				0b11011000	    /*sets the HOME position*/
#define MOTOR_CMD_SOFT_STOP  				0b10110000
#define MOTOR_CMD_HARD_STOP  				0b10111000
#define MOTOR_CMD_RUN					    0b01010000		/*last bit will be used to set direction*/
#define MOTOR_CMD_MOVE					    0b01000000	    /*last bit will be used to set direction*/
#define MOTOR_CMD_GOTO						0b01100000		
#define MOTOR_CMD_GOTO_DIR					0b01101000		/*last bit will be used to set direction*/

//STATUS BITS
#define MOTOR_STATUS_SCK_MOD				0x8000			//in step-clock mode
#define MOTOR_STATUS_STEP_LOSS_B			0x4000			//stall detected on bridge b
#define MOTOR_STATUS_STEP_LOSS_A			0x2000			//stall detected on bridge a
#define MOTOR_STATUS_OCD					0x1000			//over current detection
#define MOTOR_STATUS_TH_SD					0x800			//thermal shutdown
#define MOTOR_STATUS_TH_WRN					0x400			//thermal warning
#define MOTOR_STATUS_UVLO					0x200			//under voltage lockout
#define MOTOR_STATUS_WRONG_CMD				0x100			//wrong command, doesn't exist
#define NOTPERF_CMD							0x80			//wrong command, cannot perform now
#define MOTOR_STATUS_MOT_STATUS				0x60			//stopped 0b00, acceleration 0b01, deceleration 0b10, constant speed 0b11
#define MOTOR_STATUS_DIR					0x10			//direction 1-forward, 0-reverse
#define	MOTOR_STATUS_SW_EVN					0x8				//switch turn-on event
#define	MOTOR_STATUS_SW_F					0x4				//switch input status
#define MOTOR_STATUS_BUSY					0x2				//busy if at constant speed, positioning, or command under execution
#define	MOTOR_STATUS_HIZ					0x1				//bridges are in high-impedance state

#define MOTOR_CLOSE							1				//direction of throttle close
#define MOTOR_OPEN							0				//direction of throttle open


#define MOTOR_HOMEOFFSET					3300			//offset from stall in the closed position - just at beginning of plate opening
#define MOTOR_WOT_CNTS						11520			//wide open throttle counts, from home position

//this macro calculates the WOT increment, based on an 8-bit scaled command (0..255 corresponding to 0..100 % WOT)
#define MOTOR_WOT_INCREMENT		MOTOR_WOT_CNTS / 255			
#define MOTOR_HOME				0				//home count value

#define MOTOR_TORQUE_MIN		32				//sets torque to 50% Vs
#define MOTOR_TORQUE_MAX		64				//sets torque to 100% Vs

#define MOTOR_STOP_HARD			1
#define MOTOR_STOP_SOFT			0

/* FreeRTOS message queue for SPI Stepper control*/
xQueueHandle xSPIStepperQHndle;

/*enumeration of motor commands to be initiated via freeRTOS messaging*/
enum mot_cmd_type {RESET, MOVE_THROT_POS, HOME};

/* structure for SPI Stepper commands to be placed in freeRTOS queue */
typedef struct {
	enum mot_cmd_type uxCmdID; /*see enumeration above*/
	uint16_t     uxThrotPos; /*8-bit, 0..255 -> 0.. 100 % WOT*/
} SPI_Stepper_cmd_t;

#define BUF_LENGTH 20

#define SLAVE_SELECT_PIN CONF_MASTER_SS_PIN

#define SPI_DELAY_CYCLES 50

/*define length of the command queue in xTaskSPIStepper */
#define STEPPER_RTOS_CMD_QUEUE_LEN		10

/*Tics to wait for CMD QUEUE message in xTaskSPIStepper*/
#define MOTOR_CMD_TICS_TO_WAIT			500	

/*SPI data*/
struct spi_module spi_master_instance;
struct spi_slave_inst slave;

/*this is the SPI return code*/
enum status_code ret_spi_status;

/*forward decelerations*/
void xTaskSPIStepper(void *pvParameters);

/*Motor Init / Status*/
void              configure_SPI_master (void);	
uint16_t          motSendCmdByte(uint16_t cmd);
int32_t           motGetParameter (uint8_t cmdtype, uint16_t param);
uint16_t          motSetParameter (uint16_t parameter, uint16_t value, uint8_t lenbytes);
uint16_t          motRun (uint8_t dir, uint32_t speed);
uint16_t		  motStop (uint8_t hard_or_soft);
uint16_t		  motReset (void);
uint16_t          motMove (uint8_t dir, uint32_t num_steps);
uint32_t		  motGoTo (int32_t abs_pos);
uint16_t          motResetAbsPos (void);
int32_t           motGetABS_POS (void);
uint16_t	      motSetTorques (uint8_t mot_torque_setpoint);
int32_t           motGetStatusGlobal (void);
uint8_t           motGetAllParameters(void);
uint8_t           motMoveHome(uint16_t home_offset);
uint32_t          motMoveThrottle(uint8_t throttle_pos);


#endif /* CNTRL_SPI_STEPPER_H_ */