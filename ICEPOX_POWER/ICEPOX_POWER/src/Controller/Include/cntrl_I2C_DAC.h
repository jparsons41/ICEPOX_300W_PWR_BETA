/*
 * cntrl_I2C_DAC.h
 *
 * Created: 3/21/2017 12:16:37 PM
 *  Author: Gerald
 */ 


#ifndef CNTRL_I2C_DAC_H_
#define CNTRL_I2C_DAC_H_

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

/* basic defines for the Microchip  MCP4728 */
/* see http://ww1.microchip.com/downloads/en/DeviceDoc/22187E.pdf */
#define DAC_MCP4728_DEVICE_ID					0b1100     //predefined MCP4728 device id
#define DAC_MCP4728_DEVICE_ADDR					0b0000	   //default device address for A2,A1,A0;
#define DAC_MCP4728_CMD_WRITE_VREF				0b1000	   //write Vref Command C0=0, C1=0, C2=1;
#define DAC_MCP4728_CMD_WRITE_GAIN_SELECT		0b1100     //write gain Command C0=0, C1=1, C2=1;
#define DAC_MCP4728_CMD_WRITE_SINGLE			0b1011     //single write command
#define DAC_MCP4728_CMD_WRITE_FAST				0b0000     //fast write command C2=0, C1=0, PD0=0, PD0=0

/* MCP4728 Channel Defines */
#define I2C_DAC_CH_A		1
#define I2C_DAC_CH_B		2
#define I2C_DAC_CH_C		3
#define I2C_DAC_CH_D		4

/* DAC constants for this setup, all channels configured identically*/
#define DAC_VREF_MV			( (double) 2048 )		/*internal precision reference voltage in mV*/
#define DAC_COUNTS			( (double) 4096 )		/*full scale counts*/

/*VOUT C - BAT I SET*/
/*Range is 0 to 1500 mV*/
/*0 sets DC/DC to 0.0 A*/
/*1500 mV sets DC/DC to max current */
/*scale such that DAC output 2048 BmV -> DC/DC setpoint of 1500 V*/
#define BATT_I_SET_R31						( (double) 3650 )
#define BATT_I_SET_R33						( (double) 10000 )
#define BATT_I_SET_DIV					    ( (double) (BATT_I_SET_R33 / (BATT_I_SET_R33 + BATT_I_SET_R31)))
#define BATT_I_SET_DAC_MV_TO_CNTS(x)			( (uint16_t) ((( (double) x / BATT_I_SET_DIV) * DAC_COUNTS )  / DAC_VREF_MV))

/*VOUT B - DC/DC SET*/
/*Iset= 2.180V sets DC/DC to 25A limit*/
/*Amplifier gain = 1+R1/R2 , 1+649/10K = 1.065 */
#define ISET_R34						( (double) 649 )
#define ISET_R35						( (double) 10000 )
#define ISET_DIV					    ( (double) (1 + (ISET_R34 / ISET_R35)) )
#define ISET_MAX_I						( 25000 )
#define ISET_DAC_MA_TO_CNTS(x)			( (uint16_t) (( (double) x / ISET_MAX_I) * DAC_COUNTS ) )
//#define ISET_DAC_MV_TO_CNTS(x)			( (uint16_t) ((( (double) x / ISET_DIV) * DAC_COUNTS )  / DAC_VREF_MV))

/*VOUT A - VSET DAC */
/*NOTE: This need to be tested against the attenuation circuit !!!*/

//This enables the power-up printout of the scaling check
#define DAC_SCALING_DEBUG				( 0 )	  /*flag to show scaling information on power-up*/

//This is a limiter macro to ensure DAC values are within range
#define DAC_OUT_LIMITER(x) \
 ( \
    ( \
        (x) > (4095) ? 4095 : \
        (x) < (1) ?    0 : \
        (x)\
    ) \
  )

/* deceleration of a basic I2C_DAC packet */
struct i2c_master_packet packet;

/* FreeRTOS message queue for I2C_DAC control*/
xQueueHandle xI2CDacQHndle;

/* structure for I2C_DAC commands to be placed in queue */
typedef struct {
	UBaseType_t  uxChId;
	UBaseType_t  uxCounts;
} I2CDac_cmd_t;

/*define length of the command queue for the I2C_DAC task */
#define I2C_DAC_RTOS_CMD_QUEUE_LEN		10

/*forward decelerations*/
void    configure_i2c_master   (void);
uint8_t writeVref_DAC          (uint8_t);
uint8_t writeGainSelect_DAC    (uint8_t);
uint8_t writeSingleI2C_DAC     (uint8_t, uint16_t);
uint8_t writeFastI2C_DAC       (uint16_t *);
void    xTaskI2CDac            (void *);		/*this is the FreeRTOS task*/

/*this function is used to sainity check DAC scaling */
void dacScalingDebugCheck(void);

#define MAX_DATA_LENGTH					10
static uint8_t write_buffer[MAX_DATA_LENGTH];

/* Initialize software module. */
struct i2c_master_module i2c_master_instance;

#endif /* CNTRL_I2C_DAC_H_ */