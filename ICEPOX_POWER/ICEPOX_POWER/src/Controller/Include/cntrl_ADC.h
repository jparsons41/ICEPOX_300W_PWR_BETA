/*
 * cntrl_ADC.h
 *
 * Created: 3/12/2017 1:10:18 PM
 *  Author: Gerald
 */

#ifndef CNTRL_ADC_H_
#define CNTRL_ADC_H_

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

/*globals*/
#include "cntrl_GLOBALS.h"
#include "cntrl_CONTROL_LOOP.h"

/*this is the number of ADC samples to acquire per callback*/
#define NUM_SAMPLES_ACQUIRE			( 8 )

/*this is the number of ADC samples to store in the ring buffer JUST FOR DEBUGGING */
#define MAX_ADC_QUEUE_ITEMS			( 25 )

/*sets the ADC peripheral system interrupt priority 0 .. 3 (0 is highest)*/
#define ADC_SYSTEM_INTERRUPT_PRIORITY		( 3 )

/*ADC macros (be careful w/ casting!!)*/
#define ADC_REF_MV						( (double) 3265 )					           /*Vref ANA in mV*/  //measured 3325
#define ADC_BITS						( 16 )								           /*number of effective ADC bits*/
#define ADC_COUNTS						( (double) pow( 2, ADC_BITS)- 1.0)			   /*number of counts*/			//65535
#define	ADC_MVVOLTS_SCALE_FACTOR		( (double) ( ADC_REF_MV / ADC_COUNTS) )		   /*scale cnts to mV*/
#define ADC_RAW_TO_MV(x)				( (uint16_t) (x * ADC_MVVOLTS_SCALE_FACTOR) )  /*mv per count*/

/*calibrated ADC transfer */
#define ADC_REF_MV_OFFSET				( (double) 22.821 )			/*b in mx=b equation for count to mVolt into adc */
#define	ADC_MVVOLTS_SCALE				( (double) (.0503)	)	   /*y in mx=b equation for count to mVolt into adc */
#define ADC_RAW_TO_MV(x)				( (uint16_t) ((x * ADC_MVVOLTS_SCALE) + ADC_REF_MV_OFFSET) )  /*mv per count*/

/*PA02_DCDC_Imon*/
/*http://www.synqor.com/Datasheets/NQ60W60QTx25_Datasheet.pdf*/
#define DCDC_IMON_OFFSET_MV				( (double) 200 )			     /*mV offset specified in data sheet*/
#define DCDC_IMON_FULL_SCALE_MA			( (double) 25000 )				 /*mA full scale*/
#define DCDC_IMON_FULL_SCALE_MV			( (double) 2200 )				 /*mV full scale*/
#define DCDC_IMON_SENSITIVITY			(  DCDC_IMON_FULL_SCALE_MA / (DCDC_IMON_FULL_SCALE_MV - DCDC_IMON_OFFSET_MV) ) /*sensitivity in mA/mV*/

#define DCDC_IMON_MV_TO_MA(x)			( (uint16_t) (( (double) x - DCDC_IMON_OFFSET_MV) * DCDC_IMON_SENSITIVITY)  ) /*scaled mA*/

/*Calibrated count to scale iLoad_Measn*/
#define DCDC_IMON_DIV_MULTIPLIER		( (double) (.582) ) /*m for mx+b for count to Vmon curve*/
#define DCDC_IMON_DIV_OFFSET			( (double) (-2750) ) /*b for mx+b for count to Vmon curve*/
#define DCDC_IMON_COUNT_TO_MV(x)		( (uint16_t) ((x * DCDC_IMON_DIV_MULTIPLIER) + DCDC_IMON_DIV_OFFSET))     /*output bat vmon in mV*/


/*PA03_DCDC_Vmon*/
/*http://www.synqor.com/Datasheets/NQ60W60QTx25_Datasheet.pdf*/
#define DCDC_R14						( (double) 47000 )
#define DCDC_R17						( (double) 5600 )
#define DCDC_VMON_DIV					( (double) (DCDC_R17 / (DCDC_R17 + DCDC_R14)))
#define DCDC_VMON_MV_TO_MV(x)			( (uint16_t) ( (double) x * (3300/65535) / DCDC_VMON_DIV))     /*converts ADC mv to 28V bus monitor volts, about factor of 9.4*/

/*Calibrated count to scaleDCDC_Vmon*/
#define DCDC_VMON_DIV_MULTIPLIER		( (double) (.431) ) /*m for mx+b for count to Vmon curve*/
#define DCDC_VMON_DIV_OFFSET			( (double) (0) ) /*b for mx+b for count to Vmon curve*/
#define DCDC_VMON_COUNT_TO_MV(x)		( (uint16_t) ((x * DCDC_VMON_DIV_MULTIPLIER) + DCDC_VMON_DIV_OFFSET))     /*output bat vmon in mV*/



/*PB08_iLoad_Meas */
/*http://www.allegromicro.com/~/media/Files/Datasheets/ACS722-Datasheet.ashx*/
#define ILOAD_OFFSET_MV					( (double) 330 )					 /*mV offset specified in data sheet*/
#define ILOAD_SENSITIVITY				( (double) 132 )					 /*sensitivity mv/A*/
#define ILOAD_MV_TO_MA(x)				( (uint16_t) (( (double) x - ILOAD_OFFSET_MV) / ILOAD_SENSITIVITY * 1000.0) ) /*Iload in mA*/

/*Calibrated count to scale iLoad_Measn*/
#define ILOAD_DIV_MULTIPLIER		( (double) (.724) ) /*m for mx+b for count to Vmon curve*/
#define ILOAD_DIV_OFFSET			( (double) (-26050) ) /*b for mx+b for count to Vmon curve*/
#define ILOAD_COUNT_TO_MA(x)		( (int16_t) ((x * ILOAD_DIV_MULTIPLIER) + ILOAD_DIV_OFFSET))     /*output bat vmon in mV*/




/*PB09_iShortCircuit*/
/*http://www.infineon.com/dgdl/Infineon-BTS50055-1TMC-DS-v01_00-EN.pdf?fileId=5546d4625a888733015aa9b0007235e9*/
#define KILIS							( (double) 14200 )				/*data sheet Iload->Isource sensitivity (does change w/ temperature)*/
#define RIS								( (double) 768 )					/*series resistance*/
#define ISHORTCIR_MV_TO_MA(x)			( (int16_t) (( (double) x  / RIS) * KILIS) )  /*calculated Is in mA*/





///*PA04_Batt_Imon*/
///*http://www.allegromicro.com/~/media/Files/Datasheets/ACS722-Datasheet.ashx*/
#define IBAT_OFFSET_MV					( (double) 1620 )					 /*mV offset specified in data sheet*/
#define IBAT_SENSITIVITY				( (double) -66 )					 /*sensitivity mv/A*/
#define IBAT_MV_TO_MA(x)				( (uint16_t) ( ( ( (double) x - IBAT_OFFSET_MV) / IBAT_SENSITIVITY ) * 1000 )) /*IBat in mA*/


/*Calibrated count to scale iLoad_Measn*/
#define IBAT_DIV_MULTIPLIER		( (double) (-.7088) ) /*m for mx+b for count to Vmon curve*/
#define IBAT_DIV_OFFSET			( (double) (25550) ) /*b for mx+b for count to Vmon curve*/
#define IBAT_COUNT_TO_MV(x)		( (int16_t) ((x * IBAT_DIV_MULTIPLIER) + IBAT_DIV_OFFSET))     /*output bat vmon in mV*/




/*PA05_Bat_Vmon*/
#define BATV_R45						( (double) 48000 )
#define BATV_R46						( (double) 10000 )
#define BATV_VMON_DIV					( (double) (BATV_R46 / (BATV_R46 + BATV_R45))) /*about 0.1724*/
#define BATV_VMON_MV_TO_MV(x)			( (uint16_t) ( (double) x / BATV_VMON_DIV))     /*output bat vmon in mV*/

/* count to scale Bat_Vmon*/
#define BATV_VMON_DIV_MULTIPLIER		( (double) (.259) ) /*m for mx+b for count to Vmon curve*/
#define BATV_VMON_DIV_OFFSET			( (double) (102) ) /*b for mx+b for count to Vmon curve*/
#define BATV_VMON_COUNT_TO_MV(x)		( (uint16_t) ((x * BATV_VMON_DIV_MULTIPLIER) + BATV_VMON_DIV_OFFSET))     /*output bat vmon in mV*/


/*PA06_ALT_Unreg_Vmon*/
#define ALTV_R59						( (double) 47000 )
#define ALTV_R60						( (double) 2700 )
#define ALTV_VMON_DIV					( (double) (ALTV_R60 / (ALTV_R60 + ALTV_R59))) /*about 0.0543*/
#define ALTV_VMON_MV_TO_MV(x)			( (uint16_t) ( (double) x * (3300/65535) / ALTV_VMON_DIV))     /*output v_unreg in mV*/

/*Calibrated count to scale ALT_Unreg_Vmon*/
#define ALTV_VMON_DIV_MULTIPLIER		( (double) (0.855) ) /*m for mx+b for count to Vmon curve*/
#define ALTV_VMON_DIV_OFFSET			( (double) (0) ) /*b for mx+b for count to Vmon curve*/
#define ALTV_VMON_COUNT_TO_MV(x)		( (uint16_t) ((x * ALTV_VMON_DIV_MULTIPLIER) + ALTV_VMON_DIV_OFFSET))     /*output bat vmon in mV*/


/*PA07_Thermistor*/
/*https://datasheet.ciiva.com/2848/r44e-2848447.pdf*/
/*http://www.thinksrs.com/downloads/programs/Therm%20Calc/NTCCalibrator/NTCcalculator.htm*/
/*https://electronics.stackexchange.com/questions/8754/how-to-measure-temperature-using-a-ntc-thermistor*/
/*https://en.wikipedia.org/wiki/Thermistor*/
#define THERM_R65						( 4700  ) /* voltage divider reference resistor */
#define THERM_BETA						( 3380  ) /* beta constant from data sheet*/
#define THERM_R0						( 10000 ) /* from data sheet @ 25 deg C */
#define THERM_T0						( 298   ) /* ref temp 25 deg C + 273 in K */
#define THERM_VREF						( 3300  ) /* Vref */
#define THERM_MV_TO_DEGC(x)				( scaleTherm(x) ) /*Implements Beta scaling method*/
/*for readability and future optimization, this scaling won't be done in a macro*/
/*see double scaleTherm(uint16_t VmeasMV)*/

#define ADC_SCALING_DEBUG				( 0 )	  /*flag to show scaling information on power-up*/

/*Stores information specific to each message ID defined above*/
/*up to four 16-bit data values in each message */
typedef struct adc_data_s {
	uint16_t cnts;				/*16-bit raw adc data*/
} adc_data_t;

/* This is the structure for adc data for the power board */
typedef struct powerbrd_adc_data_s{
	adc_data_t ain0_DCDCImon;			/*PA02 DC/DC I monitor*/
	adc_data_t ain1_DCDCVmon;			/*PA03 DC/DC C monitor*/
	adc_data_t ain2_ILoadMeas;			/*PB08 Output Current Measurement (hall sensor)*/
	adc_data_t ain3_IShortMeas;			/*PB09 Short Circuit Current Measurement*/
	adc_data_t ain4_IBattery;			/*PA04 Battery Current Measurement*/
	adc_data_t ain5_VBattery;			/*PA05 Battery Voltage Measurement*/
	adc_data_t ain6_AltUnregVmon;		/*PA06 Alternator Unregulated Voltage Measurement*/
	adc_data_t ain7_Thermistor;			/*PA07 Thermistor*/
} powerbrd_adc_data_t;

/*data type for data queue to store can rx data*/
typedef struct adc_queue_data_s {
	uint32_t				  cnt;			/*a free running counter for debug, increments each ADC packet*/
	powerbrd_adc_data_t       adc_data;	    /*data structure holding ADC data for each channel*/
} adc_queue_data_t;

/*ADC Data Storage Queue (FIFO)*/
/*this queue structure implements a circular ring buffer (FIFO) ADC measurements between crank pulses */
typedef struct adc_circularQueue_s
{
	uint8_t     first;									/*this is tracking the head (most recent) */
	uint8_t     last;									/*this is tracking the tail (oldest) */
	uint8_t     validItems;								/*this is the number of entries in the queue */
	adc_queue_data_t   q_data[MAX_ADC_QUEUE_ITEMS];		/*this is the data itself*/
} adc_circularQueue_t;

/*variable that holds the ADC queue*/
adc_circularQueue_t adc_queue;			/*this is the queue for storing ADC data records*/
adc_queue_data_t    adc_q_data;		    /*this is the structure of the data itself */

/*forward declarations for ADC functions*/
void configure_adc(void);
void configure_adc_callbacks(void);
void adc_complete_callback(struct adc_module *const module);
void adc_start_async_read();

/*function to return temperature based on measured mV in voltage divider*/
/*see constants defined above. Requires inclusion of math.h*/
uint16_t scaleTherm(uint16_t VmeasMV);

/*Used to print out debug for ADC scaling equations*/
/*Not normally used during operation*/
void adcScalingDebugCheck(void);

/*forward declarations for ADC queue*/
void    adc_q_init   (adc_circularQueue_t *theQueue);							/*inits the queue structure*/
uint8_t adc_q_isEmpty(adc_circularQueue_t *theQueue);							/*checks if empty*/
uint8_t adc_q_putItem(adc_circularQueue_t *theQueue, adc_queue_data_t qItem);   /*puts newest item into  queue*/
uint8_t adc_q_getItem(adc_circularQueue_t *theQueue, adc_queue_data_t *qItem);  /*gets oldest from queue*/
uint8_t adc_q_getLast(adc_circularQueue_t *theQueue, adc_queue_data_t *qItem);  /*peeks at newest in queue*/

#endif /* CNTRL_ADC_H_ */