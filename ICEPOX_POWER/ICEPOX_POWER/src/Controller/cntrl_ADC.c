/*
 * cntrl_ADC.c
 *
 * Created: 3/12/2017 1:10:06 PM
 *  Author: Gerald
 */ 

#include <asf.h>

#include "cntrl_ADC.h"

/*this is the results buffer*/
uint16_t adc_result_buffer[NUM_SAMPLES_ACQUIRE];

/*this is the adc module*/ 
struct adc_module adc_instance;

 /*-----------------------------------------------------------*/
 void configure_adc(void)
 {

	struct adc_config config_adc;

	/* initialize adc queue for keeping a log of last adc data reads*/
	adc_q_init(&adc_queue);

	/*get the defaults*/
	adc_get_config_defaults(&config_adc);

	/*this module uses GCLK_GENERATOR_0 with ADC_CLOCK_PRESCALER_DIV2;

	/*custom setup for this application*/
	/*THIS DOCUMENT IS ESSENTIAL -->>>*/
	/*see http://www.atmel.com/Images/Atmel-42451-SAM-Analog-to-Digital-Converter-ADC-Driver_ApplicationNote_AT11380.pdf*/
	config_adc.clock_prescaler      = ADC_CLOCK_PRESCALER_DIV16;	    /*possible clock prescaler values for the ADC*/
	config_adc.reference            = ADC_REFCTRL_REFSEL_INTVCC2;		/*according to 39.8.3. Reference Control, p938, set 0x5 -> VDDANA*/
	config_adc.positive_input       = ADC_POSITIVE_INPUT_PIN0;			/*this should be the first input in the scan!! In this case, AIN[0] on PA02*/
    config_adc.negative_input       = ADC_NEGATIVE_INPUT_GND;			/*ref is gnd*/
	config_adc.resolution			= ADC_RESOLUTION_CUSTOM;			/*16-bit derived from averaging*/
	config_adc.accumulate_samples   = ADC_ACCUMULATE_SAMPLES_512;
	config_adc.divide_result        = 512;								/*this must be the same as the accumulate value above*/

	//NOTE: This combination of Clock_Prescaler Accumulate values yields about 27.77 SPS (36 mS) sample rate.

	/*set pin sequence here ... */
	config_adc.positive_input_sequence_mask_enable = 0xFF;				/*POWER BOARD -> enable on AIN7 .. AIN0 -> 0b1111 1111  

	/*init the adc instance*/
	adc_init(&adc_instance, ADC0, &config_adc);							/**!!Make sure this is the right ADC!!*/
	
	/*init the sequence (bit mapping of ADC input pins)*/
	adc_enable_positive_input_sequence(&adc_instance,(uint32_t) config_adc.positive_input_sequence_mask_enable);
	
	/*enable the adc module*/
	adc_enable(&adc_instance);

	/*set system interrupt priority*/
	uint8_t int_enabled, int_priority;
	int_enabled = system_interrupt_is_enabled(SYSTEM_INTERRUPT_MODULE_ADC0);
	system_interrupt_set_priority(SYSTEM_INTERRUPT_MODULE_ADC0, ADC_SYSTEM_INTERRUPT_PRIORITY);
	int_priority =  system_interrupt_get_priority(SYSTEM_INTERRUPT_MODULE_ADC0);

	/*ensure interrupt priority set OK*/
	Assert(ADC_SYSTEM_INTERRUPT_PRIORITY==int_priority);

	/*start asynchronous ADC read*/
	/*ADC has already been configured in main prvSetupHardware() */
	adc_start_async_read();

 }//configure_adc

 /*-----------------------------------------------------------*/
 void configure_adc_callbacks(void)
 {
	 adc_register_callback(&adc_instance, adc_complete_callback, ADC_CALLBACK_READ_BUFFER);
	 adc_enable_callback(&adc_instance, ADC_CALLBACK_READ_BUFFER);
 }//configure_adc_callbacks

/*-----------------------------------------------------------*/
 void adc_start_async_read(void)
 {
	/*start asynchronous ADC read*/
 	adc_read_buffer_job(&adc_instance, adc_result_buffer, NUM_SAMPLES_ACQUIRE);
 }//adc_start_async_read

 /*-----------------------------------------------------------*/
 void adc_complete_callback( struct adc_module *const module)
 {
	 
	 uint32_t adc_status = adc_get_status(module);
	 
		 
	if ((adc_status & ADC_STATUS_RESULT_READY)) {
		 
		 //get data from ADC results buffer, and store in FIFO Queue
		 adc_q_data.cnt++;   /*never ending 32-bit counter*/
		 adc_q_data.adc_data.ain0_DCDCImon.cnts		=  adc_result_buffer[0];
		 adc_q_data.adc_data.ain1_DCDCVmon.cnts		=  adc_result_buffer[1];
		 adc_q_data.adc_data.ain2_ILoadMeas.cnts		=  adc_result_buffer[2];
		 adc_q_data.adc_data.ain3_IShortMeas.cnts	=  adc_result_buffer[3];
		 adc_q_data.adc_data.ain4_IBattery.cnts		=  adc_result_buffer[4];
		 adc_q_data.adc_data.ain5_VBattery.cnts		=  adc_result_buffer[5];
		 adc_q_data.adc_data.ain6_AltUnregVmon.cnts	=  adc_result_buffer[6];
		 adc_q_data.adc_data.ain7_Thermistor.cnts	=  adc_result_buffer[7];

		 /*load the adc q*/
		 adc_q_putItem(&adc_queue, adc_q_data);

		 /*store the latest sample in the global structure*/
		 /*these are all in raw counts, and will be scaled when used in the control loop*/
		 gbl_AnalogIn.ain0_DCDCImon					= adc_result_buffer[0];
		 gbl_AnalogIn.ain1_DCDCVmon					= adc_result_buffer[1];
		 gbl_AnalogIn.ain2_ILoadMeas				= adc_result_buffer[2];
		 gbl_AnalogIn.ain3_IShortMeas				= adc_result_buffer[3];
		 gbl_AnalogIn.ain4_IBattery					= adc_result_buffer[4];
		 gbl_AnalogIn.ain5_VBattery					= adc_result_buffer[5];
		 gbl_AnalogIn.ain6_AltUnregVmon				= adc_result_buffer[6];
		 gbl_AnalogIn.ain7_Thermistor				= adc_result_buffer[7];

		 /*debug debug debug debug debug*/
		 /*enable this to measure actual sample rate*/
		 LED_Toggle(LED_GREEN);
	 }
	 


	/*enable the adc module*/
	adc_disable(&adc_instance);

	/*enable the adc module*/
	adc_enable(&adc_instance);

	/*start asynchronous ADC read*/
	adc_read_buffer_job(&adc_instance, adc_result_buffer, NUM_SAMPLES_ACQUIRE);

 }//adc_complete_callback

/*-----------------------------------------------------------*/
//Scales thermistor to deg C, based on voltage divider and data sheet constants*/
//Uses the "Beta Model"
//See -->>>
/*https://datasheet.ciiva.com/2848/r44e-2848447.pdf*/
/*http://www.thinksrs.com/downloads/programs/Therm%20Calc/NTCCalibrator/NTCcalculator.htm*/
/*https://electronics.stackexchange.com/questions/8754/how-to-measure-temperature-using-a-ntc-thermistor*/
/*https://en.wikipedia.org/wiki/Thermistor*/
uint16_t scaleTherm(uint16_t VmeasMV){

	/*because of non-linear terms, this calculation is sensitive to numerical error*/
	/*be careful with casting and change of variable types*/
	double	VrefmV	= (double) THERM_VREF;
	double  R65		= (double) THERM_R65;
	double  R0		= (double) THERM_R0;
	double  B		= (double) THERM_BETA;
	double  T0		= (double) THERM_T0;

	double	Vmeas   = (double) VmeasMV / 1000.0;							//Measured voltage in Volts
	double  Vref	= (double) VrefmV / 1000.0;								//Reference voltage in volts
	double  Rtherm  = (double) ((Vmeas * R65) / (Vref - Vmeas));			//Solve voltage divider, and calculate Thermistor Resistance in Ohms
	double	Rinf    = (double) (R0 * exp(-1.0 * (B / T0)));					//Rinf term in Beta Equation (this is a const - could be re-written to only calculate once*/
	double  logterm = (double) log(Rtherm / Rinf);							//This is the ln term in the denominator of the Beta equation*/
	double  TempDegC= (double) ((B / logterm) - 273.0);						//Solve the Beta Equation, convert from K back to C

	return  (uint16_t) (TempDegC * 100.0);		//returns scaled deg C * 100

 }//uint_16 scaleTherm

/*-----------------------------------------------------------*/
//This function is used to test/debug scaling of ADC inputs
//The input counts can be manipulated and checked against reference spreadsheet*/
 void adcScalingDebugCheck(void){

 	printf("!!!!! ADC SCALING DEBUG CHECK !!!! \r\n");

 	printf("**PA02_DCDC_Imon -->\r\n");
 	uint16_t testcounts = 35535;
 	uint16_t testmV = ADC_RAW_TO_MV(testcounts);
 	printf("**ADC_RAW_TO_MV test --> %d cnts = %d mV \r\n", testcounts, testmV  );
 	printf("**DCDC_IMON_MV_TO_MA test --> %d mv = %d mA \r\n", testmV, DCDC_IMON_MV_TO_MA(testmV));

 	printf("**PA03_DCDC_Vmon -->\r\n");
 	testcounts = 59800;
 	testmV = ADC_RAW_TO_MV(testcounts);
 	printf("**ADC_RAW_TO_MV test --> %d cnts = %d mV \r\n", testcounts, testmV  );
 	printf("**DCDC_VMON_MV_TO_MV test --> %d mv = %d mV \r\n", testmV, DCDC_VMON_MV_TO_MV(testmV));

 	printf("**PB08_iLoad_Meas -->\r\n");
 	testcounts = 59800;
 	testmV = ADC_RAW_TO_MV(testcounts);
 	printf("**ADC_RAW_TO_MV test --> %d cnts = %d mV \r\n", testcounts, testmV  );
 	printf("**ILOAD_MV_TO_MA test --> %d mv = %d mA \r\n", testmV, ILOAD_MV_TO_MA(testmV));

 	printf("**PB09_iShortCircuit -->\r\n");
 	testcounts = 4095;
 	testmV = ADC_RAW_TO_MV(testcounts);
 	printf("**ADC_RAW_TO_MV test --> %d cnts = %d mV \r\n", testcounts, testmV  );
 	printf("**ISHORTCIR_MV_TO_MA test --> %d mv = %d mA \r\n", testmV, ISHORTCIR_MV_TO_MA(testmV));

 	printf("**PA04_Batt_Imon -->\r\n");
 	testcounts = 59800;
 	testmV = ADC_RAW_TO_MV(testcounts);
 	printf("**ADC_RAW_TO_MV test --> %d cnts = %d mV \r\n", testcounts, testmV  );
 	printf("**IBAT_MV_TO_MA test --> %d mv = %d mA \r\n", testmV, IBAT_MV_TO_MA(testmV));

 	printf("**PA05_Bat_Vmon -->\r\n");
 	testcounts = 65535;
 	testmV = ADC_RAW_TO_MV(testcounts);
 	printf("**ADC_RAW_TO_MV test --> %d cnts = %d mV \r\n", testcounts, testmV  );
 	printf("**BATV_VMON_MV_TO_MV test --> %d mv = %d mV \r\n", testmV, BATV_VMON_MV_TO_MV(testmV));

 	printf("**PA06_ALT_Unreg_Vmon -->\r\n");
 	testcounts = 55000;
 	testmV = ADC_RAW_TO_MV(testcounts);
 	printf("**ADC_RAW_TO_MV test --> %d cnts = %d mV \r\n", testcounts, testmV  );
 	printf("**ALTV_VMON_MV_TO_MV test --> %d mv = %d mV \r\n", testmV, ALTV_VMON_MV_TO_MV(testmV));

 	printf("**PA07_Termistor -->\r\n");
 	testcounts = 43672;
 	testmV = ADC_RAW_TO_MV(testcounts);
 	printf("**ADC_RAW_TO_MV test --> %d cnts = %d mV \r\n", testcounts, testmV  );
 	printf("**scaleTherm test --> %d mv = %d dec C (X100) \r\n", testmV, scaleTherm(testmV));

 }//adcScalingDebugCheck

 /*-----------------------------------------------------------*/
 void adc_q_init(adc_circularQueue_t *theQueue)
 {
	 uint8_t i;
	 theQueue->validItems  =  0;
	 theQueue->first       =  0;
	 theQueue->last        =  0;
	 for(i=0; i<MAX_ADC_QUEUE_ITEMS; i++)
	 {
		 theQueue->q_data[i].cnt = 0;
         theQueue->q_data[i].adc_data.ain0_DCDCImon.cnts		= 0;
		 theQueue->q_data[i].adc_data.ain1_DCDCVmon.cnts		= 0;
		 theQueue->q_data[i].adc_data.ain2_ILoadMeas.cnts		= 0;
		 theQueue->q_data[i].adc_data.ain3_IShortMeas.cnts		= 0;
		 theQueue->q_data[i].adc_data.ain4_IBattery.cnts		= 0;
		 theQueue->q_data[i].adc_data.ain5_VBattery.cnts		= 0;
		 theQueue->q_data[i].adc_data.ain6_AltUnregVmon.cnts	= 0;
		 theQueue->q_data[i].adc_data.ain7_Thermistor.cnts		= 0;

	 }
	 return;
 }//adc_q_init

 /*-----------------------------------------------------------*/
 uint8_t adc_q_isEmpty(adc_circularQueue_t *theQueue)
 {
	 if(theQueue->validItems==0)
	 return(1);
	 else
	 return(0);
 }//adc_q_isEmpty

 /*-----------------------------------------------------------*/
 uint8_t adc_q_putItem(adc_circularQueue_t *theQueue, adc_queue_data_t qItem)
 {
	 if(theQueue->validItems<MAX_ADC_QUEUE_ITEMS)
	 {
		 theQueue->validItems++;
	 }
	 theQueue->q_data[theQueue->last] = qItem;
	 theQueue->last = (theQueue->last+1) % MAX_ADC_QUEUE_ITEMS;

 }//adc_q_putItem

/*-----------------------------------------------------------*/
 uint8_t adc_q_getItem(adc_circularQueue_t *theQueue, adc_queue_data_t *qItem)
 {
	 if(adc_q_isEmpty(theQueue))
	 {
		 return(-1);
	 }
	 else
	 {
		 *qItem=theQueue->q_data[theQueue->first];
		 theQueue->first=(theQueue->first+1) % MAX_ADC_QUEUE_ITEMS;
		 theQueue->validItems--;
		 return(0);
	 }
 }//adc_q_getItem

/*--------------------------------------------------------------  */
/*this functions copies the most recent data from the queue, but */
/*does NOT remove from the buffer. This is what can be used to    */
/*copy data for processing in the control loop.				      */
 uint8_t adc_q_getLast(adc_circularQueue_t *theQueue, adc_queue_data_t *qItem)
 {
	 if(adc_q_isEmpty(theQueue))
	 {
		 return(-1);
	 }
	 else
	 {
		 *qItem=theQueue->q_data[theQueue->first];
		 return(0);
	 }
 }//adc_q_getLast


