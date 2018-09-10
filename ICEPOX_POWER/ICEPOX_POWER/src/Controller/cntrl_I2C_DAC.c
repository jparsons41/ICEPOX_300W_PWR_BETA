/*
 * cntrl_I2C_DAC.c
 *
 * Created: 3/21/2017 12:16:07 PM
 *  Author: Gerald
 */ 

 #include "cntrl_I2C_DAC.h"

/////////////////////////////////////////////////////////////
/// MCP4728 I2C DIGITAL TO ANALOG CONVERTOR
/////////////////////////////////////////////////////////////

/*-----------------------------------------------------------*/
//! [initialize_i2c]
void configure_i2c_master (void)
{
	/* Initialize config structure and software module. */
	//! [init_conf]
	struct i2c_master_config config_i2c_master;
	i2c_master_get_config_defaults(&config_i2c_master);
	//! [init_conf]

	/* Change buffer timeout to something longer. */
	//! [conf_change]
	config_i2c_master.buffer_timeout = 10000;
	config_i2c_master.baud_rate		 = I2C_MASTER_BAUD_RATE_100KHZ; /*this is baud rate in kHz*/
	config_i2c_master.pinmux_pad0    = PINMUX_PA12C_SERCOM2_PAD0;   /** PA12 (SDA) pinmux */
	config_i2c_master.pinmux_pad1    = PINMUX_PA13C_SERCOM2_PAD1;   /** PA13 (SCL) pinmux */

	//! [conf_change]
	/* Initialize and enable device with config. */
	//! [init_module] CONF_I2C_MASTER_MODULE points to SERCOM2
	i2c_master_init(&i2c_master_instance, CONF_I2C_MASTER_MODULE, &config_i2c_master);
	//! [init_module]

	//! [enable_module]
	i2c_master_enable(&i2c_master_instance);
	//! [enable_module]

	/* create the queue for I2C_DAC message, this has to be the size of the command typedef I2CDac_cmd_t */
	xI2CDacQHndle = xQueueCreate(I2C_DAC_RTOS_CMD_QUEUE_LEN , sizeof(I2CDac_cmd_t));

	/* setup Vref and Gain configuration*/
	writeVref_DAC(0b0000);		/*set Vref=2.048 (internal, high precision) on all four channels*/
	delay_ms(50);

	writeGainSelect_DAC(0b0000); /*set gain = 1 on all four channels*/
	delay_ms(50);

	///* initialize DAC Channels*/
////	writeSingleI2C_DAC(I2C_DAC_CH_A, 2048);    //default to 2.048V out which sets DCDC out to ~28V
	//writeSingleI2C_DAC(I2C_DAC_CH_A, 2048);    //default to 2.048V out which sets DCDC out to ~28V
	//delay_ms(50);
	//writeSingleI2C_DAC(I2C_DAC_CH_B, 2000);    //default to 2.00V out which sets DCDC Iset to 25A
	//delay_ms(50);
	//writeSingleI2C_DAC(I2C_DAC_CH_C, 0);
	//delay_ms(50);
	//writeSingleI2C_DAC(I2C_DAC_CH_D, 0);
	//delay_ms(50);

}//configure_i2c_master

/*-----------------------------------------------------------*/
// function  writeVref_DAC
// This function transmits via I2C1 the Voltage reference setup bits (Vref) to the selected DAC
uint8_t writeVref_DAC(uint8_t vrefSelect) {

	 uint8_t retVal=0;								//return value
	 uint8_t data1,data2;							//data words to be constructed for single write command

	 data1 = ((DAC_MCP4728_DEVICE_ID<<3) | (DAC_MCP4728_DEVICE_ADDR));		//device id (4) + slave addr (3)
	 data2 = ((DAC_MCP4728_CMD_WRITE_VREF<<4)  | (vrefSelect));		//cmd bits (4) + vref configuration (4)

	 write_buffer[0] = data2;

	 struct i2c_master_packet packet = {
		 .address         = data1,
		 .data_length     = 1,
		 .data            = write_buffer,
		 .ten_bit_address = false,
		 .high_speed      = false,
		 .hs_master_code  = 0x0,
	 };

	 retVal = i2c_master_write_packet_wait(&i2c_master_instance, &packet);

	 return retVal;

}//writeVref_DAC

/*-----------------------------------------------------------*/
// function  writeGainSelect_DAC
// This function transmits via I2C1 the Gain Select Bits (Gx) to the DAC register
// Using gain, the output will be proportional Vref or VDD based on writeVref_DAC setting
uint8_t writeGainSelect_DAC(uint8_t gainSelect) {

	 uint8_t retVal=0;							//return value
	 uint8_t data1,data2;						//data words to be constructed for single write command

	 data1 = ((DAC_MCP4728_DEVICE_ID<<3) | (DAC_MCP4728_DEVICE_ADDR));		//device id (4) + slave addr (3)
	 data2 = ((DAC_MCP4728_CMD_WRITE_GAIN_SELECT<<4)  | (gainSelect));		//cmd bits (4) + gain select (4)
	  
	 write_buffer[0] = data2;

	 struct i2c_master_packet packet = {
		 .address         = data1,
		 .data_length     = 1,
		 .data            = write_buffer,
		 .ten_bit_address = false,
		 .high_speed      = false,
		 .hs_master_code  = 0x0,
	 };

	 retVal = i2c_master_write_packet_wait(&i2c_master_instance, &packet);

	 return retVal;

}//writeGainSelect_DAC

/*-----------------------------------------------------------*/
// function  writeSingleI2C_DAC
// This function transmits via I2C1 a single write command to the selected DAC chip / selected channel
// This routine will use the "single write Command" to write a single channel voltage to a single DAC
// This voltage is retained in EEPROM memory of the device.
// In this scenario, use Gain = x2, Internal Reference (2.048V), and No power down mode
// Therefore max VDC out = 4.096V = 4096 count
// Refer to MCP4728 datasheet, page 41, Figure 5-10 for a helpful diagram
uint8_t writeSingleI2C_DAC(uint8_t chSelect, uint16_t adcCounts)
 {

	 uint8_t retVal=0;								//return value
	 uint8_t data1,data2,data3,data4;				//data words to be constructed for single write command
	 uint8_t chID;						    		//this will be the channel id: 00-A, 01-B, 10-C, 11-D
	 uint8_t setup				=	0b1000;			//this upper 4 bits of the 3rd byte

	 chID = chSelect - 1;
	 
	 //construct data words to transmit in the single write command
	 data1 = ((DAC_MCP4728_DEVICE_ID<<3) | (DAC_MCP4728_DEVICE_ADDR));		//device id (4) + slave addr (3)
	 data2 = ((DAC_MCP4728_CMD_WRITE_SINGLE<<3) | (chID<<1));			    //cmd bits (3) + write fun (2) + ch select (2) + nUDAC (1)
	 data3 = (setup<<4) | ((adcCounts&0b111100000000)>>8);	//vref (1) + pd mode (2) + Gain (1) + upper (4) data bits
	 data4 = (adcCounts&0b000011111111);					//lower (8) data bits
 
	 write_buffer[0] = data2;
	 write_buffer[1] = data3;
	 write_buffer[2] = data4;

	 struct i2c_master_packet packet = {
		 .address         = data1,
		 .data_length     = 3,
		 .data            = write_buffer,
		 .ten_bit_address = false,
		 .high_speed      = false,
		 .hs_master_code  = 0x0,
	 };

	 retVal = i2c_master_write_packet_wait(&i2c_master_instance, &packet);

	 return retVal;

}//writeSingleI2C_DAC


/*-----------------------------------------------------------*/ 
// function  writeFastI2C_DAC
// This function transmits via I2C1 a fast write command to the selected DAC chip
// This routine will use the "Fast Write Command" to sequentially write from Channel A(1) to D(4)
// This voltage IS NOT retained in EEPROM memory.
// In this scenario, use Gain = x2, Internal Reference, and No power down mode
// Therefore max VDC out = 4.096V = 4096 count
// Refer to MCP4728 datasheet, page 38, Figure 5-7 for a helpful diagram
uint8_t writeFastI2C_DAC(uint16_t *dacArray)
 {
	 uint8_t retVal = 0;						//return value
	 uint8_t fail   = 0;						//fail flag
	 uint8_t data[10];							//data words to be constructed for single write command

	 //construct data words to transmit in the "Fast Write Command"
	 //byte 1, device addressing
	 data[0] = ((DAC_MCP4728_DEVICE_ID<<3) | (DAC_MCP4728_DEVICE_ADDR));		//device id (4) + slave addr (3)
	 //byte 2, command + upper 4 bits chA data
	 data[1] = ((DAC_MCP4728_CMD_WRITE_FAST<<4) | ((*dacArray&0b111100000000)>>8));	//cmd bits (4) + upper (8) data bits, DAC CHA
	 //byte 3, lower 8 bits chA data
	 data[2] = (*dacArray&0b000011111111);						//lower (8) data bits, DAC CHA

	 //byte 4, upper 4 bits chB data
	 dacArray++;
	 data[3] = ((*dacArray&0b111100000000)>>8);					//upper (4) data bits, DAC CHB
	 //byte 5, lower 8 bits chB data
	 data[4] = (*dacArray&0b000011111111);						//lower (8) data bits, DAC CHB

	 //byte 6, upper 4 bits chC data
	 dacArray++;
	 data[5] = ((*dacArray&0b111100000000)>>8);					//upper (4) data bits, DAC CHC
	 //byte 7, lower 8 bits chC data
	 data[6] = (*dacArray&0b000011111111);						//lower (8) data bits, DAC CHC

	 //byte 9, upper 4 bits chD data
	 dacArray++;
	 data[7] = ((*dacArray&0b111100000000)>>8);					//upper (4) data bits, DAC CHD
	 //byte 9, lower 8 bits chD data
	 data[8] = (*dacArray&0b000011111111);						//lower (8) data bits, DAC CHD

	 write_buffer[0] = data[1];
	 write_buffer[1] = data[2];
	 write_buffer[2] = data[3];
	 write_buffer[3] = data[4];
	 write_buffer[4] = data[5];
	 write_buffer[5] = data[6];
	 write_buffer[6] = data[7];
	 write_buffer[7] = data[8];

	 struct i2c_master_packet packet = {
		 .address         = data[0],
		 .data_length     = 8,
		 .data            = write_buffer,
		 .ten_bit_address = false,
		 .high_speed      = false,
		 .hs_master_code  = 0x0,
	 };

	 retVal = i2c_master_write_packet_wait(&i2c_master_instance, &packet);

	 return retVal;

}//writeFastI2C_DAC

/**
 * \brief This task sends commands to the I2C_DAC driver
 */
void xTaskI2CDac(void *pvParameters)
{
	/*declare command which will be received from the queue */
	I2CDac_cmd_t xI2CDacCmd;
	
	uint32_t xTaskTicCnt, xNumMessages;
	
	uint8_t retVal;

	/* this is the infinite task loop */
	for (;;) {

	    /* get a I2C_DAC command from the queue */
		if (xQueueReceive(xI2CDacQHndle, &xI2CDacCmd, portMAX_DELAY)) {
			

			/*call function to write selected voltage level to selected channel*/
			writeSingleI2C_DAC(xI2CDacCmd.uxChId, xI2CDacCmd.uxCounts);
			
			xTaskTicCnt = xTaskGetTickCount();
			
			xNumMessages = uxQueueMessagesWaiting(xI2CDacQHndle);

			printf	("-- xTaskI2CDac - writeSingleI2C_DAC OK Tics: %d, Msgs: %d, ChId: %d, Counts: %d\r\n",xTaskTicCnt, xNumMessages, xI2CDacCmd.uxChId,xI2CDacCmd.uxCounts);
							
			/*check for error*/
			if (retVal==STATUS_OK) {
				printf	("-- xTaskI2CDac - writeSingleI2C_DAC OK ChId: %d, Counts: %d\n",xI2CDacCmd.uxChId,xI2CDacCmd.uxCounts);
			} else if (retVal==STATUS_ERR_TIMEOUT){
				printf	("-- xTaskI2CDac - writeSingleI2C_DAC ERR_TIMEOUT !!\n");
			} else if (retVal==STATUS_ERR_DENIED){
				printf	("-- xTaskI2CDac - writeSingleI2C_DAC ERR_DENIED!!\n");
			} else if (retVal==STATUS_ERR_PACKET_COLLISION){
				printf	("-- xTaskI2CDac - writeSingleI2C_DAC PACKET_COLLISION !!\n");;
			} else if (retVal==STATUS_ERR_BAD_ADDRESS){
				printf	("-- xTaskI2CDac - writeSingleI2C_DAC BAD_ADDRESS!!\n");
			}
						
		}//end if xQueueReceive
		
		vTaskDelay( pdMS_TO_TICKS( 10) );

	}//end for

	/* for completeness, this should never happen */
	vTaskDelete( NULL );

}//end xTaskI2CDac

/*-----------------------------------------------------------*/
//This function is used to test/debug scaling of DAC inputs
//The DAC outputs counts can be manipulated and checked against reference spreadsheet*/
void dacScalingDebugCheck(void){

	printf("\r\n\r\n !!!!! DAC SCALING DEBUG CHECK !!!! \r\n");

	printf("**VOUT C - BAT I SET -->\r\n");
	uint16_t testmV = 750  ;  /*mV to output to DAC*/
	printf("**BATT_I_SET_DAC_MV_TO_CNTS test --> %d mv = %d DAC cnts \r\n", testmV, BATT_I_SET_DAC_MV_TO_CNTS(testmV));

	printf("**VOUT B - Niqor DC/DC setpoint -->\r\n");
	testmV = 2180  ;  /*mV to output to DAC*/
	printf("**ISET_DAC_MV_TO_CNTS test --> %d mv = %d DAC cnts \r\n", testmV, ISET_DAC_MA_TO_CNTS(testmV));

	printf("**VOUT A - DAC Voltage Output -->\r\n");
	testmV = 20000  ;  /*this is the requested command mV output of the DC/DC converter*/
	//printf("**VSET_MV_CNTS test --> command: %d mv = %d DAC cnts \r\n", testmV, VSET_DAC_MV_CNTS(testmV));

	printf("**VOUT A - DAC Voltage Output -->\r\n");
	testmV = 24500  ;  /*this is the requested command mV output of the DC/DC converter*/
	//printf("**VSET_MV_CNTS test --> command: %d mv = %d DAC cnts \r\n", testmV, VSET_DAC_MV_CNTS(testmV));

//	printf("**VOUT A - DAC Voltage Output -->\r\n");
//	testmV = 29000  ;  /*this is the requested command mV output of the DC/DC converter*/
//	printf("**VSET_MV_CNTS test --> command: %d mv = %d DAC cnts \r\n", testmV, VSET_DAC_MV_CNTS(testmV));

	//This tests the DAC_OUT_LIMITER, which is a macro which can be used to ensure proper command
	//values are sent to the device.
	uint16_t dacCnts = -1;
	printf("**DAC Test Output Limiter %d --> Out to DAC %d\r\n", dacCnts, DAC_OUT_LIMITER(dacCnts) );
	dacCnts = 0;
	printf("**DAC Test Output Limiter %d --> Out to DAC %d\r\n", dacCnts, DAC_OUT_LIMITER(dacCnts) );
	dacCnts = 4000;
	printf("**DAC Test Output Limiter %d --> Out to DAC %d\r\n", dacCnts, DAC_OUT_LIMITER(dacCnts) );
	dacCnts = 4999;
	printf("**DAC Test Output Limiter %d --> Out to DAC %d\r\n", dacCnts, DAC_OUT_LIMITER(dacCnts) );
			
}//dacScalingDebugCheck
