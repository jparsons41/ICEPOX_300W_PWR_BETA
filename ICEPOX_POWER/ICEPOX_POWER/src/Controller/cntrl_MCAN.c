/*
 * cntrl_MCAN.c
 *
 * Created: 3/10/2017 11:14:31 AM
 *  Author: Gerald
 */

#include "cntrl_MCAN.h"
#include "cntrl_GLOBALS.h"


/* mcan_receive_message_setting */
volatile uint32_t standard_receive_index = 0;

struct can_rx_element_fifo_0 rx_element_fifo_0;

struct can_rx_element_buffer rx_element_buffer;

//SPI_Stepper_cmd_t xSPIStepperCmd;   /*this is the command structure, see cntrl_SPI_STEPPER.h*/


/*-----------------------------------------------------------*/
void configure_mcan(void)
{

	/* initialize msg queue for tracking RX messages*/
	mcan_q_init(&rx_can_queue);

	/* Set up the CAN TX/RX pins */
	struct system_pinmux_config pin_config;
	system_pinmux_get_config_defaults(&pin_config);
	pin_config.mux_position = CAN_TX_MUX_SETTING;
	system_pinmux_pin_set_config(CAN_TX_PIN, &pin_config);
	pin_config.mux_position = CAN_RX_MUX_SETTING;
	system_pinmux_pin_set_config(CAN_RX_PIN, &pin_config);

	/* Initialize the module. */
	struct can_config config_can;
	can_get_config_defaults(&config_can);
	can_init(&can_instance, CAN_MODULE, &config_can);

	/* Enable interrupts for this CAN module */
	system_interrupt_enable(SYSTEM_INTERRUPT_MODULE_CAN0);
	can_enable_interrupt(&can_instance, CAN_PROTOCOL_ERROR_ARBITRATION
	| CAN_PROTOCOL_ERROR_DATA);

	/*initialize data structure holding received controller board data*/
	/*this is message 0x40 POWER_COMMAND*/
	rcv_cntrlbd_data.cmd1.data_val[0]   = 0;
	rcv_cntrlbd_data.cmd1.data_val[1]   = 0;
	rcv_cntrlbd_data.cmd1.data_val[2]   = 0;
	rcv_cntrlbd_data.cmd1.data_val[3]   = 0;
	rcv_cntrlbd_data.cmd1.data_val[4]   = 0;
	rcv_cntrlbd_data.cmd1.data_val[5]   = 0;
	rcv_cntrlbd_data.cmd1.data_val[6]   = 0;
	rcv_cntrlbd_data.cmd1.data_val[7]   = 0;
	rcv_cntrlbd_data.cmd1.tic_this      = 0;
	rcv_cntrlbd_data.cmd1.tic_last      = 0;
	rcv_cntrlbd_data.cmd1.tic_elapsed   = 0;
	rcv_cntrlbd_data.cmd1.is_first_msg  = 0;

	/*other messages would be initialized here ...*/

	/*stats about the last message*/
	rcv_cntrlbd_data.last_msg_id	 = 0;
	rcv_cntrlbd_data.bus_flt		 = 0;
	rcv_cntrlbd_data.tic_last        = 0;
	rcv_cntrlbd_data.tic_elapsed     = 0;

	/*create the message queue for MCAN RX incoming message */
	if ( xMCANRXQHndle == NULL){
		xMCANRXQHndle = xQueueCreate(MCAN_RTOS_RX_QUEUE_LEN, sizeof( struct can_rx_q_msg_s * ) );
	}

	can_err_data.fault_status = 0;
}

/*-----------------------------------------------------------*/
void mcan_send_standard_message(uint32_t id_value, uint8_t *data,
	uint32_t data_length)
{
	uint32_t i;
	struct can_tx_element tx_element;

	can_get_tx_buffer_element_defaults(&tx_element);
	tx_element.T0.reg |= CAN_TX_ELEMENT_T0_STANDARD_ID(id_value);
	tx_element.T1.bit.DLC = data_length;
	for (i = 0; i < data_length; i++) {
		tx_element.data[i] = *data;
		data++;
	}
	can_set_tx_buffer_element(&can_instance, &tx_element,
	CAN_TX_BUFFER_INDEX);
	can_tx_transfer_request(&can_instance, 1 << CAN_TX_BUFFER_INDEX);
}


/*-----------------------------------------------------------*/
/**
 * \brief set receive standard MCAN ID, dedicated buffer
 *
 */
void mcan_set_standard_filter_0(void)
{
	struct can_standard_message_filter_element sd_filter;

	/*gwg - IMPORTANT - Chp. 49, Sec. 5.7.5, page 1398 for MCAN filter configuration */
	/*http://www.atmel.com/images/Atmel-44003-32-bit-Cortex-M7-Microcontroller-SAM-V71Q-SAM-V71N-SAM-V71J_Datasheet.pdf*/

	can_get_standard_message_filter_element_default(&sd_filter);
	sd_filter.S0.bit.SFT    = 0 ;   /*Range filter from SF1ID to SF2ID (SF2ID ? SF1ID)*/
	sd_filter.S0.bit.SFID2 = MCAN_RX_FILTER_0_ID_HI;  /*high range standard address*/
	sd_filter.S0.bit.SFID1 = MCAN_RX_FILTER_0_ID_LOW; /*low range standard address*/
	sd_filter.S0.bit.SFEC =  1;     /*Store in Rx FIFO 0 if filter matches*/
	/*set filter 0*/
	can_set_rx_standard_filter(&can_instance, &sd_filter,
			MCAN_RX_STANDARD_FILTER_INDEX_0);
	/*enable interrupt*/
	can_enable_interrupt(&can_instance, CAN_RX_FIFO_0_NEW_MESSAGE);
}

/*-----------------------------------------------------------*/
/**
 * \brief Interrupt handler for MCAN,
 *   including RX,TX,ERROR and so on processes.
 */
void CAN0_Handler(void)
{

	uint8_t xVerboseDebug = 1;

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	volatile uint32_t status, i, rx_buffer_index;

	struct can_rx_q_msg_s *pxMessage;
	pxMessage = &xMessage;

	/*read interrupt status*/
	status = can_read_interrupt_status(&can_instance);

	/*if message is from FIFO 0 */
	if (status & CAN_RX_FIFO_0_NEW_MESSAGE) {
		can_clear_interrupt_status(&can_instance, CAN_RX_FIFO_0_NEW_MESSAGE);
		can_get_rx_fifo_0_element(&can_instance, &rx_element_fifo_0,
				standard_receive_index);
		can_rx_fifo_acknowledge(&can_instance, 0,
				standard_receive_index);
		standard_receive_index++;
		if (standard_receive_index == CONF_CAN0_RX_FIFO_0_NUM) {
			standard_receive_index = 0;
		}
		/*retrieve data packet from fifo 0, load into can_rx_msg*/
		if (!rx_element_fifo_0.R0.bit.XTD) {
		    /*if a standard message, ID is in bits 18..28, so extract*/
		    pxMessage->ucID   = ((rx_element_fifo_0.R0.bit.ID & (0x1FFC0000ul))>>18);    /* standard id */
		} else {
			/*if extended identifier, ID id in bits 0..28*/
			pxMessage->ucID   = (rx_element_fifo_0.R0.bit.ID & (0x1FFFFFFFul));          /* extended id */
		}
		pxMessage->ucLEN  = rx_element_fifo_0.R1.bit.DLC;   /* len */
		pxMessage->ucRXTS = rx_element_fifo_0.R1.bit.RXTS;  /* rx timestamp */
		pxMessage->ucESI  = rx_element_fifo_0.R0.bit.ESI;   /* error state bit */
		pxMessage->ucFIFO = 0; /*fifo from which data was captured (should be based on filter config)*/
		pxMessage->ucFaultCode = MCAN_RX_FLT_ALL_OK;
		/*copy data*/
		for (i = 0; i < rx_element_fifo_0.R1.bit.DLC; i++) {
			pxMessage->ucData[i] = rx_element_fifo_0.data[i];
		}

		/*this is helpful for debug, but will cause message flood*/
		#if (CNTRL_MCAN_RX_VERBOSE)
			printf(("-- CAN0_Handler Msg ID x%X in FIFO 0, Len %d, Data is: \r\n",pxMessage->ucID,rx_element_fifo_0.R1.bit.DLC));
			for (i = 0; i < rx_element_fifo_0.R1.bit.DLC; i++) {
				printf("  0x%X",rx_element_fifo_0.data[i]);
			}
			printf("\r\n");
		#endif // CNTRL_MCAN_RX_VERBOSE

	}

	/*if buss has turned off*/
	if (status & CAN_BUS_OFF) {
		can_clear_interrupt_status(&can_instance, CAN_BUS_OFF);
		can_stop(&can_instance);
		printf("!! CAN0_Handler - CAN_BUS_OFF !! \r\n");
		pxMessage->ucFaultCode = MCAN_RX_FLT_CAN_BUS_OFF;
		clearCANcommands();					// clears gbl CAN commands from memory
	}

	/*CAN_PROTOCOL_ERROR_DATA*/
	if (status & CAN_PROTOCOL_ERROR_DATA) {
		can_clear_interrupt_status(&can_instance, CAN_PROTOCOL_ERROR_DATA);
		can_stop(&can_instance);
		printf("!! CAN0_Handler - CAN_PROTOCOL_ERROR_DATA !! \r\n");
		pxMessage->ucFaultCode = MCAN_RX_FLT_CAN_PROTOCOL_ERROR_ARBITRATION;
		clearCANcommands();					// clears gbl CAN commands from memory
	}

	/*CAN_PROTOCOL_ERROR_ARBITRATION*/
	if (status & CAN_PROTOCOL_ERROR_ARBITRATION) {
		can_clear_interrupt_status(&can_instance, CAN_PROTOCOL_ERROR_ARBITRATION);
		can_stop(&can_instance);
		//printf("!! CAN0_Handler - CAN_PROTOCOL_ERROR_ARBITRATION !! \r\n"); // lmp uncomment
		pxMessage->ucFaultCode = MCAN_RX_FLT_CAN_PROTOCOL_ERROR_DATA;
		clearCANcommands();					// clears gbl CAN commands from memory
	}


	//if (pxMessage->ucFaultCode == MCAN_RX_FLT_ALL_OK) {
		if (!xQueueSendToBackFromISR( xMCANRXQHndle, &pxMessage, &xHigherPriorityTaskWoken )) {
			printf("!! CAN0_Handler - xMCANRXQHndle Queue is Full !! \r\n");
		}
	//}

	/*yield from ISR*/
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

}//MCAN1_Handler

/*-----------------------------------------------------------*/
void xTaskMCAN_RX_Handler( void *pvParameters )
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	BaseType_t xThisTicCnt  = 0;    /*stores tic time of this received message*/
	BaseType_t xNum_waiting = 0;	/*number of messages waiting in queue */
	BaseType_t xStatus      = 0;    /*queue status */

	uint8_t static last_cmd_pos = 0; /*used as a simple check to not send the identical position to the stepper back-to-back*/

	struct can_rx_q_msg_s *ptr_can_rx_q_msg;
	ptr_can_rx_q_msg = &xMessage;

	/*start the can process*/
	can_start(&can_instance);

	/*reset error handling stats*/
	can_err_data.fault_status		= 0;    /*there is no error */
	can_err_data.last_fault_code	= 0;	/*clear last fault code */
	can_err_data.retry_cnt			= 0;    /*clear retry counter*/

	/* As per most tasks, this task is implemented within an infinite loop. */
	for( ;; )
	{
		/* get a value to write from somewhere else */
		xStatus = xQueueReceive(xMCANRXQHndle, &ptr_can_rx_q_msg, portMAX_DELAY);

		if( xStatus == pdPASS ) {

			xThisTicCnt = xTaskGetTickCount();

			/*for fun, get the number of message waiting in the queue*/
			xNum_waiting = uxQueueMessagesWaiting(xMCANRXQHndle);

			/* parse the received message */
			/* this is general status stuff ...*/
			rcv_cntrlbd_data.last_msg_id = ptr_can_rx_q_msg->ucID;                  /*message id, 11 or 29 bit*/
			rcv_cntrlbd_data.tic_elapsed = xThisTicCnt - rcv_cntrlbd_data.tic_last; /*calculate elapsed time since last message */
			rcv_cntrlbd_data.tic_last    = xThisTicCnt;                             /*message last rcv tics*/
			rcv_cntrlbd_data.bus_flt     = ptr_can_rx_q_msg->ucESI;                 /*message error state*/
			rcv_cntrlbd_data.in_queue    = xNum_waiting;		                    /*number of message waiting in queue*/
			rcv_cntrlbd_data.flt_code	 = ptr_can_rx_q_msg->ucFaultCode;           /*fault code*/

			/*track errors*/
			if (!rcv_cntrlbd_data.flt_code){
				can_err_data.fault_status = 0;									    /*there is no error */
				can_err_data.retry_cnt = 0;											/*clear retry counter*/
			}//end if
			else {
				can_err_data.last_fault_code = rcv_cntrlbd_data.flt_code;	        /*this is the fault code */
				can_err_data.fault_status = 1;									    /*set error */
				/* Create CAN ERROR Handler Task */
				//printf( "-- CAN_RX: - Starting xTaskCAN_ERROR_Handler, Fault Code %d !!!!! ...\r\n",can_err_data.last_fault_code ); // lmp uncomment
				if (xTaskCreate(xTaskMCAN_ERROR_Handler, "CAN_ERR_HNDLR", CAN_ERR_HANDLER_TASK_STACK_SIZE, NULL,
					CAN_ERR_HANDLER_TASK_PRIORITY, NULL) != pdPASS) {
					printf("-- CAN_RX: - Failed to Create xTaskCAN_ERROR_Handler !!!!! ...\r\n");
				}//end xTaskCAN_ERROR
			}

			/*helpful for debug but will cause message flood*/
			#if (CNTRL_MCAN_RX_VERBOSE)
				printf("-- CAN_RX: ID 0x%X, Fifo %d, Len %d, Tic %d, Elap %d, Flt %d, InQ %d\r\n",
					rcv_cntrlbd_data.last_msg_id,
					ptr_can_rx_q_msg->ucFIFO,
					ptr_can_rx_q_msg->ucLEN,
					rcv_cntrlbd_data.tic_last,
					rcv_cntrlbd_data.tic_elapsed,
					rcv_cntrlbd_data.flt_code,
					xNum_waiting);
			#endif // CNTRL_MCAN_RX_VERBOSE

			/*if not an error message */
			if ((!ptr_can_rx_q_msg->ucESI)||(!rcv_cntrlbd_data.flt_code)){

				/*store this in the RX queue for debugging (used in CLI)*/
				new_rx_data.msg_id = rcv_cntrlbd_data.last_msg_id;
				new_rx_data.rvcd_tic = rcv_cntrlbd_data.tic_last;
				new_rx_data.tic_elapsed = rcv_cntrlbd_data.tic_elapsed;
				new_rx_data.inQ = xNum_waiting;

				/*place in debug q*/
				mcan_q_putItem(&rx_can_queue, new_rx_data);

			   /*switch based on message ID, which must have passed the filter*/
			   switch (ptr_can_rx_q_msg->ucID) {

				  /*this decodes POWER_COMMAND 0x40 message*/
				  case MCAN_RX_POWER_COMMAND:
				        /*extract data from the controller*/
						/*motor command speed, 8-bits*/
						rcv_cntrlbd_data.cmd1.data_val[0] = (uint16_t)  (ptr_can_rx_q_msg->ucData[0]);
						if (rcv_cntrlbd_data.cmd1.data_val[0]) {
							uint8_t dummy = 0;
						}
						/*bit mapped command byte, 8-bits*/
						rcv_cntrlbd_data.cmd1.data_val[1] = (uint16_t)  (ptr_can_rx_q_msg->ucData[1]);
						//voltage set-point byte, 8-bits//
						rcv_cntrlbd_data.cmd1.data_val[2] = (uint16_t)  (ptr_can_rx_q_msg->ucData[2]);
						//current set-point byte, 8-bits//
						rcv_cntrlbd_data.cmd1.data_val[3] = (uint16_t)  (ptr_can_rx_q_msg->ucData[3]);

						/*store last time message was processed*/
						rcv_cntrlbd_data.cmd1.tic_last = rcv_cntrlbd_data.cmd1.tic_this;
						/*get the current time message is being processed */
						rcv_cntrlbd_data.cmd1.tic_this = xTaskGetTickCount();
						/*calculate elapsed tics (may go goofy on a rollover) */
						if (!rcv_cntrlbd_data.cmd1.is_first_msg){
							rcv_cntrlbd_data.cmd1.tic_elapsed = 0;
							rcv_cntrlbd_data.cmd1.is_first_msg = 1;
						}
						else rcv_cntrlbd_data.cmd1.tic_elapsed = rcv_cntrlbd_data.cmd1.tic_this - rcv_cntrlbd_data.cmd1.tic_last;
						#if (CNTRL_MCAN_RX_VERBOSE)
							printf("^^ %6s: %x\t %x\t %x\t %x || %d\t %d\t %d\r\n",
								MCAN_RX_POWER_COMMAND_STR,
								rcv_cntrlbd_data.cmd1.data_val[0],		//motor command, 8-bits
								rcv_cntrlbd_data.cmd1.data_val[1],		//bit mapped command byte, 8-bits
								rcv_cntrlbd_data.cmd1.data_val[2],		//voltage set-point byte, 8-bits
								rcv_cntrlbd_data.cmd1.data_val[3],		//current set-point, 8-bits
								rcv_cntrlbd_data.cmd1.tic_last,			//last time this message was received
								rcv_cntrlbd_data.cmd1.tic_this,			//time this messages was received
								rcv_cntrlbd_data.cmd1.tic_elapsed);		//elapsed time since the last message
						#endif

						//Parse POWER_COMMAND (0x40) into the global variable structure
						//Set data in the gbl_PwrCmd variable structure for processing in the control loop(s)
						if (rcv_cntrlbd_data.flt_code==0){
							gbl_PwrCmd.motor				= rcv_cntrlbd_data.cmd1.data_val[0];					//motor command, 8-bits
							gbl_PwrCmd.cmd					= rcv_cntrlbd_data.cmd1.data_val[1];					//bit mapped command byte, 8-bits
							gbl_PwrCmd.set_v				= rcv_cntrlbd_data.cmd1.data_val[2],					//voltage set-point byte, 8-bits
							gbl_PwrCmd.set_i				= rcv_cntrlbd_data.cmd1.data_val[3],					//current set-point, 8-bits
							gbl_PwrCmd.stats.rcv_tic_last	= rcv_cntrlbd_data.cmd1.tic_last;						//last time this message was received
							gbl_PwrCmd.stats.rcv_tic_this	= rcv_cntrlbd_data.cmd1.tic_this;						//time this messages was received
							gbl_PwrCmd.stats.rcv_elapsed	= rcv_cntrlbd_data.cmd1.tic_elapsed;					//elapsed time since the last message
							gbl_PwrCmd.stats.fault			= rcv_cntrlbd_data.flt_code;

							//parse out and set gbl_CmdFlags (this is the same data as in gbl_PwrCmd.cmd)
							gbl_CmdFlags.sys_run			= ((rcv_cntrlbd_data.cmd1.data_val[1] & 0x01) >> 0);	//shutdown flag
							gbl_CmdFlags.output_en			= ((rcv_cntrlbd_data.cmd1.data_val[1] & 0x02) >> 1);	//output enable flag
							gbl_CmdFlags.bat_chrg_en		= ((rcv_cntrlbd_data.cmd1.data_val[1] & 0x04) >> 2);	//battery charge enable flag
							gbl_CmdFlags.resv_3				= ((rcv_cntrlbd_data.cmd1.data_val[1] & 0x08) >> 3);
							gbl_CmdFlags.resv_4				= ((rcv_cntrlbd_data.cmd1.data_val[1] & 0x10) >> 4);
							gbl_CmdFlags.resv_5				= ((rcv_cntrlbd_data.cmd1.data_val[1] & 0x20) >> 5);
							gbl_CmdFlags.resv_6				= ((rcv_cntrlbd_data.cmd1.data_val[1] & 0x40) >> 6);
							gbl_CmdFlags.resv_7				= ((rcv_cntrlbd_data.cmd1.data_val[1] & 0x80) >> 7);
						} else {

							gbl_PwrCmd.motor				= 0;					//motor command, 8-bits
							gbl_PwrCmd.cmd					= 0;					//bit mapped command byte, 8-bits
							gbl_PwrCmd.set_v				= 0,					//voltage set-point byte, 8-bits
							gbl_PwrCmd.set_i				= 0,					//current set-point, 8-bits
							gbl_PwrCmd.stats.rcv_tic_last	= 0;					//last time this message was received
							gbl_PwrCmd.stats.rcv_tic_this	= 0;					//time this messages was received
							gbl_PwrCmd.stats.fault			= rcv_cntrlbd_data.flt_code;
							//parse out and set gbl_CmdFlags (this is the same data as in gbl_PwrCmd.cmd)
							gbl_CmdFlags.sys_run			= 0;	//shutdown flag
							gbl_CmdFlags.output_en			= 0;	//output enable flag
							gbl_CmdFlags.bat_chrg_en		= 0;	//battery charge enable flag
							gbl_CmdFlags.resv_3				= 0;
							gbl_CmdFlags.resv_4				= 0;
							gbl_CmdFlags.resv_5				= 0;
							gbl_CmdFlags.resv_6				= 0;
							gbl_CmdFlags.resv_7				= 0;

						}
						#define TEMP_BATTERY_CHARGE_EN			 PIN_PB31
						if (gbl_CmdFlags.bat_chrg_en == 1) port_pin_set_output_level(TEMP_BATTERY_CHARGE_EN, true);	// lmp remove for test batt charge only
						else port_pin_set_output_level(TEMP_BATTERY_CHARGE_EN, false);

					break;
					 /*this decodes MCAN_RX_POWER_CFG_COMMAND 0x41 message*/
				  case MCAN_RX_POWER_CFG_COMMAND:
				        /*extract data from the controller*/
						/*motor command speed, 8-bits*/
						rcv_cntrlbd_data.cmd1.data_val[0] = (uint16_t)  (ptr_can_rx_q_msg->ucData[0]);
						/*bit mapped command byte, 8-bits*/
						rcv_cntrlbd_data.cmd1.data_val[1] = (uint16_t)  (ptr_can_rx_q_msg->ucData[1]);
						//voltage set-point byte, 8-bits//
						rcv_cntrlbd_data.cmd1.data_val[2] = (uint16_t)  (ptr_can_rx_q_msg->ucData[2]);
						//current set-point byte, 8-bits//
						rcv_cntrlbd_data.cmd1.data_val[3] = (uint16_t)  (ptr_can_rx_q_msg->ucData[3]);
						rcv_cntrlbd_data.cmd1.data_val[4] = (uint16_t)  (ptr_can_rx_q_msg->ucData[4]);
						/*store last time message was processed*/
						rcv_cntrlbd_data.cmd1.tic_last = rcv_cntrlbd_data.cmd1.tic_this;
						/*get the current time message is being processed */
						rcv_cntrlbd_data.cmd1.tic_this = xTaskGetTickCount();
						/*calculate elapsed tics (may go goofy on a rollover) */
						if (!rcv_cntrlbd_data.cmd1.is_first_msg){
							rcv_cntrlbd_data.cmd1.tic_elapsed = 0;
							rcv_cntrlbd_data.cmd1.is_first_msg = 1;
						}
						else rcv_cntrlbd_data.cmd1.tic_elapsed = rcv_cntrlbd_data.cmd1.tic_this - rcv_cntrlbd_data.cmd1.tic_last;
						#if (CNTRL_MCAN_RX_VERBOSE)
							printf("^^ %6s: %x\t %x\t %x\t %x || %d\t %d\t %d\r\n",
								MCAN_RX_POWER_CFG_COMMAND_STR,
								rcv_cntrlbd_data.cmd1.data_val[0],		//motor command, 8-bits
								rcv_cntrlbd_data.cmd1.data_val[1],		//bit mapped command byte, 8-bits
								rcv_cntrlbd_data.cmd1.data_val[2],		//voltage set-point byte, 8-bits
								rcv_cntrlbd_data.cmd1.data_val[3],		//current set-point, 8-bits
								rcv_cntrlbd_data.cmd1.tic_last,			//last time this message was received
								rcv_cntrlbd_data.cmd1.tic_this,			//time this messages was received
								rcv_cntrlbd_data.cmd1.tic_elapsed);		//elapsed time since the last message
						#endif

						//Parse POWER_COMMAND (0x40) into the global variable structure
						//Set data in the gbl_PwrCmd variable structure for processing in the control loop(s)
						gbl_PwrCfgCmd.command		    	= rcv_cntrlbd_data.cmd1.data_val[0];					//motor command, 8-bits
						gbl_PwrCfgCmd.set_value		     	= (rcv_cntrlbd_data.cmd1.data_val[1]<< 24 | rcv_cntrlbd_data.cmd1.data_val[2] <<16 | rcv_cntrlbd_data.cmd1.data_val[3] << 8 | rcv_cntrlbd_data.cmd1.data_val[4]);					//bit mapped command byte, 8-bits
						gbl_PwrCfgCmd.stats.rcv_tic_last	= rcv_cntrlbd_data.cmd1.tic_last;						//last time this message was received
						gbl_PwrCfgCmd.stats.rcv_tic_this	= rcv_cntrlbd_data.cmd1.tic_this;						//time this messages was received
						gbl_PwrCfgCmd.stats.rcv_elapsed	= rcv_cntrlbd_data.cmd1.tic_elapsed;					//elapsed time since the last message


						switch (gbl_PwrCfgCmd.command) {
							case 0:     break;
							case MCAN_RX_I_BATT_SP_MSG:			gbl_PwrCfg.cfg_Ibatt_Sp          = gbl_PwrCfgCmd.set_value;	break;
							case MCAN_RX_V_BATT_HIGH_MSG:		gbl_PwrCfg.cfg_Vbatt_high        = gbl_PwrCfgCmd.set_value;	break;
							case MCAN_RX_V_BATT_LOW_MSG:		gbl_PwrCfg.cfg_Vbatt_low         = gbl_PwrCfgCmd.set_value;	break;
							case MCAN_RX_I_BATT_HIGH_MSG:		gbl_PwrCfg.cfg_Ibatt_high        = gbl_PwrCfgCmd.set_value;	break;
							case MCAN_RX_I_BATT_LOW_MSG:		gbl_PwrCfg.cfg_Ibatt_low         = gbl_PwrCfgCmd.set_value;	break;
							case MCAN_RX_I_BATT_TRICKLE_MSG:	gbl_PwrCfg.cfg_Ibatt_trickle_cut = gbl_PwrCfgCmd.set_value;	break;
							case MCAN_RX_V_BATT_RECHARGE_MSG:	gbl_PwrCfg.cfg_Vbatt_recharge    = gbl_PwrCfgCmd.set_value;	break;
							case MCAN_RX_ALT_UNREG_MIN_MSG:		gbl_PwrCfg.cfg_Alt_unreg_min     = gbl_PwrCfgCmd.set_value;	break;
							case MCAN_RX_ALT_UNREG_MAX_MSG:		gbl_PwrCfg.cfg_Alt_unreg_max     = gbl_PwrCfgCmd.set_value;	break;
							case MCAN_RX_I_LOAD_MAX_MSG:		gbl_PwrCfg.cfg_I_load_max        = gbl_PwrCfgCmd.set_value;  break;
						}

					break;
					/////////////////////////////////////////////////////////
					// ADD DECODING FOR OTHER MESSAGE HERE ....
					/////////////////////////////////////////////////////////

				  default:
				    break;

			   }//end switch

			}//end ptr_can_rx_q_msg->ucESI

		}//end xStatus == pdPASS

	}//end for (;;)

	/* for completeness, this should never happen */
	vTaskDelete( NULL );

}//xTaskMCAN_RX_Handler

/*-----------------------------------------------------------*/
/**
 * \brief This is the task that writes MCAN messages 0x30 and 0x31 to the controller
 * This has been modified to execute every 50 ms (ie 20 Hz)
 */
void xTaskMCAN_TX(void *pvParameters)
{
	UNUSED(pvParameters);

	BaseType_t i			= 0;	/*just an index*/
	BaseType_t xRcv_value   = 0;	/*value to be received */
	BaseType_t xWrite_value = 0;    /*value to be written */
	BaseType_t xNum_waiting = 0;	/*number of messages waiting in queue */
	BaseType_t xStatus      = 0;    /*queue status */
	BaseType_t ping_pong	= 0;	/*used to alternate at 2X msg frequency between 0x30 and 0x31*/

	/*define temp structure to store MCAN messages 0x30 and 0x31*/
	uint8_t CAN_tx_msg_x30[MCAN_TX_MSG_LEN], CAN_tx_msg_x31[MCAN_TX_MSG_LEN];

	/* Initialize the memory for the CAN test message*/
	for (i = 0; i < MCAN_TX_MSG_LEN; i++) {
		CAN_tx_msg_x30[i] = 0;
		CAN_tx_msg_x31[i] = 0;
	}//end for i

	for (;;) {

		/*for simplicity, since there are only two messages, will simply ping-pong at 2X desired RX rate*/
		if (!ping_pong) {
			/*Encode MCAN message 0x30 BATTERY_STATUS*/
			uint16_t	CAN_bat_i = gbl_BatStatus.bat_i + 32768;	// add scalar to battery current to xmit signed int

			CAN_tx_msg_x30[0] = (uint8_t) ( CAN_bat_i & 0x00ff);						/*battery current, mA LSB*/
			CAN_tx_msg_x30[1] = (uint8_t) ((CAN_bat_i & 0xff00)>>8);					/*battery current, mA MSB*/
			CAN_tx_msg_x30[2] = (uint8_t) ( gbl_BatStatus.bat_v & 0x00ff);				/*battery voltage, mV LSB*/
			CAN_tx_msg_x30[3] = (uint8_t) ((gbl_BatStatus.bat_v & 0xff00)>>8);			/*battery voltage, mV MSB*/
			CAN_tx_msg_x30[4] = (uint8_t) ( gbl_BatStatus.bat_chrg_status);				/*battery charge status 1-charging, 0-off*/
			CAN_tx_msg_x30[5] = (uint8_t) ( gbl_BatStatus.buss_v & 0x00ff);				/*buss voltage, mV LSB*/
			CAN_tx_msg_x30[6] = (uint8_t) ((gbl_BatStatus.buss_v & 0xff00)>>8);			/*buss voltage, mV MSB*/
			CAN_tx_msg_x30[7] = (uint8_t) ( gbl_BatStatus.state_of_chrg);				/*state of charge*/
		} else {
			/*Encode MCAN message 0x31 POWER_STATUS*/
			CAN_tx_msg_x31[0] = (uint8_t) ( gbl_PwrStatus.load_i & 0x00ff);				/*load current, mA LSB*/
			CAN_tx_msg_x31[1] = (uint8_t) ((gbl_PwrStatus.load_i & 0xff00)>>8);			/*load current, mA MSB*/
			CAN_tx_msg_x31[2] = (uint8_t) ( gbl_PwrStatus.load_v & 0x00ff);				/*load voltage, mV LSB*/
			CAN_tx_msg_x31[3] = (uint8_t) ((gbl_PwrStatus.load_v & 0xff00)>>8);			/*load voltage, mV MSB*/
			CAN_tx_msg_x31[4] = (uint8_t) ( gbl_PwrStatus.dc_dc_i& 0x00ff);				/*dc/dc current, mA LSB*/
			CAN_tx_msg_x31[5] = (uint8_t) ((gbl_PwrStatus.dc_dc_i& 0xff00)>>8);			/*dc/dc current, mA MSB*/
			CAN_tx_msg_x31[6] = (uint8_t) ( gbl_PwrStatus.pwr_status);					/*gbl_PwrStatusFlags bit-mapping*/
			CAN_tx_msg_x31[7] = (uint8_t) ( gbl_PwrStatus.faults);					    /*gbl_PwrFaultFlags bit-mapping*/
		}	//end if ping_pong

		if (!can_err_data.fault_status){

			#if (CNTRL_MCAN_TX_VERBOSE)
				printf ("-- xTaskMCAN_TX - Send message init val %d  ... \r\n", xWrite_value );
			#endif

			if (!ping_pong) {
				/*send 0x30*/
				mcan_send_standard_message(MCAN_TX_MSG_BATTERY_STATUS, CAN_tx_msg_x30, MCAN_TX_MSG_LEN);
				ping_pong = 1;
			} else {
				/*send 0x31*/
				mcan_send_standard_message(MCAN_TX_MSG_POWER_STATUS, CAN_tx_msg_x31, MCAN_TX_MSG_LEN);
				ping_pong = 0;
			}

		} else{
			#if (CNTRL_MCAN_TX_VERBOSE)
				printf ("-- xTaskMCAN_TX - Can't send message, MCAN Fault Code %d, Retry Cnt %d  ... \r\n", can_err_data.last_fault_code, can_err_data.retry_cnt);
			#endif
		}//end if

		/*DEBUG ONLY - toggle the LED*/
		//LED_Toggle(LED0);

		 /*delay till next time (calc 1/2 the delay, since we're ping-pong'in 0x30 and 0x31 at 2X the desired rate*/
		 vTaskDelay(MCAN_TX_MSDELAY >> 1);

	}//end for;;

	/* for completeness, this should never happen */
	vTaskDelete( NULL );

}//end xTaskMCAN_TX


/*-----------------------------------------------------------*/
void xTaskMCAN_ERROR_Handler( void *pvParameters )
{

	can_err_data.retry_cnt++;

	//printf ("-- xTaskMCAN_ERROR_Handler - Error handling sequence triggered, count %d ...\r\n",can_err_data.retry_cnt); // lmp uncomment

	//shutdown can gracefully (hopefully)
	can_stop(&can_instance);
	//disable interrupts for this CAN module
	system_interrupt_disable(SYSTEM_INTERRUPT_MODULE_CAN0);
	can_disable_interrupt(&can_instance, CAN_PROTOCOL_ERROR_ARBITRATION
							| CAN_PROTOCOL_ERROR_DATA);

	/*delay*/
	//printf ("-- xTaskMCAN_ERROR_Handler - Start Delay %d ...\r\n",500); // lmp uncomment
	vTaskDelay(500);
	//printf ("-- xTaskMCAN_ERROR_Handler - Delay DONE !!! ...\r\n"); // lmp uncomment

	/*reconfigure mcan (starting from scratch)*/
	//printf ("-- xTaskMCAN_ERROR_Handler - Reconfigure MCAN ...\r\n"); // lmp uncomment
	configure_mcan();

	/*start the can process*/
	//printf ("-- xTaskMCAN_ERROR_Handler - Restart MCAN Process ...\r\n"); // lmp uncomment
	can_start(&can_instance);

	vTaskDelete( NULL );

}//end xTaskMCAN_ERROR_Handler

/*-----------------------------------------------------------*/
void mcan_q_init(can_circularQueue_t *theQueue)
{
    int i;
    theQueue->validItems  =  0;
    theQueue->first       =  0;
    theQueue->last        =  0;
    for(i=0; i<MAX_DEBUG_CAN_RX_QUEUE_ITEMS; i++)
    {
        theQueue->q_data[i].flt = 0;
		theQueue->q_data[i].inQ = 0;
		theQueue->q_data[i].msg_id = 0;
		theQueue->q_data[i].rvcd_tic = 0;
	    theQueue->q_data[i].tic_elapsed = 0;
    }
    return;
}
/*-----------------------------------------------------------*/
uint8_t mcan_q_isEmpty(can_circularQueue_t *theQueue)
{
    if(theQueue->validItems==0)
        return(1);
    else
        return(0);
}
/*-----------------------------------------------------------*/
uint8_t mcan_q_putItem(can_circularQueue_t *theQueue, can_rx_queue_data_t qItem)
{
    if(theQueue->validItems<MAX_DEBUG_CAN_RX_QUEUE_ITEMS)
    {
        theQueue->validItems++;
    }
        theQueue->q_data[theQueue->last] = qItem;
        theQueue->last = (theQueue->last+1)%MAX_DEBUG_CAN_RX_QUEUE_ITEMS;
}
/*-----------------------------------------------------------*/
uint8_t mcan_q_getItem(can_circularQueue_t *theQueue, can_rx_queue_data_t *qItem)
{
    if(mcan_q_isEmpty(theQueue))
    {
        return(-1);
    }
    else
    {
        *qItem=theQueue->q_data[theQueue->first];
        theQueue->first=(theQueue->first+1)%MAX_DEBUG_CAN_RX_QUEUE_ITEMS;
        theQueue->validItems--;
        return(0);
    }
}