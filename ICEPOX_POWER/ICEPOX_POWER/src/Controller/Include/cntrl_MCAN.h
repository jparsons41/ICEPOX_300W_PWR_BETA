/*
 * cntrl_MCAN.h
 *
 * Created: 3/10/2017 11:14:44 AM
 *  Author: Gerald
 */

#ifndef CNTRL_MCAN_H_
#define CNTRL_MCAN_H_

/* Standard includes */
#include <asf.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>

 #include "status_codes.h"

/* Driver includes */
#include "conf_can.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"

#include "conf_application.h"

#include "cntrl_GLOBALS.h"

/*!! CAUTION - Enabling VERBOSE to serial console will cause message flood!!*/
#define  CNTRL_MCAN_RX_VERBOSE		0
#define  CNTRL_MCAN_TX_VERBOSE		0

#define  CAN_TX_BUFFER_INDEX        0

/* mcan_filter_setting */
#define MCAN_RX_STANDARD_FILTER_INDEX_0						0			/*this is the index of the filter used for RX*/

#define MCAN_TX_MSG_BATTERY_STATUS							0x30		/*this is ID of BATTERY_STATUS message*/
#define MCAN_TX_MSG_POWER_STATUS							0x31		/*this is ID of POWER_STATUS message*/
#define MCAN_TX_MSG_LEN										8			/*this the length used for above messages*/

#define MCAN_TX_MSDELAY										50			/*this set the frequency in ms of CAN TX messages to controller*/

/*message filter range for incoming MCAN message, FIFO 1 & 2 */
/*for now, these are totally arbitrary, and since this will be a closed*
/*network, can really be set to anything...*/
/*for simplicity, we'll just use standard 11-bit address range*/
#define MCAN_RX_FILTER_0_ID_LOW								0x01		/*lower filter index*/
#define MCAN_RX_FILTER_0_ID_HI								0x45		/*upper filter index*/

/*message IDs*/
#define THROTTLE_CMD										0x10

/*because of range, all will end up in FIFO_0*/
#define MCAN_RX_POWER_COMMAND								0x40		/*control board command and data message id*/
#define MCAN_RX_POWER_COMMAND_STR							"PWR_CMD"	/*control board message string (for debug)*/

#define MCAN_RX_POWER_CFG_COMMAND							0x41		/*control board command and data message id*/
#define MCAN_RX_POWER_CFG_COMMAND_STR						"PWR_CFG_CMD"	/*control board message string (for debug)*/

#define MCAN_RX_I_BATT_SP_MSG										0x1		/*xxx*/
#define MCAN_RX_V_BATT_HIGH_MSG										0x2		/*xxx*/
#define MCAN_RX_V_BATT_LOW_MSG										0x3		/*xxx*/
#define MCAN_RX_I_BATT_HIGH_MSG										0x4		/*xxx*/
#define MCAN_RX_I_BATT_LOW_MSG										0x5		/*xxx*/
#define MCAN_RX_I_BATT_TRICKLE_MSG									0x6		/*xxx*/
#define MCAN_RX_V_BATT_RECHARGE_MSG									0x7		/*xxx*/
#define MCAN_RX_ALT_UNREG_MIN_MSG									0x8		/*xxx*/
#define MCAN_RX_ALT_UNREG_MAX_MSG									0x9		/*xxx*/
#define MCAN_RX_I_LOAD_MAX_MSG										0xa		/*xxx*/

/*fault codes passed in RX message*/
#define MCAN_RX_FLT_ALL_OK									0
#define MCAN_RX_FLT_CAN_BUS_OFF								1
#define MCAN_RX_FLT_CAN_PROTOCOL_ERROR_DATA					2
#define MCAN_RX_FLT_CAN_PROTOCOL_ERROR_ARBITRATION          3

/*Stores information specific to each message ID defined above*/
/*up to four 16-bit data values in each message */
typedef struct msg_info {
	uint16_t data_val[8];		 /*16-bit data*/
	uint8_t  is_first_msg;       /*for elapsed time calculation, stores if this is the first msg received*/
	uint32_t tic_this;           /*FreeRTOS tic time when this message ID is processed*/
	uint32_t tic_last;			 /*FreeRTOS tic time last received*/
	uint16_t tic_elapsed;		 /*stores calculation of elapsed tic time since this message was received*/
} msg_info_t;

/* This is the structure for all data received from the control board board*/
typedef struct cntrlbd_data{
	msg_info_t cmd1;			/*holds CMD1 message data*/
	uint8_t    last_msg_id;		/*holds the ID of the last received message*/
	uint8_t    bus_flt;		    /*holds a fault register*/
	uint32_t   tic_last;	    /*tic time of the last message*/
	uint16_t   tic_elapsed;	    /*calculated elapsed time since last message*/
	uint8_t	   in_queue;		/*number of message backed up in the queue*/
	uint8_t	   flt_code;	    /*holds the fault code*/
} cntrlbd_data_t;

/*variable that holds received control board data*/
cntrlbd_data_t rcv_cntrlbd_data;

/*Structure for the Message used to transfer data from MCAN ISR to the message queue*/
struct can_rx_q_msg_s
{
	uint8_t  ucFIFO;		  /*fifo from which data originated 0,1*/
	uint32_t ucID;            /*msg identifier*/
	uint8_t  ucLEN;           /*payload length*/
	uint32_t ucRXTS;          /*rx timestamp*/
	uint8_t  ucESI;           /*error state id bit*/
	uint32_t ucData[ 8 ];     /*data payload*/
	uint8_t  ucFaultCode;	  /*fault code for MCAN faults*/
} xMessage;

#define MAX_DEBUG_CAN_RX_QUEUE_ITEMS    50 /*this queue is for debug CLI command only, not for processing */

/*data type for data queue to store can rx data*/
typedef struct can_rx_queue_data {
	uint8_t		msg_id;
	uint32_t    rvcd_tic;
	uint16_t    tic_elapsed;
    uint8_t		flt;
    uint8_t		inQ;
} can_rx_queue_data_t;

typedef	struct can_error_handler {
	uint8_t	    last_fault_code;
	uint8_t     fault_status;
	uint16_t    retry_cnt;
} can_error_handler_t;

/*hold info about can errors*/
can_error_handler_t can_err_data;

/*this queue structure implements a circular ring buffer for storing the last XX CAN_RX messages */
typedef struct circularQueue_s
{
	uint8_t     first;
	uint8_t     last;
	uint8_t     validItems;
	can_rx_queue_data_t   q_data[MAX_DEBUG_CAN_RX_QUEUE_ITEMS];
} can_circularQueue_t;
/*variable that holds RX CAN queue*/
can_circularQueue_t rx_can_queue;
can_rx_queue_data_t new_rx_data;

/* mcan_transfer_message_setting */
#define MCAN_TX_BUFFER_INDEX    0

/*define length of TX and RX Queues */
#define MCAN_RTOS_RX_QUEUE_LEN	50
#define MCAN_RTOS_TX_QUEUE_LEN	50

/* message queue for incoming MCAN RX data */
xQueueHandle xMCANRXQHndle;

/* queue for periodic test transmit message */
//xQueueHandle  xMCANTXQHndle;

static struct can_module can_instance;

/*forward declarations for CAN functions*/
void configure_mcan(void);
void mcan_send_standard_message(uint32_t id_value, uint8_t *data, uint32_t data_length);
void mcan_send_extended_message(uint32_t id_value, uint8_t *data, uint32_t data_length);
void mcan_set_standard_filter_0(void);
//void mcan_set_standard_filter_1(void);

/*forward declarations for CAN related tasks*/
void xTaskMCAN_RX_Handler( void *pvParameters );
void xTaskMCAN_TX( void *pvParameters);
void xTaskMCAN_ERROR_Handler( void *pvParameters );

/*forward declarations for CAN msg queue*/
void mcan_q_init  (can_circularQueue_t *theQueue);
uint8_t mcan_q_isEmpty(can_circularQueue_t *theQueue);
uint8_t mcan_q_putItem(can_circularQueue_t *theQueue, can_rx_queue_data_t qItem);
uint8_t mcan_q_getItem(can_circularQueue_t *theQueue, can_rx_queue_data_t *qItem);

#endif /* CNTRL_MCAN_H_ */