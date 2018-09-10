/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

 /******************************************************************************
 *
 * See the following URL for information on the commands defined in this file:
 * http://www.FreeRTOS.org/FreeRTOS-Plus/FreeRTOS_Plus_UDP/Embedded_Ethernet_Examples/Ethernet_Related_CLI_Commands.shtml
 *
 ******************************************************************************/


/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* FreeRTOS+CLI includes. */
#include "FreeRTOS_CLI.h"

/*system includes*/
#include "cntrl_GLOBALS.h"
#include "cntrl_DISCRETE_IO.h"
#include "cntrl_MCAN.h"
#include "cntrl_ADC.h"
#include "cntrl_PWM.h"
#include "cntrl_I2C_DAC.h"

#ifndef  configINCLUDE_TRACE_RELATED_CLI_COMMANDS
	#define configINCLUDE_TRACE_RELATED_CLI_COMMANDS 0
#endif

/*-----------------------------------------------------------*/
/*                    FORWARD DECLERATIONS                   */
/*-----------------------------------------------------------*/

/*
 * Implements MCAN Stats
 */
static BaseType_t prvMCANStats( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
/*
 * Implements ADC Stats
 */
static BaseType_t prvADCStats( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );

/*
 * Implements GPIO Stats
 */
static BaseType_t prvGPIOStats( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );

/*
 * Implements PWM manual control command
 */
static BaseType_t prvPWMCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );

/*
 * Implements I2C_DAC manual control command
 */
static BaseType_t prvI2CDACCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );


#if (configGENERATE_RUN_TIME_STATS)
	/*
	 * Implements the run-time-stats command.
	 */
	static portBASE_TYPE prvTaskStatsCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );

	/*
	 * Implements the task-stats command.
	 */
	static portBASE_TYPE prvRunTimeStatsCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
#endif

/*
 * Implements the echo-three-parameters command.
 */
static portBASE_TYPE prvThreeParameterEchoCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );

/*
 * Implements the echo-parameters command.
 */
static portBASE_TYPE prvParameterEchoCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );

/*
 * Implements the "trace start" and "trace stop" commands;
 */
#if configINCLUDE_TRACE_RELATED_CLI_COMMANDS == 1
	static portBASE_TYPE prvStartStopTraceCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
#endif

/*
 * Implements command to set operating MODE (MANUAL/AUTO) via command line
 */
static portBASE_TYPE prvSetModeCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );

/*
 * Implements command to set output manually via command line
 */
static portBASE_TYPE prvSetOutputCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );

/*
 * Implements command to bring all devices to ESTOP ( ALL OFF)
 */
static portBASE_TYPE prvEStopCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );


/*-----------------------------------------------------------*/
/*                    COMMAND DEFINITIONS                    */
/*-----------------------------------------------------------*/

#if (configGENERATE_RUN_TIME_STATS)
	/* Structure that defines the "run-time-stats" command line command.   This
	generates a table that shows how much run time each task has */
	static const CLI_Command_Definition_t xRunTimeStats =
	{
		"run-time-stats", /* The command string to type. */
		"\r\nrun-time-stats:\r\n Displays a table showing how much processing time each FreeRTOS task has used\r\n",
		prvRunTimeStatsCommand, /* The function to run. */
		0 /* No parameters are expected. */
	};

	/* Structure that defines the "task-stats" command line command.  This generates
	a table that gives information on each task in the system. */
	static const CLI_Command_Definition_t xTaskStats =
	{
		"task-stats", /* The command string to type. */
		"\r\ntask-stats:\r\n Displays a table showing the state of each FreeRTOS task\r\n",
		prvTaskStatsCommand, /* The function to run. */
		0 /* No parameters are expected. */
	};
#endif

/* Structure that defines the "echo_3_parameters" command line command.  This
takes exactly three parameters that the command simply echos back one at a
time. */
static const CLI_Command_Definition_t xThreeParameterEcho =
{
	"echo-3-parameters",
	"\r\necho-3-parameters <param1> <param2> <param3>:\r\n Expects three parameters, echos each in turn\r\n",
	prvThreeParameterEchoCommand, /* The function to run. */
	3 /* Three parameters are expected, which can take any value. */
};

/* Structure that defines the "echo_parameters" command line command.  This
takes a variable number of parameters that the command simply echos back one at
a time. */
static const CLI_Command_Definition_t xParameterEcho =
{
	"echo-parameters",
	"\r\necho-parameters <param1> <param2> ... <paramN>:\r\n Take variable number of parameters, echos each in turn\r\n",
	prvParameterEchoCommand, /* The function to run. */
	-1 /* The user can enter any number of commands. */
};

#if configINCLUDE_TRACE_RELATED_CLI_COMMANDS == 1
	/* Structure that defines the "trace" command line command.  This takes a single
	parameter, which can be either "start" or "stop". */
	static const CLI_Command_Definition_t xStartStopTrace =
	{
		"trace",
		"\r\ntrace <start | stop>:\r\n Starts or stops a trace recording for viewing in FreeRTOS+Trace\r\n",
		prvStartStopTraceCommand, /* The function to run. */
		-1 /* One parameter is expected.  Valid values are "start" and "stop". */
	};
#endif /* configINCLUDE_TRACE_RELATED_CLI_COMMANDS */

/*-----------------------------------------------------------*/
/* Structure that defines the MCAN Stats command line command. */
static const CLI_Command_Definition_t xMCANStats =
{
	"mcan-stats", /* The command string to type. */
	"\r\nmcan-stats:\r\n Displays current MCAN RX data buffer ...\r\n",
	prvMCANStats, /* The function to run. */
	-1 /* will loop till all data in buffer prints out */
};

/*-----------------------------------------------------------*/
/* Structure that defines the ADC Stats command line command. */
static const CLI_Command_Definition_t xADCStats =
{
	"adc-stats", /* The command string to type. */
	"\r\nadc-stats:\r\n Displays current ADC data buffer ...\r\n",
	prvADCStats, /* The function to run. */
	-1 /* will loop till all data in buffer prints out */
};

/*-----------------------------------------------------------*/
/* Structure that defines the GPIO Stats command line command. */
static const CLI_Command_Definition_t xGPIOStats =
{
	"gpio-stats", /* The command string to type. */
	"\r\ngpio-stats:\r\n Displays current gpio status ...\r\n",
	prvGPIOStats, /* The function to run. */
	-1 /* will loop till all data in buffer prints out */
};

/* Structure that defines the PWM test command line command. */
static const CLI_Command_Definition_t xPWMCommand =
{
	"pwm-cmd", /* The command string to type. */
	"\r\npwm-cmd <ChID> <Off/On> <DutyCycle>:\r\n Expects three parameters, ChId (1..2) Off/On DutyCycle (0..100) \r\n",
	prvPWMCommand, /* The function to run. */
	-1 /* loop until all commands are parsed, normally 3 */
};

/* Structure that defines the I2C DAC test command line command. */
static const CLI_Command_Definition_t xI2CDacCommand =
{
	"i2c-dac-cmd", /* The command string to type. */
	"\r\ni2c-dac-cmd <ChID> <Counts>:\r\n Expects two parameters, ChId (1, 2, 3, 4), Counts (0 .. 4095) \r\n",
	prvI2CDACCommand, /* The function to run. */
	2 /*Expects two parameters */
};

/* Structure that set operating MODE for debug. Must be in MANUAL to control outputs from command line */
static const CLI_Command_Definition_t xSetModeCommand =
{
	"set-mode", /* The command string to type. */
	"\r\nset-mode <MODE>:\r\n Expects one parameter, MODE (1=AUTO, 0=MANUAL)\r\n",
	prvSetModeCommand, /* The function to run. */
	1 /*Expects one parameters */
};


/* Structure that sets Outputs manually (must be in MANUAL MODE i.e. not running in AUTO) */
static const CLI_Command_Definition_t xSetOutputCommand =
{
	"set-output", /* The command string to type. */
	"\r\nset-output <NUM> <STATE>:\r\n Expects two parameters, OUTPUT_NUM (1 .. N), STATE (1=On/0=Off) \r\n",
	prvSetOutputCommand, /* The function to run. */
	2 /*Expects two parameters */
};

/* Structure that defines ESTOP command */
static const CLI_Command_Definition_t xEStopCommand =
{
	"estop", /* The command string to type. */
	"\r\nestop:\r\n Causes all IO to be turned OFF, including PWM and DAC channels.\r\n",
	prvEStopCommand, /* The function to run. */
	0 /* no parameters*/
};


/*-----------------------------------------------------------*/
/*                    COMMAND REGISTRATION                   */
/*-----------------------------------------------------------*/

void vRegisterSampleCLICommands( void )
{
	/* Register all the command line commands defined immediately above. */
	#if (configGENERATE_RUN_TIME_STATS)
		FreeRTOS_CLIRegisterCommand( &xTaskStats );
		FreeRTOS_CLIRegisterCommand( &xRunTimeStats );
    #endif

	FreeRTOS_CLIRegisterCommand( &xThreeParameterEcho );
	FreeRTOS_CLIRegisterCommand( &xParameterEcho );

	FreeRTOS_CLIRegisterCommand( &xMCANStats);				/* register MCAN Stats command   */
	FreeRTOS_CLIRegisterCommand( &xADCStats);				/* register ADC Stats command */
	FreeRTOS_CLIRegisterCommand( &xGPIOStats);				/* register GPIO Stats command */
	FreeRTOS_CLIRegisterCommand( &xPWMCommand );			/* register PWM command */
	FreeRTOS_CLIRegisterCommand( &xI2CDacCommand);			/* register I2C_DAC command */

	FreeRTOS_CLIRegisterCommand( &xSetModeCommand);			/* register Set Mode command */
	FreeRTOS_CLIRegisterCommand( &xSetOutputCommand);		/* register Set Output command */
	FreeRTOS_CLIRegisterCommand( &xEStopCommand);		    /* register ESTOP */

	#if( configINCLUDE_TRACE_RELATED_CLI_COMMANDS == 1 )
		FreeRTOS_CLIRegisterCommand( & xStartStopTrace );
	#endif
}

/*-----------------------------------------------------------*/
/*                    COMMAND IMPLEMENTAION                  */
/*-----------------------------------------------------------*/

static portBASE_TYPE prvTaskStatsCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
const char *const pcHeader = "\r\nTask\tState\tPriority\tSHWM(wrds)\t#\r\n************************************************\r\n";
const char str1[10];

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	/* Generate a table of task stats. */
	strcpy( pcWriteBuffer, pcHeader );
	vTaskList( pcWriteBuffer + strlen( pcHeader ) );

	/*added by GWG to get heap size data*/
	strncat( pcWriteBuffer, "\r\n", strlen( "\r\n" ) );

	strncat( pcWriteBuffer, ">>>Free Now Heap Size (Bytes): ", strlen( ">>>Free Now Heap Size (Bytes): " ) );
	sprintf(str1,"%d\r\n", xPortGetFreeHeapSize() );
	strncat( pcWriteBuffer, str1, strlen( str1 ) );

	strncat( pcWriteBuffer, ">>>Min Ever Heap Size (Bytes): ", strlen( ">>>Min Ever Heap Size (Bytes): " ) );
	sprintf(str1,"%d\r\n", xPortGetMinimumEverFreeHeapSize() );
	strncat( pcWriteBuffer, str1, strlen( str1 ) );

	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/
static portBASE_TYPE prvRunTimeStatsCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
const char * const pcHeader = "\r\nTask\t\tAbs Time\t% Time\r\n****************************************\r\n";

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	/* Generate a table of task stats. */
	strcpy( pcWriteBuffer, pcHeader );
	vTaskGetRunTimeStats( pcWriteBuffer + strlen( pcHeader ) );

	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/
static portBASE_TYPE prvThreeParameterEchoCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
const char *pcParameter;
portBASE_TYPE xParameterStringLength, xReturn;
static portBASE_TYPE lParameterNumber = 0;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	if( lParameterNumber == 0 )
	{
		/* The first time the function is called after the command has been
		entered just a header string is returned. */
		sprintf( pcWriteBuffer, "The three parameters were:\r\n" );

		/* Next time the function is called the first parameter will be echoed
		back. */
		lParameterNumber = 1L;

		/* There is more data to be returned as no parameters have been echoed
		back yet. */
		xReturn = pdPASS;
	}
	else
	{
		/* Obtain the parameter string. */
		pcParameter = FreeRTOS_CLIGetParameter
							(
								pcCommandString,		/* The command string itself. */
								lParameterNumber,		/* Return the next parameter. */
								&xParameterStringLength	/* Store the parameter string length. */
							);

		/* Sanity check something was returned. */
		configASSERT( pcParameter );

		/* Return the parameter string. */
		memset( pcWriteBuffer, 0x00, xWriteBufferLen );
		sprintf( pcWriteBuffer, "%d: ", ( int ) lParameterNumber );
		strncat( pcWriteBuffer, pcParameter, xParameterStringLength );
		strncat( pcWriteBuffer, "\r\n", strlen( "\r\n" ) );

		/* If this is the last of the three parameters then there are no more
		strings to return after this one. */
		if( lParameterNumber == 3L )
		{
			/* If this is the last of the three parameters then there are no more
			strings to return after this one. */
			xReturn = pdFALSE;
			lParameterNumber = 0L;
		}
		else
		{
			/* There are more parameters to return after this one. */
			xReturn = pdTRUE;
			lParameterNumber++;
		}
	}

	return xReturn;
}

/*-----------------------------------------------------------*/
static portBASE_TYPE prvParameterEchoCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
const char *pcParameter;
portBASE_TYPE xParameterStringLength, xReturn;
static portBASE_TYPE lParameterNumber = 0;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	if( lParameterNumber == 0 )
	{
		/* The first time the function is called after the command has been
		entered just a header string is returned. */
		sprintf( pcWriteBuffer, "The parameters were:\r\n" );

		/* Next time the function is called the first parameter will be echoed
		back. */
		lParameterNumber = 1L;

		/* There is more data to be returned as no parameters have been echoed
		back yet. */
		xReturn = pdPASS;
	}
	else
	{
		/* Obtain the parameter string. */
		pcParameter = FreeRTOS_CLIGetParameter
							(
								pcCommandString,		/* The command string itself. */
								lParameterNumber,		/* Return the next parameter. */
								&xParameterStringLength	/* Store the parameter string length. */
							);

		if( pcParameter != NULL )
		{
			/* Return the parameter string. */
			memset( pcWriteBuffer, 0x00, xWriteBufferLen );
			sprintf( pcWriteBuffer, "%d: ", ( int ) lParameterNumber );
			strncat( pcWriteBuffer, pcParameter, xParameterStringLength );
			strncat( pcWriteBuffer, "\r\n", strlen( "\r\n" ) );

			/* There might be more parameters to return after this one. */
			xReturn = pdTRUE;
			lParameterNumber++;
		}
		else
		{
			/* No more parameters were found.  Make sure the write buffer does
			not contain a valid string. */
			pcWriteBuffer[ 0 ] = 0x00;

			/* No more data to return. */
			xReturn = pdFALSE;

			/* Start over the next time this command is executed. */
			lParameterNumber = 0;
		}
	}

	return xReturn;
}

/*-----------------------------------------------------------*/
#if configINCLUDE_TRACE_RELATED_CLI_COMMANDS == 1

	static portBASE_TYPE prvStartStopTraceCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
	{
	const char *pcParameter;
	portBASE_TYPE lParameterStringLength;

		/* Remove compile time warnings about unused parameters, and check the
		write buffer is not NULL.  NOTE - for simplicity, this example assumes the
		write buffer length is adequate, so does not check for buffer overflows. */
		( void ) pcCommandString;
		( void ) xWriteBufferLen;
		configASSERT( pcWriteBuffer );

		/* Obtain the parameter string. */
		pcParameter = FreeRTOS_CLIGetParameter
							(
								pcCommandString,		/* The command string itself. */
								1,						/* Return the first parameter. */
								&lParameterStringLength	/* Store the parameter string length. */
							);

		/* Sanity check something was returned. */
		configASSERT( pcParameter );

		/* There are only two valid parameter values. */
		if( strncmp( pcParameter, "start", strlen( "start" ) ) == 0 )
		{
			/* Start or restart the trace. */
			vTraceStop();
			vTraceClear();
			vTraceStart();

			sprintf( pcWriteBuffer, "Trace recording (re)started.\r\n" );
		}
		else if( strncmp( pcParameter, "stop", strlen( "stop" ) ) == 0 )
		{
			/* End the trace, if one is running. */
			vTraceStop();
			sprintf( pcWriteBuffer, "Stopping trace recording.\r\n" );
		}
		else
		{
			sprintf( pcWriteBuffer, "Valid parameters are 'start' and 'stop'.\r\n" );
		}

		/* There is no more data to return after this single string, so return
		pdFALSE. */
		return pdFALSE;
	}

#endif /* configINCLUDE_TRACE_RELATED_CLI_COMMANDS */


/*-----------------------------------------------------------*/
static BaseType_t prvMCANStats( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{

    portBASE_TYPE xReturn;
    static portBASE_TYPE lParameterNumber = 0;

	const char * const pcHeader = "\r\nCAN_ID\tTIC\t\tELAP\tinQ\tFLT\r\n****************************************\r\n";
	const char * const pcFooter = "\r\n";

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	memset( pcWriteBuffer, 0x00, xWriteBufferLen );

	const char str1[80];

	can_rx_queue_data_t this_rx_data;

	uint8_t mcan_q_isEmpty= 0;

	if( lParameterNumber == 0 )
	{
		sprintf( pcWriteBuffer, "%s", pcHeader);

		/* Next time the function is called the first parameter will be echoed
		back. */
		lParameterNumber++;

		/* There is more data to be returned as no parameters have been echoed
		back yet. */
		xReturn = pdPASS;
	}
	else if ((lParameterNumber >=1)&&(lParameterNumber<=MAX_DEBUG_CAN_RX_QUEUE_ITEMS)) {

		mcan_q_isEmpty  = mcan_q_getItem(&rx_can_queue, &this_rx_data);

		if (!mcan_q_isEmpty) {
			sprintf(str1, "0x%X\t%6d\t%4d\t%02\t%01d\r\n",
			this_rx_data.msg_id,
			this_rx_data.rvcd_tic,
			this_rx_data.tic_elapsed,
			this_rx_data.inQ,
			this_rx_data.flt);
			strncat(pcWriteBuffer,str1,strlen(str1));
		}

		/* Next time the function is called the first parameter will be echoed
		back. */
		lParameterNumber++;

		xReturn = pdPASS;

	} else if (lParameterNumber>MAX_DEBUG_CAN_RX_QUEUE_ITEMS) {

		sprintf( pcWriteBuffer, "%s", pcFooter);
		lParameterNumber = 0;
		xReturn = pdFALSE;
	}

	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return xReturn;

}//end

/*-----------------------------------------------------------*/
static BaseType_t prvADCStats( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{

    portBASE_TYPE xReturn;
    static portBASE_TYPE lParameterNumber = 0;
	const char *pcParameter;
	portBASE_TYPE lParameterStringLength;
	uint8_t xRawScaled =0;

	const char * const pcHeader = "\r\n\tCNT\t\tDCDCI\tDCDCV\tILoad\tIShrt\tIBatt\tVBatt\tAltUnV\tTherm\r\n****************************************************************************\r\n";
	const char * const pcFooter = "\r\n";

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	memset( pcWriteBuffer, 0x00, xWriteBufferLen );

	const char str1[80];

	adc_queue_data_t this_adc_data;

	uint8_t adc_q_isEmpty= 0;

	/* Determine whether to print.cnts or scaled data */
	/* Obtain the parameter string. */
	pcParameter = FreeRTOS_CLIGetParameter
						(
							pcCommandString,		/* The command string itself. */
							1,						/* Return the first parameter. */
							&lParameterStringLength	/* Store the parameter string length. */
						);

	/* Sanity check something was returned. */
	//configASSERT( pcParameter );

	/* There are only two valid parameter values. */
	if( strncmp( pcParameter, "cnts", strlen( "cnts" ) ) == 0 )
	{
		xRawScaled = 1;
	}
	else if( strncmp( pcParameter, "mv", strlen( "mv" ) ) == 0 )
	{
		xRawScaled = 2;
	}

	else if( strncmp( pcParameter, "scale", strlen( "scale" ) ) == 0 )
	{
		xRawScaled = 3;
	}
	else
	{
		sprintf( pcWriteBuffer, "Valid parameters are 'cnts', 'mv', and 'scale', try again ...\r\n" );
		return pdFALSE;
	}

	if( lParameterNumber == 0 )
	{
		sprintf( pcWriteBuffer, "%s", pcHeader);

		/* Next time the function is called the first parameter will be echoed
		back. */
		lParameterNumber++;

		/* There is more data to be returned as no parameters have been echoed
		back yet. */
		xReturn = pdPASS;
	}
	else if ((lParameterNumber >=1)&&(lParameterNumber<=MAX_ADC_QUEUE_ITEMS)) {

		adc_q_isEmpty  = adc_q_getItem(&adc_queue, &this_adc_data);

		if (!adc_q_isEmpty) {

			//SHOWS RAW COUNTS
			if (xRawScaled==1) {
				sprintf(str1, "%8d\t%05d\t%05d\t%05d\t%05d\t%05d\t%05d\t%05d\t%05d\r\n",
					this_adc_data.cnt,
					this_adc_data.adc_data.ain0_DCDCImon.cnts,
					this_adc_data.adc_data.ain1_DCDCVmon.cnts,
					this_adc_data.adc_data.ain2_ILoadMeas.cnts,
					this_adc_data.adc_data.ain3_IShortMeas.cnts,
					this_adc_data.adc_data.ain4_IBattery.cnts,
					this_adc_data.adc_data.ain5_VBattery.cnts,
					this_adc_data.adc_data.ain6_AltUnregVmon.cnts,
					this_adc_data.adc_data.ain7_Thermistor.cnts
				);

			//SHOW MV
			} else if (xRawScaled ==2) {
				sprintf(str1, "%8d\t%5d\t%5d\t%5d\t%5d\t%5d\t%5d\t%5d\t%5d\r\n",
					this_adc_data.cnt,
					ADC_RAW_TO_MV(this_adc_data.adc_data.ain0_DCDCImon.cnts),
					ADC_RAW_TO_MV(this_adc_data.adc_data.ain1_DCDCVmon.cnts),
					ADC_RAW_TO_MV(this_adc_data.adc_data.ain2_ILoadMeas.cnts),
					ADC_RAW_TO_MV(this_adc_data.adc_data.ain3_IShortMeas.cnts),
					ADC_RAW_TO_MV(this_adc_data.adc_data.ain4_IBattery.cnts),
					ADC_RAW_TO_MV(this_adc_data.adc_data.ain5_VBattery.cnts),
					ADC_RAW_TO_MV(this_adc_data.adc_data.ain6_AltUnregVmon.cnts),
					ADC_RAW_TO_MV(this_adc_data.adc_data.ain7_Thermistor.cnts)
				);

			//SHOW SCALED
			} else if (xRawScaled ==3 ) {
				sprintf(str1, "%8d\t%5d\t%5d\t%5d\t%5d\t%5d\t%5d\t%5d\t%5d\r\n",
					this_adc_data.cnt,
					DCDC_IMON_COUNT_TO_MV(this_adc_data.adc_data.ain0_DCDCImon.cnts),      //mA
					DCDC_VMON_COUNT_TO_MV(this_adc_data.adc_data.ain1_DCDCVmon.cnts),      //mV
					ILOAD_COUNT_TO_MA(this_adc_data.adc_data.ain2_ILoadMeas.cnts),     //mA
					ISHORTCIR_MV_TO_MA		(ADC_RAW_TO_MV(this_adc_data.adc_data.ain3_IShortMeas.cnts)),    //mA
					IBAT_COUNT_TO_MV(this_adc_data.adc_data.ain4_IBattery.cnts),      //mA
					BATV_VMON_COUNT_TO_MV(this_adc_data.adc_data.ain5_VBattery.cnts),      //mV
					ALTV_VMON_COUNT_TO_MV(this_adc_data.adc_data.ain6_AltUnregVmon.cnts),  //mV
					THERM_MV_TO_DEGC		(ADC_RAW_TO_MV(this_adc_data.adc_data.ain7_Thermistor.cnts))     //deg C * 100
				);

			}

			strncat(pcWriteBuffer,str1,strlen(str1));
		}

		/* Next time the function is called the first parameter will be echoed
		back. */
		lParameterNumber++;

		xReturn = pdPASS;

	} else if (lParameterNumber>MAX_ADC_QUEUE_ITEMS) {

		sprintf( pcWriteBuffer, "%s", pcFooter);
		lParameterNumber = 0;
		xReturn = pdFALSE;
	}

	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return xReturn;

}//end

static BaseType_t prvGPIOStats( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{

    portBASE_TYPE xReturn;
    static portBASE_TYPE lParameterNumber = 0;
	const char *pcParameter;
	portBASE_TYPE lParameterStringLength;
	uint8_t xRawScaled =0;
	bool motorDir_level;
	bool output_level;
	bool battery_En_level;
	bool battery_Charge_level;
	
	const char * const pcHeader = "\r\n\tGPIO\t\tMotor_Dir\tOutput_En\tBatt_En\tBatt_Charge\r\n****************************************************************************\r\n";
	const char * const pcFooter = "\r\n";

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	memset( pcWriteBuffer, 0x00, xWriteBufferLen );

	const char str1[80] = "hello world";

	motorDir_level = port_pin_get_output_level(MOTOR_DIR);
	output_level = port_pin_get_output_level(OUTPUT_EN);
	battery_En_level = port_pin_get_output_level(BATTERY_EN);
	battery_Charge_level = port_pin_get_output_level(BATTERY_CHARGE_EN);

	sprintf( pcWriteBuffer, "%s", pcHeader);
	sprintf(str1, "\t\t\t\%8d\t%8d\t%8d\t%8d\r\n",
	              motorDir_level,
				  output_level,
				  battery_En_level,
				  battery_Charge_level);

	strncat(pcWriteBuffer,str1,strlen(str1));
	xReturn = pdFALSE;
	//uxParameterNumber = 0;

	

}//end



/*-----------------------------------------------------------*/
static BaseType_t prvPWMCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{

	const char *pcParameter;
	BaseType_t xParameterStringLength, xReturn;
	static UBaseType_t uxParameterNumber = 0;

	static pwm_cmd_t xPwmCmd = {
		.uxChId = 0,
		.uxDuty = 0,
		.uxOffOn = 0
	};

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	if( uxParameterNumber == 0 )
	{
		/* The first time the function is called after the command has been
		entered just a header string is returned. */
		sprintf( pcWriteBuffer, "The three PWM parameters (chID, off/on, duty_cycle) were:\r\n" );

		/* Next time the function is called the first parameter will be echoed
		back. */
		uxParameterNumber = 1U;

		/* There is more data to be returned as no parameters have been echoed
		back yet. */
		xReturn = pdPASS;
	}
	else
	{
		/* Obtain the parameter string. */
		pcParameter = FreeRTOS_CLIGetParameter
						(
							pcCommandString,		/* The command string itself. */
							uxParameterNumber,		/* Return the next parameter. */
							&xParameterStringLength	/* Store the parameter string length. */
						);

		/* Sanity check something was returned. */
		configASSERT( pcParameter );

		/* Return the parameter string. */
		memset( pcWriteBuffer, 0x00, xWriteBufferLen );
		sprintf( pcWriteBuffer, "%d: ", ( int ) uxParameterNumber );
		strncat( pcWriteBuffer, pcParameter, ( size_t ) xParameterStringLength );
		strncat( pcWriteBuffer, "\r\n", strlen( "\r\n" ) );

		/* parse out command value */
		if (uxParameterNumber==1) xPwmCmd.uxChId  = atoi(pcParameter);		/*the pwm channel number */

		/* parse out on/off command */
		if (uxParameterNumber==2) {
			/* There are only two valid parameter values. */
			if( strncmp( pcParameter, "on", strlen( "on" ) ) == 0 ) {
				xPwmCmd.uxOffOn = 1;
			} else if( strncmp( pcParameter, "off", strlen( "off" ) ) == 0 ) {
				xPwmCmd.uxOffOn = 0;
				/*send command to PWM task*/
				if (!xQueueSend(xPwmQHndle, &xPwmCmd, 1000)){
					printf	("\r\nfail to send to xPwmQHndle queue\n");
				}
				/*no need to do last parameter, so exit here */
				return pdFALSE;
			}
			else {
				sprintf( pcWriteBuffer, "Valid parameters are 'on and 'off', try again ...\r\n" );
				xPwmCmd.uxOffOn = 0;
				return pdFALSE;
			}	
		}//uxParameterNumber==2

		if (uxParameterNumber==3) xPwmCmd.uxDuty  = atoi(pcParameter);		/*the duty cycle 0-100 for this channel */

		/* If this is the last of the three parameters then there are no more
		strings to return after this one. */
		if( uxParameterNumber == 3U )
		{
			/* If this is the last of the three parameters then there are no more
			strings to return after this one. */
			xReturn = pdFALSE;
			uxParameterNumber = 0;

			/* now, do a simple check to see if the parameters are valid */
			UBaseType_t uxOK = 0;
			if ((xPwmCmd.uxChId == 1)||(xPwmCmd.uxChId == 2)) {
				if ((xPwmCmd.uxOffOn==0)||(xPwmCmd.uxOffOn==1)) {
					if ((xPwmCmd.uxDuty>=0)&&(xPwmCmd.uxDuty<=100)){
						uxOK = 1;
					}//end if 
				}// end if 
			}//end if 

			if (uxOK==1) {
				strncat( pcWriteBuffer, "PWM Command Valid!\r\n", strlen( "PWM Command Valid!\r\n" ) );
				/*send command to PWM task*/
				if (!xQueueSend(xPwmQHndle, &xPwmCmd, 1000)){
					printf	("\r\nfail to send to xPwmQHndle queue\n");
				}
			} else {
				strncat( pcWriteBuffer, "PWM Command INVALID !!!!\r\n", strlen( "PWM Command INVALID !!!!\r\n" ) );
			}//end if 

		}
		else
		{
			/* There are more parameters to return after this one. */
			xReturn = pdTRUE;
			uxParameterNumber++;
		}
	}

	return xReturn;

}//end

/*-----------------------------------------------------------*/
static BaseType_t prvI2CDACCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{

	const char *pcParameter;
	BaseType_t xParameterStringLength, xReturn;
	static UBaseType_t uxParameterNumber = 0;
	static UBaseType_t uxOK = 0;

	static I2CDac_cmd_t xI2CDacCmd = {
		.uxChId   = 0,
		.uxCounts = 0
	};

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	if( uxParameterNumber == 0 ) {
		/* The first time the function is called after the command has been
		entered just a header string is returned. */
		sprintf( pcWriteBuffer, "The two I2C_DAC parameters (chID, counts) were:\r\n" );

		/* Next time the function is called the first parameter will be echoed
		back. */
		uxParameterNumber = 1U;

		/* There is more data to be returned as no parameters have been echoed
		back yet. */
		xReturn = pdPASS;
	} else {
		/* Obtain the parameter string. */
		pcParameter = FreeRTOS_CLIGetParameter
						(
							pcCommandString,		/* The command string itself. */
							uxParameterNumber,		/* Return the next parameter. */
							&xParameterStringLength	/* Store the parameter string length. */
						);

		/* Sanity check something was returned. */
		configASSERT( pcParameter );

		/* Return the parameter string. */
		memset( pcWriteBuffer, 0x00, xWriteBufferLen );
		sprintf( pcWriteBuffer, "%d: ", ( int ) uxParameterNumber );
		strncat( pcWriteBuffer, pcParameter, ( size_t ) xParameterStringLength );
		strncat( pcWriteBuffer, "\r\n", strlen( "\r\n" ) );

		/* parse out command value */
		if (uxParameterNumber==1) xI2CDacCmd.uxChId  = atoi(pcParameter);		/* the channel number 1..4 */

		/* parse out on/off command */
		if (uxParameterNumber==2) xI2CDacCmd.uxCounts  = atoi(pcParameter);		/* the channel counts 0..4095 */
	
		/* If this is the last of the three parameters then there are no more
		strings to return after this one. */
		uxOK = 0;
		if( uxParameterNumber == 2 ){
			/* If this is the last of the three parameters then there are no more
			strings to return after this one. */
			xReturn = pdFALSE;
			uxParameterNumber = 0;
			/* now, do a simple check to see if the parameters are valid */
			if ((xI2CDacCmd.uxChId >= 1)&&(xI2CDacCmd.uxChId <= 4)) {
				if ((xI2CDacCmd.uxCounts>=0)&&(xI2CDacCmd.uxCounts<=4095)) {
						uxOK = 1;
				}//end if 
			}//end if 

			if (uxOK==1) {
				strncat( pcWriteBuffer, "I2C_DAC Command Valid!\r\n", strlen( "I2C_DAC Command Valid!\r\n" ) );
				/*send command to I2C_DAC task*/
				if (!xQueueSend(xI2CDacQHndle, &xI2CDacCmd, 1000)){
					printf	("\r\nfail to send to xI2CDacQHndle queue\n");
				}
			} else {
				strncat( pcWriteBuffer, "I2C_DAC Command INVALID !!!!\r\n", strlen( "I2C_DAC Command INVALID !!!!\r\n" ) );
			}//end if 

		} else
		{
			/* There are more parameters to return after this one. */
			xReturn = pdTRUE;
			uxParameterNumber++;
		}
	}

	return xReturn;

}//end

/*-----------------------------------------------------------*/
static BaseType_t prvSetModeCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{

	const char *pcParameter;
	BaseType_t xParameterStringLength, xReturn;
	static UBaseType_t uxParameterNumber = 0;
	uint8_t cmd_mode =0; 

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	if( uxParameterNumber == 0 ) {
		/* The first time the function is called after the command has been
		entered just a header string is returned. */
		sprintf( pcWriteBuffer, "The requested operating mode is (1=AUTO, 0=MANUAL) is \r\n" );

		/* Next time the function is called the first parameter will be echoed
		back. */
		uxParameterNumber = 1U;

		/* There is more data to be returned as no parameters have been echoed
		back yet. */
		xReturn = pdPASS;
	} else {
		/* Obtain the parameter string. */
		pcParameter = FreeRTOS_CLIGetParameter
						(
							pcCommandString,		/* The command string itself. */
							uxParameterNumber,		/* Return the next parameter. */
							&xParameterStringLength	/* Store the parameter string length. */
						);

		/* Sanity check something was returned. */
		configASSERT( pcParameter );

		/* Return the parameter string. */
		memset( pcWriteBuffer, 0x00, xWriteBufferLen );
		sprintf( pcWriteBuffer, "%d: ", ( int ) uxParameterNumber );
		strncat( pcWriteBuffer, pcParameter, ( size_t ) xParameterStringLength );
		strncat( pcWriteBuffer, "\r\n", strlen( "\r\n" ) );

		/* parse out command  */
		if( uxParameterNumber == 1 ){
		    cmd_mode  = atoi(pcParameter);		/* the mode */
			if ((cmd_mode==0)||(cmd_mode==1)){
				//set mode
				gbl_PwrStatusFlags.auto_man = cmd_mode;
			}//end if 
			if (cmd_mode==3){  //toggle monitor mode
				//set mode
				gbl_configType.monitor_mode = !(gbl_configType.monitor_mode);
			}//end if 
			xReturn = pdFALSE;
			uxParameterNumber = 0;		
		} 
		else
		{
			/* There are more parameters to return after this one. */
			xReturn = pdTRUE;
			uxParameterNumber++;
		}
	}

	return xReturn;

}//end

/*-----------------------------------------------------------*/
static BaseType_t prvSetOutputCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{

	const char *pcParameter;
	BaseType_t xParameterStringLength, xReturn;

	static UBaseType_t uxParameterNumber = 0;
	static UBaseType_t uxOK = 0;
    static UBaseType_t uxOutputNumber = 0;
	static UBaseType_t uxOutputState= 0;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	if( uxParameterNumber == 0 ) {
		/* The first time the function is called after the command has been
		entered just a header string is returned. */
		sprintf( pcWriteBuffer, "Set Output Number \r\n" );

		/* Next time the function is called the first parameter will be echoed
		back. */
		uxParameterNumber = 1U;

		/* There is more data to be returned as no parameters have been echoed
		back yet. */
		xReturn = pdPASS;
	} else {
		/* Obtain the parameter string. */
		pcParameter = FreeRTOS_CLIGetParameter
						(
							pcCommandString,		/* The command string itself. */
							uxParameterNumber,		/* Return the next parameter. */
							&xParameterStringLength	/* Store the parameter string length. */
						);

		/* Sanity check something was returned. */
		configASSERT( pcParameter );

		/* Return the parameter string. */
		memset( pcWriteBuffer, 0x00, xWriteBufferLen );
		sprintf( pcWriteBuffer, "%d: ", ( int ) uxParameterNumber );
		strncat( pcWriteBuffer, pcParameter, ( size_t ) xParameterStringLength );
		strncat( pcWriteBuffer, "\r\n", strlen( "\r\n" ) );

		/* parse out output number to be commanded */
		if (uxParameterNumber==1) uxOutputNumber = atoi(pcParameter);		/* the output index */

		/* parse out state to set output 1 or 0 */
		if (uxParameterNumber==2) uxOutputState  = atoi(pcParameter);		/* the state */
	
		if( uxParameterNumber == 2 ){

			//quick check for valid number
			if ((uxOutputNumber>=0)&&(uxOutputNumber<5)) {

				//quick check for valid state
				if ((uxOutputState==0)||(uxOutputState==1)){
					
					switch(uxOutputNumber) {

						/*ALL OFF */
						case 0:
							port_pin_set_output_level(BATTERY_CHARGE_EN, BATTERY_CHARGE_EN_INACTIVE);
							port_pin_set_output_level(MOTOR_DIR, MOTOR_DIR_INACTIVE);
							port_pin_set_output_level(OUTPUT_EN, OUTPUT_EN_INACTIVE);
							port_pin_set_output_level(BATTERY_EN, BATTERY_EN_INACTIVE);
							strncat( pcWriteBuffer, "SET ALL OUTPUTS OFF ...\r\n", strlen( "SET ALL OUTPUTS OFF ...\r\n" ) );
						break;

						/*Motor DIR (OUTPUT), PB23*/
						case 1:
							if (uxOutputState == 1) {
								port_pin_set_output_level(MOTOR_DIR, MOTOR_DIR_ACTIVE);
								strncat( pcWriteBuffer, "PB23 MOTOR DIR ACTIVE ...\r\n", strlen( "PB23 MOTOR DIR ACTIVE ...\r\n" ) );
							} else {
								port_pin_set_output_level(MOTOR_DIR, MOTOR_DIR_INACTIVE);
								strncat( pcWriteBuffer, "PB23 MOTOR DIR INACTIVE ...\r\n", strlen( "PB23 MOTOR DIR INACTIVE ...\r\n" ) );
							}//end if
							break; 

						/*Output EN (OUTPUT), PA27*/
						case 2:
							if (uxOutputState == 1) {
								port_pin_set_output_level(OUTPUT_EN, OUTPUT_EN_ACTIVE);
								strncat( pcWriteBuffer, "PA27 OUTPUT_EN ACTIVE ...\r\n", strlen( "PA27 OUTPUT_EN ACTIVE ...\r\n" ) );
								} else {
								port_pin_set_output_level(OUTPUT_EN, OUTPUT_EN_INACTIVE);
								strncat( pcWriteBuffer, "PA27 OUTPUT_EN INACTIVE ...\r\n", strlen( "PA27 OUTPUT_EN INACTIVE ...\r\n" ) );
							}//end if					
							break; 

						/*Battery EN (OUTPUT), PB30 */
						case 3:
							if (uxOutputState == 1) {
								port_pin_set_output_level(BATTERY_EN, BATTERY_EN_ACTIVE);
								strncat( pcWriteBuffer, "PB30 BATTERY_EN ACTIVE ...\r\n", strlen( "PB30 BATTERY_EN ACTIVE ...\r\n" ) );
								} else {
								port_pin_set_output_level(BATTERY_EN, BATTERY_EN_INACTIVE);
								strncat( pcWriteBuffer, "PB30 BATTERY_EN INACTIVE ...\r\n", strlen( "PB30 BATTERY_EN INACTIVE ...\r\n" ) );
							}//end if						
						break;	
						
						/*Battery RUN/CHARGE (OUTPUT), PB31 */
						case 4:
							if (uxOutputState == 1) {
								port_pin_set_output_level(BATTERY_CHARGE_EN, BATTERY_CHARGE_EN_ACTIVE);
								strncat( pcWriteBuffer, "PB31 BATTERY_CHARGE_EN ACTIVE ...\r\n", strlen( "PB31 BATTERY_CHARGE_EN ACTIVE ...\r\n" ) );
								} else {
								port_pin_set_output_level(BATTERY_CHARGE_EN, BATTERY_CHARGE_EN_INACTIVE);
								strncat( pcWriteBuffer, "PB31 BATTERY_CHARGE_EN INACTIVE ...\r\n", strlen( "PB31 BATTERY_CHARGE_EN INACTIVE ...\r\n" ) );
							}//end if						
						break;	
																
						/*ALL OFF */
						default:
							port_pin_set_output_level(BATTERY_CHARGE_EN, BATTERY_CHARGE_EN_INACTIVE);
							port_pin_set_output_level(MOTOR_DIR, MOTOR_DIR_INACTIVE);
							port_pin_set_output_level(OUTPUT_EN, OUTPUT_EN_INACTIVE);
							port_pin_set_output_level(BATTERY_EN, BATTERY_EN_ACTIVE);
							strncat( pcWriteBuffer, "CASE DEFAULT ALL OUTPUTS OFF ...\r\n", strlen( "CASE DEFAULT OUTPUTS OFF ...\r\n" ) );
						break;

					}//end switch
					
				}//end if 

			}//end if 
			
			xReturn = pdFALSE;
			uxParameterNumber = 0;

		} else
		{
			/* There are more parameters to return after this one. */
			xReturn = pdTRUE;
			uxParameterNumber++;
		}
	}
	
	return xReturn;

}//end

static portBASE_TYPE prvEStopCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
	const char *const pcHeader = "\r\n!!!!!!!!!!!!!!!!  ESTOP  !!!!!!!!!!!!!!!!!!!!!!!\r\n\r\n";
	const char str1[10];

	static I2CDac_cmd_t xI2CDacCmd = {
		.uxChId   = 0,
		.uxCounts = 0
	};
	static pwm_cmd_t xPwmCmd = {
		.uxChId = 0,
		.uxDuty = 0,
		.uxOffOn = 0
	};

	uint8_t i;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	( void ) pcCommandString;
	( void ) xWriteBufferLen;
	configASSERT( pcWriteBuffer );

	/* Generate a table of task stats. */
	strcpy( pcWriteBuffer, pcHeader );

	port_pin_set_output_level(BATTERY_CHARGE_EN, BATTERY_CHARGE_EN_INACTIVE);
	port_pin_set_output_level(MOTOR_DIR, MOTOR_DIR_INACTIVE);
	port_pin_set_output_level(OUTPUT_EN, OUTPUT_EN_INACTIVE);
	port_pin_set_output_level(BATTERY_EN, OUTPUT_EN_INACTIVE);
	
	/*sets all DAC channels to zero */
	for (i=0;i<4;i++)
	{
		xI2CDacCmd.uxChId = i;
		xI2CDacCmd.uxCounts = 0;
		/*send command to I2C_DAC task*/
		if (!xQueueSend(xI2CDacQHndle, &xI2CDacCmd, 1000)){
			printf	("\r\nfail to send to xI2CDacQHndle queue\n");
		}
	}

	/*set PWM to zero */
	xPwmCmd.uxChId = 0;
	xPwmCmd.uxDuty = 0;
	xPwmCmd.uxOffOn = 0;
	/*send command to PWM task*/
	if (!xQueueSend(xPwmQHndle, &xPwmCmd, 1000)){
		printf	("\r\nfail to send to xPwmQHndle queue\n");
	}

	//set mode to MANUAL
	gbl_PwrStatusFlags.auto_man = 0;

	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}



