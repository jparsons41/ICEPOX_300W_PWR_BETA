/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# Minimal main function that starts with a call to system_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */


 /* Board/Hardware includes */
 #include "asf.h"
 #include "stdio_serial.h"
 #include "conf_uart_serial.h"
 #include "conf_application.h"

 /* FreeRTOS includes. */
 #include "FreeRTOS.h"
 #include "task.h"
 #include "timers.h"
 #include "queue.h"
 #include "semphr.h"
 #include "UARTCommandConsole.h"

 /* User Application Includes */
 #include "cntrl_GLOBALS.h"
 #include "cntrl_DISCRETE_IO.h"
 #include "cntrl_MCAN.h"
 #include "cntrl_ADC.h"
 #include "cntrl_PWM.h"
 #include "cntrl_I2C_DAC.h"
 #include "cntrl_EXTINT.h"
 #include "cntrl_CONTROL_LOOP.h"
 #include "cntrl_BLDC_CONTROLLER.h"

 /*-----------------------------------------------------------*/
 /*				     FORWARD DECLERATIONS                     */
 /*-------------------_---------------------------------------*/

//this is the console usart module
static struct usart_module cdc_uart_module;

/*Forward declaration for task Heartbeat*/
static void xTaskHeartbeat(void *pvParameters);

/*
 * Perform any application specific hardware configuration.
 */
static void prvSetupHardware( void );

/*
 * Prototypes for the FreeRTOS hook/callback functions.  See the comments in
 * the implementation of each function for more information.
 */
void vApplicationMallocFailedHook( void );
void vApplicationIdleHook( void );
void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName );
void vApplicationTickHook( void );

/* Used in the run time stats calculations. */
static unsigned long ulClocksPer10thOfAMilliSecond = 0UL;

/**
 *  Configure UART console.
 */
static void configure_console(void)
{
	struct usart_config usart_conf;

	usart_get_config_defaults(&usart_conf);
	/*see conf_uart_serial.h*/
	usart_conf.mux_setting = CONF_STDIO_MUX_SETTING;
	usart_conf.pinmux_pad0 = CONF_STDIO_PINMUX_PAD0;
	usart_conf.pinmux_pad1 = CONF_STDIO_PINMUX_PAD1;
	usart_conf.pinmux_pad2 = CONF_STDIO_PINMUX_PAD2;
	usart_conf.pinmux_pad3 = CONF_STDIO_PINMUX_PAD3;
	usart_conf.baudrate    = CONF_STDIO_BAUDRATE;

	stdio_serial_init(&cdc_uart_module, CONF_STDIO_USART_MODULE, &usart_conf);
	usart_enable(&cdc_uart_module);

}//configure_console

/**
 *  Setup Hardware (done at beginning of Main)
 */
static void prvSetupHardware( void )
{

	/* Initialization is performed by the Atmel board support package. */
	system_init();

	/*Configure UART console.*/
	configure_console();

	/*Initialize the delay driver*/
	delay_init();

	/* Output example information */
	puts(STRING_HEADER);

	#if (BLDC_CONTROLLER_SET_CONFIGS)
		bldc_controller_config();
		bldc_controller_set_cfg();
	#endif

	/*Enable system interrupt*/
	system_interrupt_enable_global();

	/*Init board Discrete IO*/
	init_DisreteIO();

	//Init globals to safe value//
	initGlobalsSafe();

	///*Configure CAN driver*/
	#if (TASK_MCAN_ENABLED)
		printf("-- INIT - Enable MCAN Driver ...\n\r") ;
		/*Initialize the MCAN driver, see cntrl_MCAN.c */
		configure_mcan();
		/*set the standard filter 0*/
		printf ("-- INIT - Set MCAN fifo_0 filter x%X to x%X ...\n\r",MCAN_RX_FILTER_0_ID_LOW,MCAN_RX_FILTER_0_ID_HI);
		mcan_set_standard_filter_0();
	#endif

	#if (PROCESS_ADC_ENABLED)
		printf("-- INIT - Enable ADC (SAR) Driver for the MAP Sensor ...\n\r") ;
		/*Initialize the ADC driver, see cntrl_ADC.c */
		configure_adc();
		/*register and enable the ADC callbacks*/
		configure_adc_callbacks();
		//For debug only, show ADC scaling test ...
		//May be used to compare reference counts against spreadsheet
		#if (ADC_SCALING_DEBUG)
			adcScalingDebugCheck();
		#endif
	#endif

	#if (TASK_PWM_ENABLED)
		printf( "-- INIT - Instantiate PWM Driver  ...\r\n" );
		/*configure TTC for the motor PWM*/
		configure_tcc();
	#endif

	#if (TASK_I2C_DAC_ENABLED)
		printf( "-- INIT - Instantiate I2C_DAC Driver  ...\r\n" );
		/*configure I2C_DAC Driver*/
		configure_i2c_master();
		//For debug only, showDAC scaling test ...
		//May be used to compare reference counts against spreadsheet
		#if (DAC_SCALING_DEBUG)
			dacScalingDebugCheck();
		#endif
	#endif

	#if (TASK_CONTROL_LOOP_ENABLED)
		printf( "-- INIT - Init the CONTROL LOOP  ...\r\n" );
		xInitCntrlLoop();
	#endif

	//ready, set, go into FreeRTOS ...

}//prvSetupHardwar


/*-----------------------------------------------------------*/
/*                          MAIN                             */
/*-----------------------------------------------------------*/
int main (void)
{

	/* Initialize the System*/
	prvSetupHardware();

	uint32_t check_clk = system_clock_source_get_hz( SYSTEM_CLOCK_SOURCE_DPLL );
	printf( "-- MAIN - System Clock is %d Hz ...\r\n", check_clk );

	/*starts the heartbeat task*/
	#if (TASK_HEARTBEAT_ENABLED)
		/* Create task to create heartbeat */
		printf( "-- MAIN - Starting Heartbeat Task ...\r\n" );
		/*create heartbeat task - good place for periodic functions*/
		if (xTaskCreate(xTaskHeartbeat, "Heartbeat", HEARTBEAT_TASK_STACK_SIZE, NULL,
			HEARTBEAT_TASK_PRIORITY, NULL) != pdPASS) {
		    printf("-- MAIN - Failed to create xHeartbeat ...\r\n");
		}//end xTaskCreate xTaskHeartbeat
	#endif

	/*starts two tasks for CAN TX/RX message processing*/
	#if (TASK_MCAN_ENABLED)
		//* create the test MCAN TX task */
		printf( "-- MAIN - Create MCAN_TX Task  ...\r\n" );
		if (xTaskCreate(xTaskMCAN_TX, "MCAN_TX", CAN_TX_TASK_STACK_SIZE, NULL,
			CAN_TX_TASK_PRIORITY, NULL) != pdPASS) {
			printf("-- MAIN - Failed to create xTaskMCAN_TX...\r\n");
		}//end
		printf( "-- MAIN - Create MCAN_RX Task  ...\r\n" );
		/* create the MCAN_RX_Handler for deferred interrupt processing*/
		if (xTaskCreate(xTaskMCAN_RX_Handler, "MCAN_RX", CAN_RX_TASK_STACK_SIZE, NULL,
		CAN_RX_TASK_PRIORITY, NULL) != pdPASS) {
			printf("-- MAIN - Failed to create xTaskMCAN_RX_Handler...\r\n");
		}//end
	#endif

	#if (TASK_I2C_DAC_ENABLED)
		printf( "-- MAIN - Create I2C DAC Task  ...\r\n" );
		/* create the task for deferred interrupt processing*/
		if (xTaskCreate(xTaskI2CDac, "I2C_DAC", I2C_DAC_TASK_STACK_SIZE, NULL,
			I2C_DAC_TASK_PRIORITY, NULL) != pdPASS) {
			printf("-- MAIN - Failed to create xTaskI2CDac ...\r\n");
		}//end
	#endif

	#if (TASK_PWM_ENABLED)
		printf( "-- MAIN - Create PWM Task  ...\r\n" );
		/* create the task ADC_RX for deferred interrupt processing*/
		if (xTaskCreate(xTaskPwm, "PWM", PWM_TASK_STACK_SIZE, NULL,
			PWM_TASK_PRIORITY, NULL) != pdPASS) {
			printf("-- MAIN - Failed to create xTaskPWM ...\r\n");
		}//end
	#endif

	/*starts the process for handling the under voltage trip interrupt*/
	#if (PROCESS_UNDERVOLTAGE_TRIP_ENABLED)
		configure_extint_channel();
		/*configure the callback function*/
		configure_extint_callbacks();
	#endif

		/*starts the main CONTROL LOOP TASK*/
	#if (TASK_CONTROL_LOOP_ENABLED)
		printf( "-- MAIN - Create the CONTROL LOOP Task ...\r\n" );
		/* create the task CONTROL_LOOP*/
		if (xTaskCreate(xTaskControlLoop, "CNTRL", CONTROL_LOOP_TASK_STACK_SIZE, NULL,
			CONTROL_LOOP_TASK_PRIORITY, NULL) != pdPASS) {
			printf("-- MAIN - Failed to create xControlLoop ...\r\n");
		}//end
	#endif

	/*starts the serial Command Line Interface (in this app, for debugging only)*/
	#if (TASK_CLI_ENABLED)
		printf( "-- MAIN - Start the  UART Console CLI ...\r\n" );
		/*yield back uart to the CLI task, will be setup again in here ...*/
		usart_disable(&cdc_uart_module);
		/* Create the task to start the UART Console for the CLI */
		vUARTCommandConsoleStart(CLI_TASK_STACK_SIZE, CLI_TASK_PRIORITY);
	#endif

	/* Start the FreeRTOS scheduler. */
	vTaskStartScheduler();

	/* If all is well, the scheduler will now be running, and the following
	line will never be reached.  If the following line does execute, then
	there was insufficient FreeRTOS heap memory available for the idle and/or
	timer tasks	to be created.  See the memory management section on the
	FreeRTOS web site for more details.  http://www.freertos.org/a00111.html */
	for( ;; )
	{

		/* "I cannot help it - in spite of myself, infinity torments me." - Alfred de Musset */

	}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;

}

/*-----------------------------------------------------------*/
/*                   FREERTOS CALLBACKS                      */
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/
void vApplicationMallocFailedHook( void )
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/
void vApplicationIdleHook( void )
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If the application makes use of the
	vTaskDelete() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */
}
/*-----------------------------------------------------------*/
void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	 * identify which task has overflowed its stack.
	 */
    printf(("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n"));
	printf(("!!!!!! FreeRTOS Stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName));
    printf(("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n"));

	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/
void vApplicationTickHook( void )
{
	/* This function will be called by each tick interrupt if
	configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
	added here, but the tick hook is called from an interrupt context, so
	code must not attempt to block, and only the interrupt safe FreeRTOS API
	functions can be used (those that end in FromISR()). */

}
/*-----------------------------------------------------------*/
void vMainConfigureTimerForRunTimeStats( void )
{
	/* Used by the optional run-time stats gathering functionality. */

	/* How many clocks are there per tenth of a millisecond? */
	ulClocksPer10thOfAMilliSecond = configCPU_CLOCK_HZ / 10000UL;
}

/*-----------------------------------------------------------*/
#if (configGENERATE_RUN_TIME_STATS)
	/* gwg - required for getRunTimeStats */
	unsigned long ulMainGetRunTimeCounterValue( void )
	{
		unsigned long ulSysTickCounts, ulTickCount, ulReturn;
		const unsigned long ulSysTickReloadValue = ( configCPU_CLOCK_HZ / configTICK_RATE_HZ ) - 1UL;
		volatile unsigned long * const pulCurrentSysTickCount = ( ( volatile unsigned long *) 0xe000e018 );
		volatile unsigned long * const pulInterruptCTRLState = ( ( volatile unsigned long *) 0xe000ed04 );
		const unsigned long ulSysTickPendingBit = 0x04000000UL;

		/* Used by the optional run-time stats gathering functionality. */

		/* NOTE: There are potentially race conditions here.  However, it is used
		anyway to keep the examples simple, and to avoid reliance on a separate
		timer peripheral. */

		/* The SysTick is a down counter.  How many clocks have passed since it was
		last reloaded? */
		ulSysTickCounts = ulSysTickReloadValue - *pulCurrentSysTickCount;

		/* How many times has it overflowed? */
		ulTickCount = xTaskGetTickCountFromISR();

		/* This is called from the context switch, so will be called from a
		critical section.  xTaskGetTickCountFromISR() contains its own critical
		section, and the ISR safe critical sections are not designed to nest,
		so reset the critical section. */
		portSET_INTERRUPT_MASK_FROM_ISR();

		/* Is there a SysTick interrupt pending? */
		if( ( *pulInterruptCTRLState & ulSysTickPendingBit ) != 0UL )
		{
			/* There is a SysTick interrupt pending, so the SysTick has overflowed
			but the tick count not yet incremented. */
			ulTickCount++;

			/* Read the SysTick again, as the overflow might have occurred since
			it was read last. */
			ulSysTickCounts = ulSysTickReloadValue - *pulCurrentSysTickCount;
		}

		/* Convert the tick count into tenths of a millisecond.  THIS ASSUMES
		configTICK_RATE_HZ is 1000! */
		ulReturn = ( ulTickCount * 10UL ) ;

		/* Add on the number of tenths of a millisecond that have passed since the
		tick count last got updated. */
		ulReturn += ( ulSysTickCounts / ulClocksPer10thOfAMilliSecond );

		return ulReturn;
	}
#endif


/*-----------------------------------------------------------*/
/*                   FREERTOS TASKS                          */
/*-----------------------------------------------------------*/
/**
 * \xTaskHeartbeat when activated, make LED blink at a fixed rate, or other fixed-time-interval stuff
 */
static void xTaskHeartbeat(void *pvParameters)
{

	BaseType_t  xTest_CAN_msg_cntr = 0;
	BaseType_t  xDelay_done = 0;
	BaseType_t  xTest_CAN_cnt =0;

	UNUSED(pvParameters);

	static uint8_t test = 0;

	for (;;) {

		/*this block of code used to test MCAN TX messages, sends a dummy value */
		//#if (TASK_MCAN_ENABLED)
			//xTest_CAN_msg_cntr++;
			//if ((xTest_CAN_msg_cntr>1)&&(!xDelay_done)) {
				//xDelay_done = 1;
				//xTest_CAN_msg_cntr = 0;
			//}
			//xTest_CAN_cnt++;
			//if (xDelay_done) {
				///*send command to test CAN task*/
  				//if (!xQueueSendToBack(xMCANTXQHndle, &xTest_CAN_msg_cntr, 100)){
					//printf("!! xTaskBlinky - Fail to send to queue xMCANTXQHndle...\n");
				//}//end
				//xTest_CAN_cnt = 0;
			//}
		//#endif

		/*toggle the LEDs*/
		//if (!test){
		     //LED_Toggle(LED0);
			 //test = 1;
        //} else {
			 //LED_Toggle(LED1);
			 //test = 0;
		//}
		//printf("-- DEBUG %d \n\r",test) ;

		printf("xTaskHeartbeat - Data Fault Code %d \r\n", gbl_PwrCmd.stats.fault);

		/*delay till next time*/
		vTaskDelay(LED_TOGGLE_PERIOD);

	}//end for

	/* for completeness, this should never happen */
	vTaskDelete( NULL );

}//end xTaskBlinky
