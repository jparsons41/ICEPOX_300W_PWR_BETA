/*
 * cntrl_PWM.h
 *
 * Created: 3/15/2017 12:03:35 PM
 *  Author: Gerald
 */ 


#ifndef CNTRL_PWM_H_
#define CNTRL_PWM_H_

 #include <asf.h>
 #include "status_codes.h"

 /* FreeRTOS includes. */
 #include "FreeRTOS.h"
 #include "task.h"
 #include "timers.h"
 #include "queue.h"
 #include "semphr.h"

/*IMPORTANT NOTE ABOUT PIN MUXING!!*
// see http://www.atmel.com/Images/Atmel-42256-SAM-Timer-Counter-for-Control-Applications-TCC-Driver_ApplicationNote_AT07058.pdf
// page 15/16.
/*      PWM_CHANNEL = 0 OUTPUT only on WO[0] or WO [4]. */
/*      PWM_CHANNEL = 1 OUTPUT only on WO[1] or WO [5]. */
/*      PWM_CHANNEL = 2 OUTPUT only on WO[2] or WO [6]. */
/*      PWM_CHANNEL = 3 OUTPUT only on WO[3] or WO [7]. */
	

#define CONF_PWM_MODULE      TCC0		
/** PWM channel */
#define CONF_PWM_CHANNEL     2		
/** PWM output */
#define CONF_PWM_OUTPUT      2		
/** PWM output pin */
#define CONF_PWM_OUT_PIN     PIN_PB12F_TCC0_WO6
/** PWM output pinmux */
#define CONF_PWM_OUT_MUX     MUX_PB12F_TCC0_WO6

/*define the master PWM period*/
/*in this case, we'll start with 2^12 bits = 4096*/
/*this will yield an appox 11.82 kHz PWM rep rate*/
/*note, for TCC0, max is 24-bits*/
#define USER_PWM_PERIOD			 0xFFF

 /* FreeRTOS message queue for PWM control*/
 xQueueHandle xPwmQHndle;

/* structure for PWM commands to be placed in queue */
typedef struct {
	UBaseType_t  uxChId;
	UBaseType_t  uxOffOn;
	UBaseType_t  uxDuty;
} pwm_cmd_t;

/*define length of the command queue for the PWM task */
#define PWM_RTOS_CMD_QUEUE_LEN	10

/*forward declarations*/
void configure_tcc(void);				/*this is the pwm config/init routine, called from main*/
void xTaskPwm(void *pvParameters);		/*this is the FreeRTOS task*/

#endif /* CNTRL_PWM_H_ */