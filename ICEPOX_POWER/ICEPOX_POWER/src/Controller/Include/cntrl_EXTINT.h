/*
 * cntrl_EXTINT.h
 *
 * Created: 3/29/2017 5:00:33 PM
 *  Author: Gerald
 */ 


#ifndef CNTRL_EXTINT_H_
#define CNTRL_EXTINT_H_

/* Standard includes */
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

#include "conf_application.h"

#include "cntrl_GLOBALS.h"

#define  EXTINT_CH8					8

/*forward decelerations*/
void configure_extint_channel(void);
void configure_extint_callbacks(void);
void undervoltage_int_callback(void);

#endif /* CNTRL_EXTINT_H_ */