/*
 * cntrl_ESC.h
 *
 * Created: 9/10/2018 17:24:59
 *  Author: lphillips
 */


#ifndef CNTRL_ESC_H_
#define CNTRL_ESC_H_


#include <asf.h>

#define ESC_RELAY_PIN	PIN_PB13
#define ESC_RELAY_MUX	PORT_PB13
#define ESC_RELAY_ACTIVE	true
#define ESC_RELAY_INACTIVE  !ESC_RELAY_ACTIVE



#define ESC_PWM_MODULE			TCC1
#define ESC_PWM_CHANNEL			1
#define ESC_PWM_OUTPUT_SELECT	1
#define ESC_PWM_PIN		PIN_PA11E_TCC1_WO1
#define ESC_PWM_MUX		MUX_PA11E_TCC1_WO1



#endif /* CNTRL_ESC_H_ */