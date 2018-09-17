/*
 * cntrl_MOTOR.h
 *
 * Created: 9/14/2018 13:58:22
 *  Author: lphillips
 */


#ifndef CNTRL_MOTOR_H_
#define CNTRL_MOTOR_H_



#include <asf.h>
/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"


#define MOTOR_WRITE_CONFIG_ENABLED 1
#define BLDC_CFG_LEN  31

#define MOTOR_DIR_FORWARD  1
#define MOTOR_DIR_REVERSE  (!MOTOR_DIR_FORWARD)


#  define CONF_SPI_MASTER_ENABLE  true	// these are asf MACROs for using master/slave
#  define CONF_SPI_SLAVE_ENABLE  false

#define BLDC_SPI_MODULE  SERCOM1
#define BLDC_SS_PIN  PIN_PA17
#define BLDC_MUX_SETTING SPI_SIGNAL_MUX_SETTING_E
#define BLDC_PINMUX_PAD0 PINMUX_PA16C_SERCOM1_PAD0
#define BLDC_PINMUX_PAD1 PINMUX_UNUSED
#define BLDC_PINMUX_PAD2 PINMUX_PA18C_SERCOM1_PAD2
#define BLDC_PINMUX_PAD3 PINMUX_PA19C_SERCOM1_PAD3



extern void motor_set_torque(uint16_t);  // 10 bit value
extern void motor_config(void);
extern void motor_task(void*);



#endif /* CNTRL_MOTOR_H_ */