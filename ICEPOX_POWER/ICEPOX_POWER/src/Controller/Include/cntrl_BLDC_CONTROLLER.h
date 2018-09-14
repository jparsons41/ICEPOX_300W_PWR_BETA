/*
 * cntrl_BLDC_CONTROLLER.h
 *
 * Created: 9/11/2018 13:19:20
 *  Author: lphillips
 */


#ifndef CNTRL_BLDC_CONTROLLER_H_
#define CNTRL_BLDC_CONTROLLER_H_



#include <asf.h>


#  define CONF_SPI_MASTER_ENABLE  true	// these are asf MACROs for using master/slave
#  define CONF_SPI_SLAVE_ENABLE  false

#define BLDC_SPI_MODULE  SERCOM1
#define BLDC_SS_PIN  PIN_PA17
#define BLDC_MUX_SETTING SPI_SIGNAL_MUX_SETTING_E
#define BLDC_PINMUX_PAD0 PINMUX_PA16C_SERCOM1_PAD0
#define BLDC_PINMUX_PAD1 PINMUX_UNUSED
#define BLDC_PINMUX_PAD2 PINMUX_PA18C_SERCOM1_PAD2
#define BLDC_PINMUX_PAD3 PINMUX_PA19C_SERCOM1_PAD3


extern void bldc_controller_config(void);
extern void bldc_controller_set_cfg(void);
extern void bldc_controller_send_msg(uint8_t*, uint32_t);



#endif /* CNTRL_BLDC_CONTROLLER_H_ */