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

#define BLDC_SPI_PIN
#define BLDC_SPI_PINMUX

#define BLDC_SPI_MODULE 
#define BLDC_SPI_


//[definition_master]
#define CONF_MASTER_SPI_MODULE  SERCOM1
#define CONF_MASTER_SS_PIN      
#define CONF_MASTER_MUX_SETTING SPI_SIGNAL_MUX_SETTING_E
#define CONF_MASTER_PINMUX_PAD0 
#define CONF_MASTER_PINMUX_PAD1 
#define CONF_MASTER_PINMUX_PAD2 
#define CONF_MASTER_PINMUX_PAD3 
//[definition_master]
#endif /* CNTRL_BLDC_CONTROLLER_H_ */