/*
 * conf_uart_serial.h
 *
 * Created: 5/30/2017 3:18:28 PM
 *  Author: Gerald
 */ 


#ifndef CONF_UART_SERIAL_H_
#define CONF_UART_SERIAL_H_


#include <board.h>

//! [conf_uart_serial_settings]
#define CONF_STDIO_USART_MODULE  SERCOM3
#define CONF_STDIO_MUX_SETTING   USART_RX_1_TX_0_XCK_1 
#define CONF_STDIO_PINMUX_PAD0   PINMUX_PA22C_SERCOM3_PAD0
#define CONF_STDIO_PINMUX_PAD1   PINMUX_PA23C_SERCOM3_PAD1
#define CONF_STDIO_PINMUX_PAD2   PINMUX_UNUSED
#define CONF_STDIO_PINMUX_PAD3   PINMUX_UNUSED
#define CONF_STDIO_BAUDRATE      115200
//! [conf_uart_serial_settings]


#endif /* CONF_UART_SERIAL_H_ */