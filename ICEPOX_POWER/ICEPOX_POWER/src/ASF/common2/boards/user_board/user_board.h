/**
 * \file
 *
 * \brief User board definition template
 *
 */

 /* This file is intended to contain definitions and configuration details for
 * features and devices that are available on the board, e.g., frequency and
 * startup time for an external crystal, external memory devices, LED and USART
 * pins.
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#ifndef USER_BOARD_H
#define USER_BOARD_H

#include <conf_board.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \ingroup group_common_boards
 * \defgroup user_board_group User board
 *
 * @{
 */

void system_board_init(void);

/** Name string macro */
#define BOARD_NAME                "ICEPOXII_POWER"

/** \name Resonator definitions
 *  @{ */
#define BOARD_FREQ_SLCK_XTAL      (32768U)
#define BOARD_FREQ_SLCK_BYPASS    (32768U)
#define BOARD_FREQ_MAINCK_XTAL    0 /* Not Mounted */
#define BOARD_FREQ_MAINCK_BYPASS  0 /* Not Mounted */
#define BOARD_MCK                 CHIP_FREQ_CPU_MAX
#define BOARD_OSC_STARTUP_US      15625

/** \name LED0 definitions (RED)
 *  @{ */
#define LED0_PIN                  PIN_PA10			/*green*/
#define LED0_ACTIVE               false
#define LED0_INACTIVE             !LED0_ACTIVE

#define LED_0_NAME                "LED0 (red)"
#define LED_0_PIN                 LED0_PIN
#define LED_0_ACTIVE              LED0_ACTIVE
#define LED_0_INACTIVE            LED0_INACTIVE
#define LED0_GPIO                 LED0_PIN
#define LED_RED                   LED0_PIN

/** \name LED0 definitions (GREEN)
 *  @{ */
#define LED1_PIN                  PIN_PA11			/*red*/
#define LED1_ACTIVE               false
#define LED1_INACTIVE             !LED0_ACTIVE

#define LED_1_NAME                "LED1 (green)"
#define LED_1_PIN                 LED1_PIN
#define LED_1_ACTIVE              LED1_ACTIVE
#define LED_1_INACTIVE            LED1_INACTIVE
#define LED1_GPIO                 LED1_PIN
#define LED_GREEN                 LED1_PIN

/** Number of on-board LEDs */
#define LED_COUNT                  2
/** Number of on-board buttons */
#define BUTTON_COUNT			   0

/** \name CAN interface definitions
 * @{
 */
#define CAN_MODULE              CAN0
#define CAN_TX_PIN              PIN_PA24G_CAN0_TX
#define CAN_TX_MUX_SETTING      MUX_PA24G_CAN0_TX
#define CAN_RX_PIN              PIN_PA25G_CAN0_RX
#define CAN_RX_MUX_SETTING      MUX_PA25G_CAN0_RX
/** @} */

/**
 * \brief Turns off the specified LEDs.
 *
 * \param led_gpio LED to turn off (LEDx_GPIO).
 *
 * \note The pins of the specified LEDs are set to GPIO output mode.
 */
#define LED_Off(led_gpio)     port_pin_set_output_level(led_gpio,true)

/**
 * \brief Turns on the specified LEDs.
 *
 * \param led_gpio LED to turn on (LEDx_GPIO).
 *
 * \note The pins of the specified LEDs are set to GPIO output mode.
 */
#define LED_On(led_gpio)      port_pin_set_output_level(led_gpio,false)

/**
 * \brief Toggles the specified LEDs.
 *
 * \param led_gpio LED to toggle (LEDx_GPIO).
 *
 * \note The pins of the specified LEDs are set to GPIO output mode.
 */
#define LED_Toggle(led_gpio)  port_pin_toggle_output_level(led_gpio)





/** @} */

#ifdef __cplusplus
}
#endif

#endif // USER_BOARD_H
