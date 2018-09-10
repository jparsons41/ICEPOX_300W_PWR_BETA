/*
 * cntrl_DISCRETE_IO.h
 *
 * Created: 5/31/2017 11:39:17 AM
 *  Author: Gerald
 */ 


#ifndef CNTRL_DISCRETE_IO_H_
#define CNTRL_DISCRETE_IO_H_

/*Motor DIR (OUTPUT), PB23*/
#define MOTOR_DIR					  PIN_PB23			
#define MOTOR_DIR_ACTIVE              true
#define MOTOR_DIR_INACTIVE            !MOTOR_DIR_ACTIVE
#define MOTOR_DIR_NAME                "MOTOR DIR PB23"

/*Output EN (OUTPUT), PA27*/
#define OUTPUT_EN					  PIN_PA27
#define OUTPUT_EN_ACTIVE              true
#define OUTPUT_EN_INACTIVE            !OUTPUT_EN_ACTIVE
#define OUTPUT_EN_NAME                "OUTPUT EN PA27"

/*Battery EN (OUTPUT), PB30*/
#define BATTERY_EN					  PIN_PB30
#define BATTERY_EN_ACTIVE             true
#define BATTERY_EN_INACTIVE           !BATTERY_EN_ACTIVE
#define BATTERY_EN_NAME               "BATTERY EN PB30"

/*Battery RUN/CHARGE (OUTPUT), PB31*/
#define BATTERY_CHARGE_EN			 PIN_PB31
#define BATTERY_CHARGE_EN_ACTIVE     true
#define BATTERY_CHARGE_EN_INACTIVE   !BATTERY_CHARGE_EN_ACTIVE
#define BATTERY_CHARGE_EN_NAME       "BATTERY RUN/CHARGE PB31"

/*MCU Switch Status (INPUT), PB00*/
#define MCU_SWITCH_STATUS			 PIN_PB00
#define MCU_SWITCH_STATUS_ACTIVE     true
#define MCU_SWITCH_STATUS_INACTIVE   !MCU_SWITCH_STATUS_ACTIVE
#define MCU_SWITCH_STATUS_NAME       "MCU_SWITCH_STATUS PB00"

/*Fan Output (OUTPUT), PB01*/
#define FAN_OUTPUT_STATUS			 PIN_PB01
#define FAN_OUTPUT_STATUS_ACTIVE     true
#define FAN_OUTPUT_STATUS_INACTIVE   !FAN_OUTPUT_STATUS_ACTIVE
#define FAN_OUTPUT_STATUS_NAME       "FAN_OUTPUT_STATUS PB00"

/*forward decelerations*/
void init_DisreteIO(void);

#endif /* CNTRL_DISCRETE_IO_H_ */