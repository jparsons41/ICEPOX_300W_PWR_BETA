/*
 * cntrl_MOTOR.c
 *
 * Created: 9/14/2018 13:58:11
 *  Author: lphillips
*/


#include "cntrl_MOTOR.h"


static void motor_send_msg(uint8_t *, uint32_t);
static void motor_set_cfg(void);



static uint8_t preBuff[2] = { 0xC7, 0xFF };
static uint8_t flushBuff[2] = { 0xC4, 0x00 };
static uint8_t motorFuncBuff[2] = { 0xDC, 0x05 };
static uint8_t clearBuff[2] = { 0xE8, 0x01 };

static uint8_t cfgBuff[BLDC_CFG_LEN * 2] = {	// config register values to initialize the bldc controller
			0x04,0x4D,  // r0
			0x0C,0x01,  // r1
			0x14,0x40,  // r2
			0x1C,0x00,  // r3
			0x24,0x01,  // r4
			0x2C,0x00,  // r5
			0x34,0xFF,  // r6
			0x3C,0x7F,  // r7
			0x44,0x7F,  // r8
			0x4C,0x00,  // r9
			0x54,0x00,  // r10
			0x5D,0x72,  // r11
			0x65,0x73,  // r12
			0x6C,0x3D,  // r13
			0x74,0x62,  // r14
			0x7D,0x49,  // r15
			0x84,0x38,  // r16
			0x8D,0x4F,  // r17
			0x95,0xE0,  // r18
			0x9C,0xA4,  // r19
			0xA4,0x1E,  // r20
			0xAE,0x3F,  // r21
			0xB4,0x08,  // r22
			0xBC,0x0F,  // r23
			0xC4,0x00,  // r24
			0xCE,0x84,  // r25
			0xD4,0x01,  // r26
			0xDC,0x05,  // r27
			0xE6,0x00,  // r28
			0xEC,0x81,  // r29  check
			0xF7,0xFF	// r30
		};


struct spi_module bldc_spi_master_instance;
struct spi_slave_inst bldc_slave;


////
//  PUBLIC

void motor_config (void) {
	struct spi_config config_spi_master;
	struct spi_slave_inst_config slave_dev_config;
	spi_slave_inst_get_config_defaults(&slave_dev_config);
	slave_dev_config.ss_pin = BLDC_SS_PIN;
	spi_attach_slave(&bldc_slave, &slave_dev_config);
	spi_get_config_defaults(&config_spi_master);
	config_spi_master.transfer_mode = SPI_TRANSFER_MODE_3;
	config_spi_master.mux_setting = BLDC_MUX_SETTING;
	config_spi_master.pinmux_pad0 = BLDC_PINMUX_PAD0;
	config_spi_master.pinmux_pad1 = BLDC_PINMUX_PAD1;
	config_spi_master.pinmux_pad2 = BLDC_PINMUX_PAD2;
	config_spi_master.pinmux_pad3 = BLDC_PINMUX_PAD3;

	spi_init(&bldc_spi_master_instance, BLDC_SPI_MODULE, &config_spi_master);

	spi_enable(&bldc_spi_master_instance);


}

 void calculateparity(char val)
 {
	char scancode = val;
	 unsigned char parity = 0;
	 while(scancode > 0)          // if it is 0 there are no more 1's to count
	 {
		 if(scancode & 0x01)    //see if LSB is 1
		 {
			 parity++;                // why yes it is
		 }
		 scancode = scancode >> 1; //shift to next bit
	 }
	 return (parity & 0x01);  // only need the low bit to determine odd / even
 }

void motor_set_torque(uint16_t trqVal) {
	static uint8_t trqBuff[2] = { 0xF7, 0xFF };
	uint32_t parityCount = 0;
	uint16_t trqCalcd = 0;

	if (trqVal == 0) {
		motorFuncBuff[1] = 0x00;
		motor_send_msg(&motorFuncBuff[0], 1);
		return;
	}

	if (trqVal < 0) {
		motorFuncBuff[1] = 0x02 | (MOTOR_DIR_REVERSE << 2);
		trqVal *= (-1);
	}

	else {
		motorFuncBuff[1] = 0x02 | (MOTOR_DIR_FORWARD << 2);
	}



	motor_send_msg(&motorFuncBuff[0], 1);
	trqCalcd = 0xF<<12 | (0x0 <<11) | ((0x03FF & trqVal) << 1);

	uint16_t temp = trqCalcd;

	while ( temp > 0 ) {
		if ((temp & 0x0001) == 1) parityCount++;
		temp = temp >> 1;
	}
	if ((parityCount & 0x0001) != 1) trqCalcd+=1;


	trqBuff[0] = trqCalcd >> 8;
	trqBuff[1] = trqCalcd & 0x00FF;
	motor_send_msg(&trqBuff[0], 1);
}



//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

void motor_task(void *p) {
	UNUSED(p);

	vTaskDelay(pdMS_TO_TICKS(250));
	motor_set_cfg();

	motor_send_msg(&clearBuff[0], 1);
	motorFuncBuff[1] = 0x05;
	motor_send_msg(&motorFuncBuff[0], 1);

	motor_set_torque(0);

	for (;;) {
		vTaskDelay(pdMS_TO_TICKS(300));
		motor_send_msg(&flushBuff[0], 1);
		motor_send_msg(&clearBuff[0], 1);
	}
}




////
//  PRIVATE METHODS



void motor_send_msg(uint8_t *buff, uint32_t length) {
	for (uint i=0; i<length; i++) {
		//delay_us(50);
		spi_select_slave(&bldc_spi_master_instance, &bldc_slave, true);
		spi_write_buffer_wait(&bldc_spi_master_instance, &buff[i*2], 2);
		spi_select_slave(&bldc_spi_master_instance, &bldc_slave, false);
	}
}



void motor_set_cfg (void) {


	// 0xC7FF nvm write bits = 1 1 need to be 1 0 to write
	motor_send_msg(&preBuff[0], 1);

	// delay 10 ms
	//vTaskDelay(pdMS_TO_TICKS(10));

	// write configs
	motor_send_msg(&cfgBuff[0], BLDC_CFG_LEN);

	// delay 10 ms
	//vTaskDelay(pdMS_TO_TICKS(10));

	// 0xC400   nvm write = 10 and writes configs to nvm
	motor_send_msg(&flushBuff[0], 1);
	// may need watchdog spam

	//vTaskDelay(pdMS_TO_TICKS(10));



}