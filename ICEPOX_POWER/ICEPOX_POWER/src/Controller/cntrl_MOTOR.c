#include "cntrl_MOTOR.h"
#include "cntrl_DISCRETE_IO.h"


static void motor_send_msg(uint8_t *, uint32_t);
static void motor_configure_registers(void);
static void motor_set_gains(void);
static void motor_set_run_bit(uint8_t);

static const uint8_t STARTUP_COUNT_LIMIT = 1000; //  was "10"...What is this...If this is 10 then the system stops even after a successuful cranking event???
static const uint8_t RPM_CHECK_TIME = 7; // usually 7 in "Tim's" constant current control
static const uint16_t STARTUP_SPEED = 1023; //startup speed is actually "demand" it is only "speed" in speed control mode
static const uint16_t SUCCESSFUL_STARTUP_RPM = 1000;
static uint16_t actualRpm;
static bool run = false;

static uint8_t startupRegisterConfigurationValues[BLDC_CFG_LEN * 2] = {
	0x06,0x10,    //r0
	0x0c,0x01,    //r1
	0x14,0x64,    //r2       increased mosfet switching dead time to 2.5uS
+	0x1c,0x00,    //r3
	0x24,0x01,    //r4
	0x2c,0x00,    //r5
	0x34,0x7E,    //r6		Dsp via EC  changed from fe to ff for correct parity  8/14/2019    10/18/2019 reduced current limit blanking time to 1 uS from 1.8uS
	0x3d,0x1B,    //r7      Was 0x3d, 0x3f - this gave VDS limit of 1.55 volts...too much.  adjusting to lower limits. new limit is 0.25V
    0x45,0x35,    //r8      registry number was wrong (10/18/2019) and the blanking period was very long.  reducing blanking period to minimum
	0x4c,0x00,    //r9
	0x54,0x00,    //r10
	0x5d,0xb1,    //r11
	0x65,0xb0,    //r12
	0x6c,0x3e,    //r13
	0x74,0x62,    //r14
	0x7c,0x00,    //r15    turning off alignment hold at startup and minimizing the the duty cycle
	0x84,0x38,    //r16
	0x8d,0x01,    //r17   turning ON windmill and minimizing counter rotation braking (it should never be active but just in case)
	0x94,0x42,    //r18
	0x9d,0xe2,    //r19
	0xa4,0x06,    //r20
	0xae,0xbe,    //r21
	0xb4,0x86,    //r22     changed resolution from 3.2 Hz to 0.8 Hz
	0xbc,0x14,    //r23 was 0xbc 0x0f changing to reduce overspeed limit 10-14-19
	0xc4,0x00,    //r24
	0xce,0xa5,    //r25		DsP via ClancyE, changed from a1 to 81  8/14/2019   (a1 => LWK = 1,  81  => LWK =0   10/18/2019 changed to closed loop current control
	0xd4,0x01,    //r26
	0xdd,0x15,    //r27		DSp via EC, changed from 15 to 14 for parity bit   8/14/2019
	0xe6,0x00,    //r28
	0xec,0x8e,    //r29
	0xf5,0x01    //r30
};
static uint8_t steadyStateRegisterConfigurationValues[BLDC_CFG_LEN * 2] = {
	0x06,0x10,    //r0
	0x0c,0x01,    //r1
	0x14,0x64,    //r2       increased mosfet switching dead time to 2.5uS
	0x1c,0x00,    //r3
	0x24,0x01,    //r4
	0x2c,0x00,    //r5
	0x34,0x7E,    //r6		Dsp via EC  changed from fe to ff for correct parity  8/14/2019    10/18/2019 reduced current limit blanking time to 1 uS from 1.8uS
	0x3d,0x1B,    //r7      Was 0x3d, 0x3f - this gave VDS limit of 1.55 volts...too much.  adjusting to lower limits. new limit is 0.25V
	0x45,0x35,    //r8      registry number was wrong (10/18/2019) and the blanking period was very long.  reducing blanking period to minimum 
	0x4c,0x00,    //r9
	0x54,0x00,    //r10
	0x5d,0xb1,    //r11
	0x65,0xb0,    //r12
	0x6c,0x3e,    //r13
	0x74,0x62,    //r14
	0x7c,0x00,    //r15    turning off alignment hold at startup and minimizing the the duty cycle
	0x84,0x38,    //r16
	0x8d,0x01,    //r17   turning ON windmill and minimizing counter rotation braking (it should never be active but just in case)
	0x94,0x42,    //r18
	0x9d,0xe2,    //r19
	0xa4,0x06,    //r20
	0xae,0xbe,    //r21
	0xb4,0x86,    //r22     changed resolution from 3.2 Hz to 0.8 Hz
	0xbc,0x0F,    //r23 
	0xc4,0x00,    //r24
	0xce,0xa5,    //r25		DsP via ClancyE, changed from a1 to 81  8/14/2019   (a1 => LWK = 1,  81  => LWK =0   10/18/2019 changed to closed loop current control
	0xd4,0x01,    //r26
	0xdd,0x15,    //r27		DSp via EC, changed from 15 to 14 for parity bit   8/14/2019
	0xe6,0x00,    //r28
	0xec,0x8e,    //r29
	0xf5,0x01    //r30
};

struct spi_module bldc_spi_master_instance;
struct spi_slave_inst bldc_slave;

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

void motor_run(uint16_t shouldRun)
{
	run = (shouldRun > 0);
}

void motor_update_actual_rpm(uint16_t rpm)
{
	actualRpm = rpm;
}

void motor_set_run_bit(uint8_t runFlag){
	uint8_t motorFuncBuff[2] = { 0xDC, 0x14 };
	if(runFlag == 1){
		motorFuncBuff[1] = 0x17;
	}
	motor_send_msg(&motorFuncBuff[0], 1);
}

void motor_set_demand_input(uint16_t demand_input) {
	static uint8_t demandInputBuffer[2] = { 0xF7, 0xFF };
	uint32_t parityCount = 0;
	uint16_t calculatedDemandInput = 0;

	calculatedDemandInput = 0xF<<12 | (0x0 <<11) | ((0x03FF & demand_input) << 1);

	uint16_t temp = calculatedDemandInput;

	while ( temp > 0 ) {
		if ((temp & 0x0001) == 1) parityCount++;
		temp = temp >> 1;
	}
	if ((parityCount & 0x0001) != 1) calculatedDemandInput+=1;


	demandInputBuffer[0] = calculatedDemandInput >> 8;
	demandInputBuffer[1] = calculatedDemandInput & 0x00FF;
	motor_send_msg(&demandInputBuffer[0], 1);
}


void step(){
	static uint8_t startupCount = 0;
	if(run == 1){
		if(startupCount == 0)
		{
			port_pin_set_output_level(LIN_PIN, LIN_PIN_INACTIVE);  // LIN pin: 0
			vTaskDelay(pdMS_TO_TICKS(500));  // delay for wake up
			port_pin_set_output_level(LIN_PIN, LIN_PIN_ACTIVE);  // LIN pin: 1
			vTaskDelay(pdMS_TO_TICKS(500));  // delay for wake up
			motor_configure_registers();
			motor_set_run_bit(1);
			motor_set_demand_input(STARTUP_SPEED);
		}
		else if (startupCount >= RPM_CHECK_TIME && actualRpm < SUCCESSFUL_STARTUP_RPM)
		{
			motor_set_demand_input(0);
			startupCount = 0;
			return;
		}
		else if (startupCount == STARTUP_COUNT_LIMIT)
		{
			motor_send_msg(&steadyStateRegisterConfigurationValues[0], 31);
			motor_set_demand_input(STARTUP_SPEED);
		}
		
		if(startupCount <= STARTUP_COUNT_LIMIT){
			startupCount++;
		}
	}
	else {
		vTaskDelay(pdMS_TO_TICKS(1000));  // delay waiting for sleep
		static uint8_t goToSleep[2] = {0xdd, 0x15};
		motor_send_msg(&goToSleep[0], 1);  // GTS: 0
		vTaskDelay(pdMS_TO_TICKS(1000));  // delay waiting for sleep
		goToSleep[0] = 0xdd;
		goToSleep[1] = 0x94;
		motor_send_msg(&goToSleep[0], 1);  // GTS: 1
		vTaskDelay(pdMS_TO_TICKS(500));  // delay waiting for sleep
		port_pin_set_output_level(LIN_PIN, LIN_PIN_ACTIVE);  // LIN pin: 1
		motor_set_run_bit(0); // this should be ignored since asleep
		startupCount = 0;
	}
}

void motor_task(void *p) {
	UNUSED(p);

	vTaskDelay(pdMS_TO_TICKS(2000));

	static uint8_t clearBuff[2] = { 0xE8, 0x01 };
	motor_send_msg(&clearBuff[0], 1);
	motor_set_run_bit(0);

	for (;;) {
		vTaskDelay(pdMS_TO_TICKS(300));
		step();
	}
}


void motor_send_msg(uint8_t *buff, uint32_t length) {
	uint8_t retBuff[2] = {0xFF, 0xFF};
	for (uint i=0; i<length; i++) {
		spi_select_slave(&bldc_spi_master_instance, &bldc_slave, true);
		spi_transceive_buffer_wait(&bldc_spi_master_instance, &buff[i*2], &retBuff[0], 2);
		spi_select_slave(&bldc_spi_master_instance, &bldc_slave, false);
	}
}


void motor_configure_registers (void)
{
	static uint8_t preBuff[2] = { 0xC7, 0xFF };	// 0xC7FF nvm write bits = 1 1 need to be 1 0 to write
	static uint8_t flushBuff[2] = { 0xC4, 0x00 };
	
	motor_send_msg(&preBuff[0], 1);
	motor_send_msg(&startupRegisterConfigurationValues[0], 31);

	// NVM write = 01 to setup for nvm write. Page 72 of A4964 datasheet.
	flushBuff[0] = 0xC2;
	motor_send_msg(&flushBuff[0], 1);
	// NVM write = 10 and writes configs to nvm
	flushBuff[0] = 0xC4;
	motor_send_msg(&flushBuff[0], 1);
}