/*
 * imu.c
 *
 *  Created on: 3 avr. 2020
 *      Author: cyril
 */

#include "imu.h"

uint8_t BNO055_Init(void){
	uint8_t id = 0;

	uint8_t data;

	//Reset
	data = 0x20;
	BSP_I2C1_Write(BNO055_ADDRESS_A, BNO055_SYS_TRIGGER_ADDR, &data,1);
	delay_ms(700);

	//Search
	BSP_I2C1_Read(BNO055_ADDRESS_A, BNO055_CHIP_ID_ADDR, &id,1);

	if (id != BNO055_ID)
	{
		my_printf("Can't find BNO055, id: 0x%02x. Please check your wiring.\r\n");
		return 0x00;
	}
	else{

		//Set Mode Config
		data = OPERATION_MODE_CONFIG;
		BSP_I2C1_Write(BNO055_ADDRESS_A, BNO055_OPR_MODE_ADDR, &data,1);
		delay_ms(30);

		//Reset
		data = 0x20;
		BSP_I2C1_Write(BNO055_ADDRESS_A, BNO055_SYS_TRIGGER_ADDR, &data,1);
		delay_ms(700);

		//Set to Normal Power Mode
		data = POWER_MODE_NORMAL;
		BSP_I2C1_Write(BNO055_ADDRESS_A, BNO055_PWR_MODE_ADDR, &data,1);
		delay_ms(30);

		//Select page 0
		BNO055_SetPage(0);

		//Select external cyrtal
		data = 0x00;
		BSP_I2C1_Write(BNO055_ADDRESS_A, BNO055_SYS_TRIGGER_ADDR, &data,1);
		delay_ms(30);

		//Set Mode Config
		data = OPERATION_MODE_IMUPLUS;
		BSP_I2C1_Write(BNO055_ADDRESS_A, BNO055_OPR_MODE_ADDR, &data,1);
		delay_ms(1000);

		return 0x01;
	}
}

void BNO055_SetPage(uint8_t page) {
	BSP_I2C1_Write(BNO055_ADDRESS_A, BNO055_PAGE_ID_ADDR, &page,1);
}

