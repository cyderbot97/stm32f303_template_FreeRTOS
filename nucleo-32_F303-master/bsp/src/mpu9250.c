/*
 * mpu9250.c
 *
 *  Created on: 24 d�c. 2019
 *      Author: cyril
 */

#include "mpu9250.h"

void BSP_MPU9250_Init()
{
	// MPU9250 Basic init
	BSP_SPI_Write(MPUREG_PWR_MGMT_1, BIT_H_RESET);			// Reset Device
	delay_ms(1);
	BSP_SPI_Write(MPUREG_PWR_MGMT_1, BIT_CLKSEL_AUTO);		// Auto-select clock source
	delay_ms(1);
	BSP_SPI_Write(MPUREG_PWR_MGMT_2, BITS_ALL_ON);			// Enable all Acc & Gyro sensors
	delay_ms(1);

	// Setup Low-pass filters for Gyros and Temperature sensor
	BSP_SPI_Write(MPUREG_CONFIG, BITS_DLPF_CFG_3);			// DLPF_CFG is set to 0 (41Hz Gyros, 42Hz Temp)
	delay_ms(1);

	// Setup Full Scales & filters
	BSP_SPI_Write(MPUREG_GYRO_CONFIG, BITS_FS_500DPS);		// Set Gyro to +-500dps and use DLPF as above
	delay_ms(1);
	BSP_SPI_Write(MPUREG_ACCEL_CONFIG, BITS_FS_2G);			// Set Accl to +-2G
	delay_ms(1);
	BSP_SPI_Write(MPUREG_ACCEL_CONFIG_2, BITS_DLPF_CFG_3);	// Set Accl datarate to 1kHz, enable LPF, BW 41Hz
	delay_ms(1);

	// Setup Interrupts
	BSP_SPI_Write(MPUREG_SMPLRT_DIV, 9);						// Sample rate is 100Hz (10ms)
	delay_ms(1);
	BSP_SPI_Write(MPUREG_INT_ENABLE, BIT_INT_RAW_RDY_EN);	// Setup interrupt when raw data is ready
	delay_ms(1);

	// Setup for Magnetometer
	BSP_SPI_Write(MPUREG_USER_CTRL, 0x02);					// I2C Master Reset
	delay_ms(1);
	BSP_SPI_Write(MPUREG_USER_CTRL, 0x20);					// I2C Master Mode Enable
	delay_ms(1);

	BSP_SPI_Write(MPUREG_I2C_MST_CTRL, 0x0D);				// I2C Master at 400KHz
	delay_ms(1);

	BSP_SPI_Write(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR);	// Set Slave0 address to AK8963 adress
	delay_ms(1);

	BSP_SPI_Write(MPUREG_I2C_SLV0_REG, AK8963_CNTL2);		// I2C slave 0 register address from where to begin data transfer
	delay_ms(1);
	BSP_SPI_Write(MPUREG_I2C_SLV0_DO, 0x01);					// Reset AK8963
	delay_ms(1);
	BSP_SPI_Write(MPUREG_I2C_SLV0_CTRL, 0x81);				// Enable I2C and set 1 byte
	delay_ms(1);

	BSP_SPI_Write(MPUREG_I2C_SLV0_REG, AK8963_CNTL1);		// I2C slave 0 register address from where to begin data transfer
	delay_ms(1);
	BSP_SPI_Write(MPUREG_I2C_SLV0_DO, 0x12);					// Register value to continuous measurement in 16bit
	delay_ms(1);
	BSP_SPI_Write(MPUREG_I2C_SLV0_CTRL, 0x81);				// Enable I2C and set 1 byte
	delay_ms(1);

	// Continuously read 7 bytes from Magnetometers
	BSP_SPI_Write(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR|READ_FLAG);
	delay_ms(1);
	BSP_SPI_Write(MPUREG_I2C_SLV0_REG,  AK8963_HXL);
	delay_ms(1);
	BSP_SPI_Write(MPUREG_I2C_SLV0_CTRL,  0x87);
	delay_ms(1);
}
