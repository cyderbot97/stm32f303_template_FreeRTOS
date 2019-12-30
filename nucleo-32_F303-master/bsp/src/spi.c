/*
 * spi.c
 *
 *  Created on: 24 déc. 2019
 *      Author: cyril
 */

#include "stm32f3xx.h"
#include "mpu9250.h"

uint8_t BSP_SPI_SendReceive(uint8_t tx_byte)
{
	uint8_t	rx_byte;

	// Make sure TXE is set before sending data
	while((SPI1->SR & SPI_SR_TXE_Msk) == 0);

	// Send tx_byte
	*(__IO uint8_t *)&SPI1->DR = tx_byte;

	// Wait until incoming data has arrived
	while((SPI1->SR & SPI_SR_RXNE_Msk) == 0);

	// Read data
	rx_byte = *(__IO uint8_t *)&SPI1->DR;

	return rx_byte;
}


void BSP_SPI_Write(uint8_t register_address, uint8_t data)
{
	// Set FIFO threshold to 1-byte
	SPI1->CR2 |= SPI_CR2_FRXTH;

	// Select slave (CS -> low)
	GPIOA->BSRR = GPIO_BSRR_BR_4;

	// Send register address to write to
	BSP_SPI_SendReceive(register_address);

	// Send data to write
	BSP_SPI_SendReceive(data);

	// Release slave (CS -> High)
	GPIOA->BSRR = GPIO_BSRR_BS_4;
}

void BSP_SPI_Read(uint8_t readAddr, uint8_t *readBuf, uint8_t bytes)
{
    uint8_t  i = 0;

    GPIOA->BSRR = GPIO_BSRR_BR_4;// set CS (low)

    BSP_SPI_SendReceive(readAddr | READ_FLAG);		// Send address with R/W bit set to R

    for (i=0; i<bytes; i++)
    {
    	readBuf[i] = BSP_SPI_SendReceive(0x00);	// Read SPI answer
    }

    GPIOA->BSRR = GPIO_BSRR_BS_4;			// Reset CS (high)
}

void BSP_SPI1_Init()
{
	// SPI_SCK  -> PA5 (AF5)
	// SPI_MISO -> PA6 (AF5)
	// SPI_MOSI -> PA7 (AF5)
	// CS pin	-> PA4 (GPIO)

	// Configure PA4 as CS pin

	// Enable PA4 clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	// Configure PA4 as output
	GPIOA->MODER &= ~GPIO_MODER_MODER4_Msk;
	GPIOA->MODER |= (0x01 <<GPIO_MODER_MODER4_Pos);

	// Configure PA4 as Push-Pull output
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_4;

	// Configure PA4 as High-Speed Output
	GPIOA->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR4_Msk;
	GPIOA->OSPEEDR |= (0x03 <<GPIO_OSPEEDER_OSPEEDR4_Pos);

	// Disable PA4 Pull-up/Pull-down
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR4_Msk;

	// Idle state is high
	GPIOA->BSRR = GPIO_BSRR_BS_4;

	// Configure PA5, PA6, PA7 as SPI1 pins (AF0)

	// Enable GPIOA clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	// Configure PA5, PA6, PA7 as AF mode
	GPIOA->MODER &= ~(GPIO_MODER_MODER5 | GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
	GPIOA->MODER |= (0x02 << GPIO_MODER_MODER5_Pos) |(0x02 <<GPIO_MODER_MODER6_Pos) |(0x02 <<GPIO_MODER_MODER7_Pos);

	// Connect to SPI1 (AF5)
	GPIOA->AFR[0] &= ~(0xFFF00000);
	GPIOA->AFR[0] |=  (0x55500000);

	// Enable SPI1 Clock
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	// Configure SPI with default config
	// 4-wire = full-duplex (MISO/MOSI on separated lines)
	// 8-bit, no CRC, MSB first
	// CPHA = 0, CPOL = 0 -> SPI mode 0 (first clock rising edge)
	SPI1->CR1 = 0x0000;
	SPI1->CR2 = 0x0000;

	// Set the baudrate to 64MHz /256 = 250kHz (slow, but easy to debug)
	SPI1->CR1 |= 0x07 <<SPI_CR1_BR_Pos;

	// Set data size to 8-bit
	SPI1->CR2 |= 0x07 <<SPI_CR2_DS_Pos;

	// Set as master (SSI must be high), with software managed NSS
	SPI1->CR1 |= SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM;

	// Enable SPI1
	SPI1->CR1 |= SPI_CR1_SPE;
}
