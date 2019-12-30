/*
 * spi.h
 *
 *  Created on: 24 déc. 2019
 *      Author: cyril
 */

#ifndef BSP_INC_SPI_H_
#define BSP_INC_SPI_H_

#include "main.h"

uint8_t BSP_SPI_SendReceive(uint8_t);
void BSP_SPI_Write(uint8_t, uint8_t);
void BSP_SPI_Read(uint8_t, uint8_t *, uint8_t );
void BSP_SPI1_Init();

#endif /* BSP_INC_SPI_H_ */
