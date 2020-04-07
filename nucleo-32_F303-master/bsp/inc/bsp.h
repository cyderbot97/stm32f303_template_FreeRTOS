#ifndef BSP_INC_BSP_H_
#define BSP_INC_BSP_H_

#include "main.h"

void adc_init			(void);
void servo_init 		(void);
void uart_init			(void);
void BSP_NVIC_Init		(void);
void BSP_LED_Init		(void);
void BSP_LED_Toggle		(void);

void BSP_SERVO_INIT		(void);
void BSP_SONAR			(void);
void BSP_Console_Init	(void);

#endif /* BSP_INC_BSP_H_ */
