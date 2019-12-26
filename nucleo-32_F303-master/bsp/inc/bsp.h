#ifndef BSP_INC_BSP_H_
#define BSP_INC_BSP_H_

#include "stm32f3xx.h"

/*
 * LED driver functions
 */

void	BSP_LED_Init	(void);
void	BSP_LED_On		(void);
void	BSP_LED_Off		(void);
void	BSP_LED_Toggle	(void);


/*
 * Debug Console driver functions
 */

void	BSP_Console_Init	(void);



#endif /* BSP_INC_BSP_H_ */
