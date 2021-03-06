#include "main.h"


extern int sonar_data;
extern xSemaphoreHandle xSem_New_Data_Sonar;

void TIM2_IRQHandler(){

	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	if((TIM2->SR & TIM_SR_CC2IF) == TIM_SR_CC2IF){
		sonar_data = (sonar_data + TIM2->CCR2)/2;
		//BSP_LED_Toggle();
		xSemaphoreGiveFromISR(xSem_New_Data_Sonar, &xHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	}
}

extern xSemaphoreHandle xSem_New_Data_Uart;
void USART1_IRQHandler(){

	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	if ((USART1->ISR & USART_ISR_RTOF) == USART_ISR_RTOF)
		{
			// Clear the interrupt pending bit
			USART1->ICR |= USART_ICR_RTOCF;

			xSemaphoreGiveFromISR(xSem_New_Data_Uart, &xHigherPriorityTaskWoken);

			portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
		}

}

extern uint8_t rx_dma_buffer[16];

void uart_init()
{

	// Enable GPIOA clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	// Enable GPIOB clock
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	// Configure PA9 and PA10 as Alternate function
	GPIOA->MODER &= ~(GPIO_MODER_MODER9 | GPIO_MODER_MODER10);
	GPIOA->MODER |=  (0x02 << GPIO_MODER_MODER9_Pos) | (0x02 << GPIO_MODER_MODER10_Pos);

	// Configure PB6 and PB7 as Alternate function 7
	//GPIOB->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
	//GPIOB->MODER |=  (0x02 << GPIO_MODER_MODER6_Pos) | (0x02 << GPIO_MODER_MODER7_Pos);

	// Set PA9 and PA10 to AF10 (USART1)
	GPIOA->AFR[1] &= ~(0x00000FF0);
	GPIOA->AFR[1] |=  (0x00000770);

	// Set PB7 and PB6 to AF7 (USART1)
	//GPIOB->AFR[0] &= ~(0xFF000000);
	//GPIOB->AFR[0] |=  (0x77000000);

	// Enable USART1 clock
	RCC -> APB2ENR |= RCC_APB2ENR_USART1EN;

	// Clear USART2 configuration (reset state)
	// 8-bit, 1 start, 1 stop, CTS/RTS disabled
	USART1->CR1 = 0x00000000;
	USART1->CR2 = 0x00000000;
	USART1->CR3 = 0x00000000;

	RCC->CFGR3 &= ~RCC_CFGR3_USART1SW_Msk;

	// Baud Rate = 115200
	USART1->CR1 &= ~USART_CR1_OVER8;
	USART1->BRR = 0x0115;

	// Enable both Transmitter and Receiver
	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;


	//Start DMA clock
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;

	//Reset DMA1 channel 5 configuration
	DMA1_Channel5->CCR &= 0x00000000;

	//Set direction Peripheral to memory
	DMA1_Channel5->CCR &= ~DMA_CCR_DIR;

	//Péripheral is USART1 RDR
	DMA1_Channel5->CPAR = (uint32_t)&USART1->RDR;

	//peripheral data size is 8-bit (byte)
	DMA1_Channel5->CCR |= (0x00<<DMA_CCR_PSIZE_Pos);

	//Disable auto-increment Peripheral adress
	DMA1_Channel5->CCR &= ~DMA_CCR_PINC;

	// Memory is rx_dma_buffer
	DMA1_Channel5->CMAR = (uint32_t)rx_dma_buffer;

	// Memory data size is 8-bit (byte)
	DMA1_Channel5->CCR |= (0x00 <<DMA_CCR_MSIZE_Pos);

	// Enable auto-increment Memory address
	DMA1_Channel5->CCR |= DMA_CCR_MINC;

	// Set Memory Buffer size
	DMA1_Channel5->CNDTR = 16;

	// DMA mode is circular
	//DMA1_Channel5->CCR |= DMA_CCR_CIRC;
	DMA1_Channel5->CCR &= ~DMA_CCR_CIRC;

	// Enable DMA1 Channel 5
	DMA1_Channel5->CCR |= DMA_CCR_EN;

	// Enable USART2 DMA Request on RX
	USART1->CR3 |= USART_CR3_DMAR;

	USART1->RTOR = 120;

	// Enable RTO and RTO interrupt
	USART1->CR2 |= USART_CR2_RTOEN;
	USART1->CR1 |= USART_CR1_RTOIE;

	// Enable USART1
	USART1->CR1 |= USART_CR1_UE;
}

/*
void BSP_TIMER_Timebase_Init()
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;

	// Reset TIM6 configuration
	TIM7->CR1 = 0x0000;
	TIM7->CR2 = 0x0000;

	// Set TIM6 prescaler
	// Fck = 64MHz -> /64 = 1kHz counting frequency
	TIM7->PSC = (uint16_t) 64000 -1;

	// Set TIM6 auto-reload register for 1s
	TIM7->ARR = (uint16_t) 10000;

	// Enable auto-reload preload
	TIM7->CR1 |= TIM_CR1_ARPE;


	// Start TIM6 counter
	TIM7->CR1 |= TIM_CR1_CEN;
}
*/

void BSP_Console_Init()
{
	// Enable GPIOA clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	// Configure PA2 and PA3 as Alternate function
	GPIOA->MODER &= ~(GPIO_MODER_MODER2 | GPIO_MODER_MODER3);
	GPIOA->MODER |=  (0x02 <<4U) | (0x02 <<6U);

	// Set PA2 and PA3 to AF7 (USART2)
	GPIOA->AFR[0] &= ~(0x0000FF00);
	GPIOA->AFR[0] |=  (0x00007700);

	// Enable USART2 clock
	RCC -> APB1ENR |= RCC_APB1ENR_USART2EN;

	// Clear USART2 configuration (reset state)
	// 8-bit, 1 start, 1 stop, CTS/RTS disabled
	USART2->CR1 = 0x00000000;
	USART2->CR2 = 0x00000000;
	USART2->CR3 = 0x00000000;

	// Baud Rate = 115200
	USART2->CR1 &= ~USART_CR1_OVER8;
	USART2->BRR = 0x0115;

	// Enable both Transmitter and Receiver
	USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;


	//Start DMA clock
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;

	//Reset DMA1 channel 6 configuration
	DMA1_Channel6->CCR &= 0x00000000;

	//Set direction Peripheral to memory
	DMA1_Channel6->CCR &= ~DMA_CCR_DIR;

	//Péripheral is USART2 RDR
	DMA1_Channel6->CPAR = (uint32_t)&USART2->RDR;

	//peripheral data size is 8-bit (byte)
	DMA1_Channel6->CCR |= (0x00<<DMA_CCR_PSIZE_Pos);

	//Disable auto-increment Peripheral adress
	DMA1_Channel6->CCR &= ~DMA_CCR_PINC;

	// Memory is rx_dma_buffer
	DMA1_Channel6->CMAR = (uint32_t)rx_dma_buffer;

	// Memory data size is 8-bit (byte)
	DMA1_Channel6->CCR |= (0x00 <<DMA_CCR_MSIZE_Pos);

		// Enable auto-increment Memory address
	DMA1_Channel6->CCR |= DMA_CCR_MINC;

		// Set Memory Buffer size
	DMA1_Channel6->CNDTR = 8;

	// DMA mode is circular
	DMA1_Channel6->CCR |= DMA_CCR_CIRC;

	// Enable DMA1 Channel 5
	DMA1_Channel6->CCR |= DMA_CCR_EN;

	// Enable USART2 DMA Request on RX
	USART2->CR3 |= USART_CR3_DMAR;

	// Enable USART2
	USART2->CR1 |= USART_CR1_UE;
}



void adc_init(void)
{
	RCC->CFGR2 |= RCC_CFGR2_ADCPRE12_DIV1;
	RCC->AHBENR |= RCC_AHBENR_ADC12EN;

	//Enable gpioA clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	//configure PA0 as analog
	GPIOA->MODER &= ~GPIO_MODER_MODER0_Msk;
	GPIOA->MODER |= (0x03 << GPIO_MODER_MODER0_Pos);

	//Start calibration
	ADC1->CR &= ~ADC_CR_ADVREGEN;
	ADC1->CR |= ADC_CR_ADVREGEN_0;

	for(int i=0; i<4200; i++);

	ADC1->CR &= ~ADC_CR_ADCALDIF; // calibration in Single-ended inputs Mode.
	ADC1->CR |= ADC_CR_ADCAL;
	while (ADC1->CR & ADC_CR_ADCAL);

	//ADC configuration
	ADC1->CFGR |= ADC_CFGR_CONT;
	//ADC1->CFGR &= ADC_CFGR_RES; //disable ADC_CFGR_CONT ?

	ADC1->SQR1 |= ADC_SQR1_SQ1_0;
	ADC1->SQR1 &= ~ADC_SQR1_L;

	ADC1->SMPR1 |= ADC_SMPR1_SMP7_1 | ADC_SMPR1_SMP7_0;

	ADC1->CR |= ADC_CR_ADEN; // Enable ADC1
	while(!ADC1->ISR & ADC_ISR_ADRD); // wait for ADRDY
 	ADC1->CR |= ADC_CR_ADSTART;

}

void BSP_SERVO_INIT(void)
{
	//Enable TIMER clock
	RCC -> APB2ENR |= RCC_APB2ENR_TIM1EN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;

	//Enable gpioA clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	//Enable gpioB clock
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	//
	GPIOA->MODER &= ~(GPIO_MODER_MODER8_Msk | GPIO_MODER_MODER11_Msk | GPIO_MODER_MODER12);
	GPIOA->MODER |= (0x02<<GPIO_MODER_MODER8_Pos) | (0x02<<GPIO_MODER_MODER11_Pos) | (0x02<<GPIO_MODER_MODER12_Pos);

	GPIOB->MODER &= ~(GPIO_MODER_MODER0_Msk | GPIO_MODER_MODER1_Msk | GPIO_MODER_MODER4_Msk| GPIO_MODER_MODER5_Msk );
	GPIOB->MODER |= (0x02<<GPIO_MODER_MODER0_Pos)| (0x02<<GPIO_MODER_MODER1_Pos)| (0x02<<GPIO_MODER_MODER4_Pos)| (0x02<<GPIO_MODER_MODER5_Pos);

	//Set alternate function
	GPIOA->AFR[1] &= ~(0x000FF0FF);
	GPIOA->AFR[1] |=  (0x0001B066);

	GPIOB->AFR[0] &= ~(0x00FF00FF);
	GPIOB->AFR[0] |=  (0x00220022);

	//Reset Timer configuration
	TIM16->CR1  = 0x0000;
	TIM16->CR2  = 0x0000;
	TIM16->CCER = 0x0000;

	TIM1->CR1  = 0x0000;
	TIM1->CR2  = 0x0000;
	TIM1->CCER = 0x0000;

	TIM3->CR1  = 0x0000;
	TIM3->CR2  = 0x0000;
	TIM3->CCER = 0x0000;

	//set TIM prescaler
	TIM1->PSC =  (uint16_t) 64-1;
	TIM3->PSC =  (uint16_t) 64-1;
	TIM16->PSC = (uint16_t) 64-1;

	//Set auto reload
	TIM1->ARR = (uint16_t) 11000;
	TIM3->ARR = (uint16_t) 11000;
	TIM16->ARR = (uint16_t) 11000;

	// Enable Auto-Reload Preload register
	TIM1->CR1 |= TIM_CR1_ARPE;
	TIM3->CR1 |= TIM_CR1_ARPE;
	TIM16->CR1 |= TIM_CR1_ARPE;

	// Setup Input Capture
	TIM1->CCMR1 = 0x0000;
	TIM1->CCMR2 = 0x0000;

	TIM3->CCMR1 = 0x0000;
	TIM3->CCMR2 = 0x0000;

	TIM16->CCMR1 = 0x0000;
	TIM16->CCMR2 = 0x0000;

	// Setup PWM mode 1 output ch1,ch2,ch3,ch4
	TIM1->CCMR1 |= (0x06 <<TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
	TIM1->CCMR1 |= (0x06 <<TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
	TIM1->CCMR2 |= (0x06 <<TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;
	TIM1->CCMR2 |= (0x06 <<TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;

	TIM3->CCMR1 |= (0x06 <<TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
	TIM3->CCMR1 |= (0x06 <<TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
	TIM3->CCMR2 |= (0x06 <<TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;
	TIM3->CCMR2 |= (0x06 <<TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;

	TIM16->CCMR1 |= (0x06 <<TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;

	// Set default PWM values
	TIM1->CCR1 = 1500; // right arm
	TIM1->CCR2 = 1500; // left arm
	TIM1->CCR3 = 1500;
	TIM1->CCR4 = 1500; // head

	TIM3->CCR1 = 1500;
	TIM3->CCR2 = 1500;
	TIM3->CCR3 = 1500;
	TIM3->CCR4 = 1500;

	TIM16->CCR1 = 1500;

	// Enable Outputs
	TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

	TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

	TIM16->CCER |= TIM_CCER_CC1E;

	// Enable Main output
	TIM1->BDTR |= TIM_BDTR_MOE;
	TIM16->BDTR |= TIM_BDTR_MOE;

	// Enable TIM3 and TIM1
	TIM1->CR1 |= TIM_CR1_CEN;
	TIM3->CR1 |= TIM_CR1_CEN;
	TIM16->CR1 |= TIM_CR1_CEN;


	delay_ms(2000);

	TIM1->CCR1 = 0; // right arm
	TIM1->CCR2 = 0; // left arm
	TIM1->CCR3 = 0;
	TIM1->CCR4 = 0; // head

	TIM3->CCR1 = 0;
	TIM3->CCR2 = 0;
	TIM3->CCR3 = 0;
	TIM3->CCR4 = 0;

	TIM16->CCR1 = 0;

}

void BSP_LED_Init()
{
	// Enable GPIOB clock
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	// Configure PB3 as output
	GPIOB->MODER &= ~GPIO_MODER_MODER3;
	GPIOB->MODER |= (0x01 <<6U);

	// Configure PB3 as Push-Pull output
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT_3;

	// Configure PB3 as High-Speed Output
	GPIOB->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR3_Msk;
	GPIOB->OSPEEDR |= (0x03 <<GPIO_OSPEEDER_OSPEEDR3_Pos);

	// Disable PB3 Pull-up/Pull-down
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR3_Msk;

	// Set Initial State OFF
	GPIOB->BSRR = GPIO_BSRR_BR_3;
}

void BSP_LED_Toggle()
{
	GPIOB->ODR ^= GPIO_ODR_3;
}
void BSP_LED_On()
{
	GPIOB->BSRR |= GPIO_BSRR_BS_3;
}
void BSP_LED_Off()
{
	GPIOB->BSRR |= GPIO_BSRR_BR_3;
}
/*
void BSP_Ultrason_Init()
{
	// Enable GPIOB clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	// Configure PB3 as output
	GPIOA->MODER &= ~GPIO_MODER_MODER0;
	GPIOA->MODER |= (0x01 <<GPIO_MODER_MODER0_Pos);

	// Configure PB3 as Push-Pull output
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_0;

	// Configure PB3 as High-Speed Output
	GPIOA->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR0_Msk;
	GPIOA->OSPEEDR |= (0x03 <<GPIO_OSPEEDER_OSPEEDR0_Pos);

	// Disable PB3 Pull-up/Pull-down
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR0_Msk;

	// Set Initial State OFF
	GPIOA->BSRR = GPIO_BSRR_BR_0;
}*/

void BSP_SONAR()
{
	// Enable GPIOB clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	// Configure PA2 as Alternate function
	GPIOA->MODER &= ~(GPIO_MODER_MODER0_Msk);
	GPIOA->MODER |=  (0x02 <<GPIO_MODER_MODER0_Pos);
	// Set PA2 to AF1 (TIM2_CH1)
	GPIOA->AFR[0] &= ~(0x0000000F);
	GPIOA->AFR[0] |=  (0x00000001);
	// Enable TIM2 clock
	RCC -> APB1ENR |= RCC_APB1ENR_TIM2EN;
	// Reset TIM2 configuration
	TIM2->CR1  = 0x0000;
	TIM2->CR2  = 0x0000;
	TIM2->CCER = 0x0000;
	// Set TIM3 prescaler
	// Fck = 48MHz -> /48000 = 1MHz counting frequency
	TIM2->PSC = (uint16_t) 64 -1;
	// Set Auto-Reload to maximum value
	TIM2->ARR = (uint16_t) 0xFFFF;
	// Setup Input Capture
	TIM2->CCMR1 = 0x0000;
	TIM2->CCMR2 = 0x0000;
	// Channel 1 input on TI1
	TIM2->CCMR1 |= (0x01 <<TIM_CCMR1_CC1S_Pos);
	// Channel 2 input also on TI1
	TIM2->CCMR1 |= (0x02 <<TIM_CCMR1_CC2S_Pos);
	// Filter with N=8
	TIM2->CCMR1 |= (0x00 <<TIM_CCMR1_IC1F_Pos) | (0x00 <<TIM_CCMR1_IC2F_Pos);
	// Select rising edge for channel 1
	TIM2->CCER |= (0x00 <<TIM_CCER_CC1NP_Pos) | (0x00 <<TIM_CCER_CC1P_Pos);
	// Select falling edge for channel 2
	TIM2->CCER |= (0x00 <<TIM_CCER_CC2NP_Pos) | (0x01 <<TIM_CCER_CC2P_Pos);
	// Enable capture on channel 1 & channel 2
	TIM2->CCER |= (0x01 <<TIM_CCER_CC1E_Pos) | (0x01 <<TIM_CCER_CC2E_Pos);
	// Choose Channel 1 as trigger input
	TIM2->SMCR |= (0x05 <<TIM_SMCR_TS_Pos);
	// Slave mode -> Resets counter when trigger occurs
	TIM2->SMCR |= (0x4 <<TIM_SMCR_SMS_Pos);

	TIM2->DIER |= TIM_DIER_CC2IE;
	//TIM2->DIER |= TIM_DIER_UIE;

	// Enable TIM3
	TIM2->CR1 |= TIM_CR1_CEN;


}
void BSP_DELAY_TIM_init(void)
{
	// Enable TIM6 clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

	// Reset TIM6 configuration
	TIM6->CR1 = 0x0000;
	TIM6->CR2 = 0x0000;

	// Set TIM6 prescaler
	// Fck = 64MHz -> /64 = 1MHz counting frequency
	TIM6->PSC = (uint16_t) 64 -1;

	// Set ARR to maximum value
	TIM6->ARR = (uint16_t) 0xFFFF;
}
void BSP_DELAY_TIM_us(uint16_t ms)
{
	// Resets TIM6 counter
	TIM6->EGR |= TIM_EGR_UG;

	// Start TIM6 counter
	TIM6->CR1 |= TIM_CR1_CEN;

	// Wait until TIM6 counter reaches delay
	while(TIM6->CNT < ms);

	// Stop TIM6 counter
	TIM6->CR1 &= ~TIM_CR1_CEN;
}
void BSP_PB_Init()
{
	// Enable GPIOC clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	// Configure PC13 as input
	GPIOA->MODER &= ~GPIO_MODER_MODER1_Msk;
	GPIOA->MODER |= (0x00 <<GPIO_MODER_MODER1_Pos);

	// Disable PC13 Pull-up/Pull-down
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR1_Msk;
}
uint8_t BSP_PB_GetState()
{
	uint8_t state;

	if ((GPIOA->IDR & GPIO_IDR_1) == GPIO_IDR_1)
	{
		state = 0;
	}
	else
	{
		state = 1;
	}

	return state;
}


