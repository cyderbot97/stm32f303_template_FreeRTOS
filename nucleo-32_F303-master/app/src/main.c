#include "main.h"

#define aucun 0
#define droite 1
#define gauche 2
#define off 0
#define on 1

#define Servo_Right_Leg TIM3->CCR1
#define Servo_Left_Leg TIM3->CCR2

#define Servo_Right_Hip TIM3->CCR3
#define Servo_Left_Hip TIM3->CCR4

static uint8_t SystemClock_Config(void);


xSemaphoreHandle sem_new_data_uart;

void OSCILLATEUR 	(void *pvParameters);
void HEAD_TRACKING 	(void *pvParameters);
void DECODE_UART_DATA 	(void *pvParameters);

uint8_t	  rx_dma_buffer[16];
float servo1,servo2,servo3,servo4,valeur;
uint16_t out_pulse;

int id_tag, x_tag, y_tag = 0;
int etat = 0;
int sens, marche;
int val;

int main()
{
	SystemClock_Config();

	BSP_Console_Init();
	delay_ms(100);

	BSP_SERVO_INIT();
	delay_ms(100);


	my_printf("\r\nConsole Ready!\r\n");

	sem_new_data_uart = xSemaphoreCreateBinary();

	xTaskCreate(OSCILLATEUR, "OSCILLATEUR", 256, NULL, 2, NULL);
	xTaskCreate(HEAD_TRACKING, "HEAD_TRACKING", 256, NULL, 3, NULL);
	xTaskCreate(DECODE_UART_DATA, "DECODE_UART_DATA", 256, NULL, 4, NULL);

	vTaskStartScheduler();

	while(1);
}

void DECODE_UART_DATA(void *pvParameters){

	portBASE_TYPE	xStatus;

	uart_init();

	NVIC_SetPriority(USART1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
	NVIC_EnableIRQ(USART1_IRQn);

	while(1){
		xStatus = xSemaphoreTake(sem_new_data_uart, 500);

		if (xStatus == pdPASS)
		{
			USART1->CR1 &= ~USART_CR1_UE;

			if(rx_dma_buffer[0]=='$'){
				switch (rx_dma_buffer[1]) {
					case 'T':
						etat = sscanf(&rx_dma_buffer[2],"%d,%d,%d",&id_tag,&x_tag,&y_tag);
						//my_printf("\r\n TAG ID = %d, x = %d, y = %d.\r\n", id_tag, x_tag, y_tag);
						break;

					default:
						my_printf("\r\n error TAG data\r\n");
						break;
				}
			}

			// Make sure DMA1 channel 5 is disabled
			while ( (DMA1_Channel5->CCR & DMA_CCR_EN) != 0)
			{
				DMA1_Channel5->CCR &= ~DMA_CCR_EN;
			}

			// Number of data items to transfer
			DMA1_Channel5->CNDTR = 16;

			// Enable DMA1 channel 5
			DMA1_Channel5->CCR |= DMA_CCR_EN;

			// Enable USART1
			USART1->CR1 |= USART_CR1_UE;
		}
		else{
			my_printf("\r\n . \r\n");
		}


	}
}

void HEAD_TRACKING(void *pvParameters){

	float error, integral;

	while(1){
		error = 0 - (x_tag-80);
		integral = integral + error;
		val = 1500 + (int)(2*error+ integral*0.2);

		if(val > 1800){
			if(val > 2000) val = 2000;
			marche = on;
			sens = droite;
		}
		else if(val < 1200){
			if(val < 1000) val = 1000;
			marche = on;
			sens = gauche;
		}
		else{
			marche = off;
			sens = aucun;
		}

		TIM1->CCR4 = val;
		vTaskDelay(10);
	}

}

void OSCILLATEUR(void *pvParameters){

	float F = 250;
	float R = 100; //torsion

	float dt = 10; //temps boucle

	float signal_sin,signal_cos;

	float t = 25; // init a 25 car cos(25*....) = 0 evite le sursaut au demarage


	while(1){

		if(marche == 1)
		{
			signal_cos = (F*cos((t/100)*2.0*3.14)); // signal de commande
			signal_sin =(R*sin((t/100)*2.0*3.14));

			if(sens == droite)
			{
				Servo_Right_Hip = 1500 - signal_sin;
				Servo_Left_Hip = 1500 + signal_sin;
			}
			else if(sens == gauche)
			{
				Servo_Right_Hip = 1500 + signal_sin;
				Servo_Left_Hip = 1500 - signal_sin;
			}else
			{
				Servo_Left_Hip = 1500;
				Servo_Right_Hip = 1500;
			}

			Servo_Left_Leg = 1500 + signal_cos + 50;
			Servo_Right_Leg = 1500 + signal_cos - 50;

			t++;
			if(t>=100){t=0;}
		}else
		{
			if((t != 25)&&(t != 75))
			{
				signal_cos = (F*cos((t/100)*2.0*3.14));

				Servo_Left_Leg = 1500 + signal_cos + 50;
				Servo_Right_Leg = 1500 + signal_cos - 50;

				t++;
				if(t>=100){t=0;}
			}else
			{
				Servo_Left_Hip = 0;
				Servo_Right_Hip = 0;
				Servo_Left_Leg = 0;
				Servo_Right_Leg = 0;
			}
		}
		vTaskDelay(dt);
	}


}

/*
 * 	Clock configuration for the Nucleo STM32F303K8 board
 *
 * 	Default solder bridges configuration is for HSI (HSE is not connected)
 * 	You can change this by swapping solder bridges SB4 and SB6 (see Nucleo User manual)
 *
 * 	HSI clock configuration provides :
 * 	- SYSCLK, AHB	-> 64MHz
 * 	- APB2			-> 64MHz
 * 	- APB1			-> 32MHz (periph) 64MHz (timers)
 *
 * 	HSE clock configuration from ST-Link 8MHz MCO provides :
 * 	- SYSCLK, AHB	-> 72MHz
 * 	- APB2			-> 72MHz
 * 	- APB1			-> 36MHz (periph) 72MHz (timers)
 *
 * 	Select configuration by setting one of these symbol in your build configuration
 * 	- USE_HSI
 * 	- USE_HSE
 *
 */


#ifdef USE_HSI

static uint8_t SystemClock_Config()
{
	uint32_t	status;
	uint32_t	timeout = 0;

	// Start 8MHz HSI (it should be already started at power on)
	RCC->CR |= RCC_CR_HSION;

	// Wait until HSI is stable
	timeout = 1000;

	do
	{
		status = RCC->CR & RCC_CR_HSIRDY;
		timeout--;
	} while ((status == 0) && (timeout > 0));

	if (timeout == 0) return (1);	// HSI error


	// Configure Flash latency according to the speed (2WS)
	FLASH->ACR &= ~FLASH_ACR_LATENCY;
	FLASH->ACR |= 0x02 <<0;

	// Set HSI as PLL input
	RCC->CFGR |= RCC_CFGR_PLLSRC_HSI_DIV2;	// 4MHz from HSI

	// Configure the main PLL
	#define PLL_MUL		16					// 4MHz HSI to 64MHz
	RCC->CFGR |= (PLL_MUL-2) <<18;

	// Enable the main PLL
	RCC-> CR |= RCC_CR_PLLON;

	// Configure AHB/APB prescalers
	// AHB  Prescaler = /1	-> 64 MHz
	// APB1 Prescaler = /2  -> 32/64 MHz
	// APB2 Prescaler = /1  -> 64 MHz
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

	// Wait until PLL is ready
	timeout = 1000;

	do
	{
		status = RCC->CR & RCC_CR_PLLRDY;
		timeout--;
	} while ((status == 0) && (timeout > 0));

	if (timeout == 0) return (2);	// PLL error


	// Select the main PLL as system clock source
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	// Wait until PLL is switched on
	timeout = 1000;

	do
	{
		status = RCC->CFGR & RCC_CFGR_SWS;
		timeout--;
	} while ((status != RCC_CFGR_SWS_PLL) && (timeout > 0));

	if (timeout == 0) return (3);	// SW error

	// Update System core clock
	SystemCoreClockUpdate();
	return (0);
}

#endif


#ifdef USE_HSE

static uint8_t SystemClock_Config()
{
	uint32_t	status;
	uint32_t	timeout;

	// Start HSE
	RCC->CR |= RCC_CR_HSEBYP;
	RCC->CR |= RCC_CR_HSEON;

	// Wait until HSE is ready
	timeout = 1000;

	do
	{
		status = RCC->CR & RCC_CR_HSERDY;
		timeout--;
	} while ((status == 0) && (timeout > 0));

	if (timeout == 0) return (1); 	// HSE error


	// Configure Flash latency according to the speed (2WS)
	FLASH->ACR &= ~FLASH_ACR_LATENCY;
	FLASH->ACR |= 0x02 <<0;

	// Configure the main PLL
	#define PLL_MUL		9	// 8MHz HSE to 72MHz
	RCC->CFGR |= (PLL_MUL-2) <<18;

	// Set HSE as PLL input
	RCC->CFGR |= RCC_CFGR_PLLSRC_HSE_PREDIV;

	// Enable the main PLL
	RCC-> CR |= RCC_CR_PLLON;

	// Configure AHB/APB prescalers
	// AHB  Prescaler = /1	-> 72 MHz
	// APB1 Prescaler = /2  -> 36/72 MHz
	// APB2 Prescaler = /1  -> 72 MHz
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

	// Wait until PLL is ready
	timeout = 1000;

	do
	{
		status = RCC->CR & RCC_CR_PLLRDY;
		timeout--;
	} while ((status == 0) && (timeout > 0));

	if (timeout == 0) return (2); 	// PLL error


	// Select the main PLL as system clock source
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	// Wait until PLL is switched on
	timeout = 1000;

	do
	{
		status = RCC->CR & RCC_CR_PLLRDY;
		timeout--;
	} while ((status != RCC_CFGR_SWS_PLL) && (timeout > 0));

	if (timeout == 0) return (3); 	// SW error


	// Update System core clock
	SystemCoreClockUpdate();
	return (0);
}

#endif
