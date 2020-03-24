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

#define Servo_Head TIM1->CCR4

static uint8_t SystemClock_Config(void);

// Define Event Group flags
#define	Head_Reset			( (EventBits_t)( 0x01 <<0) )
#define	Head_Turn_Stop		( (EventBits_t)( 0x01 <<1) )
#define Head_Turn_Right		( (EventBits_t)( 0x01 <<2) )
#define Head_Turn_Left		( (EventBits_t)( 0x01 <<3) )
#define Head_Pos_Max_Right	( (EventBits_t)( 0x01 <<4) )
#define Head_Pos_Max_Left	( (EventBits_t)( 0x01 <<5) )

#define Walk_Stop			( (EventBits_t)( 0x01 <<0) )
#define	Walk_Forward		( (EventBits_t)( 0x01 <<1) )
#define	Walk_Backward		( (EventBits_t)( 0x01 <<2) )
#define	Walk_Turn_Stop		( (EventBits_t)( 0x01 <<3) )
#define Walk_Turn_Right		( (EventBits_t)( 0x01 <<4) )
#define Walk_Turn_Left		( (EventBits_t)( 0x01 <<5) )


// Kernel objects
EventGroupHandle_t EventGroupHead;
EventGroupHandle_t EventGroupWalk;

xSemaphoreHandle xSem_New_Data_Uart;
xSemaphoreHandle xSem_New_Data_Sonar;

xSemaphoreHandle xSem_Tag_Found;
xSemaphoreHandle xSem_Tag_Lost;

xSemaphoreHandle xSem_Obstacle_Disapear;
xSemaphoreHandle xSem_Obstacle_Present;

void CMD_HEAD 	(void *pvParameters);
void OSCILLATEUR 	(void *pvParameters);
void HEAD_TRACKING 	(void *pvParameters);
void DECODE_UART_DATA 	(void *pvParameters);
void GET_SONAR 	(void *pvParameters);

uint8_t rx_dma_buffer[16];
int sonar_data;
float servo1,servo2,servo3,servo4,valeur;
uint16_t out_pulse;

int id_tag, x_tag, y_tag = 0;
int val;

int main()
{
	//Initialize Clock (64MHz)
	SystemClock_Config();

	//Initialize Console
	BSP_Console_Init();
	delay_ms(100);
	//Initialize LED for debug
	BSP_LED_Init();
	delay_ms(100);

	//Initialize Servo
	BSP_SERVO_INIT();
	delay_ms(100);

	// Create Event Group                   // <-- Create Event Group here
	EventGroupHead = xEventGroupCreate();
	EventGroupWalk = xEventGroupCreate();

	// Create Semaphore object
	xSem_New_Data_Uart 		= xSemaphoreCreateBinary();
	xSem_New_Data_Sonar 	= xSemaphoreCreateBinary();

	xSem_Tag_Found 			= xSemaphoreCreateBinary();
	xSem_Tag_Lost 			= xSemaphoreCreateBinary();

	xSem_Obstacle_Disapear	= xSemaphoreCreateBinary();
	xSem_Obstacle_Present	= xSemaphoreCreateBinary();

	// Create Tasks
	//xTaskCreate(GET_SONAR, "GET_SONAR", 256, NULL, 3, NULL);
	xTaskCreate(CMD_HEAD, "CMD_HEAD", 256, NULL, 2, NULL);
	xTaskCreate(OSCILLATEUR, "OSCILLATEUR", 256, NULL, 2, NULL);
	//xTaskCreate(HEAD_TRACKING, "HEAD_TRACKING", 256, NULL, 2, NULL);
	//xTaskCreate(DECODE_UART_DATA, "DECODE_UART_DATA", 256, NULL, 4, NULL);

	//Start Scheduler
	my_printf("\r\n Pepi Ready! \r\n");
	vTaskStartScheduler();

	while(1){

	}
}

void CMD_HEAD(void *pvParameters){

	while(1)
	{
		//xEventGroupSetBits(EventGroupHead, Head_Turn_Stop);
		//vTaskDelay(3000);
		//my_printf("(RE)");
		//xEventGroupSetBits(EventGroupHead, Head_Reset);



		//vTaskDelay(1000);
		//my_printf("(ST)");
		//xEventGroupSetBits(EventGroupHead, Head_Turn_Stop);
		/*
		vTaskDelay(3000);
		my_printf("(Fr)");
		xEventGroupSetBits(EventGroupWalk, Walk_Forward);
		vTaskDelay(3000);
		my_printf("(St)");
		xEventGroupSetBits(EventGroupWalk, Walk_Stop);

		vTaskDelay(3000);
		my_printf("(Lf)");
		xEventGroupSetBits(EventGroupWalk, Walk_Turn_Left);

		vTaskDelay(3000);
		my_printf("(Tst)");
		xEventGroupSetBits(EventGroupWalk, Walk_Turn_Stop);

		vTaskDelay(3000);
		my_printf("(Ri)");
		xEventGroupSetBits(EventGroupWalk, Walk_Turn_Right);

		vTaskDelay(3000);
		my_printf("(Ba)");
		xEventGroupSetBits(EventGroupWalk, Walk_Backward);*/

	}

}

void HEAD_TRACKING(void *pvParameters){

	float error, integral;

	//Initialize Sonar
	//BSP_UTLRASONIC();

	//Set NVIC priority
	NVIC_SetPriority(TIM2_IRQn , configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2);
	NVIC_EnableIRQ(TIM2_IRQn);

	EventBits_t uxBits;

	while(1)
	{
		uxBits = xEventGroupWaitBits(EventGroupHead, Head_Reset | Head_Turn_Left | Head_Turn_Right | Head_Turn_Stop, pdTRUE, pdFALSE, portMAX_DELAY);

		if((uxBits & Head_Turn_Stop) == Head_Turn_Stop)
		{
			my_printf("St");
		}
		if((uxBits & Head_Reset) == Head_Reset)
		{
			while( xEventGroupWaitBits(EventGroupHead,Head_Reset | Head_Turn_Left | Head_Turn_Right | Head_Turn_Stop,	pdFALSE, pdTRUE, 10)
					== 0)
			{
				my_printf("Re");
				Servo_Head = 1500;
				xEventGroupSetBits(EventGroupHead, Head_Turn_Stop);
			}
			my_printf(".");
		}
		else if((uxBits & Head_Turn_Right) == Head_Turn_Right)
		{
			while( xEventGroupWaitBits(EventGroupHead,Head_Reset | Head_Turn_Left | Head_Turn_Right | Head_Turn_Stop,	pdFALSE, pdTRUE, 10)
					== 0)
			{
				my_printf("Ri");
				Servo_Head -= 10;
				if(Servo_Head < 1010) xEventGroupSetBits(EventGroupHead, Head_Turn_Stop);
			}
			my_printf(".");
		}
		else if((uxBits & Head_Turn_Left) == Head_Turn_Left)
		{
			while( xEventGroupWaitBits(EventGroupHead,Head_Reset | Head_Turn_Left | Head_Turn_Right | Head_Turn_Stop,	pdFALSE, pdTRUE, 10)
					== 0)
			{
				my_printf("Le");
				Servo_Head += 10;
				if(Servo_Head > 1990) xEventGroupSetBits(EventGroupHead, Head_Turn_Stop);
			}
			my_printf(".");
		}


	}

}

void GET_SONAR(void *pvParameters){

	//Initialize Sonar
	BSP_UTLRASONIC();

	//Set NVIC priority
	NVIC_SetPriority(TIM2_IRQn , configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2);
	NVIC_EnableIRQ(TIM2_IRQn);

	while(1){
		xSemaphoreTake(xSem_New_Data_Sonar, portMAX_DELAY);

		if(sonar_data<350){
			xSemaphoreTake(xSem_Obstacle_Disapear,0);
			xSemaphoreGive(xSem_Obstacle_Present);
		}else{
			xSemaphoreTake(xSem_Obstacle_Present,0);
			xSemaphoreGive(xSem_Obstacle_Disapear);
		}


	}

}

void DECODE_UART_DATA(void *pvParameters){

	portBASE_TYPE	xStatus;

	//Initialize UART
	uart_init();

	//Set NVIC priority
	NVIC_SetPriority(USART1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
	NVIC_EnableIRQ(USART1_IRQn);

	while(1){
		xStatus = xSemaphoreTake(xSem_New_Data_Uart, 300);

		if (xStatus == pdPASS)
		{
			USART1->CR1 &= ~USART_CR1_UE;

			if(rx_dma_buffer[0]=='$'){
				switch (rx_dma_buffer[1]) {
					case 'T':
						sscanf(&rx_dma_buffer[2],"%d,%d,%d",&id_tag,&x_tag,&y_tag);
						xSemaphoreGive(xSem_Tag_Found);
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
			xSemaphoreGive(xSem_Tag_Lost);
		}


	}
}

void OSCILLATEUR(void *pvParameters){

	float F = 250;
	float R = 100; //torsion

	float dt = 10; //temps boucle

	float signal_sin,signal_cos;

	float t = 25; // init a 25 car cos(25*....) = 0 evite le sursaut au demarage


	EventBits_t uxBits;

	while(1)
	{

		uxBits = xEventGroupWaitBits(EventGroupWalk, Walk_Forward | Walk_Backward |Walk_Turn_Left | Walk_Turn_Right | Walk_Turn_Stop, pdTRUE, pdFALSE, portMAX_DELAY);
		//my_printf("!");
		while( xEventGroupWaitBits(EventGroupWalk,
				Walk_Stop | Walk_Forward | Walk_Backward | Walk_Turn_Left | Walk_Turn_Right | Walk_Turn_Stop,
				pdFALSE, pdFALSE, 9) == 0)
		{
			signal_cos = (F*cos((t/100)*2.0*3.14)); // signal de commande
			signal_sin =(R*sin((t/100)*2.0*3.14));

			if((uxBits & Walk_Turn_Left) == Walk_Turn_Left)
			{
				Servo_Right_Hip = 1500 - signal_sin;
				Servo_Left_Hip = 1500 + signal_sin;
			}
			else if((uxBits & Walk_Turn_Right) == Walk_Turn_Right)
			{
				Servo_Right_Hip = 1500 + signal_sin;
				Servo_Left_Hip = 1500 - signal_sin;
			}else
			{
				Servo_Right_Hip = 1500 - signal_sin;
				Servo_Left_Hip = 1500 - signal_sin;
			}

			if((uxBits & Walk_Backward) == Walk_Backward)
			{
				Servo_Left_Leg = 1500 - signal_cos + 50;
								Servo_Right_Leg = 1500 - signal_cos - 50;
			}
			else
			{
				Servo_Left_Leg = 1500 + signal_cos + 50;
				Servo_Right_Leg = 1500 + signal_cos - 50;
			}

			t++;
			if(t>=100){t=0;}
		}
		if((xEventGroupWaitBits(EventGroupWalk, Walk_Stop, pdTRUE, pdFALSE, 0) & Walk_Stop) == Walk_Stop)
		{
			//Eventgroup sur Walk_Stop -> on descend pepere
			//my_printf("|");
			while((t != 25)&&(t != 75))
			{
				signal_cos = (F*cos((t/100)*2.0*3.14));
				Servo_Left_Leg = 1500 + signal_cos + 50;
				Servo_Right_Leg = 1500 + signal_cos - 50;

				t++;
				if(t>=100){t=0;}
				vTaskDelay(10);
			}

			//On coupe les servo's
			Servo_Left_Hip = 0;
			Servo_Right_Hip = 0;
			Servo_Left_Leg = 0;
			Servo_Right_Leg = 0;
		}

		//my_printf(".");
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
