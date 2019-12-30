#include "stm32f3xx.h"
#include "main.h"
#include "bsp.h"
#include "delay.h"
#include "math.h"
#include "MadgwickAHRS.h"
#include "spi.h"
#include "mpu9250.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "queue.h"
#include "event_groups.h"
#include "stream_buffer.h"

static uint8_t SystemClock_Config(void);

uint8_t	  rx_dma_buffer[16];


uint16_t A;
uint16_t B;
uint16_t consigne_B;
uint16_t inclinaison;


float a,b,c;

float roll,pitch,yaw, tampon;
float kp,ki;

float error, integral, output;

int8_t mpu_data[14];

int16_t raw_ax, raw_ay, raw_az,imu_temp, raw_gx, raw_gy, raw_gz;
float imu_ax, imu_ay, imu_az,imu_gx,imu_gy,imu_gz;
float consigne;

void Obtenir_Inclinaison 	(void *pvParameters);
void Signal_Consigne 	(void *pvParameters);

int main()
{
	SystemClock_Config();

	a = 0 ;
	b = 0 ;
	c = 0 ;
	consigne = 0;

	kp = 0.9;
	ki = 10;

	/*
	 * Initialisation
	 */

	//BSP_LED_Init();
	//delay_ms(100);

	BSP_SPI1_Init();
	delay_ms(100);

	BSP_MPU9250_Init();
	delay_ms(100);

	BSP_Console_Init();
	delay_ms(100);

	servo_init();
	delay_ms(100);

	my_printf("\r\nConsole Ready!\r\n");


	xTaskCreate(Obtenir_Inclinaison, "Obtenir_Inclinaison", 256, NULL, 1, NULL);
	xTaskCreate(Signal_Consigne, "Signal_Consigne", 256, NULL, 2, NULL);

	// Start the Scheduler
	vTaskStartScheduler();

	// Loop forever
	while(1)
	{
	}
}

void Signal_Consigne (void *pvParameters){

	float min = -15;
	float max = 15;
	int t = 5; // temps entre 2 valeurs

	while(1){

		for(float i = min; i < max; i = i + 0.1){
			consigne = i;
			vTaskDelay(t);
		}
		for(float i = max; i > min; i = i - 0.1){
			consigne = i;
			vTaskDelay(t);
		}
	}
}

void Obtenir_Inclinaison (void *pvParameters)
{
	while(1){
		//BSP_LED_On(); //verifier le temps de boucle

		BSP_SPI_Read(MPUREG_ACCEL_XOUT_H, mpu_data, 14);

		raw_ax = ((int16_t)mpu_data[0]<<8) | (int16_t)mpu_data[1];
		raw_ay = ((int16_t)mpu_data[2]<<8) | (int16_t)mpu_data[3];
		raw_az = ((int16_t)mpu_data[4]<<8) | (int16_t)mpu_data[5];

		// Scale Accelerometers with offset cancellation
		imu_ax = raw_ax * MPU9250A_2g;
		imu_ay = raw_ay * MPU9250A_2g;
		imu_az = raw_az * MPU9250A_2g;

		// Record temperature
		imu_temp = ((int16_t)mpu_data[6]<<8) | (int16_t)mpu_data[7];

		raw_gx = ((int16_t)mpu_data[8]<<8)  | (int16_t)mpu_data[9];
		raw_gy = ((int16_t)mpu_data[10]<<8) | (int16_t)mpu_data[11];
		raw_gz = ((int16_t)mpu_data[12]<<8) | (int16_t)mpu_data[13];

		// Scale Gyros with offset cancellation
		imu_gx = raw_gx * MPU9250G_500dps;
		imu_gy = raw_gy * MPU9250G_500dps;
		imu_gz = raw_gz * MPU9250G_500dps;


		MadgwickAHRSupdateIMU(imu_gx, imu_gy, imu_gz, imu_ax, imu_ay, imu_az);

		roll = atan2f(2.0f * (q0*q1 + q2*q3), q0*q0 - q1*q1 - q2*q2 + q3*q3)*180/3.14;

		error = consigne - roll;

		integral = integral + error*0.006;

		output = kp*error + ki*integral;

		consigne_B = output*1000/120 + 1500;

		kinematic_bascule(consigne_B);

		//BSP_LED_Off();

		vTaskDelay(5);
	}
}

void kinematic_bascule(uint16_t inclinaison_pulse){

	//
	//calculate pulse width for 2 bascule motor
	//state machine

	if((inclinaison_pulse <= 1500) && (inclinaison_pulse >= 1000)){

		//calcul
		A = 1500 - (1500-inclinaison_pulse) * 3;
		B = inclinaison_pulse;

		if(A<1000){
			A = 1000;
		}

		TIM3->CCR1 = A;
		TIM3->CCR2 = B;

	}else if((inclinaison_pulse <= 2000) && (inclinaison_pulse >= 1500)){ // 1650 = 78 deg

		//Calcul
		B = 1500 + (inclinaison_pulse - 1500)*3;
		A = inclinaison_pulse;

		if(B>2000) {
			B = 2000;
		}

		//set PWM motor value
		TIM3->CCR2 = B;
		TIM3->CCR1 = A;

	}else {

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
