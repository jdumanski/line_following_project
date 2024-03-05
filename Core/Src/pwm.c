#include "stm32f4xx.h"
#include <stdio.h>


#define TIM2EN			(1U<<0)
#define TIM3EN			(1U<<1)
#define TIM4EN			(1U<<2)

#define CR1_CEN			(1U<<0)
#define OC1_PWM_MODE1	((1U<<5) | (1U<<6))
#define OC2_PWM_MODE1	((1U<<13) | (1U<<14))

#define SMS_ENC_MODE3	((1U<<0) | (1U<<1))

#define CCER_CC1E 		(1U<<0)
#define CCER_CC1P 		(1U<<1)
#define CCER_CC1NP 		(1U<<3)

#define CCER_CC2E		(1U<<4)
#define CCER_CC2P 		(1U<<5)
#define CCER_CC2NP 		(1U<<7)

#define GPIOAEN			(1U<<0)
#define GPIOBEN			(1U<<1)


#define TIM_PRESCALER	8
#define ARR_PRESACLER	100
#define ARR_MAX			65535
#define DUTY_CYCLE		40
#define OFF				0

#define CHANNEL1		1
#define CHANNEL2		2

#define IN1				(1U<<5)
#define IN2				(1U<<4)
#define IN3				(1U<<6)
#define IN4				(1U<<7)



void pwm_set_frequency(uint32_t Freq);
void pwm_set_dutycycle(uint32_t DutyCycle, uint32_t channel);
void Motor_A_Status(void);
void Motor_B_Status(void);

/* Delay function  in milliseconds*/
void Delay(uint32_t duration)
{
	duration = duration * 1000;
	for (int i = 0; i < duration; i++){}
}

void tim2_pa5_pwm(void)
{
	/*Enable clock access to GPIOA*/
	RCC->AHB1ENR |= GPIOAEN;

	/*Set PA5 to alternate function mode*/
	GPIOA->MODER &= ~(1U<<10);
	GPIOA->MODER |= (1U<<11);

	/*Set alternate function type to TIM2_CH1*/
	GPIOA->AFR[0] |=  (1U<<20);
	GPIOA->AFR[0] &= ~(1U<<21);
	GPIOA->AFR[0] &= ~(1U<<22);
	GPIOA->AFR[0] &= ~(1U<<23);


	/*Enable clock access to tim2*/
	RCC->APB1ENR |= TIM2EN;
	/*Set prescaler value*/
	TIM2->PSC = 16000 - 1; // 16 000 000 / 16 000 = 1 000Hz
	/*Set auto-reload value
	 * This sets the motor frequency to 20kHz, which is a frequency at the edge of the
	 * human hearing spectrum*/
	TIM2->ARR = 20 - 1; // 1000Hz / 20 =  50Hz

	/*Set output compare toggle mode*/
	TIM2->CCMR1 |= OC1_PWM_MODE1;
	/*Set duty cycle of PWM	to 50% of ARR value*/
	TIM2->CCR1 = 50 - 1;
	/*Enable tim2 ch1 in compare mode*/
	TIM2->CCER |= CCER_CC1E;


	/*Clear counter*/
	TIM2->CNT = 0;
	/*Enable counter*/
	TIM2->CR1 |= CR1_CEN;
}

void Tim2_Ch1_Init(void)
{
	/*Enable clock access to GPIOA*/
	RCC->AHB1ENR |= GPIOAEN;

	/*Set PA0 to alternate function mode*/
	GPIOA->MODER &= ~(1U<<0);
	GPIOA->MODER |= (1U<<1);

	/*Configure the alternate function type to TIM2_CH1*/
	GPIOA->AFR[0] |=  (1U<<0);
	GPIOA->AFR[0] &= ~(1U<<1);
	GPIOA->AFR[0] &= ~(1U<<2);
	GPIOA->AFR[0] &= ~(1U<<3);

	/*Enable clock access to TIM2*/
	RCC->APB1ENR |= TIM2EN;

	/*Set prescaler value*/
	TIM2->PSC = TIM_PRESCALER - 1; // 16 000 000 / 8 = 2 000 000Hz

	/*Set auto-reload value
	 * By default, this sets the motor frequency to 20kHz, which is a frequency at the edge of the
	 * human hearing spectrum*/
	pwm_set_frequency(ARR_PRESACLER);

	/*Set output compare toggle mode*/
	TIM2->CCMR1 = OC1_PWM_MODE1; // Register unique to each channel

	/*Set duty cycle of PWM	% of ARR value
	 * By default, the duty cycle is set to 40% of the ARR_PRESCALER*/
	pwm_set_dutycycle(DUTY_CYCLE, CHANNEL1); // Register unique to each channel

	/*Enable Timer 2 Channel 1 in compare mode*/
	TIM2->CCER |= CCER_CC1E; // Register unique to each channel

	/*Clear counter*/
	TIM2->CNT = 0;
	/*Enable counter*/
	TIM2->CR1 |= CR1_CEN;
}

void Tim2_Ch2_Init(void)
{
	/*Enable clock access to GPIOA*/
	RCC->AHB1ENR |= GPIOAEN;

	/*Set PA1 to alternate function mode*/
	GPIOA->MODER &= ~(1U<<2);
	GPIOA->MODER |= (1U<<3);

	/*Configure the alternate function type to TIM2_CH1*/
	GPIOA->AFR[0] |=  (1U<<4);
	GPIOA->AFR[0] &= ~(1U<<5);
	GPIOA->AFR[0] &= ~(1U<<6);
	GPIOA->AFR[0] &= ~(1U<<7);

	/*Enable clock access to TIM2*/
	RCC->APB1ENR |= TIM2EN;

	/*Set prescaler value*/
	TIM2->PSC = TIM_PRESCALER - 1; // 16 000 000 / 8 = 2 000 000Hz

	/*Set auto-reload value
	 * By default, this sets the motor frequency to 20kHz, which is a frequency at the edge of the
	 * human hearing spectrum*/
	pwm_set_frequency(ARR_PRESACLER);

	/*Set output compare toggle mode*/
	TIM2->CCMR1 = OC2_PWM_MODE1; // Register unique to each channel

	/*Set duty cycle of PWM	% of ARR value
	 * By default, the duty cycle is set to 40% of the ARR_PRESCALER*/
	pwm_set_dutycycle(DUTY_CYCLE, CHANNEL2); // Register unique to each channel

	/*Enable Timer 2 Channel 2 in compare mode*/
	TIM2->CCER |= CCER_CC2E; // Register unique to each channel

	/*Clear counter*/
	TIM2->CNT = 0;
	/*Enable counter*/
	TIM2->CR1 |= CR1_CEN;
}

/*Frequency prescaler dividing down 2Mhz to a usable frequency range
 * 2 000 000 / Freq = Desired Frequency
 */

void pwm_set_frequency(uint32_t Freq)
{
	TIM2->ARR = Freq - 1;
}

/*Set the duty cycle of the PWM signal*/
void pwm_set_dutycycle(uint32_t DutyCycle, uint32_t channel)
{
	if (channel == 1)
	{
		/*Set duty cycle of Timer 2 channel 1 PWM to % of ARR value*/
		TIM2->CCR1 = DutyCycle - 1;
	}
	else if (channel == 2)
	{
		/*Set duty cycle of Timer 9 channel 1 PWM to % of ARR value*/
		TIM2->CCR2 = DutyCycle - 1;
	}
}

static void MotorPin_Init(void)
{
	/*Enable clock access to GPIOA*/
	RCC->AHB1ENR |= GPIOAEN;

	/*Set PA5 to output mode*/
	GPIOA->MODER |= (1U<<10);
	GPIOA->MODER &= ~(1U<<11);
	/*Set PA5 to pull down mode*/
	GPIOA->PUPDR &= ~(1U<<10);
	GPIOA->PUPDR |= (1U<<11);

	/*Set PA4 to output mode*/
	GPIOA->MODER |= (1U<<8);
	GPIOA->MODER &= ~(1U<<9);
	/*Set PA4 to pull down mode*/
	GPIOA->PUPDR &= ~(1U<<8);
	GPIOA->PUPDR |= (1U<<9);

	/*Set PA6 to output mode*/
	GPIOA->MODER |= (1U<<12);
	GPIOA->MODER &= ~(1U<<13);
	/*Set PA6 to pull down mode*/
	GPIOA->PUPDR &= ~(1U<<12);
	GPIOA->PUPDR |= (1U<<13);

	/*Set PA7 to output mode*/
	GPIOA->MODER |= (1U<<14);
	GPIOA->MODER &= ~(1U<<15);
	/*Set PA7 to pull down mode*/
	GPIOA->PUPDR &= ~(1U<<14);
	GPIOA->PUPDR |= (1U<<15);
}

void Encoder_A_Init(void)// Configuring Timer 4 channel 1 and 2 for encoder readings
{
	/*Enable clock access to GPIOB*/
	RCC->AHB1ENR |= GPIOBEN;

	/*Set PB6 and PB7 to alternate function mode*/
	GPIOD->MODER &= ~(1U<<12);
	GPIOD->MODER |= (1U<<13);
	GPIOD->MODER &= ~(1U<<14);
	GPIOD->MODER |= (1U<<15);

	/*Configure the alternate function type to TIM4_CH1*/
	GPIOD->AFR[0] &= ~(1U<<24);
	GPIOD->AFR[0] |=  (1U<<25);
	GPIOD->AFR[0] &= ~(1U<<26);
	GPIOD->AFR[0] &= ~(1U<<27);

	/*Configure the alternate function type to TIM4_CH2*/
	GPIOD->AFR[0] &= ~(1U<<28);
	GPIOD->AFR[0] |=  (1U<<29);
	GPIOD->AFR[0] &= ~(1U<<30);
	GPIOD->AFR[0] &= ~(1U<<31);

	/*Enable clock access to TIM4*/
	RCC->APB1ENR |= TIM4EN;

	/*Set auto-reload value*/
	TIM4->ARR = ARR_MAX;

	/*Setting Encoder mode*/
	TIM4->SMCR |= SMS_ENC_MODE3;
//	TIM4->SMCR &= ~(1U<<1);
//	TIM4->SMCR &= ~(1U<<2);

	/*Set timer channels in capture (input) mode*/
	// Channel 1 is mapped on TI1
	TIM4->CCMR1 &= ~(1U<<0);
	TIM4->CCMR1 |= (1U<<1);
	// Channel 2 is mapped on TI2
	TIM4->CCMR1 &= ~(1U<<8);
	TIM4->CCMR1 |= (1U<<9);

	/*Setting the Encoder polarity*/
	// Channel 1
	TIM4->CCER &= ~CCER_CC1P;
	TIM4->CCER &= ~CCER_CC1NP;
	// Channel 2
	TIM4->CCER &= ~CCER_CC2P;
	TIM4->CCER &= ~CCER_CC2NP;

	/*Setting the Encoder filter*/
	// Channel 1
	TIM4->CCMR1 &= ~(1U<<4);
	TIM4->CCMR1 &= ~(1U<<5);
	TIM4->CCMR1 &= ~(1U<<6);
	TIM4->CCMR1 &= ~(1U<<7);
	// Channel 2
	TIM4->CCMR1 &= ~(1U<<12);
	TIM4->CCMR1 &= ~(1U<<13);
	TIM4->CCMR1 &= ~(1U<<14);
	TIM4->CCMR1 &= ~(1U<<15);


	/*Enable Timer 4 Channel 1 in capture mode*/
	TIM4->CCER |= CCER_CC1E; // Register unique to each channel
	/*Enable Timer 4 Channel 2 in capture mode*/
	TIM4->CCER |= CCER_CC2E; // Register unique to each channel

	/*Clear counter*/
//	TIM4->CNT = 0;

	/*Enable counter*/
	TIM4->CR1 |= CR1_CEN;
}

/* NOTE: Have to add deadtime delay to prevent shoothrough*/
void Motor_A_Forward(uint32_t speed)
{
	MotorPin_Init();
	GPIOA->ODR |= IN1;
	GPIOA->ODR &= ~IN2;
	pwm_set_dutycycle(speed, CHANNEL1);
//	Motor_A_Status();
}
void Motor_A_Reverse(uint32_t speed)
{
	MotorPin_Init();
	GPIOA->ODR &= ~IN1;
	GPIOA->ODR |= IN2;
	pwm_set_dutycycle(speed, CHANNEL1);
//	Motor_A_Status();
}

void Motor_A_Brake(void)
{
	MotorPin_Init();
	GPIOA->ODR &= ~IN1;
	GPIOA->ODR &= ~IN2;
	Motor_A_Status();
}

void Motor_A_Status(void)
{
	if (!(GPIOA->ODR & (1U<<5)))
	{
		printf("IN1 OFF...\n\r");
	}
	else if ((GPIOA->ODR & (1U<<5)))
	{
		printf("IN1 ON...\n\r");
	}


	if (!(GPIOA->ODR & (1U<<4)))
	{
		printf("IN2 OFF...\n\r");
	}
	else if ((GPIOA->ODR & (1U<<4)))
	{
		printf("IN2 ON...\n\r");
	}
}

//////////////////////////////////////////////
void Motor_B_Forward(uint32_t speed)
{
	MotorPin_Init();
	GPIOA->ODR |= IN3;
	GPIOA->ODR &= ~IN4;
	pwm_set_dutycycle(speed, CHANNEL2);
	Motor_B_Status();
}

void Motor_B_Reverse(uint32_t speed)
{
	MotorPin_Init();
	GPIOA->ODR &= ~IN3;
	GPIOA->ODR |= IN4;
	pwm_set_dutycycle(speed, CHANNEL2);
//	Motor_B_Status();
}
void Motor_B_Brake(void)
{
	MotorPin_Init();
	GPIOA->ODR &= ~IN3;
	GPIOA->ODR &= ~IN4;
	Motor_B_Status();
}

void Motor_B_Status(void)
{
	if (!(GPIOA->ODR & (1U<<6)))
	{
		printf("IN3 OFF...\n\r");
	}
	else if ((GPIOA->ODR & (1U<<6)))
	{
		printf("IN3 ON...\n\r");
	}


	if (!(GPIOA->ODR & (1U<<7)))
	{
		printf("IN4 OFF...\n\r");
	}
	else if ((GPIOA->ODR & (1U<<7)))
	{
		printf("IN4 ON...\n\r");
	}
}
