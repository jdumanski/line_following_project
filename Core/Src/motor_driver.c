#include "stm32f4xx.h"
#include "motor_driver.h"
#include <stdio.h>

#define SYSCFGEN		(1U<<14)

#define EXTI8			((1U<<0) | (1U<<1) | (1U<<2) | (1U<<3))
#define IMR_MR8			(1U<<8)
#define RTSR_TR8		(1U<<8)
#define FTSR_TR8		(1U<<8)

#define EXTI10			((1U<<8) | (1U<<9) | (1U<<10) | (1U<<11))
#define IMR_MR10		(1U<<10)
#define RTSR_TR10		(1U<<10)
#define FTSR_TR10		(1U<<10)


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
#define PPR				1920 // Encoder pulses per full revolution
#define PI				3.1415

#define TIMER2			2
#define TIMER4			4

#define IN1				(1U<<5)
#define IN2				(1U<<4)
#define IN3				(1U<<6)
#define IN4				(1U<<7)

long int Encoder_A_Pin8_Last = 0;
long int Encoder_A_counts = 0;
long int Encoder_B_Pin10_Last = 0;
long int Encoder_B_counts = 0;
uint8_t direction_A = 0;
uint8_t direction_B = 0;
int Motor_A_Distance = 0;
int Motor_B_Distance = 0;

void pwm_set_frequency(uint32_t Freq, uint32_t timer);
void pwm_set_dutycycle(uint32_t DutyCycle, uint32_t timer);

void Motor_A_Status(void);
static void exti8_callback(void);
int get_Encoder_A_counts(void);
void reset_Encoder_A_counts(void);

void Motor_B_Status(void);
static void exti10_callback(void);
int get_Encoder_B_counts(void);
void reset_Encoder_B_counts(void);


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
	TIM2->PSC = TIM_PRESCALER - 1; // 16 000 000 / 16 000 = 1 000Hz
	/*Set auto-reload value
	 * This sets the motor frequency to 20kHz, which is a frequency at the edge of the
	 * human hearing spectrum*/
	pwm_set_frequency(ARR_PRESACLER, TIMER2);

	/*Set output compare toggle mode*/
	TIM2->CCMR1 |= OC1_PWM_MODE1;
	/*Set duty cycle of PWM	to 50% of ARR value*/
	pwm_set_dutycycle(30, TIMER2); // Register unique to each channel
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
	pwm_set_frequency(ARR_PRESACLER, TIMER2);

	/*Set output compare toggle mode*/
	TIM2->CCMR1 = OC1_PWM_MODE1; // Register unique to each channel

	/*Set duty cycle of PWM	% of ARR value
	 * By default, the duty cycle is set to 40% of the ARR_PRESCALER*/
	pwm_set_dutycycle(DUTY_CYCLE, TIMER2); // Register unique to each channel

	/*Enable Timer 2 Channel 1 in compare mode*/
	TIM2->CCER |= CCER_CC1E; // Register unique to each channel

	/*Clear counter*/
	TIM2->CNT = 0;
	/*Enable counter*/
	TIM2->CR1 |= CR1_CEN;
}

void Tim4_Ch1_Init(void)
{
	/*Enable clock access to GPIOB*/
	RCC->AHB1ENR |= GPIOBEN;

	/*Set PB6 to alternate function mode*/
	GPIOB->MODER &= ~(1U<<12);
	GPIOB->MODER |= (1U<<13);

	/*Configure the alternate function type to TIM4_CH1*/
	GPIOB->AFR[0] &= ~(1U<<24);
	GPIOB->AFR[0] |= (1U<<25);
	GPIOB->AFR[0] &= ~(1U<<26);
	GPIOB->AFR[0] &= ~(1U<<27);

	/*Enable clock access to TIM4*/
	RCC->APB1ENR |= TIM4EN;

	/*Set prescaler value*/
	TIM4->PSC = TIM_PRESCALER - 1; // 16 000 000 / 8 = 2 000 000Hz

	/*Set auto-reload value
	 * By default, this sets the motor frequency to 20kHz, which is a frequency at the edge of the
	 * human hearing spectrum*/
	pwm_set_frequency(ARR_PRESACLER, TIMER4);

	/*Set output compare toggle mode*/
	TIM4->CCMR1 = OC1_PWM_MODE1; // Register unique to each channel

	/*Set duty cycle of PWM	% of ARR value
	 * By default, the duty cycle is set to 40% of the ARR_PRESCALER*/
	pwm_set_dutycycle(DUTY_CYCLE, TIMER4); // Register unique to each channel

	/*Enable Timer 4 Channel 1 in compare mode*/
	TIM4->CCER |= CCER_CC1E; // Register unique to each channel

	/*Clear counter*/
	TIM4->CNT = 0;
	/*Enable counter*/
	TIM4->CR1 |= CR1_CEN;
}

/*Frequency prescaler dividing down 2Mhz to a usable frequency range
 * 2 000 000 / Freq = Desired Frequency
 */
void pwm_set_frequency(uint32_t Freq, uint32_t timer)
{
	if (timer == 2)
	{
		TIM2->ARR = Freq - 1;
	}
	else if (timer == 4)
	{
		TIM4->ARR = Freq - 1;
	}

}

/*Set the duty cycle of the PWM signal*/
void pwm_set_dutycycle(uint32_t DutyCycle, uint32_t timer)
{
	if (timer == 2)
	{
		/*Set duty cycle of Timer 2 channel 1 PWM to % of ARR value*/
		TIM2->CCR1 = DutyCycle - 1;
	}
	else if (timer == 4)
	{
		/*Set duty cycle of Timer 4 channel 1 PWM to % of ARR value*/
		TIM4->CCR1 = DutyCycle - 1;
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


/* NOTE: Have to add deadtime delay to prevent shoothrough*/
void Motor_A_Forward(uint32_t speed)
{
	MotorPin_Init();
	GPIOA->ODR |= IN1;
	GPIOA->ODR &= ~IN2;
	pwm_set_dutycycle(speed, TIMER2);
//	Motor_A_Status();
}
void Motor_A_Reverse(uint32_t speed)
{
	MotorPin_Init();
	GPIOA->ODR &= ~IN1;
	GPIOA->ODR |= IN2;
	pwm_set_dutycycle(speed, TIMER2);
//	Motor_A_Status();
}

void Motor_A_Brake(void)
{
	MotorPin_Init();
	GPIOA->ODR &= ~IN1;
	GPIOA->ODR &= ~IN2;
//	Motor_A_Status();
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
	pwm_set_dutycycle(speed, TIMER4);
//	Motor_B_Status();
}

void Motor_B_Reverse(uint32_t speed)
{
	MotorPin_Init();
	GPIOA->ODR &= ~IN3;
	GPIOA->ODR |= IN4;
	pwm_set_dutycycle(speed, TIMER4);
//	Motor_B_Status();
}
void Motor_B_Brake(void)
{
	MotorPin_Init();
	GPIOA->ODR &= ~IN3;
	GPIOA->ODR &= ~IN4;
//	Motor_B_Status();
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

void Encoder_A_Init(void)
{
	/*Disable global interrupts*/
	__disable_irq();

	/*Enable clock access to GPIOA*/
	RCC->AHB1ENR |= GPIOAEN;

	/*Set PA8 to input mode*/
	GPIOA->MODER &= ~(1U<<16);
	GPIOA->MODER &= ~(1U<<17);
	/*Set PA8 to no pull mode*/
	GPIOA->PUPDR &= ~(1U<<16);
	GPIOA->PUPDR &= ~(1U<<17);

	/*Set PA9 to input mode*/
	GPIOA->MODER &= ~(1U<<18);
	GPIOA->MODER &=	~(1U<<19);
	/*Set PA9 to no pull mode*/
	GPIOA->PUPDR &= ~(1U<<18);
	GPIOA->PUPDR &= ~(1U<<19);


	/*Enable clock access to SYSCFG*/
	RCC->APB2ENR |= SYSCFGEN;

	/*Select PORTA on EXTI8*/
	SYSCFG->EXTICR[2] &= ~EXTI8; //EXTICR[3:0] chooses from the four configuration registers

	/*Unmask EXTI8*/
	EXTI->IMR |= IMR_MR8;
	/*Select rising edge trigger*/
	EXTI->RTSR |= RTSR_TR8;
	/*Select falling edge trigger*/
	EXTI->FTSR |= FTSR_TR8;

	/*Set priority of EXTI8 in NVIC*/
	NVIC_SetPriority(EXTI9_5_IRQn, 0);
	/*Enable EXTI8 line in NVIC*/
	NVIC_EnableIRQ(EXTI9_5_IRQn); //EXTI9_5_IRQn selects external Line[9:5] interrupts

	/*Enable global interrupts*/
	__enable_irq();

}

static void exti8_callback(void)
{
	long int last_state_A = GPIOA->IDR & Encoder_A_Pin8;
	if ((Encoder_A_Pin8_Last == GPIO_PIN_CUSTOM_RESET) && (last_state_A == GPIO_PIN_CUSTOM_SET))
	{
		long int val_A = GPIOA->IDR & Encoder_A_Pin9;
		if ((val_A == GPIO_PIN_CUSTOM_RESET) && direction_A)
		{
			direction_A = 0; // Reverse
		}
		else if ((val_A == GPIO_PIN_CUSTOM_SET) && (direction_A == 0))
		{
			direction_A = 1; // Forward
		}
	}
	Encoder_A_Pin8_Last = last_state_A;

	if (!direction_A)
		Encoder_A_counts++;
	else
		Encoder_A_counts--;
}

void EXTI9_5_IRQHandler(void)
{
	if((EXTI->PR & Encoder_A_Pin8)!=0) //If Pending register on Line 8 is triggered
	{
		/*Clear PR flag*/
		EXTI->PR |= Encoder_A_Pin8;

		exti8_callback();
	}

}

int get_Encoder_A_counts(void)
{
	return Encoder_A_counts;
}

void reset_Encoder_A_counts(void)
{
	Encoder_A_counts = 0;
}

int Motor_A_Dist_mm(uint16_t diameter)
{
	Motor_A_Distance = (2*PI*(diameter/2)*get_Encoder_A_counts())/PPR;

	return Motor_A_Distance;
}

void Encoder_B_Init(void)
{
	/*Disable global interrupts*/
	__disable_irq();

	/*Enable clock access to GPIOA*/
	RCC->AHB1ENR |= GPIOAEN;

	/*Set PA10 to input mode*/
	GPIOA->MODER &= ~(1U<<20);
	GPIOA->MODER &= ~(1U<<21);
	/*Set PA10 to no pull mode*/
	GPIOA->PUPDR &= ~(1U<<20);
	GPIOA->PUPDR &= ~(1U<<21);

	/*Set PA11 to input mode*/
	GPIOA->MODER &= ~(1U<<22);
	GPIOA->MODER &=	~(1U<<23);
	/*Set PA11 to no pull mode*/
	GPIOA->PUPDR &= ~(1U<<22);
	GPIOA->PUPDR &= ~(1U<<23);

	/*Enable clock access to SYSCFG*/
	RCC->APB2ENR |= SYSCFGEN;

	/*Select PORTA on EXTI10*/
	SYSCFG->EXTICR[2] &= ~EXTI10; //EXTICR[3:0] chooses from the four configuration registers

	/*Unmask EXTI10*/
	EXTI->IMR |= IMR_MR10;
	/*Select rising edge trigger*/
	EXTI->RTSR |= RTSR_TR10;
	/*Select falling edge trigger*/
	EXTI->FTSR |= FTSR_TR10;

	/*Set priority of EXTI10 in NVIC*/
	NVIC_SetPriority(EXTI15_10_IRQn, 0);
	/*Enable EXTI10 line in NVIC*/
	NVIC_EnableIRQ(EXTI15_10_IRQn); //EXTI15_10_IRQn selects external Line[15:10] interrupts

	/*Enable global interrupts*/
	__enable_irq();

}

static void exti10_callback(void)
{
	long int last_state_B = GPIOA->IDR & Encoder_B_Pin10;
	if ((Encoder_B_Pin10_Last == GPIO_PIN_CUSTOM_RESET) && (last_state_B == GPIO_PIN_CUSTOM_SET))
	{
		long int val_B = GPIOA->IDR & Encoder_B_Pin11;
		if ((val_B == GPIO_PIN_CUSTOM_RESET) && direction_B)
		{
			direction_B = 0; // Reverse
		}
		else if ((val_B == GPIO_PIN_CUSTOM_SET) && (direction_B == 0))
		{
			direction_B = 1; // Forward
		}
	}
	Encoder_B_Pin10_Last = last_state_B;

	if (!direction_B)
		Encoder_B_counts++;
	else
		Encoder_B_counts--;
}

void EXTI15_10_IRQHandler(void)
{
	if((EXTI->PR & Encoder_B_Pin10)!=0) //If Pending register on Line 10 is triggered
	{
		/*Clear PR flag*/
		EXTI->PR |= Encoder_B_Pin10;

		exti10_callback();
	}

}

int get_Encoder_B_counts(void)
{
	return Encoder_B_counts;
}

void reset_Encoder_B_counts(void)
{
	Encoder_B_counts = 0;
}

int Motor_B_Dist_mm(uint16_t diameter)
{
	Motor_B_Distance = (2*PI*(diameter/2)*get_Encoder_B_counts())/PPR;

	return Motor_B_Distance;
}

/***ARCHIVE***/

/*
void Tim2_Ch2_Init(void)
{
	Enable clock access to GPIOA
	RCC->AHB1ENR |= GPIOAEN;

	Set PA1 to alternate function mode
	GPIOA->MODER &= ~(1U<<2);
	GPIOA->MODER |= (1U<<3);

	Configure the alternate function type to TIM2_CH1
	GPIOA->AFR[0] |=  (1U<<4);
	GPIOA->AFR[0] &= ~(1U<<5);
	GPIOA->AFR[0] &= ~(1U<<6);
	GPIOA->AFR[0] &= ~(1U<<7);

	Enable clock access to TIM2
	RCC->APB1ENR |= TIM2EN;

	Set prescaler value
	TIM2->PSC = TIM_PRESCALER - 1; // 16 000 000 / 8 = 2 000 000Hz

	Set auto-reload value
	 * By default, this sets the motor frequency to 20kHz, which is a frequency at the edge of the
	 * human hearing spectrum
	pwm_set_frequency(ARR_PRESACLER);

	Set output compare toggle mode
	TIM2->CCMR1 = OC2_PWM_MODE1; // Register unique to each channel

	Set duty cycle of PWM	% of ARR value
	 * By default, the duty cycle is set to 40% of the ARR_PRESCALER
	pwm_set_dutycycle(DUTY_CYCLE, CHANNEL2); // Register unique to each channel

	Enable Timer 2 Channel 2 in compare mode
	TIM2->CCER |= CCER_CC2E; // Register unique to each channel

	Clear counter
	TIM2->CNT = 0;
	Enable counter
	TIM2->CR1 |= CR1_CEN;
}

void Encoder_A_Init(void)// Configuring Timer 4 channel 1 and 2 for encoder readings
{
	Enable clock access to GPIOB
	RCC->AHB1ENR |= GPIOBEN;

	Set PB6 and PB7 to alternate function mode, CHANGE TO DIFFERENT PINS
	GPIOD->MODER &= ~(1U<<12);
	GPIOD->MODER |= (1U<<13);
	GPIOD->MODER &= ~(1U<<14);
	GPIOD->MODER |= (1U<<15);

	Configure the alternate function type to TIM4_CH1
	GPIOD->AFR[0] &= ~(1U<<24);
	GPIOD->AFR[0] |=  (1U<<25);
	GPIOD->AFR[0] &= ~(1U<<26);
	GPIOD->AFR[0] &= ~(1U<<27);

	Configure the alternate function type to TIM4_CH2
	GPIOD->AFR[0] &= ~(1U<<28);
	GPIOD->AFR[0] |=  (1U<<29);
	GPIOD->AFR[0] &= ~(1U<<30);
	GPIOD->AFR[0] &= ~(1U<<31);

	Enable clock access to TIM4
	RCC->APB1ENR |= TIM4EN;

	Set auto-reload value
	TIM4->ARR = ARR_MAX;

	Setting Encoder mode
	TIM4->SMCR |= SMS_ENC_MODE3;
//	TIM4->SMCR &= ~(1U<<1);
//	TIM4->SMCR &= ~(1U<<2);

	Set timer channels in capture (input) mode
	// Channel 1 is mapped on TI1
	TIM4->CCMR1 &= ~(1U<<0);
	TIM4->CCMR1 |= (1U<<1);
	// Channel 2 is mapped on TI2
	TIM4->CCMR1 &= ~(1U<<8);
	TIM4->CCMR1 |= (1U<<9);

	Setting the Encoder polarity
	// Channel 1
	TIM4->CCER &= ~CCER_CC1P;
	TIM4->CCER &= ~CCER_CC1NP;
	// Channel 2
	TIM4->CCER &= ~CCER_CC2P;
	TIM4->CCER &= ~CCER_CC2NP;

	Setting the Encoder filter
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


	Enable Timer 4 Channel 1 in capture mode
	TIM4->CCER |= CCER_CC1E; // Register unique to each channel
	Enable Timer 4 Channel 2 in capture mode
	TIM4->CCER |= CCER_CC2E; // Register unique to each channel

	Clear counter
//	TIM4->CNT = 0;

	Enable counter
	TIM4->CR1 |= CR1_CEN;
}
*/
