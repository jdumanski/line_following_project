#ifndef MOTOR_DRIVER_H_
#define MOTOR_DRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>
#include "uart.h"

#define Encoder_A_Pin8		(1U<<8)
#define Encoder_A_Pin9		(1U<<9)
#define Encoder_B_Pin10		(1U<<10)
#define Encoder_B_Pin11		(1U<<11)

#define GPIO_PIN_CUSTOM_RESET		0
#define GPIO_PIN_CUSTOM_SET		32

void tim2_pa5_pwm(void);
void Delay(uint32_t duration);

void Tim2_Ch1_Init(void); //32-bit resolution
void Tim4_Ch1_Init(void);

void pwm_set_frequency(uint32_t Freq, uint32_t timer);
void pwm_set_dutycycle(uint32_t DutyCycle, uint32_t timer);

/** Motor A Functions **/
void Encoder_A_Init(void); //Initializing encoder on Motor A
void EXTI9_5_IRQHandler(void);
int get_Encoder_A_counts(void);
void reset_Encoder_A_counts(void);
int Motor_A_Dist_mm(uint16_t diameter);

void Motor_A_Forward(uint32_t speed);
void Motor_A_Reverse(uint32_t speed);
void Motor_A_Brake(void); // This is a soft break from the motor controller
void Motor_A_Status(void);

/** Motor B Functions **/
void Encoder_B_Init(void); //Initializing encoder on Motor B
void EXTI15_10_IRQHandler(void);
int get_Encoder_B_counts(void);
void reset_Encoder_B_counts(void);
int Motor_B_Dist_mm(uint16_t diameter);


void Motor_B_Forward(uint32_t speed);
void Motor_B_Reverse(uint32_t speed);
void Motor_B_Brake(void); // Make soft break
void Motor_B_Status(void);

void Motors_Forward(uint32_t speed);

// ARCHIVE
//void Tim2_Ch2_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_DRIVER_H_ */
