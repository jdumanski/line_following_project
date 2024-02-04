/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string>
#include <vector>
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim11;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */

struct Color
{
	uint8_t r = 0;
	uint8_t g = 0;
	uint8_t b = 0;
	Color(uint8_t r, uint8_t g, uint8_t b) : r(r), g(g), b(b){}
	double getMagnitude(){
		return sqrt(r*r+g*g+b*b);
	}
};

void USART_Transmit(UART_HandleTypeDef *huart, uint8_t *TextString) {
	uint8_t TextStringLength;

	/* Calculate the length of the text string to be sent */
	TextStringLength = 0;
	while (TextString[TextStringLength++] != '\0')
		;
	TextStringLength--;

	/* Use the HAL function to send the text string via USART */
	HAL_UART_Transmit(huart, TextString, TextStringLength, 10);
}

//return frequency
//uses low period

int calcTotalTime(int t1, int t0, int tMax){
	if(t0 <= t1){
		return t1-t0;
	}
	int returnVal = t1 + tMax - t0;
	return returnVal;
}

int calcPeriod(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin){
	uint8_t prev = 2; //prev output value (initially set as 2 bc output values can either be 0 or 1

	//time per count is 0.00001 sec
	const int MAX_TIME = 65535;
	bool started = false;

	HAL_TIM_Base_Start(&htim11);
	int t0 = __HAL_TIM_GET_COUNTER(&htim11);

	do{
		GPIO_PinState output = HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
		if(output==GPIO_PIN_RESET && prev==1){
			started = true;
			t0 = __HAL_TIM_GET_COUNTER(&htim11);
		}else if(started && output==GPIO_PIN_SET && prev==0){
			return calcTotalTime(__HAL_TIM_GET_COUNTER(&htim11), t0, MAX_TIME);
		}
		prev = output;
	}while(calcTotalTime(__HAL_TIM_GET_COUNTER(&htim11), t0, MAX_TIME) < MAX_TIME || !started); //if we have started (encountered falling edge)

	return -1; //if timer counts beyond max val (shouldn't happen -> max val is 0.65535 sec and max freq of output is 2Hz which has half period (since we only measure half period - time when signal is continuously low within a period) of 0.25 sec)
}

Color normalizeToRGB(int r, int g, int b){
	if(r<0 || b < 0 || g < 0){
		return Color(0, 0, 0);
	}
	double maxChannelVal = std::max(std::max(r, g), b);
	uint8_t red = r / maxChannelVal * 255;
	uint8_t green = g / maxChannelVal * 255;
	uint8_t blue = b / maxChannelVal * 255;
	return Color(red, green, blue);
}

int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_TIM11_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	GPIO_InitTypeDef freq; //color sensor frequency output
	freq.Pin = GPIO_PIN_0;
	freq.Mode = GPIO_MODE_INPUT;
	freq.Pull = GPIO_NOPULL;
	freq.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOA, &freq);

	GPIO_InitTypeDef s2;
	s2.Pin = GPIO_PIN_5;
	s2.Mode = GPIO_MODE_OUTPUT_PP;
	s2.Pull = GPIO_NOPULL;
	s2.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOA, &s2);

	GPIO_InitTypeDef s3;
	s3.Pin = GPIO_PIN_4;
	s3.Mode = GPIO_MODE_OUTPUT_PP;
	s3.Pull = GPIO_NOPULL;
	s3.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOA, &s3);

	//by default, s2 and s3 are zero, so we get freq of red as output

	GPIO_InitTypeDef s1;
	s1.Pin = GPIO_PIN_1;
	s1.Mode = GPIO_MODE_OUTPUT_PP;
	s1.Pull = GPIO_NOPULL;
	s1.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOA, &s1);

	GPIO_InitTypeDef s0;
	s0.Pin = GPIO_PIN_0;
	s0.Mode = GPIO_MODE_OUTPUT_PP;
	s0.Pull = GPIO_NOPULL;
	s0.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOB, &s0);

	//set output frequency scaling to 2% (s0 = 1, s1 = 1)
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); //s1
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); //s0

	//0.00001sec per tick bc hclock is 80MHz - we use prescale factor of 800 (set below), so tim11 is 80MHz/800 = 100KHz -> period of tim11 = 1/100000Hz = 0.00001 sec -> therefore each count in 0.00001 sec
	const double TIMER_PERIOD = 0.00001;
	while (1) {

		//set output to red channel
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //s3
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); //s2
		int redPeriod = calcPeriod(GPIOA, GPIO_PIN_0);
		int redFreq = 1.0 / (2 * redPeriod * TIMER_PERIOD);

		//set output to green channel
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); //s3
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); //s2
		int greenPeriod = calcPeriod(GPIOA, GPIO_PIN_0);
		int greenFreq = 1.0 / (2 * greenPeriod * TIMER_PERIOD);

		//set output to blue channel
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); //s3
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); //s2
		int bluePeriod = calcPeriod(GPIOA, GPIO_PIN_0);
		int blueFreq = 1.0 / (2 * bluePeriod * TIMER_PERIOD);

		//RGB OK in function, but when it gets to main the values are all weird - why?? - try doing caluclatuons in main instead of fucniton and see what happens
		Color color = normalizeToRGB(redFreq, greenFreq, blueFreq);
		std::string s = "(" + std::to_string(color.r) + ", " + std::to_string(color.g) + ", " + std::to_string(color.b) + ")";
		const char* c = s.c_str();
		USART_Transmit(&huart2, (uint8_t*)c);
		//uint8_t colorOutput = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
		//uint8_t colorOuputPrintable = colorOutput + 48;
		//uint8_t *s = &colorOuputPrintable;
		//USART_Transmit(&huart2, s);
		/*
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == 1) {
			USART_Transmit(&huart2, (uint8_t*) "charles");
		}
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == 1) {
					USART_Transmit(&huart2, (uint8_t*) "ryein");
		}
		*/
	}
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 800-1;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 65535;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
