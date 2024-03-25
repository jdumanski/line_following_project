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
#include <deque>
#include <math.h>
#include "motor_driver.h"
#include "uart.h"
#include <numeric>

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
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM3_Init(void);

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
	int r = 0;
	int g = 0;
	int b = 0;
	Color(int r, int g, int b) : r(r), g(g), b(b){}
	double getMagnitude(){
		return sqrt(r*r+g*g+b*b);
	}
	double dotProduct(Color other){
		return r*other.r + g*other.g + b*other.b;
	}
	double cosineSimilarity(Color otherC){
		return dotProduct(otherC)/(getMagnitude()*otherC.getMagnitude());
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


// calculates total number of counts between start and end count values
//
int calcTotalCount(int c1, int c0, int cMax){
	if(c0 <= c1){
		return c1-c0;
	}
	return c1 + cMax - c0;
}

// returns period
// uses low period
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
			return calcTotalCount(__HAL_TIM_GET_COUNTER(&htim11), t0, MAX_TIME);
		}
		prev = output;
	}while(calcTotalCount(__HAL_TIM_GET_COUNTER(&htim11), t0, MAX_TIME) < MAX_TIME || !started); //if we have started (encountered falling edge)

	return -1; //if timer counts beyond max val (shouldn't happen -> max val is 0.65535 sec and max freq of output is 2Hz which has half period (since we only measure half period - time when signal is continuously low within a period) of 0.25 sec)
}

Color normalizeToRGB(int r, int g, int b){
	if(r<0 || b < 0 || g < 0){
		return Color(0, 0, 0);
	}
	double maxChannelVal = std::max(std::max(r, g), b);
	int red = r / maxChannelVal * 255;
	int green = g / maxChannelVal * 255;
	int blue = b / maxChannelVal * 255;
	return Color(red, green, blue);
}

double colorToDisplacement(Color left, Color middle, Color right){
	Color red(255, 0, 0);
	double distance = 127.7*(1-1.088*middle.cosineSimilarity(red));
	if(right.cosineSimilarity(red) < left.cosineSimilarity(red)){
		return -distance;
	}
	return distance;
}

double differentiate(double fn, double fn_1, double fn_2, double timeStep){
	return (3*fn-4*fn_1+fn_2)/(2*timeStep);
}

//vector is in order of s3, s2, f
Color extractColorFromSensor(std::vector<std::pair<GPIO_TypeDef*, uint16_t>> pins, double timerPeriod){
	//set output to red channel
	HAL_GPIO_WritePin(pins[0].first, pins[0].second, GPIO_PIN_RESET); //s3
	HAL_GPIO_WritePin(pins[1].first, pins[1].second, GPIO_PIN_RESET); //s2
	int redPeriod = calcPeriod(pins[2].first, pins[2].second);
	int redFreq = 1.0 / (2 * redPeriod * timerPeriod);

	//set output to green channel
	HAL_GPIO_WritePin(pins[0].first, pins[0].second, GPIO_PIN_SET); //s3
	HAL_GPIO_WritePin(pins[1].first, pins[1].second, GPIO_PIN_SET); //s2
	int greenPeriod = calcPeriod(pins[2].first, pins[2].second);
	int greenFreq = 1.0 / (2 * greenPeriod * timerPeriod);

	//set output to blue channel
	HAL_GPIO_WritePin(pins[0].first, pins[0].second, GPIO_PIN_SET); //s3
	HAL_GPIO_WritePin(pins[1].first, pins[1].second, GPIO_PIN_RESET); //s2
	int bluePeriod = calcPeriod(pins[2].first, pins[2].second);
	int blueFreq = 1.0 / (2 * bluePeriod * timerPeriod);

	return normalizeToRGB(redFreq, greenFreq, blueFreq);
}

//returns error is deg
double calcError(Color left, Color right){
	Color red(255, 75, 107);
	// if left more similar to red, drifting to right, therefore displacement is negative, so error is positive;
	int sign = left.cosineSimilarity(red) > right.cosineSimilarity(red) ? 1 : -1;
	return sign * (acos(left.cosineSimilarity(right))*180/M_PI);
}

double averageError(std::deque<int> const& errors, int iters=0){
	if(iters==0){
		iters = errors.size();
	}
	int sum = 0;
	int i = 0;
	for(auto it = errors.begin(); i < iters; i++, it++){
		sum += *it;
	}
	return (double)sum/iters;
}

// each vector<double> is a row in the transform
// assumes 3x3 transform matrix
Color transformColorSpace(std::vector<std::vector<double>> transform, Color c){
	std::vector c_vector = {c.r, c.g, c.b};
	int r_new = std::inner_product(c_vector.begin(), c_vector.end(), transform[0].begin(), 0);
	int g_new = std::inner_product(c_vector.begin(), c_vector.end(), transform[1].begin(), 0);
	int b_new = std::inner_product(c_vector.begin(), c_vector.end(), transform[2].begin(), 0);
	return Color(r_new, g_new, b_new);
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
	//SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_TIM11_Init();
	MX_TIM10_Init();
	MX_TIM3_Init();
	Tim2_Ch1_Init(); //32-bit resolution
	Tim4_Ch1_Init();

	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */


	//Right color sensor -------------------------------------------------------------------------------

	//PC9
	GPIO_InitTypeDef freq_r; // color sensor frequency output
	freq_r.Pin = GPIO_PIN_9;
	freq_r.Mode = GPIO_MODE_INPUT;
	freq_r.Pull = GPIO_NOPULL;
	freq_r.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOC, &freq_r);

	// PC8
	GPIO_InitTypeDef s2_r;
	s2_r.Pin = GPIO_PIN_8;
	s2_r.Mode = GPIO_MODE_OUTPUT_PP;
	s2_r.Pull = GPIO_NOPULL;
	s2_r.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOC, &s2_r);

	// PA15
	GPIO_InitTypeDef s3_r;
	s3_r.Pin = GPIO_PIN_15;
	s3_r.Mode = GPIO_MODE_OUTPUT_PP;
	s3_r.Pull = GPIO_NOPULL;
	s3_r.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOA, &s3_r);

	//by default, s2 and s3 are zero, so we get freq of red as output
	// PB7
	GPIO_InitTypeDef s1_r;
	s1_r.Pin = GPIO_PIN_7;
	s1_r.Mode = GPIO_MODE_OUTPUT_PP;
	s1_r.Pull = GPIO_NOPULL;
	s1_r.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOB, &s1_r);

	// PC6
	GPIO_InitTypeDef s0_r;
	s0_r.Pin = GPIO_PIN_6;
	s0_r.Mode = GPIO_MODE_OUTPUT_PP;
	s0_r.Pull = GPIO_NOPULL;
	s0_r.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOC, &s0_r);

	//set output frequency scaling to 2% (s0 = 0, s1 = 1)
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); //s1
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET); //s0

	//right color sensor end ---------------------------------------------------------------------------------
	//Left color sensor ---------------------------------------------------------------------------------------
	//PC10
	GPIO_InitTypeDef freq_l; // color sensor frequency output
	freq_l.Pin = GPIO_PIN_10;
	freq_l.Mode = GPIO_MODE_INPUT;
	freq_l.Pull = GPIO_NOPULL;
	freq_l.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOC, &freq_l);

	// PB12
	GPIO_InitTypeDef s2_l;
	s2_l.Pin = GPIO_PIN_12;
	s2_l.Mode = GPIO_MODE_OUTPUT_PP;
	s2_l.Pull = GPIO_NOPULL;
	s2_l.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOB, &s2_l);

	// PC12
	GPIO_InitTypeDef s3_l;
	s3_l.Pin = GPIO_PIN_12;
	s3_l.Mode = GPIO_MODE_OUTPUT_PP;
	s3_l.Pull = GPIO_NOPULL;
	s3_l.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOC, &s3_l);

	//by default, s2 and s3 are zero, so we get freq of red as output
	// PC11
	GPIO_InitTypeDef s1_l;
	s1_l.Pin = GPIO_PIN_11;
	s1_l.Mode = GPIO_MODE_OUTPUT_PP;
	s1_l.Pull = GPIO_NOPULL;
	s1_l.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOC, &s1_l);

	// PC2
	GPIO_InitTypeDef s0_l;
	s0_l.Pin = GPIO_PIN_2;
	s0_l.Mode = GPIO_MODE_OUTPUT_PP;
	s0_l.Pull = GPIO_NOPULL;
	s0_l.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOC, &s0_l);

	//set output frequency scaling to 2% (s0 = 0, s1 = 1)
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET); //s1
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET); //s0
	//left color sensor end -------------------------------------------------------------------------------

	// MIDDLE COLOR SENSOR---------------------------------------------------------------------------------
	// PB4
	GPIO_InitTypeDef freq; // color sensor frequency output
	freq.Pin = GPIO_PIN_4;
	freq.Mode = GPIO_MODE_INPUT;
	freq.Pull = GPIO_NOPULL;
	freq.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOB, &freq);

	// PB3
	GPIO_InitTypeDef s2;
	s2.Pin = GPIO_PIN_3;
	s2.Mode = GPIO_MODE_OUTPUT_PP;
	s2.Pull = GPIO_NOPULL;
	s2.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOB, &s2);

	// PB5
	GPIO_InitTypeDef s3;
	s3.Pin = GPIO_PIN_5;
	s3.Mode = GPIO_MODE_OUTPUT_PP;
	s3.Pull = GPIO_NOPULL;
	s3.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOB, &s3);

	//by default, s2 and s3 are zero, so we get freq of red as output
	// PC1
	GPIO_InitTypeDef s1;
	s1.Pin = GPIO_PIN_1;
	s1.Mode = GPIO_MODE_OUTPUT_PP;
	s1.Pull = GPIO_NOPULL;
	s1.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOC, &s1);

	// PB0
	GPIO_InitTypeDef s0;
	s0.Pin = GPIO_PIN_0;
	s0.Mode = GPIO_MODE_OUTPUT_PP;
	s0.Pull = GPIO_NOPULL;
	s0.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOB, &s0);

	//set output frequency scaling to 2% (s0 = 0, s1 = 1)
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET); //s1
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); //s0

	//middle color sensor end -----------------------------------------------------------------------------

	// for both timers:
	// 0.00001sec per tick (count) bc hclock is 84MHz - we use prescale factor of 840 (set below), so tim11 is 84MHz/840 = 100KHz -> period of tim11 = 1/100000Hz = 0.00001 sec -> therefore each count in 0.00001 sec

	const uint32_t CLOCK_SPEED = 84000000; // in Hz

	int BASE_MOTOR_SPEED = 50;
	const uint32_t MAX_TIME = 65535;

	const double TIMER10_PERIOD = 0.00001;
	// double TIMER10_PERIOD = 1.0/((double)CLOCK_SPEED/htim10.Init.Prescaler);
	const double TIMER11_PERIOD = 0.00001;
	// double TIMER11_PERIOD = 1.0/((double)CLOCK_SPEED/htim11.Init.Prescaler);
	const double TIMER3_PERIOD = 0.00001;

	//works:
	//speed: 55, kp: 3.5, ki: 0, kd: 0.01
	//speed: 60, kp = 4.7, kd = 0.032
	//60, 3.5, 0.014
	//60, 4, 0, 0.014 -- does whole course!!!!
	//100 3.3, 0.027 - down straights
	double Kp = 2;//3.75
	double Ki = 0;//100;
	double Kd = 0.012;//0.036;//10;

	double cumulativeError = 0;

	double error_n_1 = 0; // error from 1 iteration ago
	double error_n_2 = 0; // error from 2 iterations ago

	double derivative_n_1 = 0;
	double derivative_n_2 = 0;

	//Motor_A_Forward(60);
	//Motor_B_Forward(60);

	HAL_TIM_Base_Start(&htim10);

	int prevCount_tim10 = __HAL_TIM_GET_COUNTER(&htim10);
	int prevCount_tim3 = __HAL_TIM_GET_COUNTER(&htim3);

	//HAL_Delay(3000);

	//
	//Motor_A_Forward(100);
	//Motor_B_Forward(100);
	//

	std::deque<int> prevErrors(3, 0);

	HAL_TIM_Base_Start(&htim3);
	int c0 = __HAL_TIM_GET_COUNTER(&htim3);

	Encoder_A_Init();
	Encoder_B_Init();

	int dl_n_1 = 0;
	int dl_n_2 = 0;

	int dr_n_1 = 0;
	int dr_n_2 = 0;

	int kdCounter = 0;
	bool counting = false;

	while (1) {

		// pin order: s3, s2, f
		//Color middleColor = extractColorFromSensor({{GPIOB, GPIO_PIN_5},{GPIOB, GPIO_PIN_3}, {GPIOB, GPIO_PIN_4}}, TIMER11_PERIOD);
		Color leftColor = extractColorFromSensor({{GPIOC, GPIO_PIN_12},{GPIOB, GPIO_PIN_12}, {GPIOC, GPIO_PIN_10}}, TIMER11_PERIOD);
		Color rightColor = extractColorFromSensor({{GPIOA, GPIO_PIN_15},{GPIOC, GPIO_PIN_8}, {GPIOC, GPIO_PIN_9}}, TIMER11_PERIOD);


		Color red(255, 75, 107);
		Color green(140, 228, 187);
		Color blue(59, 79, 180);
		//int colorCosineForPrint = acos(leftColor.cosineSimilarity(blue))*180/M_PI;
		std::string s = "L(" + std::to_string(leftColor.r) + ", " + std::to_string(leftColor.g) + ", " + std::to_string(leftColor.b) + ")" + "R(" + std::to_string(rightColor.r) + ", " + std::to_string(rightColor.g) + ", " + std::to_string(rightColor.b) + "), "; ////"(" + std::to_string(colorCosineForPrint) + ")";
		const char* c = s.c_str();
		USART_Transmit(&huart2, (uint8_t*)c);

		// positive displacement is to left
		// error is yd-y -> yd=0, so error = -y

		//double error = -colorToDisplacement(leftColor, middleColor, rightColor);
		double error = calcError(leftColor, rightColor);
		prevErrors.push_front(error);
		prevErrors.pop_back();
		double averagedError = averageError(prevErrors);
		//double averagedErrorForD = averageError(prevErrors);
		int currCount_tim10 = __HAL_TIM_GET_COUNTER(&htim10);
		double timeStep = calcTotalCount(currCount_tim10, prevCount_tim10, MAX_TIME) * TIMER10_PERIOD;
		prevCount_tim10 = currCount_tim10;
		double errorDerivative = differentiate(averagedError, error_n_1, error_n_2, timeStep);
		error_n_2 = error_n_1;
		error_n_1 = averagedError;
		cumulativeError += averagedError * timeStep;

		/*
		double secondDerivative = differentiate(errorDerivative, derivative_n_1, derivative_n_2, timeStep);

		if((errorDerivative < 300 && secondDerivative > 100000) || counting){
			Kd=0;
			if(!counting){
				std::string mssg = "in!   ";
				const char* c = mssg.c_str();
				USART_Transmit(&huart2, (uint8_t*)c);
				kdCounter = 50;
				counting = true;
			}else if(kdCounter==0){
				counting = false;
			}else{
				kdCounter--;
			}
		}
		*/
		int increment = Kp*averagedError + Ki*cumulativeError + Kd*errorDerivative;

		//A=right
		//B=left

		int errorForPrint = averagedError;
		//int integralForPrint = cumulativeError * 10000;
		int derivativeForPrint = errorDerivative;
		//int sdfp = secondDerivative;

		//std::string s = "(" + std::to_string(sdfp) + ")";//;", " + std::to_string(integralForPrint) + ")";//", " + std::to_string(derivativeForPrint) + ")";
		//const char* c = s.c_str();
		//USART_Transmit(&huart2, (uint8_t*)c);

		if(increment < 0){
			Motor_A_Forward(std::max(0, BASE_MOTOR_SPEED-abs(increment)));
			Motor_B_Forward(BASE_MOTOR_SPEED);

		}else{
			Motor_A_Forward(BASE_MOTOR_SPEED);
			Motor_B_Forward(std::max(0, BASE_MOTOR_SPEED-abs(increment)));
		}
		//std::string s = "(" + std::to_string(std::max(0, BASE_MOTOR_SPEED-abs(increment))) + ")";//;", " + std::to_string(integralForPrint) + ")";//", " + std::to_string(derivativeForPrint) + ")";
		//const char* c = s.c_str();
		//USART_Transmit(&huart2, (uint8_t*)c);

		// if drift to left, y displacement is positive, and so will derivative and integral, so increment will be negative

		int currCount_tim3 = __HAL_TIM_GET_COUNTER(&htim3);
		double timeStep_tim3 = calcTotalCount(currCount_tim3, prevCount_tim3, 65535)*0.0001;
		prevCount_tim3 = currCount_tim3;

		//int intTime = time;

		//int errorForPrint = averagedError;
		//int integralForPrint = cumulativeError * 10000;
		//int derivativeForPrint = errorDerivative * 10000;

		//std::string s = "(" + std::to_string(intTime) + ")";//;", " + std::to_string(integralForPrint) + ")";//", " + std::to_string(derivativeForPrint) + ")";
		//const char* c = s.c_str();
		//USART_Transmit(&huart2, (uint8_t*)c);

		uint16_t wheelDiam_mm = 65;
		int dr = Motor_A_Dist_mm(wheelDiam_mm);
		int dl = Motor_B_Dist_mm(wheelDiam_mm);
		double vr = differentiate(dr, dr_n_1, dr_n_2, timeStep_tim3);
		double vl = differentiate(dl, dl_n_1, dl_n_2, timeStep_tim3);
		int velPrint = abs(vl-vr);

		int vel = vl;
		//std::string s2 = "(" + std::to_string(velPrint);//;", " + std::to_string(integralForPrint) + ")";//", " + std::to_string(derivativeForPrint) + ")";
		//const char* c2 = s2.c_str();
		//USART_Transmit(&huart2, (uint8_t*)c2);

		//int vel2 = vr;
		//std::string s2 = "," + std::to_string(vel2) + ")";//;", " + std::to_string(integralForPrint) + ")";//", " + std::to_string(derivativeForPrint) + ")";
		//const char* c2 = s2.c_str();
		//USART_Transmit(&huart2, (uint8_t*)c2);

		dr_n_2 = dr_n_1;
		dr_n_1 = dr;
		dl_n_2 = dl_n_1;
		dl_n_1 = dl;

		if(fabs(vr-vl) > 150){
			std::string s2 = "slowing!!";//;", " + std::to_string(integralForPrint) + ")";//", " + std::to_string(derivativeForPrint) + ")";
			const char* c2 = s2.c_str();
			USART_Transmit(&huart2, (uint8_t*)c2);
			BASE_MOTOR_SPEED = 46;
			Kp = 2.5;
			//Ki = 0;//100;
			//Kd = 0.014;//10;
		}
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

static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1600-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 840-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 840-1;
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
