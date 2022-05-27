/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define T 60000
#define COUNTER 1000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

void togglepin(uint8_t value);
void delay(int delay);

void dec_to_bin(char ch) {
	for (char div = 128; div > 0; div /= 2) {
		if (ch >= div) {    // 1
			ch -= div;
			togglepin(1);
		} else
			togglepin(0);
	}
	delay(T);
}

// This function provides delay (in 20 nanoseconds)
void delay(int delay) {
	if (delay > 0xFFFF)
		delay = 0xFFFF;
	(&htim2)->Instance->CNT = 0;
	while ((&htim2)->Instance->CNT < delay)
		;
}

void togglepin(uint8_t value) {
	//	turning off pin
	TIM1->CCR2 = 0;
	TIM1->CNT = COUNTER;
	if (value == 1) {
		delay(T * 0.8);
		//	turning on pin
		TIM1->CCR2 = COUNTER + 1;
		TIM1->CNT = COUNTER;
		delay(T * 0.2);
//		char c = '1';
//		HAL_UART_Transmit(&huart2, &c, 1, -1);
	} else if (value == 0) {
		delay(T * 0.2);
		//	turning on pin
		TIM1->CCR2 = COUNTER + 1;
		TIM1->CNT = COUNTER;
		delay(T * 0.8);
//		char c = '0';
//		HAL_UART_Transmit(&huart2, &c, 1, -1);
	} else
		return;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void output(char* string) {
	char* buf = string;
	HAL_UART_Transmit(&huart2, buf, strlen(buf), 100);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
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
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */

	// channel state 0V
	TIM1->ARR = COUNTER;
	TIM1->CCR2 = 0;

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

	HAL_TIM_Base_Start(&htim2);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	char buf[] = "Hello world!\rBetter late than never\r"
			"Hang in there\rMake a long story short\r"
			"Good things come to those who wait\r"
			"\r\nThe machines rose from the ashes of the nuclear fire.\r"
			"Their war to exterminate mankind has raged for decades,\r"
			"but the final battle would not be fought in the future.\r"
			"It would be fought here, in our present. Tonight.\r"
			"\rWhen you walk through a storm\r"
			"Hold your head up high\r"
			"And don't be afraid of the dark\r"
			"At the end of a storm\r"
			"There's a golden sky\r"
			"And the sweet silver song of a lark\r"
			"Walk on through the wind\r"
			"Walk on through the rain\r"
			"Though your dreams be tossed and blown\r"
			"Walk on, walk on\r"
			"With hope in your heart\r"
			"And you'll never walk alone\r"
			"You'll never walk alone\r"
			"Walk on, walk on\r"
			"With hope in your heart\r"
			"And you'll never walk alone\r"
			"You'll never walk alone\r"
			"\r\n"
			"Didn't know what time it was, the lights were low\r"
			"I leaned back on my radio\r"
			"Some cat was layin down some rock 'n' roll\r"
			"Lotta soul, he said\r"
			"Then the loud sound did seem to fade\r"
			"Came back like a slow voice on a wave of phase\r"
			"That weren't no DJ, that was hazy cosmic jive\r"
			"\r\rThere's a starman waiting in the sky\r"
			"He'd like to come and meet us\r"
			"But he thinks he'd blow our minds\r"
			"There's a starman waiting in the sky\r"
			"He's told us not to blow it\r"
			"'Cause he knows it's all worthwhile\r"
			"He told me\r"
			"Let the children lose it\r"
			"Let the children use it\r"
			"Let all the children boogie\r"
			"\r\nI had to phone someone so I picked on you\r"
			"Hey, that's far out, so you heard him too\r"
			"Switch on the TV, we may pick him up on channel two\r"
			"Look out your window, I can see his light\r"
			"If we can sparkle he may land tonight\r"
			"Don't tell your poppa or he'll get us locked up in fright\r\r\r"
			"There's a starman waiting in the sky\r"
			"He'd like to come and meet us\rBu"
			"t he thinks he'd blow our minds\r"
			"There's a starman waiting in the sky\r"
			"He's told us not to blow it\r"
			"'Cause he knows it's all worthwhile\r"
			"He told me\rLet the children lose it\r"
			"Let the children use it\r"
			"Let all the children boogie\r\r\r"
			"Starman waiting in the sky\r"
			"He'd like to come and meet us\r"
			"But he thinks he'd blow our minds\r"
			"There's a starman waiting in the sky\r"
			"He's told us not to blow it\r"
			"'Cause he knows it's all worthwhile\r"
			"He told me\r"
			"Let the children lose it\r"
			"Let the children use it\r"
			"Let all the children boogie\r";

	//  Hang in there\rMake a long story short\r
	char out[] = "still work!!\r\n";
	HAL_UART_Transmit(&huart2, out, strlen(out), -1);
	output("Hello world!\rBetter late than never\r\n"
			"Hang in there\rMake a long story short\r\n"
			"Good things come to those who wait\r\n"
			"\r\nThe machines rose from the ashes of the nuclear fire.\r\n"
			"Their war to exterminate mankind has raged for decades,\r\n"
			"but the final battle would not be fought in the future.\n"
			"It would be fought here, in our present. Tonight.\n"
			"\rWhen you walk through a storm\r\n");
	char c = 'l';
	while (1) {

		// turning on channel
		delay(10 * T);
		// channel state 0V -> 3.3V
		TIM1->CCR2 = COUNTER + 1;
		TIM1->CNT = COUNTER;
		HAL_Delay(2000);

//		HAL_UART_Transmit(&huart2, &buf, strlen(buf), -1);
		// information transmission
		for (int i = 0; i < strlen(buf); i++) {
			dec_to_bin(buf[i]);
		}
//		dec_to_bin(c);
		// end of data transmission channel state still 3.3V
		HAL_Delay(2000);

		// turning off channel 3.3V -> 0V
		TIM1->CCR2 = 0;
		TIM1->CNT = COUNTER;
//		HAL_Delay(2000);
//		char message[] = "End of transmission\r";
//		HAL_UART_Transmit(&huart2, message, strlen(message), -1);

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 1 - 1;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 1 - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 4294967295;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

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
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();

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
     ex: printf("Wrong parameters value: file %s on line %d\r", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

