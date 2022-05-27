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
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define T 5000
#define COUNTER 5000
#define SYN 22
#define ACK 6

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/*
 * <------------------ RECEIVER CODE ------------------------->
 */

void output(char *string) {
	char *buf = string;
	HAL_UART_Transmit(&huart2, buf, strlen(buf), 100);
}

/* ---------------to do list---------------
 * write else in capture callback
 * check working of algorithm on high speed ?????
 * check all kind of signals
 * 		- Long IDEL 3.3V +++
 * 		- Long IDEL 0V +++++
 * 		- rising from 0V without information (turning PWM transiver on) ++++
 * 		- falling down to zero (turning off transiver) ++++
 *		- etc
 * make code better &??
 * create byte LED
 * ---------------------------------------
 */

// use in CaptureCallback, PeriodElepsed and other functions
int callback_count = 0;
int bit_count = 0;
int *bit;
int connection = 0;

/* main function of device
 *
 * catch changing state of PWM channel
 * check period of signal to avoid interferences (should write else)
 * write rising times in bit array
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim2) {
		int falling_edge = 0;
		int rising_edge = 0;
//		output("From callback\r\n");
//		char buf[128] = { 0 };

		callback_count += 1;

		falling_edge = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
		rising_edge = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
//		sprintf(buf, "r - %d f - %d c - %d\r\n", rising_edge, falling_edge, callback_count);
//		HAL_UART_Transmit(&huart2, &buf, strlen(buf), -1);
		if (callback_count % 2 != 0 && callback_count != 1) {
			int period = rising_edge + falling_edge;
			if (period < T * 1.05 && period > T * 0.95) { // 10% gap for period
//				sprintf(buf, "r - %d\r\n", rising_edge);
//				HAL_UART_Transmit(&huart2, &buf, strlen(buf), -1);
				bit[bit_count] = rising_edge;
			}
			bit_count += 1;
		}

		TIM2->CNT = 0;
		TIM16->CNT = 0;

	}
}

void display_pwm_information(int *bit);
void start_finish_connection(void);

/*
 * function catch IDEL signal
 * IDEL signal then there is one period without capture callback
 * Use for reading last bit in signal
 *
 * AND display letter to user by display_pwm_information()
 *
 * 3 types of IDEL
 *
 * IDEL after catching 8 bit
 * function display letter
 *
 * IDEL after less 8 bit
 * function display error message
 *
 * IDEL without catching any bit
 * Timer run function every period with or without information
 * function do nothing in this case
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim2) { // reading last bite of byte
		if (bit_count == 7 && callback_count > 1) {

//			sprintf(buf, "r - %d\r\n", HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2));
//			HAL_UART_Transmit(&huart2, &buf, strlen(buf), -1);

			bit[bit_count] = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
			display_pwm_information(bit);

			bit_count = 0;
			callback_count = 0;
//			connection = 1;
		} else if (callback_count > 1 && bit_count < 7) { // when caught less then 8 bit before IDEL give message about lost byte
			output("#LOST BYTE#");
			if (callback_count % 2 != 0) {
//				start_finish_connection();
				bit_count = 0;
				callback_count = 0;
				connection = 0;
				output(" # Disconnect");
			} else {
//				connection = 1;
				bit_count = 0;
				callback_count = 0;
			}
		} else if (callback_count == 1) { // turning on/off transiver state of channel change 0V -> 3.3V / 3.3V -> 0V
//			start_finish_connection();
			if (connection == 1) {
				output(" # Disconnection\r\n");
				connection = 0;
			}
			bit_count = 0;
			callback_count = 0;
		} else
			// TIM run function every period ending
			return;
//		char buf[64] = { 0 };
//		sprintf(buf, "End of byte\r\n\r\n");
//		HAL_UART_Transmit(&huart2, &buf, strlen(buf), -1);
	}
}

/* function check on/off channel state and show user information
 * about connection or disconnection transiver
 */

//void start_finish_connection(void) {
//	bit_count = 0;
//	callback_count = 0;
//	if (connection == 0) { // transiver connected
//		char message[] = "\r\nConnected. Ready for recive.\r\n";
//		HAL_UART_Transmit(&huart2, &message, strlen(message), -1);
//		connection = 1;
//	} else { // transiver disconnected
//		char message[] = "\r\nDisconected. Bye.\r\n";
//		HAL_UART_Transmit(&huart2, &message, strlen(message), -1);
//		connection = 0;
//	}
//}

char rises_to_char(int*);

/* function display information from pwm signal to user with UART
 *
 * receives int array contain 8 rising times
 * convert rising times to char and display it
 *
 */
void display_pwm_information(int *bit) {
//	output("kek\r\n");
	char ch = rises_to_char(bit);
//	char buf[32] = { 0 };
//	sprintf(buf, " %d\r\n", ch);
//	HAL_UART_Transmit(&huart2, buf, strlen(buf), -1);
	if (ch == SYN) {
		output(" - Start connection...\r\n");
		connect();
		return;
	} else if (ch == ACK && connection == -1) {
		output(" - Device connected successfully\r\n");
		return;
		connection = 1;
	}
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, -1);
}

/* function return char from 8 rising times array
 * bit - pointer to 8 rising times array contain 8 bits
 * If rising time more then 50% period then this is a 1, else 0
 */
char rises_to_char(int *bit) {
	char c = 0;
	for (int i = 0, rank = 128; i < 8 && rank > 0; i++, rank /= 2) {
		if (bit[i] > T * 0.5)
			c += rank;
	}
	return c;
}

//<------------------- END RECEIVER CODE ------------------->

//<------------------- TRANCIVER CODE ---------------------->

void togglepin(uint8_t value);
void delay(int delay);

void connect(void) {

//	turning off receiver timers
	HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_2);
	delay(10 * T);
//	turning on tranceiver timers
	TIM1->ARR = COUNTER;
	TIM1->CCR2 = COUNTER + 1;
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

	delay(10 * T);
	dec_to_bin(SYN);
	dec_to_bin(ACK);
	delay(10 * T);
//	turning off tranceiver timers
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
//	turning on receiver timers
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);

	delay(10 * T);
	output(" - Wait\r\n");
	connection = -1;
}

void dec_to_bin(char ch) {
	for (char div = 128; div > 0; div /= 2) {
		if (ch >= div) {    // 1
			ch -= div;
			togglepin(1);
//			output("1\r\n");
		} else {
			togglepin(0);
//			output("0\r\n");
		}
	}
	delay(T);
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
	} else if (value == 0) {
		delay(T * 0.2);
		//	turning on pin
		TIM1->CCR2 = COUNTER + 1;
		TIM1->CNT = COUNTER;
		delay(T * 0.8);
	} else
		return;
}

//<------------------- END TRANCIVER CODE ------------------>

// function provides delay (in 20 nanoseconds)
void delay(int delay) {
	if (delay > 0xFFFF)
		delay = 0xFFFF;
	(&htim16)->Instance->CNT = 0;
	while ((&htim16)->Instance->CNT < delay)
		;
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
	MX_USART2_UART_Init();
	MX_TIM2_Init();
	MX_TIM16_Init();
	MX_TIM1_Init();
	/* USER CODE BEGIN 2 */
	//	timer for read values
	TIM2->ARR = T;
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);

	// start TIM2 again to catch IDEL after empty period
	HAL_TIM_Base_Start_IT(&htim2);



	// Timer for calculating delays in micro or nano seconds
	HAL_TIM_Base_Start_IT(&htim16);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

////	char input[128] = { 0 };
	output("Still working\r\n");
	bit = calloc(8, sizeof(int));
	if (bit == NULL) {
		char exit[] = "Memory end\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t*) exit, strlen(exit), -1);
		return 1;
	}
	connection = 0;

	while (1) {
//		HAL_UART_Transmit(&huart2, &mesedge, strlen(mesedge), -1);
//		HAL_Delay(1000);

// delay for finding idel after transmission
//		delay(2 * T);
//		// after catching idel write last rising edge to bit array
//		if (TIM16->CNT > 2 * T)
//			bin[7] = rising_edge;
//		ch = bin_to_char(bin);
//		HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, -1);
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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
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
	htim1.Init.Prescaler = 12 - 1;
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
	TIM_IC_InitTypeDef sConfigIC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 12 - 1;
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
	if (HAL_TIM_IC_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
	if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM16 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM16_Init(void) {

	/* USER CODE BEGIN TIM16_Init 0 */

	/* USER CODE END TIM16_Init 0 */

	/* USER CODE BEGIN TIM16_Init 1 */

	/* USER CODE END TIM16_Init 1 */
	htim16.Instance = TIM16;
	htim16.Init.Prescaler = 12 - 1;
	htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim16.Init.Period = 65535;
	htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim16.Init.RepetitionCounter = 0;
	htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim16) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM16_Init 2 */

	/* USER CODE END TIM16_Init 2 */

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
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LED_Pin | POWER_Pin, GPIO_PIN_SET);

	/*Configure GPIO pins : LED_Pin POWER_Pin */
	GPIO_InitStruct.Pin = LED_Pin | POWER_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

