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
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define T 5000
#define COUNTER 5000
#define SYN 22
#define ACK 6

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

void output(char *string) {
	char *buf = string;
	HAL_UART_Transmit(&huart2, (uint8_t*) buf, strlen(buf), 100);
}

/*
 * <------------- TRANSIVER CODE ----------------->
 */

void togglepin(uint8_t value);
void delay(int delay);

void dec_to_bin(char ch) {
	for (char div = 128; div > 0; div /= 2) {
		if (ch >= div) {    // 1
			ch -= div;
			togglepin(1);
//			output("1");
		} else {
			togglepin(0);
//			output("0");
		}
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
	} else if (value == 0) {
		delay(T * 0.2);
		//	turning on pin
		TIM1->CCR2 = COUNTER + 1;
		TIM1->CNT = COUNTER;
		delay(T * 0.8);
	} else
		return;
}

/*
 * <-------------------- END TRANSIVER CODE ----------->
 */

/*
 * <-------------------- RECIVER CODE ----------------->
 */

int byte[2] = { 0 };
int byte_count = 0;
// use in CaptureCallback, PeriodElepsed and other functions
int callback_count = 0;
int bit_count = 0;
int *bit;

/*
 * SYN -->
 * SYN, ACK <--
 * ACK -->
 */
int connect(void) {

	int connection = 0;
	dec_to_bin(SYN);
	output("First transmission\r\n");
	delay(10 * T);

	// Turning off transmitter timers
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);

	// Turning on receive timers
	TIM3->ARR = T;
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim3);

	delay(10 * T);


	// wait time for 2 byte -> 20 T
	// to take SYN and ACK from receiver
	for (int i = 0; byte_count < 2 && i < 20; i++) {
		delay(T);
	}


//	char buf[128] = { 0 };
//	sprintf(buf, "%d - byte\r\n", byte_count);
//	HAL_UART_Transmit(&huart2, buf, strlen(buf), -1);

	// Turning off reciver timers
	HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_2);
	HAL_TIM_Base_Stop_IT(&htim3);
	delay(10 * T);

	// Turning on transive timers
	TIM1->CCR2 = COUNTER + 1;
	TIM1->CNT = COUNTER;
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	delay(10 * T);

	if (byte[0] == SYN && byte[1] == ACK) {
		byte_count = 0;
		byte[0] = byte[1] = 0;
		connection = 1;
		output("Device connected\r\n");
		dec_to_bin(ACK);
	} else {
		byte_count = 0;
		byte[0] = byte[1] = 0;
		output("Connection was failed\r\n");
		connection = 0;
	}
	delay(10 * T);
	return connection;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim3) {
		int falling_edge = 0;
		int rising_edge = 0;
//		char buf_1[128] = { 0 };
//		output("from callback");

		callback_count += 1;

		falling_edge = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);
		rising_edge = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2);
//		sprintf(buf_1, "r - %d f - %d c - %d\r\n", rising_edge, falling_edge, callback_count);
//		HAL_UART_Transmit(&huart2, &buf_1, strlen(buf_1), -1);
		if (callback_count % 2 != 0 && callback_count != 1) {
			int period = rising_edge + falling_edge;
			if (period < T * 1.2 && period > T * 0.8) { // 10% gap for period
//				char buf_2[128] = { 0 };
//				sprintf(buf_2, "r - %d\r\n", rising_edge);
//				HAL_UART_Transmit(&huart2, &buf_2, strlen(buf_2), -1);
				bit[bit_count] = rising_edge;
			}
			bit_count += 1;
		}
		TIM3->CNT = 0;
		TIM2->CNT = 0;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim3) { // reading last bite of byte
//		output("In elapse\r\n");
		if (bit_count == 7 && callback_count > 1) {
			char buf[64] = { 0 };
//			output("In elapse\r\n");

//			sprintf(buf, "r - %d\r\n", HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2));
//			HAL_UART_Transmit(&huart2, &buf, strlen(buf), -1);

			bit[bit_count] = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2);
			display_pwm_information(bit);

			bit_count = 0;
			callback_count = 0;
//			connect = 1;
		} else if (callback_count > 1 && bit_count < 7) { // when caught less then 8 bit before IDEL give message about lost bytes
			output("#LOST BYTE#");
			if (callback_count % 2 != 0) {
				output("_1_\r\n"); // start_finish_connection
				bit_count = 0;
				callback_count = 0;
//				start_finish_connection();
			} else {
				bit_count = 0;
				callback_count = 0;
			}
		} else if (callback_count == 1) { // turning on/off transiver state of channel change 0V -> 3.3V / 3.3V -> 0V
			output("_2_\r\n"); // start_finish_connection
			bit_count = 0;
			callback_count = 0;
//			start_finish_connection();
		} else
			// TIM run function every period ending
			return;
	}
}

char rises_to_char(int*);

/* function display information from pwm signal to user with UART
 *
 * receives int array contain 8 rising times
 * convert rising times to char and display it
 *
 */

void display_pwm_information(int *bit) {
	byte[byte_count] = rises_to_char(bit);
	byte_count += 1;
//	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, -1);
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

/*
 * <-------------------- END RECIVER CODE ----------->
 */

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
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_USART2_UART_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */

	// channel state 0V
	TIM1->ARR = COUNTER;
	TIM1->CCR2 = 0;

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

	HAL_TIM_Base_Start(&htim2);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
//	char buf[] = "Hello\r\n";
	char buf[] = "Hello world!\r\nBetter late than never\r\n"
			"Hang in there\r\nMake a long story short\r\n"
			"Good things come to those who wait\r\n"
			"\r\n\nThe machines rose from the ashes of the nuclear fire.\r\n"
			"Their war to exterminate mankind has raged for decades,\r\n"
			"but the final battle would not be fought in the future.\r\n"
			"It would be fought here, in our present. Tonight.\r\n";
	//  Hang in there\r\nMake a long story short\r\n
//	char enter[] = "\r\n\n";
//	char ch = 'l';

	output("I'm still alive\r\n\n");

	bit = calloc(8, sizeof(int));
	if (bit == NULL) {
		char exit[] = "Memory end\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t*) exit, strlen(exit), -1);
		return 1;
	}

	HAL_Delay(1000);



	int connection = 0;

	while (1) {

//		 turning on channel
		// channel state 0V -> 3.3V
		TIM1->CCR2 = COUNTER + 1;
		TIM1->CNT = COUNTER;
		HAL_Delay(1000);
		while (connection != 1) {
			connection = connect();
			HAL_Delay(100);
		}

		output("Start transmission\r\n");
		HAL_Delay(1000);
//		HAL_UART_Transmit(&huart2, &buf, strlen(buf), -1);
		// information transmission
		for (int i = 0; i < strlen(buf); i++) {
			dec_to_bin(buf[i]);
		}
		// end of data transmission channel state still 3.3V
		HAL_Delay(1000);

		// turning off channel 3.3V -> 0V
		TIM1->CCR2 = 0;
		TIM1->CNT = COUNTER;
		connection = 0;
		HAL_Delay(1000);

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
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
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

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 12 - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65535;
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
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_IC_InitTypeDef sConfigIC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 12 - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_IC_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
	if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

