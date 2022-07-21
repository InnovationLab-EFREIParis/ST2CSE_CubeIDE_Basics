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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include  <stdio.h>
#include  <errno.h>
#include  <sys/unistd.h>
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

/* USER CODE BEGIN PV */

const float sound_velocity = 0.34; // Sound velocity 0.34 mm/µs

// Fonction nécessaire au fonctionnement de printf()
// ELle définit huart2 comme périphérique d'affichage
int _write(int file, char *data, int len) {
	if ((file != STDOUT_FILENO) && (file != STDERR_FILENO)) {
		errno = EBADF;
		return -1;
	}

	// arbitrary timeout 1000
	HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2, (uint8_t*) data, len,
			1000);

	// return # of bytes written - as best we can tell
	return (status == HAL_OK ? len : 0);
}

int sonar_distance_mm();
void delay_us(uint16_t us);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char uart_rx_buffer[2];
GPIO_PinState PinState;

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */
<<<<<<< HEAD

	int cpt_ms = 0;
	int distance = 0;
	const int velotcite = 340;

=======
	char uart_tx_buff[50];
	int uart_tx_buf_len;
>>>>>>> refs/remotes/origin/main
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
	/* USER CODE BEGIN 2 */
<<<<<<< HEAD
// Light up green led
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
=======
	// Light up green led
	HAL_GPIO_WritePin(GPIOA, LD2_Pin, SET);
>>>>>>> refs/remotes/origin/main
	// blink green led
	for (int i = 0; i < 5; i++) {
		HAL_GPIO_TogglePin(GPIOA, LD2_Pin);
		HAL_Delay(100);
	}

	// Send welcome message on UART
	uart_tx_buf_len = sprintf(uart_tx_buff, "Nucleo L476RG connected\n\r");
	if (HAL_UART_Transmit(&huart2, (uint8_t*) uart_tx_buff, uart_tx_buf_len,
			100) != HAL_OK)
		Error_Handler();
	printf("Function printf also working\r\n");
	printf("Device ID: %lu\r\n", HAL_GetDEVID());
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		// Get Distance from HC_SR04
		printf("Sonar distance %d mm\r\n", sonar_distance_mm());
		HAL_Delay(500);

		// Gpio, push button
		// By default, PUSH_BUTTON is at 1, report to board schematic
		PinState = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);
		if (PinState == 1)
			HAL_GPIO_WritePin(GPIOA, LD2_Pin, RESET);
		else
			HAL_GPIO_WritePin(GPIOA, LD2_Pin, SET);

		// UART (transmit received character)
		if (HAL_UART_Receive(&huart2, uart_rx_buffer, 1, 10) == HAL_OK) {
			HAL_UART_Transmit(&huart2, uart_rx_buffer, 1, 10);
		} else {
			__HAL_UART_CLEAR_OREFLAG(&huart2);
		}

		HAL_GPIO_WritePin(GPIOA, D8_Pin, RESET);
		HAL_GPIO_WritePin(GPIOA, D8_Pin, SET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(GPIOA, D8_Pin, RESET);
		printf("Trig Done\n\r", distance);
		while (!HAL_GPIO_ReadPin(D9_GPIO_Port, D9_Pin)){}
		printf("Echo Received\n\r", distance);
			while(HAL_GPIO_ReadPin(D9_GPIO_Port, D9_Pin) == 1) {
				cpt_ms++;
				HAL_Delay(1);
			}
	    distance = cpt_ms * velotcite/2;
	    printf("Distance = %d \n\r", distance);
	    cpt_ms=0;
	    distance=0;
	    HAL_Delay(1000);



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

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
			!= HAL_OK) {
		Error_Handler();
	}
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 10;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
}

<<<<<<< HEAD
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
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LD2_Pin | D8_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LD2_Pin D8_Pin */
	GPIO_InitStruct.Pin = LD2_Pin | D8_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : D9_Pin */
	GPIO_InitStruct.Pin = D9_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(D9_GPIO_Port, &GPIO_InitStruct);

}

=======
>>>>>>> refs/remotes/origin/main
/* USER CODE BEGIN 4 */
void delay_us(uint16_t us) {
	__HAL_TIM_SET_COUNTER(&htim2, 0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim2) < us)
		;  // wait for the counter to reach the us input in the parameter
}

int sonar_distance_mm() {
	/*
	 * Trig
	 */
	HAL_TIM_Base_Init(&htim2); // Init counter
	HAL_GPIO_WritePin(GPIOA, D8_Trig_Pin, SET); //Set Trig pin
	HAL_TIM_Base_Start(&htim2); //Start counter
	while (__HAL_TIM_GET_COUNTER(&htim2) < 10)
		//Wait 10 tick of counter (10µs)
		;
	HAL_GPIO_WritePin(GPIOA, D8_Trig_Pin, RESET); //Reset Trig pin

	/*
	 * Echo
	 */
	HAL_TIM_Base_Stop(&htim2); //Stop counter
	HAL_TIM_Base_Init(&htim2); //Init counter
	while (!HAL_GPIO_ReadPin(D9_Echo_GPIO_Port, D9_Echo_Pin))
		// Wait rising edge of echo pin
		;
	HAL_TIM_Base_Start(&htim2); // Start counter

	while (HAL_GPIO_ReadPin(D9_Echo_GPIO_Port, D9_Echo_Pin))
		// Wait falling edge of echo pin
		;
	HAL_TIM_Base_Stop(&htim2); //Stop counter
	return __HAL_TIM_GET_COUNTER(&htim2) * sound_velocity / 2; // distance = time*velocity/2
}

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

