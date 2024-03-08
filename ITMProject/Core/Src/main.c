/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef TxHeader_bms;
// general used for testing purposes
CAN_TxHeaderTypeDef TxHeader_general;
// Rx used for testing
CAN_RxHeaderTypeDef RxHeader;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint8_t TxData_bms[8] = {0};
//  uint8_t TxData_general[8] = {0};
  // Rx Data used for testing
  uint8_t RxData[8] = {0};

  uint32_t raw_ADC_output[23] = {0};
  int16_t temperature[23] = {0};
  uint16_t error_index = 23;

  int16_t current_lowest_temp = HIGHEST_TEMP;
  int16_t current_highest_temp = LOWEST_TEMP;
  int16_t current_average_temp = 0;

  char msgBuffer[100] = {0};
  uint16_t therm_count = 0;

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  uint32_t TxMailbox = CAN_TX_MAILBOX0;

  TxData_bms[0] = MODULE_NUMBER - 1;
  TxData_bms[4] = THERMISTORS_ENABLED;
  TxData_bms[5] = HIGHEST_THERM_ID;
  TxData_bms[6] = LOWEST_THERM_ID;
  TxData_bms[7] = CHECK_SUM;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	// Resetting values every loop
	current_lowest_temp = HIGHEST_TEMP;
	current_highest_temp = LOWEST_TEMP;
	current_average_temp = 0;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
	// LED for testing
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_RESET)
	{
		HAL_ADC_Start_DMA(&hadc1, raw_ADC_output, 12);
		for(int i = 0; i < 12; i++)
		{
			temperature[i] = binary_search(raw_ADC_output[i]);
			if((temperature[i] >= 86 || temperature[i] <= -41) && error_index == 23)
			{
				error_index = i;
				TxData_bms[4] = 0x80 + THERMISTORS_ENABLED;

				/*
				 * error present: possible ways it needs to be handled correctly
				 * TxData_bms:
				 * 3) Number of thermistors enabled should be set to 0x80
				 * 	  if error is present. Byte 4 (zero based)
				 * */
			}
			else if(error_index < 23 && !(temperature[error_index] >= 86 || temperature[error_index] <= -41))
			{
				error_index = 23;
				TxData_bms[4] = THERMISTORS_ENABLED;
			}
			current_average_temp += temperature[i];
			if(temperature[i] < current_lowest_temp)
			{
				current_lowest_temp = temperature[i];
			}
			if(temperature[i] > current_highest_temp)
			{
				current_highest_temp = temperature[i];
				if(current_highest_temp >= 60)
				{
//					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
				}
			}
			// Lines below are for testing
			memset(msgBuffer, '\0', 100);
			sprintf(msgBuffer, "ADC_read_%d: %ld temperature_%d: %d\r\n", i, raw_ADC_output[i], i, temperature[i]);
			HAL_UART_Transmit(&huart2, (uint8_t*)msgBuffer, strlen(msgBuffer), HAL_MAX_DELAY);
		}
		HAL_ADC_Stop_DMA(&hadc1);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	}
	// Check if select pin has been set high
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET)
	{
		HAL_ADC_Start_DMA(&hadc1, &raw_ADC_output[12], 11);
		for(int i = 12; i < 23; i++)
		{
			temperature[i] = binary_search(raw_ADC_output[i]);
			current_average_temp += temperature[i];
			if((temperature[i] >= 86 || temperature[i] <= -41) && error_index == 23)
			{
				error_index = i;
				TxData_bms[4] = 0x80 + THERMISTORS_ENABLED;;

				/*
				 * error present: possible ways it needs to be handled correctly
				 * TxData_bms:
				 * 3) Number of thermistors enabled should be set to 0x80
				 * 	  if error is present. Byte 4 (zero based)
				 * */
			}
			else if(error_index < 23 && !(temperature[error_index] >= 86 || temperature[error_index] <= -41))
			{
				error_index = 23;
				TxData_bms[4] = THERMISTORS_ENABLED;
			}
			if(temperature[i] < current_lowest_temp)
			{
				current_lowest_temp = temperature[i];
			}
			if(temperature[i] > current_highest_temp)
			{
				current_highest_temp = temperature[i];
				if(current_highest_temp >= 60)
				{
//					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
				}
			}
			// Lines below are for testing
			memset(msgBuffer, '\0', 100);
			sprintf(msgBuffer, "ADC_read_%d: %ld temperature_%d: %d\r\n", i, raw_ADC_output[i], i, temperature[i]);
			HAL_UART_Transmit(&huart2, (uint8_t*)msgBuffer, strlen(msgBuffer), HAL_MAX_DELAY);
		}
		HAL_ADC_Stop_DMA(&hadc1);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	}

	current_average_temp = (current_average_temp / 23);

	TxData_bms[1] = current_lowest_temp;
	TxData_bms[2] = current_highest_temp;
	TxData_bms[3] = current_average_temp;
	TxData_bms[7] = CHECK_SUM;

	// Completing checksum
	for (int i = 0; i < 7; i++) {
		TxData_bms[7] += TxData_bms[i];
		memset(msgBuffer, '\0', 100);
		sprintf(msgBuffer, "TxData_bms[7] = TxData_bms[7] + TxData_bms[%d] = %d + %d = %d\r\n",
				i, TxData_bms[7], TxData_bms[i], TxData_bms[7] + TxData_bms[i]);
		HAL_UART_Transmit(&huart2, (uint8_t*)msgBuffer, strlen(msgBuffer), HAL_MAX_DELAY);
	}

	for(int i = 0; i < 8; i++)
	{
		memset(msgBuffer, '\0', 100);
		sprintf(msgBuffer, "TxData_bms[%d] = %d\r\n", i, TxData_bms[i]);
		HAL_UART_Transmit(&huart2, (uint8_t*)msgBuffer, strlen(msgBuffer), HAL_MAX_DELAY);
	}
	/*
	 * The TxData_bms lines below are for testing below with the bms
	 * If want to test with hard values just uncomment
	 * */
//	TxData_bms[0] = MODULE_NUMBER - 1;
//	TxData_bms[1] = 0x23;
//	TxData_bms[2] = 0x3D;
//	TxData_bms[3] = 0x26;
//	TxData_bms[4] = 0x01; // Represents # of therms enabled but if 0x80 error present
//	TxData_bms[5] = 0x01;
//	TxData_bms[6] = 0x00;
//	TxData_bms[7] = CHECK_SUM;
//	for (int i = 0; i < 7; i++) {
//		TxData_bms[7] += TxData_bms[i];
//	}

	therm_count++;
	if(therm_count == 23) {
		therm_count = 0;
	}

	if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0)
	{
		for (int i = 0; i < 2; i++) {
			if (i == 0) {
				if(HAL_CAN_AddTxMessage(&hcan, &TxHeader_bms, TxData_bms, &TxMailbox) != HAL_OK) {
					memset(msgBuffer, '\0', 100);
					strcat(msgBuffer, "Failed to send CAN message\r\n");
					HAL_UART_Transmit(&huart2, (uint8_t*)msgBuffer, strlen(msgBuffer), HAL_MAX_DELAY);
				}
				else {
				  memset(msgBuffer, '\0', 100);
					strcat(msgBuffer, "CAN Message Sent\r\n");
					HAL_UART_Transmit(&huart2, (uint8_t*)msgBuffer, strlen(msgBuffer), HAL_MAX_DELAY);
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
				}
			}
		}
	}

	// Rx recieving for testing purposes
	if(HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0)
	{
		if(HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
		{
//			  Error_Handler();
		}
		else
		{
		  for(int i = 0; i < sizeof(RxData); i++)
		  {
			  memset(msgBuffer, '\0', 100);
			  sprintf(msgBuffer,"Received message RxData[%d] = %d\r\n", i, RxData[i]);
			  HAL_UART_Transmit(&huart2, (uint8_t *)msgBuffer, sizeof(msgBuffer), HAL_MAX_DELAY);
		  }
		}
	}
	else
	{
		memset(msgBuffer, '\0', 100);
		strcat(msgBuffer, "No message received\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t *)msgBuffer, sizeof(msgBuffer), HAL_MAX_DELAY);
	}

	HAL_Delay(17);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 12;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_10;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_11;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_12;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  	CAN_FilterTypeDef canfilterconfig;

    canfilterconfig.FilterActivation = ENABLE;
    canfilterconfig.FilterBank = 0;
    canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    canfilterconfig.FilterIdHigh = 0x0000;
    canfilterconfig.FilterIdLow = 0x0000;
    canfilterconfig.FilterMaskIdHigh = 0x0000;
    canfilterconfig.FilterMaskIdLow = 0x0000;
    canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilterconfig.FilterScale = CAN_FILTERSCALE_16BIT;
    canfilterconfig.SlaveStartFilterBank = 0;

    if(HAL_CAN_ConfigFilter(&hcan, &canfilterconfig) != HAL_OK)
    {
    	Error_Handler();
    }

    if(HAL_CAN_Start(&hcan) != HAL_OK)
    {
  	  Error_Handler();
    }

    TxHeader_bms.DLC = 8; // Data length
    TxHeader_bms.IDE = CAN_ID_EXT; // Specifies a standard identifier for the header
    TxHeader_bms.RTR = CAN_RTR_DATA; // Specifies the type of frame in this case a data frame
    TxHeader_bms.StdId = 0x00; // ID of the sender
    TxHeader_bms.ExtId = BMS_ID;
    TxHeader_bms.TransmitGlobalTime = DISABLE;

    TxHeader_general.DLC = 8; // Data length
    TxHeader_general.IDE = CAN_ID_EXT; // Specifies a standard identifier for the header
    TxHeader_general.RTR = CAN_RTR_DATA; // Specifies the type of frame in this case a data frame
    TxHeader_general.StdId = 0x00; // ID of the sender
    TxHeader_general.ExtId = BMS_ID + 1;
    TxHeader_general.TransmitGlobalTime = DISABLE;

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  char msgBuffer[100] = {0};
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
	  memset(msgBuffer, '\0', 100);
	  strcat(msgBuffer, "In error handler\r\n");
	  HAL_UART_Transmit(&huart2, (uint8_t*)msgBuffer, strlen(msgBuffer), HAL_MAX_DELAY);
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
