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
#include <stdarg.h>
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim1_ch1;
DMA_HandleTypeDef  hdma_tim2_ch3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

struct Buffer{
	int empty;
	int busy;
	int len;
	uint8_t* tab;
};

uint8_t buffer_T_tab[512];
uint8_t buffer_R_tab[512];

struct Buffer buffer_T = {0,0,512,buffer_T_tab};
struct Buffer buffer_R = {0,0,512,buffer_R_tab};

#define codingChar 0x60
#define startFrame 0x7B
#define endFrame 0x7D
#define codeStartcharFromFrame 0x5B
#define codeEndcharFromFrame 0x5D

#define frameMaxSize 265
#define frameMinSize 10
#define frameMin 8
const char device_address[4] = "STM";
char wrong[18];
uint16_t DMA_PWM_In[512];
uint16_t DMA_PWM_Out[512];

volatile uint8_t buf_RX[512];

struct us_sensor_str distance_sensor;
volatile uint16_t IDX_RX_EMPTY;

volatile uint32_t puls[1] = {10};
volatile uint32_t pwmInResult[1];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
int16_t code_Command(char *src, char *dst, uint8_t com_len);
uint8_t CRC_100(char *src, uint8_t len);
uint8_t analizeFrame(char *bufferedFrame, int16_t len, char *sender_add);
int16_t getFrame(char *bufferedFrame);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(TIM1 == htim->Instance)
	{
		uint32_t echo_us = pwmInResult[0];

//		echo_us = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		distance_sensor.distance_cm = hc_sr_04_convert_us_to_cm(echo_us);
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart2, &buf_RX[IDX_RX_EMPTY], 1);
  HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_3, (uint32_t *)puls, 1);
  //HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)pwmInResult, 1);
  hc_sr_04_init(&distance_sensor, &htim1, &htim2, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);  // Enable PWM input capture with interrupts
  HAL_DMA_Start_IT(&hdma_tim1_ch1, (uint32_t)&TIM3->CCR1, (uint32_t)pwmInResult, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  USART_fSend("START\n\r");
  int16_t length = 0;
  char bFrame[300];
  char senderAddress[4];
  while (1)
  {
	  if ((length = getFrame(bFrame)) != -1) {
		 if ((length = analizeFrame(bFrame,length,senderAddress)) != 0) {
			 analizeCommend(bFrame,length,senderAddress);
		 }
	  }
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
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_TIM2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 62500-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
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

/* USER CODE BEGIN 4 */



void GetSurvey()
{
	uint32_t echo_us = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_2);
	distance_sensor.distance_cm = hc_sr_04_convert_us_to_cm(echo_us);
	//HAL_DMA_Start(&hdma_tim1_ch1, &distance_sensor.distance_cm, &dmaDestination, 10);
	//while (HAL_DMA_PollForTransfer(&hdma_tim1_ch1, HAL_DMA_FULL_TRANSFER, 100) != HAL_OK)
	//{
	//	__NOP();
	//}
}

uint8_t USART_keyboardhit(){
	if(buffer_R.empty==buffer_R.busy){
		return 0;
	}
	return 1;
}

int8_t USART_getchar(){
	int16_t znak;
	if(USART_keyboardhit()){
		znak = buffer_R.tab[buffer_R.busy];
		buffer_R.busy++;
		buffer_R.busy %= buffer_R.len;
		return znak;
	}
	else{
		return -1;
	}
}

void USART_fSend(char *msg, ...){
	char tempDataToSend[128];
	int idx;
	va_list arglist;
	va_start(arglist, msg);
	vsprintf(tempDataToSend, msg, arglist);
	va_end(arglist);
	idx = buffer_T.empty;
	for (int i = 0; i < strlen(tempDataToSend); i++){
		buffer_T.tab[idx] = tempDataToSend[i];
		idx++;
		idx %= buffer_T.len;
	}
	__disable_irq();
	//Sprawdzamy czy nie trwa w tym momencie transmisja, jeśli nie to przestawiamy index i ją uruchamiamy
	if((buffer_T.busy == buffer_T.empty) && (__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TXE) == SET)){
		buffer_T.empty = idx;
		HAL_UART_Transmit_IT(&huart2,&buffer_T.tab[buffer_T.busy],1);
		buffer_T.busy++;
		buffer_T.busy %= buffer_T.len;
	}
	else{
		buffer_T.empty = idx;
	}
	__enable_irq();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == &huart2 && buffer_T.busy != buffer_T.empty){
		uint8_t znak = buffer_T.tab[buffer_T.busy];
		buffer_T.busy++;
		buffer_T.busy %= buffer_T.len;
		HAL_UART_Transmit_IT(&huart2,&znak,1);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == &huart2){
			buffer_R.empty++;
			buffer_R.empty %= buffer_R.len;
			HAL_UART_Receive_IT(&huart2,&buffer_R.tab[buffer_R.empty],1);
		}
}

int16_t getFrame(char *bufferedFrame) {
	static int16_t numberOfChars = -1;
	static char frameBuf[frameMaxSize - 2];
	int16_t charFromFrame;
    int16_t ret;
	static uint8_t tildeOccured = 0;
	if ((charFromFrame = USART_getchar()) != -1) {
		if (charFromFrame == startFrame) {
			numberOfChars = 0;
			tildeOccured = 0;
		}
		else if (numberOfChars >= 0) {
			if (tildeOccured == 1) {
				switch (charFromFrame) {
					case codingChar:
						frameBuf[numberOfChars++] = codingChar;
						break;
					case codeStartcharFromFrame:
						frameBuf[numberOfChars++] = startFrame;
						break;
					case codeEndcharFromFrame:
						frameBuf[numberOfChars++] = endFrame;
						break;
					default:
						numberOfChars = -1;
						break;
				}
				tildeOccured = 0;
			}
			else {
				if (charFromFrame == codingChar) {
					tildeOccured = 1;
				}
				else if (charFromFrame == endFrame) {
					ret = numberOfChars;
					numberOfChars = -1;
					if (ret >= (frameMinSize-2)) {
						memcpy(bufferedFrame, frameBuf, ret + 1);
						return ret;
					}
				}
				else {
					frameBuf[numberOfChars] = charFromFrame;
					numberOfChars++;
				}
			}
			if (numberOfChars >= (frameMaxSize - 2)) {
				numberOfChars = -1;
			}
		}
	}
	return -1;
}


uint8_t analizeFrame(char *bufferedFrame, int16_t len, char *sender_add) {
	char stringCRC[3];
	uint8_t intCRC;
	uint16_t commandLength;
	uint8_t i;
	uint8_t crc_temp;

	if (strncmp(bufferedFrame, device_address, 3) == 0) {
		//Pobranie i sprawdzenie nadawcy
		memcpy(sender_add, bufferedFrame + 3, 3);
		//string null-terminated na końcu
		sender_add[3]=0;
		for (i = 0; i < 3; ++i) {
			if (!((sender_add[i] >= 0x41 && sender_add[i] <= 0x5A)
					|| (sender_add[i] >= 0x61 && sender_add[i] <= 0x7A))) {
				return 0;
			}
		}
		//Pobranie i sprawdzenie zakresu znaków (dla crc)
		//string crc - wartość crc zapisana w postaci znakowej np."25"
		memcpy(stringCRC, bufferedFrame + len - 2, 2);
		stringCRC[2] = 0;
		for (i = 0; i < 2; ++i) {
			if (!(stringCRC[i] >= 0x30 && stringCRC[i] <= 0x39)) {
				return 0;
			}
		}
		//Pobranie danych
		//Pobieranie długości danych (długość ramki - stałe elementy)
		commandLength = len - 8;
		memcpy(bufferedFrame, bufferedFrame + 6, commandLength);
		//string null-terminated na końcu
		bufferedFrame[commandLength] = 0;

		//Sprawdzenie konkretnych wartości CRC
		intCRC = atoi(stringCRC);
		if ((crc_temp = CRC_100(bufferedFrame, commandLength)) != intCRC) {
			sprintf(wrong, "WRONG_CRC_%02d;", crc_temp);
			sendFrame(sender_add, wrong, strlen(wrong));
			return 0;
		}
		return 1;
	}
	return 0;
}

void sendFrame(char dst[4], char *com, uint8_t com_len) {
	//com_len dlugość komendy przed kodowaniem
	//510 podano same znaki kodujące w ramce (2 razy większa ilość danych, ramka po zakodowaniu znaków)
	//+1 null terminated
	char frameToSend[frameMin + 510 + 1];
	uint16_t codeCommandLength;
	uint8_t crc;
	char codeCommand[510];

	codeCommandLength = code_Command(com, codeCommand, com_len);

	frameToSend[0] = startFrame;
	memcpy(frameToSend + 1, dst, 3);
	memcpy(frameToSend + 1 + 3, device_address, 3);

	memcpy(frameToSend + 1 + 3 + 3, codeCommand, codeCommandLength);

	//przed zakodowaniem znaków
	crc = CRC_100(com, com_len);
	// 78/10 =7
	// 7+48 = 55
	// "7"
	frameToSend[1 + 3 + 3 + codeCommandLength] = crc / 10 + 48;
	// 78 % 10 = 8
	// 8 + 48 = 56
	// "8"
	frameToSend[1 + 3 + 3  + codeCommandLength + 1] = crc % 10 + 48;
	frameToSend[1 + 3 + 3  + codeCommandLength + 2] = endFrame;
	frameToSend[1 + 3 + 3  + codeCommandLength + 3] = 0;

	USART_fSend(frameToSend);
}

//Przerobienie znaków początku, końca i znaku kodującego na nadające się do przesyłania w ramce
int16_t code_Command(char *src, char *dst, uint8_t com_len) {
	uint16_t i;
	uint16_t j;
	for (i = 0, j = 0; i < com_len; ++i, ++j) {

		switch (src[i]) {
		case codingChar:
			dst[j++] = codingChar;
			dst[j] = codingChar;
			break;
		case startFrame:
			dst[j++] = codingChar;
			dst[j] = '[';
			break;
		case endFrame:
			dst[j++] = codingChar;
			dst[j] = ']';
			break;
		default:
			dst[j] = src[i];
			break;
		}
	}
	return j;
}

uint8_t CRC_100(char *src, uint8_t len) {
	uint8_t i;
	uint8_t temp = src[0];
	for (i = 1; i < len; ++i) {
		temp ^= src[i];
	}
	temp %= 100;
	return temp;
}

void analizeCommend(char* com, uint8_t len, char* sender_add) {
	if (strncmp(com,"GetSurvey()", (unsigned)11) == 0) {
		GetSurvey();
		char stringDistance[20];
		//sprintf(stringDistance, "%lu", (unsigned long)dmaDestination);
		strcat(stringDistance, "cm");
		sendFrame(sender_add, stringDistance, strlen(stringDistance));
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
