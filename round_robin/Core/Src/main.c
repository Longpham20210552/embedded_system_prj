/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "liquidcrystal_i2c.h"
#include "DS1307.h"
#include "DHT.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t hour;
uint8_t min;
uint8_t sec;
int set_hour = 0xFF;
int set_min = 0xFF;
int set_sec = 0xFF;
float temp = 27;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

osThreadId DHT11Handle;
osThreadId DS1307Handle;
osThreadId RXTaskHandle;
osThreadId LCDTaskHandle;
osThreadId TXTaskHandle;
osThreadId LCDROW2Handle;
osMessageQId LCDHandle;
osMessageQId TXHandle;
osMutexId I2CHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
void StartDHT11(void const * argument);
void StartIDS1307(void const * argument);
void StartRX(void const * argument);
void StartLCD(void const * argument);
void StartTX(void const * argument);
void StartTaskROW2(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t decToBcd(uint8_t val) {
  return ((val / 10) << 4) | (val % 10);
}

uint8_t bcdToDec(uint8_t val) {
  return ((val >> 4) * 10) + (val & 0x0F);
}

void setDS1307Time(uint8_t set_hour, uint8_t set_min, uint8_t set_sec) {
  uint8_t data[3];

  data[0] = decToBcd(set_sec); // Giây
  data[1] = decToBcd(set_min); // Phút
  data[2] = decToBcd(set_hour); // Giờ

  // Ghi dữ liệu vào DS1307
  HAL_I2C_Mem_Write(&hi2c1, 0xD0, 0x00, 1, data, 3, 1000); // Địa chỉ DS1307 là 0xD0
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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  HD44780_Init(2);
  DS1307_Init(&hi2c1);
  float t = DHT11();
  if(t != 0)
  {
	  temp = t;
  }
  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of I2C */
  osMutexDef(I2C);
  I2CHandle = osMutexCreate(osMutex(I2C));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of LCD */
  osMessageQDef(LCD, 5, 20);
  LCDHandle = osMessageCreate(osMessageQ(LCD), NULL);

  /* definition and creation of TX */
  osMessageQDef(TX, 10, 50);
  TXHandle = osMessageCreate(osMessageQ(TX), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of DHT11 */
  osThreadDef(DHT11, StartDHT11, osPriorityHigh, 0, 512);
  DHT11Handle = osThreadCreate(osThread(DHT11), NULL);

  /* definition and creation of DS1307 */
  osThreadDef(DS1307, StartIDS1307, osPriorityHigh, 0, 512);
  DS1307Handle = osThreadCreate(osThread(DS1307), NULL);

  /* definition and creation of RXTask */
  osThreadDef(RXTask, StartRX, osPriorityHigh, 0, 512);
  RXTaskHandle = osThreadCreate(osThread(RXTask), NULL);

  /* definition and creation of LCDTask */
  osThreadDef(LCDTask, StartLCD, osPriorityHigh, 0, 512);
  LCDTaskHandle = osThreadCreate(osThread(LCDTask), NULL);

  /* definition and creation of TXTask */
  osThreadDef(TXTask, StartTX, osPriorityHigh, 0, 512);
  TXTaskHandle = osThreadCreate(osThread(TXTask), NULL);

  /* definition and creation of LCDROW2 */
  osThreadDef(LCDROW2, StartTaskROW2, osPriorityHigh, 0, 512);
  LCDROW2Handle = osThreadCreate(osThread(LCDROW2), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDHT11 */
/**
  * @brief  Function implementing the DHT11 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDHT11 */
void StartDHT11(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
osDelay(250);
  for(;;)
  {
	  float t = DHT11();
	  if(t != 0)
	  {
		  temp = t;
	  }
		char Mess[20];
		sprintf(Mess, "H: %0.1f%%", temp);
		osMessagePut(TXHandle, (uint32_t)Mess, osWaitForever);
      osDelay(250);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartIDS1307 */
/**
* @brief Function implementing the DS1307 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIDS1307 */
void StartIDS1307(void const * argument)
{
  /* USER CODE BEGIN StartIDS1307 */
  /* Infinite loop */
  for(;;)
  {
	  if (osMutexWait(I2CHandle, osWaitForever) == osOK)
	  {
		  if(set_hour < 24 && set_min < 60 && set_sec < 60 && set_hour >= 0 && set_min >= 0 && set_sec >= 0)
		  {
			  setDS1307Time(set_hour, set_min, set_sec);
			  set_hour = 0xFF;
			  set_min = 0xFF;
			  set_sec = 0xFF;
		  }
		  else
		  {
			hour = DS1307_GetHour();
			min = DS1307_GetMinute();
			sec = DS1307_GetSecond();
			char Mess[20];
			sprintf(Mess, "Time Now: %02d:%02d:%02d\n", hour, min,sec);
			  osMessagePut(TXHandle, (uint32_t)Mess, osWaitForever);
		  }
			osMutexRelease(I2CHandle);
	  }
	  osDelay(250);
  }
  /* USER CODE END StartIDS1307 */
}

/* USER CODE BEGIN Header_StartRX */
/**
* @brief Function implementing the RXTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRX */
void StartRX(void const * argument)
{
  /* USER CODE BEGIN StartRX */
  /* Infinite loop */
	char Rx_data[50];
  for(;;)
  {
		memset(Rx_data,0,sizeof(Rx_data));
		HAL_UART_Receive(&huart1, (uint8_t*)Rx_data, sizeof(Rx_data), 500);
		if(strlen(Rx_data) > 0)
		{
			if (Rx_data[0] == 'S' && Rx_data[1] == 'T' && strlen(Rx_data) > 5)
			{
				int set_h, set_m, set_s;
				if (sscanf(Rx_data, "ST%d%d%d", &set_h, &set_m, &set_s) == 3)
				{
					set_hour = set_h;
					set_min = set_m;
					set_sec = set_s;
					char Mess[] = "Set DS1307\n";
					osMessagePut(TXHandle, (uint32_t)Mess, osWaitForever);
					HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
				}
				else
				{
					char Mess[] = "Invalid\n";
					osMessagePut(TXHandle, (uint32_t)Mess, osWaitForever);
				}
			}
			else if (Rx_data[0] == 'S' && Rx_data[1] == 'P')
			{
				char *p = &Rx_data[3];
				osMessagePut(LCDHandle, (uint32_t)p, osWaitForever);
				char Mess[] = "Print To LCD\n";
				osMessagePut(TXHandle, (uint32_t)Mess, osWaitForever);
			}
			else if (Rx_data[0] == 'G')
			{
				char Mess[20];
				sprintf(Mess, "Time: %02d:%02d:%02d\n", hour, min,sec);
				osMessagePut(TXHandle, (uint32_t)Mess, osWaitForever);
			}
			else if(strlen(Rx_data)>1)
			{
				char Err[] = "Non_true_form\n";
				osMessagePut(TXHandle, (uint32_t)Err, osWaitForever);
				char Err1[] = "Non_true_form";
				osMessagePut(LCDHandle, (uint32_t)Err1, osWaitForever);
			}
		}
      osDelay(250);
  }
  /* USER CODE END StartRX */
}

/* USER CODE BEGIN Header_StartLCD */
/**
* @brief Function implementing the LCDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLCD */
void StartLCD(void const * argument)
{
  /* USER CODE BEGIN StartLCD */
  /* Infinite loop */
  char LCD_ROW_1[20];
  for(;;)
  {
	  if (osMutexWait(I2CHandle, osWaitForever) == osOK)
	  {
		  HD44780_SetCursor(0,0);
		  HD44780_PrintStr("                 ");
		  HD44780_SetCursor(0,0);
		  sprintf(LCD_ROW_1, "%.1f%% ", temp);
		  HD44780_PrintStr(LCD_ROW_1);
		  sprintf(LCD_ROW_1, "%02d:%02d:%02d   ", hour, min,sec);
		  HD44780_SetCursor(8,0);
		  HD44780_PrintStr(LCD_ROW_1);
		  osMutexRelease(I2CHandle);
	  }
	  osDelay(250);
  }
  /* USER CODE END StartLCD */
}

/* USER CODE BEGIN Header_StartTX */
/**
* @brief Function implementing the TXTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTX */
void StartTX(void const * argument)
{
  /* USER CODE BEGIN StartTX */
  /* Infinite loop */
	osEvent event;
  for(;;){
	  event = osMessageGet(TXHandle, osWaitForever);
	  if (event.status == osEventMessage){
		  HAL_UART_Transmit(&huart1, (uint8_t*)event.value.v, strlen((char*)event.value.v), HAL_MAX_DELAY);
	  }
	  osDelay(250);
  }
  /* USER CODE END StartTX */
}

/* USER CODE BEGIN Header_StartTaskROW2 */
/**
* @brief Function implementing the LCDROW2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskROW2 */
void StartTaskROW2(void const * argument)
{
  /* USER CODE BEGIN StartTaskROW2 */
  /* Infinite loop */
	osEvent event;
  for(;;){
	  event = osMessageGet(LCDHandle, osWaitForever);
	  if (event.status == osEventMessage){
		  if (osMutexWait(I2CHandle, osWaitForever) == osOK){
			  HD44780_SetCursor(0,1);
			  HD44780_PrintStr("                 ");
			  HD44780_SetCursor(0,1);
			  HD44780_PrintStr((char *)event.value.v);
			  osMutexRelease(I2CHandle);
		  }
	  }
	  osDelay(250);
  }
  /* USER CODE END StartTaskROW2 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
