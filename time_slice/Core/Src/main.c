/*
 * main.c
 *
 *  Created on: Jan 21, 2025
 *      Author: levan
 */


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
uint8_t hour; // Gi�? thực
uint8_t min;  // Phút thực
uint8_t sec;  // Giây thực
uint8_t set_hour; // Gi�? thực
uint8_t set_min;  // Phút thực
uint8_t set_sec;  // Giây thực
float t;
char tx_buff[100] = "START_TX";
char Lcd_send1[20];
char LCD_send2[20];
uint8_t count=0;
uint16_t time1=0;
uint16_t time2=0;
uint16_t T=0;
uint8_t printf_to_com[20]; // mang luu gia tri thu tu computer de in ra man hinh
uint8_t Rx_data[50]; // Mảng lưu dữ liệu nhận được
uint8_t Rx_data_reel[50];
uint16_t rx_len = 0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void I2C_DS1307(void)
{
	  hour = DS1307_GetHour(); 		// Lấy gi�?
	  min = DS1307_GetMinute();	  	// Lấy phút
	  sec = DS1307_GetSecond();
}
void One_write_DHT11(void)
{
	t = DHT11();
}
void I2C_LCD(void)
{
	  HD44780_Clear();
	  HD44780_SetCursor(0,0);
	  sprintf(Lcd_send1, "%.1f%%", t);
	  HD44780_PrintStr(Lcd_send1);


	  HD44780_SetCursor(7,0);
	  sprintf(LCD_send2, "%02d:%02d:%02d   ", hour, min,sec);
	  HD44780_PrintStr(LCD_send2);

	  HAL_Delay(100);

	  if(rx_len!=0)
	  {
	  }
}
void UART_TX(void)
{
	int ret = snprintf((char*)tx_buff, sizeof(tx_buff), "H : %.2f\r\nTIME : %02u:%02u:%02u\r\n", t, hour, min, sec);
	    if (ret < 0 || ret >= sizeof(tx_buff)) {
	        // Xử lý lỗi (chuỗi quá dài)
	        // Ví dụ: in thông báo lỗi ra debug hoặc bỏ qua
	        // Bạn *cần* xử lý lỗi này để tránh crash chương trình
	        const char* error_msg = "Error: Buffer too small\r\n";
	        HAL_UART_Transmit(&huart1, (uint8_t*)error_msg, strlen(error_msg), HAL_MAX_DELAY);
	        return; // Dừng hàm nếu có lỗi
	    }
	    HAL_UART_Transmit(&huart1, tx_buff, strlen((char*)tx_buff), HAL_MAX_DELAY);
}
void UART_RX(void)
{
	memset(Rx_data, 0, sizeof(Rx_data));
//	memset(Rx_data_reel, 0, sizeof(Rx_data));
//	memset(printf_to_com, 0, sizeof(Rx_data));
	HAL_UART_Receive(&huart1, Rx_data, sizeof(Rx_data), 1000);
	rx_len=0;
	if(strlen(Rx_data) > 0)
	{
	    if (Rx_data[0] == 'S' && Rx_data[1] == 'T')
	    {
	    	char *p = &Rx_data[3];
	    	sscanf(p,"%d%d%d", &set_hour, &set_min, &set_sec);
			//HD44780_SetCursor(0,1);
			//sprintf(LCD_send2, "Time:%02d:%02d:%02d   ", set_hour, set_min,set_sec);
			//HD44780_PrintStr(LCD_send2);
			//HAL_Delay(1000);
	    	setDS1307Time(set_hour, set_min, set_sec);

	    }
	    else if (Rx_data[0] == 'S' && Rx_data[1] == 'P')
	    {

	    	char *p = &Rx_data[2];
	    	HD44780_SetCursor(0,1);
	    	HD44780_PrintStr("              ");
	    	HD44780_SetCursor(0,1);
	    	HD44780_PrintStr(p);
	    	HAL_Delay(3000);
	    }
	    else if (Rx_data[0] == 'G')
	    {
	    	UART_TX();
	    }
	    else if(strlen(Rx_data)>1)
	    {
	    	HD44780_SetCursor(0,1);
	    	HD44780_PrintStr("Non_true_form");
	    	HAL_Delay(3000);
	    }

	}

}

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



typedef struct {
    int task_id;         	// ID của tác vụ
    int execution_time;  	// Thời gian thực thi (ms)--> C
    int t1;          	// Deadline (ms)  --> D  + execution_time (not essential)
    int t2;       		// Chu kì lặp lại --> T
    int ready;           	// Trạng thái sẵn sàng (1 = sẵn sàng, 0 = chưa)
} Task;
#define NUM_TASKS 5  // Số lượng tác vụ
Task tasks[NUM_TASKS] = {
    {1, 45, 100, 300, 0},  		// Tác vụ TX:
    {2, 100, 200, 400, 0}, 		// Tác vụ RX:
	{3, 50, 100, 200, 0}, 		// Tác vụ DS1307:
	{4, 30, 60, 3000, 0}, 		// Tác vụ DHT11:
    {5, 50, 100, 300, 0}  		// Tác vụ LCD:
};



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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  HD44780_Init(2);
  DS1307_Init(&hi2c1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {
//	  const char* test_msg = "Hello World!\r\n";
//	  HAL_UART_Transmit(&huart1, (uint8_t*)test_msg, strlen(test_msg), HAL_MAX_DELAY);
//	  One_write_DHT11();
//	  I2C_DS1307();
//	  I2C_LCD();
//	  UART_RX();




	  for (int i = 0; i < NUM_TASKS; i++)
	  	  {
	  	      if (HAL_GetTick() >= tasks[i].t2)
	  	      {
	  	           tasks[i].ready = 1;               // Đánh dấu tác vụ sẵn sàng
	  	      }
	  	  }

	  	  if(tasks[3].ready)
	  	  {
//	  		  time1= HAL_GetTick();
	  		  One_write_DHT11();
//	  		  time2= HAL_GetTick();
//	  		  T =time2-time1;
	  		  tasks[3].t2 = tasks[3].execution_time + HAL_GetTick();  // Reset thời gian co the thực thi
	  		  tasks[3].ready = 0;
	  	  }
	  	  else if (tasks[2].ready)
	  	  {
	  		  I2C_DS1307();
	  		  tasks[2].t2 = tasks[2].execution_time + HAL_GetTick();  // Reset thời gian co the thực thi
	  		  tasks[2].ready = 0;
	  	  }
	  	  else if (tasks[4].ready)
	  	  {
	  		  I2C_LCD();
	  		  tasks[4].t2 = tasks[4].execution_time + HAL_GetTick();  // Reset thời gian co the thực thi
	  		  tasks[4].ready = 0;
	  	  }
	  	  else if (tasks[1].ready)
	  	  {
	  		  UART_RX();
	  		  tasks[1].t2 = tasks[1].execution_time + HAL_GetTick();  // Reset thời gian co the thực thi
	  		  tasks[1].ready = 0;
	  	  }
	  	  else if (tasks[0].ready)
	  	  {
	  		  UART_TX();
	  		  tasks[0].t2 = tasks[0].execution_time + HAL_GetTick();  // Reset thời gian co the thực thi
	  		  tasks[0].ready = 0;
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
