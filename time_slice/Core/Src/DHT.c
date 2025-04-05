/*
 * DHT.c
 *
 *  Created on: Jan 19, 2025
 *      Author: levan
 */
#include "DHT.h" // Include file header DHT.h
#include "stm32f1xx_hal_conf.h"
// Định nghĩa các biến toàn cục (đã được khai báo extern trong DHT.h)
extern TIM_HandleTypeDef htim1;
uint8_t RHI, RHD, TCI, TCD, SUM;
uint32_t pMillis, cMillis;
float tCelsius = 0;
float tFahrenheit = 0;
float RH = 0;
uint8_t TCI;
uint8_t SUM;
uint8_t TCD;
uint8_t RHD;
uint8_t RHI;
// Định nghĩa hàm microDelay
void microDelay (uint16_t delay)
{
  __HAL_TIM_SET_COUNTER(&htim1, 0); // Đặt lại bộ đếm của timer

  while (__HAL_TIM_GET_COUNTER(&htim1) < delay); // Chờ cho đến khi bộ đếm đạt giá trị delay
}

// Định nghĩa hàm DHT11_Start
uint8_t DHT11_Start (void)
{
  uint8_t Response = 0;
  GPIO_InitTypeDef GPIO_InitStructPrivate = {0};

  GPIO_InitStructPrivate.Pin = DHT11_PIN;
  GPIO_InitStructPrivate.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStructPrivate); // set the pin as output

  HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 0);   // pull the pin low
  HAL_Delay(20);   // wait for 20ms
  HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 1);   // pull the pin high
  microDelay (40);   // wait for 40us

  GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructPrivate.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStructPrivate); // set the pin as input

  microDelay (40); // wait for 40us
  if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))
  {
    microDelay (80); // wait 80us if time very lag --> cannot start
    if ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))
    {
      Response = 1;
    }
  }

  pMillis = HAL_GetTick();
  cMillis = HAL_GetTick();
  while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)) && pMillis + 2 > cMillis)
  {
    cMillis = HAL_GetTick();
  }
  return Response;
}

// Định nghĩa hàm DHT11_Read
uint8_t DHT11_Read (void)
{
  uint8_t a,b;
  for (a=0;a<8;a++)
  {
    pMillis = HAL_GetTick();
    cMillis = HAL_GetTick();
    while (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)) && pMillis + 2 > cMillis)
    {  // wait for the pin to go high
      cMillis = HAL_GetTick();
    }
    microDelay (37);   // wait for 27-70 us
    if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))   // if the pin is low
      b&= ~(1<<(7-a));
    else
      b|= (1<<(7-a));
    pMillis = HAL_GetTick();
    cMillis = HAL_GetTick();
    while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)) && pMillis + 2 > cMillis)
    {  // wait for the pin to go low
      cMillis = HAL_GetTick();
    }
  }
  return b;
}

float DHT11 (void)
{
	if(DHT11_Start())
	{
	RHI = DHT11_Read(); // Relative humidity integral
	RHD = DHT11_Read(); // Relative humidity decimal
	TCI = DHT11_Read(); // Celsius integral
	TCD = DHT11_Read(); // Celsius decimal
	SUM = DHT11_Read(); // Check sum
	if (RHI + RHD + TCI + TCD == SUM)
	{
	// Can use RHI and TCI for any purposes if whole number only needed
	tCelsius = (float)TCI + (float)(TCD/10.0);
	tFahrenheit = tCelsius * 9/5 + 32;
	RH = (float)RHI + (float)(RHD/10.0);
	// Can use tCelsius, tFahrenheit and RH for any purposes
	}
	return RH;
	}
}
