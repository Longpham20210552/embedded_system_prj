/*
 * DHT.h
 *
 *  Created on: Jan 19, 2025
 *      Author: levan
 */

#ifndef DHT_H_
#define DHT_H_

#include "main.h" // Hoặc include header file chứa định nghĩa GPIO và HAL

// Định nghĩa các chân kết nối
#define DHT11_PORT GPIOB
#define DHT11_PIN GPIO_PIN_9

// Khai báo các biến
extern uint8_t RHI, RHD, TCI, TCD, SUM;
extern uint32_t pMillis, cMillis;
extern float tCelsius;
extern float tFahrenheit;
extern float RH;

// Khai báo các hàm
void microDelay (uint16_t delay);
uint8_t DHT11_Start (void);
uint8_t DHT11_Read (void);
float DHT11(void);

#endif /* DHT_H_ */
