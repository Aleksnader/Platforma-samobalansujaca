/*
 * UART_ESP.h
 *
 *  Created on: 5 cze 2023
 *      Author: 48669
 */
#pragma once
#include <stdint.h>
#include <float.h>
#include "usart.h"


extern uint8_t Received[8];
extern float P;
extern float I;
extern float D;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
