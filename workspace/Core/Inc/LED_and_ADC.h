/*
 * LED_and_ADC.h
 *
 *  Created on: 4 cze 2023
 *      Author: 48669
 */

#pragma once

#include <stdint.h>
#include "gpio.h"
#include <stdbool.h>
void LED_ADC_Init();
float getBatteryVoltage();
void switchLED(bool led1, bool led2, bool led3, bool led4);
