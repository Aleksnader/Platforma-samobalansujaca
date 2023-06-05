/*
 * LED_and_ADC.c
 *
 *  Created on: 4 cze 2023
 *      Author: 48669
 */
#include "LED_and_ADC.h"
#include "adc.h"
#include "gpio.h"
#include <stdint.h>
#include <stdbool.h>
void LED_ADC_Init(){

	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);


}
float getBatteryVoltage(){
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1);

	uint32_t value = HAL_ADC_GetValue(&hadc1);
	float voltage = (3.3f * value/ 4096.0f)*6;
	printf("ADC = %.3f\n", voltage);


	if(voltage<=9.5f){switchLED(0,0,0,0);}
	else if(voltage>=9.5f&&voltage<=10.325f){switchLED(0,0,0,1);}
	else if(voltage>10.325f&&voltage<=11.15f){switchLED(0,0,1,1);}
	else if(voltage>11.15f&&voltage<=11.975f){switchLED(0,1,1,1);}
	else if(voltage>11.975f&&voltage<=12.8f){switchLED(1,1,1,1);}

	return voltage;


}
void switchLED(bool led1, bool led2, bool led3, bool led4){
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin,led1);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin,led2);
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin,led3);
		HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin,led4);

}
