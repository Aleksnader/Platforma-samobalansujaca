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
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <float.h>
#include <math.h>
#include <string.h>
#include "LED_and_ADC.h"
#include "UART_ESP.h"
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
int16_t gyroData=0;
int16_t accData=0;
float angleGyro= 0.0f;
float angleAcc= 0.0f;
float angleX= 0.0f;
float SumAngleGyro= 0.0f;
float voltage=0;
float motorRPM=0.0f;
int stepTime = 0;
//uint8_t Received[8];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */


float error; // błąd regulacji
float integral = 0; // całka błędu
float derivative; // pochodna błędu
float lastError = 0; // ostatni błąd
float updatePID(float angle,float setpoint, float kp, float ki, float kd)
{

  // obliczenie błędu regulacji
  error = setpoint - angle;

  // obliczenie całki błędu
  integral = integral + (error*0.01f);

  // obliczenie pochodnej błędu
  derivative = (error - lastError)*100;

  // zapamiętanie ostatniego błędu
  lastError = error;

  // obliczenie wyjścia kontrolera
  return kp*error + ki*integral + kd*derivative;

}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim7) {			//przerwanie pomiarow i regulatora pid 100hz
	  //HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
	  gyroData = mpu_getGyroData();
	  accData = mpu_getAccData();

	  angleGyro+=(gyroData+93)*0.01*0.0076;

	  angleAcc = (accData*0.0001204)*90;

	  SumAngleGyro=(gyroData-65)*0.01*0.0076+angleX;
	  angleX = 0.999f * SumAngleGyro+ (1- 0.999f) * angleAcc;
	  //if( fabs(angleX)> 90){
		//  HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, 1);
	  //}else{
		//  HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, 0);
	  //}
	  motorRPM = updatePID(angleX, -4.0f, 6.0f ,200.0f , 0.01f);		//w miare dziala 4.5 0.5 2.0

  }
  if (htim == &htim6) {

  		  stepTime=stepTime+50000;			// czas miedzy krokami timera, powiekszone * 100

  		  if(stepTime>=60000000000.0f/(800.0f*fabs(motorRPM))&& fabs(motorRPM)>0.5f){   // 800- kroki do pelnego obrotu * ilosc obrotow na minute - powiekszone *100

  			  if(motorRPM>0){
  				//HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, 1);
  				GPIOA->ODR |= (1<<8);
  				//HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, 1);
  				GPIOB->ODR |= (1<<0);
  				//HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 1);
  				GPIOA->ODR |= (1<<6);
  			  }else{
  				//HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, 0);
  				//GPIOA->ODR |= (0<<8);
  				GPIOA->ODR &= ~(1<<8);
  				//HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, 0);
  				//GPIOB->ODR |= (0<<0);
  				GPIOB->ODR &= ~(1<<0);
  				//HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 0);
  				//GPIOA->ODR |= (0<<6);
  				GPIOA->ODR &= ~(1<<6);

  			  }
  			  //printf("predkosc: %s\n\r", predkosc);
  			  //HAL_GPIO_WritePin(STP2_GPIO_Port, STP2_Pin,1);
  			  GPIOA->ODR |= (1<<12);
  			  //HAL_GPIO_WritePin(STP_GPIO_Port, STP_Pin,1);
  			  GPIOB->ODR |= (1<<6);

  			  //HAL_GPIO_WritePin(STP2_GPIO_Port, STP2_Pin,0);
  			  //GPIOA->ODR |= (0<<12);
  			  GPIOA->ODR &= ~(1<<12);
  			  //HAL_GPIO_WritePin(STP_GPIO_Port, STP_Pin,0);
  			  //GPIOB->ODR |= (0<<6);
  			  GPIOB->ODR &= ~(1<<6);

  			  //HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
  			  //printf("krok\r\n");
  			  //printf("krok");

  	    stepTime=0;

  		  }
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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(M0_GPIO_Port, M0_Pin, 0);
  HAL_GPIO_WritePin(M1_GPIO_Port, M1_Pin, 1);
  HAL_GPIO_WritePin(M2_GPIO_Port, M2_Pin, 0);
  HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, 0);
  HAL_GPIO_WritePin(STP2_GPIO_Port, STP2_Pin, 0);

  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, 0);
  HAL_GPIO_WritePin(STP_GPIO_Port, STP_Pin, 0);

  HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, 0);



  //HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
  //HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
  //HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 0);
  //HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 0);
  HAL_Delay(250);
  MPU6050_Init();
  HAL_Delay(250);
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Base_Start_IT(&htim6);


  //HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  //LED_ADC_Init();

  //voltage = getBatteryVoltage();



  HAL_UART_Receive_IT(&huart1, Received, 26);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint32_t last_ms = HAL_GetTick();
  while (1)
  {



	  uint32_t now = HAL_GetTick();
	  //printf("x= %d\n",now);
	  if(now - last_ms > 5000){
		  //HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		 // voltage = getBatteryVoltage();


		  last_ms=now;

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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
