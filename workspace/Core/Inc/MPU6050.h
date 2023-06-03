/*
 * MPU6050.h
 *
 *  Created on: Jun 3, 2023
 *      Author: 48669
 */

#pragma once

#include <stdint.h>

//REJESTRY MPU5060
#define MPU6050_WHO_AM_I 0x75	//zwraca rejestr czujnika
#define MPU6050_ADDR 	 0xD0	//rejestr czujnika MPU6050
#define MPU6050_GYRO	 69		//rejestr żyroskopu
#define MPU6050_ACC		 63		//rejestr akclerometru


void MPU6050_Init();
uint8_t mpu_read(uint8_t reg);	//odczyt wartosci z danego rejestru
uint8_t mpu_read_WhoAmI();		//test whoami
int16_t mpu_getGyroData();		//pobierz dane z żyroskopu
int16_t mpu_getAccData();		//pobierz dane z akcelerometru
uint8_t mpu_write(uint8_t reg, uint8_t data);

