/*
 * MPU6050.c
 *
 *  Created on: Jun 3, 2023
 *      Author: 48669
 */

#include "MPU6050.h"
#include "i2c.h"
void MPU6050_Init(){				//restart i2c
	HAL_I2C_Init(&hi2c1);
	mpu_write(0x6B,0b00000000);	//uspienie na 0,
	mpu_write(0x1B,0b00000000);	// konfiguracja zyroskopu / max 200 stopni na sekunde
	mpu_write(0x1C,0b00001000);	//konfiguracja akcelerometru / 4g
	//HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
}


uint8_t mpu_read(uint8_t reg)		//odczytaj dane z pojedynczego rejestru
{

	uint8_t value = 0;
	HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, reg, 1, &value, sizeof(value),1);
	if(ret==HAL_OK){
			printf("odczytalem 0x%02X z rejestru 0x%02X\n",value,reg,ret);
			return value;
		}else{
			printf("nie udalo sie odczytac danych z rejestru 0x%02X\n",reg);
			printf(reg);
			MPU6050_Init();
			return ret;
		}

}
uint8_t mpu_write(uint8_t reg, uint8_t data){	//zapisz dane w rejestrze

	HAL_StatusTypeDef ret = HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, reg, 1, &data, sizeof(data), 1);

	if(ret==HAL_OK){
		printf("zapisalem 0x%02X w rejestrze 0x%02X\r\n",data,reg);

	}else{
		//printf("nie udalo sie zapisac w 0x%02X rejestrze 0x%02\n",data,reg);

	}
	return ret;
}

uint8_t mpu_read_WhoAmI()	 	//test who am i
{
	return mpu_read(MPU6050_WHO_AM_I);
}

int16_t mpu_getGyroData(){		//pobierz dane z żyroskopu
	uint8_t data[2];
	int16_t x_gyro;
	HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR,MPU6050_GYRO , 1, data, 2, 1);
	if(ret==HAL_OK){
		x_gyro = ((int16_t)data[0]<<8) + data[1];
		//printf("x axis gyro %d\n",x_gyro);
	return x_gyro;
	}else{
		MPU6050_Init();
		//printf("nie udalo sie pobrac danych zyroskopu\r\n");
		return 404;
	}


}
int16_t mpu_getAccData(){		//pobierz dane z akcelerometru
	uint8_t data[2];
	int16_t x_acc;
	HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR,MPU6050_ACC , 1, data, 2, 1);
	if(ret==HAL_OK){
		x_acc = ((int16_t)data[0]<<8) + data[1];
		//printf("x axis acc %d\n",x_acc);
	return x_acc;
	}else{
		MPU6050_Init();
		return 404;
	}
}



