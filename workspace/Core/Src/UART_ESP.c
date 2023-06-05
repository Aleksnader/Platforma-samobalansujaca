/*
 * UART_ESP.c
 *
 *  Created on: 5 cze 2023
 *      Author: 48669
 */
#include "usart.h"
#include <stdio.h>
#include "main.h"
		char a;
		float  b;
		uint8_t Received[8];
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {		//odbieranie danych

	if (huart == &huart1) {

	//printf("Odebrana wiadomosc: %s\n\r", b);
	//sscanf(Received1, "%c%f", &a, &b);;

	printf("Odbieram %s\n\r", Received);

	//HAL_UART_Transmit_IT(&huart1,"as" , 2); // Rozpoczecie nadawania danych z wykorzystaniem przerwan
	HAL_UART_Receive_IT(&huart1, Received, 3); // Ponowne włączenie nasłuchiwania

	}

}
