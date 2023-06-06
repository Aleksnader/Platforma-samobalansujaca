/*
 * UART_ESP.c
 *
 *  Created on: 5 cze 2023
 *      Author: 48669
 */
#include "usart.h"
#include <stdio.h>
#include "main.h"

uint8_t Received[26];
char chP[5];
char chI[5];
char chD[5];

float  P;
float  I;
float  D;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {		//odbieranie danych


	if (huart == &huart1) {

	//printf("Odebrana wiadomosc: %s\n\r", b);
	//sscanf(Received1, "%c%f", &a, &b);;

		//printf("Odbieram %s\n\r", Received);

	int i=0;


			while(Received[i]!=0){
				//printf("received %u \n\r ",Received[i]);
				if(i>=2&&i<=6){
					chP[i-2]= Received[i];

					//printf("wtkniete do chP %u , %u \n ",Received[i] ,i-2);
				}
				if(i>=2&&i<=6){
									chP[i-2]= Received[i];

									//printf("wtkniete do chP %u , %u \n ",Received[i] ,i-2);
								}
				if(i>=10&&i<=14){
									chI[i-10]= Received[i];

									//printf("wtkniete do chP %u , %u \n ",Received[i] ,i-2);
								}
				if(i>=18&&i<=22){
													chD[i-18]= Received[i];

													//printf("wtkniete do chP %u , %u \n ",Received[i] ,i-2);
												}
				i++;
			}
			//printf("chP = %s\n\r", chP);
			sscanf(chP, "%f", &P);
			sscanf(chI, "%f", &I);
			sscanf(chD, "%f", &D);


			printf("moje P %.3f\n\r",P);
			printf("moje I %.3f\n\r",I);
			printf("moje D %.3f\n\r",D);


	//HAL_UART_Transmit_IT(&huart1,"as" , 2); // Rozpoczecie nadawania danych z wykorzystaniem przerwan
	HAL_UART_Receive_IT(&huart1, Received, 26); // Ponowne włączenie nasłuchiwania

	}

}
