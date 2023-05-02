//////////////////////////////////////////////////////////////////////////////////////
/* This file contains the implementation of the functions relating to 
   measuring the distance to an obstacle from the front of the vehicle 
	 using the ultrasonic sensor in UART mode. 
*////////////////////////////////////////////////////////////////////////////////////
#include <stdio.h>
#include <stdlib.h>
#include "stm32f0xx.h"
#include "distance_front.h"


//Dist Buffer
uint8_t bytes[2];

/*
	This function intializes the USART3 Peripheral which includes setting the 
	GPIOC pins 11 and 10 to alternate funciton mode and selecting AF1 for both
	pins to put them in USART3 Tx and USART3 Rx modes.
	
	INPUTS: None
	OUTPUTS: None
*/


void USART3_Init(void) {
	
	
	RCC -> AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC -> APB1ENR |= RCC_APB1ENR_USART4EN;
	
	
	//Configuring PA0 and PA1 to be in alternate mode
	GPIOA -> MODER &= ~((1<<3) | (1<<2) | (1<<1) | (1<<0));
	GPIOA -> MODER |= (1<<3) | (1<<1);
	
	//Configuring GPIOB AFR register
	GPIOA -> AFR[0] |=   (0x04 << GPIO_AFRL_AFSEL1_Pos) | (0x04 <<GPIO_AFRL_AFSEL0_Pos);

	//Setting USART3 Baud rate to 9600
	USART4 -> BRR = HAL_RCC_GetHCLKFreq()/9600;
	
	//setting the DMAR bit in USART_CR3 register
	USART4-> CR3 |= 1<<6;
	
	//Enabling Tx, Rx and the peripheral
	USART4 -> CR1 |= (1<<0) | (1<<2) | (1<<3) | (1<<5) | (1<<7);

}	

//////////////////////////////////////////////////////////////////////////////////////////////

/*
	This function transmits a given character on the
	USART3 Tx pin.
	INPUTS: char letter - character (which is automatically converted to 
					an integer) or integer value of desired character to transmit
	OUTPUTS: None
*/
void transmit(char letter){
	
	int i = 0;
	//Busy loop while TXE bit is still 0
	while(!(USART4 -> ISR & (1<<7))) {
		i = i;
	}
	
	//Setting character value to transmit data register
	USART4 -> TDR = letter ;
		
}

/*
	This function transmits a given string of characters on the
	USART3 Tx pin.
	INPUTS: char string[] - array of characters (which are automatically 
					converted to an integer) or integer value of desired character 
					to transmit
	OUTPUTS: None
*/
void transmit_str(char string[]) {
	int i = 0;
	while (string[i] != 0) {	
		transmit(string[i]);
		i = i+1;
	}
	return;
}

/*
	This function receives a byte from the RDR register stored from the
	USART3 Rx pin.
	INPUTS: None
	OUTPUTS: uint8_t character - Character read from the RDR register. 
*/
uint8_t receive_char(void) {
	uint8_t character = 'a';
	while(!(USART4 -> ISR & USART_ISR_RXNE));
	character = USART4 -> RDR;

	return character;
}

/*
	This function transmits the value 0x55 to the ultrasonic sensor and
	then saves the 16 bits of data that are returned which represent the distance 
	to the obstacle in mm. 
	
	INPUTS: None
	OUTPUTS: int frnt_dst - integer representing the distance to an obstacle in 
					 in front of the over given in mm. 
*/


void GetFrontDistance(void){
	
	uint16_t dist = 0;
	
  transmit(0x55);
	
	bytes[0] = receive_char();
	bytes[1] = receive_char();

	
	dist = bytes[0]*256 + bytes[1];
	
	if(dist == 0) {
		GPIOC -> ODR |= 1<<6;
		GPIOC -> ODR &= ~((1<<7) | (1<<8) | (1<<9));
	}
	else 
	if(dist > 0 && dist < 50) {
		GPIOC -> ODR |= 1<<7;
		GPIOC -> ODR &= ~((1<<6) | (1<<8) | (1<<9));
	}
	else if(dist >=50 && dist < 100) {
		GPIOC -> ODR |= 1<<8;
		GPIOC -> ODR &= ~((1<<6) | (1<<7) | (1<<9));

	}
	else if(dist >= 100 && dist < 256) {
		GPIOC -> ODR |= 1<<9;
		GPIOC -> ODR &= ~((1<<7) | (1<<8) | (1<<6));
	}
	else {
		GPIOC -> ODR &= ~((1<<9)|(1<<8)|(1<<7)|(1<<6));
	}
}	
	
uint16_t Sonar_GetDistance(void){
	uint16_t dist = 0;
	
  transmit(0x55);
	
	bytes[0] = receive_char();
	bytes[1] = receive_char();

	
	dist = (bytes[0]<<8) + bytes[1];
	return dist;

}


	
		

