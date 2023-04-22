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
/*
void USART3_Init(void) {
	//Configuring PB10 and PB11 to be in alternate mode
	GPIOB -> MODER &= ~((1<<23) | (1<<22) | (1<<21) | (1<<20));
	GPIOB -> MODER |= (1<<23) | (1<<21);
	
	//Configuring GPIOB AFR register
	GPIOB -> AFR[1] |=   (0x04 << GPIO_AFRL_AFSEL1_Pos) | (0x04 <<GPIO_AFRL_AFSEL0_Pos);

	//Setting USART3 Baud rate to 9600
	USART3 -> BRR = HAL_RCC_GetHCLKFreq()/9600;
	
	//setting the DMAR bit in USART_CR3 register
	USART3-> CR3 |= 1<<6;
	
	//Enabling Tx, Rx and the peripheral
	USART3 -> CR1 |= (1<<0) | (1<<2) | (1<<3) | (1<<5) | (1<<7);
	
}
*/

////////BACKUP FUNCTION for if PB10,PB11,PC4,PC5 are needed for something else////////////////////

void USART3_Init(void) {
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

/*////////BACKUP FUNCTION for if PB10,PB11,PA1,PA0  are needed for something else////////////////////

void USART3_Init(void) {
	//Configuring PC4 and PC5 to be in alternate mode
	GPIOC -> MODER &= ~((1<<11) | (1<<10) | (1<<9) | (1<<8));
	GPIOC -> MODER |= (1<<11) | (1<<9);
	
	//Configuring GPIOC AFR register
	GPIOC -> AFR[0] |=   (0x01 << GPIO_AFRL_AFSEL5_Pos) | (0x01 <<GPIO_AFRL_AFSEL4_Pos);

	//Setting USART3 Baud rate to 9600
	USART3 -> BRR = HAL_RCC_GetHCLKFreq()/9600;
	
	//setting the DMAR bit in USART_CR3 register
	USART3-> CR3 |= 1<<6;
	
	//Enabling Tx, Rx and the peripheral
	USART3 -> CR1 |= (1<<0) | (1<<2) | (1<<3) | (1<<5) | (1<<7);
	
}

*///////////////////////////////////////////////////////////
	
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
	
	char* message = "hi";
	transmit_message(message);
	
	if(dist == 0) {
		GPIOC -> ODR |= 1<<6;
		GPIOC -> ODR &= ~((1<<7) | (1<<8) | (1<<9));
	}
	else 
	if(dist > 0 && dist < 50) {
		GPIOC -> ODR |= 1<<7;
		GPIOC -> ODR &= ~((1<<6) | (1<<8) | (1<<9));
		char* message = "short \n";
		transmit_message(message);
	}
	else if(dist >=50 && dist < 100) {
		GPIOC -> ODR |= 1<<8;
		GPIOC -> ODR &= ~((1<<6) | (1<<7) | (1<<9));
		char* message = "med \n";
		transmit_message(message);
	}
	else if(dist >= 100 && dist < 256) {
		GPIOC -> ODR |= 1<<9;
		GPIOC -> ODR &= ~((1<<7) | (1<<8) | (1<<6));
		char* message = "long \n";
		transmit_message(message);
	}
	else {
		GPIOC -> ODR &= ~((1<<9)|(1<<8)|(1<<7)|(1<<6));
	}
	
	
	

}	


#include "stm32f0xx_hal.h"

void init_usart3(void){
	//enable GPIOC
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; 
	
	
	//set mode to Alternate function
	GPIOC->MODER |= (0x2 << 10) | (0x2 << 8);
	//Set AFR to AF1
  GPIOC->AFR[0] = (0x1 << 20) | (0x1 << 16);
	
	//ENABLE USART3
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN; 
	//HC-05 supported baud rates 9600,19200,38400,57600,115200,230400,460800
	USART3->BRR =   (0xFFFF  & (HAL_RCC_GetHCLKFreq() / 9600));
	
	
	//set word length to 8 set Bit 12 M0: and Bit 28 M1: 
	USART3->CR1  &= ~(1 << 12);
	USART3->CR1  &= ~(1 << 28);
	
	//Enable RXNE interrupt Bit 5 RXNEIE:
	USART3->CR1  |= (1 << 5);


	//Enable USART3 Transmitter enable Bit 3 TE:
	USART3->CR1  |= (1 << 3);
	//Enable USART3 receiver enable Bit 2 RE:
	USART3->CR1  |= (1 << 2);
	//Enable USART3 Bit 0 UE:
	USART3->CR1  |= (1 << 0);
	

    //for the USART
	//NVIC_EnableIRQ(USART3_4_IRQn); 
	//NVIC_SetPriority(USART3_4_IRQn,1);
	
	
}

int state = 0;

//if we need usart to start an interrupt
void USART3_4_IRQHandler(void){
     __disable_irq();
		 USART3->TDR = USART3->RDR;

//	 else{
//		 if (state == 0){
//				 bytes[0] = receive_char();
//			   state = 1;
//		 } else {
//				bytes[1] = receive_char();
//		    dist = bytes[0]*256 + bytes[1];
//			  state = 0;
//		 }
//		
//	 }
		 
	 
		__enable_irq();
	 	
}


// sends a transmit message
void transmit_message(char* message){
	for(int i = 0; message[i] != '\0'; i++){
					_Bool data_reg_empty_flag = ((1<<7) & ( USART3->ISR)) == 0;
					while(data_reg_empty_flag){
						//wait till reg is ready
               data_reg_empty_flag = ((1<<7) & ( USART3->ISR)) == 0;
					}
					
					USART3->TDR = message[i];
					HAL_Delay(10);

				}
}

