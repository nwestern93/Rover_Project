/////////////////////////////////////////////////////////////////////////////////////////////////////
/* This is the header file for the functions relating to measuring the distance to an obstacle
   from the front of the vehicle using the ultrasonic sensor in UART mode. 
	
*///////////////////////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <stdlib.h>
#include "stm32f0xx.h"

/*
	This function intializes the USART3 Peripheral which includes setting the 
	GPIOC pins 11 and 10 to alternate funciton mode and selecting AF1 for both
	pins to put them in USART3 Tx and USART3 Rx modes.
	
	INPUTS: None
	OUTPUTS: None
*/
void USART3_Init(void);


/*
	This function transmits a given character on the
	USART3 Tx pin.

	INPUTS: char letter - character (which is automatically converted to 
					an integer) or integer value of desired character to transmit
	OUTPUTS: None
*/
void transmit(char letter);


/*
	This function transmits a given string of characters on the
	USART3 Tx pin.

	INPUTS: char string[] - array of characters (which are automatically 
					converted to an integer) or integer value of desired character 
					to transmit
	OUTPUTS: None
*/
void transmit_str(char string[]);

/*
	This function receives the byte stored in the RDR register from the
	USART3 Rx pin.

	INPUTS: None
	OUTPUTS: uint8_t character - character stored in the RDR register
*/
uint8_t receive_char(void);

/*
	This function transmits the value 0x55 to the ultrasonic sensor and
	then saves the 16 bits of data that are returned which represent the distance 
	to the obstacle in mm. 
	
	INPUTS: None
	OUTPUTS: int frnt_dst - integer representing the distance to an obstacle in 
					 in front of the over given in mm. 

*/
void GetFrontDistance(void);


