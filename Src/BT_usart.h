#include <stdio.h>
#include <stdlib.h>
#include "stm32f0xx.h"


/*
	This initializes the USART3 for the Blue Tooth connection
	INPUTS: None
	OUTPUTS: None 
*/
void BT_usart_init(void);

/*
	This reads the USART3 Recieve register for the Blue Tooth connection
	INPUTS: None
	OUTPUTS: uint8_t 
*/
uint8_t BT_usart_receive_char(void) ;

/*
	This transmits a char array accross the USART3 for the Blue Tooth connection
	INPUTS: Char array
	OUTPUTS: None 
*/
void BT_usart_transmit_message(char* message);
/*
	This transmits a char array accross the USART3 for the Blue Tooth connection
	INPUTS: Char array
	OUTPUTS: None 
*/
char* itoa(int value, char* str, int radix);