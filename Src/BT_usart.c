#include "BT_usart.h"
#include <math.h>  

void BT_usart_init(void){
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



uint8_t BT_usart_receive_char(void) {
	uint8_t character = 'a';
	while(!(USART3 -> ISR & USART_ISR_RXNE));
	character = USART3 -> RDR;

	return character;
}



void BT_usart_transmit_message(char* message){
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

//// Function to swap two numbers
//void swap(char *x, char *y) {
//    char t = *x; *x = *y; *y = t;
//}
// 
//// Function to reverse `buffer[i…j]`
//char* reverse(char *buffer, int i, int j)
//{
//    while (i < j) {
//        swap(&buffer[i++], &buffer[j--]);
//    }
// 
//    return buffer;
//}
// 
//// Iterative function to implement `itoa()` function in C
//char* itoa(int value, char* buffer, int base)
//{
//    // invalid input
//    if (base < 2 || base > 32) {
//        return buffer;
//    }
// 
//    // consider the absolute value of the number
//    int n = abs(value);
// 
//    int i = 0;
//    while (n)
//    {
//        int r = n % base;
// 
//        if (r >= 10) {
//            buffer[i++] = 65 + (r - 10);
//        }
//        else {
//            buffer[i++] = 48 + r;
//        }
// 
//        n = n / base;
//    }
// 
//    // if the number is 0
//    if (i == 0) {
//        buffer[i++] = '0';
//    }
// 
//    // If the base is 10 and the value is negative, the resulting string
//    // is preceded with a minus sign (-)
//    // With any other base, value is always considered unsigned
//    if (value < 0 && base == 10) {
//        buffer[i++] = '-';
//    }
// 
//    buffer[i] = '\0'; // null terminate string
// 
//    // reverse the string and return it
//    return reverse(buffer, 0, i - 1);
//}




/* The Itoa code is in the puiblic domain */
char* itoa(int num, char* buffer, int base)   
{  
	int current = 0;  
	if (num == 0) {  
		buffer[current++] = '0';  
		buffer[current] = '\0';  
		return buffer;  
	}  
	int num_digits = 0;  
	if (num < 0) {  
		if (base == 10) {  
			num_digits ++;  
			buffer[current] = '-';  
			current ++;  
			num *= -1;  
		}  
		else  
		return NULL;  
	}  
	num_digits += (int)floor(log(num) / log(base)) + 1;  
	while (current < num_digits)   
	{  
		int base_val = (int) pow(base, num_digits-1-current);  
		int num_val = num / base_val;  
		 char value = num_val + '0';  
		buffer[current] = value;  
		current ++;  
		num -= base_val * num_val;  
	}  
	buffer[current] = '\0';  
	return buffer;  
}  