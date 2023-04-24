#include "BT_usart.h"


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

/* The Itoa code is in the puiblic domain */
char* Itoa(int value, char* str, int radix) {
    static char dig[] =
        "0123456789"
        "abcdefghijklmnopqrstuvwxyz";
    int n = 0, neg = 0;
    unsigned int v;
    char* p, *q;
    char c;

    if (radix == 10 && value < 0) {
        value = -value;
        neg = 1;
    }
    v = value;
    do {
        str[n++] = dig[v%radix];
        v /= radix;
    } while (v);
    if (neg)
        str[n++] = '-';
    str[n] = '\0';
    for (p = str, q = p + n/2; p != q; ++p, --q)
        c = *p, *p = *q, *q = c;
    return str;
}