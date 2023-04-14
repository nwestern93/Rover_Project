/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
//#include <stdio.h>
//#include <stdlib.h>
//#include "stm32f0xx.h"
#include "main.h"
//#include "distance_front.h"

/*
	This function intializes the USART3 Peripheral which includes setting the 
	GPIOC pins 11 and 10 to alternate funciton mode and selecting AF1 for both
	pins to put them in USART3 Tx and USART3 Rx modes.
	
	INPUTS: None
	OUTPUTS: None
*/

//DMA Buffer
uint8_t bytes[2];

void USART3_Init(void) {

	
	//Configuring PB10 and PB11 to be in alternate mode
	GPIOB -> MODER &= ~((1<<23) | (1<<22) | (1<<21) | (1<<20));
	GPIOB -> MODER |= (1<<23) | (1<<21);
	
	//Configuring GPIOB AFR register
	GPIOB -> AFR[1] |=   (0x04 << GPIO_AFRH_AFSEL11_Pos) | (0x04 <<GPIO_AFRH_AFSEL10_Pos);

	//Setting USART3 Baud rate to 9600
	USART3 -> BRR = HAL_RCC_GetHCLKFreq()/9600;
	
	//setting the DMAR bit in USART_CR3 register
	USART3-> CR3 |= 1<<6;
	
	//Enabling Tx, Rx and the peripheral
	USART3 -> CR1 |= (1<<0) | (1<<2) | (1<<3) | (1<<5) | (1<<7);
	
	
	
}
	
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
	while(!(USART3 -> ISR & (1<<7))) {
		i = i;
	}
	
	//Setting character value to transmit data register
	USART3 -> TDR = letter ;
		
}

void transmit_frontsensor(char letter){
	
	int i = 0;
	//Busy loop while TXE bit is still 0
	while(!(USART3 -> ISR & (1<<7))) {
		i = i;
	}
	
	//Setting character value to transmit data register
	USART3 -> TDR = letter ;
		
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

uint8_t receive_char(void) {
	uint8_t character = 'a';
	while(!(USART3 -> ISR & USART_ISR_RXNE));
	character = USART3 -> RDR;

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
	else if(dist > 0 && dist < 50) {
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

	
	transmit(dist);

}	


void SystemClock_Config(void);


void RCC_Init(void) {
	//Enabling GPIOA-C on RCC
	RCC -> AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC -> AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC -> AHBENR |= RCC_AHBENR_GPIOCEN;
	
	//Enabling USART3 on RCC
	RCC -> APB1ENR |= RCC_APB1ENR_USART3EN;
}

void LED_Init(void){
	//Configuring red, blue, orange, and green LEDs to Output mode
	GPIOC -> MODER &= ~((1<<19) | (1<<18) | (1<<17) | (1<<16) | (1<<15) | (1<<14) | (1<<13) | (1<<12));
	GPIOC -> MODER |= (1<<18) | (1<<16) | (1<<14) | (1<<12);

	//Setting red, blue, orange, and green LEDs to no pull up or pull down mode
	GPIOC -> PUPDR &= ~((1<<19) | (1<<18) | (1<<17) | (1<<16) | (1<<15) | (1<<14) | (1<<13) | (1<<12));
	
	//Setting red, blue, orange, and green LEDs to low speed
	GPIOC -> OSPEEDR &= ~((1<<19) | (1<<18) | (1<<17) |(1<<16) | (1<<15) | (1<<14) | (1<<13) | (1<<12));


}


int main(void)
{
	HAL_Init();
	
	SystemClock_Config();
	
	RCC_Init();

	LED_Init();

	USART3_Init();
	
	
  while (1)
  {
		GetFrontDistance();

		HAL_Delay(500);
		//GPIOC -> ODR ^= 1<<8;
	}
 
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
