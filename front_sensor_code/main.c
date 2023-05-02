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
#include <stdio.h>
#include <stdlib.h>
#include "stm32f0xx.h"
#include "main.h"
#include "distance_front.h"


void SystemClock_Config(void);


void RCC_Init(void) {
	//Enabling GPIOA-C on RCC
	RCC -> AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC -> AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC -> AHBENR |= RCC_AHBENR_GPIOCEN;
	
	//Enabling USARTS on RCC
	//RCC -> APB2ENR |= RCC_APB2ENR_USART1EN;
	//RCC -> APB1ENR |= RCC_APB1ENR_USART2EN;
	RCC -> APB1ENR |= RCC_APB1ENR_USART3EN;
	RCC -> APB1ENR |= RCC_APB1ENR_USART4EN;

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
		//GPIOC -> ODR ^= 1<<6;
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