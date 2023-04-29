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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

void ADC1_ConfigEnableStart(void){
	//Configuring ADC peripheral (disabling hardware triggers)//
	ADC1 -> CFGR1 &= ~((1<<11) | (1<<10) | (1<<13)| (1<<4) | (1<<3));
	//8bit resolution
	ADC1 -> CFGR1 |= (1<<4);
	//continuous conversion mode
	ADC1 -> CFGR1 |= (1<<13);
	
	//selecting/enabling channel 12 for PC2
	ADC1 -> CHSELR |= (1<<12);
	
	if ((ADC1->CR & ADC_CR_ADEN) != 0) /* (1) */
	{
		ADC1->CR |= ADC_CR_ADDIS; /* (2) */
	}
	while ((ADC1->CR & ADC_CR_ADEN) != 0)
	{
		/* For robust implementation, add here time-out management */
	}
	ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN; /* (3) */
	ADC1->CR |= ADC_CR_ADCAL; /* (4) */
	while ((ADC1->CR & ADC_CR_ADCAL) != 0) /* (5) */
	{
		/* For robust implementation, add here time-out management */
	}
	
	//clearing ADRDY bit 
	ADC1 -> ISR &= ~(1<<0);
	
	//setting peripheral enable
	ADC1 -> CR |= (1<<0);
	
	//waiting for ARDY flag
	while(!(ADC1 -> ISR & (1<<0))) {
		ADC1 -> CR |= 1<<0;
	}
	
	//Starting ADC 
	ADC1 -> CR |= (1<<2);

}

void potentiometer(void) {
		int value = 0;
		value = ADC1 -> DR;
		if(value >= 192) {
			GPIOC -> ODR |= (1<<6) |(1<<7) | (1<<8)| (1<<9);
		}
		else if(value >= 128  && value < 192) {
			GPIOC -> ODR &= ~(1<<6);
			GPIOC -> ODR |= (1<<7) | (1<<8) | (1<<9);
		}
		else if(value >= 64 && value < 128) {
			GPIOC -> ODR &= ~((1<<6) | (1<<9));
			GPIOC -> ODR |= (1<<7) | (1<<8);
		}
		else if(value > 5 && value < 64) {
			GPIOC -> ODR &= ~((1<<6)|(1<<7)|(1<<9));
			GPIOC -> ODR |= (1<<8);
		}
		else {
			GPIOC -> ODR &= ~((1<<6)|(1<<7)|(1<<8)|(1<<9));
		}

}

const uint8_t triangle_table[32] = {0,15,31,47,63,79,95,111,127,142,158,174,
190,206,222,238,254,238,222,206,190,174,158,142,127,111,95,79,63,47,31,15};

const uint8_t sine_table[32] = {127,151,175,197,216,232,244,251,254,251,244,
232,216,197,175,151,127,102,78,56,37,21,9,2,0,2,9,21,37,56,78,102};


void DAC_Init(void) {
	
	//Setting DAC channel 1 to software trigger mode
	DAC -> CR |= DAC_CR_TSEL1_2 | DAC_CR_TSEL1_1 | DAC_CR_TSEL1_0;  
	
	//Enabling DAC channel
	DAC -> CR |= DAC_CR_EN1;

}
	
void DAC_write(uint8_t value){
	
	DAC -> DHR8R1 = value;

}

void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  SystemClock_Config();

	//RCC Enables
	RCC -> AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC -> AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC -> APB2ENR |= RCC_APB2ENR_ADC1EN;
	RCC -> APB1ENR |= RCC_APB1ENR_DACEN;
	
	//Configuring LEDs and PC2
	GPIOC -> MODER &= ~((1<<19) | (1<<18) | (1<<17) | (1<<16) | (1<<15) | (1<<14) | (1<<13) | (1<<12));
	GPIOC -> PUPDR &= ~((1<<19) | (1<<18) | (1<<17) | (1<<16) | (1<<15) | (1<<14) | (1<<13) | (1<<12));
	GPIOC -> OSPEEDR &= ~((1<<19) | (1<<18) | (1<<17) | (1<<16) |(1<<15) | (1<<14) | (1<<13) | (1<<12));
	GPIOC -> MODER |= (1<<18) | (1<<16) | (1<<14) | (1<<12) | (1<<5) | (1<<4);
	
	//Setting PA4 to Analog mode and no PUPDR
	GPIOA -> MODER &= ~((1<<9) | (1<<8));
	GPIOA -> MODER |= (1<<9) | (1<<8);
	GPIOA -> PUPDR &= ~((1<<9) | (1<<8));
	
	//Configuring and starting ADC
	//ADC1_ConfigEnableStart();
	
	//Initializing DAC
	DAC_Init();
	
	int i=0;
  while (1)
  {
		if(i < 32) {
			i = i;
		}
		else {
			i = 0;
		}
		//Enable to read values from the potentiometer
		//potentiometer();
		
		DAC_write(sine_table[i]);
		HAL_Delay(1);
		
		i++;
  }
  /* USER CODE END 3 */
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
