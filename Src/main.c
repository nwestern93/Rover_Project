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
#include "distance_front.h"
#include "BT_usart.h"
#include "color_sensor_set.h"
#include "motor.h"

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
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
void LED_Init(void){
	RCC -> AHBENR |= RCC_AHBENR_GPIOCEN;
	//Configuring red, blue, orange, and green LEDs to Output mode
	GPIOC -> MODER &= ~((1<<19) | (1<<18) | (1<<17) | (1<<16) | (1<<15) | (1<<14) | (1<<13) | (1<<12));
	GPIOC -> MODER |= (1<<18) | (1<<16) | (1<<14) | (1<<12);

	//Setting red, blue, orange, and green LEDs to no pull up or pull down mode
	GPIOC -> PUPDR &= ~((1<<19) | (1<<18) | (1<<17) | (1<<16) | (1<<15) | (1<<14) | (1<<13) | (1<<12));
	
	//Setting red, blue, orange, and green LEDs to low speed
	GPIOC -> OSPEEDR &= ~((1<<19) | (1<<18) | (1<<17) |(1<<16) | (1<<15) | (1<<14) | (1<<13) | (1<<12));


}
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	USART3_Init();
	BT_usart_init();
	LED_Init();
  color_sensor_init();
	right_motor_init();
	left_motor_init();
  /* USER CODE END SysInit */
char* message_start = "start";
	char* message_errror = "Error";
	char* message_newCMD = "New Command:";
	char* newline = "\n";
	BT_usart_transmit_message(message_start);
	BT_usart_transmit_message(newline);
	int dist = 0;
	uint8_t bytes[2];
	char buffer[20];
	char* Str = "";
	
	int a = 1253; 
	int opCode = 0;
	int color = 0; // this variable only applies to constant polling of color (Drag race)
	
	int stop_threshold = 710;
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	BT_usart_transmit_message(message_newCMD);
  BT_usart_transmit_message(newline);
	straight();
	GPIOC->BRR = GPIO_PIN_9;
	GPIOC->BSRR = GPIO_PIN_8; // Light orange
  while (1)
  {
		color = getColor();
		Str = itoa(color, buffer, 10);
		BT_usart_transmit_message(Str);
		BT_usart_transmit_message(newline);
    if (color > stop_threshold)
    {
			BT_usart_transmit_message("reached threshold");
			BT_usart_transmit_message(newline);
			GPIOC->BRR = GPIO_PIN_8;
			GPIOC->BSRR = GPIO_PIN_9; // Light green
			stop();
   }

//	HAL_Delay(10); // read registers every 100 ms (default)
//    /* USER CODE END WHILE */
//		BT_usart_transmit_message(message_newCMD);
//		BT_usart_transmit_message(newline);
//		
//		
//		opCode = BT_usart_receive_char();
//		
//		if (opCode == '1'){ // Sonar Distance Sensor
//			int distraw=0;
//			distraw = Sonar_GetDistance();
//			Str = itoa(distraw, buffer, 10);
//			BT_usart_transmit_message(Str);
//			BT_usart_transmit_message(newline);
//			
//		} else if (opCode == '2'){  // Color sensor 
//			int colorRaw=0;
//			colorRaw = getColor();
//			Str = itoa(colorRaw, buffer, 10);
//			BT_usart_transmit_message(Str);
//			BT_usart_transmit_message(newline);
//		
//		} else if (opCode == '3'){	//Ir sensor 1
////			int distraw=0;
////			distraw = ir_1_getDist();
////			Str = itoa(distraw, buffer, 10);
////			BT_usart_transmit_message("Left: ");
////			BT_usart_transmit_message(Str);
////			BT_usart_transmit_message(newline);
////			
////			distraw = ir_2_getDist();
////			Str = itoa(distraw, buffer, 10);
////			BT_usart_transmit_message("Right: ");
////			BT_usart_transmit_message(Str);
////			BT_usart_transmit_message(newline);
//		} else if (opCode == 'a'){ // motor commands start?
//			turn_left();
//		} else if (opCode == 'd'){ // motor commands start?
//			turn_right();
//		} else if (opCode == 'w'){ // motor commands start?
//			straight();
//		} else if (opCode == 's'){ // motor commands start?
//			stop();
//		} else{
//			BT_usart_transmit_message(message_errror);
//			BT_usart_transmit_message(newline);
//		}
//		
//		HAL_Delay(1000);
//    /* USER CODE BEGIN 3 */
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
