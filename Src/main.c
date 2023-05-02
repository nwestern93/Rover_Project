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
//#include "GP2_IR.h"
#include "i3g_IR.h"
#include "motor.h"
#include "backup.h"

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
ADC_HandleTypeDef hadc;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
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



void init_all(void){
	HAL_Init();
	SystemClock_Config();
	USART3_Init();
	BT_usart_init();
	LED_Init();
	color_sensor_init();
	ir_1_init();
	right_motor_init();
	left_motor_init();
}
	
void mode_Sensor_test(void){
	char* message_mode_Sensor_test = "start mode Sensor Test";
	char* message_errror = "Error";
	char* message_newCMD = "Set opCode:";
	char* newline = "\n";
	int opCode = 0;
	
	char buffer[20];
	char* Str = "";
	
	while(1){
		BT_usart_transmit_message(message_mode_Sensor_test);
		BT_usart_transmit_message(newline);
		
		
		opCode = BT_usart_receive_char();
		
		if (opCode == '1'){ // Sonar Distance Sensor
			int distraw=0;
			distraw = Sonar_GetDistance();
			Str = itoa(distraw, buffer, 10);
			BT_usart_transmit_message(Str);
			BT_usart_transmit_message(newline);
			
		} else if (opCode == '2'){  // Color sensor 
			int colorRaw=0;
			colorRaw = getColor();
			Str = itoa(colorRaw, buffer, 10);
			BT_usart_transmit_message(Str);
			BT_usart_transmit_message(newline);
		
		} else if (opCode == '3'){	//Ir sensor 1
			int distraw=0;
			distraw = get_1();
			Str = itoa(distraw, buffer, 10);
			BT_usart_transmit_message("Left: ");
			BT_usart_transmit_message(Str);
			BT_usart_transmit_message(newline);
			
			distraw = get_2();
			Str = itoa(distraw, buffer, 10);
			BT_usart_transmit_message("Right: ");
			BT_usart_transmit_message(Str);
			BT_usart_transmit_message(newline);
		} else if (opCode == 'a'){ // motor commands start?
			turn_left();
		} else if (opCode == 'd'){ // motor commands start?
			turn_right();
		} else if (opCode == 'w'){ // motor commands start?
			straight();
		} else if (opCode == 's'){ // motor commands start?
			stop();
		} else{
			BT_usart_transmit_message(message_errror);
			BT_usart_transmit_message(newline);
		}
		
		HAL_Delay(1000);
	}
}

void mode_Drag_race(void){

	char* message_mode_Sensor_test = "start mode Drag Race";
	char* message_end = "end mode Drag Race";
	char* newline = "\n";
	
	int color = 0;
	int color_threshold = 6000;
	
	BT_usart_transmit_message(message_mode_Sensor_test);
	BT_usart_transmit_message(newline);
	
	straight();
	while (1) 
	{
		// Color sensor polling - enable for Drag race - disable for Obstacle Course
		color = getColor();
		if (color > color_threshold)
		{
			stop();
			BT_usart_transmit_message(message_end);
			BT_usart_transmit_message(newline);
		}
		//HAL_Delay(10); // read registers every 100 ms (default)
	}	

	
}


void mode_BT_Manual_Drive(void){
	char* message_mode_BT_Manual = "start mode manual driving";
	char* newline = "\n";

	BT_usart_transmit_message(message_mode_BT_Manual);
	BT_usart_transmit_message(newline);
	
	//add code here

}

void mode_Auto_Drive(void){
	char* message_mode_Auto_drive = "start mode Auto driving";
	char* newline = "\n";
	uint16_t front_dist;
	uint16_t dist_right;
	uint16_t dist_left;
	uint16_t color_val;
	
	uint16_t counter_right= 0;
	uint16_t counter_left = 0;
	_Bool toggle = 0;
	
	uint16_t front_dist_threshold = 5000;
	uint16_t right_dist_threshold = 400;
	uint16_t left_dist_threshold = 400;
	uint16_t color_threshold = 2000; 
	uint16_t turn_delay = 5000; 

	BT_usart_transmit_message(message_mode_Auto_drive);
	BT_usart_transmit_message(newline);
	
	straight();
	
	while(1){
		
		front_dist = Sonar_GetDistance();
		
		if (front_dist < front_dist_threshold){
			
			stop();
			
			dist_right = get_1();
			dist_left = get_2();
			
			if (dist_left < dist_right){
				turn_right();
				counter_right++;
				HAL_Delay(turn_delay);
				straight(); 
			}else{
				turn_right();
				counter_right++;
				HAL_Delay(turn_delay);
				straight(); 
			}
		}
		
		if((counter_right > 0) & toggle){
			dist_left = get_1();
			if(dist_left> left_dist_threshold){
				turn_left();
				counter_left++;
				HAL_Delay(turn_delay);
				straight(); 
			}
		}
		
		if((counter_left > 0) & !toggle){
			dist_right = get_1();
			if(dist_right> right_dist_threshold){
				turn_right();
				counter_right++;
				HAL_Delay(turn_delay);
				straight(); 
			}
		}
		
		toggle ^= toggle;
		
		color_val = getColor();
		if (color_val > color_threshold)
		{
			stop();
		}
		
	}	
}


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	init_all();

	char* message_start = "start";
	char* message_errror = "Error";
	char* message_mode = "Set Mode:";
	char* newline = "\n";
	int modeCode = 0;
	
	while (1)
  {
		BT_usart_transmit_message(message_start);
		BT_usart_transmit_message(newline);
		
		modeCode = BT_usart_receive_char();
		
		if (modeCode == 1){  //Sensor test
		
			mode_Sensor_test();
			
		}else if(modeCode == 2){ //Drag Race mode
			
			mode_Drag_race();
			
		}else if(modeCode == 3){ //Auto drive mode
			
			mode_Auto_Drive(); 
			
		}else if(modeCode == 4){ //operation back up plan
			
			mode_Operation_back_up_plan();
		
		}else{
		
			BT_usart_transmit_message(message_errror);
			BT_usart_transmit_message(newline);
		
		}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
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

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();

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

