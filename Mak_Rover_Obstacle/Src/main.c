
#include <stdio.h>
#include <stdlib.h>
#include "stm32f0xx.h"
#include "motor.h"

/* -------------------------------------------------------------------------------------------------------------
 *  Global Variable Declarations - MAIN FOR MOTOR CONTROL
 *  -------------------------------------------------------------------------------------------------------------
 */
volatile uint32_t debouncer;

/* -------------------------------------------------------------------------------------------------------------
 *  Miscellaneous Core Functions
 *  -------------------------------------------------------------------------------------------------------------
 */




void LED_init(void) {
    // Initialize PC8 and PC9 for LED's
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;                                          // Enable peripheral clock to GPIOC
    GPIOC->MODER |= GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0;                  // Set PC8 & PC9 to outputs
    GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9);                    // Set to push-pull output type
    GPIOC->OSPEEDR &= ~((GPIO_OSPEEDR_OSPEEDR8_0 | GPIO_OSPEEDR_OSPEEDR8_1) |
                        (GPIO_OSPEEDR_OSPEEDR9_0 | GPIO_OSPEEDR_OSPEEDR9_1));   // Set to low speed
    GPIOC->PUPDR &= ~((GPIO_PUPDR_PUPDR8_0 | GPIO_PUPDR_PUPDR8_1) |
                      (GPIO_PUPDR_PUPDR9_0 | GPIO_PUPDR_PUPDR9_1));             // Set to no pull-up/down
    GPIOC->ODR &= ~(GPIO_ODR_8 | GPIO_ODR_9);                                   // Shut off LED's
}

void  button_init(void) {
    // Initialize PA0 for button input
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;                                          // Enable peripheral clock to GPIOA
    GPIOA->MODER &= ~(GPIO_MODER_MODER0_0 | GPIO_MODER_MODER0_1);               // Set PA0 to input
    GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR0_0 | GPIO_OSPEEDR_OSPEEDR0_1);     // Set to low speed
    GPIOC->PUPDR |= GPIO_PUPDR_PUPDR0_1;                                        // Set to pull-down
}


//commented out 03/30/23
/* Called by exti Interrupt
 * Performs button debouncing, changes wave type on button rising edge
 * Updates frequency output from ADC value
 */

void EXTI0_1_IRQHandler(void) {
    // Remember that this function is called by the SysTick interrupt
    // You can't call any functions in here that use delay

    debouncer = (left_debouncer << 1);
		
    if(GPIOA->IDR & (1 << 0)) {
        debouncer |= 0x1;
    }
		
		if(debouncer == 0x7FFFFFFF) {
    switch(left_target_rpm) {
        case 75:
            left_target_rpm = 50;
            break;
        case 50:
            left_target_rpm = 75;
            break;
        case 0:
            left_target_rpm = 75;
            break;
        default:
            left_target_rpm = 0;
            break;
        }
    }
		if(debouncer == 0x7FFFFFFF) {
    switch(right_target_rpm) {
        case 75:
            right_target_rpm = 50;
            break;
        case 50:
            right_target_rpm = 75;
            break;
        case 0:
            right_target_rpm = 75;
            break;
        default:
            right_target_rpm = 0;
            break;
        }
    }		
}

/* -------------------------------------------------------------------------------------------------------------
 * Main Program Code
 *
 * Starts initialization of peripherals
 * Blinks green LED (PC9) in loop as heartbeat
 * -------------------------------------------------------------------------------------------------------------
 */
volatile uint32_t encoder_count = 0;
volatile uint32_t left_encoder_count = 0;
volatile uint32_t right_encoder_count = 0;
int main(int argc, char* argv[]) {
		//RCC->AHBENR |= RCC_AHBENR_GPIOBEN; 
	  RCC -> APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN; 
		
	  //Connecting PA0 to EXTI0
	  SYSCFG -> EXTICR[0] &= ~(SYSCFG_EXTICR1_EXTI0_PA);
	  SYSCFG -> EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA;
	
	  //Enabling interrupt generation on EXTI0
 	  EXTI -> IMR &= ~(1<<0);
 	  EXTI -> IMR |= 1<<0;
	
	  //Configuring EXTI0 to have a rising edge trigger
	  EXTI -> RTSR &= ~(1<<0);
	  EXTI -> FTSR &= ~(1<<0);
	  EXTI -> RTSR |= 1<<0;
	
	
		//EXTI interrupt
	  NVIC_EnableIRQ(EXTI0_1_IRQn);
	  NVIC_SetPriority(EXTI0_1_IRQn, 2);
	
	  ////SysTick priority
	  //NVIC_SetPriority(SysTick_IRQn, 2);
	
	  debouncer = 0;                          // Initialize global variables
		HAL_Init();															// Initialize HAL
    LED_init();                             // Initialize LED's
    button_init();                          // Initialize button
		
		
		left_motor_init();
		right_motor_init();
                              // Initialize motor code

    while (1) {
        GPIOC->ODR ^= GPIO_ODR_9;           // Toggle green LED (heartbeat)
        left_encoder_count = TIM3->CNT;
			  right_encoder_count = TIM2->CNT;
        HAL_Delay(128);                      // Delay 1/8 second
    }
}

// ----------------------------------------------------------------------------





/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Bluetooth USART
  *
  * Front sensor 
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
	NVIC_EnableIRQ(USART3_4_IRQn); 
	NVIC_SetPriority(USART3_4_IRQn,1);
	
	
}



//if we need usart to start an interrupt
void USART3_4_IRQHandler(void){

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




#include "main.h"

ADC_HandleTypeDef hadc;

void SystemClock_Config(void);
static void MX_ADC_Init(void);


int main(void)
{
			 
		SystemClock_Config();
		RCC->AHBENR |= RCC_APB2ENR_ADC1EN;//enable ADC1 in RCC
		RCC->AHBENR |= RCC_AHBENR_GPIOCEN;//enable LED clock
		GPIOC->MODER |= (3 << 0);//PC0 = ADC input to analog, no pull up/down (?default)
	        MX_ADC_Init(); //Initialize ADC. PC0 = ADC input (sensor PIN 1). Connect sensor PIN 2 (middle) = GND, PIN 3 (VCC) to 3 or 5V.
	 
	        // ADC Calibration
		ADC1->CR |= ADC_CR_ADDIS; //Set ADEN = 0;
		//while ((ADC1->CR & ADC_CR_ADEN) != 0) {}
	        ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN; // Set DMAEN = 0
		ADC1->CR |= ADC_CR_ADCAL; //Set ACAL = 1
		while ((ADC1->CR & ADC_CR_ADCAL) != 0){} //Wait until ADCAL=0
		ADC1->CR |= ADC_CR_ADEN; //Enable ADC
		ADC1->CHSELR |= (1<<10); //set ADC1 to channel 10, redundant with MX_ADC_Init()
		ADC1->CFGR1 |= ADC_CFGR1_CONT; //set ADC1 to continuous sampling, redundant with MX_ADC_Init()
		HAL_ADC_Start(&hadc);  //START ADC CONVERSION

		
		while (1)
		{
				if (ADC1->DR > 130) {target_rpm = 100);}  //left_target_rpm or right_target_rpm depending on sensor side
				else if (ADC1->DR <= 130) {target_rpm = 50);} //left_target_rpm or right_target_rpm
				else if (ADC1->DR > 173) {target_rpm = 0;} //left_target_rpm or right_target_rpm				
												
		}
  
}










// IR SENSOR CODE



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


static void MX_ADC_Init(void)
{ 
  ADC_ChannelConfTypeDef sConfig = {0};
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1; //
  hadc.Init.Resolution = ADC_RESOLUTION_8B; //
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
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
}


void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

