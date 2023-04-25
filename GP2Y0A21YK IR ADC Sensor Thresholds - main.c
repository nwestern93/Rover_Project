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


