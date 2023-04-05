#include "main.h"

ADC_HandleTypeDef hadc;

void SystemClock_Config(void);
static void MX_ADC_Init(void);


int main(void)
{
			 
		SystemClock_Config();
		RCC->AHBENR |= RCC_APB2ENR_ADC1EN;//enable ADC1 in RCC
		RCC->AHBENR |= RCC_AHBENR_GPIOCEN;//enable LED clock
		//RCC->AHBENR |= RCC_AHBENR_GPIOAEN;//enable DAC clock?
  	GPIOC->MODER |= (3 << 0);//PC0 = ADC input to analog, no pull up/down (?default)
		GPIOC->MODER |= (1 << 12) | (1 << 14) | (1 << 16) | (1 << 18);	//sets LED pins PC6-9  to General Purpose Output for ADC
	  MX_ADC_Init(); //Initialize ADC. PC0 = ADC input (potentiometer middle pin, others to GND and 3V)
	 
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

		// Sine Wave: 8-bit, 32 samples/cycle
		const uint8_t sine_table[32] = {127,151,175,197,216,232,244,251,254,251,244,232,216,197,175,151,127,102,78,56,37,21,9,2,0,2,9,21,37,56,78,102};	
			uint8_t i = 0;
		while (1)
		{
				if (ADC1->DR > 130) {GPIOC->ODR = (1 << 6);}//threshold of first LED 
				else if (ADC1->DR <= 130) {GPIOC->ODR = (0 << 6);}
				if (ADC1->DR > 173) {GPIOC->ODR = (1 << 8);}//threshold of second LED
				if (ADC1->DR > 177) {GPIOC->ODR = (1 << 7);}//threshold of third LED
				if (ADC1->DR > 177.5) {GPIOC->ODR = (1 << 9);}//threshold of fourth LED
					
												
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


