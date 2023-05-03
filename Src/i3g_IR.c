#include "i3g_IR.h"

int16_t  ir_1;
int16_t	 ir_2;

void calibration(void){
// from page 237 in perph manual
//1. Ensure that ADEN = 0 and DMAEN = 0.
//2. Set ADCAL = 1.
//3. Wait until ADCAL = 0.
//4. The calibration factor can be read from bits 6:0 of ADC_DR
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
}

void ADC_Enable(void){
		/* (1) Ensure that ADRDY = 0 */
	/* (2) Clear ADRDY */
	/* (3) Enable the ADC */
	/* (4) Wait until ADC ready */
	if ((ADC1->ISR & ADC_ISR_ADRDY) != 0) /* (1) */
	{
		ADC1->ISR |= ADC_ISR_ADRDY; /* (2) */
	}
	ADC1->CR |= ADC_CR_ADEN; /* (3) */
	while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) /* (4) */
	{
	/* For robust implementation, add here time-out management */
	}
}

void ADC_Disable(void){
	/* (1) Stop any ongoing conversion */
	/* (2) Wait until ADSTP is reset by hardware i.e. conversion is stopped */
	/* (3) Disable the ADC */
	/* (4) Wait until the ADC is fully disabled */
	ADC1->CR |= ADC_CR_ADSTP; /* (1) */
	while ((ADC1->CR & ADC_CR_ADSTP) != 0) /* (2) */
	{
	/* For robust implementation, add here time-out management */
	}
	ADC1->CR |= ADC_CR_ADDIS; /* (3) */
	while ((ADC1->CR & ADC_CR_ADEN) != 0) /* (4) */
	{
	/* For robust implementation, add here time-out management */
	}
}

void ir_1_init(void){
	ADC_Disable();
	//ADC_Enable();
	
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;//enable ADC1 in RCC
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
	GPIOA->MODER |= (0x3<<6); // PA3
	GPIOC->MODER |= (0xFF << 0);//PC0 = ADC input to analog, no pull up/down (?default)
	//GPIOC->MODER |= (3 << 2);//PC1
	//MX_ADC_Init(); //Initialize ADC. PC0 = ADC input (potentiometer middle pin, others to GND and 3V)
  // Set 8-bit resolution
  ADC1->CFGR1 |= (1 << 4);
	  // Turn off hardware triggers
  ADC1->CFGR1 &= ~((1 << 10) | (1 << 11));
//	ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN; // Set DMAEN = 0
//	ADC1->CHSELR |= (1<<10); //set ADC1 to channel 10, redundant with MX_ADC_Init()
//	ADC1->CHSELR |= (1<<11);
//	ADC1->CFGR1 &= ~(ADC_CFGR1_CONT); //set ADC1 to continuous sampling, redundant with MX_ADC_Init()
//	//HAL_ADC_Start(&hadc);  //START ADC CONVERSION
	calibration();

  ADC1->CHSELR |= (1<<10); //set ADC1 to channel 10, redundant with MX_ADC_Init()
	ADC1->CHSELR |= (1<<3);
    
  // Ensure ADSTP, ADSTART, ADDIS are = 0
  ADC1->CR &= ~(1 << 4); //STP
  ADC1->CR &= ~(1 << 2); //START
  ADC1->CR &= ~(1 << 1); //DIS

  ADC1->CR |= 0x1;

  ADC1->CR |= 1 << 2;
	ADC1->CR |= ADC_CR_ADEN; //Enable ADC
}

int16_t ir_getDist2(void){
	
	static int16_t val[2];
		/* (1) Select HSI14 by writing 00 in CKMODE (reset value) */
	/* (2) Select CHSEL0, CHSEL9, CHSEL10 andCHSEL17 for VRefInt */
	/* (3) Select a sampling mode of 111 i.e. 239.5 ADC clk to be greater
	than 17.1us */
	/* (4) Wake-up the VREFINT (only for VBAT, Temp sensor and VRefInt) */
	//ADC1->CFGR2 &= ~ADC_CFGR2_CKMODE; /* (1) */
	//ADC1->CFGR1 |= ADC_CFGR1_CONT | ADC_CFGR1_SCANDIR;
	ADC1->CHSELR = ADC_CHSELR_CHSEL10 |  ADC_CHSELR_CHSEL3; /* (2) */
	ADC1->SMPR = ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2; /* (3) */
	ADC->CCR |= ADC_CCR_VREFEN; /* (4) */
	
	//ADC1->CFGR1 |= ADC_CFGR1_SCANDIR;
	
		/* Performs the AD conversion */
	ADC1->CR |= ADC_CR_ADSTART; /* Start the ADC conversion  */
	for(int16_t i=0; i <2; i++){
		while ((ADC1->ISR & ADC_ISR_EOC) == 0) /* Wait end of conversion */
		{
		/* For robust implementation, add here time-out management */
		}
		val[i] = (int16_t)ADC1->DR; /* Store the ADC conversion result */
	}
	ADC1->CFGR1 ^= ADC_CFGR1_SCANDIR; /* Toggle the scan direction */
//	ir_1 = val[0];
//	ir_2 = val[1];
	return val[0];
}

int16_t get_1(void){
	//ADC1->SMPR = ADC_SMPR_SMP_2;
	return ir_getDist2();
	//return ir_1;
}
int16_t get_2(void){
	//ADC1->SMPR = ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2;
	return ir_getDist2();
	//return ir_2;
}