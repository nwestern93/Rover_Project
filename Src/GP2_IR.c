#include "GP2_IR.h"

int16_t  ir_1;
int16_t	 ir_2;

void ir_1_init(void){
	RCC->AHBENR |= RCC_APB2ENR_ADC1EN;//enable ADC1 in RCC
	GPIOC->MODER |= (3 << 0);//PC0 = ADC input to analog, no pull up/down (?default)
	//MX_ADC_Init(); //Initialize ADC. PC0 = ADC input (potentiometer middle pin, others to GND and 3V)
	 //Calibration
	ADC1->CR |= ADC_CR_ADDIS; //Set ADEN = 0;
	ADC1->CR |= (1<<15);
		//while ((ADC1->CR & ADC_CR_ADEN) != 0) {}
	ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN; // Set DMAEN = 0
	ADC1->CR |= ADC_CR_ADCAL; //Set ACAL = 1
	while ((ADC1->CR & ADC_CR_ADCAL) != 0){} //Wait until ADCAL=0
	
	ADC1->CR |= ADC_CR_ADEN; //Enable ADC
	ADC1->CHSELR |= (1<<10); //set ADC1 to channel 10, redundant with MX_ADC_Init()
	ADC1->CFGR1 |= ADC_CFGR1_CONT; //set ADC1 to continuous sampling, redundant with MX_ADC_Init()
	//HAL_ADC_Start(&hadc);  //START ADC CONVERSION
	
}

void ir_getDist(void){
	while((ADC1->ISR & ADC_ISR_EOC) != 1);
	ir_1 = ADC1->DR;
	while((ADC1->ISR & ADC_ISR_EOC) != 1);
	ir_2 = ADC1->DR;
}

int16_t ir_1_getDist(void){
	ir_getDist();
	return ir_1;
}

int16_t ir_2_getDist(void){
	ir_getDist();
	return ir_2;
}
