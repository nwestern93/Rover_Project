/* -------------------------------------------------------------------------------------------------------------
 *  Motor Control and Initialization Functions
 * -------------------------------------------------------------------------------------------------------------
 */
#include "motor.h"

volatile int16_t left_error_integral = 0; // Integrated error signal
volatile uint8_t left_duty_cycle = 0;    	// Output PWM duty cycle
volatile int16_t left_target_rpm = 0;    	// Desired speed target
volatile int16_t left_motor_speed = 0;   	// Measured motor speed
volatile int8_t left_adc_value = 0;      	// ADC measured motor current
volatile int16_t left_error = 0;         	// Speed error signal
volatile uint8_t left_Kp = 40;            // Proportional gain
volatile uint8_t left_Ki = 15;            // Integral gain

volatile int16_t right_error_integral = 0;  // Integrated error signal
volatile uint8_t right_duty_cycle = 0;    	// Output PWM duty cycle
volatile int16_t right_target_rpm = 0;    	// Desired speed target
volatile int16_t right_motor_speed = 0;   	// Measured motor speed
volatile int8_t right_adc_value = 0;      	// ADC measured motor current
volatile int16_t right_error = 0;         	// Speed error signal
volatile uint8_t right_Kp = 40;            	// Proportional gain
volatile uint8_t right_Ki = 15;            	// Integral gain


// Sets up the entire motor drive system

void left_motor_init(void) {
    left_pwm_init();
    left_encoder_init();
    left_ADC_init();
}
void right_motor_init(void) {
    right_pwm_init();
    right_encoder_init();
    right_ADC_init();
}

void left_pwm_init(void) {
	  // Set up pin PA4 for H-bridge PWM output (TIMER 14 CH1)
    GPIOA->MODER |= (1 << 9);
    GPIOA->MODER &= ~(1 << 8);

    // Set PA4 to AF4,
    GPIOA->AFR[0] &= 0xFFF0FFFF; // clear PA4 bits,
    GPIOA->AFR[0] |= (1 << 18);

		// Set up a PA5, PA6 as GPIO output pins for motor direction control
    GPIOA->MODER &= 0xFFFFC3FF; // clear PA5, PA6 bits,
    GPIOA->MODER |= (1 << 10) | (1 << 12);
    
    //Initialize one direction pin to high, the other low
    GPIOA->ODR |= (1 << 5);
    GPIOA->ODR &= ~(1 << 6);

    // Set up PWM timer
    RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
    TIM14->CR1 = 0;                         // Clear control registers
    TIM14->CCMR1 = 0;                       // (prevents having to manually clear bits)
    TIM14->CCER = 0;

    // Set output-compare CH1 to PWM1 mode and enable CCR1 preload buffer
    TIM14->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE);
    TIM14->CCER |= TIM_CCER_CC1E;           // Enable capture-compare channel 1
    TIM14->PSC = 1;                         // Run timer on 24Mhz
    TIM14->ARR = 1200;                      // PWM at 20kHz
    TIM14->CCR1 = 0;                        // Start PWM at 0% duty cycle
    
    TIM14->CR1 |= TIM_CR1_CEN;              // Enable timer

}
void right_pwm_init(void) {
	  // Set up pin PA9 for H-bridge PWM output (TIMER1  CH2 )
		GPIOA -> MODER &= ~((1<<19) | (1<<18));
		GPIOA -> MODER |= (1<<19);
	
		GPIOA -> AFR[1] |= (1<<5);

    // Set up a PA10, PA11 as GPIO output pins for motor direction control
    GPIOA->MODER &= ~((1<<23) | (1<<22) | (1<<21) | (1<<20)); // clear PA10, PA11 bits,
    GPIOA->MODER |= (1 << 22) | (1 << 20);
    
    //Initialize one direction pin to high, the other low
    GPIOA->ODR |= (1 << 10);
    GPIOA->ODR &= ~(1 << 11);

    // Set up PWM timer
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    TIM1->CR1 = 0;                         // Clear control registers
    TIM1->CCMR1 = 0;                       // (prevents having to manually clear bits)
    TIM1->CCER = 0;

    // Set output-compare CH1 to PWM1 mode and enable CCR1 preload buffer
    TIM1->CCMR1 |= (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE);
    TIM1->CCER |= TIM_CCER_CC2E;           // Enable capture-compare channel 1
    TIM1->PSC = 1;                         // Run timer on 24Mhz
    TIM1->ARR = 1200;                      // PWM at 20kHz
    TIM1->CCR2 = 0;                        // Start PWM at 0% duty cycle
    TIM1->BDTR |= TIM_BDTR_MOE;
    TIM1->CR1 |= TIM_CR1_CEN;              // Enable timer

}


void left_pwm_setDutyCycle(uint8_t left_duty) {
    
		if(left_duty <= 100) {
        TIM14->CCR1 = ((uint32_t)left_duty*TIM14->ARR)/100;  // Use linear transform to produce CCR1 value
        // (CCR1 == "pulse" parameter in PWM struct used by peripheral library)
    }
}

void right_pwm_setDutyCycle(uint8_t right_duty) {
		if(right_duty <= 100) {
        TIM1->CCR2 = ((uint32_t)right_duty*TIM1->ARR)/100;  // Use linear transform to produce CCR2 value
        // (CCR2 == "pulse" parameter in PWM struct used by peripheral library)
    }
}


// Sets up encoder interface to read motor speed
void left_encoder_init(void) {
    
    // Set up encoder input pins (TIMER 3 CH1 and CH2)
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
		//PB4 and PB5
    GPIOB->MODER &= ~(GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0);
    GPIOB->MODER |= (GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1);
    GPIOB->AFR[0] |= ( (1 << 16) | (1 << 20) );

    // Set up encoder interface (TIM3 encoder input mode)
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->CCMR1 = 0;
    TIM3->CCER = 0;
    TIM3->SMCR = 0;
    TIM3->CR1 = 0;

    TIM3->CCMR1 |= (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0);   // TI1FP1 and TI2FP2 signals connected to CH1 and CH2
    TIM3->SMCR |= (TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0);        // Capture encoder on both rising and falling edges
    TIM3->ARR = 0xFFFF;                                     // Set ARR to top of timer (longest possible period)
    TIM3->CNT = 0x7FFF;                                     // Bias at midpoint to allow for negative rotation
    // (Could also cast unsigned register to signed number to get negative numbers if it rotates backwards past zero
    //  just another option, the mid-bias is a bit simpler to understand though.)
    TIM3->CR1 |= TIM_CR1_CEN;                               // Enable timer

    // Configure a second timer (TIM6) to fire an ISR on update event
    // Used to periodically check and update speed variable
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    
    // Select PSC and ARR values that give an appropriate interrupt rate
    //updated 03/29/23
		TIM6->PSC = 7;
    TIM6->ARR = 37500;
    
    TIM6->DIER |= TIM_DIER_UIE;             // Enable update event interrupt
    TIM6->CR1 |= TIM_CR1_CEN;               // Enable Timer

    NVIC_EnableIRQ(TIM6_DAC_IRQn);          // Enable interrupt in NVIC
    NVIC_SetPriority(TIM6_DAC_IRQn,2);
}

void right_encoder_init(void) {
    
    // Set up encoder input pins (TIMER 2 CH1 and CH2)
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
		
	  //PA5 and PB3
    GPIOA->MODER &= ~((1<<11)| (1<<10));
    GPIOA->MODER |= (1<<11) | (1<<1);
    GPIOB->MODER &= ~((1<<7) |(1<<6));
		GPIOB->MODER |= 1<<7;
		GPIOA->AFR[0] |= (1 << 21) | (1 << 1);
		GPIOB->AFR[0] |= (1<<13);
	
    // Set up encoder interface (TIM3 encoder input mode)
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->CCMR1 = 0;
    TIM2->CCER = 0;
    TIM2->SMCR = 0;
    TIM2->CR1 = 0;

    TIM2->CCMR1 |= (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0);   // TI1FP1 and TI2FP2 signals connected to CH1 and CH2
    TIM2->SMCR |= (TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0);        // Capture encoder on both rising and falling edges
    TIM2->ARR = 0xFFFF;                                     // Set ARR to top of timer (longest possible period)
    TIM2->CNT = 0x7FFF;                                     // Bias at midpoint to allow for negative rotation
    // (Could also cast unsigned register to signed number to get negative numbers if it rotates backwards past zero
    //  just another option, the mid-bias is a bit simpler to understand though.)
    TIM2->CR1 |= TIM_CR1_CEN;                               // Enable timer

    // Configure a second timer (TIM6) to fire an ISR on update event
    // Used to periodically check and update speed variable
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
    
    // Select PSC and ARR values that give an appropriate interrupt rate
    //updated 03/29/23
		TIM7->PSC = 7;
    TIM7->ARR = 37500;
    
    TIM7->DIER |= TIM_DIER_UIE;             // Enable update event interrupt
    TIM7->CR1 |= TIM_CR1_CEN;               // Enable Timer

    NVIC_EnableIRQ(TIM7_IRQn);          // Enable interrupt in NVIC
    NVIC_SetPriority(TIM7_IRQn,2);
}

// Encoder interrupt to calculate motor speed, also manages PI controller
// left Motor 
void TIM6_DAC_IRQHandler(void) {
    // Calculate the motor speed in raw encoder counts
     // Note the motor speed is signed! Motor can be run in reverse.
     // Speed is measured by how far the counter moved from center point
     //
    left_motor_speed = (TIM3->CNT - 0x7FFF);
    TIM3->CNT = 0x7FFF; // Reset back to center point
    
    // Call the PI update function
    left_PI_update();

    TIM6->SR &= ~TIM_SR_UIF;        // Acknowledge the interrupt
}

// Right Motor 

void TIM7_IRQHandler(void) {
    /* Calculate the motor speed in raw encoder counts
     * Note the motor speed is signed! Motor can be run in reverse.
     * Speed is measured by how far the counter moved from center point
     */
    right_motor_speed = (TIM2->CNT - 0x7FFF);
    TIM2->CNT = 0x7FFF; // Reset back to center point
    
    // Call the PI update function
    right_PI_update();

    TIM7->SR &= ~TIM_SR_UIF;        // Acknowledge the interrupt
}




void left_ADC_init(void) {

    // Configure PA1 for ADC input (used for current monitoring)
    GPIOA->MODER |= (GPIO_MODER_MODER1_0 | GPIO_MODER_MODER1_1);

    // Configure ADC to 8-bit continuous-run mode, (asynchronous clock mode)
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

    ADC1->CFGR1 = 0;                        // Default resolution is 12-bit (RES[1:0] = 00 --> 12-bit)
    ADC1->CFGR1 |= ADC_CFGR1_CONT;          // Set to continuous mode
    ADC1->CHSELR |= ADC_CHSELR_CHSEL1;      // Enable channel 1

    ADC1->CR = 0;
    ADC1->CR |= ADC_CR_ADCAL;               // Perform self calibration
    while(ADC1->CR & ADC_CR_ADCAL);         // Delay until calibration is complete

    ADC1->CR |= ADC_CR_ADEN;                // Enable ADC
    while(!(ADC1->ISR & ADC_ISR_ADRDY));    // Wait until ADC ready
    ADC1->CR |= ADC_CR_ADSTART;             // Signal conversion start

}
void right_ADC_init(void) {

    // Configure PA2 for ADC input (used for current monitoring)
    GPIOA->MODER |= (GPIO_MODER_MODER2_0 | GPIO_MODER_MODER2_1);

    // Configure ADC to 8-bit continuous-run mode, (asynchronous clock mode)
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

    ADC1->CFGR1 = 0;                        // Default resolution is 12-bit (RES[1:0] = 00 --> 12-bit)
    ADC1->CFGR1 |= ADC_CFGR1_CONT;          // Set to continuous mode
    ADC1->CHSELR |= ADC_CHSELR_CHSEL2;      // Enable channel 2

    ADC1->CR = 0;
    ADC1->CR |= ADC_CR_ADCAL;               // Perform self calibration
    while(ADC1->CR & ADC_CR_ADCAL);         // Delay until calibration is complete

    ADC1->CR |= ADC_CR_ADEN;                // Enable ADC
    while(!(ADC1->ISR & ADC_ISR_ADRDY));    // Wait until ADC ready
    ADC1->CR |= ADC_CR_ADSTART;             // Signal conversion start
}


void left_PI_update(void) {
       
    
    /// TODO: Calculate integral portion of PI controller, write to "error_integral" variable
		left_error_integral = left_error_integral + (left_Ki * left_error);
		
    /// TODO: Clamp the value of the integral to a limited positive range
    if(left_error_integral < 0) {
			left_error_integral = 0;
		}
		else if (left_error_integral > 3200){
			left_error_integral = 3200;
		}
		else {
			left_error_integral = left_error_integral;
		}
    
    
		volatile int16_t left_proportional_value = 0; // Proportional error
		
    int16_t left_output = (left_Kp*left_error) + left_error_integral; 

     /// TODO: Divide the output into the proper range for output adjustment
			left_output = (left_output>>5);
     
		 /// TODO: Clamp the output value between 0 and 100 
		 if(left_output < 0) {
			left_output = 0;
		 }
		 else if(left_output > 100) {
			left_output = 100;
		 }
		 else {
			left_output = left_output;
		 }
    
    left_pwm_setDutyCycle(left_output);
    left_duty_cycle = left_output;            // For debug viewing

    // Read the ADC value for current monitoring, actual conversion into meaningful units 
    // will be performed by STMStudio
    if(ADC1->ISR & ADC_ISR_EOC) {   // If the ADC has new data for us
        left_adc_value = ADC1->DR;       // Read the motor current for debug viewing
    }
	}
	
void right_PI_update(void) {
       
    
    /// TODO: Calculate integral portion of PI controller, write to "error_integral" variable
		right_error_integral = right_error_integral + (right_Ki * right_error);
		
    /// TODO: Clamp the value of the integral to a limited positive range
    if(right_error_integral < 0) {
			right_error_integral = 0;
		}
		else if (right_error_integral > 3200){
			right_error_integral = 3200;
		}
		else {
			right_error_integral = right_error_integral;
		}
    
    
		volatile int16_t right_proportional_value = 0; // Proportional error
		
    int16_t right_output = (right_Kp*right_error) + right_error_integral; 

     /// TODO: Divide the output into the proper range for output adjustment
			right_output = (right_output>>5);
     
		 /// TODO: Clamp the output value between 0 and 100 
		 if(right_output < 0) {
			right_output = 0;
		 }
		 else if(right_output > 100) {
			right_output = 100;
		 }
		 else {
			right_output = right_output;
		 }
    
    right_pwm_setDutyCycle(right_output);
    right_duty_cycle = right_output;            // For debug viewing

    // Read the ADC value for current monitoring, actual conversion into meaningful units 
    // will be performed by STMStudio
    if(ADC1->ISR & ADC_ISR_EOC) {   // If the ADC has new data for us
        right_adc_value = ADC1->DR;       // Read the motor current for debug viewing
    }
	}
