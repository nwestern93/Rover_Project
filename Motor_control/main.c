
#include <stdio.h>
#include <stdlib.h>
#include "stm32f0xx.h"
#include "motor.h"

/* -------------------------------------------------------------------------------------------------------------
 *  Global Variable Declarations
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
			  if(USART3 -> ISR & USART_ISR_RXNE) {
					char letter = USART3 -> RDR;
					if(letter == 'w') {	
						straight();
					}
					else if(letter == 'a') {
						turn_left();
					}
					else if(letter == 's') {
						stop();
					}
					else if(letter == 'd') {
						turn_right();
					}			
					else {
						right_target_rpm = right_target_rpm;
						left_target_rpm = left_target_rpm;
					}
				}
		
		}
}

// ----------------------------------------------------------------------------

