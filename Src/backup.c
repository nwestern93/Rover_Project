#include "backup.h"

uint16_t backup_turn_delay = 5000; 
_Bool exit_flag = 0;

void mode_Operation_back_up_plan(void){
	
	interupt_init();
	
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


	BT_usart_transmit_message(message_mode_Auto_drive);
	BT_usart_transmit_message(newline);
	
	straight();
	
	while(1){
		
		if(exit_flag){
			return;
		}
		
		front_dist = Sonar_GetDistance();
		
		if (front_dist < front_dist_threshold){
			
			stop();
			
			dist_right = backup_get_1();
			dist_left = backup_get_2();
			
			if (dist_left < dist_right){
				turn_right();
				counter_right++;
				HAL_Delay(backup_turn_delay);
				straight(); 
			}else{
				turn_right();
				counter_right++;
				HAL_Delay(backup_turn_delay);
				straight(); 
			}
		}
		
		if((counter_right > 0) & toggle){
			dist_left = backup_get_2();
			if(dist_left> left_dist_threshold){
				turn_left();
				counter_left++;
				HAL_Delay(backup_turn_delay);
				straight(); 
			}
		}
		
		if((counter_left > 0) & !toggle){
			dist_right = backup_get_1();
			if(dist_right> right_dist_threshold){
				turn_right();
				counter_right++;
				HAL_Delay(backup_turn_delay);
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

void interupt_init(void){
	__HAL_RCC_SYSCFG_CLK_ENABLE();
	EXTI->IMR |= 0x0001;
	EXTI->RTSR |= 0x0001;
	SYSCFG->EXTICR[0] = 0x0; 
	NVIC_EnableIRQ(EXTI0_1_IRQn); 
	NVIC_SetPriority(EXTI0_1_IRQn,1);
	

	// Enable peripheral clock GPIO A (Button)
	RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;
	//sets the MODER for pin 0  to General purpose input mode value: 00
	GPIOA->MODER |= (0 << 1) | (0 << 0);
	//sets the OSPEEDER for pin 0 to low speed value: 00
	GPIOA->OSPEEDR |= (0 << 1) | (0 << 0);
	//sets the PUPDR for pin 0 to have  pulldown value: 01
	GPIOA->PUPDR |= (1 << 1) | (0 << 0);
}

void EXTI0_1_IRQHandler(void){
	exit_flag = 1;
};

uint16_t backup_get_1(void){
	uint16_t dist;
	turn_right(); 
	HAL_Delay(backup_turn_delay);
	dist = Sonar_GetDistance();
	turn_left(); 
	HAL_Delay(backup_turn_delay);
	
	return dist; 
}

uint16_t backup_get_2(void){
	uint16_t dist;
	turn_left(); 
	HAL_Delay(backup_turn_delay);
	dist = Sonar_GetDistance();
	turn_right(); 
	HAL_Delay(backup_turn_delay);
	
	return dist; 
}
