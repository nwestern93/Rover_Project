
#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdio.h>
#include <stdlib.h>
#include "stm32f0xx.h"

/* -------------------------------------------------------------------------------------------------------------
 *  Global Variable and Type Declarations
 *  -------------------------------------------------------------------------------------------------------------
 */
extern volatile int16_t right_error_integral;    // Integrated error signal
extern volatile uint8_t right_duty_cycle;    // Output PWM duty cycle
extern volatile int16_t right_target_rpm;    // Desired speed target
extern volatile int16_t right_motor_speed;   // Measured motor speed
extern volatile int8_t right_adc_value;      // ADC measured motor current
extern volatile int16_t right_error;         // Speed error signal
extern volatile uint8_t right_Kp;            // Proportional gain
extern volatile uint8_t right_Ki;            // Integral gain

extern volatile int16_t left_error_integral;    // Integrated error signal
extern volatile uint8_t left_duty_cycle;    // Output PWM duty cycle
extern volatile int16_t left_target_rpm;    // Desired speed target
extern volatile int16_t left_motor_speed;   // Measured motor speed
extern volatile int8_t left_adc_value;      // ADC measured motor current
extern volatile int16_t left_error;         // Speed error signal
extern volatile uint8_t left_Kp;            // Proportional gain
extern volatile uint8_t left_Ki;            // Integral gain

/* -------------------------------------------------------------------------------------------------------------
 *  Motor Control and Initialization Functions
 * -------------------------------------------------------------------------------------------------------------
 */

// Sets up the entire motor drive system
void right_motor_init(void);
void left_motor_init(void);

// Set the duty cycle of the PWM, accepts (0-100)
void right_pwm_setDutyCycle(uint8_t duty);
void left_pwm_setDutyCycle(uint8_t duty);

// PI control code is called within a timer interrupt
void right_PI_update(void);
void left_PI_update(void);

/* -------------------------------------------------------------------------------------------------------------
 *  Internal-Use Initialization Functions
 * -------------------------------------------------------------------------------------------------------------
 */

// Sets up the PWM and direction signals to drive the H-Bridge
void right_pwm_init(void);
void left_pwm_init(void);

// Sets up encoder interface to read motor speed
void right_encoder_init(void);
void left_encoder_init(void);

// Sets up ADC to measure motor current
void right_ADC_init(void);
void left_ADC_init(void);

//tells each individual motor to spin 
//void left_forward(void);
//void right_forward(void);

//tells rover to go forward 
void straight(void);

//tells rover to turn left
void turn_left(void);

//tells rover to turn right
void turn_right(void);

//tells rover to stop
void stop(void);

#endif /* MOTOR_H_ */
