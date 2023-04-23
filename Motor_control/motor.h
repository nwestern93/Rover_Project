
#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdio.h>
#include <stdlib.h>
#include "stm32f0xx.h"

/* -------------------------------------------------------------------------------------------------------------
 *  Global Variable and Type Declarations
 *  -------------------------------------------------------------------------------------------------------------
 */
extern volatile int16_t left_error_integral;    // Integrated error signal
extern volatile uint8_t left_duty_cycle;    // Output PWM duty cycle
extern volatile int16_t left_target_rpm;    // Desired speed target
extern volatile int16_t left_motor_speed;   // Measured motor speed
extern volatile int8_t left_adc_value;      // ADC measured motor current
extern volatile int16_t left_error;         // Speed error signal
extern volatile uint8_t left_Kp;            // Proportional gain
extern volatile uint8_t left_Ki;            // Integral gain

extern volatile int16_t right_error_integral;    // Integrated error signal
extern volatile uint8_t right_duty_cycle;    // Output PWM duty cycle
extern volatile int16_t right_target_rpm;    // Desired speed target
extern volatile int16_t right_motor_speed;   // Measured motor speed
extern volatile int8_t right_adc_value;      // ADC measured motor current
extern volatile int16_t right_error;         // Speed error signal
extern volatile uint8_t right_Kp;            // Proportional gain
extern volatile uint8_t right_Ki;            // Integral gain
/* -------------------------------------------------------------------------------------------------------------
 *  Motor Control and Initialization Functions
 * -------------------------------------------------------------------------------------------------------------
 */

// Sets up the entire motor drive system
void left_motor_init(void);
void right_motor_init(void);

// Set the duty cycle of the PWM, accepts (0-100)
void left_setDutyCycle(uint8_t left_duty);
void right_setDutyCycle(uint8_t right_duty);

// PI control code is called within a timer interrupt
void left_PI_update(void);
void right_PI_update(void);

/* -------------------------------------------------------------------------------------------------------------
 *  Internal-Use Initialization Functions
 * -------------------------------------------------------------------------------------------------------------
 */

// Sets up the PWM and direction signals to drive the H-Bridge
void left_pwm_init(void);
void right_pwm_init(void);

// Sets up encoder interface to read motor speed
void left_encoder_init(void);
void right_encoder_init(void);

// Sets up ADC to measure motor current
void left_ADC_init(void);
void right_ADC_init(void);

//Briefly turns on right motor and turns off left motor to turn left then stops to reevaluate
void right_turn(void);

//Briefly turns on left motor and turns off right motor to turn right then stops to reevaluate
void left_turn(void);

//Continues straight at a constant speed until told to do otherwise
void straight(void);

//Brings both motors to a halt
void stop(void);


#endif /* MOTOR_H_ */
