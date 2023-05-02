# Rover_Project
ECE6780 Group Project - Olivedrab
Group Members:  Nick Western; Braden Brown; JT Herrmann; Dmitry Panin

Overview:
In this project built a rover capable of performing the following tasks:

1. Obstacle Course- Our rover was able to navigate a course with objects placed as
obstacles intended to be avoided. Our rover avoided these obstacles by having distances transmitted from
several on-board sensors to a bluetooth USART interface from which we fed directional instructions to orient the rover away.

2. Drag Race – Our rover accelerated as quickly as possible to the finish line and stopped 
autonomously just past the line of tape using a color sensor.

In addition to the two tasks mentioned above; the obstacle course and the drag race, we also
designed our rover with the following constraints in mind:

1. Our only connection to our rover is a wireless USART or RS232. This may allow
transmission of a low resolution visible spectrum camera (only 10x10 pixels) at a low
frame rate of approximately 2fps to act as a backup to the autonomous capabilities and
stop or back up the rover in case of emergency.

2. Our rover incorporated the following: GPIO, Interrupts, Timers, UART, I2C, Analog to Digital conversion (ADC), and PID control.

Instructions:

To implement the Obstacle Course and Drag race codes it is recommended to download a cross
compilation IDE like Keil u-vision and configure a project using a tool like STM32CubeMX.

This code was developed for the STM32F072 discovery board so similar devices may require you to 
update your code to match your device's datasheet. 

To run the code on the board, assuming you are using the same device as our group, configure a 
project using the STM32CubeMX tool and open the project in keil u-vision. Once the project is open
add our .c and .h files to your project in either the soure or include folders and compile the project. 
Then load the project on your board using the ST-Link. 
