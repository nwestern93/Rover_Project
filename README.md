# Rover_Project
ECE6780 Group Project - Olivedrab
Group Members:  Nick Western; Braden Brown; JT Herrmann; Dmitry Panin

Overview:
In this project we are building a rover with two tasks to complete:

1. Obstacle Course- Our rover will need to navigate a course with objects placed as
obstacles throughout. We will need to steer the rover through the course without
striking the obstacles and without being able to see our rover.

2. Drag Race – Our rover will attempt to drive as fast as possible to a finish line and stop as
close to the finish line as possible, again, without us being able to watch the rover while
it drives.


In addition to the two tasks mentioned above; the obstacle course and the drag race, we also
have to design our rover with the following constraints in mind:

1. Our only connection to our rover is a wireless USART or RS232. This may allow
transmission of a low resolution visible spectrum camera (only 10x10 pixels) at a low
frame rate of approximately 2fps to act as a backup to the autonomous capabilities and
stop or back up the rover in case of emergency.

2. Our rover must incorporate at least three topics we have covered in labs out of the
following list: GPIO, Interrupts, Timers, UART, I2C, Analog, and PID.

3. Because we are a graduate group we are required to use PID along with three other
options.


Instructions:

To implement the Obstacle Course and Drag race codes it is recommended to download a cross
compilation IDE like Keil u-vision and configure a project using a tool like STM32CubeMX.

This code was developed for the STM32F072 discovery board so similar devices may require you to 
update your code to match your device's datasheet. 

To run the code on the board, assuming you are using the same device as our group, configure a 
project using the STM32CubeMX tool and open the project in keil u-vision. Once the project is open
add our .c and .h files to your project in either the soure or include folders and compile the project. 
Then load the project on your board using the ST-Link. 
