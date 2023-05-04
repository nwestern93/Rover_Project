# Rover_Project
ECE6780 Group Project - Olivedrab
Group Members:  Nick Western; Braden Brown; JT Herrmann; Dmitry Panin
All code written in the C language (.c and .h designators)

Overview:
In this project built a rover capable of performing the following tasks:

1. Obstacle Course- Our rover was able to navigate a course with objects placed as
obstacles intended to be avoided. Our rover avoided these obstacles by having distances transmitted from
several on-board sensors to a bluetooth USART interface from which we fed directional instructions (WASD) 
to orient the rover towards an unobstructed path.

2. Drag Race – Our rover accelerated as quickly as possible to the finish line and stopped 
autonomously just past the line of tape using a color sensor.

In addition to the two tasks mentioned above; the obstacle course and the drag race, we also
designed our rover with the following constraints in mind:

1. Our only connection to our rover is a wireless USART or RS232. This may allow
transmission of a low resolution visible spectrum camera (only 10x10 pixels) at a low
frame rate of approximately 2fps to act as a backup to the autonomous capabilities and
stop or back up the rover in case of emergency.

2. Our rover incorporated the following: GPIO, Interrupts, Timers, UART, I2C, Analog to Digital conversion (ADC), and PID control.

Files have been provided within the SRC (source) folder for all of the on-board peripherals. We used two IR sensors, a color sensor, two 12 volt motors, a bluetooth transmitter, and an ultrasound sensor. A wiring diagram has been provided showing how the peripherals connect to the STM board and work with the provided code. 

## Implementation Instructions:

To implement the Obstacle Course and Drag race codes it is recommended to download a cross
compilation IDE like Keil u-vision and configure a project using a tool like STM32CubeMX.

This code was developed for the STM32F072 discovery board so similar devices may require you to 
update your code to match your device's datasheet. 

To run the code on the board, assuming you are using the same device as our group, configure a 
project using the STM32CubeMX tool and open the project in keil u-vision. Once the project is open
add our .c and .h files to your project in either the soure or include folders and compile the project. 
Then load the project on your board using the ST-Link. 

## Bluetooth configuration
Once the rover is powered on, a bluetooth connection will need to be done. It will require any BT Serial terminal. We used [Bluetooth Serial Terminal](https://apps.microsoft.com/store/detail/bluetooth-serial-terminal/9WZDNCRDFST8?hl=en-kn&gl=kn). 

1. In computer BT setting connect to the HC-05. Default password is 1234. 
2. Then open the Bluetooth Serial Terminal. Hit connect. 

## Overview of Rover Interface
On successful bootup in the BT Serial Terminal you should see the word "Start". The rover has with 4 modes. Select a mode by sending 1-4 in the terminal. It will hang until a user selects a mode. These modes outline diffrent desired behavior. 

1. Sensor Test Mode - This mode allow the user to query each of the sensors and exicute each motor function. sending single chars will select difrent sensors. '1' will return the front sonar sensor value in aproximently mm. '2' will return the value of the color sensor in the intesinty of blue. '3' will query both the left and right ir sensors. The Ir sensors give an anolog signial so the distance wasnt very percise. 255 value would be for objects very close ~10cm from the sensor. Values less than 80 would pretty much be at the max of the sensor 80cm. 'w' would turn on both motors. 'a' would turn on the right motor to turn left 45 degrees. 'd' would turn on the left motor to turn right 45 degrees. 's' will turn off both motors. 
2. Drag Race Mode - This mode turns on both motors, Then it will check the color sensor until it senses the line then it will stop. The color threshold to stop needs to be set at compilation. 
3. Manual Drive Mode- This mode is to drive for the obstical course. Every loop it will query each of the sensors and output them to the BT serial Terminal. Then it will wait for a 'w', 'a', 'd'. 'w' will go straight for 5000 miliseconds. 'a' and 'd' will turn left and right by 45 degrees. 
4. Automous Drive mode - untested and unused.  

## Additional Documents and links:

Pin Mapping: https://docs.google.com/document/d/18roIChLPl2zdtpu9mFGXQAJIOC7Tzi_BBI56ADnR7Jg/edit?usp=sharing

Online Electrical Wiring Diagram: https://virtual-graph-paper.com/index.html?edit=9a84729d4476

Demonstration Videos: https://drive.google.com/drive/folders/1nOP6aPW4H-RzOcj3__ERbeMvrUVF-qXq?usp=sharing

Final project Writeup: https://drive.google.com/file/d/165fWnYH-OEV6ZvXJfqCc-TxmHb8OZUnj/view?usp=sharing

