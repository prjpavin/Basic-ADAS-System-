/******************************************************************************

ADAS System - Advanced Driver Assistance System
Version: 1.0
License: MIT License
Author: Your Name
Description: This program implements an Advanced Driver Assistance System (ADAS)
         using an Arduino Mega board and various components. The ADAS system
         provides features such as forward collision avoidance, adaptive
         cruise control, lane departure warning, rear cross-traffic alert,
         cruise control, and headlights control.
How it works:
The ADAS system uses ultrasonic sensors to measure distances and detect objects.
The front ultrasonic sensors monitor the distance to objects in front of the vehicle.
If the distance falls below a safe threshold, the system triggers an emergency stop.
The front ultrasonic sensors also enable adaptive cruise control by monitoring the
distance to the vehicle ahead. If the distance becomes too close, a warning is issued
and the system adjusts the vehicle's speed to maintain a safe distance.
The left and right ultrasonic sensors detect the vehicle's position within the lane.
If the vehicle deviates from the lane, a warning is issued to prevent lane departure.
The rear ultrasonic sensor detects approaching vehicles while reversing. If a vehicle
is detected, a warning is issued to prevent collisions.
The system can activate or deactivate cruise control using a keypad, and the desired
cruise speed can be adjusted using the keypad as well.
The headlights control feature allows the system to turn on or off the headlights.
Components:
Arduino Mega board
Ultrasonic sensors: 5 (Front left, Front right, Rear cross, Left lane, Right lane)
Servo motor
LCD display
Keypad
Motor control
Headlights
Hazard lights
Bluetooth module
Wiring Instructions:
Arduino Mega board:
Connect VCC to 5V power supply
Connect GND to ground
Connect digital pins 2 to 7 to the ultrasonic sensors (front left, front right,
rear cross, left lane, right lane) as trigger pins
Connect digital pins 8 to 13 to the ultrasonic sensors as echo pins
Connect digital pin 22 to the servo motor for controlling the steering
Connect digital pins 23, 24, 25, and 26 to the LCD display for data transmission
Connect digital pins 27 to 33 to the keypad for user input
Connect digital pins 34 and 35 to the motor control for controlling vehicle speed
Connect digital pin 36 to the headlights for turning them on or off
Connect digital pin 37 to the hazard lights for turning them on or off
Connect digital pins 38 and 39 to the Bluetooth module for wireless communication
Debug Mode:
The ADAS system includes a debug mode that can be enabled for troubleshooting purposes.
When debug mode is activated, the system provides additional information and output on
the LCD display or through the serial monitor.
To enable debug mode, modify the code and set the appropriate debug flag to true.
Debug mode can help diagnose issues, monitor sensor values, and track system behavior
during testing and development stages.
Licensing:
This program is licensed under the MIT License, which allows you to modify, distribute,
and use the code for both personal and commercial purposes. See the LICENSE file for
more information.
******************************************************************************/

