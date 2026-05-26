# Field Following Robot

## Overview
Embedded systems project implementing a mobile robot with autonomous path-following using magnetic field sensing, with manual override and obstacle detection. Built using an EFM8 microcontroller in C as part of an electrical engineering design course.

## Features
- Autonomous path-following using magnetic field sensor input  
- Manual control via IR-based STM32 remote (mode switching)  
- Obstacle detection using time-of-flight sensor (I2C)  
- Servo-based obstacle avoidance  
- Debugging and signal validation using oscilloscope measurements  

## System Summary
The system uses a magnetic field sensor to guide autonomous movement, processed by an EFM8 microcontroller in C.  
An STM32 IR module enables manual override, while a time-of-flight sensor provides obstacle detection via I2C, triggering a servo-based avoidance response.


## Hardware
- EFM8 microcontroller  
- STM32 IR module  
- Magnetic field sensor  
- Time-of-flight sensor (I2C)  
- Servo motor system  


## Software
- Embedded C firmware  
- I2C communication  
- State-based control logic  


## Debugging
- Diagnosed noisy sensor signals using oscilloscope measurements  
- Improved stability through iterative testing and tuning  


## Media
(Add demo video / images here)
