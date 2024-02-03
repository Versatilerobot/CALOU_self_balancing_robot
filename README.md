# CALOU_self_balancing_robot
CALOU is an open source two-wheels inverted pendulum controlled by Bluetooth through a smartphone.

![3D-Calou.png](/img/3D-Calou.png)
It is a personal project entirely 3D print and powered by two RC Brushless BR2804-100T Gimbal Motor and a 2-axis Brushless Gimbal Controller BGC3.1 MOSFET card in a close loop with angle magnetic encoders. Those parts need some modification &/o adaptation to be assembled. The Controller with the IMU and Bluetooth module HC-05 are located into the head while the Body holds in the center a 500mA – 11.1V – 30C LiPo battery with a switch. The arms and legs of the robot serve as wire guide for motors and encoders.

The program was adapted from an application example of [SimpleFOCproject](https://github.com/simplefoc) : Arduino library implementing a Field Oriented Control (FOC) algorithm for BLDC and Stepper motors: [Arduino-FOC-balancer](https://github.com/simplefoc/Arduino-FOC-balancer)

CALOU is the mascot of the French Robotic Association [Caliban](https://www.facebook.com/AssoCaliban)

It can be controlled through a smartphone by using a remote Bluetooth app and simple commands.   
I hope you will enjoy building this robot and have fun seeing it moving and stabilize and blinking its antenna.

# Assembly Instruction Manual

Assembly instructions have been written in English and French with illustrations to help you in the process step-by-step, making it easier for users to understand and follow:  
[CALOU - self-balancing robot_EN](/CALOU%20-%20self-balancing%20robot_EN.pdf)  
[CALOU - robot en gyroskate_FR](/CALOU%20-%20robot%20en%20gyroskate_FR.pdf)  

The following topics are included:  
Mechanical parts manufacturing;  
Electronic adaptations and wiring;  
The assembly steps of the robot;  
The programming, setting and tests;   
The starting procedure.

A bill of material is also included: [BOM CALOU-GYROSKATE](/BOM%20CALOU-GYROSKATE.pdf)  

As a reminder, it is given as is. The author gives no warranty and accepts no responsibility or liability for the accuracy or completeness of the information and materials contained in these instruction manuals.  

# CALOU on YouTube
[CALOU demo](https://youtu.be/TESaMDyrZCY?si=MoILjAsdVkf6BUT1)

[<img src="/img/Photo%20Calou.png" width="500">](https://youtu.be/TESaMDyrZCY?si=MoILjAsdVkf6BUT1)

# Presentation - Apérobot 0x92 - 13th Dec 2023
[Apérobot 0x92 : Guy nous présente CALOU](https://youtu.be/fIwBQCcEI_Y?si=7qKZwHnpSAFOepEM)

[![Apérobot 0x92 : Guy nous présente CALOU](https://i.ytimg.com/vi/fIwBQCcEI_Y/maxresdefault.jpg)](https://youtu.be/fIwBQCcEI_Y?si=7qKZwHnpSAFOepEM)

# Author

Guy FEUILLOLEY
https://www.facebook.com/versat.ilerobot

# MIT License
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

# Copyright
Copyright (c) 2024 Versatilerobot
