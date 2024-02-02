# CALOU_self_balancing_robot
CALOU is a two-wheels inverted pendulum controlled by Bluetooth through a smartphone.

It is a personal project entirely manufactured in 3D printing and powered by two RC Brushless BR2804-100T Gimbal Motor and a 2-axis Brushless Gimbal Controller BGC3.1 MOSFET card in a close loop with angle magnetic encoders. Those parts need some modification/adaptation to be assemble. The Controller with the IMU and Bluetooth module HC-05 are located into the head while the Body holds in the center a 500mA – 11.1V – 30C LiPo battery with a switch. The arms and legs of the robot serve as wire guide for motors and encoders.

The program was adapted from an application example developed by Antun Skuric, founder of the SimpleFOC project: an Arduino library implementing a Field Oriented Control (FOC) algorithm for BLDC and Stepper motors.  https://github.com/simplefoc/Arduino-FOC-balancer
