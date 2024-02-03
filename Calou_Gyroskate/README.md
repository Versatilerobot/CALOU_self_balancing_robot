# Arduino Simple Field Oriented Control (FOC) library 


![Library Compile](https://github.com/askuric/Arduino-FOC/workflows/Library%20Compile/badge.svg)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![arduino-library-badge](https://www.ardu-badge.com/badge/Simple%20FOC.svg?)](https://www.ardu-badge.com/badge/Simple%20FOC.svg)

Proper low-cost and low-power FOC supporting boards are very hard to find today and even may not exist. Even harder to find is a stable and simple FOC algorithm code for BLDC and Stepper motors capable of running on Arduino devices. 
Therefore this is an attempt to: 
- Demystify FOC algorithm and make a robust but simple Arduino library: [Arduino *SimpleFOClibrary*](https://docs.simplefoc.com/arduino_simplefoc_library_showcase)
- Develop a modular BLDC driver board: [Arduino *SimpleFOCShield*](https://docs.simplefoc.com/arduino_simplefoc_shield_showcase).
- ***New 📢:** Develop a modular Stepper motor board for FOC control:* <b>Arduino <span class="simple">Stepper<span class="foc">FOC</span>Shield</span></b>

<blockquote class="info"><p> <b>NEW RELEASE 📢:</b> <i>Simple<b>FOC</b>library v1.6.0</i><br></p><ul><li><strong>Stepper motor FOC support 🎨🎉 🎊 <a href="https://docs.simplefoc.com/motors">See in docs!</a></strong><ul><li>No losing steps</li><li>Backdrivable</li><li>Better dynamics than open-loop, Smoother than open-loop</li><li>short demo <a href="https://youtu.be/w_yIY0eXM5E">youtube video</a></li></ul></li><li>Teensy support by <em>Christopher Parrott</em> <br></li><li>Pull requests by <a href="https://github.com/cousinitt">@cousinitt</a><ul><li>refactoring and c++11 improvements</li><li>pid + low pass filter refactoring</li></ul></li><li>Extended configurability of the sensor classes by <a href="https://github.com/owennewo">@owennewo</a> <b><a href="https://docs.simplefoc.com/magnetic_sensor">See in docs!</a></b></li><li>configurable pwm frequency <b><a href="https://docs.simplefoc.com/motor_initialization#step-33-pwm-frequency-configuration-optional">See in docs!</a></b><ul><li>stm32,teensy,eps32 - not for Arduino</li><li>stm32 added 12bit pwm resolution by <em>Jürgen Frisch</em></li></ul></li><li>Huge refactoring done in the library 😄</li></ul></blockquote>

## Arduino *SimpleFOCShield*

<p align="">
<a href="https://youtu.be/G5pbo0C6ujE">
<img src="https://docs.simplefoc.com/extras/Images/foc_shield_video.jpg"  height="320px">
</a>
</p>

### Features
- **Plug & play**: In combination with Arduino <span class="simple">Simple<span class="foc">FOC</span>library</span> 
- **Low-cost**: Price of €15 - [Check the pricing](https://www.simplefoc.com/simplefoc_shield_product) 
- **Max power 100W** - max current 5A, power-supply 12-24V
   - Designed for Gimbal motors with the internal resistance >10 Ω. 
- **Stackable**: running 2 motors in the same time
- **Encoder/Hall sensor interface**: Integrated 3.3kΩ pullups (configurable)
- **I2C interface**: Integrated 4.7kΩ pullups (configurable)
- **Configurable pinout**: Hardware configuration - soldering connections
- **Arduino headers**: Arduino UNO, Arduino MEGA, STM32 Nucleo boards...
- **Open Source**: Fully available fabrication files - [how to make it yourself](https://www.simplefoc.com/arduino_simplefoc_shield_fabrication), 

##### If you are interested in this board, order your version on this link: [Simple FOC Shop](https://www.simplefoc.com/simplefoc_shield_product)

<p align=""><img src="https://docs.simplefoc.com/extras/Images/shield_to_v13.jpg" height="180px">   <img src="https://docs.simplefoc.com/extras/Images/shield_bo_v13.jpg"  height="180px"> <img src="https://docs.simplefoc.com/extras/Images/simple_foc_shield_v13_small.gif"  height="180x"></p>


## Arduino *SimpleFOClibrary*

<p align="">
<a href="https://youtu.be/Y5kLeqTc6Zk">
<img src="https://docs.simplefoc.com/extras/Images/youtube.png"  height="320px">
</a>
</p>

This video demonstrates the Simple FOC library basic usage, electronic connections and shows its capabilities.


### Features
- **Arduino compatible**: 
   - Arduino library code
  - Arduino Library Manager integration
- **Open-Source**: Full code and documentation available on github
- **Easy to setup and configure**: 
  - Easy hardware configuration
  - Easy [tuning the control loops](https://docs.simplefoc.com/motion_control)
- **Modular**:
  - Supports as many [sensors,  BLDC motors  and  driver boards](https://docs.simplefoc.com/supported_hardware) as possible
  - Supports multiple [MCU architectures](https://docs.simplefoc.com/microcontrollers):
     - Arduino: UNO, MEGA, any board with ATMega328 chips
     - STM32 boards: [Nucleo](https://www.st.com/en/evaluation-tools/stm32-nucleo-boards.html), [Bluepill](https://stm32-base.org/boards/STM32F103C8T6-Blue-Pill.html) ...
     - ESP32
     - Teensy boards
- **Plug & play**: Arduino <span class="simple">Simple<span class="foc">FOC</span>Shield</span>  

<p align=""> <img src="https://docs.simplefoc.com/extras/Images/uno_l6234.jpg"  height="170px">  <img src="https://docs.simplefoc.com/extras/Images/hmbgc_v22.jpg" height="170px">  <img src="https://docs.simplefoc.com/extras/Images/foc_shield_v13.jpg"  height="170px"></p>

## Getting Started
Depending on if you want to use this library as the plug and play Arduino library or you want to get insight in the algorithm and make changes there are two ways to install this code.

- Full library installation [Docs](https://docs.simplefoc.com/library_download)
- Minimal code installation [Docs](https://docs.simplefoc.com/minimal_download)

### Arduino SimpleFOC library installation to Arduino IDE
#### Arduino Library Manager 
The simplest way to get hold of the library is directly by using Arduino IDE and its integrated Library Manager. 
- Open Arduino IDE and start Arduino Library Manager by clicking: `Tools > Manage Libraries...`.
- Search for `Simple FOC` library and install the latest version.
- Reopen Arduino IDE and you should have the library examples in `File > Examples > Simple FOC`.

### Using Github website 
- Go to the [github repository](https://github.com/askuric/Arduino-FOC)
- Click first on `Clone or Download > Download ZIP`. 
- Unzip it and place it in `Arduino Libraries` folder. Windows: `Documents > Arduino > libraries`.  
- Reopen Arduino IDE and you should have the library examples in `File > Examples > Simple FOC`.

### Using terminal
- Open terminal and run
```sh  
cd *arduino libraries folder*
git clone https://github.com/askuric/Arduino-FOC.git
```
- Reopen Arduino IDE and you should have the library examples in `File > Examples > Simple FOC`.

###  SimpleFOC library minimal sketch example

For those willing to experiment and to modify the code I suggest using the [minimal version](https://github.com/askuric/Arduino-FOC/tree/minimal) of the code. 
 > This code is completely independent and you can run it as any other Arduino Sketch without the need for any libraries. 

#### Github website download
- Go to [minimal branch](https://github.com/askuric/Arduino-FOC/tree/minimal) 
- Download the code by clicking on the `Clone or Download > Download ZIP`.
- Unzip it and open the sketch in Arduino IDE. 

#### Using terminal
- Open the terminal:
  ```sh
  cd *to you desired directory*
  git clone -b minimal https://github.com/askuric/Arduino-FOC.git
  ```
- Then you just open it with the Arduino IDE and run it.

## Arduino code example
This is a simple Arduino code example implementing the velocity control program of a BLDC motor with encoder. 

NOTE: This program uses all the default control parameters.

```cpp
#include <SimpleFOC.h>

//  BLDCMotor( pin_pwmA, pin_pwmB, pin_pwmC, pole_pairs, enable (optional))
BLDCMotor motor = BLDCMotor(9, 10, 11, 11, 8);
//  Encoder(pin_A, pin_B, CPR)
Encoder encoder = Encoder(2, 3, 2048);
// channel A and B callbacks
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}


void setup() {  
  // initialize encoder hardware
  encoder.init();
  // hardware interrupt enable
  encoder.enableInterrupts(doA, doB);
  // link the motor to the sensor
  motor.linkSensor(&encoder);
  
  // use monitoring with the BLDCMotor
  Serial.begin(115200);
  // monitoring port
  motor.useMonitoring(Serial);

  // set control loop type to be used
  motor.controller = ControlType::velocity;
  // initialize motor
  motor.init();
  
  // align encoder and start FOC
  motor.initFOC();
}

void loop() {
  // FOC algorithm function
  motor.loopFOC();

  // velocity control loop function
  // setting the target velocity or 2rad/s
  motor.move(2);

  // monitoring function outputting motor variables to the serial terminal 
  motor.monitor();
}
```
You can find more details in the [SimpleFOC documentation](https://docs.simplefoc.com/).

## Example projects
Here are some of the SimpleFOC application examples. 
### Arduino Field Oriented Controlled Reaction Wheel Inverted Pendulum
This is a very cool open-source project of one of the simplest setups of the Reaction wheel inverted pendulum. Check out all the components and projects notes in the [github repository](https://github.com/askuric/Arduino-FOC-reaction-wheel-inverted-pendulum).  
<p align="">
<a href="https://youtu.be/Ih-izQyXJCI">
<img src="https://docs.simplefoc.com/extras/Images/youtube_pendulum.png"  height="320px">
</a>
</p>

**The main benefits of using the BLDC motor in this project are:**
- High torque to weight ratio
  - The lighter the better
- Lots of torque for low angular velocities
  - No need to spin the motor to very high PRM to achieve high torques
- No gearboxes and backlash
  - Very smooth operation = very stable pendulum


## Documentation
Find out more information about the Arduino SimpleFOC project in [docs website](https://docs.simplefoc.com/) 


## Arduino FOC repo structure
Branch  | Description | Status
------------ | ------------- | ------------ 
[master](https://github.com/askuric/Arduino-FOC) | Stable and tested library version | ![Library Compile](https://github.com/askuric/Arduino-FOC/workflows/Library%20Compile/badge.svg)
[dev](https://github.com/askuric/Arduino-FOC/tree/dev) | Development library version | ![Library Dev Compile](https://github.com/askuric/Arduino-FOC/workflows/Library%20Dev%20Compile/badge.svg?branch=dev)
[minimal](https://github.com/askuric/Arduino-FOC/tree/minimal) | Minimal Arduino example with integrated library | ![MinimalBuild](https://github.com/askuric/Arduino-FOC/workflows/MinimalBuild/badge.svg?branch=minimal)
